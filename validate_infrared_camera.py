#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cosysairsim as airsim
import numpy as np
import sys
import time
import cv2

def validate_infrared_camera():
    """
    Comprehensive validation script to test infrared camera functionality.
    Tests image capture, pixel values, resolution, and data integrity.
    """
    
    print("=" * 80)
    print("INFRARED CAMERA VALIDATION SCRIPT")
    print("=" * 80)
    
    try:
        # Connect to simulator
        print("\n[1] Connecting to AirSim simulator...")
        client = airsim.VehicleClient()
        client.confirmConnection()
        print(f"[OK] Connected to AirSim simulator")
        
        # Wait for simulator to stabilize
        print("\n[2] Waiting for simulator to stabilize...")
        time.sleep(1)
        
        # Get camera info
        print("\n[3] Retrieving camera information...")
        camera_info = client.simGetCameraInfo("0")
        print(f"[OK] Camera info retrieved")
        print(f"    FOV: {camera_info.fov}")
        print(f"    Position: {camera_info.pose.position}")
        
        # Get scene image types and their resolutions
        print("\n[3.5] Getting image type information...")
        try:
            # Try to get resolution info for different image types
            scene_camera_info = client.simGetCameraInfo("0")
            print(f"[OK] Scene camera info available")
        except:
            print("[WARNING] Could not retrieve detailed camera info")
            scene_camera_info = None
        
        # Get scene objects
        print("\n[4] Listing scene objects...")
        scene_objects = client.simListSceneObjects()
        print(f"[OK] Found {len(scene_objects)} scene objects")
        
        # Get instance segmentation objects
        print("\n[5] Listing instance segmentation objects...")
        seg_objects = client.simListInstanceSegmentationObjects()
        print(f"[OK] Found {len(seg_objects)} segmentation objects")
        
        # Capture Scene image for reference
        print("\n[6] Capturing SCENE image...")
        scene_response = client.simGetImage("0", airsim.ImageType.Scene)
        if scene_response:
            scene_array = np.frombuffer(scene_response, np.uint8)
            print(f"[OK] Scene image captured: {len(scene_response)} bytes")
            print(f"    Array shape: {scene_array.shape}")
            # Use reshape to get actual dimensions if available in array shape
            if len(scene_array.shape) == 1:
                # If 1D array, need to calculate dimensions
                # For RGB: bytes / 3 channels = pixels, then determine W x H
                total_bytes = len(scene_response)
                bytes_per_pixel = 3  # RGB
                total_pixels = total_bytes // bytes_per_pixel
                # Try common resolutions based on aspect ratio
                scene_width = 1280  # Default from settings
                scene_height = 720
                print(f"    Resolution (from settings): {scene_width} x {scene_height}")
            else:
                scene_height, scene_width = scene_array.shape[0], scene_array.shape[1]
                print(f"    Resolution (from array shape): {scene_width} x {scene_height}")
            print(f"    Min: {np.min(scene_array)}, Max: {np.max(scene_array)}, Mean: {np.mean(scene_array):.2f}")
            print(f"    Non-zero pixels: {np.count_nonzero(scene_array)}")
        else:
            print("[FAIL] No scene response received")
            scene_response = None
        
        # Capture Infrared image
        print("\n[7] Capturing INFRARED image...")
        ir_response = client.simGetImage("0", airsim.ImageType.Infrared)
        if ir_response:
            ir_array = np.frombuffer(ir_response, np.uint8)
            print(f"[OK] Infrared image captured: {len(ir_response)} bytes")
            print(f"    Array shape: {ir_array.shape}")
            # Use reshape to get actual dimensions if available in array shape
            if len(ir_array.shape) == 1:
                # If 1D array, need to calculate dimensions
                # Expected: 256x144 RGB = 256 * 144 * 3 = 110,592 bytes
                total_bytes = len(ir_response)
                bytes_per_pixel = 3  # RGB
                total_pixels = total_bytes // bytes_per_pixel
                # Standard infrared resolution: 256x144
                ir_width = 256
                ir_height = 144
                expected_bytes = ir_width * ir_height * bytes_per_pixel
                print(f"    Resolution (expected): {ir_width} x {ir_height}")
                if total_bytes != expected_bytes:
                    print(f"    [WARNING] Size mismatch: {total_bytes} bytes (got) vs {expected_bytes} bytes (expected)")
            else:
                ir_height, ir_width = ir_array.shape[0], ir_array.shape[1]
                print(f"    Resolution (from array shape): {ir_width} x {ir_height}")
            print(f"    Min: {np.min(ir_array)}, Max: {np.max(ir_array)}, Mean: {np.mean(ir_array):.2f}")
            print(f"    Non-zero pixels: {np.count_nonzero(ir_array)}")
            print(f"    Unique values: {len(np.unique(ir_array))}")
            
            # Check if all black
            if np.max(ir_array) == 0:
                print("[WARNING] Infrared image is completely BLACK (all zeros)")
                print("          This indicates the capture component may not be rendering properly")
            elif np.max(ir_array) < 10:
                print("[WARNING] Infrared image is very dark (max < 10)")
            else:
                print("[OK] Infrared image has valid pixel values")
        else:
            print("[FAIL] No infrared response received")
            ir_response = None
        
        # Capture Segmentation image for comparison
        print("\n[8] Capturing SEGMENTATION image...")
        seg_response = client.simGetImage("0", airsim.ImageType.Segmentation)
        if seg_response:
            seg_array = np.frombuffer(seg_response, np.uint8)
            print(f"[OK] Segmentation image captured: {len(seg_response)} bytes")
            print(f"    Array shape: {seg_array.shape}")
            # Use reshape to get actual dimensions if available in array shape
            if len(seg_array.shape) == 1:
                # If 1D array, use same resolution as Scene (should match)
                seg_width = 1280  # Default from settings
                seg_height = 720
                print(f"    Resolution (from settings): {seg_width} x {seg_height}")
            else:
                seg_height, seg_width = seg_array.shape[0], seg_array.shape[1]
                print(f"    Resolution (from array shape): {seg_width} x {seg_height}")
            print(f"    Min: {np.min(seg_array)}, Max: {np.max(seg_array)}, Mean: {np.mean(seg_array):.2f}")
            print(f"    Non-zero pixels: {np.count_nonzero(seg_array)}")
        else:
            print("[FAIL] No segmentation response received")
            seg_response = None
        
        # Compare image sizes
        print("\n[9] Comparing image sizes...")
        if scene_response and ir_response and seg_response:
            scene_size = len(scene_response)
            ir_size = len(ir_response)
            seg_size = len(seg_response)
            
            print(f"    Scene size: {scene_size} bytes ({scene_width} x {scene_height})")
            print(f"    Infrared size: {ir_size} bytes ({ir_width} x {ir_height})")
            print(f"    Segmentation size: {seg_size} bytes ({seg_width} x {seg_height})")
            
            scene_ir_ratio = scene_size / ir_size if ir_size > 0 else 0
            scene_seg_ratio = scene_size / seg_size if seg_size > 0 else 0
            ir_seg_ratio = ir_size / seg_size if seg_size > 0 else 0
            
            print(f"\n    Ratios:")
            print(f"      Scene/IR: {scene_ir_ratio:.2f}x")
            print(f"      Scene/Segmentation: {scene_seg_ratio:.2f}x")
            print(f"      IR/Segmentation: {ir_seg_ratio:.2f}x")
            
            if scene_width == ir_width and scene_height == ir_height:
                print("[OK] Scene and Infrared have matching dimensions")
            else:
                print(f"[WARNING] Scene and Infrared dimensions differ: {scene_width}x{scene_height} vs {ir_width}x{ir_height}")
            
            if scene_width == seg_width and scene_height == seg_height:
                print("[OK] Scene and Segmentation have matching dimensions")
            else:
                print(f"[WARNING] Scene and Segmentation dimensions differ: {scene_width}x{scene_height} vs {seg_width}x{seg_height}")
        elif scene_response and ir_response:
            scene_size = len(scene_response)
            ir_size = len(ir_response)
            ratio = scene_size / ir_size if ir_size > 0 else 0
            
            print(f"    Scene size: {scene_size} bytes ({scene_width} x {scene_height})")
            print(f"    Infrared size: {ir_size} bytes ({ir_width} x {ir_height})")
            print(f"    Ratio (Scene/IR): {ratio:.1f}x")
            
            if ratio > 100:
                print("[ERROR] Infrared image is significantly smaller than Scene!")
                print("        This suggests the render target is not properly sized")
                print("        Expected: Similar sizes, Got: {:.0f}x difference".format(ratio))
            elif ratio > 10:
                print("[WARNING] Infrared image is notably smaller than Scene")
            else:
                print("[OK] Image sizes are similar")
        
        # Display images at native resolution
        print("\n[9.5] Displaying captured images...")
        try:
            # Display Scene image
            if scene_response:
                scene_array = np.frombuffer(scene_response, np.uint8)
                # Try to decode as image (handles various formats)
                scene_img = cv2.imdecode(scene_array, cv2.IMREAD_UNCHANGED)
                if scene_img is not None:
                    # AirSim images are RGB, convert to BGR for OpenCV display
                    if len(scene_img.shape) == 3 and scene_img.shape[2] == 3:
                        scene_bgr = cv2.cvtColor(scene_img, cv2.COLOR_RGB2BGR)
                    else:
                        scene_bgr = scene_img
                    cv2.imshow(f'Scene Camera - {scene_width} x {scene_height}', scene_bgr)
                    print("[OK] Scene image displayed")
                else:
                    print("[WARNING] Could not decode scene image")
            
            # Display Infrared image
            if ir_response:
                ir_array = np.frombuffer(ir_response, np.uint8)
                # Try to decode as image
                ir_img = cv2.imdecode(ir_array, cv2.IMREAD_UNCHANGED)
                if ir_img is not None:
                    # For grayscale infrared, use only first channel (RGB all same for grayscale)
                    if len(ir_img.shape) == 3 and ir_img.shape[2] == 3:
                        ir_gray = ir_img[:, :, 0]
                    else:
                        ir_gray = ir_img
                    cv2.imshow(f'Infrared Camera - {ir_width} x {ir_height}', ir_gray)
                    print("[OK] Infrared image displayed")
                else:
                    print("[WARNING] Could not decode infrared image")
            
            # Display Segmentation image
            if seg_response:
                seg_array = np.frombuffer(seg_response, np.uint8)
                # Try to decode as image
                seg_img = cv2.imdecode(seg_array, cv2.IMREAD_UNCHANGED)
                if seg_img is not None:
                    # AirSim images are RGB, convert to BGR for OpenCV display
                    if len(seg_img.shape) == 3 and seg_img.shape[2] == 3:
                        seg_bgr = cv2.cvtColor(seg_img, cv2.COLOR_RGB2BGR)
                    else:
                        seg_bgr = seg_img
                    cv2.imshow(f'Segmentation Camera - {seg_width} x {seg_height}', seg_bgr)
                    print("[OK] Segmentation image displayed")
                else:
                    print("[WARNING] Could not decode segmentation image")
            
            print("[OK] Images displayed successfully")
            print("    Press any key to close image windows...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        except Exception as e:
            print(f"[WARNING] Could not display images: {e}")
            print("          (opencv may not be installed)")
        
        # Test segmentation ID setting on infrared
        print("\n[10] Testing segmentation ID functionality...")
        fountain_components = [obj for obj in seg_objects if 'fountain' in obj.lower()]
        if fountain_components:
            print(f"    Found {len(fountain_components)} fountain components")
            test_comp = fountain_components[0]
            print(f"    Testing with: {test_comp}")
            
            success = client.simSetSegmentationObjectID(test_comp, 100, is_name_regex=False)
            if success:
                print(f"[OK] Successfully set segmentation ID on {test_comp}")
                
                # Capture infrared again after setting ID
                time.sleep(0.5)
                print("    Capturing infrared image after setting segmentation ID...")
                ir_response2 = client.simGetImage("0", airsim.ImageType.Infrared)
                if ir_response2:
                    ir_array2 = np.frombuffer(ir_response2, np.uint8)
                    print(f"    New IR image: {len(ir_response2)} bytes")
                    print(f"    Min: {np.min(ir_array2)}, Max: {np.max(ir_array2)}, Mean: {np.mean(ir_array2):.2f}")
                    
                    if np.max(ir_array2) > 0 and np.max(ir_array2) != np.max(ir_array):
                        print("[OK] Infrared values changed after segmentation ID update")
                    else:
                        print("[WARNING] Infrared values did not change after segmentation ID update")
            else:
                print(f"[FAIL] Could not set segmentation ID")
        
        # Diagnostic summary
        print("\n" + "=" * 80)
        print("DIAGNOSTIC SUMMARY")
        print("=" * 80)
        
        if ir_response:
            ir_array = np.frombuffer(ir_response, np.uint8)
            
            if np.max(ir_array) == 0:
                print("\n[CRITICAL] Infrared camera is completely black")
                print("Possible causes:")
                print("  1. InfraredCaptureComponent not properly initialized in blueprint")
                print("  2. Capture component is disabled or not activated")
                print("  3. Wrong CaptureSource is being used")
                print("  4. Scene is completely dark (unlikely given Scene image is fine)")
                print("\nRecommended fixes:")
                print("  1. Check BP_PIPCamera blueprint for InfraredCaptureComponent")
                print("  2. Verify InfraredCaptureComponent is set to correct resolution (256x144)")
                print("  3. Ensure CaptureSource is set correctly (should mirror Scene camera)")
                print("  4. Check if material is being applied that blocks rendering")
                
            elif len(ir_response) < 1000:
                print("\n[CRITICAL] Infrared image size is too small")
                print(f"Current: {len(ir_response)} bytes (expected ~110k+ for 256x144x3 RGB)")
                print("\nThis indicates:")
                print("  1. Render target is not properly initialized")
                print("  2. Capture component resolution is wrong in blueprint")
                print("  3. Pixel format is not RGB")
                print("\nFix: Set InfraredCaptureComponent resolution to match Scene camera")
                
            else:
                if np.max(ir_array) > 0:
                    print("\n[OK] Infrared camera appears to be working")
                    print(f"Image size: {len(ir_response)} bytes")
                    print(f"Pixel range: {np.min(ir_array)} - {np.max(ir_array)}")
                else:
                    print("\n[WARNING] Infrared image data exists but all pixels are 0")
        
        print("\n" + "=" * 80)
        
    except Exception as e:
        print(f"\n[EXCEPTION] Error during validation: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    success = validate_infrared_camera()
    sys.exit(0 if success else 1)
