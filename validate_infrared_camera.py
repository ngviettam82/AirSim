#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cosysairsim as airsim
import numpy as np
import sys
import time

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
            print(f"    Min: {np.min(seg_array)}, Max: {np.max(seg_array)}, Mean: {np.mean(seg_array):.2f}")
            print(f"    Non-zero pixels: {np.count_nonzero(seg_array)}")
        else:
            print("[FAIL] No segmentation response received")
            seg_response = None
        
        # Compare image sizes
        print("\n[9] Comparing image sizes...")
        if scene_response and ir_response:
            scene_size = len(scene_response)
            ir_size = len(ir_response)
            ratio = scene_size / ir_size if ir_size > 0 else 0
            
            print(f"    Scene size: {scene_size} bytes")
            print(f"    Infrared size: {ir_size} bytes")
            print(f"    Ratio (Scene/IR): {ratio:.1f}x")
            
            if ratio > 100:
                print("[ERROR] Infrared image is significantly smaller than Scene!")
                print("        This suggests the render target is not properly sized")
                print("        Expected: Similar sizes, Got: {:.0f}x difference".format(ratio))
            elif ratio > 10:
                print("[WARNING] Infrared image is notably smaller than Scene")
            else:
                print("[OK] Image sizes are similar")
        
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
                print(f"Current: {len(ir_response)} bytes (expected ~150k+ for 256x144x4 RGBA)")
                print("\nThis indicates:")
                print("  1. Render target is not properly initialized")
                print("  2. Capture component resolution is wrong in blueprint")
                print("  3. Pixel format is not RGBA")
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
