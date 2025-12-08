#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cosysairsim as airsim
import sys

def set_cone_segmentation_ids():
    """
    Sets the segmentation ID to 40 for BP_fountain_off_C_1 components.
    Also lists all instance segmentation objects and scene objects.
    """
    
    # Connect to the AirSim simulator
    client = airsim.VehicleClient()
    client.confirmConnection()
    
    print("Connected to AirSim simulator")
    print("=" * 80)
    
    try:
        # Get list of all instance segmentation objects
        print("\nListing all Instance Segmentation Objects:")
        print("-" * 80)
        seg_objects = client.simListInstanceSegmentationObjects()
        
        if seg_objects:
            for i, obj in enumerate(seg_objects, 1):
                print(f"{i:3d}. {obj}")
        else:
            print("No instance segmentation objects found")
        
        print(f"\nTotal instance segmentation objects: {len(seg_objects)}")
        
        # Get list of all scene objects
        print("\n" + "=" * 80)
        print("Listing all Scene Objects:")
        print("-" * 80)
        scene_objects = client.simListSceneObjects()
        
        if scene_objects:
            for i, obj in enumerate(scene_objects, 1):
                print(f"{i:2d}. {obj}")
        else:
            print("No scene objects found")
        
        print(f"\nTotal scene objects: {len(scene_objects)}")
        
        print("\n" + "=" * 80)
        print("Setting Segmentation IDs:")
        print("=" * 80)
        
        if not seg_objects:
            print("No instance segmentation objects found in the scene")
            return
        
        # Find all fountain components
        fountain_components = [obj for obj in seg_objects if 'BP_fountain_off_C_1' in obj]
        
        if not fountain_components:
            print("No fountain components found for BP_fountain_off_C_1")
            print("\nSearching for components containing 'fountain'...")
            fountain_components = [obj for obj in seg_objects if 'fountain' in obj.lower()]
            print(f"Found {len(fountain_components)} fountain-related components:")
            for comp in fountain_components:
                print(f"  - {comp}")
            return
        
        print(f"\nFound {len(fountain_components)} components for BP_fountain_off_C_1:")
        for comp in fountain_components:
            print(f"  - {comp}")
        
        print(f"\nSetting segmentation ID to 40 for all BP_fountain_off_C_1 components...")
        print("-" * 80)
        
        success_count = 0
        
        # Set segmentation ID for each component
        for comp_name in fountain_components:
            try:
                success = client.simSetSegmentationObjectID(comp_name, 40, is_name_regex=False)
                
                if success:
                    print(f"[OK] {comp_name}")
                    success_count += 1
                else:
                    print(f"[FAIL] {comp_name}")
            except Exception as e:
                print(f"[ERROR] {comp_name}: {e}")
        
        print("-" * 80)
        print(f"\nSuccessfully updated: {success_count}/{len(fountain_components)} components")
        
        if success_count == len(fountain_components):
            print("All fountain components have been set to segmentation ID 40!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    set_cone_segmentation_ids()
