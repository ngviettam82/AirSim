#!/usr/bin/env python

import setup_path
import cosysairsim as airsim
import csv
import random
import numpy as np
from PIL import Image
from datetime import datetime

# Script that generates a list of all objects in the Airsim environment and their associated instance segmentation color
if __name__ == '__main__':

    # Make connection to AirSim API
    client = airsim.CarClient()
    client.confirmConnection()

    # Generate list of all colors available for segmentation
    print("Loading segmentation colormap...")
    colorMap = np.asarray(client.simGetSegmentationColorMap())
    print("Loaded segmentation colormap.")

    # Get names of all objects in simulation world and store in list together with the associated RGB value
    # In a dynamic world, this is never the same!!
    currentObjectList = client.simListInstanceSegmentationObjects()
    print("Generating list of all current objects...")
    with open('airsim_segmentation_colormap_list_' +  datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.csv', 'w') as f:
        f.write("ObjectName,ObjectID,R,G,B\n")
        for item in currentObjectList:
            object_id = client.simGetSegmentationObjectID(item)
            if 0 <= object_id < len(colorMap):
                color = colorMap[object_id, :]
            else:
                color = [0, 0, 0]
            f.write("%s,%s,%s\n" % (item, object_id, ','.join([str(int(x)) for x in color])))
    print("Generated list of all current objects with a total of " + str(len(currentObjectList)) + ' objects.')

