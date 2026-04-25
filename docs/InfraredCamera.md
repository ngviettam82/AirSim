# Infrared Camera in Cosys-AirSim

This page describes the current `ImageType::Infrared` output in Cosys-AirSim.

## Current runtime behavior

In this branch, `ImageType::Infrared` is handled as an annotation/object-ID image, not as a physically rendered thermal scene. The infrared annotator follows the same object discovery and ID update lifecycle as instance segmentation, but writes a grayscale ID value: object ID `n` is rendered as `(n % 256, n % 256, n % 256)`.

Important implications:

* `simSetSegmentationObjectID()` updates both the segmentation color layer and the infrared grayscale-ID layer.
* `simSetSegmentationObjectIDs()` is available for bulk ID changes and updates both layers with one final segmentation/infrared refresh if anything changed.
* Infrared ID values above 255 alias because the output is one 8-bit grayscale value per channel.
* Static meshes, skeletal meshes, instanced static mesh components, and Unreal Landscape components are supported by the current built-in ID path.
* Landscape infrared uses the same lightweight landscape annotation proxy as instance segmentation. Each `ULandscapeComponent` is listed separately, while the owning `ALandscapeProxy` name is the shared label key so one ID update changes the whole landscape.

To generate your own infrared-ID data, assign object IDs with `simSetSegmentationObjectID()` or `simSetSegmentationObjectIDs()` before capture, then request `airsim.ImageType.Infrared` through the image API. The legacy Africa-environment helper scripts sometimes referenced for thermal digital-count remapping are not present in this repository checkout; use the segmentation APIs directly or add a project-specific remapping script.

For exact ground-truth labels, keep infrared noise and distortion disabled. Those post-process settings can still be configured like other image types, but they can alter the exact grayscale values.

Finally, the details about how temperatures were estimated for plants and animals in the Africa environment, etc. can be found in this paper:

    @inproceedings{bondi2018airsim,
      title={AirSim-W: A Simulation Environment for Wildlife Conservation with UAVs},
      author={Bondi, Elizabeth and Dey, Debadeepta and Kapoor, Ashish and Piavis, Jim and Shah, Shital and Fang, Fei and Dilkina, Bistra and Hannaford, Robert and Iyer, Arvind and Joppa, Lucas and others},
      booktitle={Proceedings of the 1st ACM SIGCAS Conference on Computing and Sustainable Societies},
      pages={40},
      year={2018},
      organization={ACM}
    }
