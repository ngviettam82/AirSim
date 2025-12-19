# Segmentation Camera and Annotation System Analysis

## Scope
- Reviewed segmentation and annotation flow across these files: `Unreal/Plugins/AirSim/Source/Annotation/AnnotationComponent.h`, `Unreal/Plugins/AirSim/Source/Annotation/AnnotationComponent.cpp`, `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.h`, `Unreal/Plugins/AirSim/Source/Annotation/ObjectAnnotator.cpp`, `Unreal/Plugins/AirSim/Source/PIPCamera.h`, `Unreal/Plugins/AirSim/Source/PIPCamera.cpp`, `Unreal/Plugins/AirSim/Source/LidarCamera.h`, `Unreal/Plugins/AirSim/Source/LidarCamera.cpp`, `Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp`, `Unreal/Plugins/AirSim/Source/UnrealImageCapture.cpp`, `Unreal/Plugins/AirSim/Source/WorldSimApi.h`, `Unreal/Plugins/AirSim/Source/WorldSimApi.cpp`, `Unreal/Plugins/AirSim/Source/AirBlueprintLib.h`.

## System Overview
- Instance segmentation is implemented as a special annotation layer backed by mesh-attached proxy components that render flat colors in a dedicated scene capture.
- The segmentation camera is a scene capture component inside `APIPCamera` configured to render only annotation proxy components via a show-only list and annotation-specific show flags.
- `ASimModeBase` coordinates building and updating the list of annotation components and pushes that list to all PIP cameras and LiDAR cameras that need ground-truth segmentation.
- `UnrealImageCapture` reads the render targets and disables gamma correction for segmentation, annotation, and infrared outputs.

## Responsibilities by Component
- `FObjectAnnotator` (in `ObjectAnnotator.*`)
  - Discovers paintable mesh components per actor (static or skinned) and assigns them unique names.
  - Creates and registers `UAnnotationComponent` instances on those mesh components to drive annotation rendering.
  - Maintains maps between component names, colors, values, and components for lookup and updates.
  - Provides the list of annotation components used by segmentation and annotation cameras.
- `UAnnotationComponent` (in `AnnotationComponent.*`)
  - Provides a render proxy that overrides materials to output flat annotation colors or textures.
  - Attaches to a parent mesh component and uses the parent?s bounds and render data.
  - Suppresses rendering in normal material-based views so it only appears in annotation passes.
- `APIPCamera` (in `PIPCamera.*`)
  - Owns the segmentation scene capture, sets annotation-specific show flags, and uses a show-only list of annotation components.
  - Hides annotation components from regular scene and lighting captures and player view.
  - Supports extra annotation layers beyond segmentation (RGB, greyscale, texture) by adding extra scene capture components.
- `ALidarCamera` (in `LidarCamera.*`)
  - Uses its own segmentation scene capture configured like the PIP segmentation camera when ground-truth is enabled.
  - Receives the same annotation component list from `ASimModeBase`.
- `ASimModeBase` (in `SimModeBase.cpp`)
  - Initializes the instance segmentation and infrared annotators and refreshes their component lists.
  - Pushes annotation component lists to all PIP cameras, LiDAR cameras, and camera director views.
  - Exposes add/remove/update hooks for segmentation and annotation layers.
- `UnrealImageCapture` (in `UnrealImageCapture.cpp`)
  - Ensures capture components are enabled before capture and disables gamma for segmentation/annotation/infrared.
- `WorldSimApi` (in `WorldSimApi.*`)
  - API entrypoint that forwards segmentation and annotation commands to the active sim mode.
- `UAirBlueprintLib` (in `AirBlueprintLib.h`)
  - Contains mesh naming helpers used by other systems; includes special handling for instanced foliage actor naming.

## Segmentation and Annotation Flow
1. **Initialization**
   - `ASimModeBase` creates the instance segmentation annotator and infrared annotator and calls `Initialize` on the level.
   - `FObjectAnnotator` walks each actor, finds paintable mesh components, and creates `UAnnotationComponent` proxies with assigned colors.
2. **Component Discovery and Mapping**
   - Component names are assembled from mesh names, indices, actor names, and a persistent primitive index; this name is the identity used to map colors/IDs.
   - Colors are assigned from a deterministic color map and recorded in maps for lookup and API calls.
3. **Render Setup for Segmentation**
   - Segmentation captures use a show-only list of annotation components.
   - Annotation show flags disable materials, lighting, post-processing, and several other features to preserve the flat-color output.
   - Render targets for segmentation are configured for non-gamma corrected output, and image capture further disables gamma.
4. **Runtime Updates**
   - `ASimModeBase::updateInstanceSegmentationAnnotation` gathers the current annotation components and updates all PIP and LiDAR cameras.
   - Per-annotation-layer cameras are updated similarly via `updateAnnotation`.
5. **Capture**
   - `UnrealImageCapture` requests the correct capture component and render target, then reads pixels with gamma disabled for segmentation/annotation/infrared.

## Observed Constraints in the Current Design
- Annotation components are created per mesh component, not per mesh instance; this means instance-level differentiation is not available unless the underlying component provides instance-level rendering.
- The annotation render proxy is only specialized for static mesh and skeletal mesh components. Instanced mesh component types are not explicitly handled.
- Annotation rendering is entirely dependent on annotation components being present and renderable; the original mesh render path is intentionally excluded from segmentation captures.

## Bug Diagnosis: Hierarchical Instanced Static Mesh (HISM)
### Symptom
- When an actor is created using a hierarchical instanced static mesh component, segmentation output shows the base hierarchical instance mesh or a single proxy instead of the full set of instances, resulting in incorrect segmentation for that actor.

### Likely Root Cause
- `UAnnotationComponent` only builds render proxies for static mesh and skeletal mesh parents. Instanced and hierarchical instanced static mesh components use specialized scene proxies that carry per-instance transforms.
- The annotation proxy uses a non-instanced static mesh scene proxy, which does not render per-instance transforms. For HISM, this collapses multiple instances into a single render or otherwise fails to represent the full actor geometry.
- The segmentation camera only renders annotation components via the show-only list. If the annotation proxy does not draw all instances, the segmentation output cannot recover the missing geometry.

### Contributing Design Limits
- Component identity and color mapping are per component. A HISM component represents many instances, so all instances share a single component identity and color. This is acceptable only if the render proxy actually draws all instances, which it currently does not.
- Annotation show flags explicitly disable instanced foliage and instanced grass; if HISM is used by foliage systems, those flags further suppress expected output.

## Impacted Outputs
- PIP segmentation images.
- LiDAR ground-truth segmentation and any downstream labeling based on those captures.
- Any annotation layers that rely on the same annotation component mechanism.

## Remediation Options (No Code)
1. **Instanced Proxy Support**
   - Extend the annotation component to detect instanced and hierarchical instanced mesh parents and use the correct instanced scene proxy so all instances render with the annotation material.
2. **Per-Instance Annotation Proxies**
   - For HISM components, generate lightweight per-instance annotation proxies (or temporary static mesh components) that mirror instance transforms. This is heavier but preserves instance visibility without changing core proxy behavior.
3. **Explicit HISM Handling in Annotator**
   - Add a dedicated path in the annotator to treat HISM components differently, including proper instance enumeration and (if desired) instance-level IDs.
4. **Show Flag Review for Foliage Cases**
   - If HISM is driven by foliage, revisit the annotation show flags that disable instanced foliage and instanced grass for segmentation captures.

## Suggested Validation Steps
- Place a HISM actor with multiple instances and verify that all instances appear in the segmentation output.
- Compare segmentation output against a regular static mesh actor to confirm that annotation proxies still render correctly.
- Validate LiDAR ground-truth segmentation for HISM actors when `generate_groundtruth` is enabled.

## Assumptions and Open Questions
- The expected behavior is to render all instances of a HISM component with a single segmentation ID, rather than unique per-instance IDs.
- If per-instance IDs are required, the current component-level mapping is insufficient and needs design changes beyond proxy support.
