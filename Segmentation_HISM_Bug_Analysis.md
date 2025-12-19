# Segmentation Camera and Annotation System Analysis: HISM Rendering Bug

## Executive Summary

This document provides a comprehensive analysis of Cosys-AirSim's segmentation camera, annotation system, and PiP (Picture-in-Picture) camera architecture, with a focus on diagnosing a critical bug where actors created with Hierarchical Instanced Static Meshes (HISM) fail to render correctly in segmentation and annotation views.

**Bug Description:** When an actor is created using Hierarchical Instanced Static Mesh components, the segmentation camera and related annotation systems only render the HISM component itself instead of capturing the entire actor, resulting in incomplete or incorrect segmentation data.

---

## System Architecture Overview

### 1. Core Components

The segmentation and annotation systems in Cosys-AirSim consist of four major architectural layers:

#### Layer 1: Annotation Components (Proxy Mesh System)
- **Primary Classes:** `UAnnotationComponent`, `FStaticAnnotationSceneProxy`, `FSkeletalAnnotationSceneProxy`
- **Function:** Creates proxy meshes that override the original mesh rendering with solid colors or textures
- **Key Mechanism:** Uses Unreal's scene proxy pattern to inject custom render geometry

#### Layer 2: Object Annotators (Management Layer)
- **Primary Class:** `FObjectAnnotator`
- **Function:** Manages annotation data for all objects in the scene
- **Supported Types:**
  - Instance Segmentation (Type 3)
  - RGB Annotation (Type 0)
  - Greyscale Annotation (Type 1)
  - Texture Annotation (Type 2)
  - Infrared (Type 4)

#### Layer 3: Camera Systems
- **Primary Classes:** `APIPCamera`, `ALidarCamera`
- **Function:** Captures rendered scenes with specific show flags and filters
- **Render Modes:** Scene capture components with `PRM_UseShowOnlyList` primitive render mode

#### Layer 4: SimMode Integration
- **Primary Class:** `ASimModeBase`
- **Function:** Orchestrates initialization, actor registration, and API endpoints
- **Key Members:** `instance_segmentation_annotator_`, `infrared_annotator_`, `annotators_` map

---

## Component Identification System

### How Actors Are Discovered and Registered

The system uses a multi-step process to identify and register meshes for annotation:

#### Step 1: Actor Scanning (`getPaintableComponentMeshes`)
```
Process Flow:
1. Call actor->GetComponents<UMeshComponent>(paintable_components)
2. Iterate through each UMeshComponent found
3. Extract PersistentPrimitiveIndex from SceneProxy if available
4. Generate unique component name based on mesh name, actor hierarchy, and index
5. Store in name_to_component_map_ and component_to_name_map_
```

**Naming Convention:**
- Single component actors: `MeshName_0_ParentName_ActorName_PrimitiveIndex`
- Multi-component actors: `MeshName_ComponentIndex_ActorName_PrimitiveIndex`

#### Step 2: Supported Component Types
The system explicitly checks for:
- `UStaticMeshComponent`
- `USkinnedMeshComponent` (includes USkeletalMeshComponent)

**Critical Limitation:** The system uses `actor->GetComponents<UMeshComponent>()` which retrieves direct child components but has limitations with complex component hierarchies.

---

## The HISM Bug: Root Cause Analysis

### Problem Description

When an actor contains a Hierarchical Instanced Static Mesh Component (HISM), the annotation system fails to properly render the entire actor. Instead, only the HISM component itself is captured.

### Critical Distinction: Scene vs Segmentation Rendering

**Scene Camera renders:**
- The ORIGINAL mesh components directly
- Uses the actor's actual geometry and transforms
- HISM components render ALL instances automatically via Unreal's instanced rendering system
- Direct pipeline: Actor Component → Scene Proxy → Rendered Geometry

**Segmentation Camera renders:**
- PROXY annotation components, NOT the original meshes
- Uses mirrored geometry created by UAnnotationComponent
- Each original mesh component gets a corresponding annotation proxy
- Indirect pipeline: Actor Component → Annotation Component → Annotation Scene Proxy → Rendered Geometry

**The Problem:** For HISM, the proxy system creates ONE annotation component for the entire HISM component, but this single proxy doesn't replicate the multi-instance structure. The scene camera sees all HISM instances because it renders the original HISM directly. The segmentation camera only sees what the annotation proxy provides, which is typically just the base component's geometry without the instances.

### Root Cause: Component Discovery Limitation

The bug stems from how `GetComponents<UMeshComponent>()` interacts with HISM components:

1. **Component Hierarchy Issue:**
   - HISM components (`UHierarchicalInstancedStaticMeshComponent`) are subclasses of `UInstancedStaticMeshComponent` which inherits from `UStaticMeshComponent`
   - When `GetComponents<UMeshComponent>()` is called on an actor with HISM, it correctly identifies the HISM component
   - However, HISM components manage multiple instances internally, not as separate components

2. **Proxy Mesh Creation Problem:**
   - The annotation system creates ONE `UAnnotationComponent` proxy per detected `UMeshComponent`
   - For HISM, this creates a single proxy for the HISM component itself
   - The proxy attempts to mirror the HISM's geometry, but HISM instances are rendered through specialized instanced rendering paths

3. **Scene Proxy Mismatch:**
   - `FStaticAnnotationSceneProxy` is designed for standard `FStaticMeshSceneProxy`
   - HISM uses `FHierarchicalStaticMeshSceneProxy` with completely different rendering logic
   - The annotation proxy doesn't properly replicate HISM instance data

4. **ShowOnlyList Filtering:**
   - PiP cameras use `PRM_UseShowOnlyList` mode and add annotation components to `ShowOnlyComponents`
   - For HISM, only the base HISM component gets added, not the individual instances
   - The camera renders what it "sees" in the ShowOnlyList, which is the incomplete HISM structure

### Why Other Mesh Types Work

- **Static Meshes:** Direct 1:1 mapping between component and proxy - the proxy's `FStaticAnnotationSceneProxy` successfully mirrors the static mesh geometry
- **Skeletal Meshes:** Custom `FSkeletalAnnotationSceneProxy` properly replicates skeletal mesh render data and bone transforms
- **Standard Instanced Static Meshes:** May have similar issues but less commonly used

### Geometry Rendering Differences

**Important:** The segmentation camera does NOT render the same geometry as the scene camera. It renders DIFFERENT geometry:

1. **For Standard Meshes (Working Correctly):**
   - Scene: Renders `UStaticMeshComponent` → Uses mesh's vertex/index buffers
   - Segmentation: Renders `UAnnotationComponent` proxy → Copies same vertex/index buffers from parent
   - Result: Identical geometry, different materials

2. **For HISM (Bug Scenario):**
   - Scene: Renders `UHierarchicalInstancedStaticMeshComponent` → GPU instancing renders 100s-1000s of instances
   - Segmentation: Renders `UAnnotationComponent` proxy → Only has access to base mesh, no instance transform data
   - Result: DIFFERENT geometry - scene shows all instances, segmentation shows only one (or incomplete set)

3. **Hidden Components:**
   - Scene: Shows original mesh components, hides annotation proxies
   - Segmentation: Shows ONLY annotation proxies (via ShowOnlyComponents), hides everything else
   - These are mutually exclusive - one camera never sees what the other renders

---

## Annotation Component System Details

### Proxy Mesh Rendering Architecture

The annotation system works by creating "proxy" components that shadow the original mesh:

#### UAnnotationComponent Lifecycle

1. **Creation:**
   - Created in `PaintRGBComponent()` / `PaintTextureComponent()`
   - Attached to the parent mesh component
   - Registered with the scene

2. **Visibility Control:**
   - `SetVisibleInSceneCaptureOnly(true)` - Only visible to scene capture cameras
   - `SetVisibleInRayTracing(false)` - Hidden from ray tracing
   - `bRenderInMainPass = false` - Not rendered in main viewport

3. **Scene Proxy:**
   - `CreateSceneProxy()` generates appropriate proxy based on parent type
   - Static meshes → `FStaticAnnotationSceneProxy`
   - Skeletal meshes → `FSkeletalAnnotationSceneProxy`

4. **Material Override:**
   - Uses `AnnotationMaterial` or `SphereMaterial` with dynamic color parameters
   - MaterialRenderProxy set to custom MID with annotation color

#### View Relevance Override

Critical for performance and correctness:
```
When EngineShowFlags.Materials == true (normal rendering):
  - Returns DrawRelevance = 0 (invisible)
When EngineShowFlags.Materials == false (annotation rendering):
  - Returns full view relevance (visible)
```

This ensures annotation proxies only render in segmentation/annotation cameras where Materials flag is disabled.

---

## Camera Capture Pipeline

### PiP Camera Architecture

`APIPCamera` manages multiple capture types simultaneously:

#### Image Type Support
- **Scene:** Standard RGB camera view
- **DepthPlanar/DepthPerspective/DepthVis:** Depth rendering modes
- **Segmentation:** Instance segmentation rendering
- **Annotation:** Custom multi-layer annotation rendering
- **Infrared:** Thermal-like ID rendering
- **SurfaceNormals/OpticalFlow/Lighting:** Specialized render modes

#### Capture Component Configuration

For segmentation and annotation modes:

1. **Show Flags Setup:**
   - Materials: OFF (disables normal materials)
   - Lighting: OFF
   - Post Processing: OFF
   - BSP Triangles: ON
   - Atmosphere, Fog, Bloom, etc.: OFF

2. **Primitive Render Mode:**
   - Set to `PRM_UseShowOnlyList`
   - Only primitives in `ShowOnlyComponents` array are rendered
   - All other scene objects invisible

3. **Render Target:**
   - Gamma correction disabled for segmentation/annotation (TargetGamma = 1)
   - Pixel format: PF_B8G8R8A8 for color-based modes

#### Update Mechanism

When annotation data changes:
```
updateInstanceSegmentationAnnotation(ComponentList, only_hide):
  1. Set ShowOnlyComponents = ComponentList (all annotation proxies)
  2. Add annotation components to HiddenComponents of Scene/Lighting captures
  3. Add to PlayerController HiddenPrimitiveComponents (hide from main view)
```

This ensures:
- Annotation proxies visible ONLY in segmentation/annotation cameras
- Hidden from normal scene rendering
- Hidden from player view

---

## Instance Segmentation Specifics

### Initialization Process

1. **Level Scan (InitializeInstanceSegmentation):**
   - Iterates through all actors in the level
   - Checks if actor is "paintable" (has mesh components)
   - For each mesh component, creates annotation component with unique color

2. **Color Assignment:**
   - Uses `FColorGenerator` with predefined color map (2,744,000 unique colors)
   - Avoids problematic RGB values that might conflict with post-processing
   - Gamma correction table applied to ensure accurate color reproduction

3. **Component Tracking:**
   - Maps: name → component, component → name, color → name
   - Allows bidirectional lookup for API queries
   - Tracks persistent primitive index for Unreal's scene management

### Runtime Actor Addition

When new actors spawn (`AddNewActorToInstanceSegmentation`):
```
1. Check if actor is paintable
2. Get component meshes with tags
3. Check for "InstanceSegmentation_disable" tag → skip if present
4. If component already annotated → update color
5. If new component → assign next available color index
6. Create annotation proxy component
7. Update camera ShowOnlyComponents lists
```

---

## Multi-Layer Annotation System

### Layer Types and Configuration

Defined in settings.json:

```json
"Annotation": [
  {
    "Name": "LayerName",
    "Type": 0,  // 0=RGB, 1=Greyscale, 2=Texture
    "Default": true,  // Show unlabeled objects
    "SetDirect": false,  // false=index mode, true=direct color mode
    "ViewDistance": 10  // Maximum render distance in meters
  }
]
```

### Tag-Based Assignment

Actors/components tagged with format:
- RGB Index: `LayerName_ColorIndex`
- RGB Direct: `LayerName_R_G_B`
- Greyscale: `LayerName_FloatValue`
- Texture Direct: `LayerName_/Path/To/Texture`
- Texture Relative: `LayerName_enable`

### Dynamic Annotation Cameras

For each annotation layer, a new scene capture component is created dynamically:
- Stored in expandable `captures_` array
- Registered with `annotator_name_to_index_map_`
- Optionally includes view distance sphere (annotation_sphere)

---

## LiDAR Camera Integration

`ALidarCamera` also supports segmentation:

### Dual Capture Setup
- **3D Point Cloud:** Primary LiDAR data
- **2D Segmentation Image:** Parallel camera capture for color labeling
- **Instance Segmentation Mode:** Links point cloud colors to object IDs

### Configuration
```
capture_2D_segmentation_:
  - SetViewForAnnotationRender(ShowFlags)
  - PRM_UseShowOnlyList mode
  - Linked to instance_segmentation_ boolean flag
```

This allows LiDAR point clouds to carry RGB values representing object instance IDs.

---

## Bug Impact Analysis

### Affected Functionality

1. **Instance Segmentation:**
   - HISM-based actors render incompletely
   - Color assignments may be inconsistent
   - Object bounding boxes incorrect

2. **Annotation Layers:**
   - RGB/Greyscale/Texture annotations fail for HISM actors
   - Only base HISM component receives annotation, not instances

3. **GPU LiDAR:**
   - Point cloud segmentation data incorrect for HISM objects
   - Instance labeling incomplete

4. **API Queries:**
   - `listInstanceSegmentationObjects()` may list HISM but with wrong data
   - `listInstanceSegmentationPoses()` returns only HISM component position, not instance positions

### Scenarios Triggering the Bug

- Procedurally generated environments using HISM for performance
- Foliage/vegetation converted to HISM for wind simulation
- Large-scale instanced objects (buildings, props) using HISM
- Dynamic object spawning that creates HISM components

---

## Potential Solutions

### Solution 1: HISM-Aware Component Discovery

**Approach:** Detect HISM components and iterate through their instances

**Implementation Location:** `FObjectAnnotator::getPaintableComponentMeshes`

**Concept:**
- Check if component is `UHierarchicalInstancedStaticMeshComponent`
- Use `GetInstanceCount()` to enumerate instances
- Use `GetInstanceTransform()` to get each instance's world transform
- Create separate annotation components per instance OR create specialized HISM annotation proxy

**Challenges:**
- HISM instances aren't separate components, requiring special handling
- Performance impact of creating thousands of proxy components
- Scene proxy architecture doesn't map well to instanced rendering

### Solution 2: Custom HISM Scene Proxy

**Approach:** Create `FHISMAnnotationSceneProxy` that understands instanced rendering

**Implementation:** New proxy class similar to `FStaticAnnotationSceneProxy`

**Concept:**
- Inherit from or mirror `FHierarchicalStaticMeshSceneProxy`
- Override material rendering for all instances
- Maintain instance transform data
- Use instanced rendering API with annotation materials

**Challenges:**
- Complex Unreal rendering internals
- Requires deep understanding of instanced mesh rendering pipeline
- May need access to private Unreal Engine code

### Solution 3: Pre-Render Instance Expansion

**Approach:** Expand HISM instances to individual static mesh components before annotation

**Implementation:** Actor conversion utility

**Concept:**
- Detect HISM in actor
- Create individual `UStaticMeshComponent` for each instance
- Remove HISM component
- Apply existing annotation system

**Challenges:**
- Performance degradation (HISM exists for optimization)
- Large memory overhead
- Loses dynamic instancing benefits
- Requires environment modification

### Solution 4: Render Target Instance Tagging

**Approach:** Use GPU-based instance ID rendering

**Implementation:** Custom post-process material or render pass

**Concept:**
- Leverage Unreal's instance ID system in shaders
- Modify segmentation render pass to output per-instance colors
- Bypass proxy mesh system for HISM
- Use compute shader or post-process to remap instance IDs to segmentation colors

**Challenges:**
- Requires shader programming
- May need modified Unreal Engine source
- Integration with existing proxy system complex

### Solution 5: ShowOnlyList Expansion for HISM

**Approach:** Modify camera update logic to handle HISM specially

**Implementation:** `updateInstanceSegmentationAnnotation` in `APIPCamera`

**Concept:**
- Detect HISM components in component list
- Add HISM with all instances to ShowOnlyComponents
- Modify camera rendering to treat HISM instances as separate render items
- Use Unreal's primitive component system to expose instances

**Challenges:**
- ShowOnlyComponents list doesn't natively understand instances
- May require engine modification
- Unclear if instance-level filtering is possible without source changes

---

## Recommended Investigation Steps

1. **Verify HISM Detection:**
   - Add logging to `getPaintableComponentMeshes` to identify when HISM is encountered
   - Check if HISM components are being added to `name_to_component_map_`

2. **Test Instance Count:**
   - For detected HISM, log `GetInstanceCount()` value
   - Confirm if instances are accessible at component scan time

3. **Examine Scene Proxy Creation:**
   - Add breakpoint in `UAnnotationComponent::CreateSceneProxy`
   - Check what type of parent component is passed for HISM actors
   - Verify if `FStaticAnnotationSceneProxy` is created for HISM

4. **Camera Render Analysis:**
   - Enable `stat SceneRendering` in Unreal
   - Check primitive counts in segmentation camera vs. scene camera
   - Verify if ShowOnlyComponents includes expected primitives

5. **Color Output Test:**
   - Capture segmentation image of HISM actor
   - Analyze pixel colors to see if ANY annotation data appears
   - Compare with non-HISM equivalent actor

---

## System Limitations

### Known Constraints

1. **Mesh Type Support:**
   - Only StaticMesh and SkeletalMesh fully supported
   - Landscape, Foliage, Brush geometry unsupported
   - Decals, text render components excluded

2. **Color Space:**
   - 2,744,000 unique colors maximum
   - Certain RGB values avoided due to post-processing conflicts
   - Gamma correction applied for accurate reproduction

3. **Performance Considerations:**
   - Annotation component creation is expensive
   - Full level initialization can take seconds
   - Per-frame render state updates for skeletal meshes

4. **Tag System Dependencies:**
   - Requires manual or API-based tagging
   - No automatic semantic understanding
   - Tag format must be exact

---

## Related Systems and Files

### Key Source Files

**Annotation:**
- `AnnotationComponent.cpp/h` - Proxy component implementation
- `ObjectAnnotator.cpp/h` - Annotation manager

**Cameras:**
- `PIPCamera.cpp/h` - Multi-camera capture system
- `LidarCamera.cpp/h` - LiDAR with segmentation

**SimMode:**
- `SimModeBase.cpp/h` - Top-level orchestration

**Rendering:**
- `UnrealImageCapture.cpp/h` - Image request handling
- `RenderRequest.cpp/h` - Render pipeline coordination

**APIs:**
- `WorldSimApi.cpp/h` - RPC endpoint implementations

### Unreal Engine Classes Referenced

- `USceneCaptureComponent2D` - Camera capture mechanism
- `FPrimitiveSceneProxy` - Rendering representation
- `UMeshComponent` - Base mesh component type
- `UStaticMeshComponent` - Static mesh type
- `USkeletalMeshComponent` - Skeletal mesh type
- `UHierarchicalInstancedStaticMeshComponent` - HISM type
- `FEngineShowFlags` - Render feature toggles

---

## Conclusion

The HISM rendering bug in Cosys-AirSim's segmentation and annotation systems stems from a fundamental architectural assumption: one mesh component equals one renderable object. HISM violates this assumption by packing multiple visual instances into a single component for performance optimization.

The annotation system's component discovery mechanism (`GetComponents<UMeshComponent>`) correctly identifies the HISM component but has no awareness of the internal instance structure. Consequently, the proxy mesh system creates a single annotation proxy for the entire HISM component rather than per-instance proxies.

When the segmentation camera renders using `ShowOnlyComponents` filtering, it displays the HISM annotation proxy, which only represents the base component's geometry and transform, not the individual instances. This results in incomplete or incorrect segmentation output.

**Solving this bug requires either:**
1. Extending the component discovery system to understand and expand HISM instances
2. Creating a specialized scene proxy that handles instanced rendering with per-instance annotation
3. Modifying the camera rendering pipeline to treat HISM instances as separate render primitives

Each approach has significant implementation complexity and trade-offs between performance, code maintainability, and architectural purity. The optimal solution likely involves a combination of HISM detection, specialized proxy rendering, and camera pipeline modifications.
