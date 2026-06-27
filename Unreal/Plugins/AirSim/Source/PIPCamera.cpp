#include "PIPCamera.h"
#include "AirSimEquirectangularPreview.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "Engine/World.h"
#include "ImageUtils.h"
#include "Annotation/AnnotationComponent.h"
#include "Annotation/ObjectAnnotator.h"
#include <string>
#include <exception>
#include "AirBlueprintLib.h"

namespace
{
    bool UsesSourceStencilAnnotationCamera(FObjectAnnotator::AnnotatorType type, bool use_source_stencil_backend = false)
    {
        return use_source_stencil_backend ||
               type == FObjectAnnotator::AnnotatorType::InstanceSegmentation ||
               type == FObjectAnnotator::AnnotatorType::Infrared;
    }

    int32 ResolveRenderIndex(APIPCamera::ImageType type,
                             const std::string& annotation_name,
                             const TMap<FString, int>& annotator_name_to_index_map)
    {
        if (type == APIPCamera::ImageType::Annotation) {
            const int* render_index = annotator_name_to_index_map.Find(FString(annotation_name.c_str()));
            return render_index == nullptr ? INDEX_NONE : *render_index;
        }

        return static_cast<int32>(type);
    }

    bool UsesGlobalEquirectangularScenePipeline(int image_type)
    {
        return image_type == static_cast<int>(APIPCamera::ImageType::Scene) ||
               image_type == static_cast<int>(APIPCamera::ImageType::Lighting);
    }

    void ApplyGlobalEquirectangularSceneSettings(USceneCaptureComponentCube* capture)
    {
        if (capture == nullptr) {
            return;
        }

        // Capture all cube faces before view-local post processing. Per-face
        // exposure, local exposure, bloom, and lens effects create visible seams;
        // the unwrap path applies one global exposure and tonemap after sampling.
        capture->CaptureSource = ESceneCaptureSource::SCS_SceneColorHDRNoAlpha;
        capture->ShowFlags.SetPostProcessing(false);
        capture->ShowFlags.SetTonemapper(false);
        capture->ShowFlags.SetTemporalAA(false);
        capture->ShowFlags.SetMotionBlur(false);
        capture->ShowFlags.SetScreenSpaceReflections(false);
        capture->ShowFlags.SetAmbientOcclusion(false);
        capture->ShowFlags.SetScreenSpaceAO(false);
        capture->ShowFlags.SetDistanceFieldAO(false);
        capture->ShowFlags.SetContactShadows(false);
        capture->ShowFlags.SetLightShafts(false);
        capture->ShowFlags.SetEyeAdaptation(false);
        capture->ShowFlags.SetLocalExposure(false);
        capture->ShowFlags.SetBloom(false);
        capture->ShowFlags.SetLensFlares(false);
        capture->ShowFlags.SetDepthOfField(false);
        capture->ShowFlags.SetVignette(false);
    }

    bool UsesNearestEquirectangularPreview(APIPCamera::ImageType image_type)
    {
        return image_type == APIPCamera::ImageType::Segmentation ||
               image_type == APIPCamera::ImageType::Infrared ||
               image_type == APIPCamera::ImageType::Annotation;
    }

    bool UsesDepthEquirectangularPreview(APIPCamera::ImageType image_type)
    {
        return image_type == APIPCamera::ImageType::DepthPlanar ||
               image_type == APIPCamera::ImageType::DepthPerspective;
    }

    AirSimEquirectangularPreview::EPreviewMode GetEquirectangularPreviewMode(APIPCamera::ImageType image_type)
    {
        if (image_type == APIPCamera::ImageType::Scene ||
            image_type == APIPCamera::ImageType::Lighting) {
            return AirSimEquirectangularPreview::EPreviewMode::SceneColor;
        }
        if (UsesDepthEquirectangularPreview(image_type)) {
            return AirSimEquirectangularPreview::EPreviewMode::DepthMeters;
        }
        if (UsesNearestEquirectangularPreview(image_type)) {
            return AirSimEquirectangularPreview::EPreviewMode::ColorNearest;
        }
        return AirSimEquirectangularPreview::EPreviewMode::ColorBilinear;
    }
}

//CinemAirSim
APIPCamera::APIPCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer
                .SetDefaultSubobjectClass<UCineCameraComponent>(TEXT("CameraComponent")))
{
    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraSensorNoise.CameraSensorNoise'"));
    if (mat_finder.Succeeded()) {
        noise_material_static_ = mat_finder.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create noise material for the PIPCamera",
                                           "",
                                           LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> dist_mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraDistortion.CameraDistortion'"));
    if (dist_mat_finder.Succeeded()) {
        distortion_material_static_ = dist_mat_finder.Object;
        distortion_param_collection_ = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, TEXT("'/AirSim/HUDAssets/DistortionParams.DistortionParams'")));
    }
    else{
        UAirBlueprintLib::LogMessageString("Cannot create distortion material for the PIPCamera",
                                           "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder2(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortion.CameraSensorLensDistortion'"));
	if (mat_finder2.Succeeded())
	{
		lens_distortion_material_static_ = mat_finder2.Object;
	}
	else{
		UAirBlueprintLib::LogMessageString("Cannot create lens distortion material for the PIPCamera", "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder3(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortionInvert.CameraSensorLensDistortionInvert'"));
	if (mat_finder3.Succeeded())
	{
		lens_distortion_invert_material_static_ = mat_finder3.Object;
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create inverted lens distortion material for the PIPCamera",
			"", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder4(TEXT("Material'/AirSim/HUDAssets/CameraSensorMotionBlur.CameraSensorMotionBlur'"));
    if (mat_finder4.Succeeded())
    {
        motion_blur_material_static_ = mat_finder4.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create fake motion blur material for the PIPCamera",
            "", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder5(TEXT("Material'/AirSim/HUDAssets/CameraSensorRadialBlur.CameraSensorRadialBlur'"));
    if (mat_finder5.Succeeded())
    {
        radial_blur_material_static_ = mat_finder5.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create radial blur material for the PIPCamera",
            "", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder6(TEXT("Material'/AirSim/HUDAssets/CameraSensorGuassianBlur.CameraSensorGuassianBlur'"));
    if (mat_finder6.Succeeded())
    {
        guassian_blur_material_static_ = mat_finder6.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create guassian blur material for the PIPCamera",
            "", LogDebugLevel::Failure);


    PrimaryActorTick.bCanEverTick = true;

    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Scene), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPlanar), EPixelFormat::PF_DepthStencil); // not used. init_auto_format is called in setupCameraFromSettings()
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPerspective), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthVis), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DisparityNormalized), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Segmentation), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::SurfaceNormals), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Infrared), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlow), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlowVis), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Lighting), EPixelFormat::PF_B8G8R8A8);

    object_filter_ = FObjectFilter();

    static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Models/AnnotationSphere.AnnotationSphere'"));
    if (loadedMesh.Succeeded())
    {
        annotation_sphere_static_ = loadedMesh.Object;
    }

    static ConstructorHelpers::FObjectFinder<UMaterial> segmentation_stencil_mat_finder(TEXT("Material'/AirSim/HUDAssets/SegmentationMaterial.SegmentationMaterial'"));
    if (segmentation_stencil_mat_finder.Succeeded()) {
        segmentation_stencil_material_static_ = segmentation_stencil_mat_finder.Object;
    }
    else {
        UAirBlueprintLib::LogMessageString("Cannot create source stencil segmentation material for the PIPCamera",
                                           "", LogDebugLevel::Failure);
    }

    static ConstructorHelpers::FObjectFinder<UMaterial> infrared_stencil_mat_finder(TEXT("Material'/AirSim/HUDAssets/InfraredMaterial.InfraredMaterial'"));
    if (infrared_stencil_mat_finder.Succeeded()) {
        infrared_stencil_material_static_ = infrared_stencil_mat_finder.Object;
    }
    else {
        UAirBlueprintLib::LogMessageString("Cannot create source stencil infrared material for the PIPCamera",
                                           "", LogDebugLevel::Failure);
    }
}

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    //CinemAirSim
    camera_ = UAirBlueprintLib::GetActorComponent<UCineCameraComponent>(this, TEXT("CameraComponent"));
    captures_.Init(nullptr, imageTypeCount());
    render_targets_.Init(nullptr, imageTypeCount());
    equirectangular_captures_.Init(nullptr, imageTypeCount());
    equirectangular_render_targets_.Init(nullptr, imageTypeCount());
    equirectangular_preview_targets_.Init(nullptr, imageTypeCount());
    equirectangular_preview_updates_.Init(false, imageTypeCount());
    detections_.Init(nullptr, imageTypeCount());

    captures_[Utils::toNumeric(ImageType::Scene)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPlanar)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPlanarCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPerspective)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPerspectiveCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DisparityNormalized)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DisparityNormalizedCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Segmentation)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Infrared)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("InfraredCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlow)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlowVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Lighting)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("LightingCaptureComponent"));

    for (unsigned int i = 0; i < imageTypeCount(); ++i) {
        detections_[i] = NewObject<UDetectionComponent>(this);
        if (detections_[i]) {
            detections_[i]->SetupAttachment(captures_[i]);
            detections_[i]->RegisterComponent();
            detections_[i]->Deactivate();
        }
    }
    //set initial focal length
    camera_->CurrentFocalLength = 11.9;

    configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Segmentation)], FObjectAnnotator::AnnotatorType::InstanceSegmentation);
    configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Infrared)], FObjectAnnotator::AnnotatorType::Infrared);

    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetLighting(true);
    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetMaterials(false);
    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetPostProcessing(false);
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();

    noise_materials_.AddZeroed(imageTypeCount() + 1);
    distortion_materials_.AddZeroed(imageTypeCount() + 1);
	lens_distortion_materials_.AddZeroed(imageTypeCount() + 1);
    fake_motion_blur_materials_.AddZeroed(imageTypeCount() + 1);
    radial_blur_materials_.AddZeroed(imageTypeCount() + 1);
    guassian_blur_materials_.AddZeroed(imageTypeCount() + 1);

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        if(image_type == Utils::toNumeric(ImageType::Scene) || image_type == Utils::toNumeric(ImageType::Lighting)) {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        }
        else {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        }      
        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
    }

    //We set all cameras to start as nodisplay
    //This improves performance because the capture components are no longer updating every frame and only update while requesting an image
    onViewModeChanged(true);

    gimbal_stabilization_ = 0;
    gimbald_rotator_ = this->GetActorRotation();
    this->SetActorTickEnabled(false);
    

    if (distortion_param_collection_)
        distortion_param_instance_ = this->GetWorld()->GetParameterCollectionInstance(distortion_param_collection_);
}


msr::airlib::AirSimSettings::CameraSetting APIPCamera::getParams() const
{
    return sensor_params_;
}

msr::airlib::ProjectionMatrix APIPCamera::getProjectionMatrix() const
{
    msr::airlib::ProjectionMatrix mat;

    // TODO: This is always the case in current request, might need to change to include annotation if needed
	ImageType image_type = ImageType::Scene;

    if (isEquirectangularCapture(image_type)) {
        mat.setTo(Utils::nan<float>());
        return mat;
    }

    //TODO: avoid the need to override const cast here
    const_cast<APIPCamera*>(this)->setCameraTypeEnabled(image_type, true);
    const USceneCaptureComponent2D* capture = const_cast<APIPCamera*>(this)->getCaptureComponent(image_type, false);
    if (capture && capture->TextureTarget) {
        FMatrix proj_mat_transpose;

        FIntPoint render_target_size(capture->TextureTarget->GetSurfaceWidth(), capture->TextureTarget->GetSurfaceHeight());
        float x_axis_multiplier = 1.0f;
        float y_axis_multiplier = render_target_size.X / (float)render_target_size.Y;

        if (render_target_size.X < render_target_size.Y) {
            // if the viewport is taller than it is wide
            x_axis_multiplier = render_target_size.Y / static_cast<float>(render_target_size.X);
            y_axis_multiplier = 1.0f;
        }

        if (capture->ProjectionType == ECameraProjectionMode::Orthographic) {
            check((int32)ERHIZBuffer::IsInverted);
            const float OrthoWidth = capture->OrthoWidth / 2.0f;
            const float OrthoHeight = capture->OrthoWidth / 2.0f * x_axis_multiplier / y_axis_multiplier;

            const float NearPlane = 0;
            const float FarPlane = WORLD_MAX / 8.0f;

            const float ZScale = 1.0f / (FarPlane - NearPlane);
            const float ZOffset = -NearPlane;

            proj_mat_transpose = FReversedZOrthoMatrix(
                OrthoWidth,
                OrthoHeight,
                ZScale,
                ZOffset);
        }
        else {
            float halfFov = Utils::degreesToRadians(capture->FOVAngle) / 2;
            if ((int32)ERHIZBuffer::IsInverted) {
                proj_mat_transpose = FReversedZPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
            else {
                //The FPerspectiveMatrix() constructor actually returns the transpose of the perspective matrix.
                proj_mat_transpose = FPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
        }

        //Takes a vector from NORTH-EAST-DOWN coordinates (AirSim) to EAST-UP-SOUTH coordinates (Unreal). Leaves W coordinate unchanged.
        FMatrix coordinateChangeTranspose = FMatrix(
            FPlane(0, 0, -1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1));

        FMatrix projMatTransposeInAirSim = coordinateChangeTranspose * proj_mat_transpose;

        //Copy the result to an airlib::ProjectionMatrix while taking transpose.
        for (auto row = 0; row < 4; ++row)
            for (auto col = 0; col < 4; ++col)
                mat.matrix[col][row] = projMatTransposeInAirSim.M[row][col];
    }
    else
        mat.setTo(Utils::nan<float>());

    return mat;
}

void APIPCamera::Tick(float DeltaTime)
{
    if (gimbal_stabilization_ > 0) {
        FRotator rotator = this->GetActorRotation();
        if (!std::isnan(gimbald_rotator_.Pitch))
            rotator.Pitch = gimbald_rotator_.Pitch * gimbal_stabilization_ +
                            rotator.Pitch * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Roll))
            rotator.Roll = gimbald_rotator_.Roll * gimbal_stabilization_ +
                           rotator.Roll * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Yaw))
            rotator.Yaw = gimbald_rotator_.Yaw * gimbal_stabilization_ +
                          rotator.Yaw * (1 - gimbal_stabilization_);

        this->SetActorRotation(rotator);
    }
    if (sensor_params_.draw_sensor) {
        UAirBlueprintLib::DrawPoint(this->GetWorld(), this->GetActorTransform().GetLocation(), 5, FColor::Black, false, 0.3);
        UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), this->GetActorLocation(), this->GetActorRotation(), 25, false, 0.3, 10);
    }
    drawEquirectangularPreviews();
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    int image_count_to_delete = static_cast<int>(Utils::toNumeric(ImageType::Count));
    if (noise_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (noise_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(noise_materials_[image_type + 1]);
        }
        if (noise_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(noise_materials_[0]);
    }

	if (lens_distortion_materials_.Num()) {
		for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
			if (lens_distortion_materials_[image_type + 1])
				captures_[image_type]->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[image_type + 1]);
		}
		if (lens_distortion_materials_[0])
			camera_->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[0]);
	}

    if (radial_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (radial_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(radial_blur_materials_[image_type + 1]);
        }
        if (radial_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(radial_blur_materials_[0]);
    }

    if (guassian_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (guassian_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(guassian_blur_materials_[image_type + 1]);
        }
        if (guassian_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(guassian_blur_materials_[0]);
    }

    if (fake_motion_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (fake_motion_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(fake_motion_blur_materials_[image_type + 1]);
        }
        if (fake_motion_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(fake_motion_blur_materials_[0]);
    }

    noise_material_static_ = nullptr;
	lens_distortion_material_static_ = nullptr;
	lens_distortion_invert_material_static_ = nullptr;
    annotation_sphere_static_ = nullptr;
    guassian_blur_material_static_ = nullptr;
    radial_blur_material_static_ = nullptr;
    motion_blur_material_static_ = nullptr;
    noise_materials_.Empty();
	lens_distortion_materials_.Empty();
    radial_blur_materials_.Empty();
    guassian_blur_materials_.Empty();
    fake_motion_blur_materials_.Empty();

    if (distortion_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (distortion_materials_[image_type + 1])
            {
                if (captures_[image_type] != NULL) {
                    captures_[image_type]->PostProcessSettings.RemoveBlendable(distortion_materials_[image_type + 1]);
                }
            }                
        }
        if (distortion_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(distortion_materials_[0]);
    }

    distortion_material_static_ = nullptr;
    distortion_materials_.Empty();
    for (const auto& stencil_material_entry : source_stencil_annotation_material_map_) {
        if (IsValid(stencil_material_entry.Key) && IsValid(stencil_material_entry.Value)) {
            stencil_material_entry.Key->PostProcessSettings.RemoveBlendable(stencil_material_entry.Value);
        }
    }
    source_stencil_annotation_material_map_.Empty();
    source_stencil_annotation_materials_.Empty();
    segmentation_stencil_material_static_ = nullptr;
    infrared_stencil_material_static_ = nullptr;

	annotator_name_to_index_map_.Empty();
    annotator_name_to_type_map_.Empty();
    annotator_name_to_source_stencil_map_.Empty();
    sphere_annotation_component_map_.Empty();


    int camera_full_count = static_cast<int>(cameraCaptureCount());
    for (int current_camera = 0; current_camera < camera_full_count; ++current_camera) {
        //use final color for all calculations
        if (current_camera == Utils::toNumeric(ImageType::Segmentation) ||
            current_camera == Utils::toNumeric(ImageType::Infrared) ||
            current_camera >= image_count_to_delete - 2) {
            captures_[current_camera]->ShowOnlyComponents.Empty();
        }        
        captures_[current_camera] = nullptr;
        render_targets_[current_camera] = nullptr;
        equirectangular_captures_[current_camera] = nullptr;
        equirectangular_render_targets_[current_camera] = nullptr;
        equirectangular_preview_targets_[current_camera] = nullptr;
        detections_[current_camera] = nullptr;
    }
    captures_.Empty();
	render_targets_.Empty();
    equirectangular_captures_.Empty();
    equirectangular_render_targets_.Empty();
    equirectangular_preview_targets_.Empty();
    equirectangular_preview_updates_.Empty();
	detections_.Empty();

    Super::EndPlay(EndPlayReason);
}

unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count) - 1;
}

unsigned int APIPCamera::cameraCaptureCount()
{
    return static_cast<unsigned int>(captures_.Num());
}

void APIPCamera::updateActorTickEnabled()
{
    SetActorTickEnabled(gimbal_stabilization_ > 0 || sensor_params_.draw_sensor || hasActiveEquirectangularPreview());
}

bool APIPCamera::hasActiveEquirectangularPreview() const
{
    for (bool enabled : equirectangular_preview_updates_) {
        if (enabled) {
            return true;
        }
    }
    return false;
}

void APIPCamera::setEquirectangularPreviewUpdate(int render_index, bool enabled)
{
    if (render_index < 0) {
        return;
    }
    if (!equirectangular_preview_updates_.IsValidIndex(render_index)) {
        equirectangular_preview_updates_.SetNum(render_index + 1);
    }
    equirectangular_preview_updates_[render_index] = enabled &&
        equirectangular_preview_targets_.IsValidIndex(render_index) &&
        equirectangular_preview_targets_[render_index] != nullptr;
    updateActorTickEnabled();
}

void APIPCamera::ensureEquirectangularPreviewTarget(int render_index, const CaptureSetting& setting, ImageType type)
{
    if (render_index < 0) {
        return;
    }
    if (!equirectangular_preview_targets_.IsValidIndex(render_index)) {
        equirectangular_preview_targets_.SetNum(render_index + 1);
    }
    if (!equirectangular_preview_updates_.IsValidIndex(render_index)) {
        equirectangular_preview_updates_.SetNum(render_index + 1);
    }

    UTextureRenderTarget2D*& preview_target = equirectangular_preview_targets_[render_index];
    if (preview_target == nullptr) {
        preview_target = NewObject<UTextureRenderTarget2D>(this);
    }

    const int32 output_height = FMath::Max(1, static_cast<int32>(setting.height));
    const int32 output_width = output_height * 2;
    if (preview_target->SizeX != output_width || preview_target->SizeY != output_height ||
        preview_target->GetFormat() != EPixelFormat::PF_B8G8R8A8) {
        preview_target->InitCustomFormat(output_width, output_height, EPixelFormat::PF_B8G8R8A8, false);
    }

    if (UsesNearestEquirectangularPreview(type) || UsesDepthEquirectangularPreview(type)) {
        preview_target->TargetGamma = 1.0f;
    }
}

void APIPCamera::drawEquirectangularPreviews()
{
    for (int render_index = 0; render_index < equirectangular_preview_updates_.Num(); ++render_index) {
        if (!equirectangular_preview_updates_[render_index] ||
            !equirectangular_captures_.IsValidIndex(render_index) ||
            !equirectangular_render_targets_.IsValidIndex(render_index) ||
            !equirectangular_preview_targets_.IsValidIndex(render_index) ||
            equirectangular_captures_[render_index] == nullptr ||
            equirectangular_render_targets_[render_index] == nullptr ||
            equirectangular_preview_targets_[render_index] == nullptr) {
            continue;
        }

        const ImageType image_type = render_index < static_cast<int>(imageTypeCount())
            ? Utils::toEnum<ImageType>(render_index)
            : ImageType::Annotation;
        const int setting_index = image_type == ImageType::Annotation
            ? Utils::toNumeric(ImageType::Annotation)
            : render_index;
        if (setting_index < 0 || setting_index >= static_cast<int>(sensor_params_.capture_settings.size())) {
            continue;
        }

        const CaptureSetting& setting = sensor_params_.capture_settings.at(setting_index);
        AirSimEquirectangularPreview::Draw(
            equirectangular_render_targets_[render_index],
            equirectangular_preview_targets_[render_index],
            GetEquirectangularPreviewMode(image_type),
            setting.max_depth_meters,
            setting.equirectangular_exposure_compensation);
    }
}

void APIPCamera::showToScreen()
{
    camera_->SetVisibility(true);
    camera_->Activate();
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->SetViewTarget(this);
    UAirBlueprintLib::LogMessage(TEXT("Camera: "), GetName(), LogDebugLevel::Informational);
}

void APIPCamera::disableAll()
{
    disableMain();
    disableAllPIP();
}

bool APIPCamera::getCameraTypeEnabled(ImageType type, std::string annotation_name) const
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    return render_index >= 0 &&
           render_index < static_cast<int32>(camera_type_enabled_.size()) &&
           camera_type_enabled_[render_index];
}

bool APIPCamera::GetAnnotationNameExist(std::string annotation_name)
{
    if (annotator_name_to_index_map_.Contains(FString(annotation_name.c_str())))
        return true;
    else
        return false;
}

void APIPCamera::setCameraTypeEnabled(ImageType type, bool enabled, std::string annotation_name)
{
    enableCaptureComponent(type, enabled, annotation_name);
}

void APIPCamera::setCameraOrientation(const FRotator& rotator)
{
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    this->SetActorRelativeRotation(rotator);
}


void APIPCamera::setCaptureUpdate(USceneCaptureComponent2D* capture, bool nodisplay)
{
    capture->bCaptureEveryFrame = !nodisplay;
    capture->bCaptureOnMovement = !nodisplay;
    capture->bAlwaysPersistRenderingState = true;
}

void APIPCamera::setEquirectangularCaptureUpdate(USceneCaptureComponentCube* capture, bool nodisplay)
{
    if (capture == nullptr) {
        return;
    }

    capture->bCaptureEveryFrame = !nodisplay;
    capture->bCaptureOnMovement = !nodisplay;
    capture->bAlwaysPersistRenderingState = true;
}

void APIPCamera::ensureEquirectangularCapture(int render_index, const FString& name)
{
    if (render_index < 0) {
        return;
    }

    if (!equirectangular_captures_.IsValidIndex(render_index)) {
        equirectangular_captures_.SetNum(render_index + 1);
    }
    if (!equirectangular_render_targets_.IsValidIndex(render_index)) {
        equirectangular_render_targets_.SetNum(render_index + 1);
    }

    if (equirectangular_captures_[render_index] == nullptr) {
        const FString capture_name = name + TEXT("_EquirectangularCubeCapture");
        USceneCaptureComponentCube* cube_capture = NewObject<USceneCaptureComponentCube>(this, USceneCaptureComponentCube::StaticClass(), *capture_name);
        cube_capture->bAutoActivate = false;
        cube_capture->bCaptureRotation = true;
        cube_capture->SetRelativeRotation(FRotator(0, 0, 0));
        cube_capture->SetRelativeLocation(FVector(0, 0, 0));
        cube_capture->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
        cube_capture->RegisterComponent();
        cube_capture->Deactivate();
        setEquirectangularCaptureUpdate(cube_capture, true);
        equirectangular_captures_[render_index] = cube_capture;
    }

    if (equirectangular_render_targets_[render_index] == nullptr) {
        equirectangular_render_targets_[render_index] = NewObject<UTextureRenderTargetCube>(this);
    }
}

void APIPCamera::setCameraTypeUpdate(ImageType type, bool nodisplay, std::string annotation_name)
{
    if (isEquirectangularCapture(type, annotation_name)) {
        USceneCaptureComponentCube* cube_capture = getEquirectangularCaptureComponent(type, false, annotation_name);
        setEquirectangularCaptureUpdate(cube_capture, nodisplay);
        int render_index = Utils::toNumeric(type);
        if (type == ImageType::Annotation && annotator_name_to_index_map_.Contains(FString(annotation_name.c_str()))) {
            render_index = annotator_name_to_index_map_[FString(annotation_name.c_str())];
        }
        setEquirectangularPreviewUpdate(render_index, !nodisplay && getCameraTypeEnabled(type, annotation_name));
    }
    else {
        USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
        if (capture != nullptr)
            setCaptureUpdate(capture, nodisplay);
    }
}

void APIPCamera::setCameraPose(const msr::airlib::Pose& relative_pose)
{
    FTransform pose = ned_transform_->fromRelativeNed(relative_pose);

    FVector position = pose.GetLocation();
    this->SetActorRelativeLocation(pose.GetLocation());

    FRotator rotator = pose.GetRotation().Rotator();
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    else {
        this->SetActorRelativeRotation(rotator);
    }
}

void APIPCamera::setCameraFoV(float fov_degrees)
{
    for (unsigned int image_type = 0; image_type < cameraCaptureCount(); ++image_type) {
        captures_[image_type]->FOVAngle = fov_degrees;
    }
    camera_->SetFieldOfView(fov_degrees);
}

msr::airlib::CameraInfo APIPCamera::getCameraInfo() const
{
    msr::airlib::CameraInfo camera_info;

    camera_info.pose.position = ned_transform_->toLocalNed(this->GetActorLocation());
    camera_info.pose.orientation = ned_transform_->toNed(this->GetActorRotation().Quaternion());
    camera_info.fov = camera_->FieldOfView;
    camera_info.proj_mat = getProjectionMatrix();
    return camera_info;
}

std::vector<float> APIPCamera::getDistortionParams() const
{
    std::vector<float> param_values(5, 0.0);

    auto getParamValue = [this](const auto& name, float& val) {
        distortion_param_instance_->GetScalarParameterValue(FName(name), val);
    };

    getParamValue(TEXT("K1"), param_values[0]);
    getParamValue(TEXT("K2"), param_values[1]);
    getParamValue(TEXT("K3"), param_values[2]);
    getParamValue(TEXT("P1"), param_values[3]);
    getParamValue(TEXT("P2"), param_values[4]);

    return param_values;
}

void APIPCamera::setDistortionParam(const std::string& param_name, float value)
{
    distortion_param_instance_->SetScalarParameterValue(FName(param_name.c_str()), value);
}

bool APIPCamera::hasBlendable(USceneCaptureComponent2D* capture, UObject* blendable) const
{
    if (capture == nullptr || blendable == nullptr) {
        return false;
    }

    for (const FWeightedBlendable& weighted_blendable : capture->PostProcessSettings.WeightedBlendables.Array) {
        if (weighted_blendable.Object == blendable) {
            return true;
        }
    }
    return false;
}

void APIPCamera::configureSourceStencilAnnotationCapture(USceneCaptureComponent2D* annotation_capture,
                                                         FObjectAnnotator::AnnotatorType type)
{
    if (annotation_capture == nullptr) {
        return;
    }

    UMaterial* stencil_material = nullptr;
    if (type == FObjectAnnotator::AnnotatorType::Infrared) {
        stencil_material = infrared_stencil_material_static_;
    }
    else {
        stencil_material = segmentation_stencil_material_static_;
    }

    FObjectAnnotator::SetViewForSourceStencilAnnotationRender(annotation_capture->ShowFlags);
    annotation_capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
    annotation_capture->ShowOnlyComponents.Empty();

    if (!IsValid(stencil_material)) {
        UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Source stencil capture %s is missing its post-process material."), *annotation_capture->GetName());
        return;
    }

    UMaterialInstanceDynamic* stencil_mid = source_stencil_annotation_material_map_.FindRef(annotation_capture);
    if (!IsValid(stencil_mid)) {
        stencil_mid = UMaterialInstanceDynamic::Create(stencil_material, annotation_capture);
        if (!IsValid(stencil_mid)) {
            UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Failed to create source stencil material instance for %s."), *annotation_capture->GetName());
            return;
        }
        source_stencil_annotation_material_map_.Add(annotation_capture, stencil_mid);
        source_stencil_annotation_materials_.Add(stencil_mid);
    }

    if (!hasBlendable(annotation_capture, stencil_mid)) {
        annotation_capture->PostProcessSettings.AddBlendable(stencil_mid, 1.0f);
    }
}

void APIPCamera::updateAnnotationCapture(USceneCaptureComponent2D* annotation_capture,
                                          const TArray<TWeakObjectPtr<UPrimitiveComponent> >& component_list,
                                          bool only_hide,
                                         UPrimitiveComponent* extra_component)
{
    if (!only_hide && annotation_capture != nullptr) {
        annotation_capture->ShowOnlyComponents = component_list;
        if (extra_component != nullptr) {
            annotation_capture->ShowOnlyComponents.AddUnique(TWeakObjectPtr<UPrimitiveComponent>(extra_component));
        }
    }

    APlayerController* controller = this->GetWorld() ? this->GetWorld()->GetFirstPlayerController() : nullptr;
    for (const TWeakObjectPtr<UPrimitiveComponent>& component : component_list) {
        if (captures_[Utils::toNumeric(ImageType::Scene)] != nullptr) {
            captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(component);
        }
        if (captures_[Utils::toNumeric(ImageType::Lighting)] != nullptr) {
            captures_[Utils::toNumeric(ImageType::Lighting)]->HiddenComponents.AddUnique(component);
        }
        if (controller != nullptr) {
            controller->HiddenPrimitiveComponents.AddUnique(component);
        }
    }

    if (extra_component != nullptr) {
        if (captures_[Utils::toNumeric(ImageType::Scene)] != nullptr) {
            captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(TWeakObjectPtr<UPrimitiveComponent>(extra_component));
        }
        if (captures_[Utils::toNumeric(ImageType::Lighting)] != nullptr) {
            captures_[Utils::toNumeric(ImageType::Lighting)]->HiddenComponents.AddUnique(TWeakObjectPtr<UPrimitiveComponent>(extra_component));
        }
        if (controller != nullptr) {
            controller->HiddenPrimitiveComponents.AddUnique(TWeakObjectPtr<UPrimitiveComponent>(extra_component));
        }
    }

    for (int render_index = 0; render_index < equirectangular_captures_.Num(); ++render_index) {
        syncEquirectangularCaptureFrom2D(render_index);
    }
}

void APIPCamera::updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, bool only_hide) {
    if (only_hide) {
        updateAnnotationCapture(nullptr, ComponentList, true);
        return;
    }

    configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Segmentation)], FObjectAnnotator::AnnotatorType::InstanceSegmentation);
    syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Segmentation));
}

void APIPCamera::updateInstanceSegmentationAndInfraredAnnotation(const TArray<TWeakObjectPtr<UPrimitiveComponent> >& SegmentationComponentList,
                                                                 const TArray<TWeakObjectPtr<UPrimitiveComponent> >& InfraredComponentList,
                                                                 bool only_hide)
{
    if (!only_hide) {
        configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Segmentation)], FObjectAnnotator::AnnotatorType::InstanceSegmentation);
        configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Infrared)], FObjectAnnotator::AnnotatorType::Infrared);
        syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Segmentation));
        syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Infrared));
    }
}

void APIPCamera::updateInfraredAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, bool only_hide) {
    if (!only_hide) {
        configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Infrared)], FObjectAnnotator::AnnotatorType::Infrared);
        syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Infrared));
    }
}

void APIPCamera::updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, FString annotation_name, bool only_hide) {
    const int* render_index = annotator_name_to_index_map_.Find(annotation_name);
    if (render_index == nullptr || !captures_.IsValidIndex(*render_index)) {
        return;
    }

    if (const FObjectAnnotator::AnnotatorType* type = annotator_name_to_type_map_.Find(annotation_name)) {
        const bool use_source_stencil_backend = annotator_name_to_source_stencil_map_.FindRef(annotation_name);
        if (UsesSourceStencilAnnotationCamera(*type, use_source_stencil_backend)) {
            if (!only_hide) {
                configureSourceStencilAnnotationCapture(captures_[*render_index], *type);
                syncEquirectangularCaptureFrom2D(*render_index);
            }
            return;
        }
    }

    updateAnnotationCapture(
        captures_[*render_index],
        ComponentList,
        only_hide,
        sphere_annotation_component_map_.Contains(annotation_name) ? sphere_annotation_component_map_[annotation_name].Get() : nullptr);
}

void APIPCamera::addAnnotationCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance, bool use_source_stencil_backend)
{
    USceneCaptureComponent2D* new_capture = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D ::StaticClass(), *name);

    new_capture->bAutoActivate = false;
    new_capture->bCaptureEveryFrame = true;
    new_capture->bCaptureOnMovement = true;
    new_capture->SetRelativeRotation(FRotator(0, 0, 0));
    new_capture->SetRelativeLocation(FVector(0, 0, 0));
    new_capture->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	new_capture->RegisterComponent();
    new_capture->Deactivate();

    if (max_view_distance > 0 && !UsesSourceStencilAnnotationCamera(type, use_source_stencil_backend)) {
        FString sphereName  = name + "_hidden_sphere";
        UStaticMeshComponent* annotation_sphere = NewObject<UStaticMeshComponent>(this, FName(*sphereName));
        annotation_sphere->SetupAttachment(RootComponent);
        annotation_sphere->RegisterComponent();
        annotation_sphere->SetStaticMesh(annotation_sphere_static_);
        annotation_sphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        annotation_sphere->SetRelativeScale3D(FVector(max_view_distance * 2, max_view_distance * 2, max_view_distance * 2));

        FString annotatedSphereName = name + "_annotation_sphere";
        UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(annotation_sphere, FName(*annotatedSphereName));
        AnnotationComponent->SetupAttachment(annotation_sphere);
        AnnotationComponent->RegisterComponent();
        AnnotationComponent->MarkRenderStateDirty();
        UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(AnnotationComponent);

        sphere_annotation_component_map_.Add(name, PrimitiveComponent);
	}

    captures_.Add(new_capture);

    render_targets_.Add(NewObject<UTextureRenderTarget2D>());
    equirectangular_captures_.Add(nullptr);
    equirectangular_render_targets_.Add(nullptr);
    equirectangular_preview_targets_.Add(nullptr);
    equirectangular_preview_updates_.Add(false);
    int render_index = render_targets_.Num() - 1;
    if (type == FObjectAnnotator::AnnotatorType::RGB ||
        type == FObjectAnnotator::AnnotatorType::InstanceSegmentation ||
        type == FObjectAnnotator::AnnotatorType::Infrared) {
        render_targets_[render_index]->TargetGamma = 1;
    }

    camera_type_enabled_.push_back(false);   

    annotator_name_to_index_map_.Add(TCHAR_TO_UTF8(*name), render_index);
    annotator_name_to_type_map_.Add(TCHAR_TO_UTF8(*name), type);
    annotator_name_to_source_stencil_map_.Add(TCHAR_TO_UTF8(*name), use_source_stencil_backend);

    detections_.Add(NewObject<UDetectionComponent>(this));
    if (detections_[render_index]) {
        detections_[render_index]->SetupAttachment(captures_[render_index]);
        detections_[render_index]->RegisterComponent();
        detections_[render_index]->Deactivate();
    }

    captures_[render_index]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    setCaptureUpdate(captures_[render_index], true);    

    if (sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)).ignore_marked)captures_[render_index]->HiddenActors.Append(ignore_actors_);
    
    updateCaptureComponentSetting(captures_[render_index], render_targets_[render_index],
        false, EPixelFormat::PF_B8G8R8A8,
        sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)),
        *ned_transform_, false);

    copyCameraSettingsToSceneCapture(camera_, captures_[render_index]);

    const CaptureSetting& annotation_capture_setting = sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation));
    if (annotation_capture_setting.isEquirectangular()) {
        ensureEquirectangularCapture(render_index, name);
        updateEquirectangularCaptureComponentSetting(
            equirectangular_captures_[render_index],
            equirectangular_render_targets_[render_index],
            false,
            EPixelFormat::PF_B8G8R8A8,
            annotation_capture_setting,
            false);

        if (type == FObjectAnnotator::AnnotatorType::RGB ||
            type == FObjectAnnotator::AnnotatorType::InstanceSegmentation ||
            type == FObjectAnnotator::AnnotatorType::Infrared) {
            equirectangular_render_targets_[render_index]->TargetGamma = 1;
        }
    }

    if (UsesSourceStencilAnnotationCamera(type, use_source_stencil_backend)) {
        configureSourceStencilAnnotationCapture(captures_[render_index], type);
    }
    else {
        FObjectAnnotator::SetViewForAnnotationRender(captures_[render_index]->ShowFlags);
        captures_[render_index]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
    }

    syncEquirectangularCaptureFrom2D(render_index);
}

void APIPCamera::setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform)
{
    //TODO: should we be ignoring position and orientation settings here?

    //TODO: can we eliminate storing NedTransform?

    ned_transform_ = &ned_transform;

    sensor_params_ = camera_setting;

    gimbal_stabilization_ = Utils::clip(camera_setting.gimbal.stabilization, 0.0f, 1.0f);
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = camera_setting.gimbal.rotation.pitch;
        gimbald_rotator_.Roll = camera_setting.gimbal.rotation.roll;
        gimbald_rotator_.Yaw = camera_setting.gimbal.rotation.yaw;
    }
    updateActorTickEnabled();

    if (sensor_params_.external) {
        this->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    }

	static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
    for (TActorIterator<AActor> ActorIterator(this->GetWorld()); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))ignore_actors_.Add(Actor);
    }


    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count - 1; ++image_type) {
        const auto& capture_setting = camera_setting.capture_settings.at(image_type);
        const auto& noise_setting = camera_setting.noise_settings.at(image_type);

        if (image_type >= 0) { //scene capture components
            auto pixel_format_override = camera_setting.ue_setting.pixel_format_override_settings.find(image_type);
            EPixelFormat pixel_format = EPixelFormat::PF_Unknown;
            if (pixel_format_override != camera_setting.ue_setting.pixel_format_override_settings.end()) {
                pixel_format = static_cast<EPixelFormat>(pixel_format_override->second.pixel_format);
            }
            pixel_format = (pixel_format == EPixelFormat::PF_Unknown ? image_type_to_pixel_format_map_[image_type] : pixel_format);

            bool auto_format = true;
            bool force_linear_gamma = false;
            switch (Utils::toEnum<ImageType>(image_type)) {
            case ImageType::Scene:
            case ImageType::Infrared:
            case ImageType::Lighting:
            case ImageType::Segmentation:
                auto_format = false;
                break;
            case ImageType::SurfaceNormals:
                auto_format = true;
                force_linear_gamma = true;
                break;
            case ImageType::Annotation:
                break;
            default:
                auto_format = true;
                break;
            }

            updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], auto_format, pixel_format, capture_setting, ned_transform, force_linear_gamma);
            if (capture_setting.isEquirectangular()) {
                ensureEquirectangularCapture(image_type, captures_[image_type] ? captures_[image_type]->GetName() : FString::Printf(TEXT("ImageType%d"), image_type));
                updateEquirectangularCaptureComponentSetting(equirectangular_captures_[image_type], equirectangular_render_targets_[image_type], auto_format, pixel_format, capture_setting, force_linear_gamma);
            }

            if (image_type == Utils::toNumeric(ImageType::Infrared) || image_type == Utils::toNumeric(ImageType::Segmentation)) {
                render_targets_[image_type]->TargetGamma = 1;
                if (equirectangular_render_targets_.IsValidIndex(image_type) && equirectangular_render_targets_[image_type] != nullptr) {
                    equirectangular_render_targets_[image_type]->TargetGamma = 1;
                }
            }
            if(capture_setting.ignore_marked)captures_[image_type]->HiddenActors.Append(ignore_actors_);
            setDistortionMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings);
            setNoiseMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings, noise_setting);
            copyCameraSettingsToSceneCapture(camera_, captures_[image_type]); //CinemAirSim
            if(image_type == Utils::toNumeric(ImageType::Scene) || image_type == Utils::toNumeric(ImageType::Lighting)) {
                if (capture_setting.lumen_gi_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::Lumen;                    
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;
                }
                if (capture_setting.lumen_reflections_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
                    captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::Lumen;
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
					captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::None;
                }
                captures_[image_type]->PostProcessSettings.LumenSurfaceCacheResolution = 1;
                captures_[image_type]->PostProcessSettings.LumenFinalGatherQuality = capture_setting.lumen_final_quality;
                captures_[image_type]->PostProcessSettings.LumenSceneDetail = capture_setting.lumen_scene_detail;
                captures_[image_type]->PostProcessSettings.LumenSceneLightingQuality = capture_setting.lumen_scene_lightning_quality;
                captures_[image_type]->bUseRayTracingIfEnabled = 1;
			}
            if (capture_setting.isEquirectangular()) {
                syncEquirectangularCaptureFrom2D(image_type);
            }
        }
        else { //camera component
            updateCameraSetting(camera_, capture_setting, ned_transform);
            setDistortionMaterial(image_type, camera_, camera_->PostProcessSettings);
            setNoiseMaterial(image_type, camera_, camera_->PostProcessSettings, noise_setting);
            copyCameraSettingsToAllSceneCapture(camera_); //CinemAirSim
        }
    }
    configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Segmentation)], FObjectAnnotator::AnnotatorType::InstanceSegmentation);
    configureSourceStencilAnnotationCapture(captures_[Utils::toNumeric(ImageType::Infrared)], FObjectAnnotator::AnnotatorType::Infrared);
    syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Segmentation));
    syncEquirectangularCaptureFrom2D(Utils::toNumeric(ImageType::Infrared));
    if (camera_setting.capture_settings.at(Utils::toNumeric(ImageType::Scene)).force_update){
        setCameraTypeEnabled(ImageType::Scene, true);
        setCameraTypeUpdate(ImageType::Scene, false);
    }
}

void APIPCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target,
                                               bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting, const NedTransform& ned_transform,
                                               bool force_linear_gamma)
{
    if (auto_format) {
        render_target->InitAutoFormat(setting.width, setting.height); //256 X 144, X 480
    }
    else {
        render_target->InitCustomFormat(setting.width, setting.height, pixel_format, force_linear_gamma);
    }

    if (!std::isnan(setting.target_gamma))
		render_target->TargetGamma = setting.target_gamma;

    capture->ProjectionType = static_cast<ECameraProjectionMode::Type>(setting.projection_mode);

    if (!std::isnan(setting.fov_degrees))
        capture->FOVAngle = setting.fov_degrees;
    if (capture->ProjectionType == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        capture->OrthoWidth = ned_transform.fromNed(setting.ortho_width);

    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

void APIPCamera::updateEquirectangularCaptureComponentSetting(USceneCaptureComponentCube* capture, UTextureRenderTargetCube* render_target,
                                                       bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting,
                                                       bool force_linear_gamma)
{
    if (capture == nullptr || render_target == nullptr) {
        return;
    }

    const uint32 cube_size = FMath::Max<uint32>(1u, setting.height);
    render_target->bForceLinearGamma = force_linear_gamma;

    if (auto_format) {
        render_target->InitAutoFormat(cube_size);
    }
    else {
        const EPixelFormat equirectangular_format = UsesGlobalEquirectangularScenePipeline(setting.image_type)
            ? EPixelFormat::PF_FloatRGBA
            : pixel_format;
        render_target->Init(cube_size, equirectangular_format);
    }

    if (!std::isnan(setting.target_gamma))
        render_target->TargetGamma = setting.target_gamma;

    capture->bCaptureRotation = true;
    capture->TextureTarget = render_target;
    capture->PostProcessBlendWeight = 1.0f;
    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

void APIPCamera::syncEquirectangularCaptureFrom2D(int render_index)
{
    if (!captures_.IsValidIndex(render_index) || !equirectangular_captures_.IsValidIndex(render_index)) {
        return;
    }

    copySceneCaptureSettingsToCubeCapture(captures_[render_index], equirectangular_captures_[render_index]);

    // Scene and Lighting use a seam-safe HDR path. Other image types keep the
    // source capture settings so labels, depth units, and annotation views match
    // the normal 2D capture as closely as possible.
    if (UsesGlobalEquirectangularScenePipeline(render_index)) {
        ApplyGlobalEquirectangularSceneSettings(equirectangular_captures_[render_index]);
    }
}

//CinemAirSim
void APIPCamera::updateCameraSetting(UCineCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform)
{
    //if (!std::isnan(setting.target_gamma))
    //    camera-> = setting.target_gamma;

    camera->SetProjectionMode(static_cast<ECameraProjectionMode::Type>(setting.projection_mode));

    if (!std::isnan(setting.fov_degrees))
        camera->SetFieldOfView(setting.fov_degrees);
    if (camera->ProjectionMode == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        camera->SetOrthoWidth(ned_transform.fromNed(setting.ortho_width));

    updateCameraPostProcessingSetting(camera->PostProcessSettings, setting);
}

msr::airlib::Pose APIPCamera::getPose() const
{
    return ned_transform_->toLocalNed(this->GetActorTransform());
}

void APIPCamera::updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting)
{
    if (!std::isnan(setting.motion_blur_amount)) {
        obj.bOverride_MotionBlurAmount = 1;
        obj.MotionBlurAmount = setting.motion_blur_amount;
    }
    if (!std::isnan(setting.motion_blur_max))
	{
		obj.bOverride_MotionBlurMax = 1;
		obj.MotionBlurMax = setting.motion_blur_max;
	} 
    if (!std::isnan(setting.motion_blur_target_fps))
    {
        obj.bOverride_MotionBlurTargetFPS = 1;
        obj.MotionBlurTargetFPS = setting.motion_blur_target_fps;
    }

    if (setting.auto_exposure_method >= 0) {
        obj.bOverride_AutoExposureMethod = 1;
        obj.AutoExposureMethod = Utils::toEnum<EAutoExposureMethod>(setting.auto_exposure_method);
    }
    if (!std::isnan(setting.auto_exposure_bias)) {
        obj.bOverride_AutoExposureBias = 1;
        obj.AutoExposureBias = setting.auto_exposure_bias;
    }
    
    obj.bOverride_AutoExposureApplyPhysicalCameraExposure = 1;
    obj.AutoExposureApplyPhysicalCameraExposure = setting.auto_exposure_apply_physical_camera_exposure ? 1 : 0;
    
    if (!std::isnan(setting.auto_exposure_min_brightness)) {
        obj.bOverride_AutoExposureMinBrightness = 1;
        obj.AutoExposureMinBrightness = setting.auto_exposure_min_brightness;
    }
    if (!std::isnan(setting.auto_exposure_max_brightness)) {
        obj.bOverride_AutoExposureMaxBrightness = 1;
        obj.AutoExposureMaxBrightness = setting.auto_exposure_max_brightness;
    }
    if (!std::isnan(setting.auto_exposure_speed_up)) {
        obj.bOverride_AutoExposureSpeedUp = 1;
        obj.AutoExposureSpeedUp = setting.auto_exposure_speed_up;
    }
    if (!std::isnan(setting.auto_exposure_speed_down)) {
        obj.bOverride_AutoExposureSpeedDown = 1;
        obj.AutoExposureSpeedDown = setting.auto_exposure_speed_down;
    }
    if (!std::isnan(setting.auto_exposure_low_percent)) {
        obj.bOverride_AutoExposureLowPercent = 1;
        obj.AutoExposureLowPercent = setting.auto_exposure_low_percent;
    }
    if (!std::isnan(setting.auto_exposure_high_percent)) {
        obj.bOverride_AutoExposureHighPercent = 1;
        obj.AutoExposureHighPercent = setting.auto_exposure_high_percent;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_min)) {
        obj.bOverride_HistogramLogMin = 1;
        obj.HistogramLogMin = setting.auto_exposure_histogram_log_min;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_max)) {
        obj.bOverride_HistogramLogMax = 1;
        obj.HistogramLogMax = setting.auto_exposure_histogram_log_max;
    }

    if (!std::isnan(setting.bloom_intensity)) {
        obj.bOverride_BloomIntensity = 1;
        obj.BloomIntensity = setting.bloom_intensity;
    }
    if (!std::isnan(setting.bloom_threshold)) {
        obj.bOverride_BloomThreshold = 1;
        obj.BloomThreshold = setting.bloom_threshold;
    }

    if (!std::isnan(setting.chromatic_aberration_intensity)) {
        obj.bOverride_SceneFringeIntensity = 1;
        obj.SceneFringeIntensity = setting.chromatic_aberration_intensity;
    }
    if (!std::isnan(setting.chromatic_aberration_start_offset)) {
        obj.bOverride_ChromaticAberrationStartOffset = 1;
        obj.ChromaticAberrationStartOffset = setting.chromatic_aberration_start_offset;
    }

    if (!std::isnan(setting.camera_shutter_speed)) {
        obj.bOverride_CameraShutterSpeed = 1;
        obj.CameraShutterSpeed = setting.camera_shutter_speed;
    }
    if (!std::isnan(setting.camera_iso)) {
        obj.bOverride_CameraISO = 1;
        obj.CameraISO = setting.camera_iso;
    }
    if (!std::isnan(setting.camera_aperture)) {
        obj.bOverride_DepthOfFieldFstop = 1;
        obj.DepthOfFieldFstop = setting.camera_aperture;
    }
    if (!std::isnan(setting.camera_max_aperture)) {
        obj.bOverride_DepthOfFieldMinFstop = 1;
        obj.DepthOfFieldMinFstop = setting.camera_max_aperture;
    }
    if (!std::isnan(setting.camera_num_blades)) {
        obj.bOverride_DepthOfFieldBladeCount = 1;
        obj.DepthOfFieldBladeCount = static_cast<int32>(setting.camera_num_blades);
    }

    if (!std::isnan(setting.lens_flare_intensity)) {
        obj.bOverride_LensFlareIntensity = 1;
        obj.LensFlareIntensity = setting.lens_flare_intensity;
    }
    if (!std::isnan(setting.lens_flare_bokeh_size)) {
        obj.bOverride_LensFlareBokehSize = 1;
        obj.LensFlareBokehSize = setting.lens_flare_bokeh_size;
    }
    if (!std::isnan(setting.lens_flare_threshold)) {
        obj.bOverride_LensFlareThreshold = 1;
        obj.LensFlareThreshold = setting.lens_flare_threshold;
    }

    if (!std::isnan(setting.depth_of_field_sensor_width)) {
        obj.bOverride_DepthOfFieldSensorWidth = 1;
        obj.DepthOfFieldSensorWidth = setting.depth_of_field_sensor_width;
    }
    if (!std::isnan(setting.depth_of_field_squeeze_factor)) {
        obj.bOverride_DepthOfFieldSqueezeFactor = 1;
        obj.DepthOfFieldSqueezeFactor = setting.depth_of_field_squeeze_factor;
    }
    if (!std::isnan(setting.depth_of_field_focal_distance)) {
        obj.bOverride_DepthOfFieldFocalDistance = 1;
        obj.DepthOfFieldFocalDistance = setting.depth_of_field_focal_distance;
    }
    if (!std::isnan(setting.depth_of_field_depth_blur_amount)) {
        obj.bOverride_DepthOfFieldDepthBlurAmount = 1;
        obj.DepthOfFieldDepthBlurAmount = setting.depth_of_field_depth_blur_amount;
    }
    if (!std::isnan(setting.depth_of_field_depth_blur_radius)) {
        obj.bOverride_DepthOfFieldDepthBlurRadius = 1;
        obj.DepthOfFieldDepthBlurRadius = setting.depth_of_field_depth_blur_radius;
    }
    if (!std::isnan(setting.depth_of_field_use_hair_depth)) {
        obj.bOverride_DepthOfFieldUseHairDepth = 1;
        obj.DepthOfFieldUseHairDepth = setting.depth_of_field_use_hair_depth ? 1 : 0;
    }

}

void APIPCamera::setDistortionMaterial(int image_type, UObject* outer, FPostProcessSettings& obj)
{
    UMaterialInstanceDynamic* distortion_material = UMaterialInstanceDynamic::Create(distortion_material_static_, outer);
    distortion_materials_[image_type + 1] = distortion_material;
    obj.AddBlendable(distortion_material, 1.0f);
}

void APIPCamera::setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings)
{
    if (!settings.Enabled)
        return;

    UMaterialInstanceDynamic* noise_material = UMaterialInstanceDynamic::Create(noise_material_static_, outer);
    noise_materials_[image_type + 1] = noise_material;

    noise_material->SetScalarParameterValue("HorzWaveStrength", settings.HorzWaveStrength);
    noise_material->SetScalarParameterValue("RandSpeed", settings.RandSpeed);
    noise_material->SetScalarParameterValue("RandSize", settings.RandSize);
    noise_material->SetScalarParameterValue("RandDensity", settings.RandDensity);
    noise_material->SetScalarParameterValue("RandContrib", settings.RandContrib);
    noise_material->SetScalarParameterValue("HorzWaveContrib", settings.HorzWaveContrib);
    noise_material->SetScalarParameterValue("HorzWaveVertSize", settings.HorzWaveVertSize);
    noise_material->SetScalarParameterValue("HorzWaveScreenSize", settings.HorzWaveScreenSize);
    noise_material->SetScalarParameterValue("HorzNoiseLinesContrib", settings.HorzNoiseLinesContrib);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityY", settings.HorzNoiseLinesDensityY);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityXY", settings.HorzNoiseLinesDensityXY);
    noise_material->SetScalarParameterValue("HorzDistortionStrength", settings.HorzDistortionStrength);
    noise_material->SetScalarParameterValue("HorzDistortionContrib", settings.HorzDistortionContrib);

    obj.AddBlendable(noise_material, 1.0f);

	if (settings.LensDistortionEnable) {

		UMaterialInstanceDynamic* lens_distortion_material_;

		if (settings.LensDistortionInvert) {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_invert_material_static_, outer);
		}
		else {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_material_static_, outer);
		}

		lens_distortion_materials_[image_type + 1] = lens_distortion_material_;


		lens_distortion_material_->SetScalarParameterValue("AreaFalloff", settings.LensDistortionAreaFalloff);
		lens_distortion_material_->SetScalarParameterValue("AreaRadius", settings.LensDistortionAreaRadius);
		lens_distortion_material_->SetScalarParameterValue("Intensity", settings.LensDistortionIntensity);


		obj.AddBlendable(lens_distortion_material_, 1.0f);
	}

    if (settings.FakeMotionBlurEnable) {
        UMaterialInstanceDynamic* motion_blur_material = UMaterialInstanceDynamic::Create(motion_blur_material_static_, outer);
        fake_motion_blur_materials_[image_type + 1] = motion_blur_material;
        
        motion_blur_material->SetScalarParameterValue("MotionBlurDirectionX", settings.FakeMotionBlurDirectionX);
        motion_blur_material->SetScalarParameterValue("MotionBlurDirectionY", settings.FakeMotionBlurDirectionY);
        motion_blur_material->SetScalarParameterValue("MotionBlurMovementSpeed", settings.FakeMotionBlurMovementSpeed);
        motion_blur_material->SetScalarParameterValue("MotionBlurShutterSpeed", settings.FakeMotionBlurShutterSpeed);
        motion_blur_material->SetScalarParameterValue("MotionBlurFocalLength", settings.FakeMotionBlurFocalLength);
        motion_blur_material->SetScalarParameterValue("MotionBlurMovementSpeed", settings.FakeMotionBlurSamples);

        obj.AddBlendable(motion_blur_material, 1.0f);
    }
    if (settings.RadialBlurEnable) {
        UMaterialInstanceDynamic* radial_blur_material = UMaterialInstanceDynamic::Create(radial_blur_material_static_, outer);
        radial_blur_materials_[image_type + 1] = radial_blur_material;

        radial_blur_material->SetScalarParameterValue("RadialBlurDistance", settings.RadialBlurDistance);
        radial_blur_material->SetScalarParameterValue("RadialBlurRadius", settings.RadialBlurRadius);
        radial_blur_material->SetScalarParameterValue("RadialBlurDensity", settings.RadialBlurDensity);

        obj.AddBlendable(radial_blur_material, 1.0f);
    }
    if (settings.GuassianBlurEnable) {
        UMaterialInstanceDynamic* guassian_blur_material = UMaterialInstanceDynamic::Create(guassian_blur_material_static_, outer);
        guassian_blur_materials_[image_type + 1] = guassian_blur_material;

        guassian_blur_material->SetScalarParameterValue("GuassianDirections", settings.GuassianBlurDirections);
        guassian_blur_material->SetScalarParameterValue("GuassianQuality", settings.GuassianBlurQuality);
        guassian_blur_material->SetScalarParameterValue("GuassianSize", settings.GuassianBlurSize);

        obj.AddBlendable(guassian_blur_material, 1.0f);
    }
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled, std::string annotation_name)
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 || render_index >= static_cast<int32>(camera_type_enabled_.size())) {
        return;
    }

    if (isEquirectangularCapture(type, annotation_name)) {
        USceneCaptureComponentCube* cube_capture = getEquirectangularCaptureComponent(type, false, annotation_name);
        UTextureRenderTargetCube* cube_target = getEquirectangularRenderTarget(type, false, annotation_name);

        if (cube_capture != nullptr) {
            if (is_enabled) {
                if (!cube_capture->IsActive() || cube_capture->TextureTarget == nullptr) {
                    cube_capture->TextureTarget = cube_target;
                    cube_capture->Activate();
                }
            }
            else {
                if (cube_capture->IsActive() || cube_capture->TextureTarget != nullptr) {
                    cube_capture->Deactivate();
                    cube_capture->TextureTarget = nullptr;
                }
                setEquirectangularPreviewUpdate(render_index, false);
            }

            camera_type_enabled_[render_index] = is_enabled;
        }
        return;
    }

    USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
    if (capture != nullptr) {
        UDetectionComponent* detection = getDetectionComponent(type, false, annotation_name);
        if (is_enabled) {
            //do not make unnecessary calls to Activate() which otherwise causes crash in Unreal
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = getRenderTarget(type, false, annotation_name);
                capture->Activate();
                if (detection != nullptr) {
                    detection->texture_target_ = capture->TextureTarget;
                    detection->Activate();
                }
            }
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
                if (detection != nullptr) {
                    detection->Deactivate();
                    detection->texture_target_ = nullptr;
                }
            }
        }
        camera_type_enabled_[render_index] = is_enabled;
    }
    //else nothing to enable
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 ||
        render_index >= static_cast<int32>(camera_type_enabled_.size()) ||
        !render_targets_.IsValidIndex(render_index)) {
        return nullptr;
    }

    return (!if_active || camera_type_enabled_[render_index]) ? render_targets_[render_index] : nullptr;
}

UTextureRenderTarget2D* APIPCamera::getPreviewRenderTarget(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    if (!isEquirectangularCapture(type, annotation_name)) {
        return getRenderTarget(type, if_active, annotation_name);
    }

    int render_index = Utils::toNumeric(type);
    int setting_index = render_index;
    if (type == ImageType::Annotation) {
        if (!annotator_name_to_index_map_.Contains(FString(annotation_name.c_str()))) {
            return nullptr;
        }
        render_index = annotator_name_to_index_map_[FString(annotation_name.c_str())];
        setting_index = Utils::toNumeric(ImageType::Annotation);
    }

    if (render_index < 0 || render_index >= static_cast<int>(camera_type_enabled_.size())) {
        return nullptr;
    }

    if (if_active && !camera_type_enabled_[render_index]) {
        return nullptr;
    }

    if (setting_index < 0 || setting_index >= static_cast<int>(sensor_params_.capture_settings.size())) {
        return nullptr;
    }

    ensureEquirectangularPreviewTarget(render_index, sensor_params_.capture_settings.at(setting_index), type);
    setEquirectangularPreviewUpdate(render_index, camera_type_enabled_[render_index]);

    return equirectangular_preview_targets_.IsValidIndex(render_index)
        ? equirectangular_preview_targets_[render_index]
        : nullptr;
}

bool APIPCamera::isEquirectangularCapture(const APIPCamera::ImageType type, std::string annotation_name) const
{
    if (type == ImageType::Annotation &&
        ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_) == INDEX_NONE) {
        return false;
    }

    const int32 setting_index = type == ImageType::Annotation
        ? static_cast<int32>(ImageType::Annotation)
        : static_cast<int32>(type);
    if (setting_index < 0 || setting_index >= static_cast<int32>(sensor_params_.capture_settings.size())) {
        return false;
    }

    return sensor_params_.capture_settings.at(setting_index).isEquirectangular();
}

USceneCaptureComponentCube* APIPCamera::getEquirectangularCaptureComponent(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 ||
        render_index >= static_cast<int32>(camera_type_enabled_.size()) ||
        !equirectangular_captures_.IsValidIndex(render_index)) {
        return nullptr;
    }

    return (!if_active || camera_type_enabled_[render_index]) ? equirectangular_captures_[render_index] : nullptr;
}

UTextureRenderTargetCube* APIPCamera::getEquirectangularRenderTarget(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 ||
        render_index >= static_cast<int32>(camera_type_enabled_.size()) ||
        !equirectangular_render_targets_.IsValidIndex(render_index)) {
        return nullptr;
    }

    return (!if_active || camera_type_enabled_[render_index]) ? equirectangular_render_targets_[render_index] : nullptr;
}

UDetectionComponent* APIPCamera::getDetectionComponent(const ImageType type, bool if_active, std::string annotation_name) const
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 ||
        render_index >= static_cast<int32>(camera_type_enabled_.size()) ||
        !detections_.IsValidIndex(render_index)) {
        return nullptr;
    }

    return (!if_active || camera_type_enabled_[render_index]) ? detections_[render_index] : nullptr;
}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    const int32 render_index = ResolveRenderIndex(type, annotation_name, annotator_name_to_index_map_);
    if (render_index < 0 ||
        render_index >= static_cast<int32>(camera_type_enabled_.size()) ||
        !captures_.IsValidIndex(render_index)) {
        return nullptr;
    }

    return (!if_active || camera_type_enabled_[render_index]) ? captures_[render_index] : nullptr;
}

void APIPCamera::disableAllPIP()
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
		if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
		{
			for (auto& annotator : annotator_name_to_index_map_)
			{
				enableCaptureComponent(ImageType::Annotation, false, TCHAR_TO_UTF8(*annotator.Key));
			}
		}
        else {
            enableCaptureComponent(Utils::toEnum<ImageType>(image_type), false);
        }
    }
}

void APIPCamera::disableMain()
{
    camera_->Deactivate();
    camera_->SetVisibility(false);
    //APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    //if (controller && controller->GetViewTarget() == this)
    //    controller->SetViewTarget(nullptr);
}

void APIPCamera::onViewModeChanged(bool nodisplay)
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
        {
            for (auto& annotator : annotator_name_to_index_map_)
            {
                const std::string annotation_name = TCHAR_TO_UTF8(*annotator.Key);
                if (isEquirectangularCapture(ImageType::Annotation, annotation_name)) {
                    setEquirectangularCaptureUpdate(getEquirectangularCaptureComponent(ImageType::Annotation, false, annotation_name), nodisplay);
                }
                else {
                    USceneCaptureComponent2D* capture = getCaptureComponent(ImageType::Annotation, false, annotation_name);
                    if (capture) {
                        setCaptureUpdate(capture, nodisplay);
                    }
                }
            }
        }
        else
        {
            if (Utils::toEnum<ImageType>(image_type) != ImageType::Scene)
            {
                ImageType current_type = static_cast<ImageType>(image_type);
                if (isEquirectangularCapture(current_type)) {
                    setEquirectangularCaptureUpdate(getEquirectangularCaptureComponent(current_type, false), nodisplay);
                }
                else {
                    USceneCaptureComponent2D* capture = getCaptureComponent(current_type, false);
                    if (capture) {
                        setCaptureUpdate(capture, nodisplay);
                    }
                }
            }           
        }                
    }
}

//CinemAirSim methods
std::vector<std::string> APIPCamera::getPresetLensSettings() const
{
    std::vector<std::string> vector;
    const TArray<FNamedLensPreset> lens_presets = UCineCameraSettings::GetLensPresets();
    for (const FNamedLensPreset& preset : lens_presets) {
        std::ostringstream current_lens_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        current_lens_string << "Name: " << name << ";\n\t MinFocalLength: " << preset.LensSettings.MinFocalLength << "; \t MaxFocalLength: " << preset.LensSettings.MaxFocalLength;
        current_lens_string << "\n\t Min FStop: " << preset.LensSettings.MinFStop << "; \t Max Fstop: " << preset.LensSettings.MaxFStop;
        vector.push_back(current_lens_string.str());
    }
    return vector;
}

std::string APIPCamera::getLensSettings() const
{
    const FCameraLensSettings current_lens_params = camera_->LensSettings;

    std::ostringstream current_lens_string;

    const FString lens_preset_name = camera_->GetLensPresetName();
    std::string name = (TCHAR_TO_UTF8(*lens_preset_name));

    current_lens_string << "Name: " << name;
    current_lens_string << ";\n\t MinFocalLength: " << current_lens_params.MinFocalLength;
    current_lens_string << "; \t MaxFocalLength: " << current_lens_params.MaxFocalLength;
    current_lens_string << "\n\t Min FStop: " << current_lens_params.MinFStop;
    current_lens_string << "; \t Max Fstop: " << current_lens_params.MaxFStop;
    current_lens_string << "\n\t Diaphragm Blade Count: " << current_lens_params.DiaphragmBladeCount;
    current_lens_string << "\n\t Minimum focus distance: " << current_lens_params.MinimumFocusDistance;

    return current_lens_string.str();
}

void APIPCamera::setPresetLensSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetLensPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::vector<std::string> APIPCamera::getPresetFilmbackSettings() const
{
    std::vector<std::string> vector_all_presets;
    TArray<FNamedFilmbackPreset> lens_presets = UCineCameraSettings::GetFilmbackPresets();
    for (const FNamedFilmbackPreset& preset : lens_presets) {
        std::ostringstream preset_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        preset_string << "Name: " << name << ";\n\t Sensor Width: " << preset.FilmbackSettings.SensorWidth << "; \t Sensor Height: " << preset.FilmbackSettings.SensorHeight;
        preset_string << "\n\t Sensor Aspect Ratio: " << preset.FilmbackSettings.SensorAspectRatio;
        vector_all_presets.push_back(preset_string.str());
    }
    return vector_all_presets;
}

void APIPCamera::setPresetFilmbackSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetFilmbackPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::string APIPCamera::getFilmbackSettings() const
{
    FCameraFilmbackSettings current_filmback_settings = camera_->Filmback;

    const FString filmback_present_name = camera_->GetFilmbackPresetName();
    std::ostringstream current_filmback_string;
    std::string name = (TCHAR_TO_UTF8(*filmback_present_name));

    current_filmback_string << "Name: " << name << ";\n\t Sensor Width: " << current_filmback_settings.SensorWidth << "; \t Sensor Height: " << current_filmback_settings.SensorHeight;
    current_filmback_string << "\n\t Sensor Aspect Ratio: " << current_filmback_settings.SensorAspectRatio;
    return current_filmback_string.str();
}

float APIPCamera::setFilmbackSettings(float sensor_width, float sensor_height)
{
    camera_->Filmback.SensorWidth = sensor_width;
    camera_->Filmback.SensorHeight = sensor_height;

    copyCameraSettingsToAllSceneCapture(camera_);

    return camera_->Filmback.SensorAspectRatio;
}

float APIPCamera::getFocalLength() const
{
    return camera_->CurrentFocalLength;
}

void APIPCamera::setFocalLength(float focal_length)
{
    camera_->CurrentFocalLength = focal_length;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableManualFocus(bool enable)
{
    if (enable) {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Manual;
    }
    else {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Disable;
    }
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusDistance() const
{
    return camera_->FocusSettings.ManualFocusDistance;
}

void APIPCamera::setFocusDistance(float focus_distance)
{
    camera_->FocusSettings.ManualFocusDistance = focus_distance;
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusAperture() const
{
    return camera_->CurrentAperture;
}

void APIPCamera::setFocusAperture(float focus_aperture)
{
    camera_->CurrentAperture = focus_aperture;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableFocusPlane(bool enable)
{
#if WITH_EDITOR
    camera_->FocusSettings.bDrawDebugFocusPlane = enable;
#endif
}

std::string APIPCamera::getCurrentFieldOfView() const
{
    std::ostringstream field_of_view_string;
    field_of_view_string << "Current Field Of View:\n\tHorizontal Field Of View: " << camera_->GetHorizontalFieldOfView() << ";\n\t Vertical Field Of View: " << camera_->GetVerticalFieldOfView();
    return field_of_view_string.str();
}

void APIPCamera::copyCameraSettingsToAllSceneCapture(UCameraComponent* camera)
{
    int image_count = static_cast<int>(cameraCaptureCount());
    for (int image_type = image_count - 1; image_type >= 0; image_type--) {
        copyCameraSettingsToSceneCapture(camera_, captures_[image_type]);
        syncEquirectangularCaptureFrom2D(image_type);
    }
}

void APIPCamera::copyCameraSettingsToSceneCapture(UCameraComponent* src, USceneCaptureComponent2D* dst)
{
    if (src && dst) {
        dst->SetWorldLocationAndRotation(src->GetComponentLocation(), src->GetComponentRotation());

        FMinimalViewInfo camera_view_info;
        src->GetCameraView(/*DeltaTime =*/0.0f, camera_view_info);

        const FPostProcessSettings& src_pp_settings = camera_view_info.PostProcessSettings;
        FPostProcessSettings& dst_pp_settings = dst->PostProcessSettings;

        FWeightedBlendables dst_weighted_blendables = dst_pp_settings.WeightedBlendables;

        // Copy all of the post processing settings
        dst_pp_settings = src_pp_settings;

        // But restore the original blendables
        dst_pp_settings.WeightedBlendables = dst_weighted_blendables;
    }
}

void APIPCamera::copySceneCaptureSettingsToCubeCapture(USceneCaptureComponent2D* src, USceneCaptureComponentCube* dst)
{
    if (src == nullptr || dst == nullptr) {
        return;
    }

    UTextureRenderTargetCube* texture_target = dst->TextureTarget;
    const bool was_active = dst->IsActive();

    dst->SetWorldLocationAndRotation(src->GetComponentLocation(), src->GetComponentRotation());
    dst->CaptureSource = src->CaptureSource;
    dst->PrimitiveRenderMode = src->PrimitiveRenderMode;
    dst->HiddenComponents = src->HiddenComponents;
    dst->HiddenActors = src->HiddenActors;
    dst->ShowOnlyComponents = src->ShowOnlyComponents;
    dst->ShowOnlyActors = src->ShowOnlyActors;
    dst->LODDistanceFactor = src->LODDistanceFactor;
    dst->MaxViewDistanceOverride = src->MaxViewDistanceOverride;
    dst->CaptureSortPriority = src->CaptureSortPriority;
    dst->bUseRayTracingIfEnabled = src->bUseRayTracingIfEnabled;
    dst->ShowFlagSettings = src->ShowFlagSettings;
    dst->ShowFlags = src->ShowFlags;
    dst->PostProcessSettings = src->PostProcessSettings;
    dst->PostProcessBlendWeight = src->PostProcessBlendWeight;
    dst->bCaptureRotation = true;
    dst->TextureTarget = texture_target;

    if (!was_active) {
        dst->Deactivate();
    }
}

//end CinemAirSim methods
