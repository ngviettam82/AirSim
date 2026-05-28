#include "AirSimEquirectangularPreview.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "GlobalShader.h"
#include "PipelineStateCache.h"
#include "RenderResource.h"
#include "RenderingThread.h"
#include "RHIStaticStates.h"
#include "ShaderParameterUtils.h"
#include "TextureResource.h"

namespace
{
    class FAirSimEquirectangularPreviewVS : public FGlobalShader
    {
        DECLARE_SHADER_TYPE(FAirSimEquirectangularPreviewVS, Global);

    public:
        FAirSimEquirectangularPreviewVS()
        {
        }

        FAirSimEquirectangularPreviewVS(const ShaderMetaType::CompiledShaderInitializerType& initializer)
            : FGlobalShader(initializer)
        {
        }

        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& parameters)
        {
            return IsFeatureLevelSupported(parameters.Platform, ERHIFeatureLevel::SM5);
        }
    };

    class FAirSimEquirectangularPreviewPS : public FGlobalShader
    {
        DECLARE_SHADER_TYPE(FAirSimEquirectangularPreviewPS, Global);

    public:
        FAirSimEquirectangularPreviewPS()
        {
        }

        FAirSimEquirectangularPreviewPS(const ShaderMetaType::CompiledShaderInitializerType& initializer)
            : FGlobalShader(initializer)
        {
            source_cube_.Bind(initializer.ParameterMap, TEXT("SourceCube"));
            source_sampler_.Bind(initializer.ParameterMap, TEXT("SourceSampler"));
            preview_mode_.Bind(initializer.ParameterMap, TEXT("PreviewMode"));
            max_depth_meters_.Bind(initializer.ParameterMap, TEXT("MaxDepthMeters"));
            exposure_multiplier_.Bind(initializer.ParameterMap, TEXT("ExposureMultiplier"));
        }

        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& parameters)
        {
            return IsFeatureLevelSupported(parameters.Platform, ERHIFeatureLevel::SM5);
        }

        void SetParameters(
            FRHIBatchedShaderParameters& batched_parameters,
            FTextureRHIRef source_cube,
            AirSimEquirectangularPreview::EPreviewMode mode,
            float max_depth_meters,
            float exposure_compensation)
        {
            const bool use_linear_sampling =
                mode == AirSimEquirectangularPreview::EPreviewMode::ColorBilinear ||
                mode == AirSimEquirectangularPreview::EPreviewMode::SceneColor;
            FSamplerStateRHIRef sampler = use_linear_sampling
                ? TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI()
                : TStaticSamplerState<SF_Point, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();

            const float depth_preview_range = FMath::IsFinite(max_depth_meters) && max_depth_meters > 0.0f
                ? max_depth_meters
                : 100.0f;
            const float exposure_bias = FMath::IsFinite(exposure_compensation)
                ? exposure_compensation
                : 0.0f;

            SetTextureParameter(batched_parameters, source_cube_, source_sampler_, sampler, source_cube);
            SetShaderValue(batched_parameters, preview_mode_, static_cast<uint32>(mode));
            SetShaderValue(batched_parameters, max_depth_meters_, depth_preview_range);
            SetShaderValue(batched_parameters, exposure_multiplier_, FMath::Pow(2.0f, exposure_bias));
        }

    private:
        LAYOUT_FIELD(FShaderResourceParameter, source_cube_);
        LAYOUT_FIELD(FShaderResourceParameter, source_sampler_);
        LAYOUT_FIELD(FShaderParameter, preview_mode_);
        LAYOUT_FIELD(FShaderParameter, max_depth_meters_);
        LAYOUT_FIELD(FShaderParameter, exposure_multiplier_);
    };

    IMPLEMENT_SHADER_TYPE(, FAirSimEquirectangularPreviewVS, TEXT("/Plugin/AirSimShaders/Private/EquirectangularPreview.usf"), TEXT("MainVS"), SF_Vertex)
    IMPLEMENT_SHADER_TYPE(, FAirSimEquirectangularPreviewPS, TEXT("/Plugin/AirSimShaders/Private/EquirectangularPreview.usf"), TEXT("MainPS"), SF_Pixel)

    void Draw_RenderThread(
        FRHICommandListImmediate& rhi_cmd_list,
        FTextureRenderTargetCubeResource* source_resource,
        FTextureRenderTargetResource* output_resource,
        AirSimEquirectangularPreview::EPreviewMode mode,
        float max_depth_meters,
        float exposure_compensation)
    {
        check(IsInRenderingThread());
        if (source_resource == nullptr || output_resource == nullptr ||
            source_resource->TextureRHI == nullptr || output_resource->GetRenderTargetTexture() == nullptr) {
            return;
        }

        FTextureRHIRef source_texture = source_resource->TextureRHI;
        FRHITexture* output_texture = output_resource->GetRenderTargetTexture();
        const FIntPoint output_size = output_resource->GetSizeXY();
        if (output_size.X <= 0 || output_size.Y <= 0) {
            return;
        }

        rhi_cmd_list.Transition(FRHITransitionInfo(source_texture, ERHIAccess::Unknown, ERHIAccess::SRVMask));
        rhi_cmd_list.Transition(FRHITransitionInfo(output_texture, ERHIAccess::Unknown, ERHIAccess::RTV));

        FRHIRenderPassInfo render_pass_info(output_texture, ERenderTargetActions::Clear_Store);
        rhi_cmd_list.BeginRenderPass(render_pass_info, TEXT("AirSimEquirectangularPreview"));
        {
            rhi_cmd_list.SetViewport(0, 0, 0.0f, output_size.X, output_size.Y, 1.0f);

            FGlobalShaderMap* shader_map = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            TShaderMapRef<FAirSimEquirectangularPreviewVS> vertex_shader(shader_map);
            TShaderMapRef<FAirSimEquirectangularPreviewPS> pixel_shader(shader_map);

            FGraphicsPipelineStateInitializer graphics_pso_init;
            rhi_cmd_list.ApplyCachedRenderTargets(graphics_pso_init);
            graphics_pso_init.DepthStencilState = TStaticDepthStencilState<false, CF_Always>::GetRHI();
            graphics_pso_init.BlendState = TStaticBlendState<>::GetRHI();
            graphics_pso_init.RasterizerState = TStaticRasterizerState<>::GetRHI();
            graphics_pso_init.PrimitiveType = PT_TriangleList;
            graphics_pso_init.BoundShaderState.VertexDeclarationRHI = GetVertexDeclarationFVector4();
            graphics_pso_init.BoundShaderState.VertexShaderRHI = vertex_shader.GetVertexShader();
            graphics_pso_init.BoundShaderState.PixelShaderRHI = pixel_shader.GetPixelShader();
            SetGraphicsPipelineState(rhi_cmd_list, graphics_pso_init, 0);

            SetShaderParametersLegacyPS(
                rhi_cmd_list,
                pixel_shader,
                source_texture,
                mode,
                max_depth_meters,
                exposure_compensation);

            rhi_cmd_list.DrawPrimitive(0, 2, 1);
        }
        rhi_cmd_list.EndRenderPass();
    }
}

void AirSimEquirectangularPreview::Draw(
    UTextureRenderTargetCube* source_cube,
    UTextureRenderTarget2D* output_target,
    EPreviewMode mode,
    float max_depth_meters,
    float exposure_compensation)
{
    if (source_cube == nullptr || output_target == nullptr) {
        return;
    }

    FTextureRenderTargetCubeResource* source_resource =
        static_cast<FTextureRenderTargetCubeResource*>(source_cube->GameThread_GetRenderTargetResource());
    FTextureRenderTargetResource* output_resource = output_target->GameThread_GetRenderTargetResource();
    if (source_resource == nullptr || output_resource == nullptr) {
        return;
    }

    ENQUEUE_RENDER_COMMAND(AirSimEquirectangularPreviewDraw)(
        [source_resource, output_resource, mode, max_depth_meters, exposure_compensation](FRHICommandListImmediate& rhi_cmd_list) {
            Draw_RenderThread(
                rhi_cmd_list,
                source_resource,
                output_resource,
                mode,
                max_depth_meters,
                exposure_compensation);
        });
}
