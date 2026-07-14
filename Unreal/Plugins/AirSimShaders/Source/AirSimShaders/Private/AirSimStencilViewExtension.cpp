#include "AirSimStencilViewExtension.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "GlobalShader.h"
#include "PixelShaderUtils.h"
#include "PostProcess/PostProcessMaterialInputs.h"
#include "RenderGraphBuilder.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "ScreenPass.h"

namespace
{
    class FAirSimStencilOutputPS : public FGlobalShader
    {
        DECLARE_GLOBAL_SHADER(FAirSimStencilOutputPS);
        SHADER_USE_PARAMETER_STRUCT(FAirSimStencilOutputPS, FGlobalShader);

        BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
            SHADER_PARAMETER_STRUCT_INCLUDE(FSceneTextureShaderParameters, SceneTextures)
            SHADER_PARAMETER(FIntPoint, SceneTextureMin)
            SHADER_PARAMETER(FIntPoint, OutputTextureMin)
            SHADER_PARAMETER(uint32, OutputMode)
            RENDER_TARGET_BINDING_SLOTS()
        END_SHADER_PARAMETER_STRUCT()

        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& parameters)
        {
            return IsFeatureLevelSupported(parameters.Platform, ERHIFeatureLevel::SM5);
        }
    };

    IMPLEMENT_GLOBAL_SHADER(
        FAirSimStencilOutputPS,
        "/Plugin/AirSimShaders/Private/StencilLabelOutput.usf",
        "MainPS",
        SF_Pixel);

    class FAirSimStencilViewExtension final
        : public ISceneViewExtension
        , public TSharedFromThis<FAirSimStencilViewExtension, ESPMode::ThreadSafe>
    {
    public:
        explicit FAirSimStencilViewExtension(AirSimStencilViewExtension::EOutputMode output_mode)
            : output_mode_(static_cast<uint32>(output_mode))
        {
        }

        virtual void SetupViewFamily(FSceneViewFamily& /*in_view_family*/) override
        {
        }

        virtual void SetupView(FSceneViewFamily& /*in_view_family*/, FSceneView& /*in_view*/) override
        {
        }

        virtual void BeginRenderViewFamily(FSceneViewFamily& /*in_view_family*/) override
        {
        }

        virtual void SubscribeToPostProcessingPass(
            EPostProcessingPass pass,
            const FSceneView& /*view*/,
            FPostProcessingPassDelegateArray& callbacks,
            bool /*pass_enabled*/) override
        {
            if (pass == EPostProcessingPass::Tonemap)
            {
                callbacks.Add(FPostProcessingPassDelegate::CreateRaw(
                    this,
                    &FAirSimStencilViewExtension::PostProcessAfterTonemap_RenderThread));
            }
        }

    private:
        FScreenPassTexture PostProcessAfterTonemap_RenderThread(
            FRDGBuilder& graph_builder,
            const FSceneView& view,
            const FPostProcessMaterialInputs& inputs)
        {
            if (inputs.CustomDepthTexture == nullptr || view.Family == nullptr)
            {
                return inputs.ReturnUntouchedSceneColorForPostProcessing(graph_builder);
            }

            const FScreenPassTexture scene_color = FScreenPassTexture::CopyFromSlice(
                graph_builder,
                inputs.GetInput(EPostProcessMaterialInput::SceneColor));
            if (!scene_color.IsValid())
            {
                return inputs.ReturnUntouchedSceneColorForPostProcessing(graph_builder);
            }

            FScreenPassRenderTarget output = inputs.OverrideOutput;
            if (!output.IsValid())
            {
                output = FScreenPassRenderTarget::CreateFromInput(
                    graph_builder,
                    scene_color,
                    view.GetOverwriteLoadAction(),
                    TEXT("AirSimStencilOutput"));
            }

            FAirSimStencilOutputPS::FParameters* pass_parameters =
                graph_builder.AllocParameters<FAirSimStencilOutputPS::FParameters>();
            pass_parameters->SceneTextures = inputs.SceneTextures;
            pass_parameters->SceneTextureMin = view.UnscaledViewRect.Min;
            pass_parameters->OutputTextureMin = output.ViewRect.Min;
            pass_parameters->OutputMode = output_mode_;
            pass_parameters->RenderTargets[0] = output.GetRenderTargetBinding();

            FGlobalShaderMap* shader_map = GetGlobalShaderMap(view.GetFeatureLevel());
            TShaderMapRef<FAirSimStencilOutputPS> pixel_shader(shader_map);
            FPixelShaderUtils::AddFullscreenPass(
                graph_builder,
                shader_map,
                RDG_EVENT_NAME("AirSim.StencilOutput"),
                pixel_shader,
                pass_parameters,
                output.ViewRect);

            return output;
        }

        const uint32 output_mode_;
    };
}

TSharedPtr<ISceneViewExtension, ESPMode::ThreadSafe> AirSimStencilViewExtension::Create(EOutputMode output_mode)
{
    return MakeShared<FAirSimStencilViewExtension, ESPMode::ThreadSafe>(output_mode);
}
