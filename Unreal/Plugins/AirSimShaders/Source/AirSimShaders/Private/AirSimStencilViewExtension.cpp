#include "AirSimStencilViewExtension.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "GlobalShader.h"
#include "MeshPassProcessor.h"
#include "PixelShaderUtils.h"
#include "PostProcess/PostProcessMaterialInputs.h"
#include "RenderGraphBuilder.h"
#include "SceneInterface.h"
#include "ScenePrivate.h"
#include "SceneRendering.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "ScreenPass.h"
#include "SimpleMeshDrawCommandPass.h"

namespace
{
    BEGIN_SHADER_PARAMETER_STRUCT(FAirSimStencilRasterPassParameters, )
        SHADER_PARAMETER_STRUCT_INCLUDE(FViewShaderParameters, View)
        SHADER_PARAMETER_STRUCT_INCLUDE(FInstanceCullingDrawParams, InstanceCullingDrawParams)
        RENDER_TARGET_BINDING_SLOTS()
    END_SHADER_PARAMETER_STRUCT()

    class FAirSimStencilOutputPS : public FGlobalShader
    {
        DECLARE_GLOBAL_SHADER(FAirSimStencilOutputPS);
        SHADER_USE_PARAMETER_STRUCT(FAirSimStencilOutputPS, FGlobalShader);

        BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
            SHADER_PARAMETER_STRUCT_INCLUDE(FSceneTextureShaderParameters, SceneTextures)
            SHADER_PARAMETER_RDG_TEXTURE(Texture2D, AirSimDepthTexture)
            SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<uint2>, AirSimStencilTexture)
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

    bool IsNonNanitePrimitive(const FScene& scene, const FPrimitiveSceneProxy* primitive_scene_proxy)
    {
        if (primitive_scene_proxy == nullptr || !primitive_scene_proxy->ShouldRenderCustomDepth())
        {
            return false;
        }

        const FPrimitiveSceneInfo* primitive_scene_info = primitive_scene_proxy->GetPrimitiveSceneInfo();
        if (primitive_scene_info == nullptr)
        {
            return false;
        }

        const int32 primitive_index = primitive_scene_info->GetIndex();
        return scene.PrimitiveFlagsCompact.IsValidIndex(primitive_index) &&
            !scene.PrimitiveFlagsCompact[primitive_index].bIsNaniteMesh;
    }

    void AddVisibleStaticMeshes(
        const FScene& scene,
        const FViewInfo& view,
        FMeshPassProcessor& mesh_processor)
    {
        const int32 static_mesh_count = view.StaticMeshVisibilityMap.Num();
        for (int32 static_mesh_index = 0; static_mesh_index < static_mesh_count; ++static_mesh_index)
        {
            if (!view.StaticMeshVisibilityMap[static_mesh_index] ||
                !scene.StaticMeshes.IsAllocated(static_mesh_index))
            {
                continue;
            }

            const FStaticMeshBatch* static_mesh = scene.StaticMeshes[static_mesh_index];
            if (static_mesh == nullptr ||
                static_mesh->PrimitiveSceneInfo == nullptr ||
                !IsNonNanitePrimitive(scene, static_mesh->PrimitiveSceneInfo->Proxy))
            {
                continue;
            }

            const FPrimitiveSceneProxy* primitive_scene_proxy = static_mesh->PrimitiveSceneInfo->Proxy;
            if (static_mesh->bViewDependentArguments)
            {
                FMeshBatch view_dependent_mesh(*static_mesh);
                primitive_scene_proxy->ApplyViewDependentMeshArguments(view, view_dependent_mesh);
                mesh_processor.AddMeshBatch(
                    view_dependent_mesh,
                    ~0ull,
                    primitive_scene_proxy,
                    static_mesh->Id);
            }
            else
            {
                mesh_processor.AddMeshBatch(
                    *static_mesh,
                    ~0ull,
                    primitive_scene_proxy,
                    static_mesh->Id);
            }
        }
    }

    void AddVisibleDynamicMeshes(
        const FScene& scene,
        const FViewInfo& view,
        FMeshPassProcessor& mesh_processor)
    {
        const int32 dynamic_mesh_count = view.DynamicMeshElements.Num();
        for (int32 dynamic_mesh_index = 0; dynamic_mesh_index < dynamic_mesh_count; ++dynamic_mesh_index)
        {
            if (!view.DynamicMeshElementsPassRelevance.IsValidIndex(dynamic_mesh_index) ||
                !view.DynamicMeshElementsPassRelevance[dynamic_mesh_index].Get(EMeshPass::CustomDepth))
            {
                continue;
            }

            const FMeshBatchAndRelevance& mesh_and_relevance = view.DynamicMeshElements[dynamic_mesh_index];
            if (mesh_and_relevance.Mesh == nullptr ||
                !IsNonNanitePrimitive(scene, mesh_and_relevance.PrimitiveSceneProxy))
            {
                continue;
            }

            mesh_processor.AddMeshBatch(
                *mesh_and_relevance.Mesh,
                ~0ull,
                mesh_and_relevance.PrimitiveSceneProxy);
        }
    }

    FRDGTextureRef AddAirSimStencilRasterPass(
        FRDGBuilder& graph_builder,
        const FScene& scene,
        const FViewInfo& view,
        FIntPoint texture_extent)
    {
        const ETextureCreateFlags texture_flags =
            TexCreate_DepthStencilTargetable |
            TexCreate_ShaderResource |
            TexCreate_NoFastClear;
        FRDGTextureRef depth_stencil_texture = graph_builder.CreateTexture(
            FRDGTextureDesc::Create2D(
                texture_extent,
                PF_DepthStencil,
                FClearValueBinding::DepthFar,
                texture_flags),
            TEXT("AirSimStencilDepth"));

        FAirSimStencilRasterPassParameters* pass_parameters =
            graph_builder.AllocParameters<FAirSimStencilRasterPassParameters>();
        pass_parameters->View.View = view.ViewUniformBuffer;
        pass_parameters->View.InstancedView = view.GetInstancedViewUniformBuffer();
        pass_parameters->RenderTargets.DepthStencil = FDepthStencilBinding(
            depth_stencil_texture,
            ERenderTargetLoadAction::EClear,
            ERenderTargetLoadAction::EClear,
            FExclusiveDepthStencil::DepthWrite_StencilWrite);

        FSimpleMeshDrawCommandPass* simple_mesh_pass =
            graph_builder.AllocObject<FSimpleMeshDrawCommandPass>(view, nullptr);
        const EShadingPath shading_path = GetFeatureLevelShadingPath(scene.GetFeatureLevel());
        TUniquePtr<FMeshPassProcessor> mesh_processor(
            FPassProcessorManager::CreateMeshPassProcessor(
                shading_path,
                EMeshPass::CustomDepth,
                scene.GetFeatureLevel(),
                &scene,
                &view,
                simple_mesh_pass->GetDynamicPassMeshDrawListContext()));
        if (mesh_processor.IsValid())
        {
            AddVisibleStaticMeshes(scene, view, *mesh_processor);
            AddVisibleDynamicMeshes(scene, view, *mesh_processor);
        }

        simple_mesh_pass->BuildRenderingCommands(
            graph_builder,
            view,
            scene,
            pass_parameters->InstanceCullingDrawParams);
        pass_parameters->InstanceCullingDrawParams.Scene =
            view.GetSceneUniforms().GetBuffer(graph_builder);

        graph_builder.AddPass(
            RDG_EVENT_NAME("AirSim.StencilRaster"),
            pass_parameters,
            ERDGPassFlags::Raster,
            [simple_mesh_pass, pass_parameters, view_rect = view.ViewRect](
                FRDGAsyncTask,
                FRHICommandList& rhi_command_list)
            {
                rhi_command_list.SetViewport(
                    view_rect.Min.X,
                    view_rect.Min.Y,
                    0.0f,
                    view_rect.Max.X,
                    view_rect.Max.Y,
                    1.0f);
                simple_mesh_pass->SubmitDraw(
                    rhi_command_list,
                    pass_parameters->InstanceCullingDrawParams);
            });

        return depth_stencil_texture;
    }

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
            FAfterPassCallbackDelegateArray& callbacks,
            bool /*pass_enabled*/) override
        {
            if (pass == EPostProcessingPass::Tonemap)
            {
                callbacks.Add(FAfterPassCallbackDelegate::CreateRaw(
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
            if (inputs.CustomDepthTexture == nullptr ||
                !view.bIsViewInfo ||
                view.Family == nullptr ||
                view.Family->Scene == nullptr)
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

            const FScene* scene = view.Family->Scene->GetRenderScene();
            if (scene == nullptr)
            {
                return inputs.ReturnUntouchedSceneColorForPostProcessing(graph_builder);
            }

            const FViewInfo& view_info = static_cast<const FViewInfo&>(view);
            FRDGTextureRef airsim_depth_stencil = AddAirSimStencilRasterPass(
                graph_builder,
                *scene,
                view_info,
                inputs.CustomDepthTexture->Desc.Extent);

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
            pass_parameters->AirSimDepthTexture = airsim_depth_stencil;
            pass_parameters->AirSimStencilTexture = graph_builder.CreateSRV(
                FRDGTextureSRVDesc::CreateWithPixelFormat(airsim_depth_stencil, PF_X24_G8));
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
