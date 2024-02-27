/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <EditorModeFeedbackFeatureProcessor.h>
#include <Pass/State/FocusedEntityState.h>
#include <Pass/State/SelectedEntityState.h>

#include <Atom/RPI.Public/Pass/PassFilter.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/Utils/Utils.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>

namespace AZ
{
    namespace Render
    {
        namespace
        {
            [[maybe_unused]] const char* const Window = "EditorModeFeedback";
        }

        // Creates the material for the mask pass shader
        static Data::Instance<RPI::Material> CreateMaskMaterial()
        {
            const AZStd::string path = "shaders/editormodemask.azmaterial";
            const auto materialAsset = RPI::AssetUtils::LoadCriticalAsset<RPI::MaterialAsset>(path);
            const auto maskMaterial = RPI::Material::FindOrCreate(materialAsset);
            return maskMaterial;
        }

        void EditorModeFeatureProcessor::Reflect(ReflectContext* context)
        {
            if (auto* serializeContext = azrtti_cast<SerializeContext*>(context))
            {
                serializeContext->Class<EditorModeFeatureProcessor, RPI::FeatureProcessor>()->Version(1);
            }
        }

        void EditorModeFeatureProcessor::Activate()
        {
            EnableSceneNotification();

            EditorStateList editorStates;
            editorStates.push_back(AZStd::make_unique<FocusedEntityState>());
            editorStates.push_back(AZStd::make_unique<SelectedEntityState>());
            m_editorStatePassSystem = AZStd::make_unique<EditorStatePassSystem>(AZStd::move(editorStates));
            AZ::TickBus::Handler::BusConnect();
        }

        void EditorModeFeatureProcessor::Deactivate()
        {
            AZ::TickBus::Handler::BusDisconnect();
            m_editorStatePassSystem.reset();
            DisableSceneNotification();
        }

        void EditorModeFeatureProcessor::OnRenderPipelineChanged(
            RPI::RenderPipeline* renderPipeline, RPI::SceneNotification::RenderPipelineChangeType changeType)
        {
            if (!m_editorStatePassSystem)
            {
                return;
            }

            if (changeType == RPI::SceneNotification::RenderPipelineChangeType::Added ||
                changeType == RPI::SceneNotification::RenderPipelineChangeType::PassChanged)
            {
                m_editorStatePassSystem->ConfigureStatePassesForPipeline(renderPipeline);
            }
            else if (changeType == RPI::SceneNotification::RenderPipelineChangeType::Removed)
            {
                m_editorStatePassSystem->RemoveStatePassesForPipeline(renderPipeline);
            }
        }

        void EditorModeFeatureProcessor::AddRenderPasses(RPI::RenderPipeline* renderPipeline)
        {
            if (!m_editorStatePassSystem)
            {
                return;
            }

            m_editorStatePassSystem->AddPassesToRenderPipeline(renderPipeline);

            if (!m_maskRenderers.empty())
            {
                return;
            }

            for (const auto& mask : m_editorStatePassSystem->GetMasks())
            {
                // Emplaces the mask key and mask renderer value in place for the mask renderers map
                m_maskRenderers.emplace(AZStd::piecewise_construct, AZStd::forward_as_tuple(mask), AZStd::forward_as_tuple(mask));
            }
        }

        void EditorModeFeatureProcessor::Render([[maybe_unused]] const RenderPacket& packet)
        {
            if (!m_editorStatePassSystem || !m_maskMaterial)
            {
                return;
            }

            AZStd::string name = "Pool";
            AZStd::unordered_set<AZ::EntityId> entityIds;
            AZ::ComponentApplicationBus::Broadcast(
                &AZ::ComponentApplicationRequests::EnumerateEntities,
                [&entityIds, name](AZ::Entity* entity)
                {
                    bool meshVisible = false;
                    AZ::Render::MeshComponentRequestBus::EventResult(
                        meshVisible, entity->GetId(), &AZ::Render::MeshComponentRequests::GetVisibility);
                    if (meshVisible && AZ::StringFunc::Contains(entity->GetName(), name))
                    {
                        entityIds.insert(entity->GetId());
                    }
                });

            const auto entityMaskMap = m_editorStatePassSystem->GetEntitiesForEditorStates();
            if (!entityMaskMap.empty())
            {
                const auto sampleMask = entityMaskMap.begin()->first;
                if (auto it = m_maskRenderers.find(sampleMask); it != m_maskRenderers.end())
                {
                    it->second.RenderMaskEntities(m_maskMaterial, entityIds);
                }
            }
        }

        void EditorModeFeatureProcessor::Simulate([[maybe_unused]] const SimulatePacket& packet)
        {
            if (!m_editorStatePassSystem)
            {
                return;
            }

            m_editorStatePassSystem->Update();
        }

        void EditorModeFeatureProcessor::OnTick(float, AZ::ScriptTimePoint)
        {
            // Attempt deferred loading of mask material until the asset is ready
            if (!m_maskMaterial)
            {
                m_maskMaterial = CreateMaskMaterial();
            }

            if (m_maskMaterial)
            {
                AZ::TickBus::Handler::BusDisconnect();
            }
        }

        void EditorModeFeatureProcessor::SetEnableRender(bool enableRender)
        {
            enableRender = true;
            if (!m_editorStatePassSystem)
            {
                return;
            }

            const auto templateName = Name(m_editorStatePassSystem->GetParentPassTemplateName());

            auto passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(templateName, GetParentScene());
            AZ::RPI::PassSystemInterface::Get()->ForEachPass(
                passFilter,
                [enableRender](RPI::Pass* pass) -> RPI::PassFilterExecutionFlow
                {
                    pass->SetEnabled(enableRender);
                    return RPI::PassFilterExecutionFlow::ContinueVisitingPasses;
                });
        }
    } // namespace Render
} // namespace AZ
