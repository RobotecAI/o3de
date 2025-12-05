/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/EditorD6JointComponent.h>
#include <Source/D6JointComponent.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

namespace PhysX
{


    void EditorD6JointComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorD6JointComponent, EditorJointComponent>()
                ->Version(1)
                ->Field("D6 Limits", &EditorD6JointComponent::m_d6LimitConfig)
                ->Field("Component Mode", &EditorD6JointComponent::m_componentModeDelegate)
                ;

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorD6JointComponent>("PhysX D6 Joint", "D6 joint provides 6 degree of freedom constraint")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "PhysX")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorD6JointComponent::m_d6LimitConfig, "D6 Limits", "D6 joint limits")
                    ;
            }
        }
    }

    void EditorD6JointComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PhysicsJointService"));
    }

    void EditorD6JointComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
    }

    void EditorD6JointComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("NonUniformScaleService"));
    }

    void EditorD6JointComponent::Activate()
    {
        EditorJointComponent::Activate();
    }

    void EditorD6JointComponent::Deactivate()
    {
        EditorJointComponent::Deactivate();
    }

    void EditorD6JointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        m_config.m_followerEntity = GetEntityId();

        D6JointComponentConfiguration d6Config;

        gameEntity->CreateComponent<D6JointComponent>(
            m_config.ToGameTimeConfig(), m_config.ToGenericProperties(), d6Config);
    }

    float EditorD6JointComponent::GetLinearValue([[maybe_unused]] const AZStd::string& parameterName)
    {
        return 0.0f;
    }

    AngleLimitsFloatPair EditorD6JointComponent::GetLinearValuePair([[maybe_unused]] const AZStd::string& parameterName)
    {
        return AngleLimitsFloatPair();
    }

    AZStd::vector<JointsComponentModeCommon::SubModeParameterState> EditorD6JointComponent::GetSubComponentModesState()
    {
        return AZStd::vector<JointsComponentModeCommon::SubModeParameterState>();
    }

    void EditorD6JointComponent::SetBoolValue([[maybe_unused]] const AZStd::string& parameterName, [[maybe_unused]] bool value)
    {
    }

    void EditorD6JointComponent::SetLinearValue([[maybe_unused]] const AZStd::string& parameterName, [[maybe_unused]] float value)
    {
    }

    void EditorD6JointComponent::SetLinearValuePair([[maybe_unused]] const AZStd::string& parameterName, [[maybe_unused]] const AngleLimitsFloatPair& valuePair)
    {
    }

    void EditorD6JointComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo,
        [[maybe_unused]] AzFramework::DebugDisplayRequests& debugDisplay)
    {
    }
} // namespace PhysX