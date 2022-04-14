/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2FrameComponent::Activate()
    {
        const AZ::TransformInterface* transformInterface = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        m_staticTFPublisher = AZStd::make_unique<TransformPublisher>(GetParentFrameID(), GetFrameID(),
                                                                     GetParentFrameTransform(transformInterface));
    }

    void ROS2FrameComponent::Deactivate()
    {
        m_staticTFPublisher.reset();
    }

    const ROS2FrameComponent* GetParentROS2FrameComponent()
    {
        if (const AZ::TransformInterface* parentEntity = GetEntityTransformInterface()->GetParent())
        {
            return parentEntity->FindComponent<ROS2FrameComponent>())
        }
        return nullptr;
    }

    const ROS2FrameComponent::AZ::Transform& GetFrameTransform() const
    {
        if (GetParentROS2FrameComponent() != nullptr)
        {
            return transformInterface->GetLocalTM();
        }
        return transformInterface->GetWorldTM();
    }

    const AZ::TransformInterface* GetEntityTransformInterface() const
    {
        return GetEntity()->FindComponent<AzFramework::TransformComponent>();
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID()
    {
        AZStd::string parentID = "world";
        if (auto parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
    }

    AZStd::string ROS2FrameComponent::GetFrameID()
    {
        return ROS2Names::GetNamespacedName(m_namespace, m_frameName);
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        return m_namespace;
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()
                ->Version(1)
                ->Field("Namespace", &ROS2FrameComponent::m_namespace)
                ->Field("Frame Name", &ROS2FrameComponent::m_frameName)
                ;
        }
    }

    void ROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
    }
} // namespace ROS2
