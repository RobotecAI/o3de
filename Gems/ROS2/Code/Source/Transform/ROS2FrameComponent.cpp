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
    namespace Internal
    {
        ROS2FrameComponent* GetParentROS2FrameComponent(const AZ::TransformInterface* transformInterface)
        {
            if (const AZ::TransformInterface* parentEntity = transformInterface->GetParent())
            {
                return parentEntity->FindComponent<ROS2FrameComponent>())
            }
            return nullptr;
        }

        const AZ::Transform& GetParentFrameTransform(const AZ::TransformInterface* transformInterface) const
        {
            if (Internal::GetParentROS2FrameComponent(transformInterface))
            {   // it is used as "HasParentROS2FrameComponent(..)"
                return transformInterface->GetLocalTM();
            }
            return transformInterface->GetWorldTM();
        }
    }

    void ROS2FrameComponent::Activate()
    {
        m_transformInterface = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        m_frameTransform = GetParentFrameTransform(m_transformInterface);

        m_staticTFPublisher = AZStd::make_unique<TransformPublisher>(GetParentFrameID(), GetFrameID(), m_frameTransform);
    }

    void ROS2FrameComponent::Deactivate()
    {
        m_staticTFPublisher.reset();
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID()
    {
        AZStd::string parentID = "world";
        if (auto parentFrame = Internal::GetParentROS2FrameComponent(GetTransformInterface()); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
    }

    AZStd::string ROS2FrameComponent::GetFrameID()
    {
        return ROS2Names::GetNamespacedName(m_namespace, m_frameName);
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

    void ROS2FrameComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
    }
} // namespace ROS2
