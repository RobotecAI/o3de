/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2/ROS2Bus.h"
#include "ROS2SensorComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2SensorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SensorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ROS2SensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SensorComponent, AZ::Component>()
                ->Version(1)
                ->Field("SensorConfiguration", &ROS2SensorComponent::m_sensorConfiguration)
                ;
        }
    }

    const SensorConfiguration& ROS2SensorComponent::GetConfiguration() const
    {
        return m_sensorConfiguration;
    }

    void ROS2SensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService")); //TODO - requires ROS2Frame service
    }

    void ROSSensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        auto hz = m_sensorConfiguration.m_hz;

        // TODO - add range validation (Attributes?)
        auto frameTime = hz == 0 ? 1 : 1 / hz;

        static float elapsed = 0;
        elapsed += deltaTime;
        if (elapsed < frameTime)
            return;

        elapsed -= frameTime;
        if (deltaTime > frameTime)
        {   // Frequency higher than possible, not catching up, just keep going with each frame.
            elapsed = 0;
        }

        // Note that sensor frequency can be limited by simulation tick rate (if higher sensor Hz is desired).
        FrequencyTick();
    }
} // namespace ROS2
