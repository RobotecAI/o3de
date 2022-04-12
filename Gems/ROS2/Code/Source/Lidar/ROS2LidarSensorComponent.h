/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/make_unique.h>
#include <AzFramework/Components/TransformComponent.h>

#include "Sensor/ROS2SensorComponent.h"
#include "Lidar/LidarTemplate.h"

namespace ROS2
{
    class ROS2LidarSensorComponent
        : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, "{502A955F-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2LidarSensorComponent() = default;
        void ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        void Activate() override;
        void Deactivate() override;

    private:
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::SickMRS6000;

        // TODO - also add a data acquisition implementation choice (and consider propagating abstraction upwards)
    };
}  // namespace ROS2