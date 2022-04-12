/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

//#include <AzCore/Memory/Memory.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    /// General configuration for sensors
    struct SensorConfiguration
    {
    public:
        //AZ_CLASS_ALLOCATOR_DECL;
        AZ_RTTI(SensorConfiguration, "{4755363D-0B5A-42D7-BBEF-152D87BA10D7}");
        static void Reflect(AZ::ReflectContext* context);

        // TODO - consider moving to ROS2Frame struct
        AZStd::string m_namespace; // TODO - option to fill from entity name, default = true, validation
        AZStd::string m_frameName = "sensor_frame"; // TODO - option to fill from entity name, validation

        // TODO - publishing-related data
        AZStd::string m_topic; // TODO - apply namespace, default to standard names per type, validation
        bool m_publishData = true;
        float m_hz = 10;
        // TODO - add QoS here (struct, mapped to ros2 QoS).

        bool m_visualise = true;

        // TODO - transform-related data (ROS2Transform needs ROS2Frame, ROS2Publisher also needs ROS2Frame)
        bool m_publishTransform = true;
        AZStd::string m_parentFrameName = "base_link"; // TODO - option to fill from parent entity, default = false, validation

        AZStd::string GetFrameID() const;
        AZStd::string GetPublishTopic() const;
    }
}  // namespace ROS2

