/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "SensorConfiguration.h"
//#include <AzCore/Memory/SystemAllocator.h>

namespace ROS2
{
    namespace Internal
    {
        AZStd::string GetNamespacedName(const AZStd::string& ns, const AZStd::string& name)
        {
            AZStd::string namespacedName;
            if (!ns.empty())
            {
                namespacedName = ns + "/";
            }
            namespacedName += name;
        }
    }

    //AZ_CLASS_ALLOCATOR_IMPL(ROS2::SensorConfiguration, AZ::SystemAllocator, 0);

    void SensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorConfiguration>()
                ->Version(1)
                ->Field("Namespace", &SensorConfiguration::m_namespace)
                ->Field("Frame Name", &SensorConfiguration::m_frameName)
                ->Field("Topic", &SensorConfiguration::m_topic)
                ->Field("Publishing Data", &SensorConfiguration::m_publishData)
                ->Field("Frequency (HZ)", &SensorConfiguration::m_hz)
                ->Field("Visualise", &SensorConfiguration::m_visualise)
                ->Field("Publishing Transform", &RigidBodyConfiguration::m_publishTransform)
                ->Field("Parent Frame Name", &SensorConfiguration::m_parentFrameName)
                ;
        }
    }

    AZStd::string SensorConfiguration::GetFrameID() const
    {
        return Internal::GetNamespacedName(m_namespace, m_frameName);
    }

    AZStd::string SensorConfiguration::GetPublishTopic() const
    {
        return Internal::GetNamespacedName(m_namespace, m_topic);
    }
}  // namespace ROS2

