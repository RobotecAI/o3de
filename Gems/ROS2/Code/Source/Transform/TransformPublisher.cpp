/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "TransformPublisher.h"
#include "ROS2/ROS2Bus.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ROS2
{
    TransformPublisher::TransformPublisher(
        const AZStd::string& parentFrame, const AZStd::string& childFrame, const AZ::Transform& o3deTransform)
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_staticTFPublisher = AZstd::make_unique<tf2_ros::StaticTransformBroadcaster>(ros2Node);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        t.header.frame_id = parentFrame;
        t.child_frame_id = childFrame;
        t.transform.translation = ROS2Conversions::ToROS2Vector(o3deTransform.GetTranslation());
        t.transform.rotation = ROS2Conversions::ToROS2Quaternion(o3deTransform.GetRotation());
        tf_publisher_->sendTransform(t);
    }
}  // namespace ROS2
