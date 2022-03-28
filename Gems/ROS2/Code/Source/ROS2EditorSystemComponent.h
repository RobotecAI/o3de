/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include <ROS2SystemComponent.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace ROS2
{
    /// System component for ROS2 editor
    class ROS2EditorSystemComponent
        : public ROS2SystemComponent
        , private AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2SystemComponent;
    public:
        AZ_COMPONENT(ROS2EditorSystemComponent, "{de922f72-e8da-43f3-a646-6b08458ed798}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        ROS2EditorSystemComponent();
        ~ROS2EditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2
