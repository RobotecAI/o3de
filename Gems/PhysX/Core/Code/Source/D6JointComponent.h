/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Source/JointComponent.h>
namespace PhysX
{


    struct D6JointComponentConfiguration
    {
        enum class D6JointAxis : AZ::u32
        {
            Free,
            Limited,
            Locked
        };

        static AZStd::vector<AZStd::string> GetD6JointAxisNames()
        {
            return { "Free", "Limited", "Locked" };
        };

        AZ_CLASS_ALLOCATOR(D6JointComponentConfiguration, AZ::SystemAllocator);
        AZ_TYPE_INFO(D6JointComponentConfiguration, "{C8A3B7E2-9F4D-4A21-8F6A-2D3E5C7A9B1F}");
        static void Reflect(AZ::ReflectContext* context);

        D6JointComponentConfiguration() = default;

        D6JointAxis m_motioneX = D6JointAxis::Locked;
        D6JointAxis m_motioneY = D6JointAxis::Locked;
        D6JointAxis m_motioneZ = D6JointAxis::Locked;
        D6JointAxis m_motioneTwist = D6JointAxis::Locked;
        D6JointAxis m_motioneSwing1 = D6JointAxis::Locked;
        D6JointAxis m_motioneSwing2 = D6JointAxis::Locked;

    };

    //! Runtime D6 joint component.
    //! Provides a 6 degree of freedom joint constraint between two entities.
    //! D6 joints allow full control over all linear and angular motion, making them
    //! suitable for complex constraints like ragdoll joints, sliding doors with rotation, etc.
    class D6JointComponent
        : public JointComponent
        , public JointRequestBus::Handler
    {
    public:
        AZ_COMPONENT(D6JointComponent, "{F3A4C5D6-E7F8-9A0B-1C2D-3E4F5A6B7C8D}", JointComponent);

        static void Reflect(AZ::ReflectContext* context);

        D6JointComponent() = default;
        D6JointComponent(
            const JointComponentConfiguration& configuration,
            const JointGenericProperties& genericProperties,
            const D6JointComponentConfiguration& d6Configuration);
        ~D6JointComponent() = default;

        // JointRequestBus::Handler overrides
        float GetPosition() const override;
        float GetVelocity() const override;
        AZ::Transform GetTransform() const override;
        void SetVelocity(float velocity) override;
        void SetMaximumForce(float force) override;
        AZStd::pair<float, float> GetLimits() const override;

    protected:
        // JointComponent overrides
        void InitNativeJoint() override;
        void DeinitNativeJoint() override;

    private:
        void CachePhysXNativeD6Joint();

        physx::PxD6Joint* m_nativeD6Joint{ nullptr };
        D6JointComponentConfiguration m_d6Configuration;
    };
} // namespace PhysX