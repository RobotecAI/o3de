/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/D6JointComponent.h>
#include <PhysX/Joint/Configuration/PhysXJointConfiguration.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <Joint/PhysXJointUtils.h>
namespace PhysX
{
    void D6JointComponentConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<D6JointComponentConfiguration>()
                            ->Version(1)
                            ->Field("MotionX", &D6JointComponentConfiguration::m_motioneX)
                            ->Field("MotionY", &D6JointComponentConfiguration::m_motioneY)
                            ->Field("MotionZ", &D6JointComponentConfiguration::m_motioneZ)
                            ->Field("MotionTwist", &D6JointComponentConfiguration::m_motioneTwist)
                            ->Field("MotionSwing1", &D6JointComponentConfiguration::m_motioneSwing1)
                            ->Field("MotionSwing2", &D6JointComponentConfiguration::m_motioneSwing2);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<D6JointComponentConfiguration>("D6 Joint Configuration", "Configuration for D6 joint motion")
                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneX,
                               "Motion X",
                               "Linear motion along X axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited")

                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneY,
                               "Motion Y",
                               "Linear motion along Y axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited")

                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneZ,
                               "Motion Z",
                               "Linear motion along Z axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited")

                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneTwist,
                               "Motion Twist",
                               "Angular motion around X axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited")

                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneSwing1,
                               "Motion Swing 1",
                               "Angular motion around Y axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited")

                           ->DataElement(
                               AZ::Edit::UIHandlers::ComboBox,
                               &D6JointComponentConfiguration::m_motioneSwing2,
                               "Motion Swing 2",
                               "Angular motion around Z axis")
                           ->EnumAttribute(D6JointAxis::Free, "Free")
                           ->EnumAttribute(D6JointAxis::Locked, "Locked")
                           ->EnumAttribute(D6JointAxis::Limited, "Limited");
            }
        }
    }

    void D6JointComponent::Reflect(AZ::ReflectContext* context)
    {
        D6JointComponentConfiguration::Reflect(context);

        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<D6JointComponent, JointComponent>()
                ->Version(1)
                ->Field("D6 Configuration", &D6JointComponent::m_d6Configuration)
            ;
        }
    }

    D6JointComponent::D6JointComponent(
        const JointComponentConfiguration& configuration,
        const JointGenericProperties& genericProperties,
        const D6JointComponentConfiguration& d6Configuration)
        : JointComponent(configuration, genericProperties)
        , m_d6Configuration(d6Configuration)
    {

    }
    physx::PxD6Motion::Enum ConvertD6JointAxisToPxD6Motion(D6JointComponentConfiguration::D6JointAxis axis)
    {
        switch (axis)
        {
        case D6JointComponentConfiguration::D6JointAxis::Free:
            return physx::PxD6Motion::eFREE;
        case D6JointComponentConfiguration::D6JointAxis::Limited:
            return physx::PxD6Motion::eLIMITED;
        case D6JointComponentConfiguration::D6JointAxis::Locked:
            return physx::PxD6Motion::eLOCKED;
        default:
            AZ_Assert(false, "Unsupported D6 joint axis type");
            return physx::PxD6Motion::eLOCKED;
        }
    }
    void D6JointComponent::InitNativeJoint()
    {
        JointComponent::LeadFollowerInfo leadFollowerInfo;
        ObtainLeadFollowerInfo(leadFollowerInfo);
        if (leadFollowerInfo.m_followerActor == nullptr ||
            leadFollowerInfo.m_followerBody == nullptr)
        {
            return;
        }
        // if there is no lead body, this will be a constraint of the follower's global position, so use invalid body handle.
        AzPhysics::SimulatedBodyHandle parentHandle = AzPhysics::InvalidSimulatedBodyHandle;
        if (leadFollowerInfo.m_leadBody != nullptr)
        {
            parentHandle = leadFollowerInfo.m_leadBody->m_bodyHandle;
        }
        else
        {
            AZ_Warning(
                "PhysX", false, "Entity [%s] Hinge Joint component missing lead entity. This joint will be a global constraint on the follower's global position.",
                GetEntity()->GetName().c_str());
        }
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No sceneInterface");

        PhysX::D6JointLimitConfiguration configuration;
        if (auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get())
        {
            m_jointHandle = sceneInterface->AddJoint(
                leadFollowerInfo.m_followerBody->m_sceneOwner,
                &configuration,
                parentHandle,
                leadFollowerInfo.m_followerBody->m_bodyHandle);
            // get native joint

            const auto* joint = sceneInterface->GetJointFromHandle(m_jointSceneOwner, m_jointHandle);
            physx::PxJoint* native = static_cast<physx::PxJoint*>(joint->GetNativePointer());
            m_nativeD6Joint = native->is<physx::PxD6Joint>();

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eX, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneX));

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eY, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneY));

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eZ, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneZ));

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eTWIST, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneTwist));

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eSWING1, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneSwing1));

            m_nativeD6Joint->setMotion(
                physx::PxD6Axis::eSWING2, ConvertD6JointAxisToPxD6Motion(m_d6Configuration.m_motioneSwing2));

            m_jointSceneOwner = leadFollowerInfo.m_followerBody->m_sceneOwner;
        }


    }

    void D6JointComponent::DeinitNativeJoint()
    {
    }

    void D6JointComponent::CachePhysXNativeD6Joint()
    {

    }

    float D6JointComponent::GetPosition() const
    {
        return 0.0f;
    }

    float D6JointComponent::GetVelocity() const
    {
        return 0.0f;
    }

    AZStd::pair<float, float> D6JointComponent::GetLimits() const
    {
        return AZStd::make_pair(0.0f, 0.0f);
    }

    AZ::Transform D6JointComponent::GetTransform() const
    {
        return AZ::Transform::CreateIdentity();
    }

    void D6JointComponent::SetVelocity([[maybe_unused]] float velocity)
    {
    }

    void D6JointComponent::SetMaximumForce([[maybe_unused]] float force)
    {
    }
} // namespace PhysX