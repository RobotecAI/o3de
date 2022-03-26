
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <rclcpp/node.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ROS2
{
    class ROS2Requests
    {
    public:
        AZ_RTTI(ROS2Requests, "{a9bdbff6-e644-430d-8096-cdb53c88e8fc}");
        virtual ~ROS2Requests() = default;

        // Put your public methods here
        virtual std::shared_ptr<rclcpp::Node> GetNode() const = 0;
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const = 0;
    };
    
    class ROS2BusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2RequestBus = AZ::EBus<ROS2Requests, ROS2BusTraits>;
    using ROS2Interface = AZ::Interface<ROS2Requests>;

} // namespace ROS2
