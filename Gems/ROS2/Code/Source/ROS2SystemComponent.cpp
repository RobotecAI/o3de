
#include <ROS2SystemComponent.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void ROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SystemComponent>("ROS2", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void ROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2SystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2SystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2SystemComponent::ROS2SystemComponent()
    {
        if (ROS2Interface::Get() == nullptr)
        {
            ROS2Interface::Register(this);
        }
    }

    ROS2SystemComponent::~ROS2SystemComponent()
    {
        if (ROS2Interface::Get() == this)
        {
            ROS2Interface::Unregister(this);
        }
    }

    void ROS2SystemComponent::Init()
    {
        rclcpp::init(0, 0);
        ros2_node = std::make_shared<rclcpp::Node>("o3de_ros2_node");
        chat_publisher = ros2_node->create_publisher<std_msgs::msg::String>("chatter_o3de", 10);
    }

    void ROS2SystemComponent::Activate()
    {
        ROS2RequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ROS2RequestBus::Handler::BusDisconnect();
    }

    void ROS2SystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        static int i = 0;
        auto message = std_msgs::msg::String();
        message.data = "O3DE says hello to ROS " + std::to_string(i++);
        chat_publisher->publish(message);
    }

} // namespace ROS2
