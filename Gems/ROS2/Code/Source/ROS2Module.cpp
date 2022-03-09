

#include <ROS2ModuleInterface.h>
#include <ROS2SystemComponent.h>

namespace ROS2
{
    class ROS2Module
        : public ROS2ModuleInterface
    {
    public:
        AZ_RTTI(ROS2Module, "{e15f6cfd-b4b4-43b9-8efd-64dc87ef3227}", ROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2Module, AZ::SystemAllocator, 0);
    };
}// namespace ROS2

AZ_DECLARE_MODULE_CLASS(Gem_ROS2, ROS2::ROS2Module)
