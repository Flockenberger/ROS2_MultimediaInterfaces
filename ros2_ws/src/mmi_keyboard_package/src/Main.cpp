#include "PublisherNode.h"
#include "SerialNode.h"
#include "ComputeNode.h"

int main(int argc, char* argv[])
{
    //Initialize rclcpp otherwise we will fail when we create our Nodes
    rclcpp::init(argc, argv);
    
    //Note the order of creating the nodes is somewhat important
    mmi::Ref<mmi::ComputeNode> computeNode = mmi::CreateRef<mmi::ComputeNode>();
    mmi::Ref<mmi::PublisherNode> publisherNode = mmi::CreateRef<mmi::PublisherNode>();

    mmi::SerialConnection serial = {};
    serial.AutoDetectPort = true;
    serial.Port = "COM5";
    serial.Baud = mmi::SerialBaud::BAUD_115200;
    mmi::Ref<mmi::SerialNode> serialNode = mmi::CreateRef<mmi::SerialNode>(serial);

    //We use a multi threaded executor here
    //to be able to use more than one node within our package
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(publisherNode);
    executor.add_node(serialNode);
    executor.add_node(computeNode);

    executor.spin(); //they spin me right round baby right round...

	rclcpp::shutdown();

    return 0;
}
