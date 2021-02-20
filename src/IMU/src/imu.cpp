// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/msg/Imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <iostream>

#include <MPU6050.h>

using namespace std::chrono_literals;

// a useful function to get the index of a string in a vector of strings
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
    const auto elem = std::find(names.begin(), names.end(), name);
    return std::distance(names.begin(), elem);
}


class ImuNode : public rclcpp::Node
{
public:
    ImuNode(rclcpp::NodeOptions options) : Node("imu_node", options)
    {
        // initialize device
        printf("Initializing I2C devices...\n");
        accelgyro.initialize();

        // verify connection
        printf("Testing device connections...\n");
        printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

        // init whatever is needed for your node
        
        // init subscribers
        // subscriber = create_subscription<geometry_msgs::msg::Twist>(
        //     "topic",    // which topic
        //     10,         // QoS            
        //     [this](geometry_msgs::msg::Twist::UniquePtr msg)    // callback are perfect for lambdas
        //     {
        //         last_twist = *msg;
        //     });
            
        // init publishers
        // publisher = create_publisher<sensor_msgs::msg::Imu>("ground_truth", 10);   // topic + QoS
        publisher = create_publisher<std_msgs::msg::String>("ground_truth", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(1000ms,    // rate
                                          [&]() 
                                          {loop();});
    }

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // display accel/gyro x/y/z values
    sprintf(buffer, "a/g: %6hd %6hd %6hd   %6hd %6hd %6hd\n",ax,ay,az,gx,gy,gz);
    message.data = buffer;
    std::cout << message.data << std::endl ;
    publisher->publish(message);
}

private:
    char buffer[300];
    MPU6050 accelgyro;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    std_msgs::msg::String message;
    
    rclcpp::TimerBase::SharedPtr publish_timer;    
};

// register this plugin
// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ImuNode>(options));
  rclcpp::shutdown();
  return 0;
}
