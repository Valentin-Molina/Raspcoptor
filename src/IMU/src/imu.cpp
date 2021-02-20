// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/Imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <iostream>

#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

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
        // init publishers
        imuMessagePublisher = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);   // topic + QoS
        stringPublisher = create_publisher<std_msgs::msg::String>("imu_status", 10);   // topic + QoS

        // initialize device
        publish_string_message("Initializing I2C devices...");
        mpu.initialize();

        // verify connection
        publish_string_message("Testing device connections...");
        
        sprintf(buffer, mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed");
        publish_string_message(buffer);

        // load and configure the DMP
        publish_string_message("Initializing DMP...");
        devStatus = mpu.dmpInitialize();
        
        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            
            publish_string_message("Enabling DMP...");
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
            //attachInterrupt(0, dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            publish_string_message("DMP ready!");
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            sprintf(buffer, "DMP Initialization failed (code %d)", devStatus);
            publish_string_message(buffer);
        }

        // init timer - the function will be called with the given rate
        read_data_timer = create_wall_timer(1000ms,    // rate
                                            [&]() 
                                            {read_data();});
    }

void read_data() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        
        publish_string_message("FIFO overflow!");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        imuMessage.orientation.set__x(q.x);
        imuMessage.orientation.set__y(q.y);
        imuMessage.orientation.set__z(q.z);
        imuMessage.orientation.set__w(q.w);
        imuMessagePublisher->publish(imuMessage);
        sprintf(buffer, "Quaternion --> x: %f y: %f z: %f w: %f", q.x, q.y, q.z, q.w);
        std::cout << buffer << std::endl;
    }
}

void publish_string_message(std::string messageData) {
    std::cout << messageData << std::endl;
    stringMessage.data = messageData;
    stringPublisher->publish(stringMessage);
}

private:
    char buffer[300];
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    Quaternion q;

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuMessagePublisher;
    sensor_msgs::msg::Imu imuMessage;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stringPublisher;
    std_msgs::msg::String stringMessage;
    
    rclcpp::TimerBase::SharedPtr read_data_timer;    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<ImuNode>(options));
    rclcpp::shutdown();
    return 0;
}
