#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <pigpio.h>

//=========================================================
//Rotary encoder variables
int pin1 = 18;  // to A
int pin2 = 16;  // to B
int rotary_encoder_update_rate = 25; //usec
int rotary_encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
float pre_theta2 = 0;

void rotary_encoder()
{  
    static int code; 
    //check the movement
    code = ((code << 2) + (gpioRead(pin2) << 1) + gpioRead(pin1)) & 0xf;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}

int main() {
    ros::init(argc, argv, "encoder_publisher");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<std_msgs::Float64>("theta_data2_pub", 1);

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        return 1;
    }
    
    int code = 0;

    // ピンを入力モードに設定
    gpioSetMode(pin1, PI_INPUT);
    gpioSetMode(pin2, PI_INPUT);

    ros::Rate rate(4000);  // パブリッシュの頻度を設定 (4000 Hz)

    while (ros::ok()) {
        rotary_encoder()
        float theta_data2 = encoder_value * (2 * 3.14f) / (4 * rotary_encoder_resolution);
        std_msgs::Float64 imu_msg;
        imu_msg.data = theta_data2;
        imu_pub.publish(imu_msg);
        ROS_INFO("Published from encoder: %d", imu_msg.data);
        rate.sleep();
    }

    gpioTerminate();
    return 0;
}