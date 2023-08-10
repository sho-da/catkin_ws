#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <pigpio.h>

//=========================================================
//Rotary encoder variables
int pin1 = 18;  // to A
int pin2 = 16;  // to B
int rotary_encoder_frequency = 40000; //usec
int rotary_encoder_resolution = 100;
int encoder_value = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

void rotary_encoder()
{  
    static int code; 
    //check the movement
    code = ((code << 2) + (gpioRead(pin2) << 1) + gpioRead(pin1)) & 0xf; // !caution!
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}

int main() {
    ros::init(argc, argv, "encoder_publisher");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<std_msgs::Float64>("theta2_topic", 1);

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        return 1;
    }
    //-------------------------------------------
    //Rotary encoder initialization
    //-------------------------------------------
    encoder_value = 0;  
    
    // ピンを入力モードに設定
    gpioSetMode(pin1, PI_INPUT);
    gpioSetMode(pin2, PI_INPUT);

    ros::Rate rate(rotary_encoder_frequency);  // パブリッシュの頻度を設定 (40000 Hz)

    float theta2_data;

    while (ros::ok()) {
        rotary_encoder();
        theta2_data = encoder_value * (2 * 3.14f) / (4 * rotary_encoder_resolution);
        std_msgs::Float64 imu_msg;
        imu_msg.data = theta_data2;
        imu_pub.publish(imu_msg);
        ROS_INFO("Published from encoder: %d", imu_msg.data);
        rate.sleep();
    }

    gpioTerminate();
    return 0;
}