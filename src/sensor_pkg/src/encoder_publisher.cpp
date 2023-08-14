#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <pigpiod_if2.h>

//=========================================================
//Rotary encoder variables
const int pin1 = 23;  // to A
const int pin2 = 24;  // to B

const int rotary_encoder_frequency = 40000; //hz
const int rotary_encoder_resolution = 100;
const int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
int encoder_value = 0;

int pi2;

void rotary_encoder()
{  
    static int code; 
    //check the movement
    code = ((code << 2) + (gpio_read(pi2,pin2) << 1) + gpio_read(pi2, pin1)) & 0xf; // !caution!
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder_publisher");
    ros::NodeHandle nh;
    ros::Publisher enc_pub = nh.advertise<std_msgs::Float64>("theta2_topic", 10);

    pi2=pigpio_start(NULL,NULL);

    //-------------------------------------------
    //Rotary encoder initialization
    //-------------------------------------------
    encoder_value = 0;  
    
    // ピンを入力モードに設定
    set_mode(pi2,pin1, PI_INPUT);
    set_mode(pi2,pin2, PI_INPUT);

    ros::Rate rate(rotary_encoder_frequency);  // パブリッシュの頻度を設定 (40000 Hz)

    float theta2_data;

    while (ros::ok()) {
        rotary_encoder();
        theta2_data = encoder_value * (2 * 3.14f) / (4 * rotary_encoder_resolution);
        std_msgs::Float64 enc_msg;
        enc_msg.data = theta2_data;
        enc_pub.publish(enc_msg);
        ROS_INFO("Published from rotary encoder: %f", enc_msg.data);
        rate.sleep();
    }

    pigpio_stop(pi2);
    return 0;
}