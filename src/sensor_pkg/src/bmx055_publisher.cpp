#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <wiringPi.h> // WiringPiを使用する場合

// GPIOピン番号
const int led_pin = 11;

// I2Cアドレス
const int ACCL_ADDR = 0x19;

// I2Cバス
int bus;

void ledControl(int value)
{
    digitalWrite(led_pin, value);
}

double getAccData()
{
    int y_data, z_data;
    unsigned char data[4];
    
    i2cReadReg8(bus, 0x04, data[0]);
    i2cReadReg8(bus, 0x05, data[1]);
    i2cReadReg8(bus, 0x06, data[2]);
    i2cReadReg8(bus, 0x07, data[3]);

    y_data = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (y_data > 2047)
    {
        y_data -= 4096;
    }

    z_data = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (z_data > 2047)
    {
        z_data -= 4096;
    }

    double theta_rad = atan(static_cast<double>(z_data) / y_data);
    double theta_deg = theta_rad * 180.0 / M_PI;
    return theta_deg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bmx055_publisher");
    ros::NodeHandle nh;

    wiringPiSetup(); // WiringPiの初期化

    pinMode(led_pin, OUTPUT);

    int bus = wiringPiI2CSetup(ACCL_ADDR); // I2Cバスを初期化

    ros::Publisher imu_pub = nh.advertise<std_msgs::Float64>("theta_data", 1);

    while (ros::ok())
    {
        ledControl(HIGH);
        delay(2000);
        ledControl(LOW);

        double theta_deg = getAccData();

        std_msgs::Float64 imu_msg;
        imu_msg.data = theta_deg;

        imu_pub.publish(imu_msg);

        ros::spinOnce();
        delay(2000);
    }

    return 0;
}
