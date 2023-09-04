#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "pigpiod_if2.h"
#include <cmath>
#include <unistd.h>

const int ACC_ADDR=0x19;
const int GYR_ADDR=0x69;
int pi;

//=========================================================
//Accelerometer and gyro statistical data
const int sample_num = 100;
const float meas_interval = 0.01;
float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;

//=========================================================
//Kalman filter (for angle estimation) variables
//Update rate
const float theta_update_freq = 400; //Hz
const float theta_update_interval = 1.0f/theta_update_freq;
//State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
//Covariance matrix
float P_theta_predict[2][2];
//"A" of the state equation
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};  //const
//"B" of the state equation
float B_theta[2][1] = {{theta_update_interval}, {0}};         //const
//"C" of the state equation
float C_theta[1][2] = {{1, 0}};                               //const

//=========================================================
// Matrix common functions
//=========================================================
//Matrix addition
void mat_add(float *m1, float *m2, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = m1[i*column + j] + m2[i*column + j];    
        }    
    }
    return;
}

//Matrix subtraction
void mat_sub(float *m1, float *m2, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = m1[i*column + j] - m2[i*column + j];    
        }    
    }
    return;
}

//Matrix multiplication
void mat_mul(float *m1, float *m2, float *sol, int row1, int column1, int row2, int column2)
{
    for(int i=0; i<row1; i++)
    {
        for(int j=0; j<column2; j++)
        {
            sol[i*column2 + j] = 0;
            for(int k=0; k<column1; k++)
            {
                sol[i*column2 + j] += m1[i*column1 + k]*m2[k*column2 + j];    
            }
        }    
    }
    return;
}

//Matrix transposition
void mat_tran(float *m1, float *sol, int row_original, int column_original)
{
    for(int i=0; i<row_original; i++)
    {
        for(int j=0; j<column_original; j++)
        {
            sol[j*row_original + i] = m1[i*column_original + j];    
        }    
    }
    return;
}

//Matrix scalar maltiplication
void mat_mul_const(float *m1,float c, float *sol, int row, int column)
{
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            sol[i*column + j] = c * m1[i*column + j];    
        }    
    }
    return;
}

float get_acc_data(int bus) {
    unsigned char data[4];
    i2c_read_i2c_block_data(pi,bus, 0x04, (char *)data, 4);
    
    int y_data = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (y_data > 2047) {y_data -= 4096;}

    int z_data = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (z_data > 2047) {z_data -= 4096;}

    float theta1_deg = atan2( float(z_data),float(y_data)) * 57.29578f;
    return theta1_deg;
}

void acc_init(int bus)
{             
    //initialize ACC register 0x0F (range)
    //Full scale = +/- 2 G
    i2c_write_byte_data(pi,bus, 0x0F, 0x03);
    //initialize ACC register 0x10 (band width)
    //Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi,bus, 0x10, 0x0F);
 
    //get data
    float theta_array[sample_num];
    for(int i=0; i<sample_num; i++)
    {
        theta_array[i] = get_acc_data(bus);    
        sleep( meas_interval );
    }
    
    //calculate mean
    theta_mean = 0;
    for(int i=0; i<sample_num; i++)
    {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;
    
    //calculate variance
    float temp;
    theta_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp*temp;
    }
    theta_variance /= sample_num;
    return;
}

float get_gyr_data(int bus) {
    unsigned char data[2];
    i2c_read_i2c_block_data(pi,bus, 0x02, (char*)data, 2);
    
    int theta1_dot = data[0] + 256 * data[1];
    if (theta1_dot > 32767) {theta1_dot -= 65536;}
    theta1_dot = -1 * theta1_dot; // !caution!
    // +1000 (deg/sec) / 2^15 = 0.0305176
    return float(theta1_dot) * 0.0305176f;
}

//statistical data of gyro
void gyr_init(int bus)
{             
    //initialize Gyro register 0x0F (range)
    //Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi,bus, 0x0F, 0x01);
    //initialize Gyro register 0x10 (band width)
    //Data rate = 1000 Hz, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi,bus, 0x10, 0x02);
  
    //get data
    float theta_dot_array[sample_num];
    for(int i=0;i<sample_num;i++)
    {
        theta_dot_array[i] = get_gyr_data(bus);    
        sleep(meas_interval);
    }
    
    //calculate mean
    theta_dot_mean = 0;
    for(int i=0;i<sample_num;i++)
    {
        theta_dot_mean += theta_dot_array[i];    
    }
    theta_dot_mean /= sample_num;
 
    //calculate variance
    float temp;
    theta_dot_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp*temp;    
    }
    theta_dot_variance /= sample_num;
    return;
}

//=========================================================
//Kalman filter for "theta" & "theta_dot_bias" 
//It takes 650 usec. (NUCLEO-F401RE 84MHz, BMX055)
//=========================================================
void update_theta(int bus_acc,int bus_gyr)
{     
    //measurement data
    float y = get_acc_data(bus_acc); //degree
    
    //input data
    float theta_dot_gyr = get_gyr_data(bus_gyr); //degree/sec
      
    //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    float tran_C_theta[2][1] = {};
    float P_CT[2][1] = {};
    mat_tran(C_theta[0], tran_C_theta[0], 1, 2);//C^T
    mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);//P'C^T
    float G_temp1[1][1] = {};
    mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1,2, 2,1);//CP'C^T
    float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);//(W+CP'C^T)^-1
    float G[2][1] = {};
    mat_mul_const(P_CT[0], G_temp2, G[0], 2, 1);//P'C^T(W+CP'C^T)^-1
    
    //theta_data estimation: theta = theta'+G(y-Ctheta')
    float C_theta_theta[1][1] = {};
    mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2, 2, 1);//Ctheta'
    float delta_y = y - C_theta_theta[0][0];//y-Ctheta'
    float delta_theta[2][1] = {};
    mat_mul_const(G[0], delta_y, delta_theta[0], 2, 1);
    mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);
           
    //calculate covariance matrix: P=(I-GC)P'
    float GC[2][2] = {};
    float I2[2][2] = {{1,0},{0,1}};
    mat_mul(G[0], C_theta[0], GC[0], 2, 1, 1, 2);//GC
    float I2_GC[2][2] = {};
    mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
    float P_theta[2][2] = {};
    mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);//(I-GC)P'
      
    //predict the next step data: theta'=Atheta+Bu
    float A_theta_theta[2][1] = {};
    float B_theta_dot[2][1] = {};
    mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);//Atheta
    mat_mul_const(B_theta[0], theta_dot_gyr, B_theta_dot[0], 2, 1);//Bu
    mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2, 1);//Atheta+Bu 
    
    //predict covariance matrix: P'=APA^T + BUB^T
    float AP[2][2] = {};   
    float APAT[2][2] = {};
    float tran_A_theta[2][2] = {};
    mat_tran(A_theta[0], tran_A_theta[0], 2, 2);//A^T 
    mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);//AP
    mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);//APA^T
    float BBT[2][2];
    float tran_B_theta[1][2] = {};
    mat_tran(B_theta[0], tran_B_theta[0], 2, 1);//B^T
    mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);//BB^T
    float BUBT[2][2] = {};
    mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);//BUB^T
    mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);//APA^T+BUB^T
}

int main(int argc, char **argv) {
    pi=pigpio_start(NULL,NULL);

    //I2C setting
    int bus_acc = i2c_open(pi, 1, ACC_ADDR, 0);
    int bus_gyr = i2c_open(pi, 1, GYR_ADDR, 0);
    acc_init(bus_acc); 
    gyr_init(bus_gyr);

    //Kalman filter (angle) initialization
    //initial value of theta_data_predict
    theta_data_predict[0][0] = 0;
    theta_data_predict[1][0] = theta_dot_mean;
    
    //initial value of P_theta_predict
    P_theta_predict[0][0] = 1;
    P_theta_predict[0][1] = 0;
    P_theta_predict[1][0] = 0;
    P_theta_predict[1][1] = theta_dot_variance;

    ros::init(argc, argv, "bmx055_kalman_publisher");
    ros::NodeHandle nh;
    ros::Publisher var_pub1 = nh.advertise<std_msgs::Float64>("c_var_topic", 10);
    ros::Publisher var_pub2 = nh.advertise<std_msgs::Float64>("cdot_var_topic", 10);
    ros::Publisher imu_pub1 = nh.advertise<std_msgs::Float64>("theta1_topic", 10);
    ros::Publisher imu_pub2 = nh.advertise<std_msgs::Float64>("theta1dot_temp_topic", 10);
    
    ros::Rate rate(400);  // パブリッシュの頻度を設定 (400 Hz)

    float theta1dot_temp;
    int count=0;
    while (ros::ok()) {
        std_msgs::Float64 c_var_msg;
        c_var_msg.data = theta_variance;
        var_pub1.publish(c_var_msg);
        std_msgs::Float64 cdot_var_msg;
        cdot_var_msg.data = theta_dot_variance;
        var_pub2.publish(cdot_var_msg);

        update_theta(bus_acc, bus_gyr);
        
        theta1dot_temp = get_gyr_data(bus_gyr);
        std_msgs::Float64 imu_msg1;
        imu_msg1.data = theta_data[0][0];
        imu_pub1.publish(imu_msg1);
        std_msgs::Float64 imu_msg2;
        imu_msg2.data = (theta1dot_temp - theta_data[1][0]) * 3.14f/180;
        imu_pub2.publish(imu_msg2);

        count += 1;
        if (count % 100 == 0){
            ROS_INFO("Publish theta1 from BMX055: %f", imu_msg1.data);
            ROS_INFO("Publish theta1dot_temp from BMX055: %f", imu_msg2.data);
        }
        
        
        rate.sleep();
    }

    i2c_close(pi,bus_acc);
    i2c_close(pi,bus_gyr);
    pigpio_stop(pi);
    return 0;
}
