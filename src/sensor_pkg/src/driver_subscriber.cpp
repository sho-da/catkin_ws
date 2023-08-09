#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <unistd.h>

//=========================================================
//Kalman filter (for all system estimation) variables
//State vector
//[[theta1(rad)], [theta1_dot(rad/s)], [theta2(rad)]. [theta2_dot(rad/s)]]
float x_data_predict[4][1];
float x_data[4][1];
//Covariance matrix
float P_x_predict[4][4];
float P_x[4][4];
//"A" of the state equation (update freq = 100 Hz)
float A_x[4][4] = {
{1.00210e+00,1.00070e-02,0.00000e+00,3.86060e-05},
{4.20288e-01,1.00210e+00,0.00000e+00,7.65676e-03},
{-1.15751e-03,-3.87467e-06,1.00000e+00,9.74129e-03},
{-2.29569e-01,-1.15751e-03,0.00000e+00,9.48707e-01}
};
//"B" of the state equation (update freq = 100 Hz)
float B_x[4][1] = {
{-2.70805e-04},
{-5.37090e-02},
{1.81472e-03},
{3.59797e-01}
};
//"C" of the state equation (update freq = 100 Hz)
float C_x[4][4] = {
{1, 0, 0, 0},
{0, 1, 0, 0},
{0, 0, 1, 0},
{0, 0, 0, 1}
};
//measurement noise
float measure_variance_mat[4][4];
//System noise
float voltage_error = 0.01; //volt
float voltage_variance = voltage_error * voltage_error;

void Kalman_main(float y[4][1]){
     //---------------------------------------
    //Kalman Filter (all system)
    //---------------------------------------
    //measurement data
    y[0][0] = theta_data[0][0] * 3.14f/180;
    theta1_dot_temp = get_gyro_data();
    y[1][0] = ( theta1_dot_temp - theta_data[1][0]) * 3.14f/180;
    y[2][0] = encoder_value * (2*3.14f)/(4*rotary_encoder_resolution);
    y[3][0] = (y[2][0] - pre_theta2)/feedback_rate;    
                
    //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    mat_tran(C_x[0], tran_C_x[0], 4, 4);//C^T
    mat_mul(P_x_predict[0], tran_C_x[0], P_CT[0], 4, 4, 4, 4);//P'C^T
    mat_mul(C_x[0], P_CT[0], G_temp1[0], 4, 4, 4, 4);//CPC^T
    mat_add(G_temp1[0], measure_variance_mat[0], G_temp2[0], 4, 4);//W+CP'C^T
    mat_inv(G_temp2[0], G_temp2_inv[0], 4, 4);//(W+CP'C^T)^-1
    mat_mul(P_CT[0], G_temp2_inv[0], G[0], 4, 4, 4, 4); //P'C^T(W+CP'C^T)^-1
      
    //x_data estimation: x = x'+G(y-Cx')
    mat_mul(C_x[0], x_data_predict[0], C_x_x[0], 4, 4, 4, 1);//Cx'
    mat_sub(y[0], C_x_x[0], delta_y[0], 4, 1);//y-Cx'
    mat_mul(G[0], delta_y[0], delta_x[0], 4, 4, 4, 1);//G(y-Cx')
    mat_add(x_data_predict[0], delta_x[0], x_data[0], 4, 1);//x'+G(y-Cx')
        
    //calculate covariance matrix: P=(I-GC)P'
    mat_mul(G[0], C_x[0], GC[0], 4, 4, 4, 4);//GC
    mat_sub(I4[0], GC[0], I4_GC[0], 4, 4);//I-GC
    mat_mul(I4_GC[0], P_x_predict[0], P_x[0], 4, 4, 4, 4);//(I-GC)P'
}

int main(int argc, char** argv)
{
    //-------------------------------------------
    //Kalman filter (all system) variables
    //------------------------------------------- 
    //variable for measurement data
    float y[4][1];
    
    //variables for Kalman gain calculation
    float theta1_dot_temp;
    float tran_C_x[4][4];
    float P_CT[4][4];
    float G_temp1[4][4];    
    float G_temp2[4][4];
    float G_temp2_inv[4][4];
    float G[4][4];

    //variables for x_hat estimation
    float C_x_x[4][1];
    float delta_y[4][1];
    float delta_x[4][1];

    //variables for covariance matrix calculation
    float GC[4][4];
    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    float I4_GC[4][4];
    
    //variables for x prediction
    float Vin;
    float A_x_x[4][1];
    float B_x_Vin[4][1];
   
    //variables for covariance prediction
    float tran_A_x[4][4];
    float AP[4][4]; 
    float APAT[4][4];
    float BBT[4][4];
    float tran_B_x[1][4];
    float BUBT[4][4];

    //-------------------------------------------  
    //Kalman filter (all system) initialization
    //-------------------------------------------
    //initial value of x_data_predict
    for(int i=0; i<4; i++)
    {
        x_data_predict[i][0] = 0;
    }
    
    //initial value of P_x_predict
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            P_x_predict[i][j] = 0;    
        } 
    }
    for(int i=0; i<4; i++)
    {
        P_x_predict[i][i] = 1e-4;   
    }
    
    //measurement noise matrix
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            measure_variance_mat[i][j] = 0;    
        } 
    }
    float deg_rad_coeff = (3.14*3.14)/(180*180);
    measure_variance_mat[0][0] = theta_variance * deg_rad_coeff;
    measure_variance_mat[1][1] = theta_dot_variance * deg_rad_coeff;
    float encoder_error = 0.1f*2*3.14f/(4*rotary_encoder_resolution);
    measure_variance_mat[2][2] = encoder_error * encoder_error;
    float encoder_rate_error = encoder_error / feedback_rate;
    measure_variance_mat[3][3] = encoder_rate_error * encoder_rate_error;

    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub1 = nh.subscribe("theta1_topic", 1, callback_1mu1);
    ros::Subscriber imu_sub2 = nh.subscribe("theta1dot_temp_topic", 1, callback_imu2);
    ros::Subscriber enc_sub  = nh.subscribe("theta2_topic", 1, callback_enc);

    ros::Rate rate(1429);  // サブスクライブの頻度を設定 (1429 Hz) 7oo usec

    float y[4][1];

    while (ros::ok()) {
        ros::spinOnce();
        y[0][0] = theta_data[0][0] * 3.14f/180;
        y[1][0] = ( theta1_dot_temp - theta_data[1][0]) * 3.14f/180;
        y[2][0] = encoder_value * (2*3.14f)/(4*rotary_encoder_resolution);
        y[3][0] = (y[2][0] - pre_theta2)/feedback_rate; 
        pre_theta2 = y[2][0];
        Kalman_main(y);
        std_msgs::Float64 imu_msg;
        imu_msg.data = x_data;
        imu_pub.publish(imu_msg);
        ROS_INFO("Published from Kalman_main: %d", imu_msg.data);
        rate.sleep();
    }
}

