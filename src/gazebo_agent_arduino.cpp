#include <ros/package.h>
#include "ros/ros.h" // ROS Default Header File

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"

#include "hero_msgs/hero_agent_sensor.h"
#include "hero_msgs/hero_usbl_cont.h"
#include "hero_msgs/hero_agent_state.h"
#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"
#include "hero_msgs/hero_xy_cont.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>

#include <ignition/math.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define CAMERA_HEIGHT 492
#define CAMERA_WIDTH 768

#define _USE_MATH_DEFINES
#include <cmath>

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

namespace Eigen
{
    /// \brief Definition of a 6x6 Eigen matrix
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

    /// \brief Definition of a 6 element Eigen vector
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

}

// Pi
#define PI 3.14159265359

/// \brief Conversion of a string to a double vector
inline std::vector<double> Str2Vector(std::string _input)
{
    std::vector<double> output;
    std::string buf;
    std::stringstream ss(_input);
    while (ss >> buf)
        output.push_back(std::stod(buf));
    return output;
}

/// \brief Returns the cross product operator matrix
/// for Eigen vectors
inline Eigen::Matrix3d CrossProductOperator(Eigen::Vector3d _x)
{
    Eigen::Matrix3d output;
    output << 0.0, -_x[2], _x[1], _x[2], 0.0, -_x[0], -_x[1], _x[0], 0.0;
    return output;
}

/// \brief Returns the cross product operator matrix
/// for Gazebo vectors
inline Eigen::Matrix3d CrossProductOperator(ignition::math::Vector3d _x)
{
    Eigen::Matrix3d output;
    output << 0.0, -_x[2], _x[1], _x[2], 0.0, -_x[0], -_x[1], _x[0], 0.0;
    return output;
}

inline Eigen::Vector3d ToEigen(const ignition::math::Vector3d &_x)
{
    return Eigen::Vector3d(_x[0], _x[1], _x[2]);
}

inline Eigen::Matrix3d ToEigen(const ignition::math::Matrix3d &_x)
{
    Eigen::Matrix3d m;
    m << _x(0, 0), _x(0, 1), _x(0, 2),
        _x(1, 0), _x(1, 1), _x(1, 2),
        _x(2, 0), _x(2, 1), _x(2, 2);
    return m;
}

inline Eigen::Vector6d EigenStack(const ignition::math::Vector3d &_x,
                                  const ignition::math::Vector3d &_y)
{
    Eigen::Vector3d xe = ToEigen(_x);
    Eigen::Vector3d ye = ToEigen(_y);
    Eigen::Vector6d out;
    out << xe, ye;
    return out;
}

inline ignition::math::Vector3d Vec3dToGazebo(const Eigen::Vector3d &_x)
{
    return ignition::math::Vector3d(_x[0], _x[1], _x[2]);
}

inline ignition::math::Matrix3d Mat3dToGazebo(const Eigen::Matrix3d &_x)
{
    return ignition::math::Matrix3d(_x(0, 0), _x(0, 1), _x(0, 2),
                                    _x(1, 0), _x(1, 1), _x(1, 2),
                                    _x(2, 0), _x(2, 1), _x(2, 2));
}

struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

using namespace std;

ofstream fout;

int real_time = 0;

ros::Publisher *pub_state_ptr;
ros::Publisher *pub_sensors_ptr;

ros::Publisher *pub_joint1_ptr;
ros::Publisher *pub_joint2_ptr;

ros::Publisher *pub_thruster0_ptr;
ros::Publisher *pub_thruster1_ptr;
ros::Publisher *pub_thruster2_ptr;
ros::Publisher *pub_thruster3_ptr;
ros::Publisher *pub_thruster4_ptr;
ros::Publisher *pub_thruster5_ptr;

int cont_target = 0;
// for AHRS_3DM_GX5----------------------
volatile int serial_count = 0;
volatile uint8_t inputString[100];    // a String to hold incoming data
volatile bool stringComplete = false; // whether the string is complete

volatile uint8_t imu_check = 0;

volatile union
{
    float real;
    uint32_t base;
} u_data;

volatile float roll, pitch, yaw, yaw_temp;
volatile float acc_roll, acc_pitch, acc_yaw;
volatile float lin_vel_x, lin_vel_y, lin_vel_z;
volatile float pre_acc_roll, pre_acc_pitch, pre_acc_yaw;
volatile float a_roll, a_pitch, a_yaw;

volatile float acc_x = 0, acc_y = 0, acc_z = 0;
volatile float sum_acc_x = 0, sum_acc_y = 0, sum_acc_z = 0;

volatile uint8_t ahrs_valid1, ahrs_valid2, ahrs_valid3;

volatile uint8_t yaw_calid_command = 0;
volatile char yaw_calib = 0;
volatile float pre_yaw = 0;
volatile float yaw_calib2 = 0;
//--------------------------------------

// for Trusters-------------------------
volatile int pwm_m0 = 1500, pid_pwm_m0;
volatile int pwm_m1 = 1500, pid_pwm_m1;
volatile int pwm_m2 = 1500, pid_pwm_m2;
volatile int pwm_m3 = 1500, pid_pwm_m3;
volatile int pwm_m4 = 1500, pid_pwm_m4;
volatile int pwm_m5 = 1500, pid_pwm_m5;
//--------------------------------------

// for depth sensor---------------------
double depth;
double pre_depth = 0;
//--------------------------------------

// for Control yaw and depth---------------------
volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;

volatile double T = 0.004;      // Loop time.
volatile double T_depth = 0.04; // Loop time.

volatile int throttle = 50;

volatile double P_angle_gain_yaw = 50;
volatile double P_gain_yaw = 15;
volatile double I_gain_yaw = 1;
volatile double D_gain_yaw = 0.5;

double P_gain_depth = 1000.0;
double I_gain_depth = 10.0;
double D_gain_depth = 100.0;

volatile double error_yaw;
volatile double error_pid_yaw, error_pid_yaw1;
volatile double P_angle_pid_yaw;
volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
volatile double desired_angle_yaw = -1.470796;

double error_pid_depth, error_pid_depth1;
double P_angle_pid_depth;
double P_depth, I_depth, D_depth, PID_depth;
double desired_angle_depth = 55;
//--------------------------------------

// for Control position---
volatile uint8_t cont_direc = 0;
volatile int move_speed = 10;
volatile int added_move_speed = 0;

volatile float object_x = 0, object_y = 0;
volatile uint8_t object_valid = 0;
volatile double costheta = 0;
volatile double sintheta = 0;

volatile double usbl_x = 0;
volatile double usbl_y = 0;
volatile double usbl_z = 0;

Eigen::Vector6d _vel;
Eigen::Vector6d _acc_vel;
Eigen::Vector6d dy_result;

Eigen::Vector6d error_position;
Eigen::Vector6d pre_error_position;

Eigen::Vector6d error_velocity;
Eigen::Vector6d pre_error_velocity;

Eigen::Vector6d Drag_6d;

ignition::math::Pose3d agent_pose;

double target_usbl_x = 0;
double target_usbl_y = 0;
double target_usbl_z = 0;
//-----------

// for Control taesik---
volatile uint8_t cont_taesik = 0;

volatile uint8_t cont_model = 0;
volatile uint8_t cont_drag = 0;
volatile uint8_t cont_fout = 0;
volatile uint8_t cont_tank_sim = 0;
//-----------

int yaw_stage = 0;

int target_end_count = 0;

void PID_control_yaw();
void PID_control_depth();
void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2);

void Control_taesik();

float target_x_position = 34.154;
float target_y_position = 27;
float target_z_position = 52.41;

float real_x_position = 0;
float real_y_position = 0;
float real_z_position = 0;

float real_target_x_position = 0;
float real_target_y_position = 0;
float real_target_z_position = 0;

float target_yaw = 0;

float Target_target_position = 0.5;

void messageCont_YOLO(const hero_msgs::hero_xy_cont::ConstPtr &yolo_msg)
{
    object_x = yolo_msg->TARGET_X;
    object_y = yolo_msg->TARGET_Y;
    object_valid = yolo_msg->VALID;
}

void messageCommand(const std_msgs::Int8::ConstPtr &command_msg)
{
    int Command = command_msg->data;
    std_msgs::Float64 joint_msg;

    if (Command == 'q')
    {
        cont_direc = 0;
    }
    else if (Command == 'w') // backward
    {
        cont_direc = 1;
    }
    else if (Command == 's') // forward
    {
        cont_direc = 2;
    }
    else if (Command == 'a') // right
    {
        cont_direc = 3;
    }
    else if (Command == 'd') // left
    {
        cont_direc = 4;
    }
    else if (Command == 'n')    //init yaw sensor data
    {
        desired_angle_yaw = 0;
        yaw_calid_command = 1;
    }
    else if (Command == '5') // yolo based control
    {
        cont_direc = 5;
    }
    else if (Command == 'z') // left
    {
        move_speed += 10;
    }
    else if (Command == 'x') // right
    {
        move_speed -= 10;
    }
    else if (Command == 'e')
    {
        // digitalWrite(RELAY, HIGH);
        // delay(5000);
    }
    else if (Command == 't')
    {
        // digitalWrite(RELAY, LOW);
    }
    else if (Command == 'r')
    {
        // digitalWrite(LED_SIG, HIGH);
    }
    else if (Command == 'f')
    {
        // digitalWrite(LED_SIG, LOW);
    }
    else if (Command == 'c')
    {
        // OCR1A = 450; //open gripper
        joint_msg.data = -2;
        pub_joint1_ptr->publish(joint_msg);
        joint_msg.data = 2;
        pub_joint2_ptr->publish(joint_msg);
    }
    else if (Command == 'v')
    {
        // OCR1A = 350; //stop gripper
        joint_msg.data = 0;
        pub_joint1_ptr->publish(joint_msg);
        joint_msg.data = 0;
        pub_joint2_ptr->publish(joint_msg);
    }
    else if (Command == 'b')
    {
        // OCR1A = 300; //close gripper
        joint_msg.data = 2;
        pub_joint1_ptr->publish(joint_msg);
        joint_msg.data = -2;
        pub_joint2_ptr->publish(joint_msg);
    }
    else if (Command == 'g')
    {
        pwm_m0 = 1500;
        pwm_m1 = 1500;
        pwm_m2 = 1500;
        pwm_m3 = 1500;
        pwm_m4 = 1500;
        pwm_m5 = 1500;

        esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
        esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
    }
    else if (Command == 'y')
    {
        cont_yaw_on = 1;
    }
    else if (Command == 'h')
    {
        cont_yaw_on = 0;
    }
    else if (Command == 'u')
    {
        throttle += 10;
    }
    else if (Command == 'j')
    {
        throttle -= 10;
    }
    else if (Command == 'i')
    {
        desired_angle_yaw += 0.1;
    }
    else if (Command == 'k')
    {
        desired_angle_yaw -= 0.1;
    }
    else if (Command == 'o')
    {
        desired_angle_depth += 0.1;
    }
    else if (Command == 'l')
    {
        desired_angle_depth -= 0.1;
    }
    else if (Command == '.')
    {
        desired_angle_depth += 0.01;
    }
    else if (Command == ',')
    {
        desired_angle_depth -= 0.01;
    }
    else if (Command == 'p')
    {
        cont_depth_on = 1;
    }
    else if (Command == ';')
    {
        cont_depth_on = 0;
    }
    else if (Command == '/')
    {
        desired_angle_yaw = yaw;
        desired_angle_depth = depth;
    }
    hero_msgs::hero_agent_state state_msg;

    state_msg.Yaw = yaw;
    state_msg.Target_yaw = desired_angle_yaw;
    state_msg.Throttle = throttle;
    state_msg.Valid_yaw = inputString[19];
    state_msg.Depth = depth;
    state_msg.Target_depth = desired_angle_depth;
    state_msg.Move_speed = move_speed;
    state_msg.State_addit = cont_target;

    // state_msg.Move_speed = cont_drag;

    pub_state_ptr->publish(state_msg);
}

//--------------------------------------------
volatile int check_hz = 0;

void msgCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
{
    check_hz++;

    Quaternion Q_angles;
    EulerAngles E_angles;

    Q_angles.x = msg->orientation.x;
    Q_angles.y = msg->orientation.y;
    Q_angles.z = msg->orientation.z;
    Q_angles.w = msg->orientation.w;

    E_angles = ToEulerAngles(Q_angles);

    roll = E_angles.roll;
    pitch = E_angles.pitch;
    yaw = E_angles.yaw;
    yaw_temp = E_angles.yaw;

    acc_roll = msg->angular_velocity.x;
    acc_pitch = msg->angular_velocity.y;
    acc_yaw = msg->angular_velocity.z;

    a_roll = (acc_roll - pre_acc_roll) * 125;
    a_pitch = (acc_pitch - pre_acc_pitch) * 125;
    a_yaw = (acc_yaw - pre_acc_yaw) * 125;

    pre_acc_roll = acc_roll;
    pre_acc_pitch = acc_pitch;
    pre_acc_yaw = acc_yaw;

    acc_x = msg->linear_acceleration.x;
    acc_y = msg->linear_acceleration.y;
    acc_z = msg->linear_acceleration.z;

    sum_acc_x += acc_x / 125;
    sum_acc_y += acc_y / 125;
    sum_acc_z += (acc_z - 9.80665) / 125;

    ahrs_valid1 = 1;
    ahrs_valid2 = 1;
    ahrs_valid3 = 1;

    yaw -= yaw_calib2;

    if (yaw_calid_command == 1)
    {
        yaw_calid_command = 0;
        yaw_calib2 = yaw_temp;
        pre_yaw = 0;
        yaw = 0;
        yaw_calib = 0;
    }

    if (yaw - pre_yaw > 4.712388)
        yaw_calib--;
    else if (yaw - pre_yaw < -4.712388)
        yaw_calib++;
    pre_yaw = yaw;
    yaw += 3.141592 * yaw_calib * 2;
    //
    // yaw control loop
    if (cont_yaw_on == 1)
        PID_control_yaw();
}

void msgCallback_depth(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    depth = ((msg->fluid_pressure / 101.325) - 1) * 10;

    lin_vel_z = (depth - pre_depth) * 10;

    pre_depth = depth;
    if (cont_depth_on == 1)
        PID_control_depth();
}

int main(int argc, char **argv) // Node Main Function
{

    cout << "Start!" << endl;
    int count = 0;

    ros::init(argc, argv, "gazebo_agent_arduino"); // Initializes Node Name
    ros::NodeHandle nh;

    ros::Publisher pub_state = nh.advertise<hero_msgs::hero_agent_state>("/hero_agent/state", 100);
    ros::Publisher pub_sensors = nh.advertise<hero_msgs::hero_agent_sensor>("/hero_agent/sensors", 100);

    ros::Publisher pub_joint1 = nh.advertise<std_msgs::Float64>("/hero_agent/joint1_position_controller/command", 100);
    ros::Publisher pub_joint2 = nh.advertise<std_msgs::Float64>("/hero_agent/joint2_position_controller/command", 100);

    ros::Publisher pub_thruster0 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/0/input", 100);
    ros::Publisher pub_thruster1 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/1/input", 100);
    ros::Publisher pub_thruster2 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/2/input", 100);
    ros::Publisher pub_thruster3 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/3/input", 100);
    ros::Publisher pub_thruster4 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/4/input", 100);
    ros::Publisher pub_thruster5 = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/hero_agent/thrusters/5/input", 100);

    pub_state_ptr = &pub_state;
    pub_sensors_ptr = &pub_sensors;

    pub_joint1_ptr = &pub_joint1;
    pub_joint2_ptr = &pub_joint2;

    pub_thruster0_ptr = &pub_thruster0;
    pub_thruster1_ptr = &pub_thruster1;
    pub_thruster2_ptr = &pub_thruster2;
    pub_thruster3_ptr = &pub_thruster3;
    pub_thruster4_ptr = &pub_thruster4;
    pub_thruster5_ptr = &pub_thruster5;

    ros::Rate loop_rate(25); // Hz

    ros::Subscriber sub_imu =
        nh.subscribe("/hero_agent/imu", 100, msgCallback_imu);

    ros::Subscriber sub_depth =
        nh.subscribe("/hero_agent/pressure", 100, msgCallback_depth);

    ros::Subscriber sub_command = nh.subscribe("/hero_agent/command", 100, messageCommand);
    ros::Subscriber sub_yolo_msg = nh.subscribe("/hero_agent/xy", 100, messageCont_YOLO);
 
    cout << "Start!" << endl;
    while (ros::ok())
    {

        hero_msgs::hero_agent_sensor sensors_msg;
        sensors_msg.ROLL = roll;
        sensors_msg.PITCH = pitch;
        sensors_msg.YAW = yaw;
        sensors_msg.DEPTH = depth;

        pub_sensors.publish(sensors_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::spin(); // Increase count variable by one

    return 0;
}

void PID_control_yaw()
{
    error_yaw = desired_angle_yaw - yaw;            // angle def
    P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; // angle def + outer P control

    error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

    P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
    D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
    I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
    I_yaw = constrain(I_yaw, -100.0, 100.0);                   // I control must be limited to prevent being jerk.

    PID_yaw = P_yaw + D_yaw + I_yaw;

    if (cont_direc == 0) // stop
    {
        pwm_m1 = -PID_yaw - throttle + 1500;
        pwm_m2 = -PID_yaw + throttle + 1500;
        pwm_m4 = -PID_yaw - throttle + 1500;
        pwm_m5 = -PID_yaw + throttle + 1500;
    }
    else if (cont_direc == 1) // forward
    {
        pwm_m1 = -PID_yaw - throttle + 1500 + move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 + move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 - move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 - move_speed;
    }
    else if (cont_direc == 2) // backward
    {
        pwm_m1 = -PID_yaw - throttle + 1500 - move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 - move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 + move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 + move_speed;
    }
    else if (cont_direc == 3) // left
    {
        pwm_m1 = -PID_yaw - throttle + 1500 + move_speed - added_move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 - move_speed - added_move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 - move_speed + added_move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 + move_speed + added_move_speed;
    }
    else if (cont_direc == 4) // right
    {
        pwm_m1 = -PID_yaw - throttle + 1500 - move_speed - added_move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 + move_speed - added_move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 + move_speed + added_move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 - move_speed + added_move_speed;
    }
    else if (cont_direc == 5) // concon
    {
        // costheta = (object_x-320)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));
        // sintheta = (240-object_y)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));

        costheta = (object_x - CAMERA_WIDTH / 2) / 300;
        sintheta = (CAMERA_HEIGHT / 2 - object_y) / 300;

        pwm_m1 = -PID_yaw - throttle + 1500 - (float)move_speed * (costheta - sintheta);
        pwm_m2 = -PID_yaw + throttle + 1500 - (float)move_speed * (-costheta - sintheta);
        pwm_m4 = -PID_yaw - throttle + 1500 - (float)move_speed * (-costheta + sintheta);
        pwm_m5 = -PID_yaw + throttle + 1500 - (float)move_speed * (costheta + sintheta);
    }

    // pwm_m1 = constrain(pwm_m1, 1100, 1500);
    // pwm_m2 = constrain(pwm_m2, 1500, 1900);
    // pwm_m4 = constrain(pwm_m4, 1100, 1500);
    // pwm_m5 = constrain(pwm_m5, 1500, 1900);

    pwm_m1 = constrain(pwm_m1, 1100, 1900);
    pwm_m2 = constrain(pwm_m2, 1100, 1900);
    pwm_m4 = constrain(pwm_m4, 1100, 1900);
    pwm_m5 = constrain(pwm_m5, 1100, 1900);

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

    error_pid_yaw1 = error_pid_yaw;
}

void PID_control_depth()
{
    error_pid_depth = desired_angle_depth - depth;
    P_depth = error_pid_depth * P_gain_depth;                                // Inner P control
    D_depth = (error_pid_depth - error_pid_depth1) / T_depth * D_gain_depth; // Inner D control
    I_depth += (error_pid_depth)*T_depth * I_gain_depth;                     // Inner I control
    I_depth = constrain(I_depth, -100, 100);                                 // I control must be limited to prevent being jerk.

    PID_depth = P_depth + D_depth + I_depth;

    pwm_m0 = -PID_depth + 1500;
    pwm_m3 = -PID_depth + 1500;

    pwm_m0 = constrain(pwm_m0, 1100, 1900);
    pwm_m3 = constrain(pwm_m3, 1100, 1900);

    // esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    // esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

    error_pid_depth1 = error_pid_depth;
}

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2)
{
    uuv_gazebo_ros_plugins_msgs::FloatStamped thruster_msg;
    if (ID == 0x02)
    {
        thruster_msg.data = (pwm0 - 1500) * 1;
        pub_thruster4_ptr->publish(thruster_msg);

        thruster_msg.data = (pwm1 - 1500) * -1;
        pub_thruster0_ptr->publish(thruster_msg);

        thruster_msg.data = (pwm2 - 1500) * -1;
        pub_thruster1_ptr->publish(thruster_msg);
    }
    else if (ID == 0x03)
    {
        thruster_msg.data = (pwm0 - 1500) * 1;
        pub_thruster5_ptr->publish(thruster_msg);

        thruster_msg.data = (pwm1 - 1500) * -1;
        pub_thruster3_ptr->publish(thruster_msg);

        thruster_msg.data = (pwm2 - 1500) * -1;
        pub_thruster2_ptr->publish(thruster_msg);
    }
}
