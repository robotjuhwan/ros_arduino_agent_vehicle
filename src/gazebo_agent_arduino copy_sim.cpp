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

ignition::math::Vector3d flowVel;
Eigen::Matrix6d Ma;
Eigen::Matrix6d Ca;
Eigen::Matrix6d D;

Eigen::Matrix6d Kp;
Eigen::Matrix6d Kd;

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
int ccount = 0;

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

int cont_recovery = 0;
int cont_target = 0;
//for AHRS_3DM_GX5----------------------
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

//for Trusters-------------------------
volatile int pwm_m0 = 1500, pid_pwm_m0;
volatile int pwm_m1 = 1500, pid_pwm_m1;
volatile int pwm_m2 = 1500, pid_pwm_m2;
volatile int pwm_m3 = 1500, pid_pwm_m3;
volatile int pwm_m4 = 1500, pid_pwm_m4;
volatile int pwm_m5 = 1500, pid_pwm_m5;
//--------------------------------------

//for depth sensor---------------------
double depth;
double pre_depth = 0;
//--------------------------------------

//for Control yaw and depth---------------------
volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;

volatile double T = 0.004;      // Loop time.
volatile double T_depth = 0.04; // Loop time.

volatile int throttle = 50;

volatile double P_angle_gain_yaw = 6.5;
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
double desired_angle_depth = 52.41;
//--------------------------------------

//for Control position---
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

//for Control taesik---
volatile uint8_t cont_taesik = 0;

volatile uint8_t cont_model = 0;
volatile uint8_t cont_drag = 0;
volatile uint8_t cont_fout = 0;
volatile uint8_t cont_tank_sim = 0;
//-----------

float ipcam_qr_x = 0;
float ipcam_qr_y = 0;
float ipcam_qr_z = 0;
float ipcam_qr_yaw = 0;
int ipcam_qr_valid = 0;

int yaw_stage = 0;

int target_end_count = 0;

void PID_control_yaw();
void PID_control_depth();
void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2);
void recovery_cont();
void target_cont();
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

void messageCont_IPCAM(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg::ConstPtr &ipcam_msg)
{
    ipcam_qr_x = ipcam_msg->T_X;
    ipcam_qr_y = ipcam_msg->T_Y;
    ipcam_qr_z = ipcam_msg->T_Z;
    ipcam_qr_yaw = ipcam_msg->T_YAW;
    ipcam_qr_valid = ipcam_msg->T_valid;
}

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

    if (Command == '[')
    {
        dy_result(2, 0) = 0;
        if (cont_taesik == 1)
        {
            cont_taesik = 0;
            cout << "cont_taesik 0" << endl;
        }
        else
        {
            cont_taesik = 1;
            cout << "cont_taesik 1" << endl;
        }
    }
    else if (Command == 'q')
    {
        real_target_x_position = 0.00146091;
        real_target_y_position = 0.0819414;
        real_target_z_position = -0.00168018;

        cont_direc = 0;
    }
    else if (Command == 'w') //backward
    {
        real_target_x_position = 0.00146091;
        real_target_y_position = 0.0819414;
        real_target_z_position = -0.2;

        cont_direc = 1;
    }
    else if (Command == 's') //forward
    {
        target_x_position = 34.154 - Target_target_position;
        target_y_position = 27;
        target_z_position = 52.41;
        target_yaw = 0;

        cont_direc = 2;
    }
    else if (Command == 'a') //right
    {
        target_x_position = 34.154 - Target_target_position;
        target_y_position = 27 - Target_target_position;
        target_z_position = 52.41;
        target_yaw = 0;

        cont_direc = 3;
    }
    else if (Command == 'd') //left
    {
        target_x_position = 34.154 - Target_target_position;
        target_y_position = 27;
        target_z_position = 52.41 + Target_target_position;

        cont_direc = 4;
    }
    else if (Command == ']') //down
    {
        if (cont_drag == 1)
        {
            cont_drag = 0;
            cout << "cont_drag 0" << endl;
        }
        else
        {
            cont_drag = 1;
            cout << "cont_drag 1" << endl;
        }
    }
    else if (Command == '\\') //up
    {
        if (cont_fout == 1)
        {
            cont_fout = 0;
        }
        else
        {
            cont_fout = 1;
        }
    }
    ////////////////////////////////////////////////
    else if (Command == 'f') //left
    {
        Target_target_position += 0.01;
        cout << "Target_target_position " << Target_target_position << endl;
    }
    else if (Command == 'g') //right
    {
        Target_target_position -= 0.01;
        cout << "Target_target_position " << Target_target_position << endl;
    }
    else if (Command == 'z') //left
    {
        Kp(0, 0) += 0.1;
        Kp(1, 1) += 0.1;
    }
    else if (Command == 'x') //right
    {
        Kp(0, 0) -= 0.1;
        Kp(1, 1) -= 0.1;
    }
    else if (Command == 'c') //left
    {
        Kp(2, 2) += 0.1;
    }
    else if (Command == 'v') //right
    {
        Kp(2, 2) -= 0.1;
    }
    else if (Command == 'b') //left
    {
        Kd(0, 0) += 0.1;
        Kd(1, 1) += 0.1;
    }
    else if (Command == 'n') //right
    {
        Kd(0, 0) -= 0.1;
        Kd(1, 1) -= 0.1;
    }
    else if (Command == 'm') //left
    {
        Kd(2, 2) += 0.1;
    }
    else if (Command == ',') //right
    {
        Kd(2, 2) -= 0.1;
    }
    else if (Command == 'p') //up
    {
        if (cont_model == 1)
        {
            cout << "cont_model 0" << endl;
            cont_model = 0;
        }
        else
        {
            cout << "cont_model 1" << endl;
            cont_model = 1;
        }
    }
    else if (Command == 'o') //up
    {
        if (cont_tank_sim == 1)
        {
            cout << "cont_tank_sim 0" << endl;
            cont_tank_sim = 0;
            fout.close();
        }
        else
        {
            cout << "cont_tank_sim 1" << endl;
            cont_tank_sim = 1;
            real_time = 0;
            fout.open("/home/superhero/catkin_ws/sim_output.txt");
        }
    }

    //////////////////////////////////////
    else if (Command == 'n')
    {
        yaw_calid_command = 1;
    }
    else if (Command == '5') //concon
    {
        cont_direc = 5;
    }
    else if (Command == '1') //concon
    {
        target_usbl_x = 1.5;
        target_usbl_y = 0;
        target_usbl_z = 0.08;
        cont_target = 1;
        target_end_count = 0;
    }
    /*
    else if (Command == '2') //concon
    {
        cont_recovery = 1;
    }
    else if (Command == '3') //concon
    {
        cont_recovery = 0;
        yaw_stage = 0;
    }*/
    else if (Command == '2') //concon
    {
        target_usbl_x = 3.5;
        target_usbl_y = 0;
        target_usbl_z = 1.95;
        cont_target = 1;
        target_end_count = 0;
    }
    else if (Command == '3') //concon
    {
        cont_target = 0;
    }
    else if (Command == '4') //concon
    {
        target_usbl_x = 0.9;
        target_usbl_y = 0;
        target_usbl_z = 0.08;
        cont_target = 1;
        target_end_count = 0;
    }
    else if (Command == 'z') //left
    {
        move_speed += 10;
    }
    else if (Command == 'x') //right
    {
        move_speed -= 10;
    }
    else if (Command == 'e')
    {
        //digitalWrite(RELAY, HIGH);
        //delay(5000);
    }
    else if (Command == 't')
    {
        //digitalWrite(RELAY, LOW);
    }
    else if (Command == 'r')
    {
        //digitalWrite(LED_SIG, HIGH);
    }
    else if (Command == 'f')
    {
        //digitalWrite(LED_SIG, LOW);
    }
    else if (Command == 'c')
    {

        //OCR1A = 450; //open gripper
        joint_msg.data = -2;
        pub_joint1_ptr->publish(joint_msg);
        joint_msg.data = 2;
        pub_joint2_ptr->publish(joint_msg);
    }
    else if (Command == 'v')
    {
        //OCR1A = 350; //stop gripper
        joint_msg.data = 0;
        pub_joint1_ptr->publish(joint_msg);
        joint_msg.data = 0;
        pub_joint2_ptr->publish(joint_msg);
    }
    else if (Command == 'b')
    {
        //OCR1A = 300; //close gripper
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

        Kp(0, 0) += 1;
        Kp(1, 1) += 1;
    }
    else if (Command == ';')
    {
        cont_depth_on = 0;

        Kp(0, 0) -= 1;
        Kp(1, 1) -= 1;
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

    state_msg.Move_speed = cont_drag;

    pub_state_ptr->publish(state_msg);
    ccount = 0;
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
    //yaw control loop
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

geometry_msgs::Point cy_position;
geometry_msgs::Point ag_position;

geometry_msgs::Point pre_ag_position;
ignition::math::Vector3d F_drag;
float pre_agent_z = 0;

EulerAngles cy_angles;

void msgCallback_cypose(const nav_msgs::Odometry::ConstPtr &msg)
{
    cy_position = msg->pose.pose.position;

    Quaternion Q_angles;

    Q_angles.x = msg->pose.pose.orientation.x;
    Q_angles.y = msg->pose.pose.orientation.y;
    Q_angles.z = msg->pose.pose.orientation.z;
    Q_angles.w = msg->pose.pose.orientation.w;
    cy_angles = ToEulerAngles(Q_angles);
}

void msgCallback_USBL(const nav_msgs::Odometry::ConstPtr &msg)
{
    ag_position = msg->pose.pose.position;

    agent_pose.Pos().X() = msg->pose.pose.position.x;
    agent_pose.Pos().Y() = msg->pose.pose.position.y;
    agent_pose.Pos().Z() = msg->pose.pose.position.z;
    agent_pose.Rot().X() = msg->pose.pose.orientation.x;
    agent_pose.Rot().Y() = msg->pose.pose.orientation.y;
    agent_pose.Rot().Z() = msg->pose.pose.orientation.z;
    agent_pose.Rot().W() = msg->pose.pose.orientation.w;

    usbl_x = cos(cy_angles.yaw - atan((cy_position.y - ag_position.y) / (cy_position.x - ag_position.x))) * sqrt((cy_position.x - ag_position.x) * (cy_position.x - ag_position.x) + (cy_position.y - ag_position.y) * (cy_position.y - ag_position.y));
    usbl_y = -sin(cy_angles.yaw - atan((cy_position.y - ag_position.y) / (cy_position.x - ag_position.x))) * sqrt((cy_position.x - ag_position.x) * (cy_position.x - ag_position.x) + (cy_position.y - ag_position.y) * (cy_position.y - ag_position.y));
    usbl_z = cy_position.z - ag_position.z;

    //fs << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << cy_position.x << " " << cy_position.y << " " << cy_position.z << endl;
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
    ros::Subscriber sub_ipcom_qr_msg = nh.subscribe("/hero_agent/hero_ipcam_qr_msg", 100, messageCont_IPCAM);

    ros::Subscriber sub_ag_pose = nh.subscribe("/hero_agent/pose_gt", 100, msgCallback_USBL);
    ros::Subscriber sub_cy_pose = nh.subscribe("/cyclops/pose_gt", 100, msgCallback_cypose);

    ros::Publisher pub_usbl_agent =
        nh.advertise<hero_msgs::hero_usbl_cont>("/hero_agent/usbl", 100);

    ros::Publisher pub_graph =
        nh.advertise<geometry_msgs::Point>("/hero_agent/graph", 100);

    geometry_msgs::Point graph_position;

    hero_msgs::hero_usbl_cont hero_usbl_msg;
    /*
    ros::Subscriber sub_usbl =
        nh.subscribe("/hero_agent/usbl_transceiver_agent", 100, msgCallback_usbl);
 
    
    hero_msgs::hero_usbl_cont hero_usbl_msg;
    hero_msgs::hero_yolo_cont hero_yolo_msg;
*/
    for (int row = 0; row < 6; row++)
        for (int col = 0; col < 6; col++)
        {
            Ma(row, col) = 0.0;
            D(row, col) = 0.0;
            Ca(row, col) = 0.0;
            Kp(row, col) = 0.0;
            Kd(row, col) = 0.0;
        }

    Kp(0, 0) = 5;
    Kp(1, 1) = 5;
    Kp(2, 2) = 1;

    Kd(0, 0) = 0.1;
    Kd(1, 1) = 0.1;
    Kd(2, 2) = 5;

    cout << "Start!" << endl;
    while (ros::ok())
    {

        if (cont_tank_sim == 1)
        {
            cout << real_time / 25.0 << endl;
            fout << real_time / 25.0 << '\t' << real_target_x_position << '\t' << real_target_y_position << '\t' << real_target_z_position << '\t' << real_x_position << '\t' << real_y_position << '\t' << real_z_position + 0.092444709 << endl;


            if (real_time < 3.490815066 * 25)
            {
                real_target_x_position = 	0.00197554;
                real_target_y_position = 0.0946875;
                real_target_z_position = -0.00168018;
            }
            else if (real_time < 7.915791911 * 25)
            {
                real_target_x_position = 0.00146091;
                real_target_y_position = 0.332877;
                real_target_z_position = -0.00168018;
            }
            else if (real_time < 16.7657456 * 25)
            {
                real_target_x_position = -0.0178432;
                real_target_y_position = 1.03489;
                real_target_z_position = 0.0552125;
            }
            else if (real_time < 19.22406607 * 25)
            {
                real_target_x_position = 0.0183476;
                real_target_y_position = 1.21604;
                real_target_z_position = 0.0680355;
            }
            else if (real_time < 25.1240352 * 25)
            {
                real_target_x_position = 0.0350415;
                real_target_y_position = 1.23245;
                real_target_z_position = 0.0688531;
            }
            else if (real_time < 56.5905372 * 25)
            {
                real_target_x_position = 0.270714;
                real_target_y_position = 1.18139;
                real_target_z_position = 0.0633087;
            }
            else if (real_time < 73.79878049 * 25)
            {
                real_target_x_position = 0.025502;
                real_target_y_position = 1.21644;
                real_target_z_position = 0.0753558;
            }
            else if (real_time < 85.10705465 * 25)
            {
                real_target_x_position = -0.212172;
                real_target_y_position = 1.17675;
                real_target_z_position = 0.0786181;
            }
            else if (real_time < 99.36531337 * 25)
            {
                real_target_x_position = -0.218348;
                real_target_y_position = 1.152;
                real_target_z_position = -0.0268058;
            }
            else if (real_time < 113.6235721 * 25)
            {
                real_target_x_position = -0.21244;
                real_target_y_position = 1.22714;
                real_target_z_position = -0.0917507;
            }
            else if (real_time < 118.0485489 * 25)
            {
                real_target_x_position = -0.196655;
                real_target_y_position = 1.18811;
                real_target_z_position = -0.0925437;
            }
            else if (real_time < 129.3568231 * 25)
            {
                real_target_x_position = -0.180664;
                real_target_y_position = 1.15271;
                real_target_z_position = 0.0576646;
            }
            else if (real_time < 135.2567922 * 25)
            {
                real_target_x_position = -0.00168027;
                real_target_y_position = 0.952623;
                real_target_z_position = 0.053691;
            }
            else if (real_time < 145.0900741 * 25)
            {
                real_target_x_position = 0.0111948;
                real_target_y_position = 0.466555;
                real_target_z_position = 0.0409683;
            }
            else if (real_time < 154.923356 * 25)
            {
                real_target_x_position = -0.00208145;
                real_target_y_position = 0.0676147;
                real_target_z_position = 0.0201978;
            }
            else if (real_time > 159 * 25)
            {
                cout << "cont_tank_sim 0" << endl;
                cont_tank_sim = 0;
                fout.close();
            }

            real_time++;
        }

        real_x_position = -(ag_position.y - 27);
        real_y_position = (ag_position.x - 34.154);
        real_z_position = -(depth - 52.41);

        graph_position.x = real_x_position;
        graph_position.y = real_y_position;
        graph_position.z = real_z_position;

        pub_graph.publish(graph_position);

        target_x_position = 34.154 + real_target_y_position;
        target_y_position = 27 - real_target_x_position;
        target_z_position = 52.41 - real_target_z_position;

        hero_msgs::hero_agent_sensor sensors_msg;
        sensors_msg.ROLL = roll;
        sensors_msg.PITCH = pitch;
        sensors_msg.YAW = yaw;
        sensors_msg.DEPTH = depth;

        pub_sensors.publish(sensors_msg);

        hero_usbl_msg.TARGET_X = usbl_x;
        hero_usbl_msg.TARGET_Y = usbl_y;
        hero_usbl_msg.TARGET_Z = usbl_z;

        pub_usbl_agent.publish(hero_usbl_msg);
        // pub_yolo_agent.publish(hero_yolo_msg);

        if (cont_recovery == 1)
        {
            recovery_cont();
        }

        if (cont_target == 1)
        {
            target_cont();
        }

        count++;

        if (count == 25)
        {
            cout << "P xy " << Kp(0, 0) << " P z " << Kp(2, 2) << " D xy " << Kd(0, 0) << " D z " << Kd(2, 2) << endl;
            count = 0;

            ignition::math::Vector3d P_d;
            ignition::math::Vector3d temp_v;

            temp_v.X() = ag_position.x - pre_ag_position.x;
            temp_v.X() = ag_position.y - pre_ag_position.y;
            temp_v.X() = ag_position.z - pre_agent_z;

            P_d.X() = cy_position.x - 0.085 * cos(cy_angles.yaw) - ag_position.x;
            P_d.Y() = cy_position.x - 0.085 * sin(cy_angles.yaw) - ag_position.x;
            P_d.Z() = cy_position.x + 0.15 - ag_position.x;

            float len_c = P_d.Length();
            P_d = P_d.Normalize();

            float Ct = 0.001;
            float Cn = 1.7;
            F_drag = +0.5 * 1000 * 0.004 * 3.141592 * Ct * temp_v.Dot(P_d) * temp_v.Dot(P_d) * len_c * P_d + 0.5 * 1000 * 0.004 * Cn * len_c * (temp_v - temp_v.Dot(P_d) * P_d) * (temp_v - temp_v.Dot(P_d) * P_d);

            if (temp_v.X() >= 0)
            {
                if (F_drag.X() >= 0)
                {
                    F_drag.X() = -F_drag.X();
                }
            }
            else
            {
                if (F_drag.X() < 0)
                {
                    F_drag.X() = -F_drag.X();
                }
            }

            if (temp_v.Y() >= 0)
            {
                if (F_drag.Y() >= 0)
                {
                    F_drag.Y() = -F_drag.Y();
                }
            }
            else
            {
                if (F_drag.Y() < 0)
                {
                    F_drag.Y() = -F_drag.Y();
                }
            }

            if (temp_v.Z() >= 0)
            {
                if (F_drag.Z() >= 0)
                {
                    F_drag.Z() = -F_drag.Z();
                }
            }
            else
            {
                if (F_drag.Z() < 0)
                {
                    F_drag.Z() = -F_drag.Z();
                }
            }

            ignition::math::Vector3d linear_vel;

            lin_vel_x = ag_position.x - pre_ag_position.x / 2;
            lin_vel_y = ag_position.y - pre_ag_position.y / 2;

            pre_ag_position = ag_position;
            pre_ag_position.z = depth;
            pre_agent_z = ag_position.z;

            linear_vel.X() = target_x_position - ag_position.x;
            linear_vel.Y() = target_y_position - ag_position.y;
            linear_vel.Z() = target_z_position - depth;

            linear_vel = agent_pose.Rot().RotateVectorReverse(linear_vel);

            error_position << linear_vel.X(), linear_vel.Y(), linear_vel.Z(), 0, 0, 0;
        }

        if (cont_taesik)
        {
            Control_taesik();
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::spin(); // Increase count variable by one

    return 0;
}

void PID_control_yaw()
{
    error_yaw = desired_angle_yaw - yaw;            //angle def
    P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; //angle def + outer P control

    error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

    P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
    D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
    I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
    I_yaw = constrain(I_yaw, -100.0, 100.0);                   // I control must be limited to prevent being jerk.

    PID_yaw = P_yaw + D_yaw + I_yaw;

    if (cont_direc == 0) //stop
    {
        pwm_m1 = -PID_yaw - throttle + 1500;
        pwm_m2 = -PID_yaw + throttle + 1500;
        pwm_m4 = -PID_yaw - throttle + 1500;
        pwm_m5 = -PID_yaw + throttle + 1500;
    }
    else if (cont_direc == 1) //forward
    {
        pwm_m1 = -PID_yaw - throttle + 1500 + move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 + move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 - move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 - move_speed;
    }
    else if (cont_direc == 2) //backward
    {
        pwm_m1 = -PID_yaw - throttle + 1500 - move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 - move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 + move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 + move_speed;
    }
    else if (cont_direc == 3) //left
    {
        pwm_m1 = -PID_yaw - throttle + 1500 + move_speed - added_move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 - move_speed - added_move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 - move_speed + added_move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 + move_speed + added_move_speed;
    }
    else if (cont_direc == 4) //right
    {
        pwm_m1 = -PID_yaw - throttle + 1500 - move_speed - added_move_speed;
        pwm_m2 = -PID_yaw + throttle + 1500 + move_speed - added_move_speed;
        pwm_m4 = -PID_yaw - throttle + 1500 + move_speed + added_move_speed;
        pwm_m5 = -PID_yaw + throttle + 1500 - move_speed + added_move_speed;
    }
    else if (cont_direc == 5) //concon
    {
        //costheta = (object_x-320)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));
        //sintheta = (240-object_y)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));

        costheta = (object_x - CAMERA_WIDTH / 2) / 300;
        sintheta = (CAMERA_HEIGHT / 2 - object_y) / 300;

        pwm_m1 = -PID_yaw - throttle + 1500 - (float)move_speed * (costheta - sintheta);
        pwm_m2 = -PID_yaw + throttle + 1500 - (float)move_speed * (-costheta - sintheta);
        pwm_m4 = -PID_yaw - throttle + 1500 - (float)move_speed * (-costheta + sintheta);
        pwm_m5 = -PID_yaw + throttle + 1500 - (float)move_speed * (costheta + sintheta);
    }

    //pwm_m1 = constrain(pwm_m1, 1100, 1500);
    //pwm_m2 = constrain(pwm_m2, 1500, 1900);
    //pwm_m4 = constrain(pwm_m4, 1100, 1500);
    //pwm_m5 = constrain(pwm_m5, 1500, 1900);

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

    //esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    //esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

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

void recovery_cont()
{

    if (ipcam_qr_valid == 1)
    {
        if (ipcam_qr_z > 0.5)
        {
            added_move_speed = 10;
            yaw_stage = 0;
            move_speed = 20;
            if (ipcam_qr_x > 0.01)
            {
                cont_direc = 3;
                //left
            }
            else if (ipcam_qr_x < -0.01)
            {
                cont_direc = 4;
                //right
            }
            else
            {
                cont_direc = 2;
                //forward
                if (ipcam_qr_yaw > 0.01)
                {
                    desired_angle_yaw -= 0.005;
                }
                else if (ipcam_qr_yaw < -0.01)
                {
                    desired_angle_yaw += 0.005;
                }
            }

            if (ipcam_qr_y > -0.02)
            {
                desired_angle_depth -= 0.001;
            }
            else if (ipcam_qr_y < -0.04)
            {
                desired_angle_depth += 0.001;
            }
        }
        else if (yaw_stage == 0) //<0.5
        {
            added_move_speed = 10;
            move_speed = 30;
            if (ipcam_qr_x > 0.01)
            {
                cont_direc = 3;
                //left
            }
            else if (ipcam_qr_x < -0.01)
            {
                cont_direc = 4;
                //right
            }
            else
            {
                cont_direc = 0;
                //forward
                if (ipcam_qr_yaw > 0.01)
                {
                    desired_angle_yaw -= 0.001;
                }
                else if (ipcam_qr_yaw < -0.01)
                {
                    desired_angle_yaw += 0.001;
                }
                else
                {
                    yaw_stage = 1;
                }
            }

            if (ipcam_qr_y > -0.02)
            {
                desired_angle_depth -= 0.001;
            }
            else if (ipcam_qr_y < -0.04)
            {
                desired_angle_depth += 0.001;
            }
        }
        else if (yaw_stage == 1) //<0.5
        {
            added_move_speed = 10;
            if (ipcam_qr_x > 0.001)
            {
                move_speed = 20;
                cont_direc = 3;
                //left
            }
            else if (ipcam_qr_x < -0.001)
            {
                move_speed = 20;
                cont_direc = 4;
                //right
            }
            else
            {
                move_speed = 40;
                cont_direc = 2;
                //forward
                if (ipcam_qr_yaw > 0.1)
                {
                    desired_angle_yaw -= 0.001;
                }
                else if (ipcam_qr_yaw < -0.1)
                {
                    desired_angle_yaw += 0.001;
                }
            }
            if (ipcam_qr_z >= 0.2)
            {
                if (ipcam_qr_y > -0.01)
                {
                    desired_angle_depth -= 0.001;
                }
                else if (ipcam_qr_y < -0.03)
                {
                    desired_angle_depth += 0.001;
                }
            }

            else if (ipcam_qr_z < 0.17)
            {
                added_move_speed = 0;
                move_speed = 50;
                cont_direc = 2;
                if (ipcam_qr_y > 0.01)
                {
                    desired_angle_depth -= 0.001;
                }
                else if (ipcam_qr_y < -0.01)
                {
                    desired_angle_depth += 0.001;
                }
            }
            else if (ipcam_qr_z < 0.2)
            {
                added_move_speed = 10;
                if (ipcam_qr_y > 0.0)
                {
                    desired_angle_depth -= 0.001;
                }
                else if (ipcam_qr_y < -0.02)
                {
                    desired_angle_depth += 0.001;
                }
            }
        }
    }
}
int target_count = 0;

void target_cont()
{
    int finish_z = 0;
    if (target_count == 0)
    {
        if (usbl_z < target_usbl_z - 0.2)
        {
            desired_angle_depth += 0.02;
        }
        else if (usbl_z > target_usbl_z + 0.2)
        {
            desired_angle_depth -= 0.02;
        }
        else if (usbl_z > target_usbl_z + 0.05)
        {
            desired_angle_depth -= 0.01;
        }
        else if (usbl_z > target_usbl_z + 0.05)
        {
            desired_angle_depth -= 0.01;
        }
        else
        {
            finish_z = 1;
        }

        if (usbl_x < target_usbl_x - 0.1)
        {
            move_speed = 20;
            cont_direc = 2;
        }
        else if (usbl_x > target_usbl_x + 0.1)
        {
            move_speed = 20;
            cont_direc = 1;
        }
        else
        {
            if (usbl_y > target_usbl_y + 0.05)
            {
                move_speed = 20;
                cont_direc = 3;
            }
            else if (usbl_y < target_usbl_y - 0.05)
            {
                move_speed = 20;
                cont_direc = 4;
            }
            else
            {
                if (finish_z == 1)
                {
                    cont_target = 0;
                    cont_direc = 0;
                    target_end_count++;
                }
            }
        }
    }
    else if (target_count == 12)
    {
        if (usbl_z < target_usbl_z - 0.2)
        {
            desired_angle_depth += 0.02;
        }
        else if (usbl_z > target_usbl_z + 0.2)
        {
            desired_angle_depth -= 0.02;
        }
        else if (usbl_z > target_usbl_z + 0.05)
        {
            desired_angle_depth -= 0.01;
        }
        else if (usbl_z > target_usbl_z + 0.05)
        {
            desired_angle_depth -= 0.01;
        }
        else
        {
            finish_z = 1;
        }

        if (usbl_y > target_usbl_y + 0.05)
        {
            move_speed = 20;
            cont_direc = 3;
        }
        else if (usbl_y < target_usbl_y - 0.05)
        {
            move_speed = 20;
            cont_direc = 4;
        }
        else
        {
            if (usbl_x < target_usbl_x - 0.1)
            {
                move_speed = 20;
                cont_direc = 2;
            }
            else if (usbl_x > target_usbl_x + 0.1)
            {
                move_speed = 20;
                cont_direc = 1;
            }
            else
            {
                if (finish_z == 1)
                {
                    cont_target = 0;
                    cont_direc = 0;
                    target_end_count++;
                }
            }
        }
    }

    if (target_end_count > 100) //4 seconds
    {
        target_end_count = 0;
        cont_target = 0;
    }

    target_count++;
    if (target_count > 25)
    {
        target_count = 0;
    }
}

void Control_taesik()
{

    Ma(0, 0) = 7.69341;
    Ma(1, 1) = 7.69341;
    Ma(2, 2) = 0.86;
    Ma(3, 3) = 1.92335;
    Ma(4, 4) = 1.92335;
    Ma(5, 5) = 0.0;

    D(0, 0) = -10;
    D(1, 1) = -10;
    D(2, 2) = -3;
    D(3, 3) = -5;
    D(4, 4) = -5;
    D(5, 5) = -1;

    //Kp(0,0) = 5;
    //Kp(1,1) = 5;
    //Kp(2, 2) = 1;
    Kp(5, 5) = 1;

    //Kd(0, 0) = 0.1;
    //Kd(1, 1) = 0.1;
    //Kd(2, 2) = 0.05;
    Kd(5, 5) = 0.05;

    Eigen::Vector6d ab = Ma * _vel;

    Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
    Ca << Eigen::Matrix3d::Zero(), Sa,
        Sa, -1 * CrossProductOperator(ab.tail<3>());
    ignition::math::Vector3d linear_vel;
    linear_vel.X() = lin_vel_x;
    linear_vel.Y() = lin_vel_y;
    linear_vel.Z() = lin_vel_z;

    linear_vel = agent_pose.Rot().RotateVectorReverse(linear_vel);

    lin_vel_x = linear_vel.X() / 2.0;
    lin_vel_y = linear_vel.Y() / 2.0;
    lin_vel_z = linear_vel.Z() / 2.0;

    _vel << lin_vel_x + sum_acc_x, lin_vel_y + sum_acc_y, lin_vel_z + sum_acc_z, 0, 0, acc_yaw;

    sum_acc_x = 0;
    sum_acc_y = 0;
    sum_acc_z = 0;

    _acc_vel << acc_x, acc_y, acc_z - 9.80665, 0, 0, 0;

    //cout << "_acc_vel" << _acc_vel << endl;
    //cout << "_vel" << _vel << endl;

    error_position(2, 0) = target_z_position - depth;
    error_position(5, 0) = target_yaw - yaw;
    //
    Drag_6d << F_drag.X(), F_drag.Y(), F_drag.Z(), 0, 0, 0;

    if (cont_drag == 1)
    {
        //cout << "cont_drag 1" << endl;
        if (cont_model == 1)
        {
            //dy_result = Ma * _acc_vel + Ca * _vel + D * _vel - Drag_6d + Kp * error_position + Kd * (error_position - pre_error_position);
            dy_result = Ma * _acc_vel + Ca * _vel + D * _vel - Drag_6d + Kp * error_position + Kd * _vel / 25;
        }
        else
        {
            //dy_result = -Drag_6d + Kp * error_position + Kd * (error_position - pre_error_position);
            dy_result = -Drag_6d + Kp * error_position + Kd * _vel / 25;
        }
        //dy_result = Ma * _acc_vel - Drag_6d + Kp * error_position + Kd * (error_position - pre_error_position);
    }
    else
    {
        if (cont_model == 1)
        {
            //dy_result = Ma * _acc_vel + Ca * _vel + D * _vel + Kp * error_position + Kd * (error_position - pre_error_position);
            dy_result = Ma * _acc_vel + Ca * _vel + D * _vel + Kp * error_position + Kd * _vel / 25;
        }
        else
        {
            //dy_result = Kp * error_position + Kd * (error_position - pre_error_position);
            dy_result = Kp * error_position + Kd * _vel / 25;
        }
        //cout << "cont_drag 0" << endl;
        //dy_result = Ma * _acc_vel + Kp * error_position + Kd * (error_position - pre_error_position);
    }

    pre_error_position = error_position;
    //cout << "error_position" << error_position << endl;
    //cout << "result" << dy_result << endl;

    float mag1 = -dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2);
    float mag2 = dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2);

    if (mag1 >= 0 && mag2 >= 0)
    {
        pwm_m1 = sqrt((-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m2 = sqrt((dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m4 = -sqrt((-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m5 = -sqrt((dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
    }
    else if (mag1 < 0 && mag2 >= 0)
    {
        pwm_m1 = -sqrt(-(-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m2 = sqrt((dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m4 = sqrt(-(-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m5 = -sqrt((dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
    }
    else if (mag1 >= 0 && mag2 < 0)
    {
        pwm_m1 = sqrt((-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m2 = -sqrt(-(dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m4 = -sqrt((-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m5 = sqrt(-(dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
    }
    else if (mag1 < 0 && mag2 < 0)
    {
        pwm_m1 = -sqrt(-(-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m2 = -sqrt(-(dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m4 = sqrt(-(-dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
        pwm_m5 = sqrt(-(dy_result(0, 0) / sqrt(2) + dy_result(1, 0) / sqrt(2)) / 0.000013125 / 2) + 1500;
    }

    if (dy_result(2, 0) >= 0)
    {
        pwm_m0 = -sqrt(dy_result(2, 0) / 0.000013125 / 2) + 1500;
        pwm_m3 = -sqrt(dy_result(2, 0) / 0.000013125 / 2) + 1500;
    }
    else
    {
        pwm_m0 = sqrt(-dy_result(2, 0) / 0.000013125 / 2) + 1500;
        pwm_m3 = sqrt(-dy_result(2, 0) / 0.000013125 / 2) + 1500;
    }

    error_yaw = desired_angle_yaw - yaw;            //angle def
    P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; //angle def + outer P control

    error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

    P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
    D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
    I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
    I_yaw = constrain(I_yaw, -100.0, 100.0);                   // I control must be limited to prevent being jerk.

    PID_yaw = P_yaw + D_yaw + I_yaw;

    pwm_m1 -= PID_yaw;
    pwm_m2 -= PID_yaw;
    pwm_m4 -= PID_yaw;
    pwm_m5 -= PID_yaw;

    pwm_m0 = constrain(pwm_m0, 1100 + throttle, 1900 - throttle);
    pwm_m3 = constrain(pwm_m3, 1100 + throttle, 1900 - throttle);

    pwm_m1 = constrain(pwm_m1, 1100 + throttle, 1900 - throttle);
    pwm_m2 = constrain(pwm_m2, 1100 + throttle, 1900 - throttle);
    pwm_m4 = constrain(pwm_m4, 1100 + throttle, 1900 - throttle);
    pwm_m5 = constrain(pwm_m5, 1100 + throttle, 1900 - throttle);

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
    ccount++;
    if (cont_fout == 1)
    {
        cout << ccount << endl;
        fout << ccount << cont_drag << ' ' << target_x_position << ' ' << target_y_position << ' ' << target_z_position << ' ' << desired_angle_depth << ' ' << ag_position.x << ' ' << ag_position.y << ' ' << ag_position.z << ' ' << yaw << ' ' << dy_result(0, 0) << ' ' << dy_result(1, 0) << ' ' << dy_result(2, 0) << endl;
    }
}