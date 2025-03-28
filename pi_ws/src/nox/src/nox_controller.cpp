#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <serial/serial.h>
// #include <iostream>
#include <cmath>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <jsoncpp/json/json.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sstream>
#define ON 0
#define OFF 1
#define RESET_CMD 0
#define SPEED_CMD 1
#define CONFIG_ODOM_CMD 2
#define IO_CMD 3
enum MotorControl
{
	FRONT_MOR = 0,
	REAR_MOR = 1,
	SERVO = 2,
	ESC = 3,
	OFF_ALL = 4
};
int baud = 57600;
double radius = 0.081; // width , in m
bool usePulseCount = 0;
// double wheelbase = 0.187;                          //Wheelbase, in m
double m_per_count_l;
double m_per_count_r;
double two_pi = 6.28319;
int speed_act_left = 0;
int speed_act_right = 0;
double speed_req1 = 0.0;
double speed_req2 = 0.0;
double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double last_theta = 0.0;
double theta = 0.0;
double speed_linear = 0;
double p_rot = 0;
double d_rot = 0.0;
double speed_angular = 0;
double angular_cmd_cal = 0;
double s_rot_error = 0;
double last_e = 0;
const double PI = 3.141592653589793;
double ENCODER_PULSES = 1980.0; /// 395 count
// const double radius
float WHEEL_DIAMETER = 0.07; // 32.708; // Adjust on test cm
volatile bool check_data = 0;
volatile bool new_speed = 0;
volatile float mpu_theta = 0.0;
ros::Time current_time;
ros::Time speed_time(0.0);
ros::Time previous_time;
// Json::Value poses;
/// @brief /
ros::NodeHandle n;
ros::NodeHandle nh_private_("~");
ros::Subscriber sub_cmd;
ros::Publisher odom_pub;

tf::TransformBroadcaster broadcaster;
serial::Serial ser_;
double robot_width = 0.3126;
double rate = 20.0;
double rate_odom = 20.0;
double bias = -0.0012;	   // negative = L
double GEAR_RATIO = 1.015; // robot move > khoang cach thuc te --> giam 31.38 30.25

bool publish_tf = true;
double dt = 0.0;
double dx = 0.0;
double dy = 0.0;
double dth = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double theta_odom = 0;
double last_theta_mpu = 0;
double mpu_theta_new = 0;
bool renew_odom = 0;
bool use_mpu = true;
bool print_data = 1;
bool print_yaw = 1;
char base_link[] = "base_link";
char odom[] = "odom";
char laser[] = "/laser_link";
char kinect[] = "/kinect";
char camera_link[] = "/camera_link";
int port_ = 9600;
std::string base_link_temp;

ros::Duration d(1.0);
std::string file_path = "/home/amr_robot/amr_robot/src/init_pose/depends/init_pose.json";
std::string arduino_path = "/dev/ttyACM0";
/// @param file_path
// void loadPosesFromFile(const std::string &file_path)
// {
// 	std::ifstream file(file_path, std::ifstream::binary);
// 	if (!file.is_open())
// 	{
// 		ROS_ERROR("Cannot open file: %s", file_path.c_str());
// 		return;
// 	}
// 	file >> poses;
// }

// void idInitPoseCallback(const std_msgs::String::ConstPtr &msg,
// 						ros::Publisher &initpose_pub)
// {
// 	std::string id = msg->data;
// 	if (poses["poses"].isMember(id))
// 	{
// 		geometry_msgs::PoseWithCovarianceStamped initpose;
// 		initpose.header.stamp = ros::Time::now();
// 		initpose.header.frame_id = "map";
// 		initpose.pose.pose.position.x = poses["poses"][id]["x"].asDouble();
// 		initpose.pose.pose.position.y = poses["poses"][id]["y"].asDouble();
// 		initpose.pose.pose.position.z = poses["poses"][id]["z"].asDouble();
// 		initpose.pose.pose.orientation.x = poses["poses"][id]["qx"].asDouble();
// 		initpose.pose.pose.orientation.y = poses["poses"][id]["qy"].asDouble();
// 		initpose.pose.pose.orientation.z = poses["poses"][id]["qz"].asDouble();
// 		initpose.pose.pose.orientation.w = poses["poses"][id]["qw"].asDouble();

// 		// Set covariance to zero for simplicity
// 		for (int i = 0; i < 36; ++i)
// 		{
// 			initpose.pose.covariance[i] = 0.0;
// 		}

// 		initpose.pose.covariance[0] = 0.3;	// x
// 		initpose.pose.covariance[7] = 0.3;	// y
// 		initpose.pose.covariance[35] = 0.2; // orient

// 		initpose_pub.publish(initpose);
// 		ROS_INFO("Published initpose for ID: %s", id.c_str());
// 	}
// 	else
// 	{
// 		ROS_WARN("No pose found for ID: %s", id.c_str());
// 	}
// }
// void handle_speed( const geometry_msgs::Vector3Stamped& speed) {
//   speed_act_left = speed.vector.x;
//   ROS_INFO("speed left : %d", speed_act_left);
//   speed_act_right =speed.vector.y;
//   ROS_INFO("speed right : %d", speed_act_right);
//   speed_dt = speed.vector.z;
//   speed_time = speed.header.stamp;
//   check_data=1;
// }
void PublishOdom()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
	geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);

	if (publish_tf)
	{
		geometry_msgs::TransformStamped t;
		// geometry_msgs::TransformStamped k;

		t.header.frame_id = odom;
		t.child_frame_id = base_link;
		t.transform.translation.x = x_pos;
		t.transform.translation.y = y_pos;
		t.transform.translation.z = 0.0;
		t.transform.rotation = odom_quat;
		t.header.stamp = current_time;
		broadcaster.sendTransform(t);
	}

	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = current_time;
	odom_msg.header.frame_id = odom;
	odom_msg.pose.pose.position.x = x_pos;
	odom_msg.pose.pose.position.y = y_pos;
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation = odom_quat;
	if (speed_act_left == 0 && speed_act_right == 0)
	{
		odom_msg.pose.covariance[0] = 1e-9;
		odom_msg.pose.covariance[7] = 1e-3;
		odom_msg.pose.covariance[8] = 1e-9;
		odom_msg.pose.covariance[14] = 1e6;
		odom_msg.pose.covariance[21] = 1e6;
		odom_msg.pose.covariance[28] = 1e6;
		odom_msg.pose.covariance[35] = 1e-9;
		odom_msg.twist.covariance[0] = 1e-9;
		odom_msg.twist.covariance[7] = 1e-3;
		odom_msg.twist.covariance[8] = 1e-9;
		odom_msg.twist.covariance[14] = 1e6;
		odom_msg.twist.covariance[21] = 1e6;
		odom_msg.twist.covariance[28] = 1e6;
		odom_msg.twist.covariance[35] = 1e-9;
	}
	else
	{
		odom_msg.pose.covariance[0] = 1e-3;
		odom_msg.pose.covariance[7] = 1e-3;
		odom_msg.pose.covariance[8] = 0.0;
		odom_msg.pose.covariance[14] = 1e6;
		odom_msg.pose.covariance[21] = 1e6;
		odom_msg.pose.covariance[28] = 1e6;
		odom_msg.pose.covariance[35] = 1e3;
		odom_msg.twist.covariance[0] = 1e-3;
		odom_msg.twist.covariance[7] = 1e-3;
		odom_msg.twist.covariance[8] = 0.0;
		odom_msg.twist.covariance[14] = 1e6;
		odom_msg.twist.covariance[21] = 1e6;
		odom_msg.twist.covariance[28] = 1e6;
		odom_msg.twist.covariance[35] = 1e3;
	}
	dth = theta - last_theta;
	last_theta = theta;
	vx = (dt == 0) ? 0 : dxy / dt;
	vth = (dt == 0) ? 0 : dth / dt;
	odom_msg.child_frame_id = base_link;
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.angular.z = vth;
	odom_pub.publish(odom_msg);
}
void CalculateOdom()
{
	// check_data = 0;
	double delta_l = speed_act_left * m_per_count_l;
	double delta_r = speed_act_right * m_per_count_r;
	dxy = (delta_l + delta_r) / 2;
	// ROS_INFO("dxy : %f", dxy);
	dth = ((delta_r - delta_l)) / robot_width;
	theta_odom += dth;
	double dtheta_mpu = mpu_theta - last_theta_mpu;
	// mpu_theta_new+=dtheta_mpu;
	float diff_yaw;
	if (use_mpu)
	{
		if (dth != 0)
		{
			renew_odom = 1;
		}
		if (dth == 0 && renew_odom == 1)
		{
			renew_odom = 0;
			theta_odom = theta;
		}
		theta = 0.98 * (theta + dtheta_mpu) + 0.02 * theta_odom;
		diff_yaw = (theta - last_theta) / dt;
	}
	else
	{
		theta = theta_odom;
		diff_yaw = dth / dt;
	}

	if (theta_odom >= two_pi / 2.0)
		theta_odom -= two_pi;
	else if (theta_odom <= -two_pi / 2.0)
		theta_odom += two_pi;
	if (theta >= two_pi / 2.0)
	{
		theta -= two_pi;
	}
	if (theta <= -two_pi / 2.0)
		theta += two_pi;

	last_theta = theta;
	dx = cos(theta + dth / 2.0) * dxy;
	dy = sin(theta + dth / 2.0) * dxy;
	x_pos += dx;
	y_pos += dy;
	// x_pos += (cos(theta) * dx - sin(theta) * dy);
	// y_pos += (sin(theta) * dx + cos(theta) * dy);
	if (print_data)
	{
		ROS_INFO("angular:%f", theta * 57.29);

		ROS_INFO("x : %f", x_pos);
		ROS_INFO("y : %f", y_pos);
	}
	if (print_yaw)
	{
		ROS_INFO("dth : %f", diff_yaw);
		ROS_INFO("theta : %f", theta);
		ROS_INFO("theta_odom: %f", theta_odom);
		ROS_INFO("theta_mpu: %f", mpu_theta);
	}
}
void handle_cmd_vel(const geometry_msgs::Twist &msg)
{
	speed_linear = msg.linear.x;
	speed_angular = msg.angular.z;
	new_speed = 1;
	// ROS_INFO("angular: %f", speed_angular);
}
bool SerializePulse(std::string data)
{
	if (usePulseCount)
	{
		size_t pos1 = data.find('/');

		if (pos1 != std::string::npos)
		{
			std::string left_encoder, right_encoder;
			left_encoder = data.substr(0, pos1);
			right_encoder = data.substr(pos1 + 1);
			speed_act_left = std::stoi(left_encoder);
			speed_act_right = std::stoi(right_encoder);
			current_time = ros::Time::now();
			ros::Duration time_diff = current_time - previous_time;
			previous_time = current_time;
			check_data = 1;
			dt = time_diff.toSec();
			return 1;
		}
	}
	else
	{
		size_t pos_x = data.find('x');
		size_t pos_y = data.find('y', pos_x + 1);
		size_t v_pos = data.find('v', pos_y + 1);
		if (pos_x != std::string::npos && pos_y != std::string::npos)
		{
			std::string number_1_str = data.substr(0, x_pos);					  // 'x'
			std::string number_2_str = data.substr(x_pos + 1, y_pos - x_pos - 1); // 'x' đến 'y'
			std::string number_3_str = data.substr(y_pos + 1, v_pos - y_pos - 1); // 'y' đến cuối
			std::string number_4_str = data.substr(v_pos + 1);
			try
			{
				x_pos = std::stod(number_1_str);
				y_pos = std::stod(number_2_str);
				theta = std::stod(number_3_str);
				dxy = std::stod(number_4_str);
				ROS_INFO("x:%f y:%f theta:%f v:%f", x_pos, y_pos, theta, dxy);
				return 1;
			}
			catch (const std::invalid_argument &e)
			{
				throw std::invalid_argument("Not valid value");
			}
			catch (const std::out_of_range &e)
			{
				throw std::out_of_range("Out of range value");
			}
		}
	}
	return 0;
}
void sendSerial(int command)
{
	if (command == 0)
	{
		ser_.write("RESET");
	}
	else
	{
		throw std::invalid_argument("Lệnh không hợp lệ cho sendSerial(int)");
	}
}

// sendSerial(1, 12, 12) -> "12/12;"
void sendSerial(int command, int left, int right)
{
	if (command == 1)
	{
		std::ostringstream oss;
		oss << left << "/" << right << ";";
		ser_.write(oss.str());
	}
}

// sendSerial(2, 0.3, 0.5) -> "0.3L0.5R;"
void sendSerial(int command, double left, double right, double width)
{
	if (command == 2)
	{
		std::ostringstream oss;
		oss << left << "L" << right << "R" << width << "W;";
		ROS_INFO("Send odom config:%s", oss.str());
		ser_.write(oss.str());
	}
}
void sendSerial(int command, int IO_func, bool IO_STATE)
{
	if (command == 3)
	{
		std::ostringstream oss;
		oss << IO_func << "I" << IO_STATE << ";";
		ser_.write(oss.str());
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nox_controller");

	nh_private_.getParam("initpose_file_path", file_path);
	nh_private_.getParam("publish_rate", rate);
	nh_private_.getParam("print_data", print_data);
	nh_private_.getParam("print_yaw", print_yaw);
	nh_private_.getParam("robot_width", robot_width);
	nh_private_.getParam("publish_tf", publish_tf);
	nh_private_.getParam("GEAR_RATIO", GEAR_RATIO);
	nh_private_.getParam("use_mpu", use_mpu);
	nh_private_.getParam("bias", bias);
	nh_private_.getParam("p_rot", p_rot);
	nh_private_.getParam("d_rot", d_rot);
	nh_private_.getParam("arduino", arduino_path);
	nh_private_.getParam("baud", baud);
	nh_private_.getParam("WHEEL_DIAMETER", WHEEL_DIAMETER);
	nh_private_.getParam("ENCODER_PULSES", ENCODER_PULSES);
	nh_private_.getParam("usePulseCount", usePulseCount);
	ROS_INFO("P_rot: %f", p_rot);
	ROS_INFO("bias:%f", bias);
	ROS_INFO("rate:%f", rate);
	ROS_INFO("gear:%f", GEAR_RATIO);
	sub_cmd = n.subscribe("cmd_vel", 40, handle_cmd_vel);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	// loadPosesFromFile(file_path);
	try
	{
		ser_.setPort(arduino_path);
		ser_.setBaudrate(baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(100);
		ser_.setTimeout(to);
		ser_.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR("unable opened port");
	}

	if (ser_.isOpen())
	{
		ROS_INFO("opened");
	}
	else
	{
		ROS_ERROR("Check port available");
	}
	m_per_count_l = (1 - bias) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
	m_per_count_r = (1 + bias) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
	ROS_INFO("COUNT_L:%f", m_per_count_l);
	ROS_INFO("COUNT_R:%f", m_per_count_r);
	ROS_INFO("WIDTH_ROBOT:%f", robot_width);
	sendSerial(CONFIG_ODOM_CMD, m_per_count_l, m_per_count_r, robot_width);
	ROS_INFO("CONFIG DONE");
	sendSerial(RESET_CMD);
	ros::Publisher initpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
	// ros::Subscriber id_init_pose_sub = n.subscribe<std_msgs::String>(
	// 	"id_init_pose", 10,
	// 	boost::bind(idInitPoseCallback, _1, boost::ref(initpose_pub)));
	ros::Rate r(rate);

	previous_time = ros::Time::now(); // Khởi tạo thời gian trước đó
	while (n.ok())
	{
		ros::spinOnce();
		if (ser_.available())
		{
			ros::Time check_time = ros::Time::now();

			std::string result = ser_.readline();
			ros::Duration time_read = ros::Time::now() - check_time;
			ROS_INFO("time_read:%f", time_read.toSec());
			if (SerializePulse(result))
			{

				current_time = ros::Time::now();
				ros::Duration time_diff = current_time - previous_time;
				previous_time = current_time;
				check_data = 1;
				dt = time_diff.toSec();
			}
			//  ROS_INFO("msg : %s", result.c_str());
		}

		// Time in s
		// ROS_INFO("dt : %f", dt);
		if (dt != 0.0 && check_data == 1)
		{
			check_data = 0;
			if (usePulseCount)
				CalculateOdom();
			else
			{
			}
			PublishOdom();
			if (new_speed)
			{
				float speed_des_left = speed_linear - (robot_width / 2) * speed_angular;
				float speed_des_right = speed_linear + (robot_width / 2) * speed_angular;
				int v_left_pulse = (speed_des_left * 0.02) / m_per_count_l;
				int v_right_pulse = (speed_des_right * 0.02) / m_per_count_l;
				new_speed = 0;
				sendSerial(SPEED_CMD, v_left_pulse, v_right_pulse);
			}
		}
		r.sleep();
	}
}
