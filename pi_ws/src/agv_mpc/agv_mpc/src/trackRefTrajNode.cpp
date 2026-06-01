
#include <iostream>
#include <map>
#include <math.h>
#include <angles/angles.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <mutex>
// #include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "trackRefTraj.h"
#include <Eigen/Core>
#include <Eigen/QR>

// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
public:
    MPCNode();
    ~MPCNode();
    int get_thread_numbers();

private:
    std::ofstream error_log_file_;

    struct ErrorSample
    {
        double cte;
        double etheta;
    };

    std::vector<ErrorSample> error_buffer_;

    bool logging_active_ = false;
    ros::NodeHandle _nh;
    ros::Subscriber _sub_odom, _sub_gen_path, _sub_path, _sub_goal, _sub_amcl;
    ros::Publisher _pub_totalcost, _pub_ctecost, _pub_ethetacost, _pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj;
    ros::Timer _timer1;
    tf::TransformListener _tf_listener;
    std::mutex _path_mutex;
    geometry_msgs::Point _goal_pos;
    nav_msgs::Odometry _odom;
    nav_msgs::Path _odom_path, _mpc_traj;
    geometry_msgs::Twist _twist_msg;
    double _yaw_threshold;
    double rotate_vel;
    string _globalPath_topic, _goal_topic;
    string _map_frame, _odom_frame, _car_frame;

    MPC _mpc;
    map<string, double> _mpc_params;
    double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel,
        _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

    // double _Lf;
    double _slowdown_radius;
    double _stop_radius;
    double _dt, _w, _throttle, _speed, _max_speed;
    double _pathLength, _goalRadius, _waypointsDist;
    int _controller_freq, _downSampling, _thread_numbers;
    bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;

    double _dx_hat, _dy_hat;

    double _L_pos;
    double _L_th;
    double _L_d;

    bool _estimator_initialized;

    /// @brief /
    double base_cte, curve_cte, load_cte;
    double base_etheta, curve_etheta, load_etheta;
    double base_steering_penal, curve_steering_penal, load_steering_penal;
    double base_acc_penal, curve_acc_penal, load_acc_penal;
    /// @param coeffs
    /// @param x
    /// @return
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    void updateDisturbanceObserver(double x_m, double y_m, double theta_m, double Ts);
    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);

    void desiredPathCB(const nav_msgs::Path::ConstPtr &pathMsg);

    void controlLoopCB(const ros::TimerEvent &);

    // For making global planner
    nav_msgs::Path _gen_path;
    unsigned int min_idx;

    double _mpc_etheta;
    double _mpc_cte;
    fstream file;
    unsigned int idx;
}; // end of class

MPCNode::MPCNode()
{
    // Private parameters handler
    ros::NodeHandle pn("~");

    // Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, true);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 0.50);          // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 2.0);        // unit: m
    pn.param("goal_radius", _goalRadius, 0.5);        // unit: m
    pn.param("controller_freq", _controller_freq, 10);

    _dt = double(1.0 / _controller_freq); // time step duration dt in s

    _estimator_initialized = false;
    /// param for disturbance observe
    pn.param("L_pos", _L_pos, 0.2);
    pn.param("L_th", _L_th, 0.3);
    pn.param("L_d", _L_d, 0.05);
    // Parameter for MPC adaptive
    pn.param("mpc_base_cte", base_cte, 0.0);
    pn.param("mpc_curve_cte", curve_cte, 0.0);
    pn.param("mpc_load_cte", load_cte, 0.0);
    pn.param("mpc_base_etheta", base_etheta, 0.0);
    pn.param("mpc_curve_etheta", curve_etheta, 0.0);
    pn.param("mpc_load_etheta", load_etheta, 0.0);
    pn.param("mpc_base_steering_penal", base_steering_penal, 5000.0);
    pn.param("mpc_curve_steering_penal", curve_steering_penal, 5000.0);
    pn.param("mpc_load_steering_penal", load_steering_penal, 5000.0);
    pn.param("mpc_base_acc_penal", base_acc_penal, 5000.0);
    pn.param("mpc_load_acc_penal", load_acc_penal, 5000.0);
    // Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 1.0);
    pn.param("mpc_ref_etheta", _ref_etheta, 0.0);
    pn.param("mpc_w_cte", _w_cte, 5000.0);
    pn.param("mpc_w_etheta", _w_etheta, 5000.0);
    pn.param("mpc_w_vel", _w_vel, 1.0);
    pn.param("mpc_w_angvel", _w_angvel, 100.0);
    pn.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 10.0);
    pn.param("mpc_max_angvel", _max_angvel, 3.0);       // Maximal angvel radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0);   // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3);   // Bound value for other variables
    pn.param("slowdown_radius", _slowdown_radius, 0.4); //
    pn.param("stop_radius", _stop_radius, 0.1);         //
    pn.param("yaw_threshold", _yaw_threshold, 1.2);     // 70 rad
    pn.param("rotate_vel", rotate_vel, 0.5);            // rotation velocity
    // Parameter for topics & Frame name

    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_footprint");

    // Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: " << _pub_twist_flag << endl;
    cout << "debug_info: " << _debug_info << endl;
    cout << "delay_mode: " << _delay_mode << endl;
    // cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "frequency: " << _dt << endl;
    cout << "mpc_steps: " << _mpc_steps << endl;
    cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "mpc_w_cte: " << _w_cte << endl;
    cout << "mpc_w_etheta: " << _w_etheta << endl;
    cout << "mpc_max_angvel: " << _max_angvel << endl;
    cout << "stop_radius: " << _stop_radius << endl;

    // Publishers and Subscribers
    _sub_odom = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);

    _sub_gen_path = _nh.subscribe("desired_path", 1, &MPCNode::desiredPathCB, this);

    _pub_odompath = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference
    _pub_mpctraj = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1); // MPC trajectory output
    if (_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //

    _pub_totalcost = _nh.advertise<std_msgs::Float32>("/total_cost", 1);      // Global path generated from another source
    _pub_ctecost = _nh.advertise<std_msgs::Float32>("/cross_track_error", 1); // Global path generated from another source
    _pub_ethetacost = _nh.advertise<std_msgs::Float32>("/theta_error", 1);    // Global path generated from another source

    // Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0) / _controller_freq), &MPCNode::controlLoopCB, this); // 10Hz //*****mpc

    // Init variables
    _goal_received = false;
    _goal_reached = false;
    _path_computed = false;
    _throttle = 0.0;
    _w = 0.0;
    _speed = 0.0;

    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    // Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    //_mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"] = _mpc_steps;
    _mpc_params["REF_CTE"] = _ref_cte;
    _mpc_params["REF_ETHETA"] = _ref_etheta;
    _mpc_params["REF_V"] = _ref_vel;
    _mpc_params["W_CTE"] = _w_cte;
    _mpc_params["W_EPSI"] = _w_etheta;
    _mpc_params["W_V"] = _w_vel;
    _mpc_params["W_ANGVEL"] = _w_angvel;
    _mpc_params["W_A"] = _w_accel;
    _mpc_params["W_DANGVEL"] = _w_angvel_d;
    _mpc_params["W_DA"] = _w_accel_d;
    _mpc_params["ANGVEL"] = _max_angvel;
    _mpc_params["MAXTHR"] = _max_throttle;
    _mpc_params["BOUND"] = _bound_value;
    _mpc.LoadParams(_mpc_params);

    min_idx = 0;
    idx = 0;
    _mpc_etheta = 0;
    _mpc_cte = 0;
    file.open("/home/thinhhd6/catkin_ws/src/mpc_ros/mpc.csv");
}

MPCNode::~MPCNode()
{
    file.close();
};

int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}

double MPCNode::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}
void MPCNode::updateDisturbanceObserver(
    double x_m,
    double y_m,
    double theta_m,
    double Ts)
{
    // init
    if (!_estimator_initialized)
    {

        _dx_hat = 0.0;
        _dy_hat = 0.0;

        _estimator_initialized = true;
        return;
    }

    // saturate
    double v_safe =
        std::max(-0.5, std::min(_speed, 0.5));

    double w_safe =
        std::max(-1.0, std::min(_w, 1.0));

    // prediction
    double x_pred =
        x_m +
        Ts * (v_safe * cos(theta_m) + _dx_hat);

    double y_pred =
        y_m +
        Ts * (v_safe * sin(theta_m) + _dy_hat);
    double th_pred =
        theta_m +
        Ts * w_safe;

    // errors
    double err_x = x_m - x_pred;
    double err_y = y_m - y_pred;

    double err_th =
        angles::shortest_angular_distance(
            th_pred,
            theta_m);

    // disturbance adaptation
    _dx_hat += _L_d * err_x;
    _dy_hat += _L_d * err_y;
}
// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    _odom = *odomMsg;
}

// CallBack: Update generated path (conversion to odom frame)
void MPCNode::desiredPathCB(const nav_msgs::Path::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(_path_mutex);
    if (msg->poses.empty())
        return;

    // replace path
    _gen_path = *msg;

    // IMPORTANT:
    // reset tracking state
    min_idx = 0;

    _waypointsDist = -1.0;

    _odom_path.poses.clear();
    _mpc_traj.poses.clear();

    // reset controller continuity
    _w = 0.0;
    _throttle = 0.0;

    _goal_received = true;
    _goal_reached = false;
    _ref_vel = _max_speed;

    _mpc_params["REF_V"] = _ref_vel;
    _mpc.LoadParams(_mpc_params);
    ROS_INFO("Received NEW desired path");
    // error_buffer_.clear();

    // if (error_log_file_.is_open())
    //     error_log_file_.close();

    // // tạo file mới
    // std::stringstream ss;
    // ss << "/root/ros_ws/path_tracking_ws/src/mpc_ros/log_dir/mpc_error_log_"
    //    << ros::Time::now().toSec()
    //    << ".csv";

    // error_log_file_.open(ss.str());

    // if (error_log_file_.is_open())
    // {
    //     error_log_file_ << "cte,etheta\n";
    //     logging_active_ = true;

    //     ROS_INFO_STREAM("Start logging MPC error to: " << ss.str());
    // }
    // else
    // {
    //     ROS_ERROR("Cannot open error log file");
    //     logging_active_ = false;
    // }
}

// Callback: Check if the car is inside the goal area or not

void MPCNode::controlLoopCB(const ros::TimerEvent &)
{
    if (_goal_received && !_goal_reached && !_gen_path.poses.empty()) // received goal & goal not reached
    {
        nav_msgs::Odometry odom = _odom;
        nav_msgs::Path mpc_path;
        mpc_path.header.frame_id = _odom_frame;
        mpc_path.header.stamp = ros::Time::now();
        nav_msgs::Path current_path;

        {
            std::lock_guard<std::mutex> lock(_path_mutex);

            current_path = _gen_path;
        }

        try
        {
            // =====================================================
            // BASIC CHECK
            // =====================================================
            int N = current_path.poses.size();
            if (N < 2)
            {
                ROS_WARN("Global path too short");
                return;
            }

            // =====================================================
            // CURRENT ROBOT POSE
            // =====================================================

            const double px = odom.pose.pose.position.x;
            const double py = odom.pose.pose.position.y;

            tf::Pose robot_pose;
            tf::poseMsgToTF(odom.pose.pose, robot_pose);

            double robot_yaw =
                tf::getYaw(robot_pose.getRotation());

            // =====================================================
            // SEARCH NEAREST VALID WAYPOINT
            // =====================================================

            int search_start =
                std::max(0, (int)min_idx - 10);

            int search_end = N;

            double min_dist = 1e9;
            int best_idx = min_idx;

            const double MAX_SEARCH_DIST = _pathLength * 1.2; // only search points within this distance
            const double MAX_HEADING_DIFF = M_PI / 2.0;

            for (int i = search_start; i < search_end; i++)
            {
                double wx =
                    current_path.poses[i].pose.position.x;

                double wy =
                    current_path.poses[i].pose.position.y;

                double dx = wx - px;
                double dy = wy - py;

                double dist = hypot(dx, dy);

                // reject too far
                if (dist > MAX_SEARCH_DIST)
                    continue;

                // must be in front of robot
                double forward =
                    cos(robot_yaw) * dx +
                    sin(robot_yaw) * dy;

                // nearest valid point
                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_idx = i;
                }
            }

            // update tracking index
            if (min_dist < 1e9)
                min_idx = best_idx;

            // =====================================================
            // BUILD LOCAL MPC HORIZON
            // =====================================================

            mpc_path.poses.clear();

            double total_length = 0.0;

            geometry_msgs::PoseStamped tempPose;

            // small look-ahead
            int start_idx = std::min((int)min_idx + 2, N - 1);

            for (int i = start_idx; i < N; i++)
            {
                // transform to odom frame
                _tf_listener.transformPose(
                    _odom_frame,
                    ros::Time(0),
                    current_path.poses[i],
                    _odom_frame,
                    tempPose);

                // always push first point
                if (mpc_path.poses.empty())
                {
                    mpc_path.poses.push_back(tempPose);
                    continue;
                }

                // distance from previous point
                const auto &prev =
                    mpc_path.poses.back().pose.position;

                double dx =
                    tempPose.pose.position.x - prev.x;

                double dy =
                    tempPose.pose.position.y - prev.y;

                double ds = hypot(dx, dy);

                // skip duplicated points
                if (ds < 1e-4)
                    continue;

                total_length += ds;

                // stop at desired horizon length
                if (total_length > _pathLength)
                    break;

                mpc_path.poses.push_back(tempPose);
            }

            // =====================================================
            // VALIDATE MPC PATH
            // =====================================================

            if (mpc_path.poses.size() < 2)
            {
                ROS_WARN("Too few MPC points force stop");
                _goal_reached = true;
                _goal_received = false;
                _path_computed = false;
                _speed = 0.0;
                _w = 0.0;
                _throttle = 0.0;

                _twist_msg.linear.x = 0.0;
                _twist_msg.angular.z = 0.0;

                _pub_twist.publish(_twist_msg);

                return;
            }

            // save path
            _odom_path = mpc_path;

            // publish MPC reference path
            _pub_odompath.publish(mpc_path);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        nav_msgs::Path odom_path = _odom_path;

        // Update system states: X=[x, y, theta, v]
        const double px = odom.pose.pose.position.x; // pose: odom frame
        const double py = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        const double theta = tf::getYaw(pose.getRotation());

        ////
        ////

        // ===============================
        // CHECK HEADING ERROR TO PATH
        // ===============================
        const double dx_err = odom_path.poses[0].pose.position.x - px;
        const double dy_err = odom_path.poses[0].pose.position.y - py;

        double path_yaw = atan2(dy_err, dx_err);

        // angle error
        double yaw_error = angles::shortest_angular_distance(theta, path_yaw);

        // ROS_INFO("yaw_error = %.3f", yaw_error);

        // ===============================
        // ===============================
        if (fabs(yaw_error) > _yaw_threshold)
        {
            ROS_WARN("Too large yaw error -> rotate in place");

            _speed = 0.0;
            _throttle = 0.0;

            // rotate direction
            _w = (yaw_error > 0 ? rotate_vel : -rotate_vel);

            _twist_msg.linear.x = 0.0;
            _twist_msg.angular.z = _w;

            _pub_twist.publish(_twist_msg);

            return;
        }
        const double v = odom.twist.twist.linear.x; // twist: body fixed frame
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        // const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;
        // const double Lf = _Lf;

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        // ======================================================
        // CHECK TRAJECTORY FINISHED
        // ======================================================

        const auto &goal_pose = current_path.poses.back().pose.position;

        double dx_goal = goal_pose.x - px;
        double dy_goal = goal_pose.y - py;

        double dist_to_goal = hypot(dx_goal, dy_goal);

        // end ò path
        bool near_end_of_path =
            (min_idx >= current_path.poses.size() - 5);
        // ROS_INFO("idx :%d  path size:%d, dist to goal: %.2f", min_idx, current_path.poses.size(), dist_to_goal);
        if (near_end_of_path &&
            dist_to_goal < _stop_radius)
        {
            ROS_INFO("[MPC] Trajectory completed.");

            _goal_reached = true;
            _goal_received = false;
            _path_computed = false;

            _speed = 0.0;
            _w = 0.0;
            _throttle = 0.0;

            _twist_msg.linear.x = 0.0;
            _twist_msg.angular.z = 0.0;

            _pub_twist.publish(_twist_msg);

            return;
        }
        const double costheta = cos(theta);
        const double sintheta = sin(theta);

        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for (int i = 0; i < N; i++)
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = -sintheta * dx + costheta * dy;
        }

        // Fit waypoints
        // auto coeffs = polyfit(x_veh, y_veh, 3);
        int fit_order = std::min(3, N - 1);

        auto coeffs = polyfit(x_veh, y_veh, fit_order);
        // =======================================
        // CURVATURE ESTIMATION
        // =======================================

        double curvature = 0.0;

        if (coeffs.size() >= 3)
        {
            double x_eval = 0.0;

            double dy =
                coeffs[1] + 2.0 * coeffs[2] * x_eval;

            double ddy =
                2.0 * coeffs[2];

            if (coeffs.size() >= 4)
            {
                dy +=
                    3.0 * coeffs[3] * x_eval * x_eval;

                ddy +=
                    6.0 * coeffs[3] * x_eval;
            }

            curvature =
                fabs(ddy) /
                pow(1.0 + dy * dy, 1.5);
        }
        // =======================================
        // ADAPTIVE FACTOR
        // =======================================
        updateDisturbanceObserver(px, py, theta, _dt);
        double dx_body = cos(theta) * _dx_hat + sin(theta) * _dy_hat;
        double dy_body = -sin(theta) * _dx_hat + cos(theta) * _dy_hat;
        double v_ref =
            std::max(fabs(_ref_vel), 0.1);
        curvature = std::min(fabs(curvature), 2.0);
        double curve_factor = 2.0 / (1.0 + exp(-1.5 * curvature)) - 1.0;
        double disturb_mag = hypot(_dx_hat, _dy_hat);
        double load_factor = 2.0 / (1.0 + exp(-3.0 * disturb_mag)) - 1.0;
        const double cte = polyeval(coeffs, 0.0);
        const double etheta = atan(coeffs[1]);
        // lateral tracking
        double q_cte = base_cte + curve_cte * curve_factor + load_cte * load_factor;

        // heading tracking
        double q_etheta =
            base_etheta + curve_etheta * curve_factor + load_etheta * load_factor;
        // steering penalty
        double r_w = base_steering_penal - curve_steering_penal * curve_factor - load_steering_penal * load_factor;

        r_w = std::max(2.0, r_w);

        // accel penalty
        double r_a = base_acc_penal + load_acc_penal * load_factor;
        cout << "curve_factor: " << curve_factor << ", load_factor: " << load_factor << ", q_cte: " << q_cte << ", q_etheta: " << q_etheta << ", r_w: " << r_w << ", r_a: " << r_a << endl;
        _mpc_params["W_CTE"] = q_cte;
        _mpc_params["W_EPSI"] = q_etheta;
        _mpc_params["W_ANGVEL"] = r_w;
        _mpc_params["W_A"] = r_a;
        // _mpc_params["W_CTE"] = _w_cte + 250.0 * curve_factor + 400.0 * load_factor; //
        _mpc_params["W_CTE"] = std::min(_mpc_params["W_CTE"], 3000.0);
        _mpc_params["W_EPSI"] = std::min(_mpc_params["W_EPSI"], 5000.0);

        // cout << "k: " << kappa << "curv:" << 1.0 / curvature << " mpc_param:" << _mpc_params["W_CTE"] << ", " << _mpc_params["W_EPSI"] << endl;

        _mpc_cte = cte;
        _mpc_etheta = etheta;
        // ===============================
        // LOG ERROR
        // ===============================
        if (logging_active_)
        {
            ErrorSample sample;
            sample.cte = cte;
            sample.etheta = etheta;

            error_buffer_.push_back(sample);
        }
        VectorXd state(6);
        if (_delay_mode)
        {
            
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt;        //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; // v = v + a * dt

            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;

            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }

        // Solve MPC Problem
        _mpc.UpdateWeights(_mpc_params);
        vector<double> mpc_results = _mpc.Solve(state, coeffs);

        // MPC result
        _w = mpc_results[0];         // radian/sec, angular velocity
        _throttle = mpc_results[1];  // acceleration
                                     // slowdown
        _speed = v + _throttle * dt; // speed
        bool near_end_of_path_2 =
            (min_idx >= current_path.poses.size() - 12);
        if (dist_to_goal < _slowdown_radius && near_end_of_path_2)
        {
            double ratio =
                dist_to_goal / _slowdown_radius;

            double max_near_goal_speed =
                std::max(0.03, ratio * _max_speed);

            if (_speed > max_near_goal_speed)
                _speed = max_near_goal_speed;
        }
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if (_speed <= 0.0)
            _speed = 0.0;

        if (_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            // cout << "odom_path: \n" << odom_path << endl;
            // cout << "x_points: \n" << x_veh << endl;
            // cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n"
                 << coeffs << endl;
            cout << "_w: \n"
                 << _w << endl;
            cout << "_throttle: \n"
                 << _throttle << endl;
            cout << "_speed: \n"
                 << _speed << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate
        _mpc_traj.header.stamp = ros::Time::now();
        for (int i = 0; i < _mpc.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose);
        }
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);
        // STOP condition
    }
    else
    {
        _throttle = 0.0;
        _speed = 0.0;
        _w = 0;
        // if (_goal_reached)
        //     cout << "Goal Reached: control loop !" << endl;
        // =====================================
        // SAVE ERROR LOG
        // =====================================
        // if (logging_active_ && error_log_file_.is_open())
        // {
        //     for (const auto &e : error_buffer_)
        //     {
        //         error_log_file_
        //             << e.cte << ","
        //             << e.etheta << "\n";
        //     }
        //     error_buffer_.clear();
        //     error_log_file_.flush();
        //     error_log_file_.close();

        //     ROS_INFO("Saved MPC error log");

        //     logging_active_ = false;
        // }
    }

    // publish general cmd_vel
    if (_pub_twist_flag)
    {
        _twist_msg.linear.x = _speed;
        _twist_msg.angular.z = _w;
        _pub_twist.publish(_twist_msg);

        std_msgs::Float32 mpc_total_cost;
        mpc_total_cost.data = static_cast<float>(_mpc._mpc_totalcost);
        _pub_totalcost.publish(mpc_total_cost);

        std_msgs::Float32 mpc_cte_cost;
        mpc_cte_cost.data = static_cast<float>(_mpc._mpc_ctecost);
        _pub_ctecost.publish(mpc_cte_cost);

        std_msgs::Float32 mpc_etheta_cost;
        mpc_etheta_cost.data = static_cast<float>(_mpc._mpc_ethetacost);
        _pub_ethetacost.publish(mpc_etheta_cost);

        // cout << "_mpc_totalcost: "<< _mpc._mpc_totalcost << endl;
        // cout << "_mpc_ctecost: "<< _mpc._mpc_ctecost << endl;
        // cout << "_mpc_ethetacost: "<< _mpc._mpc_ethetacost << endl;
        // cout << "_mpc_velcost: "<< _mpc._mpc_velcost << endl;
        // writefile
        // idx++;
        // cout << "idx: " << idx << endl;
        file << idx << "," << _mpc_cte << "," << _mpc_etheta << "," << _twist_msg.linear.x << "," << _twist_msg.angular.z << ",";
    }
    else
    {
        _twist_msg.linear.x = 0;
        _twist_msg.angular.z = 0;
        _pub_twist.publish(_twist_msg);
    }

    /*
    file.open("/home/thinhhd6/catkin_ws/src/mpc_ros/write.csv");
    string line;
    while (getline(file, line,'\n'))
    {
        istringstream templine(line);
        string data;
        while (getline( templine, data,','))
        {
            cout << "data.c_str(): "<< data << endl;
            matrix.push_back(atof(data.c_str()));
        }
    }
    file.close();*/
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
