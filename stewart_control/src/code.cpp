#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

class IK {
public:
    IK() {
        height = 2.0;
        b << -0.101,  0.8, 0.25, 1,
              0.101,  0.8, 0.25, 1,
              0.743, -0.313, 0.25, 1,
              0.642, -0.487, 0.25, 1,
             -0.643, -0.486, 0.25, 1,
             -0.744, -0.311, 0.25, 1;

        p << -0.642,  0.487, -0.05, 1,
              0.642,  0.487, -0.05, 1,
              0.743,  0.313, -0.05, 1,
              0.101, -0.8, -0.05, 1,
             -0.101, -0.8, -0.05, 1,
             -0.743,  0.313, -0.05, 1;

        for (int i = 0; i < 6; i++) {
            f32ma_msg.data.push_back(0);
        }

        pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/position_cmd", 100);
        sub = nh.subscribe("/stewart/platform_twist", 100, &IK::callback, this);
    }

    void run() {
        ros::spin();
    }

    void calculateIK(const std::string& motion_type, double angle) {
        if (motion_type == "roll") {
            rollIK(angle);
        } else if (motion_type == "pitch") {
            pitchIK(angle);
        } else if (motion_type == "yaw") {
            yawIK(angle);
        } else if (motion_type == "home") {
            homeIK();
        }
    }

private:
    void callback(const geometry_msgs::Twist::ConstPtr& msg) {
        float x = msg->linear.x;
        float y = msg->linear.y;
        float z = msg->linear.z;
        float roll = msg->angular.x;
        float pitch = msg->angular.y;
        float yaw = msg->angular.z;
        Eigen::Matrix<float, 4, 4> T = transformationMatrix(x, y, z + height, roll, pitch, yaw);
        for (size_t i = 0; i < 6; i++) {
            Eigen::Matrix<float, 4, 1> length = T * p.row(i).transpose() - b.row(i).transpose();
            f32ma_msg.data[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)) - height;
        }
        pub.publish(f32ma_msg);
    }

    void rollIK(double angle) {
        ROS_INFO("Calculating roll IK for angle: %f", angle);
        for (size_t i = 0; i < 6; i++) {
            f32ma_msg.data[i] += angle * 0.1;
        }
        pub.publish(f32ma_msg);
    }

    void pitchIK(double angle) {
        ROS_INFO("Calculating pitch IK for angle: %f", angle);
        for (size_t i = 0; i < 6; i++) {
            f32ma_msg.data[i] += angle * 0.1;
        }
        pub.publish(f32ma_msg);
    }

    void yawIK(double angle) {
        ROS_INFO("Calculating yaw IK for angle: %f", angle);
        for (size_t i = 0; i < 6; i++) {
            f32ma_msg.data[i] += angle * 0.1;
        }
        pub.publish(f32ma_msg);
    }

    void homeIK() {
        ROS_INFO("Returning to home position.");
        for (size_t i = 0; i < 6; i++) {
            f32ma_msg.data[i] = 0.0;
        }
        pub.publish(f32ma_msg);
    }

    Eigen::Matrix<float, 4, 4> transformationMatrix(float x, float y, float z, float roll, float pitch, float yaw) {
        Eigen::Matrix<float, 4, 4> T;
        T << cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll),  sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll), x,
             sin(yaw)*cos(pitch),  cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll), y,
                     -sin(pitch),                             cos(pitch)*sin(roll),                         cos(pitch)*cos(roll), z,
                           0,                                         0,                                       0, 1;
        return T;
    }

    float height;

    Eigen::Matrix<float, 6, 4> b, p;

    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::Float32MultiArray f32ma_msg;
};

class ModelController {
public:
    ModelController() {
        ik = new IK();

        set_state = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/position_cmd", 10);

        model_name = "stewart";
        platform_height = 0.1;
        base_height = 0.25;
        base_radius = 1.0;
        ball_radius = 0.1;
        base_platform_distance = 2.0;
        platform_radius = base_radius;
        ik_positions.resize(6, 0.0);

        base_link_pose.position.x = 0.0;
        base_link_pose.position.y = 0.0;
        base_link_pose.position.z = base_height;
        base_link_pose.orientation.x = 0.0;
        base_link_pose.orientation.y = 0.0;
        base_link_pose.orientation.z = 0.0;
        base_link_pose.orientation.w = 1.0;

        setBaseLinkState(); // Setting initial base link state

        motionSequence(); // Starting motion sequence
    }

    ~ModelController() {
        delete ik;
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::ServiceClient set_state;
    ros::Publisher pub;

    IK* ik;

    std::string model_name;
    geometry_msgs::Pose base_link_pose;
    double platform_height;
    double base_height;
    double base_radius;
    double ball_radius;
    double base_platform_distance;
    double platform_radius;
    std::vector<double> ik_positions;

    void setBaseLinkState() {
        ROS_INFO("Setting initial base link state.");
        gazebo_msgs::ModelState model_state;
        model_state.model_name = model_name;
        model_state.pose = base_link_pose;
        model_state.reference_frame = "world";
        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state = model_state;
        if (set_state.call(set_model_state)) {
            ROS_INFO("Initial base link state set successfully.");
        } else {
            ROS_ERROR("Failed to set initial base link state.");
        }
    }

    void motionSequence() {
        ROS_INFO("Starting motion sequence.");
        performMotion("yaw", 30, 4.0);
        performMotion("pitch", 20, 3.0);
        performMotion("roll", 15, 2.0);
        moveToHome(1.0);
    }

    void performMotion(const std::string& motion_type, double angle, double duration) {
        ROS_INFO("Performing %s motion: %f degrees over %f seconds.", motion_type.c_str(), angle, duration);
        ros::Rate rate(10.0);
        int steps = static_cast<int>(duration * 10);
        double angle_rad = angle * M_PI / 180.0;

        for (int step = 0; step < steps; ++step) {
            double progress = static_cast<double>(step + 1) / steps;
            double current_angle = progress * angle_rad;
            ik->calculateIK(motion_type, current_angle);  // Using the ik object to call calculateIK
            rate.sleep();
        }
    }

    void moveToHome(double duration) {
        ROS_INFO("Moving to home position over %f seconds.", duration);
        ros::Rate rate(10.0);
        int steps = static_cast<int>(duration * 10);

        for (int step = 0; step < steps; ++step) {
            double progress = static_cast<double>(step + 1) / steps;
            for (size_t i = 0; i < 6; ++i) {
                ik_positions[i] = progress * 0.1;
            }
            std_msgs::Float32MultiArray msg;
            msg.data.resize(ik_positions.size());
            for (size_t i = 0; i < ik_positions.size(); ++i) {
                msg.data[i] = static_cast<float>(ik_positions[i]);
            }
            pub.publish(msg);
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stewart_control");

    ModelController controller;
    controller.run();

    return 0;
}

