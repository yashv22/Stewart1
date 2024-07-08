#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>
#include <cmath>

class IK {
public:
    IK() {
        height = 2.0;

        // Define base and piston shaft link positions relative to base
        base_position << 0.0, 0.0, 0.0;
        piston_positions = {
            { -0.37312220398747875, 0.6426976507881956, 1.125, -0.33025008757600727, 0.0, 2.0945 }, // piston1_shaft_link
            { 0.37312220398747875, 0.6426976507881956, 1.125, -0.33025008757600727, 0.0, -2.0945 }, // piston2_shaft_link
            { 0.7431557370007575, 0.0, 1.125, -0.33025008757600727, 0.0, 0.0 }, // piston3_shaft_link
            { 0.3717050035686465, -0.6435183290001705, 1.125, -0.33025008757600727, 0.0, 2.0945 }, // piston4_shaft_link
            { -0.3713235548646821, -0.6437385082778523, 1.125, -0.33025008757600727, 0.0, -2.0945 }, // piston5_shaft_link
            { -0.7431557370007575, 0.0, 1.125, -0.33025008757600727, 0.0, 0.0 } // piston6_shaft_link
        };

        // Initialize message array for piston positions
        f32ma_msg.data.resize(6, 0.0);

        // Initialize ROS node handle, publisher, and subscriber
        ros::NodeHandle nh;
        pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/position_cmd", 100);
        sub = nh.subscribe("stewart/platform_twist", 100, &IK::callback, this);
    }

    void run() {
        ros::spin();
    }

private:
    // Callback function for twist messages
    void callback(const geometry_msgs::Twist::ConstPtr& msg) {
        float x = msg->linear.x;
        float y = msg->linear.y;
        float z = msg->linear.z;
        float roll = msg->angular.x;
        float pitch = msg->angular.y;
        float yaw = msg->angular.z;

        Eigen::Matrix<float, 4, 4> T = transformationMatrix(x, y, z + base_position(2), roll, pitch, yaw);

        for (size_t i = 0; i < piston_positions.size(); i++) {
            Eigen::Vector3f pos(piston_positions[i][0], piston_positions[i][1], piston_positions[i][2]);
            Eigen::Vector3f rotated_pos = T.block<3, 3>(0, 0) * pos + base_position.head<3>();
            f32ma_msg.data[i] = rotated_pos.z() - base_position(2);
        }

        pub.publish(f32ma_msg);
    }

    // Function to compute transformation matrix
    Eigen::Matrix<float, 4, 4> transformationMatrix(float x, float y, float z, float roll, float pitch, float yaw) {
        Eigen::Matrix<float, 4, 4> T;
        T << cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll), x,
             sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll), y,
             -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(yaw), z,
             0, 0, 0, 1;
        return T;
    }

    float height;
    Eigen::Vector3f base_position;
    std::vector<std::array<double, 6>> piston_positions;
    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::Float32MultiArray f32ma_msg;
};

int main(int argc, char** argv) {
    // Initialize IK and run ROS node
    ros::init(argc, argv, "ik_node");
    IK ik;
    ik.run();

    return 0;
}

