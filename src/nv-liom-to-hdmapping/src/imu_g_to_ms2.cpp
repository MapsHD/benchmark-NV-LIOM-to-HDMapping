// IMU relay for NV-LIOM on the Bunker DVI dataset.
//
// The Bunker DVI /livox/imu reports linear_acceleration in units of g (about
// 1.0 at rest). NV-LIOM integrates raw accelerations against gravity = 9.8 m/s^2
// and would read that as an ~8.8 m/s^2 phantom acceleration. This node
// republishes the IMU with the accelerations in m/s^2 and everything else
// (header stamps included) unchanged. The first sample decides: a magnitude
// below 3 means the stream is in g and every sample is scaled; a stream already
// in m/s^2 (about 9.8 at rest) passes through unchanged.
//
// It also keeps NV-LIOM's code untouched while avoiding a start-up race in it:
// NV-LIOM subscribes to this relay as soon as it starts, long before the bag
// plays, so its IMU connection already exists when the first sample arrives.

#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub;
int scale_to_ms2 = -1;  // -1: undecided, 0: pass through, 1: scale

void OnImu(const sensor_msgs::ImuConstPtr & msg)
{
    if (scale_to_ms2 < 0)
    {
        const auto & a = msg->linear_acceleration;
        const double norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        if (norm < 0.1)
        {
            // An all-zero sample says nothing about the units.
            pub.publish(msg);
            return;
        }
        scale_to_ms2 = norm < 3.0 ? 1 : 0;
        ROS_INFO("imu_g_to_ms2: first |acc| = %.3f, %s", norm,
                 scale_to_ms2 ? "accelerometer in g, scaling by 9.80665 to m/s^2"
                              : "accelerometer already in m/s^2, passing through");
    }

    if (scale_to_ms2 != 1)
    {
        pub.publish(msg);
        return;
    }

    constexpr double kGravity = 9.80665;
    sensor_msgs::Imu out = *msg;
    out.linear_acceleration.x *= kGravity;
    out.linear_acceleration.y *= kGravity;
    out.linear_acceleration.z *= kGravity;
    pub.publish(out);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu_g_to_ms2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string input_topic;
    std::string output_topic;
    pnh.param<std::string>("input_topic", input_topic, "/livox/imu");
    pnh.param<std::string>("output_topic", output_topic, "/livox/imu_ms2");

    pub = nh.advertise<sensor_msgs::Imu>(output_topic, 2000);
    ros::Subscriber sub = nh.subscribe(input_topic, 2000, OnImu, ros::TransportHints().tcpNoDelay());

    ROS_INFO("imu_g_to_ms2: %s -> %s", input_topic.c_str(), output_topic.c_str());
    ros::spin();
    return 0;
}
