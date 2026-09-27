// Re-frames the Bunker DVI Mid-360 point stream for NV-LIOM.
//
// /livox/pointcloud arrives as 0.1 s scans: the header stamp is the scan start
// and a per-point `time` field holds seconds from that start. NV-LIOM projects
// every frame onto a range image made for spinning lidars (64 x 1024), and one
// 0.1 s scan of the Mid-360's non-repetitive pattern fills it too sparsely for
// NV-LIOM's normal extraction. Longer frames fill it better, but NV-LIOM's
// Ouster callback drops frames whose points span more than 0.15 s ("wrong
// points"), so this node cuts the continuous point stream into frames of
// frame_duration (default 0.149 s, about 1.5 scans) and publishes them in
// NV-LIOM's Ouster point layout: x, y, z, intensity, t (uint32 ns from the
// frame start, which is the header stamp) and range (uint32 mm). Points are
// neither moved nor dropped here; keeping the real per-point times lets
// NV-LIOM's own deskew compensate the rotation within each frame.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

struct OusterPoint
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPoint,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint32_t, range, range))

struct TimedPoint
{
    double time;  // absolute, seconds
    float x, y, z, intensity;
};

ros::Publisher pub;
double frame_duration = 0.149;
std::string frame_id;
std::deque<TimedPoint> buffer;  // sorted by time
double frame_start = -1.0;

bool HasField(const sensor_msgs::PointCloud2 & msg, const std::string & name)
{
    for (const auto & f : msg.fields)
    {
        if (f.name == name)
        {
            return true;
        }
    }
    return false;
}

void PublishFrame(double start, double end)
{
    pcl::PointCloud<OusterPoint> cloud;
    while (!buffer.empty() && buffer.front().time < end)
    {
        const TimedPoint & p = buffer.front();
        OusterPoint o;
        o.x = p.x;
        o.y = p.y;
        o.z = p.z;
        o.intensity = p.intensity;
        o.t = static_cast<std::uint32_t>(std::max(0.0, p.time - start) * 1e9);
        o.range = static_cast<std::uint32_t>(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) * 1000.0);
        cloud.push_back(o);
        buffer.pop_front();
    }
    if (cloud.empty())
    {
        return;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time(start);
    msg.header.frame_id = frame_id;
    pub.publish(msg);
}

void OnScan(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    const double stamp = msg->header.stamp.toSec();
    if (frame_start >= 0.0 && stamp < frame_start)
    {
        ROS_WARN("livox_frame_merger: time jumped back, starting over");
        buffer.clear();
        frame_start = -1.0;
    }
    frame_id = msg->header.frame_id;

    using FloatIt = sensor_msgs::PointCloud2ConstIterator<float>;
    const size_t n = msg->width * msg->height;
    FloatIt it_x(*msg, "x");
    FloatIt it_y(*msg, "y");
    FloatIt it_z(*msg, "z");
    // Without a per-point time field every point gets the scan stamp.
    std::unique_ptr<FloatIt> it_t(HasField(*msg, "time") ? new FloatIt(*msg, "time") : nullptr);
    std::unique_ptr<FloatIt> it_i(HasField(*msg, "intensity") ? new FloatIt(*msg, "intensity") : nullptr);

    std::vector<TimedPoint> scan;
    scan.reserve(n);
    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
    {
        const float t = it_t ? **it_t : 0.0f;
        const float intensity = it_i ? **it_i : 0.0f;
        if (it_t) ++*it_t;
        if (it_i) ++*it_i;
        if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z))
        {
            continue;
        }
        scan.push_back({stamp + t, *it_x, *it_y, *it_z, intensity});
    }
    if (scan.empty())
    {
        return;
    }

    std::stable_sort(scan.begin(), scan.end(),
                     [](const TimedPoint & a, const TimedPoint & b) { return a.time < b.time; });
    buffer.insert(buffer.end(), scan.begin(), scan.end());
    if (frame_start < 0.0)
    {
        frame_start = buffer.front().time;
    }

    // A later scan starts where this one ends, so every frame that ends before
    // the newest buffered point is complete.
    while (frame_start + frame_duration <= buffer.back().time)
    {
        PublishFrame(frame_start, frame_start + frame_duration);
        frame_start += frame_duration;
        if (!buffer.empty() && buffer.front().time >= frame_start + frame_duration)
        {
            frame_start = buffer.front().time;  // skip a gap in the data
        }
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "livox_frame_merger");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string input_topic;
    std::string output_topic;
    pnh.param<std::string>("input_topic", input_topic, "/livox/pointcloud");
    pnh.param<std::string>("output_topic", output_topic, "/livox/pointcloud_merged");
    pnh.param<double>("frame_duration", frame_duration, 0.149);

    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    ros::Subscriber sub = nh.subscribe(input_topic, 100, OnScan, ros::TransportHints().tcpNoDelay());

    ROS_INFO("livox_frame_merger: %s -> %s, frames of %.3f s", input_topic.c_str(), output_topic.c_str(), frame_duration);
    ros::spin();
    return 0;
}
