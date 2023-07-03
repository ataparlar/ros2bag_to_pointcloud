#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>
//#include <pcl/point_types.h>
//#include <pcl/conversions.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>

  class RosbagToPointCloud : public rclcpp::Node
  {
    public:
      RosbagToPointCloud();
  private:
      typedef message_filters::sync_policies::ApproximateTime<
              sensor_msgs::msg::NavSatFix,
              sensor_msgs::msg::Imu,
              sensor_msgs::msg::PointCloud2> approximate_policy;

      // made a synchronizer type which uses approximate policy
      typedef message_filters::Synchronizer<approximate_policy> Sync;

      // make this synchronizer a shared_ptr in order to reach the callback function
      // when the node will be run **********
      std::shared_ptr<Sync> sync_;

      void topic_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gnss_msg,
                          const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg);
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_subscription_;
      message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gnss_subscription_;
      message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscription_;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

      std::unique_ptr<tf2_ros::TransformBroadcaster> gnss_tf_broadcaster_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> lidar_tf_broadcaster_;

      GeographicLib::LocalCartesian local_cartesian;
      bool origin_init = false;
      struct WGSLocal{
          // some variables for local cartesian calculations
          double x;
          double y;
          double z;
      };
      WGSLocal wgs_local;
      struct UTMOrigin{
          double X;
          double Y;
          double gamma;
          double k;
          bool northp;
          int zone;
      };
      UTMOrigin utm_origin;
      GeographicLib::Geocentric earth;


//      pcl::PointCloud<pcl::PointXYZI> whole_pcl_cloud;
  };

