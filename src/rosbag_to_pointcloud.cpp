#include <memory>
#include <fstream>

#include "rosbag_to_pointcloud/rosbag_to_pointcloud.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "Eigen/Geometry"


static std::ofstream point_cloud_file_("/home/ataparlar/projects/test_ws/src/rosbag_to_pointcloud/results/point_cloud.csv");

RosbagToPointCloud::RosbagToPointCloud() : Node("autoware_pose_to_txt") {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/poses", 10);
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    gnss_subscription_.subscribe(this, "/applanix/lvx_client/gnss/fix");
    imu_subscription_.subscribe(this, "/applanix/lvx_client/imu_raw");
    pointcloud_subscription_.subscribe(this, "/sensing/lidar/top/pointcloud_raw");

    gnss_tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    lidar_tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // synchronizer defined here with reset() function.
    // initialized with a new pointer object inside
    // with approximate policy
    sync_.reset(
            new Sync(
                    approximate_policy(100),
                    RosbagToPointCloud::gnss_subscription_,
                    RosbagToPointCloud::imu_subscription_,
                    RosbagToPointCloud::pointcloud_subscription_));

    // callback is called here.
    sync_->registerCallback(
            std::bind(&RosbagToPointCloud::topic_callback, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // create file
    // header
    if (point_cloud_file_.is_open()) {
        point_cloud_file_ << "X" << "," << "Y" << "," << "Z" << "," << "Intensity" << "\n";
    }
}

void RosbagToPointCloud::topic_callback(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gnss_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg
        ) {
    if (!origin_init) {
        origin_init = true;
        local_cartesian.Reset(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    } else {
        local_cartesian.Forward(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude,
                                wgs_local.x, wgs_local.y, wgs_local.z);
        GeographicLib::UTMUPS::Forward(
                gnss_msg->latitude,
                gnss_msg->longitude,
                utm_origin.zone,
                utm_origin.northp,
                utm_origin.X,
                utm_origin.Y,
                utm_origin.gamma,
                utm_origin.k);

        tf2::Quaternion q(
                imu_msg->orientation.x,
                imu_msg->orientation.y,
                imu_msg->orientation.z,
                imu_msg->orientation.w);
        // turn q 90 degrees around z axis
        tf2::Quaternion q_z;
        q_z.setRPY(M_PI, 0, 0);
        q = q * q_z;

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();

        geometry_msgs::msg::TransformStamped gnss_transform_msg;
        gnss_transform_msg.transform.translation.x = wgs_local.x;
        gnss_transform_msg.transform.translation.y = wgs_local.y;
        gnss_transform_msg.transform.translation.z = wgs_local.z;
        gnss_transform_msg.transform.rotation = q_msg;
        gnss_transform_msg.header.frame_id = "map";
        gnss_transform_msg.child_frame_id = "gnss";
        gnss_transform_msg.header.stamp = this->now();
        gnss_tf_broadcaster_->sendTransform(gnss_transform_msg);

        tf2::Quaternion q_lidar;
        q_lidar.setRPY(179.715196 * M_PI / 180, -0.464242 * M_PI / 180, (0.742711-90) * M_PI / 180);
        geometry_msgs::msg::TransformStamped lidar_transform_msg;
        lidar_transform_msg.transform.translation.x = 0.0;
        lidar_transform_msg.transform.translation.y = 0.0;
        lidar_transform_msg.transform.translation.z = 0.3;
        lidar_transform_msg.transform.rotation.x = q_lidar.x();
        lidar_transform_msg.transform.rotation.y = q_lidar.y();
        lidar_transform_msg.transform.rotation.z = q_lidar.z();
        lidar_transform_msg.transform.rotation.w = q_lidar.w();
        lidar_transform_msg.header.frame_id = "gnss";
        lidar_transform_msg.child_frame_id = "velodyne";
        lidar_transform_msg.header.stamp = this->now();
        lidar_tf_broadcaster_->sendTransform(lidar_transform_msg);

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        transformed_cloud.header.frame_id = "velodyne";
        transformed_cloud.header.stamp = this->now();
        transformed_cloud.data = pointcloud_msg->data;
        transformed_cloud.fields = pointcloud_msg->fields;
        transformed_cloud.height = pointcloud_msg->height;
        transformed_cloud.width = pointcloud_msg->width;
        transformed_cloud.is_bigendian = pointcloud_msg->is_bigendian;
        transformed_cloud.point_step = pointcloud_msg->point_step;
        transformed_cloud.row_step = pointcloud_msg->row_step;
        transformed_cloud.is_dense = pointcloud_msg->is_dense;
        pointcloud_publisher_->publish(transformed_cloud);


        sensor_msgs::msg::PointCloud2 global_transformed_cloud;

        // data save
        if (point_cloud_file_.is_open()) {
            sensor_msgs::PointCloud2Modifier modifier(global_transformed_cloud);
            modifier.resize(pointcloud_msg->height * pointcloud_msg->width);
            modifier.setPointCloud2Fields(4,
                                          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud_msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pointcloud_msg, "intensity");

            for (size_t i = 0; i < pointcloud_msg->height * pointcloud_msg->width;
                 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
            {
                double point_x = *iter_x;
                double point_y = *iter_y;
                double point_z = *iter_z;

//                Eigen::Affine3d axis_rotator;
//                axis_rotator.matrix().topLeftCorner<3, 3>() =
//                        Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX())
//                                .toRotationMatrix();

                Eigen::Quaterniond quat_ins_to_map(gnss_transform_msg.transform.rotation.w,
                                                   gnss_transform_msg.transform.rotation.x,
                                                   gnss_transform_msg.transform.rotation.y,
                                                   gnss_transform_msg.transform.rotation.z);
//                Eigen::Affine3d quat_ins_to_map_rotated(Eigen::Affine3d::Identity());
//                quat_ins_to_map_rotated = axis_rotator.matrix().topLeftCorner<3, 3>() * quat_ins_to_map.toRotationMatrix();

                Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
                affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
                        Eigen::AngleAxisd( (0.742711-90) * M_PI / 180, Eigen::Vector3d::UnitZ())
                                .toRotationMatrix() *
                        Eigen::AngleAxisd((0.464242) * M_PI / 180, Eigen::Vector3d::UnitY())
                                .toRotationMatrix() *
                        Eigen::AngleAxisd((179.715196) * M_PI / 180, Eigen::Vector3d::UnitX())
                                .toRotationMatrix();

//                Eigen::Affine3d axis_rotator(Eigen::Affine3d::Identity());
//                axis_rotator.matrix().topLeftCorner<3, 3>() =
//                        Eigen::AngleAxisd( 0.0, Eigen::Vector3d::UnitZ())
//                                .toRotationMatrix() *
//                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
//                                .toRotationMatrix() *
//                        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
//                                .toRotationMatrix();
//
//                Eigen::Affine3d rotated_axes(Eigen::Affine3d::Identity());
//                rotated_axes.matrix().topLeftCorner<3, 3>() = axis_rotator.matrix().topLeftCorner<3, 3>() * quat_ins_to_map.toRotationMatrix();

                // sensor to map rotation is created to add translations and get the right rotation.
                Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());
                affine_sensor2map.matrix().topLeftCorner<3, 3>() = quat_ins_to_map.toRotationMatrix() * affine_imu2lidar.rotation();

                // pose is added to the transformation matrix. - these were completed for every point in the pointclouds.
                affine_sensor2map.matrix().topRightCorner<3, 1>() << utm_origin.X, utm_origin.Y, gnss_msg->altitude;

                Eigen::Vector4d vec_point_in_first(point_x,
                                                   point_y,
                                                   point_z,
                                                   1.0);

                // create a 3D vector for transformed point to the map position and rotation.
                Eigen::Vector4d vec_point_trans = affine_sensor2map.matrix() * vec_point_in_first;

                point_x = static_cast<float>(vec_point_trans(0));
                point_y = static_cast<float>(vec_point_trans(1));
                point_z = static_cast<float>(vec_point_trans(2));

                // write to file
                point_cloud_file_ << std::fixed << std::setprecision(3) << vec_point_trans(0) << ", " << vec_point_trans(1) << ", " << vec_point_trans(2) << ", " << *iter_intensity << std::endl;

            };
        }

//        pcl::PointCloud<pcl::PointXYZI> ros2pcl_cloud;
//        pcl::fromROSMsg(transformed_global_cloud, ros2pcl_cloud);
//        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
//        pcl_cloud.resize(ros2pcl_cloud.size());
//        for (int i = 0; i < ros2pcl_cloud.size(); ++i) {
//            pcl_cloud[i].x = ros2pcl_cloud[i].x;
//            pcl_cloud[i].y = ros2pcl_cloud[i].y;
//            pcl_cloud[i].z = ros2pcl_cloud[i].z;
//            pcl_cloud[i].intensity = ros2pcl_cloud[i].intensity;
//            whole_pcl_cloud.push_back(pcl_cloud[i]);
        }
    }




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosbagToPointCloud>());
  rclcpp::shutdown();

  return 0;
}