#include <pronto_msgs/LidarOdometryUpdate.h>
#include <ros/node_handle.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class LidarOdometryVisualizer {
public:
    LidarOdometryVisualizer(ros::NodeHandle& nh) {

        lidar_odom_sub_ = nh.subscribe("/aicp/transform_msg",
                                      100,
                                      &LidarOdometryVisualizer::lidarOdomCallback,
                                      this);

        lidar_path_msg_.header.frame_id = "odom";

        lidar_path_pub_ = nh.advertise<nav_msgs::Path>("/aicp/lidar_odometry_path", 100);

        init_pose_sub_ = nh.subscribe("/state_estimator_pronto/pose",
                                        100,
                                        &LidarOdometryVisualizer::initPoseCallback,
                                        this);



    }
    void lidarOdomCallback(const pronto_msgs::LidarOdometryUpdateConstPtr& msg){

        tf::transformMsgToEigen(msg->relative_transform, rel_transf_);
        cumulative_transf_ = cumulative_transf_ * rel_transf_;

        tf::poseEigenToMsg(cumulative_transf_, cumulative_pose_.pose);
        cumulative_pose_.header.stamp = msg->curr_timestamp;
        lidar_path_msg_.header.stamp = msg->header.stamp;
        lidar_path_msg_.poses.push_back(cumulative_pose_);
        lidar_path_pub_.publish(lidar_path_msg_);
    }

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
        // take the very first one and shutdown
        tf::poseMsgToEigen(msg->pose.pose, cumulative_transf_);
        init_pose_sub_.shutdown();
    }


private:
    Eigen::Isometry3d rel_transf_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cumulative_transf_ = Eigen::Isometry3d::Identity();
    ros::Subscriber lidar_odom_sub_;
    ros::Subscriber init_pose_sub_;
    ros::Publisher lidar_path_pub_;
    nav_msgs::Path lidar_path_msg_;
    geometry_msgs::PoseStamped cumulative_pose_;


};





int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_odom_visualizer");
    ros::NodeHandle nh("lidar_odom_visualizer");
    LidarOdometryVisualizer viz(nh);

    ros::spin();
}
