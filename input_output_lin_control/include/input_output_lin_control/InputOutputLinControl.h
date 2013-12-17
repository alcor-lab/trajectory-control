/*
 * InputOutputLinControl.h
 *
 *  Created on: Jun 28, 2013
 *      Author: alcor
 */

#ifndef INPUTOUTPUTLINCONTROL_H_
#define INPUTOUTPUTLINCONTROL_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nifti_robot_driver_msgs/Tracks.h>

class InputOutputLinControl
{
public:

	ros::NodeHandle node;
	tf::TransformListener tf_;

	ros::Publisher global_path_;
	ros::Publisher local_path_;

	nav_msgs::Path global_path;
	nav_msgs::Path local_path;

	tf::StampedTransform real_robot_pose_map;
	tf::StampedTransform real_robot_pose_odom;

	tf::StampedTransform real_robot_poseB_map;
	tf::StampedTransform real_robot_poseB_odom;

	tf::StampedTransform current_real_robot_pose_odom;
	tf::StampedTransform current_real_robot_poseB_odom;

	tf::StampedTransform from_odom_to_map;

	tf::Transform relative_transformB;

	tf::Transform relative_transform;

	tf::StampedTransform current_icp_real_robot_pose;
	tf::StampedTransform current_icp_real_robot_poseB;

	visualization_msgs::Marker robot_pose_marker;
	visualization_msgs::Marker true_robot_pose_marker;
	visualization_msgs::Marker icp_robot_pose_marker;

	ros::Publisher robot_pose_marker_pub;
	ros::Publisher true_robot_pose_marker_pub;
	ros::Publisher icp_robot_pose_marker_pub;

	ros::Publisher trajectory_error_pub;
	ros::Publisher robot_commands_pub;

	ros::Subscriber imu_odom_sub;
	ros::Subscriber icp_odom_sub;

	nav_msgs::Path global_plan;
	ros::Subscriber global_plan_sub;

	double displacement;
	double plan_received;

	double vel_g;

	InputOutputLinControl();
	~InputOutputLinControl();

	bool buildCircularTrajectory(double,double,geometry_msgs::Point,double,geometry_msgs::Pose&,geometry_msgs::Pose&,geometry_msgs::Twist&);
	bool buildRandomTrajectory(double,double, geometry_msgs::Point, double,geometry_msgs::Pose&, geometry_msgs::Pose&, geometry_msgs::Twist&);
	bool buildStraightLineTrajectory(double, geometry_msgs::Point, double,geometry_msgs::Pose&, geometry_msgs::Pose&, geometry_msgs::Twist&);


	bool getRobotPose(tf::StampedTransform&);
	bool getRobotPose(tf::StampedTransform&,tf::StampedTransform&);
	bool getRobotPose(tf::StampedTransform&,tf::StampedTransform&,tf::StampedTransform&);
	bool updateRobotPoseByModel(double, double, double, tf::StampedTransform&);
	bool getRobotCommands(double, tf::StampedTransform, double, double, geometry_msgs::Pose, geometry_msgs::Twist, double&, double&);
	void saturation(double&,double&);

	void getTrueRobotPose(double,tf::StampedTransform,tf::StampedTransform&);

	void imuOdomCallback(const nav_msgs::OdometryConstPtr&);
	void icpOdomCallback(const nav_msgs::OdometryConstPtr&);

	void computeRelativeTransformation(tf::StampedTransform,tf::StampedTransform,tf::Transform&);

	void updateMapRobotPose(tf::Transform,tf::StampedTransform,tf::StampedTransform&);

	void realRobotPoseB(double,tf::StampedTransform,tf::StampedTransform&);

	void markerFromPose(std::string,double,double,double,tf::StampedTransform,visualization_msgs::Marker&);
	void updateMarkerFromPose(tf::StampedTransform,visualization_msgs::Marker&);
	void odomMsgToStampedTransform(nav_msgs::Odometry,tf::StampedTransform&);
	void odomMsgToStampedTransformB(double,double,nav_msgs::Odometry,tf::StampedTransform&);

	void getRelativeTransformation2D(tf::StampedTransform,tf::StampedTransform,tf::Transform&);

	void globalPathCallback(const nav_msgs::PathConstPtr&);
	void globalPathCallback2(const nav_msgs::PathConstPtr&);

	bool buildUserDefinedTrajectory(double,geometry_msgs::PoseStamped,geometry_msgs::Pose&,geometry_msgs::Twist&);

	void getTracksVelCmd(double,double,double,nifti_robot_driver_msgs::Tracks&);

};


#endif /* INPUTOUTPUTLINCONTROL_H_ */
