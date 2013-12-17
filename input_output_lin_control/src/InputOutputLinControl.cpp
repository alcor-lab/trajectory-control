/*
 * InputOutputLinControl.cpp
 *
 *  Created on: Jun 28, 2013
 *      Author: alcor
 */

#include <input_output_lin_control/InputOutputLinControl.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <input_output_lin_control/TrajectoryError.h>
#include <input_output_lin_control/RobotCommands.h>


InputOutputLinControl::InputOutputLinControl():
displacement(0.2),
plan_received(false),
vel_g(0.15)
{
	global_path_ = node.advertise<nav_msgs::Path>("global_path",1);
	local_path_ = node.advertise<nav_msgs::Path>("local_path",1);
	robot_pose_marker_pub = node.advertise<visualization_msgs::Marker>("robot_pose",1);
	true_robot_pose_marker_pub = node.advertise<visualization_msgs::Marker>("true_robot_pose",1);
	icp_robot_pose_marker_pub = node.advertise<visualization_msgs::Marker>("/icp_robot_pose",1);
	imu_odom_sub = node.subscribe("/imu_odom",1,&InputOutputLinControl::imuOdomCallback,this);
	//icp_odom_sub = node.subscribe("/icp_odom",1,&InputOutputLinControl::icpOdomCallback,this);

	//global_plan_sub = node.subscribe("/final_stigmergy_path",1,&InputOutputLinControl::globalPathCallback,this);
	global_plan_sub = node.subscribe("/final_stigmergy_path",1,&InputOutputLinControl::globalPathCallback2,this);

	trajectory_error_pub = node.advertise<input_output_lin_control::TrajectoryError>("/trajectory_error",1);
	robot_commands_pub = node.advertise<input_output_lin_control::RobotCommands>("/robot_commands",1);

	global_path.header.frame_id = "/map";
	global_path.header.stamp = ros::Time::now();
	local_path.header.frame_id = "/map";
	local_path.header.stamp = ros::Time::now();

	while(!getRobotPose(real_robot_pose_odom,real_robot_pose_map,from_odom_to_map))
	{
		ROS_INFO("Waiting for transformation");
	}

	markerFromPose("true_robot_pose_marker",0.98,0.2,0.82,real_robot_pose_map,true_robot_pose_marker);
	true_robot_pose_marker_pub.publish(true_robot_pose_marker);

	realRobotPoseB(displacement,real_robot_pose_map,real_robot_poseB_map);
	markerFromPose("robot_poseB_marker",0,1,0,real_robot_poseB_map,robot_pose_marker);
	robot_pose_marker_pub.publish(robot_pose_marker);

	//markerFromPose("icp_robot_pose",1,1,1,real_robot_pose_map,icp_robot_pose_marker);
	//icp_robot_pose_marker_pub.publish(icp_robot_pose_marker);

	realRobotPoseB(displacement,real_robot_pose_odom,real_robot_poseB_odom);
}

InputOutputLinControl::~InputOutputLinControl()
{}

void InputOutputLinControl::globalPathCallback(const nav_msgs::PathConstPtr& msg)
{
	double vel = 0.008;
	ros::Rate rate(10);
	double timestep = rate.expectedCycleTime().nsec/1e9; // durata dell'intervallo temporale
	double error_distance = 0.01;
	double global_duration = 0;

	global_plan.header.frame_id = msg->header.frame_id;
	global_plan.header.stamp = msg->header.stamp;

	geometry_msgs::PoseStamped current;
	current.header = msg->poses[0].header;
	current.pose = msg->poses[0].pose;
	double min_error = 10000, err;
	double distance = 0;
	for(int i = 1; i < msg->poses.size(); i++)
	{
		distance += sqrt(pow(msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x,2) + pow(msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y,2) + pow(msg->poses[i].pose.position.z - msg->poses[i-1].pose.position.z,2));
		//double distance = sqrt(pow(msg->poses[i].pose.position.x - current.pose.position.x,2) + pow(msg->poses[i].pose.position.y - current.pose.position.y,2) + pow(msg->poses[i].pose.position.z - current.pose.position.z,2));
		err = fabs(vel*timestep - distance);
		ROS_INFO_STREAM(fabs(vel*timestep - distance) << " "<<error_distance);
		//if(fabs(vel*timestep - distance) < error_distance)
		if(min_error>err)	min_error = err;
		else
		{
			double yaw = atan2(msg->poses[i].pose.position.y - current.pose.position.y,msg->poses[i].pose.position.x - current.pose.position.x);
			current.header = msg->poses[i].header;
			current.pose = msg->poses[i].pose;
			geometry_msgs::PoseStamped temp;
			temp.header = current.header;
			temp.pose.position = current.pose.position;
			temp.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			distance = 0.0;
			//min_error = 10000; // Arnab
			global_plan.poses.push_back(temp);
			ROS_INFO("[%d]",i);

			current = msg->poses[i];
			global_duration += timestep;
		}
	}

	plan_received = true;
}

void InputOutputLinControl::globalPathCallback2(const nav_msgs::PathConstPtr& msg)
{
	//if(!plan_received)
	//{

	double vel = vel_g;
	ros::Rate rate(10);
	double timestep = rate.expectedCycleTime().nsec/1e9; // durata dell'intervallo temporale

	global_plan.header.frame_id = msg->header.frame_id;
	global_plan.header.stamp = msg->header.stamp;

	double distance;

	geometry_msgs::PoseStamped current;
	current.header = msg->poses[0].header;
	current.pose.position = msg->poses[0].pose.position;


	double current_dist;
	double yaw;
	double offset=0.0;
	bool changePoint=false,finalised=false;
	geometry_msgs::PoseStamped newPoint;

	int i = 1;
	while(!finalised)
	{
		if(current.pose.position.x == msg->poses[i].pose.position.x && current.pose.position.y == msg->poses[i].pose.position.y){
		i++;
		}else{
		if(changePoint){
			distance =  fabs(vel*timestep-offset);
			current_dist = sqrt(pow(msg->poses[i].pose.position.x - newPoint.pose.position.x,2) + pow(msg->poses[i].pose.position.y - newPoint.pose.position.y,2));
			//ROS_INFO("changePoint current distance [%f]",current_dist);
			//ROS_INFO("changePoint distance [%f]",distance);
			//ROS_INFO("Offset: [%f]",offset);
			ROS_INFO_STREAM("change point event");

			yaw = atan2(msg->poses[i].pose.position.y - newPoint.pose.position.y,msg->poses[i].pose.position.x - newPoint.pose.position.x);
		}else{
			distance = vel * timestep;
			current_dist = sqrt(pow(msg->poses[i].pose.position.x - current.pose.position.x,2) + pow(msg->poses[i].pose.position.y - current.pose.position.y,2));
			yaw = atan2(msg->poses[i].pose.position.y - current.pose.position.y,msg->poses[i].pose.position.x - current.pose.position.x);

			current.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			offset=0.0;
		}
		//ROS_INFO(" offset [%f]",offset);
		//ROS_INFO("distance [%f]",distance);
		//ROS_INFO("curr distance [%f]",current_dist);
		if(current_dist >= distance)
		{

			geometry_msgs::PoseStamped temp;
			temp.header = msg->poses[i].header;
			temp.pose.position.x = current.pose.position.x + (distance * cos(yaw));
			temp.pose.position.y = current.pose.position.y + (distance * sin(yaw));
			temp.pose.position.z = 0;

			if(changePoint){
				yaw = atan2(temp.pose.position.y - current.pose.position.y,temp.pose.position.x - current.pose.position.x);
				current.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			}
			global_plan.poses.push_back(current);
			ROS_INFO_STREAM("added point: current_dist: "<< current_dist << " maggiore di distance: " << distance);
			ROS_INFO_STREAM("new distance from added point: "<< sqrt(pow(msg->poses[i].pose.position.x - temp.pose.position.x,2) + pow(msg->poses[i].pose.position.y - temp.pose.position.y,2)));
			current.header = temp.header;
			current.pose = temp.pose;
			changePoint=false;
			//add to global_pose
		}
		else
		{
			offset += fabs(current_dist - distance);
			newPoint.header = msg->poses[i].header;
			newPoint.pose = msg->poses[i].pose;
			changePoint=true;
			i++;
			if(i==msg->poses.size()-2)	finalised = true;
		}

		}
		//ROS_INFO_STREAM("Index "<<i<< "");
	}
	ROS_INFO("Number of points [%d]",msg->poses.size());
	for(int i=1;i<global_plan.poses.size()-1;i++){

		global_plan.poses[i].pose.position.x = 0.3*global_plan.poses[i-1].pose.position.x + 0.4*global_plan.poses[i].pose.position.x + 0.3*global_plan.poses[i+1].pose.position.x;
		global_plan.poses[i].pose.position.y = 0.3*global_plan.poses[i-1].pose.position.y + 0.4*global_plan.poses[i].pose.position.y + 0.3*global_plan.poses[i+1].pose.position.y;
	}
	plan_received = true;
	//}
}

void InputOutputLinControl::getTracksVelCmd(double linear_vel, double angular_vel, double robot_width, nifti_robot_driver_msgs::Tracks& tracks_cmd)
{
	double d = robot_width/2;

	tracks_cmd.left = linear_vel - (d * angular_vel);
	tracks_cmd.right = linear_vel + (d * angular_vel);

	if(tracks_cmd.left < -0.6)
	{
		tracks_cmd.left = -0.6;
	}
	if(tracks_cmd.left > 0.6)
	{
		tracks_cmd.left = 0.6;
	}
	if(tracks_cmd.right < -0.6)
	{
		tracks_cmd.right = -0.6;
	}
	if(tracks_cmd.right > 0.6)
	{
		tracks_cmd.right = 0.6;
	}
}

void InputOutputLinControl::realRobotPoseB(double displacement, tf::StampedTransform real_robot_pose, tf::StampedTransform& real_robot_poseB)
{
	real_robot_poseB.frame_id_ = real_robot_pose.frame_id_;
	real_robot_poseB.stamp_ = real_robot_pose.stamp_;
	real_robot_poseB.child_frame_id_ = real_robot_pose.child_frame_id_;

	tf::Vector3 v;
	double roll, pitch, yaw;
	real_robot_pose.getBasis().getRPY(roll,pitch,yaw);

	v.setX(real_robot_pose.getOrigin().getX() + displacement*cos(yaw));
	v.setY(real_robot_pose.getOrigin().getY() + displacement*sin(yaw));
	v.setZ(real_robot_pose.getOrigin().getZ());
	real_robot_poseB.setOrigin(v);

	real_robot_poseB.setRotation(real_robot_pose.getRotation());
}

void InputOutputLinControl::markerFromPose(std::string name_space, double red, double green, double blue, tf::StampedTransform pose, visualization_msgs::Marker& marker)
{
	marker.header.frame_id = pose.frame_id_;
	marker.header.stamp = pose.stamp_;
	marker.ns = name_space.c_str();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = pose.getOrigin().getX();
	marker.pose.position.y = pose.getOrigin().getY();
	marker.pose.position.z = pose.getOrigin().getZ();
	marker.pose.orientation.x = pose.getRotation().getX();
	marker.pose.orientation.y = pose.getRotation().getY();
	marker.pose.orientation.z = pose.getRotation().getZ();
	marker.pose.orientation.w = pose.getRotation().getW();

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.8;

	marker.color.a = 1;
	marker.color.g = green;
	marker.color.r = red;
	marker.color.b = blue;

	marker.lifetime = ros::Duration(0);
}

void InputOutputLinControl::updateMarkerFromPose(tf::StampedTransform pose, visualization_msgs::Marker& marker)
{
	marker.header.stamp = pose.stamp_;
	marker.header.frame_id = pose.frame_id_;
	marker.pose.position.x = pose.getOrigin().getX();
	marker.pose.position.y = pose.getOrigin().getY();
	marker.pose.position.z = pose.getOrigin().getZ();
	marker.pose.orientation.x = pose.getRotation().getX();
	marker.pose.orientation.y = pose.getRotation().getY();
	marker.pose.orientation.z = pose.getRotation().getZ();
	marker.pose.orientation.w = pose.getRotation().getW();
}


void InputOutputLinControl::odomMsgToStampedTransform(nav_msgs::Odometry pose_odometry, tf::StampedTransform& pose_stamped)
{
	pose_stamped.stamp_ = pose_odometry.header.stamp;
	pose_stamped.frame_id_ = pose_odometry.header.frame_id;
	pose_stamped.child_frame_id_ = pose_odometry.child_frame_id;

	tf::Vector3 v;
	v.setX(pose_odometry.pose.pose.position.x);
	v.setY(pose_odometry.pose.pose.position.y);
	v.setZ(pose_odometry.pose.pose.position.z);

	pose_stamped.setOrigin(v);

	tf::Quaternion q;
	q.setX(pose_odometry.pose.pose.orientation.x);
	q.setY(pose_odometry.pose.pose.orientation.y);
	q.setZ(pose_odometry.pose.pose.orientation.z);
	q.setW(pose_odometry.pose.pose.orientation.w);

	pose_stamped.setRotation(q);
}

void InputOutputLinControl::odomMsgToStampedTransformB(double displacement, double yaw, nav_msgs::Odometry pose_odometry, tf::StampedTransform& pose_stamped)
{
	pose_stamped.stamp_ = pose_odometry.header.stamp;
	pose_stamped.frame_id_ = pose_odometry.header.frame_id;
	pose_stamped.child_frame_id_ = pose_odometry.child_frame_id;

	tf::Vector3 v;
	v.setX(pose_odometry.pose.pose.position.x + displacement*cos(yaw));
	v.setY(pose_odometry.pose.pose.position.y + displacement*sin(yaw));
	v.setZ(pose_odometry.pose.pose.position.z);

	pose_stamped.setOrigin(v);

	tf::Quaternion q;
	q.setX(pose_odometry.pose.pose.orientation.x);
	q.setY(pose_odometry.pose.pose.orientation.y);
	q.setZ(pose_odometry.pose.pose.orientation.z);
	q.setW(pose_odometry.pose.pose.orientation.w);

	pose_stamped.setRotation(q);
}

void InputOutputLinControl::imuOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	odomMsgToStampedTransform(*msg,current_real_robot_pose_odom);

	double yaw = tf::getYaw(current_real_robot_pose_odom.getRotation());

	odomMsgToStampedTransformB(displacement,yaw,*msg,current_real_robot_poseB_odom);

	if(tf_.waitForTransform("/map","/odom",msg->header.stamp,ros::Duration(0.05)))
	{
		try
		{
			tf_.lookupTransform("/map","/base_link",msg->header.stamp,real_robot_pose_map);
			tf_.lookupTransform("/map","/odom",msg->header.stamp,from_odom_to_map);
		}

		catch(tf::LookupException& ex) {
			ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
		}
		catch(tf::ConnectivityException& ex) {
			ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
		}
	}
	else
	{

		tf::Transform transformation = from_odom_to_map * current_real_robot_pose_odom;
		real_robot_pose_map.stamp_ = current_real_robot_pose_odom.stamp_;
		real_robot_pose_map.setBasis(transformation.getBasis());
		real_robot_pose_map.setOrigin(transformation.getOrigin());
		real_robot_pose_map.setRotation(transformation.getRotation());
	}

	realRobotPoseB(displacement,real_robot_pose_map,real_robot_poseB_map);

	//imu_odom_available = true;
}

/*void InputOutputLinControl::icpOdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	odomMsgToStampedTransform(*msg,current_icp_real_robot_pose);

	updateMarkerFromPose(current_icp_real_robot_pose,icp_robot_pose_marker);
	icp_robot_pose_marker_pub.publish(icp_robot_pose_marker);

	double yaw = tf::getYaw(current_icp_real_robot_pose.getRotation());

	odomMsgToStampedTransformB(displacement,yaw,*msg,current_icp_real_robot_poseB);

	icp_odom_available = true;

}*/

bool InputOutputLinControl::buildUserDefinedTrajectory(double vel, geometry_msgs::PoseStamped in, geometry_msgs::Pose& poseB, geometry_msgs::Twist& velB)
{
	poseB = in.pose;

	double yaw = tf::getYaw(in.pose.orientation);

	in.header.stamp = ros::Time::now();
	global_path.poses.push_back(in);
	global_path_.publish(global_path);

	velB.linear.x = vel * cos(yaw);
	velB.linear.y = vel * sin(yaw);

	return true;
}

bool InputOutputLinControl::buildCircularTrajectory(double displacement,double ray, geometry_msgs::Point centre, double timestep,geometry_msgs::Pose& pose, geometry_msgs::Pose& poseB, geometry_msgs::Twist& velB)
{
	geometry_msgs::Twist vel;

	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0.075; // rad/s

	vel.linear.x = - ray * vel.angular.z * sin((vel.angular.z * timestep) + (3*M_PI/2));
	vel.linear.y = ray * vel.angular.z * cos((vel.angular.z * timestep) + (3*M_PI/2));
	vel.linear.z = 0;

	pose.position.x = centre.x + ray * cos((vel.angular.z * timestep) + (3*M_PI/2));
	pose.position.y = centre.y + ray * sin((vel.angular.z * timestep) + (3*M_PI/2));
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw((vel.angular.z * timestep) + (3*M_PI/2));

	poseB.position.x = pose.position.x + displacement*cos(vel.angular.z * timestep);
	poseB.position.y = pose.position.y + displacement*sin(vel.angular.z * timestep);
	poseB.position.z = 0;

	poseB.orientation = pose.orientation;

	geometry_msgs::PoseStamped poseB_;
	poseB_.header.frame_id = "/map";
	poseB_.header.stamp = ros::Time::now() + ros::Duration(timestep,0);
	poseB_.pose = poseB;
	global_path.poses.push_back(poseB_);
	global_path_.publish(global_path);

	velB.linear.x = vel.linear.x - displacement*sin((vel.angular.z * timestep) + (3*M_PI/2))*vel.angular.z;
	velB.linear.y = vel.linear.y + displacement*cos((vel.angular.z * timestep) + (3*M_PI/2))*vel.angular.z;
	return true;
}

bool InputOutputLinControl::buildRandomTrajectory(double displacement,double ray, geometry_msgs::Point centre, double timestep,geometry_msgs::Pose& pose, geometry_msgs::Pose& poseB, geometry_msgs::Twist& velB)
{
	geometry_msgs::Twist vel;

	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0.015; // rad/s

	vel.linear.x = 0.2;
	vel.linear.y = 0.2 * vel.angular.z * cos(vel.angular.z * timestep);
	vel.linear.z = 0;

	pose.position.x = centre.x + 0.4 * timestep;
	pose.position.y = centre.y + 0.4 * sin(vel.angular.z * timestep);
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw(vel.angular.z * timestep);

	poseB.position.x = pose.position.x + displacement*cos(atan2(vel.linear.y,vel.linear.x));
	poseB.position.y = pose.position.y + displacement*sin(atan2(vel.linear.y,vel.linear.x));
	poseB.position.z = 0;

	poseB.orientation = pose.orientation;

	geometry_msgs::PoseStamped poseB_;
	poseB_.header.frame_id = "/map";
	poseB_.header.stamp = ros::Time::now() + ros::Duration(timestep,0);
	poseB_.pose = poseB;
	global_path.poses.push_back(poseB_);
	global_path_.publish(global_path);

	velB.linear.x = vel.linear.x - displacement*sin(vel.angular.z * timestep)*vel.angular.z;
	velB.linear.y = vel.linear.y + displacement*cos(vel.angular.z * timestep)*vel.angular.z;
	return true;
}

bool InputOutputLinControl::buildStraightLineTrajectory(double displacement, geometry_msgs::Point centre, double timestep,geometry_msgs::Pose& pose, geometry_msgs::Pose& poseB, geometry_msgs::Twist& velB)
{
	geometry_msgs::Twist vel;

	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0; // rad/s

	vel.linear.x = 0.0;
	vel.linear.y = - 0.25;
	vel.linear.z = 0;

	pose.position.x = centre.x;
	pose.position.y = centre.y + vel.linear.y * timestep;
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw(vel.angular.z * timestep);

	poseB.position.x = pose.position.x + displacement*cos(atan2(vel.linear.y,vel.linear.x));
	poseB.position.y = pose.position.y + displacement*sin(atan2(vel.linear.y,vel.linear.x));
	poseB.position.z = 0;

	poseB.orientation = pose.orientation;

	geometry_msgs::PoseStamped poseB_;
	poseB_.header.frame_id = "/map";
	poseB_.header.stamp = ros::Time::now();
	poseB_.pose = poseB;
	ROS_INFO("traiettoria di riferimento x [%f]",poseB.position.x);
	ROS_INFO("traiettoria di riferimento y [%f]",poseB.position.y);
	global_path.poses.push_back(poseB_);
	global_path_.publish(global_path);

	velB.linear.x = vel.linear.x - displacement*sin(vel.angular.z * timestep)*vel.angular.z;
	velB.linear.y = vel.linear.y + displacement*cos(vel.angular.z * timestep)*vel.angular.z;
	return true;
}

bool InputOutputLinControl::getRobotPose(tf::StampedTransform& robot_pose) {

	std::string robot_frame_id("/base_link");
	std::string global_frame_id("/odom");

	if(tf_.waitForTransform(global_frame_id,robot_frame_id,ros::Time(),ros::Duration(1.0)))
	{
		try
		{
			tf_.lookupTransform(global_frame_id,robot_frame_id,ros::Time(),robot_pose);
		}
		catch(tf::LookupException& ex) {
			ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	}
	else
	{
		ROS_INFO("Transformation is not available");
		return false;
	}
}

bool InputOutputLinControl::getRobotPose(tf::StampedTransform& robot_pose_odom, tf::StampedTransform& robot_pose_map) {

	std::string robot_frame_id("/base_link");
	std::string global_frame_id("/map");
	std::string odom_frame_id("/odom");

	if(tf_.waitForTransform(global_frame_id,robot_frame_id,ros::Time(),ros::Duration(1.0)))
	{
		try
		{
			tf_.lookupTransform(global_frame_id,robot_frame_id,ros::Time(),robot_pose_map);
			tf_.lookupTransform(odom_frame_id,robot_frame_id,ros::Time(),robot_pose_odom);
		}
		catch(tf::LookupException& ex) {
			ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	}
	else
	{
		ROS_INFO("Transformation is not available");
		return false;
	}
}

bool InputOutputLinControl::getRobotPose(tf::StampedTransform& robot_pose_odom, tf::StampedTransform& robot_pose_map, tf::StampedTransform& from_odom_to_map) {

	std::string robot_frame_id("/base_link");
	std::string global_frame_id("/map");
	std::string odom_frame_id("/odom");

	if(tf_.waitForTransform(global_frame_id,robot_frame_id,ros::Time(),ros::Duration(1.0)))
	{
		try
		{
			tf_.lookupTransform(global_frame_id,robot_frame_id,ros::Time(),robot_pose_map);
			tf_.lookupTransform(odom_frame_id,robot_frame_id,ros::Time(),robot_pose_odom);
			tf_.lookupTransform(global_frame_id,odom_frame_id,ros::Time(),from_odom_to_map);
		}
		catch(tf::LookupException& ex) {
			ROS_INFO("No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_INFO("Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_INFO("Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	}
	else
	{
		ROS_INFO("Transformation is not available");
		return false;
	}
}

bool InputOutputLinControl::updateRobotPoseByModel(double timestep, double linear_vel, double angular_vel, tf::StampedTransform& pose)
{

	pose.stamp_ = ros::Time::now();
	double roll, pitch, yaw;
	pose.getBasis().getRPY(roll,pitch,yaw);

	tf::Vector3 v;

	double theta = yaw + angular_vel * timestep;

	//Eulerian integration
	v.setX(pose.getOrigin().getX() + linear_vel * timestep * cos(theta));
	v.setY(pose.getOrigin().getY() + linear_vel * timestep * sin(theta));
	v.setZ(pose.getOrigin().getZ());

	pose.setOrigin(v);
	pose.setRotation(tf::createQuaternionFromYaw(theta));
	ROS_INFO("New orientation [%f]",theta);

	return true;
}

bool InputOutputLinControl::getRobotCommands(double displacement, tf::StampedTransform robot_pose, double k1, double k2, geometry_msgs::Pose poseB, geometry_msgs::Twist velB, double& linear_vel, double& angular_vel)
{
	double roll, pitch, yaw;
	robot_pose.getBasis().getRPY(roll,pitch,yaw);

	double a = cos(yaw);
	double b = sin(yaw);
	double c = -sin(yaw)/displacement;
	double d = cos(yaw)/displacement;

	double u1 = velB.linear.x + k1 * (poseB.position.x - robot_pose.getOrigin().getX());
	double u2 = velB.linear.y + k2 * (poseB.position.y - robot_pose.getOrigin().getY());

	//double u1 = k1 * (poseB.position.x - robot_pose.getOrigin().getX());
	//double u2 = k2 * (poseB.position.y - robot_pose.getOrigin().getY());

	linear_vel = a * u1 + b * u2;
	angular_vel = c * u1 + d * u2;

	geometry_msgs::PoseStamped poseR;
	poseR.header.frame_id = robot_pose.frame_id_;
	poseR.header.stamp = robot_pose.stamp_;
	poseR.pose.position.x = robot_pose.getOrigin().getX();
	poseR.pose.position.y = robot_pose.getOrigin().getY();
	poseR.pose.position.z = robot_pose.getOrigin().getZ();
	poseR.pose.orientation.x = robot_pose.getRotation().getX();
	poseR.pose.orientation.y = robot_pose.getRotation().getY();
	poseR.pose.orientation.z = robot_pose.getRotation().getZ();
	poseR.pose.orientation.w = robot_pose.getRotation().getW();
	ROS_INFO("traiettoria del robot x [%f]",robot_pose.getOrigin().getX());
	ROS_INFO("traiettoria del robot y [%f]",robot_pose.getOrigin().getY());
	local_path.poses.push_back(poseR);
	local_path_.publish(local_path);

	return true;

}

void InputOutputLinControl::saturation(double& linear_vel, double& angular_vel)
{
	ROS_INFO("Computed linear vel [%f]",linear_vel);
	ROS_INFO("Computed angular vel [%f]",angular_vel);
	if(linear_vel < -0.4)
	{
		linear_vel = -0.4;
		ROS_INFO("Saturated linear vel [%f]",linear_vel);
	}
	if(linear_vel > 0.4)
	{
		linear_vel = 0.4;
		ROS_INFO("Saturated linear vel [%f]",linear_vel);
	}
	if(angular_vel < -0.1)
	{
		angular_vel = -0.1;
		ROS_INFO("Saturated angular vel [%f]",angular_vel);
	}
	if(angular_vel > 0.1)
	{
		angular_vel = 0.1;
		ROS_INFO("Saturated angular vel [%f]",angular_vel);
	}
}

void InputOutputLinControl::getTrueRobotPose(double displacement, tf::StampedTransform robot_poseB, tf::StampedTransform& true_robot_pose)
{
	true_robot_pose.frame_id_ = robot_poseB.frame_id_;
	true_robot_pose.stamp_ = robot_poseB.stamp_;

	double roll, pitch, yaw;
	robot_poseB.getBasis().getRPY(roll,pitch,yaw);

	tf::Vector3 v;
	v.setX(robot_poseB.getOrigin().getX() - displacement*cos(yaw));
	v.setY(robot_poseB.getOrigin().getY() - displacement*sin(yaw));
	v.setZ(robot_poseB.getOrigin().getZ());

	true_robot_pose.setOrigin(v);

	true_robot_pose.setRotation(robot_poseB.getRotation());
}

void InputOutputLinControl::computeRelativeTransformation(tf::StampedTransform start, tf::StampedTransform current, tf::Transform& relative_transform)
{
	relative_transform = start.inverse() * current;
}

void InputOutputLinControl::updateMapRobotPose(tf::Transform relative_transform, tf::StampedTransform current_icp_robot_pose, tf::StampedTransform& next_icp_robot_pose)
{
	tf::Transform transformation = relative_transform * current_icp_robot_pose;
	next_icp_robot_pose.frame_id_ = current_icp_robot_pose.frame_id_;
	next_icp_robot_pose.stamp_ = ros::Time::now();
	next_icp_robot_pose.child_frame_id_ = current_icp_robot_pose.child_frame_id_;
	next_icp_robot_pose.setBasis(transformation.getBasis());
	next_icp_robot_pose.setOrigin(transformation.getOrigin());
	next_icp_robot_pose.setRotation(transformation.getRotation());
}

void InputOutputLinControl::getRelativeTransformation2D(tf::StampedTransform v1, tf::StampedTransform v2, tf::Transform& relative_transform)
{
	double roll, pitch, yaw1, yaw2;
	v1.getBasis().getRPY(roll,pitch,yaw1);
	v2.getBasis().getRPY(roll,pitch,yaw2);

	double a11 = cos(yaw1 - yaw2);
	double a12 = sin(yaw1 - yaw2);
	double a13 = 0;

	double a21 = - a12;
	double a22 = a11;
	double a23 = 0;

	double a31 = 0;
	double a32 = 0;
	double a33 = 1;

	double t1 = v2.getOrigin().getX() - v1.getOrigin().getX() * a11 - v1.getOrigin().getY()*a12;
	double t2 = v2.getOrigin().getY() - v1.getOrigin().getY() * a11 + v1.getOrigin().getX()*a12;
	double t3 = 0;

	relative_transform = tf::Transform(btMatrix3x3(a11,a12,a13,a21,a22,a23,a31,a32,a33),btVector3(t1,t2,t3));

}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"input_output_lin_control");

	InputOutputLinControl control;

	ros::NodeHandle nh;
	ros::Publisher cmd_pub;
	ros::Publisher tracks_vel_cmd_pub;
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	tracks_vel_cmd_pub = nh.advertise<nifti_robot_driver_msgs::Tracks>("/tracks_vel_cmd",1);

	double linear_vel, angular_vel;
	linear_vel = angular_vel = 0;

	double robot_width = 0.6;

	//double duration = 200; // durata del controllo in secondi


	//double displacement = 0.4; // (0.4 0 0) posizione del punto B
	double ray = 1.5; // raggio del cerchio della traiettoria
	geometry_msgs::Point centre;
	centre.x = 0.0;
	centre.y = 1.5;
	//centre.y = 0.0;
	centre.z = 0;

	geometry_msgs::Pose pose;
	geometry_msgs::Pose poseB;
	geometry_msgs::Twist velB;

	geometry_msgs::Twist base_cmd;

	double k1 = 0.3;
	double k2 = 0.3;

	double counter = 0;

	geometry_msgs::Twist init;
	init.linear.x = 0.15;
	init.linear.y = 0.15;
	init.angular.z = 0.4;
	//cmd_pub.publish(init);

	linear_vel = sqrt(init.linear.x*init.linear.x + init.linear.y*init.linear.y);
	angular_vel = init.angular.z;

	nifti_robot_driver_msgs::Tracks tracks_cmd;
	control.getTracksVelCmd(linear_vel,angular_vel,robot_width,tracks_cmd);

	//control.buildCircularTrajectory(displacement,ray,centre,timestep,pose,poseB,velB);

	//control.updateRobotPoseByModel2(counter,linear_vel,angular_vel,control.robot_pose);


	ros::Rate rate(10);
	double timestep = rate.expectedCycleTime().nsec/1e9; // durata dell'intervallo temporale

	ROS_INFO("Current TimeStep [%f]",timestep);

	int index = 0;

	while(!control.plan_received)
	{
		ROS_INFO("Plan is not received yet");
		sleep(1);
		ros::spinOnce();
	}

	//control.global_plan_sub.suthdown();


	double duration = timestep * (control.global_plan.poses.size() - 1);

	double error_x = 0;
	double error_y = 0;
	input_output_lin_control::TrajectoryError error_msg;
	input_output_lin_control::RobotCommands commands;

	while(ros::ok() && counter < duration)
	{
		ROS_INFO(" ---- cycle step ----  ");

		//control.buildCircularTrajectory(control.displacement,ray,centre,counter,pose,poseB,velB);

		//control.buildStraightLineTrajectory(control.displacement,centre,counter,pose,poseB,velB);

		//control.buildRandomTrajectory(displacement,ray,centre,counter,pose,poseB,velB);
		//ROS_INFO("Trajectory dim [%d]",control.global_plan.poses.size());

		control.buildUserDefinedTrajectory(control.vel_g,control.global_plan.poses[index],poseB,velB);

		/*if(control.icp_odom_available)
		{
			control.real_robot_poseB_map = control.current_icp_real_robot_poseB;
			control.real_robot_pose_map = control.current_icp_real_robot_pose;
			control.icp_odom_available = false;

			if(control.imu_odom_available)
			{
				control.real_robot_pose_odom = control.current_real_robot_pose_odom;
				control.real_robot_poseB_odom = control.current_real_robot_poseB_odom;
			}
		}

		else if(control.imu_odom_available)
		{
			control.computeRelativeTransformation(control.real_robot_pose_odom,control.current_real_robot_pose_odom,control.relative_transform);
			control.real_robot_pose_odom = control.current_real_robot_pose_odom;
			control.updateMapRobotPose(control.relative_transform,control.real_robot_pose_map,control.current_icp_real_robot_pose);
			control.real_robot_pose_map = control.current_icp_real_robot_pose;

			control.computeRelativeTransformation(control.real_robot_poseB_odom,control.current_real_robot_poseB_odom,control.relative_transformB);
			control.real_robot_poseB_odom = control.current_real_robot_poseB_odom;
			control.updateMapRobotPose(control.relative_transformB,control.real_robot_poseB_map,control.current_icp_real_robot_poseB);
			control.real_robot_poseB_map = control.current_icp_real_robot_poseB;
			control.imu_odom_available = false;
		}*/

		//control.real_robot_pose_odom = control.current_real_robot_pose_odom;
		//control.real_robot_poseB_odom = control.current_real_robot_poseB_odom;

		control.updateMarkerFromPose(control.real_robot_pose_map,control.true_robot_pose_marker);
		control.true_robot_pose_marker_pub.publish(control.true_robot_pose_marker);
		control.updateMarkerFromPose(control.real_robot_poseB_map,control.robot_pose_marker);
		control.robot_pose_marker_pub.publish(control.robot_pose_marker);

		//control.getRobotCommands(control.displacement,control.real_robot_poseB_odom,k1,k2,poseB,velB,linear_vel,angular_vel);
		control.getRobotCommands(control.displacement,control.real_robot_poseB_map,k1,k2,poseB,velB,linear_vel,angular_vel);

		error_x = poseB.position.x - control.real_robot_poseB_map.getOrigin().getX();
		error_y = poseB.position.y - control.real_robot_poseB_map.getOrigin().getY();
		error_msg.timestep = counter;
		error_msg.trajectory_error_x = error_x;
		error_msg.trajectory_error_y = error_y;

		error_msg.poseB_x = poseB.position.x;
		error_msg.poseB_y = poseB.position.y;
		error_msg.yawB = tf::getYaw(poseB.orientation);

		error_msg.robot_pose_x = control.real_robot_poseB_map.getOrigin().getX();
		error_msg.robot_pose_y = control.real_robot_poseB_map.getOrigin().getY();
		error_msg.yaw_robot = tf::getYaw(control.real_robot_poseB_map.getRotation());

		control.trajectory_error_pub.publish(error_msg);
		//control.saturation(linear_vel,angular_vel);
		//init.linear.x = linear_vel;
		//init.angular.z = angular_vel;
		//cmd_pub.publish(init);

		control.getTracksVelCmd(linear_vel,angular_vel,robot_width,tracks_cmd);

		commands.timestep = counter;
		commands.linear_vel = linear_vel;
		commands.angular_vel = angular_vel;
		commands.right_track_vel = tracks_cmd.right;
		commands.left_track_vel = tracks_cmd.left;
		control.robot_commands_pub.publish(commands);

		tracks_vel_cmd_pub.publish(tracks_cmd);

		rate.sleep();

		counter += timestep;

		index++;

		//ROS_INFO("Current TimeStep [%f]",counter);

		ros::spinOnce();
	}

	control.getTracksVelCmd(0,0,robot_width,tracks_cmd);
	tracks_vel_cmd_pub.publish(tracks_cmd);
	rate.sleep();
	control.getTracksVelCmd(0,0,robot_width,tracks_cmd);
	tracks_vel_cmd_pub.publish(tracks_cmd);

	return 0;
}
