/*
 * StigmergyPlanner.h
 *
 *  Created on: Jun 27, 2013
 *      Author: alcor
 */

#ifndef STIGMERGYPLANNER_H_
#define STIGMERGYPLANNER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <math.h>

#include <visualization_msgs/Marker.h>

using namespace visualization_msgs;

class StigmergyPlanner
{
public:

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	interactive_markers::MenuHandler menu_handler;

	StigmergyPlanner();
	~StigmergyPlanner();

	Marker makeBox(InteractiveMarker&);
	InteractiveMarkerControl& makeBoxControl(InteractiveMarker&);
	void makeViewFacingMarker(tf::StampedTransform);
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

	bool getRobotPose(tf::StampedTransform&);

	void reset();

	bool starting_pose_ready;
	geometry_msgs::PoseStamped starting_pose;

	bool drawn_path;
	bool finished;
	nav_msgs::Path path3D;
	ros::Publisher path_pub;
	ros::Publisher final_path_pub;
	ros::NodeHandle node;

	InteractiveMarker int_marker;

	visualization_msgs::Marker target;
	ros::Publisher target_pub;

	tf::TransformListener tf_;
	tf::StampedTransform robot_pose;
};


#endif /* STIGMERGYPLANNER_H_ */
