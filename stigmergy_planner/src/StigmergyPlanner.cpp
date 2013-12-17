/*
 * StigmergyPlanner.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: alcor
 */

#include <stigmergy_planner/StigmergyPlanner.h>

StigmergyPlanner::StigmergyPlanner():starting_pose_ready(false),drawn_path(false),finished(false)
{
	path_pub = node.advertise<nav_msgs::Path>("/stigmergy_path",1);
	final_path_pub = node.advertise<nav_msgs::Path>("/final_stigmergy_path",1);
	target_pub = node.advertise<visualization_msgs::Marker>("/target",1);
	server.reset( new interactive_markers::InteractiveMarkerServer("stigmergy_planner","",false) );
	menu_handler.insert("Select Starting Pose",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Undo Starting Pose",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Apply Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Discard Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Undo Planning",boost::bind(&StigmergyPlanner::processFeedback,this,_1));
	menu_handler.insert("Evaluate Plan",boost::bind(&StigmergyPlanner::processFeedback,this,_1));

	while(!getRobotPose(robot_pose))
	{
		ROS_INFO("Waiting for transformation");
	}

	makeViewFacingMarker(robot_pose);
	server->applyChanges();

	target.header.frame_id = "/map";
	target.header.stamp = ros::Time::now();

	target.id = 1;
	target.ns = "target";

	target.scale.x = 0.3;
	target.scale.y = 0.3;
	target.scale.z = 0.3;
	target.type = visualization_msgs::Marker::MESH_RESOURCE;
	target.mesh_resource = "package://stigmergy_planner/meshes/barrier.dae";
	target.mesh_use_embedded_materials = true;
	target.lifetime = ros::Duration(0);

}

StigmergyPlanner::~StigmergyPlanner()
{}

void StigmergyPlanner::reset()
{
	server.reset();
}

// %Tag(processFeedback)%
void StigmergyPlanner::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      if(!finished && !starting_pose_ready && feedback->menu_entry_id == 1)
      {
    	  starting_pose.header.frame_id = "/map";
    	  starting_pose.header.stamp = ros::Time::now();
    	  starting_pose.pose = feedback->pose;
    	  starting_pose_ready = true;
    	  ROS_INFO("Starting Pose Selected");

    	  path3D.header.frame_id = "/map";
    	  path3D.header.stamp = ros::Time::now();
    	  path3D.poses.clear();
    	  path_pub.publish(path3D);
      }
      if(starting_pose_ready && feedback->menu_entry_id == 2)
      {
    	  starting_pose_ready = false;
    	  drawn_path = false;
    	  finished = false;
    	  path3D.poses.clear();
    	  path_pub.publish(path3D);
    	  ROS_INFO("Starting Pose Canceled");
      }
      if(drawn_path && !finished && feedback->menu_entry_id == 3)
      {
    	  finished = true;
    	  target.pose = path3D.poses[path3D.poses.size() - 1].pose;
    	  target.action = visualization_msgs::Marker::ADD;
    	  target_pub.publish(target);
    	  path_pub.publish(path3D);
    	  final_path_pub.publish(path3D);
    	  ROS_INFO("Plan Completed");
      }

      if(finished && feedback->menu_entry_id == 4)
      {
    	  starting_pose_ready = false;
    	  drawn_path = false;
    	  finished = false;
    	  path3D.poses.clear();
    	  target.action = visualization_msgs::Marker::DELETE;
    	  target_pub.publish(target);
    	  path_pub.publish(path3D);
    	  ROS_INFO("Plan Canceled");
      }

      if(drawn_path && !finished && feedback->menu_entry_id == 5)
      {
    	  if(path3D.poses.size() < 50)
    	  {
    		  path3D.poses.clear();
    		  geometry_msgs::Pose init;
    		  init.position.x = 0.0;
    		  init.position.y = 0.0;
    		  init.position.z = 0.0;
    		  init.orientation.x = 0;
    		  init.orientation.y = 0;
    		  init.orientation.z = 0;
    		  init.orientation.w = 1;
    		  server->setPose(int_marker.name,init);
    	  }
    	  else
    	  {
    		  double new_dim = path3D.poses.size() - 50;
    		  path3D.poses.resize(new_dim);
    		  server->setPose(int_marker.name,path3D.poses[new_dim - 1].pose);

    	  }
    	  path_pub.publish(path3D);
    	  ROS_INFO("Undo");
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      if(starting_pose_ready && !finished)
      {
    	  geometry_msgs::PoseStamped pose;
    	  pose.header.frame_id = feedback->header.frame_id;
    	  pose.header.stamp = feedback->header.stamp;
    	  pose.pose = feedback->pose;
    	  path3D.poses.push_back(pose);
    	  path_pub.publish(path3D);
    	  drawn_path = true;
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

bool StigmergyPlanner::getRobotPose(tf::StampedTransform& robot_pose) {

	std::string robot_frame_id("/base_link");
	std::string global_frame_id("/map");

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


Marker StigmergyPlanner::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://stigmergy_planner/meshes/hikingstick3.dae";
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = msg.scale * 0.3;
  marker.scale.y = msg.scale * 0.3;
  marker.scale.z = msg.scale * 0.3;

  /*marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0*/;

  return marker;
}

InteractiveMarkerControl& StigmergyPlanner::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void StigmergyPlanner::makeViewFacingMarker(tf::StampedTransform robot_pose)
{
  int_marker.header.frame_id = robot_pose.frame_id_;
  int_marker.header.stamp = robot_pose.stamp_;

  int_marker.pose.position.x = robot_pose.getOrigin().getX();
  int_marker.pose.position.y = robot_pose.getOrigin().getY();
  int_marker.pose.position.z = robot_pose.getOrigin().getZ();
  int_marker.scale = 1;

  int_marker.name = "view_facing";
  int_marker.description = "3D Planning pencil";

  InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  //control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  //control.orientation.w = 1;
  //control.name = "rotate";

  //int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;

  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu";
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&StigmergyPlanner::processFeedback,this,_1));
  menu_handler.apply(*server, int_marker.name);
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"stygmergy_planner");

	StigmergyPlanner planner;

	ros::spin();

	planner.reset();

	return 0;
}


