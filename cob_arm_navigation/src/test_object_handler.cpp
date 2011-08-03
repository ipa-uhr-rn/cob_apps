/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_apps
 * \note
 *   ROS package name: cob_arm_navigation
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: April 2011
 *
 * \brief
 *   This package provides services for handling a graspable object. 
 *   It reads data from the parameter_server in order to add it to or remove it from the environment_server as a known obstacle.
 * 	 Also it can attach such object to the robot in order to consider it as a part of the robot during the planning process.
 *
 ****************************************************************/


#include <ros/ros.h>
#include <cob_arm_navigation/HandleObject.h>

#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_object_handler");
  if (argc != 3)
  {
    ROS_INFO("usage: test_object_handler [add,remove,attach,detach] <object_name>");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client;
  
  std::string command = argv[1];

  if (command=="add")
  {
     ros::service::waitForService("object_handler/add_object");
     client = n.serviceClient<cob_arm_navigation::HandleObject>("object_handler/add_object");
  }
  else if (command=="remove")
  {
     ros::service::waitForService("object_handler/remove_object");
     client = n.serviceClient<cob_arm_navigation::HandleObject>("object_handler/remove_object");    
  }
  else if (command=="attach")
  {
     ros::service::waitForService("object_handler/attach_object");
     client = n.serviceClient<cob_arm_navigation::HandleObject>("object_handler/attach_object");    
  }
  else if (command=="detach")
  {
     ros::service::waitForService("object_handler/detach_object");
     client = n.serviceClient<cob_arm_navigation::HandleObject>("object_handler/detach_object");    
  }
  else
  {
     ROS_ERROR("Second Argument must be either add, remove, attach or detach");
     return 1;
  }

  cob_arm_navigation::HandleObject::Request req;
  cob_arm_navigation::HandleObject::Response res;
  req.object.data = argv[2];  

  if (client.call(req, res))
  {
    ROS_INFO("Result: %s", res.error_message.data.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
