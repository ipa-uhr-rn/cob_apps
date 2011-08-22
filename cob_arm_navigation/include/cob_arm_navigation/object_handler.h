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



#ifndef OBJECT_HANDLER_INCLUDED
#define OBJECT_HANDLER_INCLUDED


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <cob_arm_navigation/HandleObject.h>
#include <gazebo/GetModelState.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/GetCollisionObjects.h>



const std::string frame_id = "/map";


class Object_Handler
{
private:
	ros::NodeHandle rh;

	ros::Publisher m_object_in_map_pub;
	ros::Publisher m_att_object_in_map_pub;

	ros::ServiceClient m_state_client;
	ros::ServiceClient m_collision_objects_client;
	
	ros::ServiceServer m_add_object_server;
	ros::ServiceServer m_remove_object_server;
	ros::ServiceServer m_attach_object_server;
	ros::ServiceServer m_detach_object_server;
	

public:	
	Object_Handler()
	{
		ROS_INFO("Object_Handler_constructor called");

		ROS_WARN("waiting for services...");
		ros::service::waitForService("/gazebo/get_model_state");
		ros::service::waitForService("/cob3_environment_server/get_collision_objects");
		ROS_INFO("...done!");
		
		
		m_object_in_map_pub  = rh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 1);
		m_att_object_in_map_pub  = rh.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 1);

		
		m_state_client = rh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
		m_collision_objects_client = rh.serviceClient<arm_navigation_msgs::GetCollisionObjects>("/cob3_environment_server/get_collision_objects");

		m_add_object_server = rh.advertiseService("/object_handler/add_object", &Object_Handler::add_object, this);
		m_remove_object_server = rh.advertiseService("/object_handler/remove_object", &Object_Handler::remove_object, this);
		m_attach_object_server = rh.advertiseService("/object_handler/attach_object", &Object_Handler::attach_object, this);
		m_detach_object_server = rh.advertiseService("/object_handler/detach_object", &Object_Handler::detach_object, this);
		ROS_INFO("object_handler ready...");
	}
	
	void run()
	{
		ROS_INFO("spinning...");
		ros::spin();
	}

private:	
	//implement callbacks here
	
	bool add_object(cob_arm_navigation::HandleObject::Request  &req,
					cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("add_object-service called!");
		ROS_INFO("Adding object %s ...",req.object.data.c_str());
		
		std::string parameter_name = req.object.data;	//this should always be identical to the model_name
		std::string model_name = req.object.data;
		ROS_INFO("Model-Name: %s", model_name.c_str());


		while(!rh.hasParam(parameter_name))
		{
		  ROS_WARN("waiting for parameter \"%s\"... ", parameter_name.c_str());
		  ros::Duration(0.5).sleep();
		}

		std::string model_parameter;
		if (rh.getParam(parameter_name, model_parameter))
		{
			ROS_INFO("Getting parameter successful!");
			ROS_DEBUG("Parameter: %s", model_parameter.c_str());
		}

		arm_navigation_msgs::CollisionObject collision_object;

		//find out the geom::type of the model
		std::string pattern = "geom:box";
		std::size_t found_box = model_parameter.find(pattern);
		if (found_box!=std::string::npos)
		{

			while (found_box!=std::string::npos)
			{
				std::vector< std::string > name;
				std::vector< double > dimensions;
				std::vector< double > location;
				std::vector< double > rotation;

				ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_box));
				bool success,success_d,success_l,success_r, success_n;
				
				success_n = parse_box_name(model_parameter, name, found_box);
				success_d = parse_box_dimensions(model_parameter, dimensions,found_box);
				success_l = parse_box_location(model_parameter, location,found_box);
				success_r = parse_box_rotation(model_parameter, rotation,found_box);

				if(!(success_n && success_d && success_l && success_r))
				{
					ROS_ERROR("Error while parsing a geom:box model. Aborting!");
			  
					res.success.data = false;
					res.error_message.data = "Error while parsing a geom:box model.";
	
					return false;
				}
				
				success = compose_box(model_name, dimensions,location,rotation, collision_object, name[0]);
				
				if(!success)
				{
					ROS_ERROR("Error while composing the collision_object. Aborting!");
			  
					res.success.data = false;
					res.error_message.data = "Error while composing the collision_object.";
	
					return false;
				}

				ROS_INFO("Added %s to the object %s!",name[0].c_str(), model_name.c_str());
	
				found_box = model_parameter.find(pattern,found_box+1);
				//second search necessary since each <geom:box> has a </geom:box>
				found_box = model_parameter.find(pattern,found_box+1);

				//clear parsing for next box
				name.pop_back();
				while(!dimensions.empty())
				{
					dimensions.pop_back();
				}
				while(!location.empty())
				{
					location.pop_back();
				}
				while(!rotation.empty())
				{
					rotation.pop_back();
				}

			}
		}
		else
		{str:
		  ROS_ERROR("%s not found", pattern.c_str());
		  ROS_ERROR("I can't parse this model. Aborting!");
		  
		  res.success.data = false;
		  res.error_message.data = "No parser for the geom:type of this model available yet.";
		  
		  return false;
		}
	
		collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
	
		m_object_in_map_pub.publish(collision_object);

		ROS_INFO("%s added to environment server!",model_name.c_str());

		res.success.data = true;
		res.error_message.data = "Object added to environment server!";
		return true;
	}
	
	
	
	bool remove_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("remove_object-service called!");
		ROS_INFO("Removing object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		arm_navigation_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.collision_objects.size(); i++)
			{
				if(srv.response.collision_objects[i].id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					arm_navigation_msgs::CollisionObject collision_object = srv.response.collision_objects[i];
					
					collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
					
					m_object_in_map_pub.publish(collision_object);

					ROS_INFO("Object removed from environment server!");

					res.success.data = true;
					res.error_message.data = "Object removed from environment server!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among known objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among known objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	bool attach_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("attach_object-service called!");
		ROS_INFO("Attaching object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		arm_navigation_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.collision_objects.size(); i++)
			{
				if(srv.response.collision_objects[i].id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					
					arm_navigation_msgs::AttachedCollisionObject att_object;
					att_object.object = srv.response.collision_objects[i];
					//attach it to the SDH
					att_object.link_name = "sdh_palm_link";
					//att_object.touch_links.push_back("sdh_grasp_link");
					att_object.touch_links.push_back("sdh_finger_11_link");
					att_object.touch_links.push_back("sdh_finger_12_link");
					att_object.touch_links.push_back("sdh_finger_13_link");
					att_object.touch_links.push_back("sdh_finger_21_link");
					att_object.touch_links.push_back("sdh_finger_22_link");
					att_object.touch_links.push_back("sdh_finger_23_link");
					att_object.touch_links.push_back("sdh_thumb_1_link");
					att_object.touch_links.push_back("sdh_thumb_2_link");
					att_object.touch_links.push_back("sdh_thumb_3_link");
					
					att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
					
					m_att_object_in_map_pub.publish(att_object);

					ROS_INFO("Object attached to robot!");

					res.success.data = true;
					res.error_message.data = "Object attached to robot!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among known objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among known objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	bool detach_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("detach_object-service called!");
		ROS_INFO("Detaching object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		arm_navigation_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.attached_collision_objects.size(); i++)
			{
				if(srv.response.attached_collision_objects[i].object.id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					
					arm_navigation_msgs::AttachedCollisionObject att_object = srv.response.attached_collision_objects[i];
					
					att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
					
					m_att_object_in_map_pub.publish(att_object);

					ROS_INFO("Object detached from robot!");

					res.success.data = true;
					res.error_message.data = "Object detached from robot!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among attached objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among attached objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	
	//helper functions
	bool parse_box_name(std::string model_parameter, std::vector< std::string> &name, std::size_t start_point)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_name, found_end;

		pattern = "name=";
		found_name=model_parameter.find(pattern,start_point);
		if (found_name!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_name));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_end=model_parameter.find(pattern, found_name);
		if (found_end!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_end));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_x = found_end - 1 - found_name - 6;
		std::string name_found = model_parameter.substr(found_name+6, length_x);
		ROS_DEBUG("name: %s, real_length: %d", name_found.c_str(), int(length_x));

		ROS_INFO("geom name: %s", name_found.c_str());

		name.push_back(name_found);
			
		return true;
	}

	bool parse_box_dimensions(std::string model_parameter, std::vector< double > &dimensions, std::size_t start_point)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_size, found_p, found_x, found_y, found_z;

		pattern = "size";
		found_size=model_parameter.find(pattern,start_point);
		if (found_size!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_p=model_parameter.find(pattern, found_size);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";
		found_x=model_parameter.find_first_not_of(pattern, found_p+1);
		if (found_x!=std::string::npos)
			ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";	
		found_p=model_parameter.find_first_of(pattern, found_x);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_x = found_p - found_x;
		std::string x = model_parameter.substr(found_x, length_x);
		ROS_DEBUG("x: %s, real_length_x: %d", x.c_str(), int(length_x));

		double x_d = strtod(x.c_str(), NULL);
		ROS_DEBUG("x dimension: %f", x_d);



		pattern = " ";
		found_y=model_parameter.find_first_not_of(pattern, found_p);
		if (found_y!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		found_p=model_parameter.find_first_of(pattern, found_y);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_y = found_p - found_y;
		std::string y = model_parameter.substr(found_y, length_y);
		ROS_DEBUG("y: %s, real_length_y: %d", y.c_str(), int(length_y));

		double y_d = strtod(y.c_str(), NULL);
		ROS_DEBUG("y dimension: %f", y_d);

		pattern = " ";
		found_z=model_parameter.find_first_not_of(pattern, found_p);
		if (found_z!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = "<";	
		found_p=model_parameter.find_first_of(pattern, found_z);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_z = found_p - found_z;
		std::string z = model_parameter.substr(found_z, length_z);
		ROS_DEBUG("z: %s, real_length_z: %d", z.c_str(), int(length_z));

		double z_d = strtod(z.c_str(), NULL);
		ROS_DEBUG("z dimension: %f", z_d);

		dimensions.push_back(x_d);
		dimensions.push_back(y_d);
		dimensions.push_back(z_d);
			
		return true;
	}


	bool parse_box_location(std::string model_parameter, std::vector< double > &location, std::size_t start_point)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_size, found_p, found_x, found_y, found_z;

		pattern = "xyz";
		found_size=model_parameter.find(pattern,start_point);
		if (found_size!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_p=model_parameter.find(pattern, found_size);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";
		found_x=model_parameter.find_first_not_of(pattern, found_p+1);
		if (found_x!=std::string::npos)
			ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";	
		found_p=model_parameter.find_first_of(pattern, found_x);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_x = found_p - found_x;
		std::string x = model_parameter.substr(found_x, length_x);
		ROS_DEBUG("x: %s, real_length_x: %d", x.c_str(), int(length_x));

		double x_d = strtod(x.c_str(), NULL);
		ROS_DEBUG("x location: %f", x_d);



		pattern = " ";
		found_y=model_parameter.find_first_not_of(pattern, found_p);
		if (found_y!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		found_p=model_parameter.find_first_of(pattern, found_y);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_y = found_p - found_y;
		std::string y = model_parameter.substr(found_y, length_y);
		ROS_DEBUG("y: %s, real_length_y: %d", y.c_str(), int(length_y));

		double y_d = strtod(y.c_str(), NULL);
		ROS_DEBUG("y location: %f", y_d);


		pattern = " ";
		found_z=model_parameter.find_first_not_of(pattern, found_p);
		if (found_z!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = "<";	
		found_p=model_parameter.find_first_of(pattern, found_z);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_z = found_p - found_z;
		std::string z = model_parameter.substr(found_z, length_z);
		ROS_DEBUG("z: %s, real_length_z: %d", z.c_str(), int(length_z));

		double z_d = strtod(z.c_str(), NULL);
		ROS_DEBUG("z location: %f", z_d);

		location.push_back(x_d);
		location.push_back(y_d);
		location.push_back(z_d);
			
		return true;
	}
	

	bool parse_box_rotation(std::string model_parameter, std::vector< double > &rotation, std::size_t start_point)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_size, found_p, found_x, found_y, found_z;

		pattern = "rpy";
		found_size=model_parameter.find(pattern,start_point);
		if (found_size!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_p=model_parameter.find(pattern, found_size);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";
		found_x=model_parameter.find_first_not_of(pattern, found_p+1);
		if (found_x!=std::string::npos)
			ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";	
		found_p=model_parameter.find_first_of(pattern, found_x);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_x = found_p - found_x;
		std::string x = model_parameter.substr(found_x, length_x);
		ROS_DEBUG("r: %s, real_length_r: %d", x.c_str(), int(length_x));

		double x_d = strtod(x.c_str(), NULL);
		ROS_DEBUG("r rotation: %f", x_d);

		pattern = " ";
		found_y=model_parameter.find_first_not_of(pattern, found_p);
		if (found_y!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		found_p=model_parameter.find_first_of(pattern, found_y);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_y = found_p - found_y;
		std::string y = model_parameter.substr(found_y, length_y);
		ROS_DEBUG("p: %s, real_length_p: %d", y.c_str(), int(length_y));

		double y_d = strtod(y.c_str(), NULL);
		ROS_DEBUG("p rotation: %f", y_d);


		pattern = " ";
		found_z=model_parameter.find_first_not_of(pattern, found_p);
		if (found_z!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = "<";	
		found_p=model_parameter.find_first_of(pattern, found_z);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_z = found_p - found_z;
		std::string z = model_parameter.substr(found_z, length_z);
		ROS_DEBUG("y: %s, real_length_y: %d", z.c_str(), int(length_z));

		double z_d = strtod(z.c_str(), NULL);
		ROS_DEBUG("y rotation: %f", z_d);

		rotation.push_back(x_d);
		rotation.push_back(y_d);
		rotation.push_back(z_d);
			
		return true;
	}
	

	bool compose_box(std::string model_name, std::vector< double > dimensions, std::vector< double > &location, std::vector< double > &rotation, arm_navigation_msgs::CollisionObject &collision_object, std::string name)
	{
		
		gazebo::GetModelState state_srv;

		state_srv.request.model_name = model_name;
		double x0,y0,z0,x,y,z;
		double r,p,yy,roll,pitch,yaw;
		double qx,qy,qz,qw,q0,q1,q2,q3;
		double newx,newy,newz;

		if (m_state_client.call(state_srv))
		{

			ROS_INFO("Object Dimensions (x,y,z): (%f,%f,%f)",dimensions[0],dimensions[1],dimensions[2]);

			//location of object origin
			x0 = state_srv.response.pose.position.x;
			y0 = state_srv.response.pose.position.y;
			z0 = state_srv.response.pose.position.z;
			ROS_INFO("Origin Location (x,y,z): (%f,%f,%f)",x0, y0, z0);

			//rotation of the object origin
			qx = state_srv.response.pose.orientation.x;
			qy = state_srv.response.pose.orientation.y;
			qz = state_srv.response.pose.orientation.z;
			qw = state_srv.response.pose.orientation.w;
			ROS_INFO("Origin orientation(x,y,z,w): (%f,%f,%f,%f)", qx,qy,qz,qw);

			//convert quat to rpy
			roll = atan2(2*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
			pitch = asin(-2*(qx*qz - qw*qy));
			yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
		//	roll = atan2(-2*(qy*qz - qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
		//	pitch = asin(2*(qx*qz + qw*qy));
		//	yaw = atan2(-2*(qx*qy - qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);	

			ROS_INFO("Origin RPY (r,p,y): (%f,%f,%f)", roll,pitch,yaw);

			//object's translation from origin
			x = location[0];
			y = location[1];
			z = location[2];
			ROS_INFO("Object Translation from Origin (x,y,z): (%f,%f,%f)", x,y,z);

			//for positions of objects not located at center.....need to use transfomation matrix
			newx=x*(cos(pitch)*cos(yaw)) + y*(-cos(roll)*sin(yaw)-sin(roll)*sin(pitch)*cos(yaw)) + z*(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw));
			newy=x*(cos(pitch)*sin(yaw)) + y*(cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw)) + z*(-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw));
			newz=x*(-sin(pitch)) + y*(sin(roll)*cos(pitch)) + z*(cos(roll)*cos(pitch));
			ROS_INFO("NEW Object translation from Origin (x,y,z): (%f,%f,%f)", newx,newy,newz);
			
			//add translation of object origin
			x = x0 + newx;
			y = y0 + newy;
			z = z0 + newz;
			ROS_INFO("Object actual Location (x,y,z): (%f,%f,%f)", x,y,z);

			//roll pitch yaw of object from origin in RADIANS!
			r = rotation[0]*M_PI/180;
			p = rotation[1]*M_PI/180;
			yy = rotation[2]*M_PI/180;
			ROS_INFO("Object Rotation from Origin (r,p,y): (%f,%f,%f)", r,p,yy);

			//add the rotation of the object to the rotation of the origin
			r = r+roll;
			p = p+pitch;
			yy = yy+yaw;
			ROS_INFO("Object absolute orientation (r,p,y): (%f,%f,%f)", r,p,yy);
	
			//convert rpy to quaternian
			q0=sin(r/2)*cos(p/2)*cos(yy/2) - cos(r/2)*sin(p/2)*sin(yy/2);  	//x
			q1=cos(r/2)*sin(p/2)*cos(yy/2) + sin(r/2)*cos(p/2)*sin(yy/2);  	//y
			q2=cos(r/2)*cos(p/2)*sin(yy/2) - sin(r/2)*sin(p/2)*cos(yy/2);	//z
			q3=cos(r/2)*cos(p/2)*cos(yy/2) + sin(r/2)*sin(p/2)*sin(yy/2);	//w
		//	q0=sin(r/2)*cos(p/2)*cos(yy/2) + cos(r/2)*sin(p/2)*sin(yy/2);  	//x
		//	q1=cos(r/2)*sin(p/2)*cos(yy/2) - sin(r/2)*cos(p/2)*sin(yy/2);  	//y
		//	q2=cos(r/2)*cos(p/2)*sin(yy/2) + sin(r/2)*sin(p/2)*cos(yy/2);	//z
		//	q3=cos(r/2)*cos(p/2)*cos(yy/2) - sin(r/2)*sin(p/2)*sin(yy/2);	//w
			ROS_INFO("Object quaternian(x,y,z,w): (%f,%f,%f,%f)",q0,q1,q2,q3);

			//normalize the quaternian
			double mag = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
			q0 = q0/mag;
			q1 = q1/mag;
			q2 = q2/mag;
			q3 = q3/mag;
			ROS_INFO("Normalized Object orientation(x,y,z,w): (%f,%f,%f,%f)", q0,q1,q2,q3);

		}
		else
		{
			ROS_ERROR("Failed to call service get_model_state");
			return false;
		}

		collision_object.id = model_name;

		//used to add multiple boxes to one object
		double shapes_size = collision_object.shapes.size();

		collision_object.header.frame_id = frame_id;
		collision_object.header.stamp = ros::Time::now();
		collision_object.shapes.resize(shapes_size+1);
		collision_object.poses.resize(shapes_size+1);

		//ToDo: figure out how *.model-size and *.urdf-extend are related

		//*.model origin is located at center of box
		collision_object.shapes[shapes_size].type = arm_navigation_msgs::Shape::BOX;
		collision_object.shapes[shapes_size].dimensions.push_back(dimensions[0]);
		collision_object.shapes[shapes_size].dimensions.push_back(dimensions[1]);
		collision_object.shapes[shapes_size].dimensions.push_back(dimensions[2]);

		collision_object.poses[shapes_size].position.x = x;
		collision_object.poses[shapes_size].position.y = y;
		collision_object.poses[shapes_size].position.z = z;//+(dimensions[2]/2.0);
		collision_object.poses[shapes_size].orientation.x = q0;
		collision_object.poses[shapes_size].orientation.y = q1;
		collision_object.poses[shapes_size].orientation.z = q2;
		collision_object.poses[shapes_size].orientation.w = q3;
	
		return true;
	}
};

#endif
