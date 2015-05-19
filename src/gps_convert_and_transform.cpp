/* Copyright (c) 2012, EJ Kreinar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ********************************************************************************
 * convert_gps_to_pose.cpp
 *   Converts a GPS NavSatFix message to a PoseWithCovarianceStamped
 *   (But, I could also publish a PoseStamped which can be visualized in Rviz)
 * 
 * Subscribes:
 *   - gps_fix (sensor_msgs/NavSatFix): Current GPS reading
 *   - odom (nav_msgs/Odometry): Odometry measurement
 *  
 * Publishes:
 *   - gps_pose (geometry_msgs/PoseStamped): Current state of the robot in the /map frame
 *
 * Broadcasts:
 *   - map->odom transform: Odometry correction
 *
 * Parameters:
 *   - ref_lat: Latitude reference point
 *   - ref_lon: Longitude reference point
 *   - rel_alt: Altitude reference point
 *   - ~gps: Topic name of the subscribed NavSatFix message
 *   - ~pose: Topic name of the published PoseWithCovarianceStamped message
 *
 ********************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include "GPS_Equations.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>

#define USE_COVARIANCE 0

class CutterGPSConversion
{
public:
  CutterGPSConversion();

private:
  // The callback function is called when a NavSatFix message is published to the topic gps_fix (default). The received message is the stored in the global variable fix_.
  void gpsCallback(const sensor_msgs::NavSatFix& gps);
  sensor_msgs::NavSatFix fix_;

  void publishState();

  // The following two variables a_ and mapOriginENU_ are used to transform the GPS measurement (after it's converted from LLA to ENU) into the map coordinate system. They will be calculated using GPS measurements of the origin of the map and a point along the y-axis of the map.
  // This is the rotation angle to align the vector in ENU with the map
  double a_;
  // This is the map's origin in ENU
  geometry_msgs::Point mapOriginENU_;

  // ROS stuff (the node, a publisher for the PoseWithCovarianceStamped (no orientation), and a subscriber for the NavSatFix)
  ros::NodeHandle node_;
  ros::Publisher  pose_pub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber vel_sub_;
 
};

CutterGPSConversion::CutterGPSConversion()
{
  // Get parameters - the lat, long, and alt are taken for a point at the origin of the map and a point along the y-axis of the map. These can be used to determine a_ and mapOriginENU_ that are used to transform points into the map coordinate system. These measurements are typically stored in a file called map_alignment_points.yaml.
  double originLat, originLon, originAlt, yPtLat, yPtLon, yPtAlt;
  if (!(node_.getParam("originLat",originLat) &&
        node_.getParam("originLon",originLon) &&
        node_.getParam("originAlt",originAlt) &&
	node_.getParam("yPtLat",yPtLat) &&
	node_.getParam("yPtLon",yPtLon) &&
	node_.getParam("yPtAlt",yPtLon)))
    {
      // Provide a default reference point if there's no reference params
      originLat = 0;
      originLon = 0;
      originAlt = 0;
      yPtLat = 1;
      yPtLon = 0;
      yPtAlt = 0;
      ROS_WARN("Reference parameters not found (orignLat, originLon, originAlt, yPtLat, yPtLon, yPtAlt). Using defaults");
    }

  // Messages to store the origin and the point along the y-axis in LLA
  sensor_msgs::NavSatFix originRef;
  sensor_msgs::NavSatFix yPtRef;
  // Make a Point called yPtENU to store the yPtENU in ENU
  geometry_msgs::Point yPtENU;
  // Make a Point called originPtENU to store the origin in ENU
  geometry_msgs::Point originPtENU;

  // Populate originRef and yPtRef with the parameters from above.
  originRef.latitude  = originLat;
  originRef.longitude = originLon;
  originRef.altitude  = originAlt;

  yPtRef.latitude = yPtLat;
  yPtRef.longitude = yPtLon;
  yPtRef.altitude = yPtLon;
  
  // Convert originRef_ from LLA to ENU and store in originPtENU. originPtENU goes in empty and comes back with the converted originRef.
  GPS_Equations::LLA2ENU(originRef,originPtENU);
  // Convert yPtRef_ from LLA to ENU and store in yPtENU. Again, yPtENU goes in empty and comes back with the converted yPtRef.
  GPS_Equations::LLA2ENU(yPtRef,yPtENU);
  // Now we have the originPt and yPt in ENU. If we consider the originPt and yPt to form a vector parallel to the y-axis in the MAP frame, then we can find the rotation angle necessary to transform robot coordinates from the ENU frame to the MAP frame.
  a_ = atan2(yPtENU.x-originPtENU.x,yPtENU.y-originPtENU.y);
  mapOriginENU_ = originPtENU;

  // The rest of the the algorithm works as such.
  // 1. Take a GPS reading of the robot.
  // 2. Convert it to ENU.
  // 3. Subtract the coordinates of the map's origin in ENU (originPtENU).
  // 4. Rotate it to get the coordinates of the robot in the map frame.

  // Get topic names to subscribe/publish
  std::string gps, pose, vel;
  if (!(ros::param::get("~gps",gps) &&
        ros::param::get("~pose",pose) &&
        ros::param::get("~vel",vel) ))
    {
      gps  = "gps_fix";
      pose = "gps_pose"; 
      vel  = "gps_vel";
      ROS_WARN("Reference parameters not found (~gps, ~pose, ~vel). Using defaults");
    }
  
  // Setup Publishers and subscribers
#if USE_COVARIANCE
  pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose,1);
#else
  pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>(pose,1);
#endif
  gps_sub_ = node_.subscribe(gps, 1, &CutterGPSConversion::gpsCallback, this);
}

void CutterGPSConversion::publishState()
{
  // Make a Point called rPtENU to store the robot position in ENU frame
  geometry_msgs::Point rPtENU;
  // and then convert fix_ from LLA to ENU and store in rPtENU
  GPS_Equations::LLA2ENU(fix_,rPtENU);

  // Now using the map origin (ENU) (mapOriginENU_), the robot location (ENU) (rPtENU), and the rotation angle (a_) to transform to the robot's location from the ENU frame to the map frame.
  geometry_msgs::Point rPtMap; // The robot's location in the map frame.
  rPtMap.x = (rPtENU.x-mapOriginENU_.x)*cos(a_)-(rPtENU.y-mapOriginENU_.y)*sin(a_);
  rPtMap.y = (rPtENU.x-mapOriginENU_.x)*sin(a_)+(rPtENU.y-mapOriginENU_.y)*cos(a_);

  // Now use rPtMap to populate the header, position, and covariance portions of a PoseWithCovarianceStamped message. (No orientation.)
  ros::Time current_time = ros::Time::now();  

#if USE_COVARIANCE
  // Pose message: Setup header
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = fix_.header.stamp;
  pose_msg.header.frame_id = "map";

  // Pose message: Set the pose
  pose_msg.pose.pose.position = rPtMap;

  // There are ways to determine velocity and heading from the GPS. However, these things are determined from a change in position. So, I think it makes sense to just use the position.

  // Pose message: Set the covariance
  // TODO: Set the covariance according to the NavSatFix message
  // spose_msg.pose.covariance = ....
#else
  // Pose message: Setup header
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = fix_.header.stamp;
  pose_msg.header.frame_id = "map";

  // Pose message: Set the pose
  pose_msg.pose.position = rPtMap;
#endif

  // Publish the state
  pose_pub_.publish(pose_msg);
  // ROS_INFO("Published pose");
}

void CutterGPSConversion::gpsCallback(const sensor_msgs::NavSatFix& gps)
{
  fix_ = gps;
  //ROS_INFO("Received gps fix");
  publishState();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_convert_and_transform");
  CutterGPSConversion conversion;
  ros::spin();
  
  return 0;
}
