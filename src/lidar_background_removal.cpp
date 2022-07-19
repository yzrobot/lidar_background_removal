// Copyright (C) 2022, Zhi Yan

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#define __APP_NAME__ "lidar_background_removal"

std::string map_frame_;
float inflation_;
ros::Publisher lidar_filtered_pub_;
tf::TransformListener *listener_;
nav_msgs::OccupancyGrid::ConstPtr map_;

bool lookupTransform(const std::string &source_frame, tf::StampedTransform &transform) {
  try {
    ros::Time now = ros::Time::now();
    listener_->waitForTransform(map_frame_, source_frame, now, ros::Duration(0.1));
    listener_->lookupTransform(map_frame_, source_frame, now, transform);
  } catch(tf::TransformException ex) {
    ROS_ERROR("-------> %s", ex.what());
    return false;
  }
  
  return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  tf::StampedTransform transform;
  
  if(lookupTransform(scan->header.frame_id, transform)) {
    float angle;
    geometry_msgs::Point p;
    tf::Point point_in_lidar, point_in_map;
    sensor_msgs::LaserScan scan_filtered = *scan;
    
    for(int i = 0; i < scan->ranges.size(); i++) {
      angle = scan->angle_min + (i * scan->angle_increment);
      p.x = scan->ranges[i] * cos(angle);
      p.y = scan->ranges[i] * sin(angle);
      
      tf::pointMsgToTF(p, point_in_lidar);
      point_in_map = transform * point_in_lidar;

      float x = (point_in_map.getX() - map_->info.origin.position.x) / map_->info.resolution;
      float y = (point_in_map.getY() - map_->info.origin.position.y) / map_->info.resolution;

      for(int j = x - inflation_; j <= x + inflation_; j++) {
	for(int k = y - inflation_; k <= y + inflation_; k++) {
	  int idx = j + k * (int)map_->info.width;
	  if(idx >= map_->data.size()) {
	    scan_filtered.ranges[i] = NAN;
	    scan_filtered.intensities[i] = NAN;
	  } else {
	    if(map_->data[idx] == 100 || map_->data[idx] == -1) {
	      scan_filtered.ranges[i] = NAN;
	      scan_filtered.intensities[i] = NAN;
	      k = y + inflation_;
	      j = x + inflation_;
	    }
	  }
	}
      }
    }
    
    lidar_filtered_pub_.publish(scan_filtered);
  }
}

void pointCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
  tf::StampedTransform transform;
  
  if(lookupTransform(cloud->header.frame_id, transform)) {
    geometry_msgs::Point p;
    tf::Point point_in_lidar, point_in_map;
    sensor_msgs::PointCloud pc;
    sensor_msgs::PointCloud2 cloud_filtered;
    
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud, pc);
    
    for(int i = 0; i < pc.points.size(); i++) {
      p.x = pc.points[i].x;
      p.y = pc.points[i].y;
      
      tf::pointMsgToTF(p, point_in_lidar);
      point_in_map = transform * point_in_lidar;
      
      float x = (point_in_map.getX() - map_->info.origin.position.x) / map_->info.resolution;
      float y = (point_in_map.getY() - map_->info.origin.position.y) / map_->info.resolution;

      for(int j = x - inflation_; j <= x + inflation_; j++) {
	for(int k = y - inflation_; k <= y + inflation_; k++) {
	  int idx = j + k * (int)map_->info.width;
	  if(idx >= map_->data.size()) {
	    pc.points[i].x = NAN;
	    pc.points[i].y = NAN;
	    pc.points[i].z = NAN;
	  } else {
	    if(map_->data[idx] == 100 || map_->data[idx] == -1) {
	      pc.points[i].x = NAN;
	      pc.points[i].y = NAN;
	      pc.points[i].z = NAN;
	      k = y + inflation_;
	      j = x + inflation_;
	    }
	  }
	}
      }
    }
    
    sensor_msgs::convertPointCloudToPointCloud2(pc, cloud_filtered);
    lidar_filtered_pub_.publish(cloud_filtered);
  }
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, __APP_NAME__);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  bool three_d;
  private_nh.param<bool>("three_d", three_d, false);
  private_nh.param<std::string>("map_frame", map_frame_, "/map");
  private_nh.param<float>("inflation", inflation_, 2.0);
  
  listener_ = new tf::TransformListener();
  ROS_WARN("[%s] Please make sure that the map is loaded and the localization module (e.g. amcl) is launched.", __APP_NAME__);
  
  map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");
  ROS_WARN("[%s] Map received!", __APP_NAME__);
  
  ros::Subscriber lidar_sub;
  if(three_d) {
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, pointCallback);
    lidar_filtered_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
  } else {
    lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserCallback);
    lidar_filtered_pub_ = private_nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
  }
  
  ros::spin();
  
  return 0;
}
