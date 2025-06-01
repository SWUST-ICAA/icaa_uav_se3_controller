#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

def create_circular_path(radius=2.0, height=1.0, num_points=20):
    """Create a circular path for testing"""
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        
        # Position
        pose.pose.position.x = radius * math.cos(angle)
        pose.pose.position.y = radius * math.sin(angle)
        pose.pose.position.z = height
        
        # Orientation (facing direction of motion)
        yaw = angle + math.pi/2
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        
        path.poses.append(pose)
    
    # Add first point again to close the circle
    path.poses.append(path.poses[0])
    
    return path

def create_square_path(side_length=3.0, height=1.0):
    """Create a square path for testing"""
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    
    # Define corners of the square
    corners = [
        (side_length/2, side_length/2),
        (-side_length/2, side_length/2),
        (-side_length/2, -side_length/2),
        (side_length/2, -side_length/2),
        (side_length/2, side_length/2)  # Close the square
    ]
    
    for i, (x, y) in enumerate(corners):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        
        # Position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = height
        
        # Orientation (facing next corner)
        if i < len(corners) - 1:
            dx = corners[i+1][0] - x
            dy = corners[i+1][1] - y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0
            
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        
        path.poses.append(pose)
    
    return path

def create_figure8_path(size=2.0, height=1.0, num_points=40):
    """Create a figure-8 path for testing"""
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    
    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        
        # Position (figure-8 parametric equations)
        pose.pose.position.x = size * math.sin(t)
        pose.pose.position.y = size * math.sin(t) * math.cos(t)
        pose.pose.position.z = height
        
        # Orientation (facing direction of motion)
        if i < num_points - 1:
            t_next = 2 * math.pi * (i + 1) / num_points
            dx = size * math.sin(t_next) - pose.pose.position.x
            dy = size * math.sin(t_next) * math.cos(t_next) - pose.pose.position.y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0
            
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(yaw/2)
        pose.pose.orientation.w = math.cos(yaw/2)
        
        path.poses.append(pose)
    
    # Add first point again to close the path
    path.poses.append(path.poses[0])
    
    return path

def main():
    rospy.init_node('test_path_publisher')
    
    # Get parameters
    path_type = rospy.get_param('~path_type', 'circle')  # circle, square, or figure8
    publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
    path_height = rospy.get_param('~height', 1.0)  # meters
    path_size = rospy.get_param('~size', 2.0)  # meters
    
    # Create publisher
    path_pub = rospy.Publisher('/trajectory_path', Path, queue_size=10)
    
    # Create path based on type
    if path_type == 'circle':
        path = create_circular_path(radius=path_size, height=path_height)
        rospy.loginfo("Publishing circular path with radius %.1f m at height %.1f m", path_size, path_height)
    elif path_type == 'square':
        path = create_square_path(side_length=path_size, height=path_height)
        rospy.loginfo("Publishing square path with side length %.1f m at height %.1f m", path_size, path_height)
    elif path_type == 'figure8':
        path = create_figure8_path(size=path_size, height=path_height)
        rospy.loginfo("Publishing figure-8 path with size %.1f m at height %.1f m", path_size, path_height)
    else:
        rospy.logerr("Unknown path type: %s. Using circle.", path_type)
        path = create_circular_path(radius=path_size, height=path_height)
    
    # Publish path periodically
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass