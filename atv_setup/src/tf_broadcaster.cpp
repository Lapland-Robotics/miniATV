#include "atv_tf_Parameters.h"  // My own header file for tf Parameters
#include <ros/ros.h>
// tf package provides an implementation of a tf::TransformBroadcaster to help make the task of publishing 
// transforms easier. To use the TransformBroadcaster, we need to include the tf/transform_broadcaster.h
// header file.
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "atv_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(10);
// Create a TransformBroadcaster object that we'll use later to send the base_link → base_laser transform 
// over the wire.
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
// Real work. Sending a transform with a TransformBroadcaster requires five arguments. First, we pass in
// the rotation transform, which is specified by a btQuaternion for any rotation that needs to occur
// between the two coordinate frames. In this case, we want to apply no rotation, so we send in a
// btQuaternion constructed from pitch, roll, and yaw values equal to zero. 
    broadcaster.sendTransform(
      tf::StampedTransform(
// btVector3 for any ranslation that we'd like to apply. We do, however, want to apply a translation,
// so we create a btVector3 corresponding to the laser's x offset of 10cm and z offset of 20cm from the
// robot base.
// Ignore above if offsets = 0
// Or use constant from header file
        tf::Transform(tf::Quaternion(Laser_to_Base_x_rot, Laser_to_Base_y_rot, Laser_to_Base_z_rot, 1), tf::Vector3(Laser_to_Base_x, Laser_to_Base_y, Laser_to_Base_z)),
// Transform being published a timestamp and pass the name of the parent node "base_link"
// and pass the name of the child node "base_laser"
        ros::Time::now(),"base_link", "base_laser_link"));
    r.sleep();
  }
}

