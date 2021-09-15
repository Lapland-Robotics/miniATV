#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
// tf/transform_listener.h header file that we'll need to create a tf::TransformListener.
// A TransformListener object automatically subscribes to the transform message topic over ROS and manages
// all transform data coming in over the wire.
#include <tf/transform_listener.h>

// Create a function that, given a TransformListener, takes a point in the "base_laser" frame and
// transforms it to the "base_link" frame. This function will serve as a callback for the ros::Timer
// created in the main() of our program and will fire every second.
void transformPoint(const tf::TransformListener& listener){
  //create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //some data for the point, arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{
// Point in the "base_laser" frame we want to transform it into the "base_link" frame. Use the
// TransformListener object, and call transformPoint() with three arguments: the name of 
// the frame we want to transform the point to ("base_link" in our case), the point we're transforming,
// and storage for the transformed point. So, after the call to transformPoint(), base_point holds
// the same information as laser_point did before only now in the "base_link" frame.
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
// If for some reason the base_laser → base_link transform is unavailable (perhaps the tf_broadcaster is
// not running), the TransformListener may throw an exception when we call transformPoint().
// To make sure we handle this gracefully, catch the exception and print out an error for the user.
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}

