#include <string>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

tf2_ros::Buffer tfBuffer;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> chessboard_tf_ptr;
tf::StampedTransform ts_aruco_100;
tf::StampedTransform ts_aruco_101;
tf::StampedTransform ts_aruco_102;
tf::StampedTransform ts_aruco_103;

geometry_msgs::TransformStamped camera_left_in_world;
geometry_msgs::TransformStamped camera_right_in_world;

geometry_msgs::TransformStamped getTransformByName(std::string __reference, std::string __frame_id, tf2_ros::Buffer& __buffer)
{
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.frame_id = __reference;
	transformStamped.child_frame_id = __frame_id;
	transformStamped.transform.rotation.w = 1;

	try
	{
		transformStamped = __buffer.lookupTransform(__frame_id, __reference, ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	return transformStamped;
}

void pubChessFrames(const ros::TimerEvent& __e)
{
	geometry_msgs::TransformStamped left_aruco_100 = getTransformByName("camera_left_aruco_100", "camera_left_link", tfBuffer);
	geometry_msgs::TransformStamped left_aruco_102 = getTransformByName("camera_left_aruco_102", "camera_left_link", tfBuffer);
	geometry_msgs::TransformStamped right_aruco_101 = getTransformByName("camera_right_aruco_101", "camera_right_link", tfBuffer);
	geometry_msgs::TransformStamped right_aruco_103 = getTransformByName("camera_right_aruco_103", "camera_right_link", tfBuffer);


	tf::Transform left_aruco_100_in_camera;
	tf::transformMsgToTF(left_aruco_100.transform, left_aruco_100_in_camera);

	tf::Transform camera_left_in_world_tf;
	camera_left_in_world_tf = ts_aruco_100*left_aruco_100_in_camera.inverse();

	tf::transformTFToMsg(camera_left_in_world_tf, camera_left_in_world.transform);


	tf::Transform right_aruco_101_in_camera;
	tf::transformMsgToTF(right_aruco_101.transform, right_aruco_101_in_camera);

	tf::Transform camera_right_in_world_tf;
	camera_right_in_world_tf = ts_aruco_101*right_aruco_101_in_camera.inverse();

	tf::transformTFToMsg(camera_right_in_world_tf, camera_right_in_world.transform);


	chessboard_tf_ptr->sendTransform(camera_left_in_world);
	chessboard_tf_ptr->sendTransform(camera_right_in_world);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cameras_calibration");
	std::cout << "CALIBRATION STARTED" << std::endl;

	ros::NodeHandle nh;

	chessboard_tf_ptr.reset(new tf2_ros::StaticTransformBroadcaster);

	ts_aruco_100.frame_id_ = "world";
	ts_aruco_100.child_frame_id_ = "camera_left_aruco_100";
	ts_aruco_100.setData(tf::Transform( tf::Quaternion(0, 0.7071, 0.7071, 0), tf::Vector3(-0.216, -0.216, 0.0) ) );

	ts_aruco_102.frame_id_ = "world";
	ts_aruco_102.child_frame_id_ = "camera_left_aruco_102";
	ts_aruco_102.setData(tf::Transform( tf::Quaternion(0, 0.7071, 0.7071, 0), tf::Vector3(0.216, -0.216, 0.0) ) );

	ts_aruco_101.frame_id_ = "world";
	ts_aruco_101.child_frame_id_ = "camera_left_aruco_101";
	ts_aruco_101.setData(tf::Transform( tf::Quaternion(0, 0.7071, 0.7071, 0), tf::Vector3(-0.216, 0.216, 0.0) ) );

	ts_aruco_103.frame_id_ = "world";
	ts_aruco_103.child_frame_id_ = "camera_left_aruco_103";
	ts_aruco_103.setData(tf::Transform( tf::Quaternion(0, 0.7071, 0.7071, 0), tf::Vector3(0.216, 0.216, 0.0) ) );

	camera_left_in_world.header.frame_id = "world";
	camera_left_in_world.child_frame_id = "camera_left_link";
	camera_left_in_world.transform.rotation.w = 1;

	camera_right_in_world.header.frame_id = "world";
	camera_right_in_world.child_frame_id = "camera_right_link";
	camera_right_in_world.transform.rotation.w = 1;

	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Timer chessboard_timer = nh.createTimer(ros::Duration(2), pubChessFrames);

	ros::spin();

	return 0;
};
