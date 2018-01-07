#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int error_y=0;
int zoom=1;
image_transport::Publisher image_pub_;

void zoom_callback(const std_msgs::Int32& int_msg){
	zoom=int_msg.data;
}

int diagnostic_counter=0;
void imageCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_in_ptr;
	cv_bridge::CvImagePtr cv_out_ptr;
	try
	{
	  cv_in_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	int img_height = cv_in_ptr->image.rows;
	int img_width  = cv_in_ptr->image.cols;
	int center_y = img_height/2;
	int center_x = img_width/2;
	cv::Mat cv_out_img;
	if(zoom==2){
		cv::Mat cv_cut_img(cv_in_ptr->image,cv::Rect(img_width/4, img_height/4+error_y, img_width/2, img_height/2));
		cv::resize(cv_cut_img, cv_out_img,cv::Size(),1,1);

	}
	else if(zoom==4){
		cv::Mat cv_cut_img(cv_in_ptr->image,cv::Rect(img_width*3/8, img_height*3/8+error_y, img_width/4, img_height/4));
		cv::resize(cv_cut_img, cv_out_img,cv::Size(),2,2);
	}
	else{//zoom x1
		cv::resize(cv_in_ptr->image, cv_out_img,cv::Size(),0.5,0.5);	
	}

	// Output modified video stream
	sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_out_img).toImageMsg();
	image_pub_.publish(out_msg);

	diagnostic_counter=0;
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(diagnostic_counter<10){
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
	}
	else{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Data.");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cv_zoom");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	image_transport::ImageTransport it_(n);

	//param
	pn.getParam("error_y", error_y);

	//publish
	image_pub_ = it_.advertise("image_output", 1);

	//Subscribe
	image_transport::Subscriber image_sub_ = it_.subscribe("image_input", 1, imageCb);
	ros::Subscriber joy_sub     = n.subscribe("zoom", 10, zoom_callback);

	//Diagnostic
	diagnostic_updater::Updater updater;
	updater.setHardwareID("ZoomCameraModule");
	updater.add("Active", diagnostic0);

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		updater.update();
		diagnostic_counter++;		
		ros::spinOnce();
		loop_rate.sleep();
	}

}
