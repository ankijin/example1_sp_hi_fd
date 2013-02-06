#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char TOPIC_NAME[] = "/sn_kinect1/rgb/image_color";

image_transport::Subscriber sn_kinect_rgb;

CvHaarClassifierCascade *cascade;
CvMemStorage *storage;
CvSeq *faces;
IplImage frame_copy;
HOGDescriptor hog;
ros::Publisher pub;



void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
{
	CvRect* r = 0;
	cv_bridge::CvImagePtr cv_ptr;
	try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}

	frame_copy = cv_ptr->image;
	faces = cvHaarDetectObjects(&frame_copy, cascade, storage, 1.2, 1, CV_HAAR_DO_CANNY_PRUNING, cvSize(10, 0));
	for(int i = 0; i < (faces ? faces->total : 0); i++) {
		CvRect *r = (CvRect*)cvGetSeqElem(faces, i);
		CvPoint pt1 = { r->x, r->y };
		CvPoint pt2 = { r->x + r->width,
				r->y + r->height };
		cvRectangle(&frame_copy, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
	}


	if( faces && faces->total )
	{
		r = (CvRect*)cvGetSeqElem(faces, 0);


		std_msgs::Int32MultiArray array;
		//Clear array
		array.data.clear();

		array.data.push_back(r->height);
		array.data.push_back(r->width);
		array.data.push_back(r->y);
		array.data.push_back(r->x);
		//Publish array
		pub.publish(array);
		sleep(2);
	}
	cvShowImage(TOPIC_NAME,&frame_copy);
	cv::waitKey(3);
}

void exitExample(){
	cv::destroyWindow(TOPIC_NAME);
	cvReleaseHaarClassifierCascade(&cascade);
	cvReleaseMemStorage(&storage);
}


int main(int argc, char** argv)
{
	cv::namedWindow(TOPIC_NAME);
	cv::resizeWindow(TOPIC_NAME,640,480);
	cv::moveWindow(TOPIC_NAME,0,0);
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	cascade = (CvHaarClassifierCascade *) cvLoad("example1_sp_hi_fd/haarcascade_frontalface_alt.xml", 0, 0, 0);
	storage = cvCreateMemStorage(0);

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(TOPIC_NAME, 100, imageCb_rgb);
	pub = n.advertise<std_msgs::Int32MultiArray>("face_detected", 100);
	ros::spin();
	//	exitExample();
	return 0;
}
