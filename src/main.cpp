#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW1[] = "/sn_kinect1/rgb/image_color";


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_kinect1_rgb,sub_kinect1_depth;
	image_transport::Publisher image_pub_;

	CvHaarClassifierCascade *cascade;
	CvMemStorage *storage;
	CvSeq *faces;
	IplImage* frame;
	IplImage frame1;
	HOGDescriptor hog;
	int i;
	char c;
public:
	ImageConverter()
	: it_(nh_)
	{

		cv::namedWindow(WINDOW1);
		cv::resizeWindow(WINDOW1,640,480);
		cv::moveWindow(WINDOW1,0,0);
		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());


		sub_kinect1_rgb = it_.subscribe("/ATRV/CameraMain/image", 1, &ImageConverter::imageCb_rgb1, this);
		cascade = (CvHaarClassifierCascade *) cvLoad("example1_sp_hi_fd/haarcascade_frontalface_alt.xml", 0, 0, 0);
		storage = cvCreateMemStorage(0);

	}

	~ImageConverter()
	{
		cv::destroyWindow(WINDOW1);
		cvReleaseHaarClassifierCascade(&cascade);
		cvReleaseMemStorage(&storage);


	}

	void imageCb_rgb1(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
		catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}

		frame1 = cv_ptr->image;
		faces = cvHaarDetectObjects(&frame1, cascade, storage, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(0, 0));
		for(i = 0; i < (faces ? faces->total : 0); i++) {
			CvRect *r = (CvRect*)cvGetSeqElem(faces, i);
			CvPoint pt1 = { r->x, r->y };
			CvPoint pt2 = { r->x + r->width,
					r->y + r->height };
			cvRectangle(&frame1, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
		}
		cvShowImage(WINDOW1,&frame1);
		cv::waitKey(3);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter* ic = new ImageConverter();
	ros::spin();
	return 0;
}
