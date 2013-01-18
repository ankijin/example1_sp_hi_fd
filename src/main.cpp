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

//static const char WINDOW1[] = "/camera/rgb/image_color";
//static const char WINDOW1[] = "/ATRV/CameraMain/image";
//static const char WINDOW2[] = "/camera/depth/image_raw";
static const char WINDOW1[] = "/sn_kinect1/rgb/image_color";
static const char WINDOW2[] = "/sn_kinect2/rgb/image_color";



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



	//	CvCapture* capture;
	//	IplImage* frame;
	//	capture = cvCreateFileCapture("test.avi");

	char c;
public:
	ImageConverter()
	: it_(nh_)
	{

		cv::namedWindow(WINDOW1);
		//  cv::namedWindow(WINDOW2);




		cv::resizeWindow(WINDOW1,640,480);
		//	cv::resizeWindow(WINDOW2,640,480);

		hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

		cv::moveWindow(WINDOW1,0,0);
		//	cv::moveWindow(WINDOW2,640,0);

		//cv::waitKey(3);

	    //sub_kinect1_rgb = it_.subscribe("/sn_kinect2/rgb/image_color", 1, &ImageConverter::imageCb_rgb1, this);
		sub_kinect1_rgb = it_.subscribe("/ATRV/CameraMain/image", 1, &ImageConverter::imageCb_rgb1, this);

//		sub_kinect1_rgb = it_.subscribe("/ATRV/CameraMain/image", 1, &ImageConverter::findFaces, this);
	//	sub_kinect1_rgb = it_.subscribe("/ATRV/CameraMain/image", 1, &ImageConverter::findBodies, this);

		//    sub_kinect2_rgb = it_.subscribe("/sn_kinect2/rgb/image_color", 1, &ImageConverter::imageCb_rgb2, this);
		// sub_kinect1_depth = it_.subscribe("/sn_kinect2/rgb/image_color", 1, &ImageConverter::imageCb_depth1, this);

		cascade = (CvHaarClassifierCascade *) cvLoad("/home/kijin/rosWorkspace/tutorialROSOpenCV/haarcascade_frontalface_alt.xml", 0, 0, 0);
		storage = cvCreateMemStorage(0);

	}

	~ImageConverter()
	{
		cv::destroyWindow(WINDOW1);
		//cv::destroyWindow(WINDOW2);
		cvReleaseHaarClassifierCascade(&cascade);
		cvReleaseMemStorage(&storage);


	}
/*
	void findFaces(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr;
		try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
		catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}

		frame = &IplImage(cv_ptr->image);
		faces = cvHaarDetectObjects(frame, cascade, storage, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(0, 0));
		for(i = 0; i < (faces ? faces->total : 0); i++) {
			CvRect *r = (CvRect*)cvGetSeqElem(faces, i);
			CvPoint pt1 = { r->x, r->y };
			CvPoint pt2 = { r->x + r->width,
					r->y + r->height };
			cvRectangle(frame, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
		}
		cvShowImage(WINDOW1,frame);


		//cv::imshow(WINDOW1, cv_ptr->image);
		cv::waitKey(3);
	}
*/
/*
	void findBodies(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr;
		try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
		catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}

		frame = &IplImage(cv_ptr->image);

		vector<Rect> found, found_filtered;
		hog.detectMultiScale(frame, found, 0, Size(8,8), Size(32,32), 1.05, 2);

		size_t i, j;
		for (i=0; i<found.size(); i++)
		{
			Rect r = found[i];
			for (j=0; j<found.size(); j++)
				if (j!=i && (r & found[j])==r)
					break;
			if (j==found.size())
				found_filtered.push_back(r);
		}


		for (i=0; i<found_filtered.size(); i++)
		{
			Rect r = found_filtered[i];
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.06);
			r.height = cvRound(r.height*0.9);
			cvRectangle(frame, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
			//cvRectangle(frame, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
		}
		cvShowImage(WINDOW1,frame);
		cv::waitKey(3);

	}
*/
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
/*
	void imageCb_rgb2(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);}
		catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}

		frame = &IplImage(cv_ptr->image);
		faces = cvHaarDetectObjects(frame, cascade, storage, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(0, 0));
		for(i = 0; i < (faces ? faces->total : 0); i++) {
			CvRect *r = (CvRect*)cvGetSeqElem(faces, i);
			CvPoint pt1 = { r->x, r->y };
			CvPoint pt2 = { r->x + r->width,
					r->y + r->height };
			cvRectangle(frame, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
		}
		cvShowImage(WINDOW1,frame);


		//cv::imshow(WINDOW1, cv_ptr->image);
		cv::waitKey(3);
	}

*/








	void imageCb_depth1(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);}
		catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}
		cv::imshow(WINDOW2, cv_ptr->image);
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
