#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <queue>
#include <vector>

static const char WINDOW_WORK[] = "Work";
static const char WINDOW_ORG[] = "Orginal";
static const char WINDOW_OUT[] = "Out";

cv::Mat camera_matrix;
std::vector<double> camera_distortion;
ros::Publisher chatter_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {   
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"  );
        cv::Mat sourceImage = cv_ptr->image.clone();
        cv::imshow(WINDOW_ORG, sourceImage);

        cv::Mat undistImage;
        cv::Mat new_camera_matrix = camera_matrix.clone();
        cv::undistort(sourceImage, undistImage, camera_matrix, cv::Mat(camera_distortion), new_camera_matrix);
        cv_ptr->image = undistImage;
        double fx = new_camera_matrix.ptr<double>(0)[0];
        double fy = new_camera_matrix.ptr<double>(1)[1];
        double cx = new_camera_matrix.ptr<double>(0)[2];
        double cy = new_camera_matrix.ptr<double>(1)[2];


        cv::Mat grayImage;
        cv::cvtColor(undistImage, grayImage, cv::COLOR_BGR2GRAY);
    
        cv::Mat thresholdImage;
        cv::threshold(grayImage, thresholdImage, 100, 255, cv::THRESH_BINARY);

        cv::Mat erodeImage;
        cv::erode(thresholdImage, erodeImage, cv::Mat(), cv::Point(-1, -1), 6, cv::BORDER_DEFAULT, 1);

        cv::Mat dilateImage;
        cv::dilate(erodeImage, dilateImage, cv::Mat(), cv::Point(-1, -1), 4, cv::BORDER_DEFAULT, 1);

        cv::imshow(WINDOW_WORK, dilateImage);

        cv::Mat countMap = dilateImage.clone();

        cv::Mat emptyImage;
        cv::threshold(grayImage, emptyImage, 255, 255, cv::THRESH_BINARY);

        std::vector < std::vector<cv::Point> > contours;
        cv::findContours(countMap, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        cv::Point2f center(0.f, 0.f);
        cv::Point3f output(0.f, 0.f, 0.f);
        float radius = 20.f;

        int count = 0;

        double area = 0.0;
        double maxArea = 0.0;
        double minR = 10.0;

        for (int i = 0; i < contours.size(); i++)
        {

            area = cv::contourArea(contours[i]);

            cv::Moments mom = moments(contours[i]);

            double M11 = mom.m11 - (mom.m10 * mom.m01) / mom.m00;
            double M02 = mom.m02 - (mom.m01 * mom.m01) / mom.m00;
            double M20 = mom.m20 - (mom.m10 * mom.m10) / mom.m00;

                // for circle it should be ~0.0063
            double M7 = (M20 * M02 - M11 * M11) / (mom.m00 * mom.m00 * mom.m00 * mom.m00);

                // circle
            if (M7 < 0.0064 && M7 > 0.0062)
            {
                double r = sqrt(area/(float)CV_PI);

                if (r > minR)
                {
                    cv::drawContours(cv_ptr->image, contours, i, CV_RGB(255, 0, 255));
                    cv::line(cv_ptr->image, cv::Point(cx-10, cy), cv::Point(cx+10, cy), CV_RGB(255, 0, 255), 1, 8, 0);
                    cv::line(cv_ptr->image, cv::Point(cx, cy-10), cv::Point(cx, cy+10), CV_RGB(255, 0, 255), 1, 8, 0);

                    minEnclosingCircle(contours[i], center, radius );

                    double f = (fx + fy)/2;
                    double h = f * 0.02 / r;
                    double dx = (center.x-cx) * h / fx;
                    double dy = (center.y-cy) * h / fy;

                    if(area > maxArea)
                    {
                        maxArea = area;
                        output.x = dx;
                        output.y = dy;
                        output.z = h;
                    }
                    //ROS_INFO("circle, M7 = %f , pix_area = %f, pix_x = %f, pix_y = %f, h = %f, dx = %f, dy = %f", M7, area, center.x, center.y, h, dx, dy);
                    ++count;
                }
            }
        }

        std_msgs::String msg;
        std::stringstream ss;
        ss << output;
        msg.data = ss.str();
        chatter_pub.publish(msg);

        cv::imshow(WINDOW_OUT, cv_ptr->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vr_camera");
    ros::NodeHandle nh("~");

    double camera_fx, camera_fy, camera_cx, camera_cy;
    nh.getParam("camera_fx", camera_fx);
    nh.getParam("camera_fy", camera_fy);
    nh.getParam("camera_cx", camera_cx);
    nh.getParam("camera_cy", camera_cy);
    nh.getParam("camera_distortion", camera_distortion);

    camera_matrix = cv::Mat::zeros(3,3, CV_64FC1);
    camera_matrix.ptr<double>(0)[0] = camera_fx;
    camera_matrix.ptr<double>(1)[1] = camera_fx;
    camera_matrix.ptr<double>(0)[2] = camera_cx;
    camera_matrix.ptr<double>(1)[2] = camera_cy;
    camera_matrix.ptr<double>(2)[2] = 1;

    cv::namedWindow(WINDOW_ORG);
    cv::namedWindow(WINDOW_WORK);
    cv::namedWindow(WINDOW_OUT);
    cv::startWindowThread();

    std::string subscriber;
    nh.getParam("subscriber", subscriber);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(subscriber, 1, imageCallback);

    std::string publisher;
    nh.getParam("publisher", publisher);
    chatter_pub = nh.advertise<std_msgs::String>(publisher, 1);

    ros::spin();
    cv::destroyWindow(WINDOW_ORG);
    cv::destroyWindow(WINDOW_OUT);
    cv::destroyWindow(WINDOW_WORK);
}
