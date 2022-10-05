/**
 * @file radar_img.cpp
 * @author Karam Almaghout
 * @brief 
 * @version 0.1
 * @date 2022-10-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ros/ros.h"
#include "marine_sensor_msgs/RadarScanline.h"
#include "marine_sensor_msgs/RadarSector.h"
#include <opencv2/opencv.hpp>
#include <cmath>

class radar_img {
    private:
    int sector_size = 1;
    ros::Publisher pub_img;
    ros::Subscriber sector_subscriber;
    cv::Mat intes_matrix = cv::Mat::zeros(512,int(3600),CV_8UC1);
    cv::Mat dst;
    double phi_ = 0;
    int phi = 0;

    public:

    radar_img(ros::NodeHandle *nh) {
        sector_subscriber = nh->subscribe("/radar/HaloA/data", 1000, &radar_img::sectorCallback, this);
    }


    void sectorCallback(const marine_sensor_msgs::RadarSector& msg) {
      /**
       * @brief build 2D matrix, rows = intensity array length, columns = angle range
       * 
       */
      std::vector<marine_sensor_msgs::RadarScanline> lines_;
      marine_sensor_msgs::RadarScanline line_;
      lines_ = msg.scanlines;
      for (int i = 0; i<sizeof(lines_);i++)
      {
        line_ = lines_[i];
        phi_ = line_.angle*180/M_PI;
        phi = (int) (phi_*10);
        phi = (int) (phi/sector_size);
        for (int j = 0; j < 512; j+=2)
        {
            intes_matrix.at<__uint8_t>(j,phi) = (uint8_t) line_.intensities[j+1]  << 4 | (uint8_t) line_.intensities[j];
        } 
        
      }
        
        // cv::Mat dst;
        // cv::resize(intes_matrix, dst, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
        // imshow( "radar_img", dst );
        // cv::waitKey(1);
        cartesianImg(intes_matrix);
    }

    void cartesianImg(cv::Mat intes_img)
    {
        /**
         * @brief convert intes_matrix to cartesian coordinate
         * 
         */

        cv::Mat cart_img = cv::Mat::zeros((int) intes_img.rows*2.2,(int) intes_img.rows*2.2,CV_8UC1);
        int x_ = 0;
        int y_ = 0;
        for (int i = 0; i < intes_img.cols; i++)
        {
            for (int j=0; j < intes_img.rows; j++)
            {
                x_ = (int) (j*sin(i*M_PI/1800) + ((cart_img.rows)/2));
                y_ = (int) (j*cos(i*M_PI/1800) + ((cart_img.cols)/2));
                cart_img.at<__uint8_t>(x_ , y_) =  intes_img.at<__uint8_t>(j,i);
            }
        }
        

        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 6, 6));
        
        cv::dilate( cart_img, cart_img, element,cv::Point(-1,-1), 2);
        cv::resize(cart_img, cart_img, cv::Size(), 0.75, 0.75, cv::INTER_AREA);
        // imshow( "CART_img", cart_img);
        // cv::waitKey(10);
        colorizeImg(cart_img);

        
    }

    void colorizeImg(cv::Mat gray_img)
    {
        /**
         * @brief low intensity --> blue
         *        meduim intensity --> green
         *        high intensity --> red
         */

        int low_th = 100;
        int high_th =200;
        cv::Vec3b red(255,0,0);
        cv::Vec3b green(0,255,0);
        cv::Vec3b blue(0,0,255);
        cv::Mat colored_img = cv::Mat::zeros(gray_img.rows,gray_img.cols,CV_8UC3);
        for (int i = 0; i < gray_img.rows; i++)
        {
            for (int j=0; j < gray_img.cols; j++)
            {
                if (gray_img.at<__uint8_t>(i, j) > 0 && gray_img.at<__uint8_t>(i, j) <= low_th)
                {
                    colored_img.at<cv::Vec3b>(i, j) = red;
                }
                else if (gray_img.at<__uint8_t>(i, j) > low_th && gray_img.at<__uint8_t>(i, j) < high_th)
                {
                    colored_img.at<cv::Vec3b>(i, j) = green;
                }
                else if (gray_img.at<__uint8_t>(i, j) >= high_th)
                {
                    colored_img.at<cv::Vec3b>(i, j) = blue;
                }
            }
        }
        
        imshow( "colored_img", colored_img);
        cv::waitKey(1);

    }

};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "radar_img");
    ros::NodeHandle nh;
    radar_img nc = radar_img(&nh);
    ros::spin();
}