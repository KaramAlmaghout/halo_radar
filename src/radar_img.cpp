/**
 * @file radar_img.cpp
 * @author Karam
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
    int sector_size = 6;
    ros::Publisher pub_img;
    ros::Subscriber sector_subscriber;
    cv::Mat intes_matrix = cv::Mat(512,int(360/sector_size),CV_8UC1); 
    cv::Mat dst;
    int phi = 0;

    public:

    radar_img(ros::NodeHandle *nh) {
        sector_subscriber = nh->subscribe("/radar/HaloA/data", 1000, &radar_img::sectorCallback, this);
    }


    void sectorCallback(const marine_sensor_msgs::RadarSector& msg) {
      
      std::vector<marine_sensor_msgs::RadarScanline> lines_;
      marine_sensor_msgs::RadarScanline line_;
      lines_ = msg.scanlines;
      for (int i = 0; i<sizeof(lines_);i++)
      {
        line_ = lines_[i];
        phi = (int) ((line_.angle*10000)*180/M_PI)/10000;
        phi = (int) (phi/sector_size);
        for (int j = 0; j < 512; j+=2)
        {
            intes_matrix.at<__uint8_t>(j,phi) = (uint8_t) line_.intensities[j+1]  << 4 | (uint8_t) line_.intensities[j];
            // std::cout << "i+1:"<< std::endl;
            // std::cout << (int) line_.intensities[j+1]<< std::endl;
            // std::cout << "i:"<< std::endl;
            // std::cout << (int) line_.intensities[j]<< std::endl;
            // std::cout << "iii:"<< std::endl;
            // std::cout << (int) intes_matrix.at<int>(j,phi) << std::endl;

        } 
      }
        // std::cout << "radar img:"<< std::endl;
        // std::cout << intes_matrix << std::endl;
        // std::cout << phi << std::endl;
        // cv::normalize(intes_matrix, dst, 0, 255, cv::NORM_MINMAX);
        // cv::cvtColor(intes_matrix,cv::COLOR_BGR2GRAY);
        imshow( "radar_img", intes_matrix );
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