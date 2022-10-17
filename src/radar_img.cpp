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
cv::Mat output_img;
double range = 0;
int x_ = 0, target_x = 0;
int y_ = 0, target_y = 0;
class radar_img {
    private:
    int sector_size = 1;
    ros::Publisher pub_img;
    ros::Subscriber sector_subscriber;
    cv::Mat intes_matrix = cv::Mat::zeros(1024,int(3600),CV_8UC1);
    cv::Mat disp_img;
    
    double phi_ = 0;
    int phi = 0;
    
    double phi0_ = 10000.0;
    public:

    radar_img(ros::NodeHandle *nh) {
        sector_subscriber = nh->subscribe("/radar/HaloA/data", 10, &radar_img::sectorCallback, this);
    }

    static void onMouse(int event,int x,int y,int,void*)
    {
        //this function will be called every time you move your mouse over the image
        // the coordinates will be in x and y variables
        x_ = x;
        y_ = y;

        
        
    }

    void sectorCallback(const marine_sensor_msgs::RadarSector& msg) 
    {
      /**
       * @brief build 2D matrix, rows = intensity array length, columns = angle range
       * 
       */
        std::vector<marine_sensor_msgs::RadarScanline> lines_;
        marine_sensor_msgs::RadarScanline line_;
        lines_ = msg.scanlines;
        int flag = 0, pub_flag = 0;
        for (int i = 0; i<sizeof(lines_);i++)
        {
            
            line_ = lines_[i];
            phi_ = line_.angle*180/M_PI;
            if (phi0_ == 10000.0)
            {
                phi0_ = phi_;
                flag = 1;
            }  
            else if (abs(phi0_-phi_) < 1 && flag == 0)
            {
                phi0_ = 10000.0;
                pub_flag = 1;
            }  
            range = line_.range;
            phi = (int) (phi_*10);
            phi = (int) (phi/sector_size);
            for (int j = 0; j < 1024; j+=1)
            {
                // intes_matrix.at<__uint8_t>(j,phi) = (uint8_t) line_.intensities[j+1]  << 4 | (uint8_t) line_.intensities[j];
                intes_matrix.at<__uint8_t>(j,phi) = (uint8_t) line_.intensities[j];

            } 
            
        }
        flag = 0;
        
        // cv::Mat dst;
        // cv::resize(intes_matrix, dst, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
        // imshow( "radar_img", dst );
        // cv::waitKey(1);
        // if (pub_flag)
        {
            // phi0_ = 10000.0;
            cartesianImg(intes_matrix);
            pub_flag = 0;
        }
            
    }


    void cartesianImg(cv::Mat intes_img)
    {
        /**
         * @brief convert intes_matrix to cartesian coordinate
         * 
         */

        cv::Mat cart_img = cv::Mat::zeros((int) intes_img.rows*2.3,(int) intes_img.rows*2.3,CV_8UC1);
        int x_ = 0;
        int y_ = 0;
        for (int i = 0; i < intes_img.cols; i++)
        {
            for (int j=0; j < intes_img.rows; j++)
            {
                x_ = (int) (j*cos(M_PI+i*M_PI/1800) + ((cart_img.rows)/2));
                y_ = (int) (j*sin(M_PI+i*M_PI/1800) + ((cart_img.cols)/2));
                cart_img.at<__uint8_t>(x_ , y_) =  intes_img.at<__uint8_t>(j,i);
            }
        }
        

        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5));
        
        cv::dilate( cart_img, cart_img, element,cv::Point(-1,-1), 3);
        cv::resize(cart_img, cart_img, cv::Size(), 0.25, 0.25, cv::INTER_CUBIC);
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
        contourObjects(colored_img);
        // imshow( "colored_img", colored_img);
        // cv::waitKey(1);

    }

    void contourObjects(cv::Mat rgb_img)
    {
        cv::Mat gray_img;
        cv::Mat info_img = cv::Mat::zeros((int) 80,(int) rgb_img.cols,CV_8UC3);
        disp_img = cv::Mat((int) rgb_img.rows, rgb_img.cols,CV_8UC3);
        disp_img.setTo(cv::Scalar(150, 60, 5));
        
        output_img = cv::Mat::zeros((int) rgb_img.rows + info_img.rows,(int) rgb_img.cols,CV_8UC3);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Point> centers; 
        cv::RotatedRect minRect;

        cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
        cv::Point radar_center(gray_img.cols/2, gray_img.rows/2);
        cv::circle(disp_img, radar_center, (rgb_img.rows/2.1), cv::Scalar(0, 150, 0), 1, 8, 0);
        
        cv::line( disp_img, radar_center, cv::Point(rgb_img.cols/2, rgb_img.cols/2-rgb_img.cols/2.1), cv::Scalar(0, 150, 0), 1 );
        cv::circle(disp_img, radar_center, 2, cv::Scalar(0, 0, 255), -1, 8, 0);
        cv::findContours( gray_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        cv::Moments moment_;

        for( int i = 0; i < contours.size(); i++ )
            {  
                moment_ = cv::moments( contours[i], false );
                if (moment_.m00 != 0)
                {
                    cv::drawContours(disp_img, contours, i,cv::Scalar(3, 28, 247), cv::FILLED, 8, hierarchy);
                    cv::Point center((int) moment_.m10/moment_.m00, (int) moment_.m01/moment_.m00);
                    if (abs(center.x -  gray_img.cols/2) < 2 && abs(center.y -  gray_img.rows/2) < 2)
                        continue;
                    centers.push_back(center);
                    cv::circle( disp_img, center, 2, cv::Scalar(150, 240, 0), -1, 8, 0);
                    minRect = cv::minAreaRect( contours[i] );
                    cv::Point2f rect_points[4];
                    minRect.points( rect_points );
                    for ( int j = 0; j < 4; j++ )
                    {
                        cv::line( disp_img, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 255, 255), 1);
                    }
                }            
            }
        
        // // create temporary image that will hold the mask
        // Mat mask_image( your_image.size(), CV_8U, Scalar(0));
        // // draw your contour in mask
        // drawContours(mask_image, contours, ind, Scalar(255), CV_FILLED);
        // // copy only non-zero pixels from your image to original image
        // your_image.copyTo(original_image, mask_image);
        cv::line( info_img, cv::Point(0, 0), cv::Point(rgb_img.cols-1, 0), cv::Scalar(255, 255, 255), 2 );
        cv::putText(info_img,"Range: "+std::to_string((int)range)+"m",cv::Point(5,info_img.rows-10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),1,false);
        if (x_ >= 0 && y_ >= 0 && x_ < disp_img.cols && y_ < disp_img.rows) 
        {
            cv::line(disp_img,cv::Point(x_,0),cv::Point(x_,disp_img.cols),cv::Scalar(0,255,0),2);
            cv::line(disp_img,cv::Point(0,y_),cv::Point(disp_img.rows,y_),cv::Scalar(0,255,0),2);
            target_x = (int)(x_ - disp_img.cols/2)*(range/disp_img.cols);
            target_y = (int)(y_ - disp_img.rows/2)*(range/disp_img.rows);
            cv::putText(info_img,"target: x:"+std::to_string(-target_y)+", y:"+std::to_string(-target_x),cv::Point(5,info_img.rows-50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),1,false);
        
        }
        
        
        info_img.copyTo(output_img(cv::Rect(0, rgb_img.rows, info_img.cols, info_img.rows)));
        disp_img.copyTo(output_img(cv::Rect(0, 0, rgb_img.cols, rgb_img.rows )));
        // imshow( "radar_img", output_img);
        cv::imshow("radar_img",output_img);
        cv::waitKey(1);
        
    }

};
int main (int argc, char **argv)
{
    
    ros::init(argc, argv, "radar_img");
    ros::NodeHandle nh;
    radar_img nc = radar_img(&nh);
    cv::namedWindow("radar_img", cv::WINDOW_NORMAL );
    cv::setMouseCallback("radar_img",radar_img::onMouse, 0);
    ros::spin();
}