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
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <opencv2/opencv.hpp>
#include <cmath>
cv::Mat output_img;
double range = 0;
int x_obj = 0, target_x = 0;
int y_obj = 0, target_y = 0;
class radar_img {
    private:
    int sector_size = 1;
    ros::Publisher pub_img,obj_array_publisher;
    ros::Subscriber sector_subscriber;
    cv::Mat intes_matrix = cv::Mat::zeros(1024,int(3600),CV_8UC1);
    cv::Mat disp_img;
    
    double phi_ = 0;
    int phi = 0;
    
    double phi0_ = 10000.0;
    public:

    radar_img(ros::NodeHandle *nh) {
        sector_subscriber = nh->subscribe("/radar/HaloA/data", 10, &radar_img::sectorCallback, this);
        obj_array_publisher = nh->advertise<autoware_msgs::DetectedObjectArray>("/radar/DetectedObjects", 10);
    }

    static void onMouse(int event,int x,int y,int,void*)
    {
        //this function will be called every time you move your mouse over the image
        // the coordinates will be in x and y variables
        x_obj = x;
        y_obj = y;

        
        
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
                if ((uint8_t) line_.intensities[j] > 4)
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

        cv::Mat cart_img = cv::Mat::zeros((int) intes_img.rows*2,(int) intes_img.rows*2,CV_8UC1);
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
        
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3));
        
        cv::erode( cart_img, cart_img, element,cv::Point(-1,-1), 1);
        cv::Mat element1 = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 9, 9));
        
        cv::dilate( cart_img, cart_img, element1,cv::Point(-1,-1), 4);
        cv::erode( cart_img, cart_img, element1,cv::Point(-1,-1), 1);
        cv::resize(cart_img, cart_img, cv::Size(), 0.3, 0.3, cv::INTER_AREA);
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
        // imshow( "radar_img", rgb_img);
        output_img = cv::Mat::zeros((int) rgb_img.rows + info_img.rows,(int) rgb_img.cols,CV_8UC3);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Point> centers; 
        cv::RotatedRect minRect;

        cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
        cv::Point radar_center(gray_img.cols/2, gray_img.rows/2);
        cv::circle(disp_img, radar_center, (rgb_img.rows/2.1), cv::Scalar(0, 150, 0), 2, 8, 0);
        
        cv::line( disp_img, radar_center, cv::Point(rgb_img.cols/2, rgb_img.cols/2-rgb_img.cols/2.1), cv::Scalar(0, 150, 0), 1 );
        cv::circle(disp_img, radar_center, 2, cv::Scalar(0, 0, 255), -1, 8, 0);
        if (range > 700)
            cv::circle(disp_img, radar_center, (int)(500*(disp_img.cols/2)/range), cv::Scalar(0, 150, 0), 1, 8, 0);
        if (range >= 1500)
            cv::circle(disp_img, radar_center, (int)(1000*(disp_img.cols/2)/range), cv::Scalar(0, 150, 0), 1, 8, 0);
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
        int obj_range;
        cv::line( info_img, cv::Point(0, 0), cv::Point(rgb_img.cols-1, 0), cv::Scalar(255, 255, 255), 2 );
        cv::putText(info_img,"Range: "+std::to_string((int)range)+"m",cv::Point(5,info_img.rows-10),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),1,false);
        if (x_obj >= 0 && y_obj >= 0 && x_obj < disp_img.cols && y_obj < disp_img.rows) 
        {
            // cv::line(disp_img,cv::Point(x_obj,0),cv::Point(x_obj,disp_img.cols),cv::Scalar(0,255,0),2);
            // cv::line(disp_img,cv::Point(0,y_obj),cv::Point(disp_img.rows,y_obj),cv::Scalar(0,255,0),2);
            cv::line(disp_img,cv::Point((int) disp_img.cols/2,(int) disp_img.rows/2),cv::Point(x_obj,y_obj),cv::Scalar(0,255,255),1);
            target_x = (int)(x_obj - disp_img.cols/2)*(range/(disp_img.cols/2));
            target_y = (int)(y_obj - disp_img.rows/2)*(range/(disp_img.rows/2));
            obj_range = round(sqrtf(pow(target_x,2)+pow(target_y,2)));
            cv::putText(info_img,"target: x:"+std::to_string(-target_y)+", y:"+std::to_string(-target_x)+", r: "+std::to_string(obj_range)+"m",cv::Point(5,info_img.rows-50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),1,false);
        
        }
        
        
        info_img.copyTo(output_img(cv::Rect(0, rgb_img.rows, info_img.cols, info_img.rows)));
        disp_img.copyTo(output_img(cv::Rect(0, 0, rgb_img.cols, rgb_img.rows )));
        // imshow( "radar_img", output_img);
        DetectedObjPub(disp_img, contours);
        cv::imshow("radar_img",output_img);
        cv::waitKey(1);
        
    }



void DetectedObjPub(cv::Mat img, std::vector<std::vector<cv::Point>> cnts)
{
    cv::Moments moment_;
    // cv::Point center;
    
    cv::RotatedRect minRect;
    std::string frame_id = "radar_link";
    autoware_msgs::DetectedObjectArray detected_obj_array_;

    for( int i = 0; i < cnts.size(); i++ )
        {  
            moment_ = cv::moments( cnts[i], false );
            if (moment_.m00 != 0)
            {
                autoware_msgs::DetectedObject detected_obj_;
                float rot_angle, obj_w, obj_h, obj_cx, obj_cy;
                cv::Point center((int) moment_.m10/moment_.m00, (int) moment_.m01/moment_.m00);
                if (abs(center.x -  img.cols/2) < 2 && abs(center.y -  img.rows/2) < 2)
                    continue;
                minRect = cv::minAreaRect( cnts[i] );
                
                rot_angle = minRect.angle;
                if (rot_angle == -90)
                {
                    rot_angle = 0;
                }

                rot_angle = -rot_angle;
                obj_w = minRect.size.width*(range/(img.cols/2));;
                obj_h = minRect.size.height*(range/(img.cols/2));;
                if (rot_angle > 45)
                {
                    rot_angle = - rot_angle +45;
                    obj_h = minRect.size.width*(range/(img.cols/2));;
                    obj_w = minRect.size.height*(range/(img.cols/2));;
                }
                
                obj_cy = -(int)(center.x - img.cols/2)*(range/(img.cols/2));
                obj_cx = -(int)(center.y - img.rows/2)*(range/(img.rows/2));


                detected_obj_.header.stamp = ros::Time();
                detected_obj_.header.frame_id = frame_id;
                
                detected_obj_.pose.position.x = obj_cx;
                detected_obj_.pose.position.y = obj_cy;

                detected_obj_.pose.orientation.z = rot_angle;

                detected_obj_.dimensions.x = obj_h;
                detected_obj_.dimensions.y = obj_w;

                detected_obj_array_.objects.push_back(detected_obj_);

            }

        }

        obj_array_publisher.publish(detected_obj_array_);


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