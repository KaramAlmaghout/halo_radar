/**
 * @file halo_rviz.cpp - to visualize detected objects by SIMARD Halo20+ in RVIZ
 * @author Karam Almaghout 
 * @brief 
 * @version 0.1
 * @date 2022-09-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "ros/ros.h"
#include "marine_sensor_msgs/RadarScanline.h"
#include "marine_sensor_msgs/RadarSector.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

class radar_halo_20p {
    private:
    ros::Publisher pub_to_rviz, pub_tf;
    ros::Subscriber detected_obj_subscriber;
    std::string Frame_id = "radar_link";
    int id = 0;


    public:

    radar_halo_20p(ros::NodeHandle *nh) {
        pub_tf = nh->advertise<tf2_msgs::TFMessage>("/tf", 10);
        pub_to_rviz = nh->advertise<visualization_msgs::MarkerArray>("/halo_rviz/marker_array", 10);    
        detected_obj_subscriber = nh->subscribe("/radar/HaloA/data", 1000, 
            &radar_halo_20p::objectArrayCallback, this);
    }


    void objectArrayCallback(const marine_sensor_msgs::RadarSector& msg) {
      
      tf2_msgs::TFMessage transforms;

      std::vector<marine_sensor_msgs::RadarScanline> detected_obj_list;
      detected_obj_list = msg.scanlines;

      marine_sensor_msgs::RadarScanline obj_i;
      int i;

      id = 0;

      

      for (i = 0; i<sizeof(detected_obj_list); i++) {

          obj_i = detected_obj_list[i];
          id++;
          visualization_msgs::Marker obj;
          std::string tf_name = "obj_" + std::to_string(id);

          geometry_msgs::TransformStamped tf;

          tf.header.stamp = ros::Time();
          tf.header.frame_id = Frame_id;

          tf.child_frame_id = tf_name;

          tf.transform.translation.x = obj_i.range*cos(obj_i.angle);
          tf.transform.translation.y = obj_i.range*sin(obj_i.angle);
          tf.transform.translation.z = 1.0;

          tf.transform.rotation.w = 1.0;
          tf.transform.rotation.x = 0.0;
          tf.transform.rotation.y = 0.0;
          tf.transform.rotation.z = 0.0;

          transforms.transforms.push_back(tf);

      }


      pub_tf.publish(transforms);

      
      
      // parsing data from the msg and send it to MarkerArray
      visualization_msgs::MarkerArray obj_list;
      obj_list.markers.clear();
      id = 0;
      
      
      // Marker for the radar position
      visualization_msgs::Marker radar_obj;
      radar_obj.type = visualization_msgs::Marker::SPHERE;
      radar_obj.action = visualization_msgs::Marker::ADD;
      radar_obj.header.frame_id = Frame_id;
      radar_obj.header.stamp = ros::Time();
      radar_obj.ns = "";
      radar_obj.id = id;
      radar_obj.type = visualization_msgs::Marker::SPHERE;
      radar_obj.action = visualization_msgs::Marker::ADD;
      radar_obj.pose.position.x = 0.0;
      radar_obj.pose.position.y = 0.0;
      radar_obj.pose.position.z = 1.0; // not real
      radar_obj.pose.orientation.x = 0.0;
      radar_obj.pose.orientation.y = 0.0;
      radar_obj.pose.orientation.z = 0.0;
      radar_obj.pose.orientation.w = 1.0;
      radar_obj.scale.x = 5;
      radar_obj.scale.y = 5;
      radar_obj.scale.z = 2;
      radar_obj.color.a = 1.0; 
      radar_obj.color.r = 0.0;
      radar_obj.color.g = 0.0;
      radar_obj.color.b = 1.0;

      obj_list.markers.push_back(radar_obj);

      std::cout<<"number of detected objects: "<<sizeof(detected_obj_list)<<std::endl;

      for (i = 0; i<sizeof(detected_obj_list); i++)
      {
        obj_i = detected_obj_list[i];
        id++;
        visualization_msgs::Marker obj;
        obj.action = visualization_msgs::Marker::ADD;
        obj.header.frame_id = Frame_id;
        obj.header.stamp = ros::Time();
        obj.ns = "";
        obj.id = id;
        obj.type = visualization_msgs::Marker::SPHERE;
        obj.pose.position.x = obj_i.range*cos(obj_i.angle);
        obj.pose.position.y = obj_i.range*sin(obj_i.angle);
        obj.pose.position.z = 1.0; // not real
        obj.pose.orientation.x = 0.0;
        obj.pose.orientation.y = 0.0;
        obj.pose.orientation.z = 0.0;
        obj.pose.orientation.w = 1.0;
        obj.scale.x = 2;
        obj.scale.y = 2;
        obj.scale.z = 1;
        obj.color.a = 1.0; 
        obj.color.r = 0.0;
        obj.color.g = 1.0;
        obj.color.b = 0.0;
        obj.lifetime = ros::Duration(10.0);
        obj_list.markers.push_back(obj);
      }

      pub_to_rviz.publish(obj_list);


    }

};
int main (int argc, char **argv)
{
    ros::init(argc, argv, "halo_rviz");
    ros::NodeHandle nh;
    radar_halo_20p nc = radar_halo_20p(&nh);
    ros::spin();
}