Clone and built in <your_ros_workspace>

[halo_radar](https://github.com/rolker/halo_radar) and [marine_sensors_msgs](https://github.com/rolker/marine_sensor_msgs)



The node subscribe to the topic `radar/" + addresses.label + "/change_state` to get the command and send packet to the radar.

msg type:
`marine_sensor_msgs::RadarControlValue`

![image](https://user-images.githubusercontent.com/94979970/209917713-365eaceb-95f0-4935-8698-565723c33c6d.png))

the message is passed to the method sendCommand to be processed and send back to the radar

###`void Radar::sendCommand(std::string const &key, std::string const &value)`


|key|value  |  |
|--|--|--|
|status|transmit|start rtansmitting|
|status|standby|set the radar to standby mode|
|range|range in meter|adjust the radar maximum range|
|bearing_alignment|value|adjust the bearing angle|
|gain|"auto" or value|Set "auto" and the radar built-in method will adjust to “best” performance. Or adjust manually.|
|sea_clutter|"auto" or value 0 - 100|Set "auto" and the radar built-in method will adjust to “best” performance. Or adjust manually.|
|rain_clutter|0 - 100|Adjust manually to a suitable level when needed. A zero value is for normal use. When heavy rain clutters the screen increase the value to best performance. A high value may also filter out important targets|
|sidelobe_suppression|auto||
|interference_rejection|0 = "off", 1 = "low", 2 = "medium", 3 = "high"|Off-Low-Medium-High. Suppress interference from other broadband (close) radars.|
|sea_state|moderate, rough||
|scan_speed|medium, high|Normal-Fast. Set the rotation speed of the radar scanner for example to follow high speed targets.|
|mode|harbor, offshore, weather, bird||
|auto_sea_clutter_nudge|value|||
|target_expansion|0 = "off", 1 = "on", 2 = "medium", 3 = "high"|Off-On. Let the radar expand targets.|
|noise_rejection|0 = "off", 1 = "low", 2 = "medium", 3 = "high"|Off-Low-High. Filter out noise what's not rain or sea clutter.|
|target_separation|0 = "off", 1 = "low", 2 = "medium", 3 = "high"|Off-Low-Medium-High. Let the radar try to distinguish between targets|
|doppler_mode|normal, approaching_only||
|doppler_speed|value||
|antenna_height|value||
|lights|value||

This two examples how to set the "status" to "transmit" and adjust the "range" to "100"m 
![image](https://user-images.githubusercontent.com/94979970/209917743-9be6f46a-f06c-4266-a985-369f96856fb7.png)
![image](https://user-images.githubusercontent.com/94979970/209917750-cc7a1f37-858d-4f75-a125-4e05e4535428.png)

To check the radar configuration 
`rostopic echo /radar/HaloA/state`
it returns the floowing:
![image](https://user-images.githubusercontent.com/94979970/209917773-9c26213b-417d-40fb-aa79-8fbb503dafd6.png)
![image](https://user-images.githubusercontent.com/94979970/209917829-45370502-cc4e-4c04-ae6e-5bd1112ddeb0.png)
![image](https://user-images.githubusercontent.com/94979970/209917839-c441fec9-a14e-4c84-9cd9-80d54543db9e.png)




### Control the radar parameters (transmit/standby, set the range, ...)

A ROS node  "ctrl_radar" has been written to allow controlling the radar from terminal
the node takes up two args from the terminal 
the first argument is either "standby" or "transmit"
second argument is the range in meters (e.g. 1000)

`rosrun halo_radar ctrl_radar`

![image](https://user-images.githubusercontent.com/94979970/209917865-6aa0e9c5-9ce8-4e82-bdd9-87953820f913.png)

 A launch file "halo_radar.launch" is written where it runs the main node "halo_radar" which fetches the TCP packet and parses it, then it runs "ctrl_radar" with default args (status = standby and range =1000)

### Visualization and publish to autoware_msgs/DetectedObjectArray

`"radar_img" node has been created to:
- visualize the detectedobjects
- allow to point aspecific object on the screen and obtain its position
- publish autoware_msgs/DetecctedObjectArray msg includes the detected objects bounding boxes centers, width, height, and rotation angle

![image](https://user-images.githubusercontent.com/94979970/209917878-1e06107d-9371-4dda-97e7-6c8626bb4102.png)

the node `radar_img` publishes the detected objects over the topic `/radar/DetectedObjects`

----------------
### To run the radar

download the [halo_radar](https://github.com/KaramAlmaghout/halo_radar/tree/halo20plus) & [marine_sensors_msgs](https://github.com/rolker/marine_sensor_msgs) & [autoware_msgs](https://github.com/streetdrone-home/Autoware/tree/master/ros/src/msgs/autoware_msgs) pkgs and build them in your ROS workspace (ROS1 melodic)

Source your workspace

in terminal
`rosrun halo_radar halo radar &`
`rosrun halo_radar radar_img &`
`rosrun halo_radar ctr_radar`

The terminal will ask `>> Enter 'key':`
type `status`
The terminal will ask `>> Enter 'value':`
type `transmit`

The radar should start transmitting

now You can adjust the range in accordance to your preferences

status: `range`
value: range in meter (e.g. 1000)

----------------

###radar_img node:

**sector_subscriber** subscribe to `/radar/HaloA/data` it receive sector msg which includes an array of scanline msg (msg of marine_sensors_msgs) the Scanline msg has the following data:
Range in meter (maximum range)
Angle in Radian (with respect to the radar head 0 to 2pi)
array of intensities of the detected objects (len = 1024)
The subscribed msg is passed to the method **`sectorCallback`** where the msgs are accumulated to construct a 2D matrix of intensities and filter out the low intensities. The constructed 2d intensitiy matrix is then passed to **`cartesianImg`** method where it is converted from polar coordinates to cartesian coordinates. Then it achieve some morophlogical operations to merge neiborhood and remove some noises.

 **`contourObjects`** detects the finalized processed image returned from **`cartesianImg`**, generate contours, draw rotating bounding boxes and returns the information of these bounding boxes. Also it allows to point on an object by the cursor and retrive its x,y and range.
This method visualize the detected objects.

**`DetectedObjPub`** arrange the information of the detected objects (height, width, rotation angle, x, y) in autoware_msgs/DetectedObjectArray and publishes them over the topic `/radar/DetectedObjects`
