#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>
using namespace std;

ros::Publisher pose_pub,pose_front;

void setup_publishers(ros::NodeHandle &node_handler)
{
    pose_pub = node_handler.advertise<nav_msgs::OccupancyGrid>("/cpp_map", 1);
    pose_front = node_handler.advertise<nav_msgs::OccupancyGrid>("/front_map", 1);
}

void chatterCallback(nav_msgs::OccupancyGrid occgrid)
{

    ros::Time msg_time = ros::Time::now();
    nav_msgs::OccupancyGrid pose_msg,front_msg;

    int width = occgrid.info.width;
    int height = occgrid.info.height;

    std::vector<signed char> frontier(width*height);
    std::vector<int> cells_front;
    std::vector<int> cells_occ;
    std::vector<int> cells_unex;
    std::vector<signed char> P1(width*height);
    std::vector<signed char> P2(width*height);
    std::vector<signed char> P3(width*height);
    std::vector<signed char> P4(width*height);

    for (int index = 0; index < occgrid.data.size(); ++index){

        int x = occgrid.data[index];
        int x1;
        int x2;
        int x3;
        int x4;

        if (index%width != width-1){
            x1 = occgrid.data[index+1];
        }else{
            x1 = 100;
        }
        if (index%width != 0){
            x2 = occgrid.data[index-1];
        }else{
            x2 = 100;
        }

        if (index > width-1){
            x3 = occgrid.data[index-width];
        }else{
            x3 = 100;
        }

        if (index+width-1 <  width*height){
            x4 = occgrid.data[index+width];
        }else{
            x4 = 100;
        }
        if (x == 50){
            cells_unex.push_back (index);
        }else if (x == 100){
            cells_occ.push_back (index);
        }
        if (x == 0 && (x1 == 50 || x2 == 50 ||x3 == 50 ||x4 == 50)){
            frontier[index] = 100;
            cells_front.push_back (index);
        //std::cout << x << ' ';

        }else{
            frontier[index] = 0;

        }
    }

    for (int index = 0; index < P1.size(); ++index){
        int row = (int)index / width;
        int col = index % width;

        double val = 0;
        double val2 = 0;
        double val3 = 0;

        for (int ii = 0; ii < cells_front.size(); ++ii) {
            int row2 = (int)cells_front[ii]/ width;
            int col2 = cells_front[ii] % width;
            val = val - exp(-(pow((row-row2)/5,2)+pow((col-col2)/5,2))/(2*pow(4,2))); 
        }

        for (int ii = 0; ii < cells_unex.size(); ++ii) {
            int row3 = (int)cells_unex[ii]/ width;
            int col3 = cells_unex[ii] % width;
            val2 = val2 - exp(-(pow((row-row3)/5,2)+pow((col-col3)/5,2))/(2*pow(1.65,2))); 
        }

        for (int ii = 0; ii < cells_occ.size(); ++ii) {
            int row4 = (int)cells_occ[ii]/ width;
            int col4 = cells_occ[ii] % width;
            val3 = val3 + exp(-(pow((row-row4)/5,2)+pow((col-col4)/5,2))/(2*pow(0.15,2))); 
        }
        

        P1[index] = val;
        P2[index] = val2;
        P3[index] = val3;
        P4[index]= (3*val + val2 + 100*val3)/50;
        
    }
    pose_msg.header.frame_id = occgrid.header.frame_id;
    pose_msg.header.stamp = occgrid.header.stamp;

    pose_msg.info.resolution = occgrid.info.resolution;
    pose_msg.info.width = occgrid.info.width;
    pose_msg.info.height = occgrid.info.height;
    pose_msg.info.map_load_time = occgrid.info.map_load_time;
    pose_msg.info.origin.position = occgrid.info.origin.position;
    pose_msg.info.origin.orientation = occgrid.info.origin.orientation;
    pose_msg.data = P4;

    front_msg.header.frame_id = occgrid.header.frame_id;
    front_msg.header.stamp = occgrid.header.stamp;

    front_msg.info.resolution = occgrid.info.resolution;
    front_msg.info.width = occgrid.info.width;
    front_msg.info.height = occgrid.info.height;
    front_msg.info.map_load_time = occgrid.info.map_load_time;
    front_msg.info.origin.position = occgrid.info.origin.position;
    front_msg.info.origin.orientation = occgrid.info.origin.orientation;
    front_msg.data = frontier;

    pose_pub.publish(pose_msg);
    pose_front.publish(front_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pot_field");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;

    ros::Subscriber sub = node_handler.subscribe("/vis/map_test", 1, chatterCallback);
    setup_publishers(node_handler);


    ros::spin();

    ros::shutdown();

    return 0;
}

