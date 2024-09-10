#include <nav_msgs/OccupancyGrid.h>
#include <own_package/Pot_Grid.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>
using namespace std;

ros::Publisher pose_pub,pose_front;

void setup_publishers(ros::NodeHandle &node_handler)
{
    pose_pub = node_handler.advertise<own_package::Pot_Grid>("/cpp_map2", 1);
    pose_front = node_handler.advertise<nav_msgs::OccupancyGrid>("/front_map2", 1);
}

void chatterCallback(nav_msgs::OccupancyGrid occgrid)
{
    int counter = 0;
    ros::Time msg_time = ros::Time::now();
    nav_msgs::OccupancyGrid front_msg;
    own_package::Pot_Grid pose_msg;
    int width = occgrid.info.width;
    int height = occgrid.info.height;

    std::vector<signed char> frontier(width*height);
    std::vector<int> cells_front;
    std::vector<int> cells_occ;
    std::vector<int> cells_unex;
    std::vector<double> P1(width*height);
    std::vector<double> P2(width*height);
    std::vector<double> P3(width*height);
    std::vector<double> P4(width*height);
    std::fill(P1.begin(),P1.end(),0);
    std::fill(P2.begin(),P2.end(),0);
    std::fill(P3.begin(),P3.end(),0);
    std::fill(P4.begin(),P4.end(),0);
    for (int index = 0; index < occgrid.data.size(); ++index){

        int x = occgrid.data[index];
        int x1;
        int x2;
        int x3;
        int x4;

        if (index%width != width-1){
            x1 = occgrid.data[index+1];
        }else{
            x1 = 90;
        }
        if (index%width != 0){
            x2 = occgrid.data[index-1];
        }else{
            x2 = 90;
        }

        if (index > width-1){
            x3 = occgrid.data[index-width];  
        }else{
            x3 = 90;
        }

        if (index+width-1 <  width*height){
            x4 = occgrid.data[index+width];
        }else{
            x4 = 90;
        }
        if (x == 50){
            cells_unex.push_back (index);
        }else if (x == 100){
            cells_occ.push_back (index);
        }
        if (x == 0 && (x1 == 50 || x2 == 50 ||x3 == 50 ||x4 == 50)){
            frontier[index] = 100;
            cells_front.push_back (index);
            counter++;

        }

    }

    if (counter < 1){
        for (int index = 0; index < occgrid.data.size(); ++index){

            int x = occgrid.data[index];
            int x1;
            int x2;
            int x3;
            int x4;

            if (index%width != width-1){
                x1 = occgrid.data[index+1];
            }else{
                x1 = 90;
            }
            if (index%width != 0){
                x2 = occgrid.data[index-1];
            }else{
                x2 = 90;
            }

            if (index > width-1){
                x3 = occgrid.data[index-width];  
            }else{
                x3 = 90;
            }

            if (index+width-1 <  width*height){
                x4 = occgrid.data[index+width];
            }else{
                x4 = 90;
            }
    
            if(x == 0 && (x1 == 90 || x2 == 90 ||x3 == 90 ||x4 == 90)){
                if (frontier[index] != 100){
                frontier[index] = 100;
                cells_front.push_back (index);
                }
            }else{
                frontier[index] = 0;

            }
        }
    }
    long test_val = 0; 

    for (int ii = 0; ii < cells_front.size(); ++ii) {
            int radius = 65;
            int row2 = (int)cells_front[ii]/ width;
            int col2 = cells_front[ii] % width;
            
            for (int xx = 0; xx < 2*radius+1; ++xx) {
                if(row2+xx-radius >= 0 && row2+xx-radius <=width-1){

                for (int yy = 0; yy < 2*radius+1; ++yy) {
                    if ( col2+yy-radius>= 0 && col2+yy-radius <=width-1){

                    int index_p = (row2+xx-radius)*width + (col2+yy-radius);
                    int rowp = index_p/ width;
                    int colp = index_p % width;

                   
                    P1[index_p] = P1[index_p] - exp(-(pow((rowp-row2),2)+pow((colp-col2),2))/(2*pow(22.5,2)));
                    
        }
        }
        }
        }
        }

    for (int ii = 0; ii < cells_unex.size(); ++ii) {
            int radius = 24;
            int row2 = (int)cells_unex[ii]/ width;
            int col2 = cells_unex[ii] % width;
            
            for (int xx = 0; xx < 2*radius+1; ++xx) {
                if(row2+xx-radius >= 0 && row2+xx-radius <=width-1){

                for (int yy = 0; yy < 2*radius+1; ++yy) {
                    if ( col2+yy-radius>= 0 && col2+yy-radius <=width-1){

                    int index_p = (row2+xx-radius)*width + (col2+yy-radius);
                    int rowp = index_p/ width;
                    int colp = index_p % width;

                    P2[index_p] = P2[index_p] - exp(-(pow((rowp-row2),2)+pow((colp-col2),2))/(2*pow(8,2))); 
                    
        }
        }
        }
        }
        }

    for (int ii = 0; ii < cells_occ.size(); ++ii) {
            int radius = 3;
            int row2 = (int)cells_occ[ii]/ width;
            int col2 = cells_occ[ii] % width;
            
            for (int xx = 0; xx < 2*radius+1; ++xx) {
                if(row2+xx-radius >= 0 && row2+xx-radius <=width-1){

                for (int yy = 0; yy < 2*radius+1; ++yy) {
                    if ( col2+yy-radius>= 0 && col2+yy-radius <=width-1){

                    int index_p = (row2+xx-radius)*width + (col2+yy-radius);
                    int rowp = index_p/ width;
                    int colp = index_p % width;
                    
                    P3[index_p] = P3[index_p] + exp(-(pow((rowp-row2),2)+pow((colp-col2),2))/(2*pow(1,2))); 
                    
                    
        }
        }
        }
        }
        }

    for (int ii = 0; ii < P4.size(); ++ii){

        int weight1 = 3;
        int weight2 = 1; 
        int weight3 = 100;
        int divide = 20;
        if ((weight1*P1[ii] + weight2*P2[ii] + weight3*P3[ii])/divide< -100){
            P4[ii] = -100;
        }else if ((weight1*P1[ii] + weight2*P2[ii] + weight3*P3[ii])/divide> 100){
            P4[ii] = 100;
        }else{
        P4[ii]= (weight1*P1[ii] + weight2*P2[ii] + weight3*P3[ii])/divide;
        }
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
    ros::init(argc, argv, "pot_field2");
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

