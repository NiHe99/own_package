#include <nav_msgs/OccupancyGrid.h>
#include <own_package/Three_cat.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>
using namespace std;

ros::Publisher pose_pub,pose_front;

void setup_publishers(ros::NodeHandle &node_handler)
{
    pose_pub = node_handler.advertise<own_package::Three_cat>("/zones", 1);

}

void chatterCallback(nav_msgs::OccupancyGrid occgrid)
{
    int counter = 0;
    ros::Time msg_time = ros::Time::now();
    own_package::Three_cat pose_msg;
    int width = occgrid.info.width;
    int height = occgrid.info.height;

    std::vector<short int> cells_front_x;
    std::vector<short int> cells_occ_x;
    std::vector<short int> cells_unex_x;
    std::vector<short int> cells_front_y;
    std::vector<short int> cells_occ_y;
    std::vector<short int> cells_unex_y;

    for (int index = 0; index < occgrid.data.size(); ++index){

        int x = occgrid.data[index];
        int x1;
        int x2;
        int x3;
        int x4;
        int row = (int)index/ width;
        int col = index % width;

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
            cells_unex_x.push_back (row);
            cells_unex_y.push_back (col);
            cells_occ_x.push_back (32000);
            cells_occ_y.push_back (32000);
        }else if (x == 100){
            cells_occ_x.push_back (row);
            cells_occ_y.push_back (col);
            cells_unex_x.push_back (32000);
            cells_unex_y.push_back (32000);
        }else{
            cells_occ_x.push_back (32000);
            cells_occ_y.push_back (32000);
            cells_unex_x.push_back (32000);
            cells_unex_y.push_back (32000);

        }

        if (x == 0 && (x1 == 50 || x2 == 50 ||x3 == 50 ||x4 == 50)){
            cells_front_x.push_back (row);
            cells_front_y.push_back (col);
            counter++;
        }else{
            cells_front_x.push_back (32000);
            cells_front_y.push_back (32000);

        }

    }

    if (counter < 1){
        for (int index = 0; index < occgrid.data.size(); ++index){
            int row = (int)index/ width;
            int col = index % width;
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
        
        
                cells_front_x.push_back (row);
                cells_front_y.push_back (col);

            }else{
                cells_front_x.push_back (32000);
                cells_front_y.push_back (32000);
            }
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
    pose_msg.front_x =  cells_front_x;
    pose_msg.front_y =  cells_front_y;
    pose_msg.unex_x =  cells_unex_x;
    pose_msg.unex_y =  cells_unex_y;
    pose_msg.obb_x =  cells_occ_x;
    pose_msg.obb_y =  cells_occ_y;   

    pose_pub.publish(pose_msg);
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

