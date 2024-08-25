#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <cmath>
#include <own_package/Num.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;

ros::Publisher pose_pub;

int width = 300;
int height = 300;

std::vector<signed char> Karte(width*height);

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

std::vector<long> bhm_line(int x1,int y1,int x2,int y2)
{
 int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
 long ind;
 std::vector<long> index;
 dx=x2-x1;
 dy=y2-y1;
 dx1=fabs(dx);
 dy1=fabs(dy);
 px=2*dy1-dx1;
 py=2*dx1-dy1;
 if(dy1<=dx1)
 {
  if(dx>=0)
  {
   x=x1;
   y=y1;
   xe=x2;
  }
  else
  {
   x=x2;
   y=y2;
   xe=x1;
  }
    ind = x*width+y;
  index.push_back (ind);
  for(i=0;x<xe;i++)
  {
   x=x+1;
   if(px<0)
   {
    px=px+2*dy1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     y=y+1;
    }
    else
    {
     y=y-1;
    }
    px=px+2*(dy1-dx1);
   }
   ind = x*width+y;
  index.push_back (ind);
  }
 }
 else
 {
  if(dy>=0)
  {
   x=x1;
   y=y1;
   ye=y2;
  }
  else
  {
   x=x2;
   y=y2;
   ye=y1;
  }
  ind = x*width+y;
  index.push_back (ind);
  for(i=0;y<ye;i++)
  {
   y=y+1;
   if(py<=0)
   {
    py=py+2*dx1;
   }
   else
   {
    if((dx<0 && dy<0) || (dx>0 && dy>0))
    {
     x=x+1;
    }
    else
    {
     x=x-1;
    }
    py=py+2*(dx1-dy1);
   }
   ind = x*width+y;
  index.push_back (ind);
  }
 }
 return index;
}

void setup_publishers(ros::NodeHandle &node_handler)
{
    pose_pub = node_handler.advertise<nav_msgs::OccupancyGrid>("/cpp_occ_map", 1);

}

double ProbabilityToLogOdds(double p){
    return log(p/(1.0-p));
}

double LogOddsToProbability(double p){
    return 1.0/(1.0+exp(-p));
}

void chatterCallback(own_package::Num keyframes)
{
    
    double occ_threshold = 0.97;
    double free_threshold = 0.03;
    double occ_update = 0.7;
    double free_update = 0.45;
    int min = -30;
    double res = 0.2;
    
    
    //cout << keyframes.skip << ' ';
    if (keyframes.skip == 0){
        
        std::vector<double> angles;
        tf::Quaternion q(keyframes.pose.orientation.x, keyframes.pose.orientation.z, -keyframes.pose.orientation.y, keyframes.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double sensor_x = keyframes.pose.position.x;
        double sensor_y = keyframes.pose.position.z;
        int grid_x = (sensor_x-min)/res;
        int grid_y = (sensor_y-min)/res;
        

        for (int index = 0; index < keyframes.range.size(); ++index){
            
            if (std::isfinite(keyframes.range[index])){

            double angle = 0.7 + index*0.025+yaw;
            
            int rangeinvoxel = round(keyframes.range[index]/res);

            int scan_x = round(grid_x+rangeinvoxel*cos(angle));
            int scan_y = round(grid_y+rangeinvoxel*sin(angle));
            
            long placeinvec = scan_x*width + scan_y;
            double test_karte = Karte[placeinvec];
            double test_pro = test_karte/100 + 0.001;

            if (ProbabilityToLogOdds(test_pro) < ProbabilityToLogOdds(occ_threshold)){
                double logods = ProbabilityToLogOdds(test_pro) + ProbabilityToLogOdds(occ_update);
                int prob = (int)(LogOddsToProbability(logods)*100);
                Karte[placeinvec] = prob;

            }
            
            
            std::vector<long> points = bhm_line(grid_x,grid_y,scan_x,scan_y);

            for (int in = 0; in < points.size(); ++in){
                if (in != placeinvec){
                double test_karte_in = Karte[points[in]];
                double test_pro_in = test_karte_in/100 + 0.001;

                if (ProbabilityToLogOdds(test_pro_in) > ProbabilityToLogOdds(free_threshold)){
                    double logods1 = ProbabilityToLogOdds(test_pro_in) + ProbabilityToLogOdds(free_update);
                    int prob1 = (int)(LogOddsToProbability(logods1)*100);
                    Karte[points[in]] = prob1;
                }        
            }
            }
            }
        }
 
    }

    else if (keyframes.skip == 1){
        
        std::fill(Karte.begin(),Karte.end(),50);
        std::vector<double> angles;
        ROS_WARN ("Start CPP");

        for (int index = 0; index < keyframes.poses.size(); ++index){
            tf::Quaternion q(keyframes.poses[index].orientation.x, keyframes.poses[index].orientation.z, -keyframes.poses[index].orientation.y, keyframes.poses[index].orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            angles.push_back (yaw);
        }
        
        for (int index = 0; index < keyframes.ranges.size(); ++index){
            
            if (std::isfinite(keyframes.ranges[index])){
            int counter = (int)index/keyframes.increments;
            double sensor_x = keyframes.poses[counter].position.x;
            double sensor_y = keyframes.poses[counter].position.z;
            
            int grid_x = (sensor_x-min)/res;
            int grid_y = (sensor_y-min)/res;

            int idx = index%70;
            double angle = 0.7 + idx*0.025+angles[counter];
            int rangeinvoxel = round(keyframes.ranges[index]/res);

            int scan_x = round(grid_x+rangeinvoxel*cos(angle));
            int scan_y = round(grid_y+rangeinvoxel*sin(angle));
            long placeinvec = scan_x*width + scan_y;
            double test_karte = Karte[placeinvec];
            double test_pro = test_karte/100 + 0.001;

            if (ProbabilityToLogOdds(test_pro) < occ_threshold){
                double logods = ProbabilityToLogOdds(test_pro) + ProbabilityToLogOdds(occ_update);
                Karte[placeinvec] = (int)(LogOddsToProbability(logods)*100);
            }

            std::vector<long> points = bhm_line(grid_x,grid_y,scan_x,scan_y);

            for (int in = 1; in < points.size()-1; ++in){

                double test_karte_in = Karte[points[in]];
                double test_pro_in = test_karte_in/100 + 0.001;

                if (ProbabilityToLogOdds(test_pro_in) > ProbabilityToLogOdds(free_threshold)){
                    double logods1 = ProbabilityToLogOdds(test_pro_in) + ProbabilityToLogOdds(free_update);
                    int prob1 = (int)(LogOddsToProbability(logods1)*100);
                    Karte[points[in]] = prob1;
            }
            }
            }
        }
        ROS_WARN ("Finish CPP");
    }
    ros::Time msg_time = ros::Time::now();
    nav_msgs::OccupancyGrid pose_msg;
    geometry_msgs::Point Point1;
    geometry_msgs::Quaternion Quaternion1;

    Point1.x = -30;
    Point1.y = -30;
    Point1.z = 0;
    Quaternion1.x = 0.7071;
    Quaternion1.y = 0.7071;
    Quaternion1.z = 0;
    Quaternion1.w = 0;
    pose_msg.header.frame_id = keyframes.header.frame_id;
    pose_msg.header.stamp = msg_time;
    pose_msg.info.map_load_time =  msg_time;
    pose_msg.data = Karte;
    pose_msg.info.height = height;
    pose_msg.info.width = width;
    pose_msg.info.resolution = res;
    pose_msg.info.origin.position = Point1;
    pose_msg.info.origin.orientation = Quaternion1;
    pose_pub.publish(pose_msg);
}

int main(int argc, char **argv)
{
    std::fill(Karte.begin(),Karte.end(),50);
    ros::init(argc, argv, "occ_mapping");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;

    ros::Subscriber sub = node_handler.subscribe("/kf_ranges", 1, chatterCallback);
    setup_publishers(node_handler);


    ros::spin();

    ros::shutdown();

    return 0;
}

