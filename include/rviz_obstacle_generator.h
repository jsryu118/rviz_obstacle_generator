#ifndef rviz_obstacle_generator_H
#define rviz_obstacle_generator_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DetectedObjectArray.h>


#include <dynamic_reconfigure/server.h>
#include <rviz_obstacle_generator/ObstacleConfig.h> 
#include <deque>


#include "obstacle.h"
#include <iostream>


namespace ryu {

class rvizObstacleGenerator_Node {
   public:
    rvizObstacleGenerator_Node(ros::NodeHandle &nh, ros::NodeHandle &nh_p);
    ~rvizObstacleGenerator_Node();
    
    void reconfigure_callback(rviz_obstacle_generator::ObstacleConfig &config, uint32_t level);
    
    void static_obstacle_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &static_obstacle_msg);
    void dynamic_obstacle_callback(const geometry_msgs::PoseStampedConstPtr &dynamic_obstacle_msg);
    void publish_obstacle(const ros::TimerEvent&);
    void propagate_obstacle();

   private:
    ros::Publisher obstacle_pub_;
    ros::Publisher obstacle_viz_pub_;
    ros::Subscriber static_obstacle_sub_;
    ros::Subscriber dynamic_obstacle_sub_;
    ros::Timer obstacle_timer;

    double speed_km_p_h, angular_vel_deg_p_sec, duration_t_static, duration_t_dynamic,\
        x_static, y_static, x_dynamic, y_dynamic;
    dynamic_reconfigure::Server<rviz_obstacle_generator::ObstacleConfig> server_;
    dynamic_reconfigure::Server<rviz_obstacle_generator::ObstacleConfig>::CallbackType f_;
    
    double hz_obstacle, dt;

    std::string static_obstacle_topic_, dynamic_obstacle_topic_;

    std::deque<ObstaclePtr> obs_buf_;

};

}  // namespace ryu

#endif // rviz_obstacle_generator_H
