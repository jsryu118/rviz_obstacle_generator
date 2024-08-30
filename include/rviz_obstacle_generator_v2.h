#ifndef rviz_obstacle_generator_H
#define rviz_obstacle_generator_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <hmcl_msgs/LaneArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>

#include <rviz_obstacle_generator/Obstaclev2Config.h> 
#include <deque>

#include "obstacle_v2.h"
#include <iostream>


namespace ryu {
enum DynamicConfigState{
    CHANGE_SPEED = 0,
    CHANGE_LANE = 1,
    KILL = 2
};

class rvizObstacleGenerator_Node {
   public:
    rvizObstacleGenerator_Node(ros::NodeHandle &nh, ros::NodeHandle &nh_p);
    ~rvizObstacleGenerator_Node();
    
    void someFunctionToUpdateParameters();
    void reconfigure_callback(rviz_obstacle_generator::Obstaclev2Config &config, uint32_t level);
    void globalCallback(const hmcl_msgs::LaneArray& lane_msg);

    void lane_following_obstacle_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &lane_following_obstacle_msg);
    void dynamic_obstacle_callback(const geometry_msgs::PoseStampedConstPtr &dynamic_obstacle_msg);
    void publish_obstacle(const ros::TimerEvent&);
    void propagate_obstacle();
    void initialize_obstacle(double cur_x, double cur_y, ObstaclePtr obs_ptr);
    // void compute_local_path();
    double distance(double x1, double y1, double x2, double y2);
    hmcl_msgs::LaneArray map_loader(const std::string osm_path);
    void update_angularvelocity();
    void update_cloestIdx();
    bool duplicate_check();
    void kill_obstacle();
    void change_speed();
    void change_lane();

   private:
    ros::Publisher obstacle_pub_;
    ros::Publisher obstacle_viz_pub_;
    ros::Publisher debug_pose_pub_;
    ros::Subscriber lane_following_obstacle_sub_, sub_traj;
    ros::Subscriber dynamic_obstacle_sub_;
    ros::Timer obstacle_timer;

    double speed_km_p_h, angular_vel_deg_p_sec, duration_t_lane_following, duration_t_dynamic,\
        x_lane_following, y_lane_following;

    double x_dynamic = 4.650;
    double y_dynamic = 1.825;
    dynamic_reconfigure::Server<rviz_obstacle_generator::Obstaclev2Config> server_;
    dynamic_reconfigure::Server<rviz_obstacle_generator::Obstaclev2Config>::CallbackType f_;
    // dynamic_reconfigure::Client<rviz_obstacle_generator::Obstaclev2Config> dr_client_;

    double hz_obstacle, dt;

    std::string lane_following_obstacle_topic_, dynamic_obstacle_topic_;

    std::deque<ObstaclePtr> obs_buf_;
    bool init_global_traj = false;
    hmcl_msgs::LaneArray global_lane_array, global_joker_lane_array;
    int obs_id;
    int dynamic_obs_id = 10;
    DynamicConfigState config_state;


    double min_lookahead = 20.0;
    double max_lookahead = 50.0;
    double max_speed = 30.0; // 최대 속도 (예시로 30 m/s)
    bool activate_function = false;
    bool infinite_obstacle;
    int lane_id;

    bool exectue_kill, exectue_update_speed, exectue_update_Lane;
};



}  // namespace ryu

#endif // rviz_obstacle_generator_H
