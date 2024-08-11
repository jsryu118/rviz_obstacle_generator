#include "rviz_obstacle_generator.h"

namespace ryu {

rvizObstacleGenerator_Node::rvizObstacleGenerator_Node(ros::NodeHandle &nh, ros::NodeHandle &nh_p) {
    // Publisher setup

    obstacle_pub_ = nh.advertise<autoware_msgs::DetectedObjectArray>("/tracking_side/objects", 10);
    obstacle_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/tracking_side/objects_markers", 10);

    static_obstacle_topic_ = "/static_obstacle";
    dynamic_obstacle_topic_ = "/dynamic_obstacle"; 
    static_obstacle_sub_ = nh.subscribe(static_obstacle_topic_, 10, &rvizObstacleGenerator_Node::static_obstacle_callback, this);
    dynamic_obstacle_sub_ = nh.subscribe(dynamic_obstacle_topic_, 10, &rvizObstacleGenerator_Node::dynamic_obstacle_callback, this);

    // Dynamic reconfigure setup
    f_ = boost::bind(&rvizObstacleGenerator_Node::reconfigure_callback, this, _1, _2);
    server_.setCallback(f_);

    nh_p.param<double>("hz_obstacle", hz_obstacle, 10.0);
    dt = 1/hz_obstacle;
    obstacle_timer = nh.createTimer(ros::Duration(dt), &rvizObstacleGenerator_Node::publish_obstacle, this);

}

rvizObstacleGenerator_Node::~rvizObstacleGenerator_Node() {}

void rvizObstacleGenerator_Node::reconfigure_callback(rviz_obstacle_generator::ObstacleConfig &config, uint32_t level) {
    speed_km_p_h = config.speed_km_p_h;
    angular_vel_deg_p_sec = config.angular_velocity_deg_p_sec;
    duration_t_static = config.duration_sec_static;
    duration_t_dynamic = config.duration_sec_dynamic;
    x_static = config.x_size_meter_static;
    y_static = config.y_size_meter_static;
    x_dynamic = config.x_size_meter_dynamic;
    y_dynamic = config.y_size_meter_dynamic;
}


void rvizObstacleGenerator_Node::publish_obstacle(const ros::TimerEvent&){
    autoware_msgs::DetectedObjectArray obstacle_array;
    visualization_msgs::MarkerArray obstacle_viz_array;
    int id = 0;

    for (const auto& obs_ptr : obs_buf_) {
        autoware_msgs::DetectedObject detected_object;
        detected_object.id = id;

        detected_object.pose.position.x = obs_ptr->state_ptr_->position.x();
        detected_object.pose.position.y = obs_ptr->state_ptr_->position.y();
        detected_object.pose.position.z = obs_ptr->state_ptr_->position.z();
        detected_object.pose.orientation.x = obs_ptr->state_ptr_->orientation.x();
        detected_object.pose.orientation.y = obs_ptr->state_ptr_->orientation.y();
        detected_object.pose.orientation.z = obs_ptr->state_ptr_->orientation.z();
        detected_object.pose.orientation.w = obs_ptr->state_ptr_->orientation.w();

        detected_object.dimensions.x = obs_ptr->state_ptr_->x_size; 
        detected_object.dimensions.y = obs_ptr->state_ptr_->y_size;
        detected_object.dimensions.z = 1.0; 

        obstacle_array.objects.push_back(detected_object);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.id = id;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = detected_object.pose;
        marker.scale.x = obs_ptr->state_ptr_->x_size; 
        marker.scale.y = obs_ptr->state_ptr_->y_size;
        marker.scale.z = 1.0;

        if (obs_ptr->state_ptr_->is_static){
            marker.color.r = 1.0; 
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        }
        else{
            marker.color.r = 0.0; 
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.8;
        }
        obstacle_viz_array.markers.push_back(marker);
        id++;

    }
    obstacle_pub_.publish(obstacle_array);
    obstacle_viz_pub_.publish(obstacle_viz_array);

    propagate_obstacle();
}

void rvizObstacleGenerator_Node::propagate_obstacle(){
    for (auto& obs_ptr : obs_buf_) {
        obs_ptr->state_ptr_->position += obs_ptr->state_ptr_->velocity * dt;

        Eigen::AngleAxisd angle_axis(dt * obs_ptr->state_ptr_->angular_velocity.angularDistance(Eigen::Quaterniond::Identity()), obs_ptr->state_ptr_->angular_velocity.vec());
        Eigen::Quaterniond delta_orientation(angle_axis);
        obs_ptr->state_ptr_->orientation = delta_orientation * obs_ptr->state_ptr_->orientation;
        
        obs_ptr->state_ptr_->velocity = obs_ptr->state_ptr_->orientation * Eigen::Vector3d(obs_ptr->state_ptr_->velocity.norm(), 0, 0);

        // 수명 감소
        if (obs_ptr->state_ptr_->lifespan > 0) {
            obs_ptr->state_ptr_->lifespan -= 1;
        }
    }

    // 수명이 다한 장애물 제거
    obs_buf_.erase(
        std::remove_if(obs_buf_.begin(), obs_buf_.end(),
                       [](const ObstaclePtr& obs_ptr) { return obs_ptr->state_ptr_->lifespan <= 0; }),
        obs_buf_.end());
}

void rvizObstacleGenerator_Node::static_obstacle_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &static_obstacle_msg){
    ObstaclePtr obs_ptr_ = std::make_unique<Obstacle>();

    obs_ptr_->state_ptr_->position = Eigen::Vector3d(
        static_obstacle_msg->pose.pose.position.x,
        static_obstacle_msg->pose.pose.position.y,
        static_obstacle_msg->pose.pose.position.z
    );
    
    obs_ptr_->state_ptr_->orientation = Eigen::Quaterniond(
        static_obstacle_msg->pose.pose.orientation.w,
        static_obstacle_msg->pose.pose.orientation.x,
        static_obstacle_msg->pose.pose.orientation.y,
        static_obstacle_msg->pose.pose.orientation.z
    );

    obs_ptr_->state_ptr_->is_static = true;
    obs_ptr_->state_ptr_->x_size = x_static;
    obs_ptr_->state_ptr_->y_size = y_static;

    obs_ptr_->state_ptr_->lifespan = int(duration_t_static/dt);
    obs_buf_.push_back(std::move(obs_ptr_));


}

void rvizObstacleGenerator_Node::dynamic_obstacle_callback(const geometry_msgs::PoseStampedConstPtr &dynamic_obstacle_msg){
    ObstaclePtr obs_ptr_ = std::make_unique<Obstacle>();

    obs_ptr_->state_ptr_->position = Eigen::Vector3d(
        dynamic_obstacle_msg->pose.position.x,
        dynamic_obstacle_msg->pose.position.y,
        dynamic_obstacle_msg->pose.position.z
    );
    
    obs_ptr_->state_ptr_->orientation = Eigen::Quaterniond(
        dynamic_obstacle_msg->pose.orientation.w,
        dynamic_obstacle_msg->pose.orientation.x,
        dynamic_obstacle_msg->pose.orientation.y,
        dynamic_obstacle_msg->pose.orientation.z
    );

    double speed_m_p_s = speed_km_p_h / 3.6;
    obs_ptr_->state_ptr_->velocity = obs_ptr_->state_ptr_->orientation * Eigen::Vector3d(speed_m_p_s, 0, 0);

    double angular_vel_rad_p_sec = angular_vel_deg_p_sec * M_PI / 180.0; 
    Eigen::AngleAxisd angle_axis(angular_vel_rad_p_sec, Eigen::Vector3d::UnitZ());
    obs_ptr_->state_ptr_->angular_velocity = Eigen::Quaterniond(angle_axis);  

    obs_ptr_->state_ptr_->x_size = x_dynamic;
    obs_ptr_->state_ptr_->y_size = y_dynamic;
    obs_ptr_->state_ptr_->lifespan = int(duration_t_static/dt);
    obs_buf_.push_back(std::move(obs_ptr_));
}

}  // namespace ryu

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_obstacle_generator_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    ryu::rvizObstacleGenerator_Node rviz_obstacle_generator_node(nh, nh_p);

    ros::spin();

    return 0;
}
