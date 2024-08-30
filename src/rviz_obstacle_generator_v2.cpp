#include "rviz_obstacle_generator_v2.h"

namespace ryu {

rvizObstacleGenerator_Node::rvizObstacleGenerator_Node(ros::NodeHandle &nh, ros::NodeHandle &nh_p){
    // Publisher setup
    std::string topic_obs, topic_obs_vis;
    nh.param<std::string>("topic_obs", topic_obs, "/tracking/car");
    nh.param<std::string>("topic_obs_vis", topic_obs_vis, "/tracking/car_marker");

    obstacle_pub_ = nh.advertise<autoware_msgs::DetectedObjectArray>(topic_obs, 10);
    obstacle_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_obs_vis, 10);
    debug_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/debug_pose", 10);

    lane_following_obstacle_topic_ = "/lane_following_obstacle";
    dynamic_obstacle_topic_ = "/dynamic_obstacle"; 
    lane_following_obstacle_sub_ = nh.subscribe(lane_following_obstacle_topic_, 10, &rvizObstacleGenerator_Node::lane_following_obstacle_callback, this);
    dynamic_obstacle_sub_ = nh.subscribe(dynamic_obstacle_topic_, 10, &rvizObstacleGenerator_Node::dynamic_obstacle_callback, this);
    sub_traj = nh.subscribe("/entire_traj", 1, &rvizObstacleGenerator_Node::globalCallback, this);


    f_ = boost::bind(&rvizObstacleGenerator_Node::reconfigure_callback, this, _1, _2);
    server_.setCallback(f_);

    nh_p.param<double>("min_lookahead", min_lookahead, 5.0);
    nh_p.param<double>("max_lookahead", max_lookahead, 10.0);
    nh_p.param<double>("max_speed", max_speed, 30.0);

    nh_p.param<double>("hz_obstacle", hz_obstacle, 10.0);
    dt = 1/hz_obstacle;
    obstacle_timer = nh.createTimer(ros::Duration(dt), &rvizObstacleGenerator_Node::publish_obstacle, this);
}

rvizObstacleGenerator_Node::~rvizObstacleGenerator_Node() {}


void rvizObstacleGenerator_Node::globalCallback(const hmcl_msgs::LaneArray& lane_msg){ 
    if(!init_global_traj){
        global_lane_array = lane_msg;
        init_global_traj = true;
    }
}

void rvizObstacleGenerator_Node::kill_obstacle() {
    obs_buf_.erase(
        std::remove_if(obs_buf_.begin(), obs_buf_.end(),
                       [this](const ObstaclePtr& obs_ptr) { return obs_ptr->obs_id == this->obs_id; }),
        obs_buf_.end());
}

void rvizObstacleGenerator_Node::change_speed() {
    for (const auto& obs_ptr : obs_buf_) {
        if(obs_ptr->obs_id == obs_id){
            obs_ptr->state_ptr_->speed = speed_km_p_h / 3.6;
        }
    }
}

void rvizObstacleGenerator_Node::change_lane() {
    for (const auto& obs_ptr : obs_buf_) {
        if(obs_ptr->obs_id == obs_id){
            obs_ptr->current_lane_id = lane_id;
        }
    }
}

void rvizObstacleGenerator_Node::reconfigure_callback(rviz_obstacle_generator::Obstaclev2Config &config, uint32_t level) {
    obs_id = config.obs_id;
    speed_km_p_h = config.speed_km_p_h;
    angular_vel_deg_p_sec = config.angular_velocity_deg_p_sec;
    duration_t_lane_following = config.duration_sec_lane_following;
    duration_t_dynamic = config.duration_sec_dynamic;
    // x_dynamic = config.x_size_meter_dynamic;
    // y_dynamic = config.y_size_meter_dynamic;
    infinite_obstacle = config.infinite_obstacle;
    // activate_function = config.Activate;
    lane_id = config.lane_id;
    // config_state = static_cast<DynamicConfigState>(config.dynamic_config_state);

    exectue_kill = config.Kill;
    exectue_update_speed = config.NewSpeed;
    exectue_update_Lane = config.NewLane;


    if(exectue_kill){
        kill_obstacle();
    }
    else if(exectue_update_speed){
        change_speed();
    }
    else if(exectue_update_Lane){
        change_lane();
    }

    // someFunctionToUpdateParameters();
}


void rvizObstacleGenerator_Node::publish_obstacle(const ros::TimerEvent&){
    update_angularvelocity();
    propagate_obstacle();

    autoware_msgs::DetectedObjectArray obstacle_array;
    visualization_msgs::MarkerArray obstacle_viz_array;
    int id = 0;

    for (const auto& obs_ptr : obs_buf_) {
        autoware_msgs::DetectedObject detected_object;
        detected_object.id = obs_ptr->obs_id;

        detected_object.pose.position.x = obs_ptr->state_ptr_->position.x();
        detected_object.pose.position.y = obs_ptr->state_ptr_->position.y();
        detected_object.pose.position.z = obs_ptr->state_ptr_->position.z();
        detected_object.pose.orientation.x = obs_ptr->state_ptr_->orientation.x();
        detected_object.pose.orientation.y = obs_ptr->state_ptr_->orientation.y();
        detected_object.pose.orientation.z = obs_ptr->state_ptr_->orientation.z();
        detected_object.pose.orientation.w = obs_ptr->state_ptr_->orientation.w();
        detected_object.velocity.linear.x = obs_ptr->state_ptr_->velocity.x();
        detected_object.velocity.linear.y = obs_ptr->state_ptr_->velocity.y();
        detected_object.velocity.linear.z = obs_ptr->state_ptr_->velocity.z();
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
        marker.lifetime = ros::Duration(dt);

        if (obs_ptr->is_lane_following) {
            marker.color.r = 1.0; 
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
        } else {
            marker.color.r = 0.0; 
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.8;
        }
        obstacle_viz_array.markers.push_back(marker);

        // 텍스트 마커 추가
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.id = id + 1; // 각 마커는 고유한 ID를 가져야 합니다.

        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose = detected_object.pose;
        text_marker.pose.position.z += 1.5; // 텍스트가 큐브 위에 위치하도록 z 좌표 조정

        text_marker.scale.z = 1.5; // 텍스트 크기 설정
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.lifetime = ros::Duration(dt);

        std::stringstream ss;
        ss << "ID: " << obs_ptr->obs_id;
        text_marker.text = ss.str(); // 텍스트 내용 설정

        obstacle_viz_array.markers.push_back(text_marker);

        id += 2; // ID를 두 개 증가시켜 CUBE와 TEXT의 ID를 구분

    }
    obstacle_pub_.publish(obstacle_array);
    obstacle_viz_pub_.publish(obstacle_viz_array);

    update_cloestIdx();
}

void rvizObstacleGenerator_Node::update_cloestIdx(){
    for (auto& obs_ptr : obs_buf_) {
        if(obs_ptr->obs_id>=0 && obs_ptr->obs_id<10){
            // int current_lane_id_ = 0;
            // double min_dist_;
            // int min_idx;
            // // for (int lane_idx = 0; lane_idx < global_lane_array.lanes.size(); lane_idx++) {
            float min_dist = 1000.0;
            float dist = 1000.0;
            int min_idx_lane = 0;
            double cur_x = obs_ptr->state_ptr_->position.x();
            double cur_y = obs_ptr->state_ptr_->position.y();            // 각 레인에 대해 최단 거리 인덱스 계산
            for (int i = 0; i < global_lane_array.lanes[obs_ptr->current_lane_id].waypoints.size(); i++) {
                dist = distance(global_lane_array.lanes[obs_ptr->current_lane_id].waypoints[i].pose.pose.position.x,
                                global_lane_array.lanes[obs_ptr->current_lane_id].waypoints[i].pose.pose.position.y,
                                cur_x, cur_y);
                if (min_dist > dist) {
                    min_dist = dist;
                    min_idx_lane = i;
                }
            }
            obs_ptr->current_lane_idx = min_idx_lane;
        }
    }
}
void rvizObstacleGenerator_Node::update_angularvelocity(){
    for (auto& obs_ptr : obs_buf_) {
        if(obs_ptr->obs_id>=0 && obs_ptr->obs_id<10){
            double speed = obs_ptr->state_ptr_->speed;
            double normalized_speed = std::min(speed / max_speed, 1.0); // 속도를 0~1로 정규화
            int lookahead = static_cast<int>(min_lookahead + normalized_speed * (max_lookahead - min_lookahead));
            int lane_size = global_lane_array.lanes[obs_ptr->current_lane_id].waypoints.size();
            int target_idx = obs_ptr->current_lane_idx + lookahead;
            
            if (target_idx >= lane_size) {
                target_idx = target_idx % lane_size; // 인덱스를 lane의 길이로 모듈러 연산하여 범위 내로 되돌림
            }

            obs_ptr->target_x = global_lane_array.lanes[obs_ptr->current_lane_id].waypoints[target_idx].pose.pose.position.x;
            obs_ptr->target_y = global_lane_array.lanes[obs_ptr->current_lane_id].waypoints[target_idx].pose.pose.position.y;
            
            // 현재 위치
            double current_x = obs_ptr->state_ptr_->position.x();
            double current_y = obs_ptr->state_ptr_->position.y();

            // 목표 지점과 현재 위치 간의 각도 차이 계산
            double target_angle = std::atan2(obs_ptr->target_y - current_y, obs_ptr->target_x - current_x);
            double current_angle = obs_ptr->state_ptr_->orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2]; // Yaw angle
            


            // 각도 차이 계산
            double angle_diff = (target_angle - current_angle);
            // std::cout <<  "!!!!!!"<< std::endl;

            // std::cout <<  "desired_angle: " << target_angle << std::endl;
            // std::cout <<  "current_angle: " << current_angle << std::endl;
            // std::cout <<  "angle_diff: " << angle_diff << std::endl;

            Eigen::Quaterniond delta_orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle_diff / dt, Eigen::Vector3d::UnitZ()));
            obs_ptr->state_ptr_->angular_velocity = delta_orientation.normalized();
        }
    }
}
void rvizObstacleGenerator_Node::propagate_obstacle(){
    for (auto& obs_ptr : obs_buf_) {


        Eigen::AngleAxisd angle_axis(dt * obs_ptr->state_ptr_->angular_velocity.angularDistance(Eigen::Quaterniond::Identity()), obs_ptr->state_ptr_->angular_velocity.vec());

        Eigen::Quaterniond delta_orientation(angle_axis);
        // obs_ptr->state_ptr_->orientation = delta_orientation * obs_ptr->state_ptr_->orientation;
        obs_ptr->state_ptr_->orientation = (delta_orientation * obs_ptr->state_ptr_->orientation).normalized();

        // std::cout<<"!!!!!!!!!!" <<std::endl;
        // std::cout << obs_ptr->state_ptr_->velocity << std::endl;
        obs_ptr->state_ptr_->velocity = obs_ptr->state_ptr_->orientation * Eigen::Vector3d(obs_ptr->state_ptr_->speed, 0, 0);
// std::cout << "Orientation (quaternion): ["
//           << obs_ptr->state_ptr_->orientation.w() << ", "
//           << obs_ptr->state_ptr_->orientation.x() << ", "
//           << obs_ptr->state_ptr_->orientation.y() << ", "
//           << obs_ptr->state_ptr_->orientation.z() << "]" << std::endl;
//         std::cout << obs_ptr->state_ptr_->velocity << std::endl;
        // obs_ptr->state_ptr_->velocity = obs_ptr->state_ptr_->orientation * Eigen::Vector3d(obs_ptr->state_ptr_->speed, 0, 0);


        obs_ptr->state_ptr_->position += obs_ptr->state_ptr_->velocity * dt;

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

double rvizObstacleGenerator_Node::distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}

void rvizObstacleGenerator_Node::initialize_obstacle(double cur_x, double cur_y, ObstaclePtr obs_ptr){
    std::vector<int> min_indices;  // 모든 레인에 대한 최소 거리 인덱스를 저장할 벡터
    std::vector<double> min_distances;  // 모든 레인에 대한 최소 거리 인덱스를 저장할 벡터
    int current_lane_id_ = 0;
    double min_dist_;
    int min_idx;

    bool empty_idx = false;
    for (int i = 0; i < 10; i++) {
        bool local_find = false;
        for (auto& obs_ptr : obs_buf_) {
            if(obs_ptr->obs_id == i){
                local_find = true;
                break;
            } 
        }
        if(!local_find){
            obs_ptr->obs_id = i;
            empty_idx = true;
            break;
        }
    }
    if(!empty_idx){
        std::cout << "There is no empty Index!!!!!" <<std::endl;
        std::cout << "There is no empty Index!!!!!" <<std::endl;
        std::cout << "There is no empty Index!!!!!" <<std::endl;
        return;
    }

    for (int lane_idx = 0; lane_idx < global_lane_array.lanes.size(); lane_idx++) {
        float min_dist = 1000.0;
        float dist = 1000.0;
        int min_idx_lane = 0;
        // 각 레인에 대해 최단 거리 인덱스 계산
        for (int i = 0; i < global_lane_array.lanes[lane_idx].waypoints.size(); i++) {
            dist = distance(global_lane_array.lanes[lane_idx].waypoints[i].pose.pose.position.x,
                            global_lane_array.lanes[lane_idx].waypoints[i].pose.pose.position.y,
                            cur_x, cur_y);
            if (min_dist > dist) {
                min_dist = dist;
                min_idx_lane = i;
            }
        }
        if(lane_idx == 0 || min_dist_ > min_dist){
          min_dist_ = min_dist;
          current_lane_id_ = lane_idx;
          min_idx = min_idx_lane;
        }
    }
    if(min_dist_ > 3){
        std::cout << "There is no near lane!!!!!" <<std::endl;
        std::cout << "There is no near lane!!!!!" <<std::endl;
        std::cout << "There is no near lane!!!!!" <<std::endl;
        obs_ptr->validity = false;
    }
    obs_ptr->current_lane_id = current_lane_id_;
    obs_ptr->current_lane_idx = min_idx;

    obs_ptr->state_ptr_->position = Eigen::Vector3d(
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.position.x),
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.position.y),
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.position.z)
    );

    obs_ptr->state_ptr_->orientation = Eigen::Quaterniond(
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.orientation.w),
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.orientation.x),
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.orientation.y),
        static_cast<double>(global_lane_array.lanes[current_lane_id_].waypoints[min_idx].pose.pose.orientation.z)
    );

    double speed_m_p_s = speed_km_p_h / 3.6;
    obs_ptr->state_ptr_->speed = speed_m_p_s;
    obs_ptr->state_ptr_->velocity = obs_ptr->state_ptr_->orientation * Eigen::Vector3d(obs_ptr->state_ptr_->speed, 0, 0);
    obs_ptr->state_ptr_->x_size = x_dynamic;
    obs_ptr->state_ptr_->y_size = y_dynamic;
    obs_ptr->state_ptr_->lifespan = int(duration_t_lane_following/dt);
    if (infinite_obstacle) {
        obs_ptr->state_ptr_->lifespan = std::numeric_limits<int>::max();
    } else {
        obs_ptr->state_ptr_->lifespan = int(duration_t_lane_following / dt);
    }
    obs_ptr->is_lane_following = true;

    // double angular_vel_rad_p_sec = angular_vel_deg_p_sec * M_PI / 180.0; 
    // Eigen::AngleAxisd angle_axis(angular_vel_rad_p_sec, Eigen::Vector3d::UnitZ());
    // obs_ptr_->state_ptr_->angular_velocity = Eigen::Quaterniond(angle_axis);  
}



void rvizObstacleGenerator_Node::lane_following_obstacle_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &lane_following_obstacle_msg){
    if(!init_global_traj){
        std::cout << "Global traj is not initialized!!!!" << std::endl;
        std::cout << "Global traj is not initialized!!!!" << std::endl;
        std::cout << "Global traj is not initialized!!!!" << std::endl;
        return;
    }


    double x = lane_following_obstacle_msg->pose.pose.position.x;
    double y = lane_following_obstacle_msg->pose.pose.position.y;
    ObstaclePtr obs_ptr_ = std::make_unique<Obstacle>();
    initialize_obstacle(x, y, obs_ptr_);

    if(obs_ptr_->validity == false) return;

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

    obs_ptr_->state_ptr_->speed = speed_m_p_s;
    obs_ptr_->state_ptr_->velocity = obs_ptr_->state_ptr_->orientation * Eigen::Vector3d(obs_ptr_->state_ptr_->speed, 0, 0);

    double angular_vel_rad_p_sec = angular_vel_deg_p_sec * M_PI / 180.0; 
    Eigen::AngleAxisd angle_axis(angular_vel_rad_p_sec, Eigen::Vector3d::UnitZ());
    obs_ptr_->state_ptr_->angular_velocity = Eigen::Quaterniond(angle_axis);  

    obs_ptr_->state_ptr_->x_size = x_dynamic;
    obs_ptr_->state_ptr_->y_size = y_dynamic;
    obs_ptr_->state_ptr_->lifespan = int(duration_t_lane_following/dt);
    obs_ptr_->obs_id = dynamic_obs_id;
    obs_ptr_->state_ptr_->speed = speed_m_p_s;

    obs_buf_.push_back(std::move(obs_ptr_));
    dynamic_obs_id++;
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
