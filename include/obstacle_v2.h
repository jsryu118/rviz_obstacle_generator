#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <hmcl_msgs/Lane.h>

namespace ryu {

class Obstacle {
   public:
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // nominal-state
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Quaterniond angular_velocity;
        double lifespan;
        double x_size;
        double y_size;
        double speed;

        State() {
            position.setZero();
            velocity.setZero();
            orientation.setIdentity();
            angular_velocity.setIdentity();
            lifespan = 0;
            x_size = 0;
            y_size = 0;
            speed = 0;
        }
    };
    using StatePtr = std::shared_ptr<State>;
    StatePtr state_ptr_;
    bool validity = true;
    int current_lane_id;
    int current_lane_idx;
    int obs_id;
    bool is_lane_following = false;
    double target_x;
    double target_y;
    bool killsign = false;
    
    // Obstacle() = delete;

    Obstacle(const Obstacle &) = delete;

    explicit Obstacle(){
        state_ptr_ = std::make_shared<State>();
    }


    ~Obstacle() {}

   private:
};

using ObstaclePtr = std::shared_ptr<Obstacle>;

}  // namespace ryu