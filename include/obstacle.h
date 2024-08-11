#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

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
        bool is_static;
        double x_size;
        double y_size;

        State() {
            position.setZero();
            velocity.setZero();
            orientation.setIdentity();
            angular_velocity.setIdentity();
            lifespan = 0;
            is_static = false;
            x_size = 0;
            y_size = 0;
        }
    };
    using StatePtr = std::shared_ptr<State>;
    StatePtr state_ptr_;

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