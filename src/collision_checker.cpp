
#include "quad_ompl/collision_checker.hpp"

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::~CollisionChecker()
{
}

CollisionChecker::CollisionChecker(ros::NodeHandle *nh) : nh(nh)
{
    nh->subscribe(OCTOMAP_TOPIC, 0, &CollisionChecker::octomap_cb, this);
}

bool CollisionChecker::is_state_valid(const ob::State *state)
{
    return true;
}

void CollisionChecker::octomap_cb(octomap_msgs::Octomap::ConstPtr& octomap)
{
}
