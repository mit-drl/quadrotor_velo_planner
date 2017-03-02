
#include "quad_ompl/collision_checker.hpp"

using namespace std;

string OCTOMAP_TOPIC = "octomap";

CollisionChecker::~CollisionChecker()
{
}

CollisionChecker::CollisionChecker(
        ros::NodeHandle *nh,
        const ob::SpaceInformationPtr &si) :
    nh(nh), ob::StateValidityChecker(si)
{
    // nh->subscribe(OCTOMAP_TOPIC, 0, &CollisionChecker::octomap_cb, this);
}

bool CollisionChecker::isValid(const ob::State *state) const
{
    return true;
}

void CollisionChecker::octomap_cb(octomap_msgs::Octomap::ConstPtr& octomap)
{
}
