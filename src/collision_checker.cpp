
#include <octomap_msgs/conversions.h>
#include "quadrotor_velo_planner/clustering.hpp"
#include "quadrotor_velo_planner/collision_checker.hpp"

using namespace std;

string PC_TOPIC = "cloud";

CollisionChecker::~CollisionChecker()
{
}

CollisionChecker::CollisionChecker(
        ros::NodeHandle *nh,
        const ob::SpaceInformationPtr &si) :
    nh(nh), ob::StateValidityChecker(si)
{
    cloud_sub = nh->subscribe(PC_TOPIC, 1, &CollisionChecker::cloud_cb, this);
}

bool CollisionChecker::isValid(const ob::State *state) const
{
    double x = state->as<ob::SE3StateSpace::StateType>()->getX();
    double y = state->as<ob::SE3StateSpace::StateType>()->getY();
    double z = state->as<ob::SE3StateSpace::StateType>()->getZ();
    return is_point_valid(x, y, z);
}

bool CollisionChecker::is_point_valid(double x0, double y0, double z0) const
{
    return true;
}

void CollisionChecker::cloud_cb(const PointCloud2Ptr msg)
{
    cloud = msg;
    clustering.get_euclidean_clusters(msg, clusters);
}
