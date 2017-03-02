
#include <octomap_msgs/conversions.h>
#include "quad_ompl/collision_checker.hpp"

using namespace std;

string OCTOMAP_TOPIC = "octomap_binary";

CollisionChecker::~CollisionChecker()
{
}

CollisionChecker::CollisionChecker(
        ros::NodeHandle *nh, const ob::SpaceInformationPtr &si) :
    nh(nh), ob::StateValidityChecker(si)
{
    octo_sub = nh->subscribe(OCTOMAP_TOPIC, 1,
            &CollisionChecker::octomap_cb, this);
}

bool CollisionChecker::isValid(const ob::State *state) const
{
    if (tree == NULL)
    {
        return false;
    }

    double x = state->as<ob::SE3StateSpace::StateType>()->getX();
    double y = state->as<ob::SE3StateSpace::StateType>()->getY();
    double z = state->as<ob::SE3StateSpace::StateType>()->getZ();
    return is_point_valid(x, y, z);
}

bool CollisionChecker::is_point_valid(double x0, double y0, double z0) const
{
    double res = tree->getResolution();
    double clearance = 1.0;
    int n_voxels = clearance / res + 1;
    double x, y, z;

    for (int i = 0; i < 2 * n_voxels; i++)
    {
        for (int j = 0; j < 2 * n_voxels; j++)
        {
            for (int k = 0; k < 2 * n_voxels; k++)
            {
                x = x0 - (i - n_voxels) * res;
                y = y0 - (j - n_voxels) * res;
                z = z0 - (k - n_voxels) * res;
                octomap::point3d query(x, y, z);
                octomap::OcTreeNode *res = tree->search(query);
                if (res != NULL and tree->isNodeOccupied(res))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

void CollisionChecker::octomap_cb(
        const octomap_msgs::Octomap::ConstPtr& msg)
{
    if (tree != NULL)
    {
        delete tree;
    }

    tree = new octomap::OcTree(msg->resolution);
    octomap_msgs::readTree(tree, *msg);
}
