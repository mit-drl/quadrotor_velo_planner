#include "ros/ros.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "quad_ompl/collision_checker.hpp"
#include "quad_ompl/quad_planner.hpp"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

string NODE_NAME = "quad_ompl_node";

QuadPlanner::QuadPlanner(/* your arguments here */)
{
}

QuadPlanner::~QuadPlanner()
{
    // destructor... don't worry about it
}

nav_msgs::Path QuadPlanner::create_plan(
        const nav_msgs::Odometry::ConstPtr& odom,
        const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    // brandons code here
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle *nh = new ros::NodeHandle;

    CollisionChecker cc(nh);

    ros::spin();
    return 0;
}
