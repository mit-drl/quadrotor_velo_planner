#ifndef QUAD_PLANNER_HPP
#define QUAD_PLANNER_HPP

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class QuadPlanner
{
    // your private variables and functions go here
    private:
        ob::ScopedState<ob::SE3StateSpace> 
        convertToScopedState(
            const geometry_msgs::Pose pose);

    // your public variables and functions go here
    public:
        ob::StateSpacePtr space;
        ob::RealVectorBounds *bounds;
        ob::SpaceInformationPtr si;
        ob::ProblemDefinitionPtr pdef;
        ob::PlannerPtr planner;

        ~QuadPlanner();
        QuadPlanner(/* your arguments here */);
        nav_msgs::Path create_plan(
                const nav_msgs::Odometry::ConstPtr& odom,
                const geometry_msgs::PoseStamped::ConstPtr& pose);
};

#endif
