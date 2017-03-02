#ifndef QUAD_PLANNER_HPP
#define QUAD_PLANNER_HPP

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class QuadPlanner
{
    // your private variables and functions go here
    private:
        int dummy_variable;

    // your public variables and functions go here
    public:
        ~QuadPlanner();
        QuadPlanner(/* your arguments here */);
        nav_msgs::Path create_plan(
                const nav_msgs::Odometry::ConstPtr& odom,
                const geometry_msgs::PoseStamped::ConstPtr& pose);
};

#endif
