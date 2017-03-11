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
#include <ros/ros.h>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
typedef nav_msgs::Odometry::ConstPtr OdomPtr;
typedef geometry_msgs::PoseStamped::ConstPtr PoseStampedPtr;

class QuadPlanner
{
    private:
        ob::ScopedState<ob::SE3StateSpace> convertToScopedState(
            const geometry_msgs::Pose& pose);
        ros::NodeHandle *nh;

    public:
        ob::StateSpacePtr space;
        ob::RealVectorBounds *bounds;
        ob::SpaceInformationPtr si;
        ob::ProblemDefinitionPtr pdef;

        ros::Subscriber goal_sub;
        ros::Subscriber odom_sub;
        ros::Publisher path_pub;
        PoseStampedPtr goal_pose;
        OdomPtr start_pose;


        ~QuadPlanner();
        QuadPlanner(ros::NodeHandle *nh);
        void start();
        void goal_cb(const PoseStampedPtr& ps);
        void odom_cb(const OdomPtr& odom);
        ob::PlannerStatus plan(
                const OdomPtr& odom,
                const PoseStampedPtr& pose,
                nav_msgs::Path& path_plan);
};

#endif
