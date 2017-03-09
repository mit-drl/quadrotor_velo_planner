#include "ros/ros.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "quad_ompl/collision_checker.hpp"
#include "quad_ompl/quad_planner.hpp"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

string NODE_NAME = "quad_ompl_node";

bool isStateValid(const ob::State *state) {
	return true;
}

QuadPlanner::QuadPlanner(ros::NodeHandle *nh)
{
	// construct the state space we are planning in
	space = ob::StateSpacePtr(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
    bounds = new ob::RealVectorBounds(3);

    // need to make these parameters
    bounds->setLow(-10);
    bounds->setHigh(10);

    space->as<ob::SE3StateSpace>()->setBounds(*bounds);

    // construct an instance of  space information from this state space
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

	goal_pose = geometry_msgs::PoseStamped::ConstPtr(new geometry_msgs::PoseStamped());
	start_pose = nav_msgs::Odometry::ConstPtr(new nav_msgs::Odometry());
    std::shared_ptr<CollisionChecker> cc = std::make_shared<CollisionChecker>(nh, si);

    // set state validity checking for this space
    si->setStateValidityChecker(cc);

    // create a problem instance
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));


    std::string goal_topic = "/setpoint_goal";
    std::string start_topic = "/odometry/filtered";
    std::string path_topic = "/ompl_path";
    this->goal_sub = nh->subscribe(goal_topic, 1, &QuadPlanner::goal_callback, this);
    this->start_sub = nh->subscribe(start_topic, 1, &QuadPlanner::start_callback, this);
    this->path_pub = nh->advertise<nav_msgs::Path>(path_topic, 10);

    run();

}

QuadPlanner::~QuadPlanner()
{
	delete bounds;
}

void QuadPlanner::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& ps) {
	goal_pose = ps;
}

void QuadPlanner::start_callback(const nav_msgs::Odometry::ConstPtr& odom) {
	start_pose = odom;
}

const ob::ScopedState<ob::SE3StateSpace>
QuadPlanner::convertToScopedState(const geometry_msgs::Pose pose)
{
	geometry_msgs::Point point = pose.position;
	geometry_msgs::Quaternion quat = pose.orientation;
	ob::ScopedState<ob::SE3StateSpace> scopedState(space);
    scopedState->setX(point.x);
    scopedState->setY(point.y);
    scopedState->setZ(point.z);
    scopedState->rotation().x = quat.x;
    scopedState->rotation().y = quat.y;
    scopedState->rotation().z = quat.z;
    scopedState->rotation().w = quat.w;

    return scopedState;
}

// return a plan from odom to pose
ob::PlannerStatus QuadPlanner::plan(
        const nav_msgs::Odometry::ConstPtr& odom,
        const geometry_msgs::PoseStamped::ConstPtr& pose,
        nav_msgs::Path& path_plan)
{
    auto start = convertToScopedState(odom->pose.pose);
	auto goal = convertToScopedState(pose->pose);
    pdef->setStartAndGoalStates(start, goal);
    ob::PlannerPtr planner(new og::RRTstar(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // make timeout a parameters
    ob::PlannerStatus solved = planner->solve(0.1);

	if (solved)
    {
        ob::PathPtr path_ptr = pdef->getSolutionPath();
        og::PathGeometric *path = path_ptr->as<og::PathGeometric>();
        path_plan.header.frame_id = "body";
        for (int i = 0; i < path->getStateCount(); i++)
        {
            ob::SE3StateSpace::StateType *state = path->getState(i)
                ->as<ob::SE3StateSpace::StateType>();
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "body";
            ps.pose.position.x = state->getX();
            ps.pose.position.y = state->getY();
            ps.pose.position.z = state->getZ();
            path_plan.poses.push_back(ps);
        }
    }

    return solved;
}

void QuadPlanner::run(){
    // make rate a parameter
	ros::Rate loop_rate(10);
	while(ros::ok()){
        ros::spinOnce();

        nav_msgs::Path path;
        ob::PlannerStatus status = QuadPlanner::plan(
                start_pose, goal_pose, path);

        if (status)
        {
            this->path_pub.publish(path);
        }

		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle *nh = new ros::NodeHandle;

    QuadPlanner qp(nh);

    ros::spin();
    return 0;
}
