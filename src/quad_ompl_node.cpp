#include "ros/ros.h"

#include "quad_ompl/quad_planner.hpp"

QuadPlanner::QuadPlanner(/* your arguments here */)
{
	// construct the state space we are planning in
	space = ob::StateSpacePtr(new ob::SE3StateSpace());

    // set the bounds for the R^3 part of SE(3)
    bounds = new ob::RealVectorBounds(3);
    bounds->setLow(-10);
    bounds->setHigh(10);

    space->as<ob::SE3StateSpace>()->setBounds(*bounds);

    // construct an instance of  space information from this state space
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    // create a problem instance
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));


    // create a planner for the defined space
    planner = ob::PlannerPtr(new og::RRTConnect(si));
}

QuadPlanner::~QuadPlanner()
{
	delete bounds;
}

ob::ScopedState<ob::SE3StateSpace> 
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
nav_msgs::Path QuadPlanner::create_plan(
        const nav_msgs::Odometry::ConstPtr& odom,
        const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	// set start state
    auto start = convertToScopedState(odom->pose.pose);
    // set goal state
	auto goal = convertToScopedState(pose->pose);

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->solve(1.0);
}

int main(int, char **)
{
    return 0;
}
