
#include "ros/ros.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include "quadrotor_velo_planner/collision_checker.hpp"
#include "quadrotor_velo_planner/quad_planner.hpp"

string NODE_NAME = "quad_ompl_node";
// string ODOM_TOPIC = "/odometry/filtered";
string ODOM_TOPIC = "/ground_truth_to_tf/pose";
string GOAL_TOPIC = "/setpoint_goal";
string PATH_TOPIC = "/ompl_path";
string FIXED_FRAME = "world";
string QUAD_FRAME = "base_link";

QuadPlanner::QuadPlanner(ros::NodeHandle *nh) :
    nh(nh),
    space(ob::StateSpacePtr(new ob::SE3StateSpace())),
    bounds(new ob::RealVectorBounds(3)),
    si(ob::SpaceInformationPtr(new ob::SpaceInformation(space))),
    goal_pose(PoseStampedPtr(new geometry_msgs::PoseStamped())),
    start_pose(PoseStampedPtr(new geometry_msgs::PoseStamped()))
    // start_pose(PosePtr(new nav_msgs::Odometry()))
{
    // need to make these parameters
    bounds->setLow(-10);
    bounds->setHigh(10);
    space->as<ob::SE3StateSpace>()->setBounds(*bounds);
    // shared_ptr<CollisionChecker> cc = make_shared<CollisionChecker>(nh, si);
    // si->setStateValidityChecker(cc);
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
}

QuadPlanner::~QuadPlanner()
{
	delete bounds;
}

void QuadPlanner::goal_cb(const PoseStampedPtr& ps)
{
	goal_pose = ps;
}

void QuadPlanner::odom_cb(const PoseStampedPtr& ps)
{
    start_pose = ps;
}

ob::ScopedState<ob::SE3StateSpace>
QuadPlanner::convertToScopedState(const geometry_msgs::Pose& pose)
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

bool QuadPlanner::get_trans_to_quad(string tag_id, tf::StampedTransform trans)
{
    try
    {
        tfl.lookupTransform(tag_id, QUAD_FRAME, ros::Time(0), trans);
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

// return a plan from odom to pose
ob::PlannerStatus QuadPlanner::plan(
        const PoseStampedPtr& odom,
        const PoseStampedPtr& pose,
        nav_msgs::Path& path_plan)
{

    vector<tf::StampedTransform> tags;
    for (string id : tag_ids)
    {
        tf::StampedTransform trans;
        get_trans_to_quad(id, trans);
        tags.push_back(trans);
    }

    auto start = convertToScopedState(odom->pose);
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
        path_plan.header.frame_id = FIXED_FRAME;

        for (int i = 0; i < path->getStateCount(); i++)
        {
            ob::SE3StateSpace::StateType *state = path->getState(i)
                ->as<ob::SE3StateSpace::StateType>();
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = FIXED_FRAME;
            ps.pose.position.x = state->getX();
            ps.pose.position.y = state->getY();
            ps.pose.position.z = state->getZ();
            path_plan.poses.push_back(ps);
        }
    }

    return solved;
}

void QuadPlanner::start()
{
    goal_sub = nh->subscribe(GOAL_TOPIC, 1, &QuadPlanner::goal_cb, this);
    odom_sub = nh->subscribe(ODOM_TOPIC, 1, &QuadPlanner::odom_cb, this);
    path_pub = nh->advertise<nav_msgs::Path>(PATH_TOPIC, 10);

    // make rate a parameter
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
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
    nh->getParam("tag_ids", tag_ids);

    QuadPlanner qp(nh);
    qp.start();
    ros::spin();
    return 0;
}
