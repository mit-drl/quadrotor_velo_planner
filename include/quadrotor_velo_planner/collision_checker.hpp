#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <sensor_msgs/PointCloud2.h>
#include "quadrotor_velo_planner/clustering.hpp"

typedef sensor_msgs::PointCloud2::ConstPtr PointCloud2Ptr;
namespace ob = ompl::base;

class CollisionChecker : public ob::StateValidityChecker
{
    public:
        ~CollisionChecker();

        CollisionChecker(ros::NodeHandle *nh,
                const ob::SpaceInformationPtr &si);
        virtual bool isValid(const ob::State *state) const;
        void cloud_cb(const PointCloud2Ptr cloud);
        void set_tags(vector<tf::StampedTransform>);

    private:
        ros::NodeHandle *nh;
        ros::Subscriber cloud_sub;
        PointCloud2Ptr cloud;
        Clustering clustering;
        vector<PCLPointCloudPtr> clusters;
        vector<tf::StampedTransform> tags;
};

#endif
