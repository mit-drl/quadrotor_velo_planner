#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ob = ompl::base;

class CollisionChecker : public ob::StateValidityChecker
{
    public:
        ~CollisionChecker();

        CollisionChecker(ros::NodeHandle *nh,
                const ob::SpaceInformationPtr &si);
        virtual bool isValid(const ob::State *state) const;
        void octomap_cb(octomap_msgs::Octomap::ConstPtr& octomap);

    private:
        ros::NodeHandle *nh;
};

#endif
