//
// Created by songjin on 18-9-4.
//

#ifndef PATH_PLANNING_PLANNER_HPP
#define PATH_PLANNING_PLANNER_HPP

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/PlannerIncludes.h>

#include <octomap/octomap.h>

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"

#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"

#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/bvh/BVH_model.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_result.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Vec3 {
    double x;
    double y;
    double z;
};

class Planner {
public:
    Planner();

    void updateMap(std::shared_ptr<const octomap::OcTree> octomap);

    bool planPath(const Vec3& start_vec, const Vec3& goal_vec, std::vector<Vec3>& v_path);

    // rrt range
    double step_length;

private:
    bool isStateValid(const ob::State *state);
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);
private:
    // construct the state space we are planning in
    ob::StateSpacePtr space;

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef;

    // create a specific planner
    ob::PlannerPtr planner;

    bool replan_flag = false;

    std::shared_ptr<fcl::CollisionGeometryf> quadcopter_object;
    std::shared_ptr<fcl::CollisionGeometryf> map_object;
};


#endif //PATH_PLANNING_PLANNER_HPP
