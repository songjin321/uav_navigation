//
// Created by songjin on 18-9-4.
//
#include "Planner.h"
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include "functional"
#include <chrono>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

typedef std::chrono::high_resolution_clock Clock;

Planner::Planner() {
    //四旋翼的障碍物几何形状
    quadcopter_object = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Box<float>(0.8, 0.8, 0.3));

    //分辨率参数设置
    fcl::OcTree<float>* tree = new fcl::OcTree<float>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.10)));
    map_object = std::shared_ptr<fcl::CollisionGeometryf>(tree);

    //解的状态空间
    space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    // construct an instance of  space information from this state space
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // set state validity checking for this space
    // si->setStateValidityChecker(isStateValid);
    si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));

    std::cout << "rrt planner initialized OK!" << std::endl;
}

void Planner::updateMap(std::shared_ptr<const octomap::OcTree> octomap)
{
    map_object = std::shared_ptr<fcl::CollisionGeometryf>(new fcl::OcTree<float>(octomap));
}

bool Planner::planPath(const Vec3& start_vec, const Vec3& goal_vec, std::vector<Vec3>& v_path)
{
    // 搜索的三维范围设置
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0,-5); bounds.setHigh(0,5);
    bounds.setLow(1,-5); bounds.setHigh(1,5);
    // bounds.setLow(2, start_vec.z - 0.1); bounds.setHigh(2, start_vec.z + 0.1);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // start
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start->values[0] = start_vec.x;
    start->values[1] = start_vec.y;
    planned_height = start_vec.z;
    // start->values[2] = start_vec.z;

    // goal
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal->values[0] = goal_vec.x;
    goal->values[1] = goal_vec.y;
    // goal->values[2] = goal_vec.z;

    // create a problem instance
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // set Optimizattion objective
    pdef->setOptimizationObjective(Planner::getThresholdPathLengthObj(si));

    //设置rrt的参数range
    auto rrt = new og::InformedRRTstar(si);
    // rrt->setGoalBias(0.05);
    rrt->setRange(step_length);

    // create a planner for the defined space
    planner = ob::PlannerPtr(rrt);

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the problem settings
    // std::cout << std::endl;
    // pdef->print(std::cout);
    // std::cout << std::endl;

    // print the settings for this space
    // si->printSettings(std::cout);

    auto t1 = Clock::now();
    ob::PlannerStatus solved = planner->solve(3);
    auto t2 = Clock::now();
    std::cout << "solved a rrt path cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " miliseconds" << std::endl;

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        //B样条曲线优化,并且保证优化后的路径和地图不碰撞
        auto pathSimplifier = new og::PathSimplifier(si);
        auto path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));

        auto t1 = Clock::now();
        pathSimplifier->smoothBSpline(*path_smooth);
        auto t2 = Clock::now();
        std::cout << "smooth a rrt path cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " miliseconds" << std::endl;

        v_path.clear();
        v_path.reserve(path_smooth->getStateCount());

        // print the path to screen
        // pdef->getSolutionPath()->print(std::cout);

        for (std::size_t path_idx = 0; path_idx < path_smooth->getStateCount(); path_idx++){

            auto pos = path_smooth->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();
            Vec3 path_point;
            path_point.x = pos->values[0];
            path_point.y = pos->values[1];
            path_point.z = planned_height;
            v_path.push_back(path_point);
        }

        return true;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return false;
    }

    planner->clear();
}

bool Planner::isStateValid(const ob::State *state)
{
    auto t1 = Clock::now();
    // cast the abstract state type to the type we expect
    const auto pos = state->as<ob::RealVectorStateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    // const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    // const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    fcl::CollisionObjectf map_collision(map_object);
    fcl::CollisionObjectf quadcopter_collision(quadcopter_object);

    // check validity of state defined by pos & rot
    Eigen::Vector3f translation(pos->values[0],pos->values[1], planned_height);
    Eigen::Quaternionf rotation(1, 0, 0, 0);
    quadcopter_collision.setTransform(rotation, translation);
    fcl::CollisionRequestf requestType(1,false,1,false);
    fcl::CollisionResultf collisionResult;
    fcl::collide<float>(&quadcopter_collision, &map_collision, requestType, collisionResult);

    auto t2 = Clock::now();
    // std::cout << "valid a state cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " miliseconds" << std::endl;

    //return (const void*)rot != (const void*)pos;
    return(!collisionResult.isCollision());
}

ob::OptimizationObjectivePtr Planner::getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    // obj->setCostThreshold(ob::Cost(100));
    return obj;
}