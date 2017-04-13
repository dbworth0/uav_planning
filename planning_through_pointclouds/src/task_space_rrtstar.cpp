/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Copyright 2014 Sanjiban Choudhury
 * rrtstar.cpp
 *
 *  Created on: Aug 23, 2014
 *      Author: Sanjiban Choudhury
 */








// ToDo: SEE WHAT THE CHANGES ARE

#include <planning_through_pointclouds/task_space_rrtstar.h>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <algorithm>
#include <limits>
#include <map>

#include <boost/math/constants/constants.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>




using namespace task_space_rrtstar::rrtstar;
//namespace ob = ompl::base;
//namespace pc = ca::planning_common;

namespace task_space_rrtstar
{

namespace
{


bool CheckMotionNominal(const ompl::base::State *parent, const ompl::base::State *child, const ompl::base::SpaceInformationPtr &si) {
  return si->checkMotion(parent, child);
}

bool CheckMotionTaskSpace(const ompl::base::State *parent, const ompl::base::State *child, const ompl::base::SpaceInformationPtr &si, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  ompl::base::State *full_state = si->allocState();
  tsmv->GetEndState(parent, child, full_state);
  bool val = si->checkMotion(parent, full_state);
  si->freeState(full_state);
  return val;
}

ompl::base::Cost SteerCostNominal(const ompl::base::State *parent, ompl::base::State *child, const ompl::base::OptimizationObjectivePtr &opt) {
  return opt->motionCost(parent, child);
}

ompl::base::Cost SteerCostTaskSpace(const ompl::base::State *parent, ompl::base::State *child, const ompl::base::OptimizationObjectivePtr &opt, const ompl::base::SpaceInformationPtr &si, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  ompl::base::State *full_state = si->allocState();
  tsmv->GetEndState(parent, child, full_state);
  ompl::base::Cost motion_cost = opt->motionCost(parent, full_state);
  si->copyState(child, full_state);
  si->freeState(full_state);
  return motion_cost;
}



void AddMotionToTreeNominal(Motion* motion, Tree &tree) {
  tree->add(motion);
  motion->parent->children.push_back(motion);
}

void AddMotionToTreeTaskSpace(Motion* motion, Tree &tree, const ompl::base::SpaceInformationPtr &si, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  ompl::base::State *full_state = si->allocState();
  tsmv->GetEndState(motion->parent->state, motion->state, full_state);
  si->copyState(motion->state, full_state);
  si->freeState(full_state);
  tree->add(motion);
  motion->parent->children.push_back(motion);
}


double DistanceFunctionNominal(const Motion* a, const Motion* b, const ompl::base::SpaceInformationPtr &si) {
  return si->distance(a->state, b->state);
}

double DistanceFunctionTaskSpace(const Motion* a, const Motion* b, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  return tsmv->Distance(a->state, b->state);
}



Motion* GetNearestMotionNominal(Motion* rmotion, Tree &tree) {
  return tree->nearest(rmotion);
}

Motion* GetNearestMotionTaskSpace(Motion* rmotion, Tree &tree, const ompl::base::SpaceInformationPtr &si, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  DistanceFn actual_distance_fn = tree->getDistanceFunction();
  tree->setDistanceFunction(boost::bind(&DistanceFunctionTaskSpace, _1, _2, tsmv));
  Motion* nmotion = tree->nearest(rmotion);

  ompl::base::State *full_state = si->allocState();
  tsmv->GetEndState(nmotion->state, rmotion->state, full_state);
  si->copyState(rmotion->state, full_state);
  si->freeState(full_state);

  tree->setDistanceFunction(actual_distance_fn);
  return nmotion;
}


void GetNearMotionNominal(Motion* motion, Tree &tree, unsigned int k, std::vector<Motion*> &nbh) {
  tree->nearestK(motion, k, nbh);
}


void GetNearMotionTaskSpace(Motion* motion, Tree &tree, unsigned int k, std::vector<Motion*> &nbh, const ompl_state_spaces::TaskSpaceMotionValidatorPtr &tsmv) {
  DistanceFn actual_distance_fn = tree->getDistanceFunction();
  tree->setDistanceFunction(boost::bind(&DistanceFunctionTaskSpace, _1, _2, tsmv));
  tree->nearestK(motion, k, nbh);
  tree->setDistanceFunction(actual_distance_fn);
}


} // end namespace



RRTstar::RRTstar(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "RRTstar")
{
  specs_.approximateSolutions = true;
  specs_.optimizingPaths = true;

  goalBias_ = 0.05;
  maxDistance_ = 0.0;
  delayCC_ = true;
  lastGoalMotion_ = NULL;

  iterations_ = 0;
  collisionChecks_ = 0;
  bestCost_ = ompl::base::Cost(std::numeric_limits<double>::quiet_NaN());

  task_space_sampling_ = false;
  feasible_only_ = false;
  goal_s_ = NULL;

  gamma_multiplier_ = 1;

  Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
  Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
  Planner::declareParam<bool>("task_space_sampling", this, &RRTstar::set_task_space_sampling, &RRTstar::task_space_sampling, "0,1");


  addPlannerProgressProperty("iterations INTEGER",
                             boost::bind(&RRTstar::getIterationCount, this));
  addPlannerProgressProperty("collision checks INTEGER",
                             boost::bind(&RRTstar::getCollisionCheckCount, this));
  // ToDo:
  // Can't use getBestCost()
  //addPlannerProgressProperty("best cost REAL",
  //                           boost::bind(&RRTstar::getBestCost, this));
}

RRTstar::~RRTstar(void)
{
  freeMemory();
}

void RRTstar::setup(void)
{
  Planner::setup();
  ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!nn_)
  {
    //std::cout << "\n error: nn_ ptr is null \n" << std::endl;

    // ToDo
    /*
    /home/dbworth/catkin_workspaces/uav_ws/devel/include/ompl/tools/config/SelfConfig.h:94:42: note:   template argument deduction/substitution failed:
     /home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/src/task_space_rrtstar.cpp:214:96: note:   cannot convert ((task_space_rrtstar::RRTstar*)this)->task_space_rrtstar::RRTstar::<anonymous>.ompl::base::Planner::si_.boost::shared_ptr<T>::operator-><ompl::base::SpaceInformation>()->ompl::base::SpaceInformation::getStateSpace() (type const StateSpacePtr {aka const boost::shared_ptr<ompl::base::StateSpace>}) to type const ompl::base::Planner*
     nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    */
    //nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));

    nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<rrtstar::Motion*>(this));



  }

  nn_->setDistanceFunction(boost::bind(&DistanceFunctionNominal, _1, _2, si_));

  if (!sampler_)
    sampler_ = si_->allocValidStateSampler();

  // Setup optimization objective
  //
  // If no optimization objective was specified, then default to
  // optimizing path length as computed by the distance() function
  // in the state space.
  if (pdef_->hasOptimizationObjective())
    opt_ = pdef_->getOptimizationObjective();
  else
  {
    OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
    opt_.reset(new ompl::base::PathLengthOptimizationObjective(si_));
  }

  // Default
  Sample = boost::bind(&RRTstar::SampleSpaceGoal, this, _1);
  ExtendSteerCost = boost::bind(&SteerCostNominal, _1, _2, opt_);
  AddMotionToTree = boost::bind(&AddMotionToTreeNominal, _1, _2);
  GetNearestMotion = boost::bind(&GetNearestMotionNominal, _1, _2);
  GetNearMotion = boost::bind(&GetNearMotionNominal, _1, _2, _3, _4);
  CheckGoal = boost::bind(&RRTstar::CheckGoalPassively, this, _1, _2, _3);
  CheckExtendMotion = boost::bind(&CheckMotionNominal, _1, _2, si_);
  if (task_space_sampling_) {
    Sample = boost::bind(&RRTstar::SampleSpace, this, _1);
    ExtendSteerCost = boost::bind(&SteerCostTaskSpace, _1, _2, opt_, si_, task_space_mv_);
    AddMotionToTree = boost::bind(&AddMotionToTreeTaskSpace, _1, _2,  si_, task_space_mv_);
    GetNearestMotion = boost::bind(&GetNearestMotionTaskSpace, _1, _2, si_, task_space_mv_);
    GetNearMotion = boost::bind(&GetNearMotionTaskSpace, _1, _2, _3, _4, task_space_mv_);
    CheckGoal = boost::bind(&RRTstar::CheckGoalByConnecting, this, _1, _2, _3);
    CheckExtendMotion = boost::bind(&CheckMotionTaskSpace, _1, _2, si_, task_space_mv_);
  }



}

void RRTstar::clear(void)
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_)
    nn_->clear();

  lastGoalMotion_ = NULL;
  goalMotions_.clear();
  motions_not_in_tree_.clear();

  iterations_ = 0;
  collisionChecks_ = 0;
  bestCost_ = ompl::base::Cost(std::numeric_limits<double>::quiet_NaN());

}

ompl::base::PlannerStatus RRTstar::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  goal_   = pdef_->getGoal().get();
  goal_s_ = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal_);

  bool symDist = si_->getStateSpace()->hasSymmetricDistance();
  bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
  bool symCost = opt_->isSymmetric();

  while (const ompl::base::State *st = pis_.nextStart())
  {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->cost = opt_->identityCost();
    nn_->add(motion);
  }

  if (nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
    sampler_ = si_->allocValidStateSampler();

  OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

  Motion *solution       = lastGoalMotion_;

  // \TODO Make this variable unnecessary, or at least have it
  // persist across solve runs
  ompl::base::Cost bestCost    = opt_->infiniteCost();

  Motion *approximation  = NULL;
  double approximatedist = std::numeric_limits<double>::infinity();
  bool sufficientlyShort = false;

  Motion *rmotion        = new Motion(si_);
  ompl::base::State *rstate    = rmotion->state;
  ompl::base::State *xstate    = si_->allocState();

  // e+e/d.  K-nearest RRT*
  double k_rrg           = gamma_multiplier_*(boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension()));

  std::vector<Motion*>       nbh;

  std::vector<ompl::base::Cost>    costs;
  std::vector<ompl::base::Cost>    incCosts;
  std::vector<std::size_t>   sortedCostIndices;

  std::vector<int>           valid;
  unsigned int               rewireTest = 0;
  unsigned int               statesGenerated = 0;

  if (solution)
  {
    // ToDo:
    // .v doesnt exist
    //OMPL_INFORM("%s: Starting with existing solution of cost %.5f", getName().c_str(), solution->cost.v);
  }
  OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size()+1))));


  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);



  while (ptc == false)
  {
    iterations_++;

    Sample(rstate);

    // Check if the motion between the nearest state and the state to add is valid
    ++collisionChecks_;

    // find closest state in the tree
    Motion *nmotion = GetNearestMotion(rmotion, nn_);

    ompl::base::State *dstate = rstate;

    // find state to add to the tree
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_)
    {
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
      dstate = xstate;
    }


    if (si_->checkMotion(nmotion->state, dstate))
    {
      // create a motion
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;
      motion->incCost = ExtendSteerCost(nmotion->state, motion->state);//opt_->motionCost(nmotion->state, motion->state);
      motion->cost =  opt_->combineCosts(nmotion->cost, motion->incCost);
      // This sounds crazy but for asymmetric distance functions this is necessary
      // For this case, it has to be FROM every other point TO our new point
      // NOTE THE ORDER OF THE boost::bind PARAMETERS
      if (!symDist)
        nn_->setDistanceFunction(boost::bind(&DistanceFunctionNominal, _1, _2, si_));

      // Find nearby neighbors of the new motion - k-nearest RRT*
      unsigned int k = std::ceil(k_rrg * log((double)(nn_->size()+1)));
      GetNearMotion(motion, nn_, k, nbh);

      rewireTest += nbh.size();
      statesGenerated++;

      // cache for distance computations
      //
      // Our cost caches only increase in size, so they're only
      // resized if they can't fit the current neighborhood
      if (costs.size() < nbh.size())
      {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
      }

      // cache for motion validity (only useful in a symmetric space)
      //
      // Our validity caches only increase in size, so they're
      // only resized if they can't fit the current neighborhood
      if (symDist && symInterp)
      {
        if (valid.size() < nbh.size())
          valid.resize(nbh.size());
        std::fill(valid.begin(), valid.begin()+nbh.size(), 0);
      }

      // Finding the nearest neighbor to connect to
      // By default, neighborhood states are sorted by cost, and collision checking
      // is performed in increasing order of cost
      if (delayCC_)
      {
        // calculate all costs and distances
        for (std::size_t i = 0 ; i < nbh.size(); ++i)
        {
          incCosts[i] = ExtendSteerCost(nbh[i]->state, motion->state);//opt_->motionCost(nbh[i]->state, motion->state);
          costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i)
          sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin()+nbh.size(),
                  compareFn);

        // collision check until a valid motion is found
        for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
            i != sortedCostIndices.begin()+nbh.size();
            ++i)
        {
          if (nbh[*i] != nmotion)
            ++collisionChecks_;
          if (nbh[*i] == nmotion || CheckExtendMotion(nbh[*i]->state, motion->state))
          {
            motion->incCost = incCosts[*i];
            motion->cost = costs[*i];
            motion->parent = nbh[*i];
            if (symDist && symInterp)
              valid[*i] = 1;
            break;
          }
          else if (symDist && symInterp)
            valid[*i] = -1;
        }
      }
      else // if not delayCC
      {
        motion->incCost = ExtendSteerCost(nmotion->state, motion->state);//opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for (std::size_t i = 0 ; i < nbh.size(); ++i)
        {
          if (nbh[i] != nmotion)
          {
            incCosts[i] = ExtendSteerCost(nbh[i]->state, motion->state);//opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
            if (opt_->isCostBetterThan(costs[i], motion->cost))
            {
              ++collisionChecks_;
              if (CheckExtendMotion(nbh[i]->state, motion->state))
              {
                motion->incCost = incCosts[i];
                motion->cost = costs[i];
                motion->parent = nbh[i];
                if (symDist && symInterp)
                  valid[i] = 1;
              }
              else if (symDist && symInterp)
                valid[i] = -1;
            }
          }
          else
          {
            incCosts[i] = motion->incCost;
            costs[i] = motion->cost;
            if (symDist && symInterp)
              valid[i] = 1;
          }
        }
      }

      // add motion to the tree
      AddMotionToTree(motion, nn_);

      bool checkForSolution = false;
      // rewire tree if needed
      //
      // This sounds crazy but for asymmetric distance functions this is necessary
      // For this case, it has to be FROM our new point TO each other point
      // NOTE THE ORDER OF THE boost::bind PARAMETERS
      if (!symDist)
      {
        nn_->setDistanceFunction(boost::bind(&DistanceFunctionNominal, _2, _1, si_));
        nn_->nearestK(motion, k, nbh);
        rewireTest += nbh.size();
      }

      for (std::size_t i = 0; i < nbh.size(); ++i)
      {
        if (nbh[i] != motion->parent)
        {
          ompl::base::Cost nbhIncCost;
          if (symDist && symCost)
            nbhIncCost = incCosts[i];
          else
            nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
          ompl::base::Cost backupCost = motion->cost;
          ompl::base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
          ompl::base::Cost oldnbhCost =  nbh[i]->cost;
          //double motion_x = motion->state->as<ompl::ompl::base::DubinsStateSpace::StateType>()->getX();
          //double motion_y = motion->state->as<ompl::ompl::base::DubinsStateSpace::StateType>()->getY();
          ompl::base::Cost oldnbhParentCost;
          if (nbh[i]->parent)
            oldnbhParentCost = nbh[i]->parent->cost;
          ompl::base::Cost child1Cost, child2Cost;
          if (nbh[i]->children.size()>0)
            child1Cost = nbh[i]->children[0]->cost;

          if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
          {
            bool motionValid;
            if (symDist && symInterp)
            {
              if (valid[i] == 0)
              {
                ++collisionChecks_;
                motionValid = si_->checkMotion(motion->state, nbh[i]->state);
              }
              else
                motionValid = (valid[i] == 1);
            }
            else
            {
              ++collisionChecks_;
              motionValid = si_->checkMotion(motion->state, nbh[i]->state);
            }
            if (motionValid)
            {
              // Remove this node from its parent list
              removeFromParent (nbh[i]);

              // Add this node to the new parent
              nbh[i]->parent = motion;
              nbh[i]->incCost = nbhIncCost;
              nbh[i]->cost = nbhNewCost;
              nbh[i]->parent->children.push_back(nbh[i]);

              // Update the costs of the node's children
              updateChildCosts(nbh[i]);

              checkForSolution = true;
            }
          }
        }
      }


      double distanceFromGoal;
      CheckGoal(motion, distanceFromGoal, checkForSolution);


      // Checking for solution or iterative improvement
      if (checkForSolution)
      {
        for (size_t i = 0; i < goalMotions_.size(); ++i)
        {
          if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
          {
            bestCost = goalMotions_[i]->cost;
            bestCost_ = bestCost;
          }

          sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
          if (sufficientlyShort)
          {
            solution = goalMotions_[i];
            break;
          }
          else if (!solution ||
              opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
            solution = goalMotions_[i];
        }
      }

      // Checking for approximate solution (closest state found to the goal)
      if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
      {
        approximation = motion;
        approximatedist = distanceFromGoal;
      }
    }

    // terminate if a sufficient solution is found
    if (solution && sufficientlyShort)
      break;

    if (feasible_only_ && solution)
      break;
  }

  bool approximate = (solution == 0);
  bool addedSolution = false;
  if (approximate)
    solution = approximation;
  else
    lastGoalMotion_ = solution;

  if (solution != 0)
  {
    // construct the solution path
    std::vector<Motion*> mpath;
    while (solution != 0)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    // set the solution path
    namespace og = ompl::geometric;
    og::PathGeometric *geoPath = new og::PathGeometric(si_);
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
      geoPath->append(mpath[i]->state);

    ompl::base::PathPtr path(geoPath);
    // Add the solution path, whether it is approximate (not reaching the goal), and the
    // distance from the end of the path to the goal (-1 if satisfying the goal).


    // ToDo
    // PlannerSolution with 3 args doesnt exist
    //ompl::base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0);
    ompl::base::PlannerSolution psol(path);


    // Does the solution satisfy the optimization objective?
    psol.optimized_ = sufficientlyShort;

    pdef_->addSolutionPath (psol);

    addedSolution = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

  return ompl::base::PlannerStatus(addedSolution, approximate);
}

void RRTstar::removeFromParent(Motion *m)
{
  std::vector<Motion*>::iterator it = m->parent->children.begin ();
  while (it != m->parent->children.end ())
  {
    if (*it == m)
    {
      it = m->parent->children.erase(it);
      it = m->parent->children.end ();
    }
    else
      ++it;
  }
}

void RRTstar::updateChildCosts(Motion *m)
{
  for (std::size_t i = 0; i < m->children.size(); ++i)
  {
    ompl::base::Cost tempc1 = m->children[i]->cost;
    m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
    ompl::base::Cost tempc2 = m->children[i]->cost;
    updateChildCosts(m->children[i]);
  }
}

void RRTstar::freeMemory(void)
{
  if (nn_)
  {
    std::vector<Motion*> motions;
    nn_->list(motions);
    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }

    for (std::size_t i = 0 ; i < motions_not_in_tree_.size() ; ++i)
    {
      if (motions_not_in_tree_[i]->state)
        si_->freeState(motions_not_in_tree_[i]->state);
      delete motions_not_in_tree_[i];
    }
  }
}

void RRTstar::getPlannerData(ompl::base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (nn_)
    nn_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

  for (std::size_t i = 0 ; i < motions.size() ; ++i)
  {
    if (motions[i]->parent == NULL)
      data.addStartVertex(ompl::base::PlannerDataVertex(motions[i]->state));
    else
      data.addEdge(ompl::base::PlannerDataVertex(motions[i]->parent->state),
                   ompl::base::PlannerDataVertex(motions[i]->state));
  }
  data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
  data.properties["collision_checks INTEGER"] =
      boost::lexical_cast<std::string>(collisionChecks_);
}

std::string RRTstar::getIterationCount(void) const
{
  return boost::lexical_cast<std::string>(iterations_);
}
std::string RRTstar::getCollisionCheckCount(void) const
{
  return boost::lexical_cast<std::string>(collisionChecks_);
}
// ToDo:
// .v doesnt exist
//std::string RRTstar::getBestCost(void) const
//{
//  return boost::lexical_cast<std::string>(bestCost_.v);
//}





void RRTstar::SampleSpaceGoal(ompl::base::State *rstate) {
  // sample random state (with goal biasing)
  // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
  if (goal_s_ && goalMotions_.size() < goal_s_->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s_->canSample() && !task_space_sampling_)
    goal_s_->sampleGoal(rstate);
  else
    sampler_->sample(rstate);
}

void RRTstar::SampleSpace(ompl::base::State *rstate) {
  sampler_->sample(rstate);
}




void RRTstar::CheckGoalPassively(Motion *motion, double &distanceFromGoal, bool& checkForSolution) {

  // Add the new motion to the goalMotion_ list, if it satisfies the goal
  if (goal_->isSatisfied(motion->state, &distanceFromGoal))
  {
    goalMotions_.push_back(motion);
    checkForSolution = true;
  }

}


void RRTstar::CheckGoalByConnecting(Motion *motion, double &distanceFromGoal, bool& checkForSolution) {

  if (goalMotions_.size() < goal_s_->maxSampleCount()) {
    Motion *goal_motion = new Motion(si_);
    goal_s_->sampleGoal(goal_motion->state);
    if (si_->checkMotion(motion->state, goal_motion->state)) {
      ompl::base::Cost cost = opt_->motionCost(motion->state, goal_motion->state);
      goal_motion->parent = motion;
      goal_motion->incCost = cost;
      goal_motion->cost =  opt_->combineCosts(motion->cost, goal_motion->incCost);
      goal_motion->parent->children.push_back(goal_motion);
      goalMotions_.push_back(goal_motion);
      motions_not_in_tree_.push_back(goal_motion);
      checkForSolution = true;
      distanceFromGoal = 0;
      return;
    }
    si_->freeState(goal_motion->state);
    delete goal_motion;
  } else {
    Motion *goal_motion = goalMotions_[rand() % goalMotions_.size()];
    if (si_->checkMotion(motion->state, goal_motion->state)) {
      ompl::base::Cost cost = opt_->motionCost(motion->state, goal_motion->state);
      ompl::base::Cost cost_to_come = opt_->combineCosts(motion->cost, cost);
      if (opt_->isCostBetterThan(cost_to_come, goal_motion->cost)) {
        goal_motion->parent = motion;
        goal_motion->incCost = cost;
        goal_motion->cost =  cost_to_come;
        goal_motion->parent->children.push_back(goal_motion);
        checkForSolution = true;
        distanceFromGoal = 0;
        return;
      }
    }
  }

  distanceFromGoal = std::numeric_limits<double>::max();
}


} // namespace namespace task_space_rrt

