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
 * rrtstar.h
 *
 *  Created on: Aug 23, 2014
 *      Author: Sanjiban Choudhury
 */

#ifndef SAMPLING_PLANNERS_INCLUDE_SAMPLING_PLANNERS_RRTSTAR_H_
#define SAMPLING_PLANNERS_INCLUDE_SAMPLING_PLANNERS_RRTSTAR_H_

#include <limits>
#include <vector>
#include <utility>

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalSampleableRegion.h>


//#include "ompl_state_spaces/motion_validator/task_space_motion_validator.h"
 #include <ompl_state_spaces/task_space_motion_validator.h>

//namespace ca {


namespace task_space_rrtstar
{

namespace rrtstar // for OMPL
{

/** \brief Representation of a motion */
class Motion
{
public:
    /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
    Motion(const ompl::base::SpaceInformationPtr &si) :
        state(si->allocState()),
        parent(NULL)
    {
    }

    ~Motion(void)
    {
    }

    /** \brief The state contained by the motion */
    ompl::base::State       *state;

    /** \brief The parent motion in the exploration tree */
    Motion            *parent;

    /** \brief The cost up to this motion */
    ompl::base::Cost        cost;

    /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
    ompl::base::Cost        incCost;

    /** \brief The set of motions descending from the current motion */
    std::vector<Motion*> children;
};


// For sorting a list of costs and getting only their sorted indices
struct CostIndexCompare
{
    CostIndexCompare(const std::vector<ompl::base::Cost>& costs,
                     const ompl::base::OptimizationObjective& opt) :
        costs_(costs), opt_(opt)
    {}
    bool operator()(unsigned i, unsigned j)
    {
        return opt_.isCostBetterThan(costs_[i],costs_[j]);
    }
    const std::vector<ompl::base::Cost>& costs_;
    const ompl::base::OptimizationObjective& opt_;
};

typedef boost::shared_ptr< ompl::NearestNeighbors<rrtstar::Motion*> > Tree;
typedef ompl::NearestNeighbors<rrtstar::Motion*>::DistanceFunction DistanceFn;



} // namespace rrtstar


/** \brief
 * Starting with an (almost) exact copy of ompl rrtstar. This is to allow more options to configure it. To be documented soon
 */

class RRTstar : public ompl::base::Planner
{
 public:

     RRTstar(const ompl::base::SpaceInformationPtr &si);

     virtual ~RRTstar(void);

     virtual void getPlannerData(ompl::base::PlannerData &data) const;

     virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

     virtual void clear(void);

     /** \brief Set the goal bias

         In the process of randomly selecting states in
         the state space to attempt to go towards, the
         algorithm may in fact choose the actual goal state, if
         it knows it, with some probability. This probability
         is a real number between 0.0 and 1.0; its value should
         usually be around 0.05 and should not be too large. It
         is probably a good idea to use the default value. */
     void setGoalBias(double goalBias)
     {
         goalBias_ = goalBias;
     }

     /** \brief Get the goal bias the planner is using */
     double getGoalBias(void) const
     {
         return goalBias_;
     }

     /** \brief Set the range the planner is supposed to use.

         This parameter greatly influences the runtime of the
         algorithm. It represents the maximum length of a
         motion to be added in the tree of motions. */
     void setRange(double distance)
     {
         maxDistance_ = distance;
     }

     /** \brief Get the range the planner is using */
     double getRange(void) const
     {
         return maxDistance_;
     }

     /** \brief Set a different nearest neighbors datastructure */
     template<template<typename T> class NN>
     void setNearestNeighbors(void)
     {
         nn_.reset(new NN<rrtstar::Motion*>());
         // nn_.reset(new NN<Motion*>());
     }

     /** \brief Option that delays collision checking procedures.
         When it is enabled, all neighbors are sorted by cost. The
         planner then goes through this list, starting with the lowest
         cost, checking for collisions in order to find a parent. The planner
         stops iterating through the list when a collision free parent is found.
         This prevents the planner from collsion checking each neighbor, reducing
         computation time in scenarios where collision checking procedures are expensive.*/
     void setDelayCC(bool delayCC)
     {
         delayCC_ = delayCC;
     }

     /** \brief Get the state of the delayed collision checking option */
     bool getDelayCC(void) const
     {
         return delayCC_;
     }

     void set_task_space_sampling(bool task_space_sampling) {
       task_space_sampling_ = task_space_sampling;
     }

     bool task_space_sampling() {
       return task_space_sampling_;
     }

     void set_feasible_only(bool feasible_only) {
       feasible_only_ = feasible_only;
     }

     bool feasible_only() {
       return feasible_only_;
     }

     void set_task_space_mv(const ompl_state_spaces::TaskSpaceMotionValidatorPtr &task_space_mv) {
       task_space_mv_ = task_space_mv;
     }

     void set_gamma_multipler(double gamma_multiplier) {
       gamma_multiplier_ = gamma_multiplier;
     }

     // Added by ca
     /*
     virtual void setLocalSeed(boost::uint32_t localSeed)
     {
       sampler_->setLocalSeed(localSeed);
     }

     virtual boost::uint32_t getLocalSeed() const
     {
       return sampler_->getLocalSeed();
     }
     */

     virtual void setup(void);

     ///////////////////////////////////////
     // Planner progress property functions
     std::string getIterationCount(void) const;

     std::string getCollisionCheckCount(void) const;

     std::string getBestCost(void) const;
     ///////////////////////////////////////

 protected:

     typedef boost::function<void(ompl::base::State*)> SampleFn;
     typedef boost::function<ompl::base::Cost(const ompl::base::State*, ompl::base::State*)> SteerCostFn;
     typedef boost::function<void(rrtstar::Motion*, rrtstar::Tree&)> AddMotionToTreeFn;
     typedef boost::function<rrtstar::Motion*(rrtstar::Motion*, rrtstar::Tree&)> GetNearestMotionFn;
     typedef boost::function<void(rrtstar::Motion*, rrtstar::Tree&, unsigned int, std::vector<rrtstar::Motion*> &)> GetNearMotionFn;
     typedef boost::function<void(rrtstar::Motion*, double&, bool&)> CheckGoalFn;
     typedef boost::function<bool(const ompl::base::State*, const ompl::base::State*)> CheckMotionFn;


     SampleFn Sample;
     SteerCostFn ExtendSteerCost;
     AddMotionToTreeFn AddMotionToTree;
     GetNearestMotionFn GetNearestMotion;
     GetNearMotionFn GetNearMotion;
     CheckGoalFn CheckGoal;
     CheckMotionFn CheckExtendMotion;

     /** \brief Free the memory allocated by this planner */
     void freeMemory(void);

     /** \brief Removes the given motion from the parent's child list */
     void removeFromParent(rrtstar::Motion *m);

     /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
     void updateChildCosts(rrtstar::Motion *m);

     /** \brief State sampler */
     ompl::base::ValidStateSamplerPtr                         sampler_;

     /** \brief A nearest-neighbors datastructure containing the tree of motions */
     rrtstar::Tree nn_;
     // boost::shared_ptr< NearestNeighbors<Motion*> > nn_;
     // typedef boost::shared_ptr< ompl::NearestNeighbors<rrtstar::Motion*> > Tree;

     /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
     double                                         goalBias_;

     /** \brief The maximum length of a motion to be added to a tree */
     double                                         maxDistance_;

     /** \brief The random number generator */
     ompl::RNG                                            rng_;

     /** \brief Option to delay and reduce collision checking within iterations */
     bool                                           delayCC_;

     /** knn multipler **/
     double gamma_multiplier_;

     /** \brief Objective we're optimizing */
     ompl::base::OptimizationObjectivePtr opt_;

     /** \brief The most recent goal motion.  Used for PlannerData computation */
     rrtstar::Motion                                         *lastGoalMotion_;

     /** \brief A list of states in the tree that satisfy the goal condition */
     std::vector<rrtstar::Motion*>                           goalMotions_;
     std::vector<rrtstar::Motion*>                           motions_not_in_tree_;

     //////////////////////////////
     // Planner progress properties

     /** \brief Number of iterations the algorithm performed */
     unsigned int                                   iterations_;

     /** \brief Number of collisions checks performed by the algorithm */
     unsigned int                                   collisionChecks_;

     /** \brief Best cost found so far by algorithm */
     ompl::base::Cost                                     bestCost_;

     /** \brief Task space sampling */
     bool task_space_sampling_;

     bool feasible_only_;

     ompl_state_spaces::TaskSpaceMotionValidatorPtr task_space_mv_;

     ompl::base::Goal                  *goal_;
     ompl::base::GoalSampleableRegion  *goal_s_;


     void SampleSpaceGoal(ompl::base::State *rstate);
     void SampleSpace(ompl::base::State *rstate);


     void CheckGoalPassively(rrtstar::Motion *motion, double &distanceFromGoal, bool& checkForSolution);
     void CheckGoalByConnecting(rrtstar::Motion *motion, double &distanceFromGoal, bool& checkForSolution);

};

} // namespace task_space_rrtstar



#endif  // SAMPLING_PLANNERS_INCLUDE_SAMPLING_PLANNERS_RRTSTAR_H_ 
