/** @brief Implementation of the iMESA algorithm interface.
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */

#include "imesa/imesa.h"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace imesa {
/**
 * #### ##    ## ######## ######## ########  ########    ###     ######  ########
 *  ##  ###   ##    ##    ##       ##     ## ##         ## ##   ##    ## ##
 *  ##  ####  ##    ##    ##       ##     ## ##        ##   ##  ##       ##
 *  ##  ## ## ##    ##    ######   ########  ######   ##     ## ##       ######
 *  ##  ##  ####    ##    ##       ##   ##   ##       ######### ##       ##
 *  ##  ##   ###    ##    ##       ##    ##  ##       ##     ## ##    ## ##
 * #### ##    ##    ##    ######## ##     ## ##       ##     ##  ######  ########
 */

/*********************************************************************************************************************/
gtsam::ISAM2Result IMESAAgent::update(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_theta) {
  // Determine if we observed new variables of other robots
  // We identify shared variables by the character of the variable key as it is used to signify the owner robot.
  std::vector<std::pair<RobotId, gtsam::Key>> new_shared_variables;
  for (const auto& key : new_factors.keys()) {
    gtsam::Symbol sym(key);
    if (sym.chr() != robot_id_ && !current_estimate_.exists(key)) {
      if (sym.chr() == GLOBAL_ID) {
        observed_global_variables_.insert(key);
      } else {
        new_shared_variables.push_back(std::make_pair(sym.chr(), key));
      }
    }
  }
  // Create new biased priors for all newly observed other robot variables
  gtsam::NonlinearFactorGraph new_biased_priors(addNewSharedVariables(new_shared_variables, new_theta, true));

  // Augment the new factors with new biased priors and any biased priors for variables declared by other robots
  gtsam::NonlinearFactorGraph augmented_new_factors(new_factors);
  augmented_new_factors.push_back(new_biased_priors);
  augmented_new_factors.push_back(new_other_robot_declared_biased_priors_);
  new_other_robot_declared_biased_priors_ = gtsam::NonlinearFactorGraph();

  // Do the update
  gtsam::ISAM2UpdateParams update_params;
  update_params.force_relinearize = true;
  update_params.extraReelimKeys = reelim_keys_;
  gtsam::ISAM2Result result = smoother_.update(augmented_new_factors, new_theta, update_params);
  current_estimate_ = smoother_.calculateEstimate();
  reelim_keys_.clear();
  return result;
}

/*********************************************************************************************************************/
DeclaredVariables IMESAAgent::declareNewSharedVariables(const RobotId& other_robot) {
  DeclaredVariables result;
  result.new_shared_variables = undeclared_robot_shared_vars_[other_robot];
  result.observed_global_variables = observed_global_variables_;

  undeclared_robot_shared_vars_[other_robot] = gtsam::KeySet();
  return result;
}

/*********************************************************************************************************************/
void IMESAAgent::receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) {
  std::vector<std::pair<RobotId, gtsam::Key>> new_shared_variables;
  for (const auto& key : declared_variables.new_shared_variables) {
    new_shared_variables.push_back(std::make_pair(other_robot, key));
  }

  for (const auto& key : declared_variables.observed_global_variables) {
    // If we have observed the key, but it is not marked as shared with the other_robot add it
    if (observed_global_variables_.count(key) != 0 && robot_shared_vars_[other_robot].count(key) == 0) {
      new_shared_variables.push_back(std::make_pair(other_robot, key));
    }
  }

  // Update book keeping and create biased priors, aggregate biased priors to be added on next update
  new_other_robot_declared_biased_priors_.push_back(
      addNewSharedVariables(new_shared_variables, current_estimate_, false));
}

/*********************************************************************************************************************/
CommunicationData::shared_ptr IMESAAgent::sendCommunication(const RobotId& other_robot) {
  IMESACommunicationData::shared_ptr cd_ptr = std::make_shared<IMESACommunicationData>();
  for (const gtsam::Key& key : robot_shared_vars_[other_robot]) {
    // Add the local solution to the shared variable
    cd_ptr->shared_var_solution.insert(key, current_estimate_.at(key));

    // Mark the variable if we need to initialize it from the other robot
    if (shared_initialization_variables_.count(key) && shared_initialization_variables_[key] == other_robot) {
      cd_ptr->vars_requested_for_init.insert(key);
    }
  }
  return cd_ptr;
}

/*********************************************************************************************************************/
void IMESAAgent::receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) {
  std::shared_ptr<IMESACommunicationData> imesa_comm_data =
      std::dynamic_pointer_cast<IMESACommunicationData>(comm_data);
  // Update shared estimate, dual, and penalty terms for all shared variables
  for (const gtsam::Key& key : robot_shared_vars_[other_robot]) {
    biased_priors::BiasedPriorInfo::shared_ptr bpi = biased_prior_values_[other_robot][key];

    // Update our value for the variable if marked for shared initialization
    if (shared_initialization_variables_.count(key) && shared_initialization_variables_[key] == other_robot) {
      current_estimate_.update(key, imesa_comm_data->shared_var_solution.at(key));
      shared_initialization_variables_.erase(key);
    }

    // If the variable is marked for initialization the other robot will use our estimate
    boost::shared_ptr<gtsam::Value> other_val = imesa_comm_data->vars_requested_for_init.count(key)
                                                    ? current_estimate_.at(key).clone()
                                                    : imesa_comm_data->shared_var_solution.at(key).clone();

    // Compute the new shared estimate and dual variable
    boost::shared_ptr<gtsam::Value> new_shared_estimate = bpi->interpolate(current_estimate_.at(key), *other_val);
    gtsam::Vector new_dual_variable =
        bpi->dual + bpi->penalty * bpi->constraint_func(current_estimate_.at(key), *new_shared_estimate, boost::none);

    // Compute the change in the shared estimate and dual variable relative to the current linearization point
    double estimate_delta = new_shared_estimate->localCoordinates_(*bpi->shared_estimate_lin_point).norm();
    double dual_delta = (new_dual_variable - bpi->dual_lin_point).norm();

    // If new, or there is sufficient change, mark for reelim and update linearization points
    if ((estimate_delta + dual_delta) >= params_.shared_var_wildfire_threshold ||
        bpi->penalty == params_.initial_penalty_param) {
      reelim_keys_.push_back(key);
      bpi->shared_estimate_lin_point = new_shared_estimate->clone();
      bpi->dual_lin_point = gtsam::Vector(new_dual_variable);
    }

    // Update shared and dual estimates
    bpi->shared_estimate = new_shared_estimate;
    bpi->dual = new_dual_variable;
    if (bpi->penalty == params_.initial_penalty_param) bpi->penalty = params_.penalty_param;
  }
}

/*********************************************************************************************************************/
std::string IMESAAgent::name() const {
  // Unweighted Variants
  if (!params_.biased_prior_noise_model_sigmas.has_value()) {
    if (params_.pose_biased_prior_type == biased_priors::BiasedPriorType::GEODESIC) {
      return UNWEIGHTED_METHOD_NAME;
    } else {
      return SPLIT_UNWEIGHTED_METHOD_NAME;
    }
  } 
  // Standard Variants
  else {
    if (params_.pose_biased_prior_type == biased_priors::BiasedPriorType::GEODESIC) {
      return METHOD_NAME; // iMESA as presented in [2]
    } else {
      return SPLIT_METHOD_NAME;
    }
  }
}

/**
 * ##     ## ######## ##       ########  ######## ########   ######
 * ##     ## ##       ##       ##     ## ##       ##     ## ##    ##
 * ##     ## ##       ##       ##     ## ##       ##     ## ##
 * ######### ######   ##       ########  ######   ########   ######
 * ##     ## ##       ##       ##        ##       ##   ##         ##
 * ##     ## ##       ##       ##        ##       ##    ##  ##    ##
 * ##     ## ######## ######## ##        ######## ##     ##  ######
 */

/*********************************************************************************************************************/
gtsam::NonlinearFactorGraph IMESAAgent::addNewSharedVariables(
    const std::vector<std::pair<RobotId, gtsam::Key>>& new_shared_vars, const gtsam::Values& new_theta,
    bool undeclared) {
  gtsam::NonlinearFactorGraph new_biased_priors;

  // Iterate over all the new shared variables for each update bookkeeping
  for (const auto& rkp : new_shared_vars) {
    const RobotId rid = rkp.first;
    const gtsam::Key key = rkp.second;

    // Construct containers if first instance of a variable or a robot
    if (robot_shared_vars_.count(rid) == 0) robot_shared_vars_[rid] = gtsam::KeySet();
    if (shared_var_robots_.count(key) == 0) shared_var_robots_[key] = std::set<RobotId>();

    // Record this shared variable
    robot_shared_vars_[rid].insert(key);
    shared_var_robots_[key].insert(rid);

    // Mark for initialization if we dont already have the variable initialized
    if (!current_estimate_.exists(key) && gtsam::Symbol(key).chr() != robot_id_) {
      shared_initialization_variables_[key] = rid;
    }

    // If these new shared variables are undeclared we need to mark them for communication to other robots
    if (undeclared) {
      if (undeclared_robot_shared_vars_.count(rid) == 0) {
        undeclared_robot_shared_vars_[rid] = gtsam::KeySet();
      }
      undeclared_robot_shared_vars_[rid].insert(key);
    }

    // Construct the biased prior
    biased_priors::BiasedPriorInfo::shared_ptr bpi = std::make_shared<biased_priors::BiasedPriorInfo>(
        params_.pose_biased_prior_type, new_theta.at(key), params_.initial_penalty_param);

    // Book keep the new biased prior and its corresponding values
    new_biased_priors.push_back(
        biased_priors::factory(params_.pose_biased_prior_type, key, bpi, params_.biased_prior_noise_model_sigmas));
    biased_prior_values_[rid][key] = bpi;
  }
  return new_biased_priors;
}

}  // namespace imesa