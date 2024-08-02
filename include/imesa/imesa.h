#pragma once
/** @brief Interface of the iMESA algorithm via an individual agent interface.
 * This solver is to be run on-board individual robots in the multi-robot team to
 * collaboratively solve the collaborative simultaneous localization and mapping problem.
 *
 * This algorithm is described in detail in
 *
 * [2]  D. McGann and M. Kaess, "iMESA: Incremental Distributed Optimization for Collaborative
 *      Simultaneous Localization and Mapping", Proc. Robotic Science and Systems (RSS), 2024
 *
 *
 * Note: The communication interface REQUIRES the following order of operations to complete
 * the two state communication process outlined in [2] - Alg. 3
 *  1. declareNewSharedVariables / receiveNewSharedVariables
 *  2. sendCommunication / receiveCommunication
 * If the two stage communication process is not completed in whole, the algorithm invariants will break
 * and the algorithm can exhibit bad performance.
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>

#include "imesa/biased_prior.h"
#include "imesa/incremental_sam_agent.h"

using namespace incremental_sam_agent;

namespace imesa {

/// @brief The data communicated between two iMESA agents
struct IMESACommunicationData : public CommunicationData {
  typedef std::shared_ptr<IMESACommunicationData> shared_ptr;
  /// @brief The current solution to all variables shared between robots
  gtsam::Values shared_var_solution;
  /// @brief The variables that this robot needs to initialize from the other robot
  gtsam::KeySet vars_requested_for_init;
};

/// @brief Configuration parameters for the iMESA algo, all agents MUST use the same params
struct IMESAAgentParams {
  /// @brief The type of biased prior to use for pose variables
  /// Note: Using SPLIT was partially explored but not rigorously evaluated or discussed in [2]. Use with caution.
  biased_priors::BiasedPriorType pose_biased_prior_type{biased_priors::BiasedPriorType::GEODESIC};
  /// @brief The initial penalty parameter used before the shared variable is initialized (should be very small)
  double initial_penalty_param{0.0001};
  /// @brief The constant penalty parameter to use for biased priors of initialzed shared variables
  double penalty_param{1};
  /// @brief The threshold for a change in shared estimate or dual variable value required to force reelimination
  /// Note: Setting  > 0.0 was partially explored, but not rigorously evaluated or discussed in [2]. Use with caution.
  double shared_var_wildfire_threshold{0.0};
  /// @brief The sigmas to use for biased prior noise models on poses (rotation_sigma, translation sigma)
  std::optional<std::pair<double, double>> biased_prior_noise_model_sigmas{std::nullopt};
};

/// @brief Implementation of the iMESA Algorithm
class IMESAAgent : public IncrementalSAMAgent {
  /** STATIC MEMBERS **/
 public:
  inline static const std::string METHOD_NAME = "imesa";
  inline static const std::string SPLIT_METHOD_NAME = "imesa_split";
  inline static const std::string UNWEIGHTED_METHOD_NAME = "imesa_unweighted";
  inline static const std::string SPLIT_UNWEIGHTED_METHOD_NAME = "imesa_split_unweighted";

  /** FIELDS **/
 protected:
  /// @brief The hyper-parameters for the iMESA algorithm
  IMESAAgentParams params_;
  /// @brief The underlying incremental smoother instance
  gtsam::ISAM2 smoother_;

  /// @brief The set of all observed global variables
  gtsam::KeySet observed_global_variables_;
  /// @brief Mapping from shared variables to the agents with which they are shared
  std::map<gtsam::Key, std::set<RobotId>> shared_var_robots_;
  /// @brief Mapping from other agents to all variables shared between this and the other agent
  std::map<RobotId, gtsam::KeySet> robot_shared_vars_;
  /// @brief The Dual and Penalty terms for each shared variable's biased prior
  std::map<RobotId, std::map<gtsam::Key, std::shared_ptr<biased_priors::BiasedPriorInfo>>> biased_prior_values_;
  /// @brief Variables marked to be initialized from their owner robot
  std::map<gtsam::Key, RobotId> shared_initialization_variables_;
  /// @brief Mapping from other agents to all vars that need to be declared as shared with the other agent
  std::map<RobotId, gtsam::KeySet> undeclared_robot_shared_vars_;
  /// @brief Shared variables updated during the last communication that need to be reeliminated to account for their
  /// new shared estimates and dual variables
  gtsam::FastList<gtsam::Key> reelim_keys_;
  /// @brief Aggregated biased priors declared during communication with other robots
  gtsam::NonlinearFactorGraph new_other_robot_declared_biased_priors_;

  /** INTERFACE **/
 public:
  /** @brief Constructor
   * @param rid: The robot id for this agent
   * @param params: The iMESA specific parameters
   * @param isam2_params: The parameters for the underlying iSAM2 solver
   * WARN: This class will override some isam2_params (relinearizeSkip, cacheLinearizedFactors) to ensure proper
   * functionality of the iMESA algorithm
   */
  IMESAAgent(const RobotId& rid, IMESAAgentParams params = IMESAAgentParams(),
             gtsam::ISAM2Params isam2_params = gtsam::ISAM2Params())
      : IncrementalSAMAgent(rid), params_(params) {
    // Note: to ensure proper behavior of iMESA we override some iSAM2 parameters
    isam2_params.relinearizeSkip = 1;
    isam2_params.cacheLinearizedFactors = false;
    smoother_ = gtsam::ISAM2(isam2_params);
  }

  /** @brief Update interface for new measurements. [2] - Alg. 2
   * @param new_factors: The new measurements. These measurements may be intra or inter robot measurements.
   * @param new_theta: Any new values affected by the new_factors that are not already part of the system.
   * @returns The ISAM2Result containing information on the update.
   */
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                            const gtsam::Values& new_theta = gtsam::Values()) override;

  /** @brief Interface for new shared variables which are the first thing communicated during inter-robot communication
   * @param other_robot: The identifier of the robot with which this agent is communicating
   * @returns Any new variables shared with the other robot since last communication, and all observed global variables
   * that may be shared
   */
  DeclaredVariables declareNewSharedVariables(const RobotId& other_robot) override;

  /** @brief Interface for receiving new shared variables from another robot. [2] - Alg. 3
   * @param other_robot: The identifier of the other robot with which this agent is communicating
   * @param declared_variables: The declared variables of the robot with whom we are communicating
   */
  void receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) override;

  /** @brief Interface to construct data to send to another robot during inter-robot communication, [2], Alg. 3
   * @param other_robot: The identifier of the robot with which this agent is communicating
   * @returns The data to send to the other_robot
   */
  CommunicationData::shared_ptr sendCommunication(const RobotId& other_robot) override;

  /** @brief Interface for receiving shared estimates from inter-robot communication. [2], Alg. 3
   * @param other_robot: The identifier of the other robot with which this agent is communicating
   * @param comm_data: The data received from the robot with whom we are communicating
   */
  void receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) override;

  /// @brief Returns the name of the method (note this is affected by parameters in IMESAAgentParams)
  std::string name() const override;

  /** HELPERS **/
 protected:
  /** @brief Adds new shared variables updating the internal book keeping and constructing biased priors. [2] - Alg. 1
   *  This routine is called both during communication procedures, and during update procedures
   * @param new_shared_vars: New shared variables and the robot they are shared with induced by a inter-robot update
   * @param new_theta: Values containing initial estimates for all new shared variables
   * @param undeclared: Flag indicating that these shared variables are "undeclared" and not yet know to the other robot
   * @returns Biased priors for each new shared variable
   */
  gtsam::NonlinearFactorGraph addNewSharedVariables(const std::vector<std::pair<RobotId, gtsam::Key>>& new_shared_vars,
                                                    const gtsam::Values& new_theta, bool undeclared);
};

}  // namespace imesa