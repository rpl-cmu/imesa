#pragma once
/** @brief This module defines a generic interface for an incremental "Smoothing and Mapping" (SAM) agent.
 * Classes (like iMESA) that inherit from this interface are meant to solve incremental Collaborative Simultaneous
 * Localization and Mapping (C-SLAM) problems for multi-robot teams. In this context each robot is to have a unique
 * instance of an "agent" solver that works similar to existing incremental SLAM solvers (e.g. gtsam::ISAM2) but with
 * additional functionality to incorporate information from communications between robots.
 *
 * Note: As discussed in [2] any incremental C-SLAM solver requires either sharing entire states or requires a 2-stage
 * communication. This interface assumes that algorithms use the more efficient 2-stage approach.
 *
 * [2]  D. McGann and M. Kaess, "iMESA: Incremental Distributed Optimization for Collaborative
 *      Simultaneous Localization and Mapping", Proc. Robotic Science and Systems (RSS), 2024
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */
#include <gtsam/nonlinear/ISAM2.h>

namespace incremental_sam_agent {

/** TYPES**/
using RobotId = char;
inline static const char GLOBAL_ID = '#';

/// @brief Base type for derived communication data
struct CommunicationData {
  typedef std::shared_ptr<CommunicationData> shared_ptr;
  virtual ~CommunicationData() = default;  // Force struct to be polymorphic
};

/// @brief Container for sharing new shared variables between robots
struct DeclaredVariables {
  /// @brief Shared variables directly observed since last communication
  gtsam::KeySet new_shared_variables;
  /// @brief List of all global variables observed, which may be shared
  gtsam::KeySet observed_global_variables;
};

class IncrementalSAMAgent {
  /** FIELDS **/
 protected:
  /// @brief The unique identifier for this agent
  RobotId robot_id_;
  /// @brief The current solution of the smoother
  gtsam::Values current_estimate_;

  /** INTERFACE **/
 public:
  /// @brief Constructor populating the unique robot identifier
  IncrementalSAMAgent(const RobotId& rid) : robot_id_(rid) {}

  /** @brief Update interface for new measurements.
   * @param new_factors: The new measurements. These measurements may be intra or inter robot measurements.
   * @param new_theta: Any new values affected by the new_factors that are not already part of the system.
   * @returns The ISAM2Result containing information on the update.
   */
  virtual gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                                    const gtsam::Values& new_theta = gtsam::Values()) = 0;

  /** @brief Interface for new shared variables which are the first thing communicated during inter-robot communication
   * @param other_robot: The identifier of the robot with which this agent is communicating
   * @returns Any new variables shared with the other robot since last communication, and all observed global variables
   * that may be shared
   */
  virtual DeclaredVariables declareNewSharedVariables(const RobotId& other_robot) = 0;

  /** @brief Interface for receiving new shared variables from another robot.
   * @param other_robot: The identifier of the other robot with which this agent is communicating
   * @param declared_variables: The declared variables of the robot with whom we are communicating
   */
  virtual void receiveNewSharedVariables(const RobotId& other_robot, const DeclaredVariables& declared_variables) = 0;

  /** @brief Interface to construct data to send to another robot during inter-robot communication,
   * @param other_robot: The identifier of the robot with which this agent is communicating
   * @returns The data to send to the other_robot
   */
  virtual CommunicationData::shared_ptr sendCommunication(const RobotId& other_robot) = 0;

  /** @brief Interface for receiving shared estimates from inter-robot communication.
   * @param other_robot: The identifier of the other robot with which this agent is communicating
   * @param comm_data: The data received from the robot with whom we are communicating
   */
  virtual void receiveCommunication(const RobotId& other_robot, const CommunicationData::shared_ptr& comm_data) = 0;

  /// @brief Returns the current solution of the distributed incremental smoother
  virtual gtsam::Values getEstimate() { return current_estimate_; }

  /// @brief Returns the name of the method of the agent
  virtual std::string name() const = 0;
};

}  // namespace incremental_sam_agent