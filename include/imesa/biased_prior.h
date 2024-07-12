#pragma once
/** @brief Implementation of Biased Priors used by the iMESA algorithm to
 * enforce equality of shared variables between robots in a multi-robot team.
 *
 * In this module we implement the Split and Geodesic Biased Priors.
 * For details on these biased priors and other options see [1].
 *
 * [1] D. McGann, K. Lassak, and M. Kaess, “Asynchronous distributed smoothing and mapping via
 * on-manifold consensus ADMM,” Proc. IEEE Intl. Conf. on Robotics and Automation (ICRA), 2024
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace biased_priors {
/**
 * ########     ###     ######  ########
 * ##     ##   ## ##   ##    ## ##
 * ##     ##  ##   ##  ##       ##
 * ########  ##     ##  ######  ######
 * ##     ## #########       ## ##
 * ##     ## ##     ## ##    ## ##
 * ########  ##     ##  ######  ########
 */

/// @brief Type for a generic constraint function
typedef std::function<gtsam::Vector(const gtsam::Value&, const gtsam::Value&, boost::optional<gtsam::Matrix&>)>
    ConstraintFunction;

/// @brief Type for a function that interpolates a new shared estimate
typedef std::function<boost::shared_ptr<gtsam::Value>(const gtsam::Value&, const gtsam::Value&)>
    SharedEstimateInterpolator;

/// @brief The types of biased priors we can use
enum BiasedPriorType { GEODESIC, SPLIT };
/// @brief Converts string form of BiasedPriorType to enumeration type
BiasedPriorType BPTypeFromString(const std::string& str);
/// @brief Converts enumeration of BiasedPriorType to string representation
std::string BPTypeToString(const BiasedPriorType& type);

/** @brief The values required by the based prior
 * These values are modified externally (during robot communication) and therefore
 * they are stored in biased priors as a pointer and accessed during relinearlization.
 */
struct BiasedPriorInfo {
  /** TYPES **/
  typedef std::shared_ptr<BiasedPriorInfo> shared_ptr;

  /** FIELDS **/
  /// @brief The current shared estimate for this biased prior ($z$)
  boost::shared_ptr<gtsam::Value> shared_estimate;
  /// @brief The current shared estimate used for linearization for this biased prior ($z$)
  boost::shared_ptr<gtsam::Value> shared_estimate_lin_point;
  /// @brief The dual term accumulated for the biased prior ($\lambda$)
  gtsam::Vector dual;
  /// @brief The dual term accumulated for linearization for the biased prior ($\lambda$)
  gtsam::Vector dual_lin_point;
  /// @brief The penalty term accumulated for the biased prior ($\beta$)
  double penalty;

  /** FUNCTIONS **/
  /// @brief Computes the shared estimate update for the biased prior $z = interpolate(x_i, x_j)$
  SharedEstimateInterpolator interpolate;
  /// @brief Computes the constraint function for the biased prior $q(x,z)$
  ConstraintFunction constraint_func;

  /** INTERFACE **/
  /// @brief default constructor
  BiasedPriorInfo() = default;
  /// @brief Constructor from initial est and initial penalty, dual and functions are auto-populated
  BiasedPriorInfo(const BiasedPriorType& type, const gtsam::Value& init_estimate, double init_penalty);

  /// @brief Output to string
  friend std::ostream& operator<<(std::ostream& o, BiasedPriorInfo const& bpi);
};

/** @brief A biased prior is the combination of Augmented Lagrangian Dual and Penalty Terms
 * We augment the factor-graph with these factors to enforce equality constraints.
 */
template <class VAR_TYPE>
class BiasedPrior : public gtsam::NoiseModelFactor1<VAR_TYPE> {
 protected:
  /// @brief Pointer to struct containing current edge and dual variables
  BiasedPriorInfo::shared_ptr vals_ptr_;

 public:
  /** @brief Constructor for base BiasedPrior
   * @param key: The key of the variable this biased prior affects
   * @param vals_ptr: Pointer to the structure containing the current edge and dual variables for this biased prior
   * @param noise_model: The noise model with which to weight this biased prior
   */
  BiasedPrior(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
              const gtsam::SharedNoiseModel& noise_model)
      : gtsam::NoiseModelFactor1<VAR_TYPE>(noise_model, key), vals_ptr_(vals_ptr) {}

  /// @brief The non-squared biased prior error $\sqrt{\beta} q(\theta) - \frac{\lambda}{\beta}$
  virtual gtsam::Vector evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const = 0;
};

/**
 *  ######   ########  #######  ########  ########  ######  ####  ######
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 * ##        ##       ##     ## ##     ## ##       ##        ##  ##
 * ##   #### ######   ##     ## ##     ## ######    ######   ##  ##
 * ##    ##  ##       ##     ## ##     ## ##             ##  ##  ##
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 *  ######   ########  #######  ########  ########  ######  ####  ######
 */
/// @brief Implementation of a Biased Prior using a Geodesic Constraint Function
template <class VAR_TYPE>
class GeodesicBiasedPrior : public BiasedPrior<VAR_TYPE> {
 public:
  /// @brief Constructor for GeodesicBiasedPrior. For params see BiasedPrior
  GeodesicBiasedPrior(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
                      const gtsam::SharedNoiseModel& noise_model)
      : BiasedPrior<VAR_TYPE>(key, vals_ptr, noise_model) {}

  /// @brief The non-squared biased prior error $\sqrt{\beta} q(\theta) - \frac{\lambda}{\beta}$
  gtsam::Vector evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const;

  /// @brief The constraint function error $q(x)$
  static gtsam::Vector constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                       boost::optional<gtsam::Matrix&> Hx = boost::none);
  /// @brief The constraint function error $q(x)$ that accepts a generic value
  static gtsam::Vector genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                              boost::optional<gtsam::Matrix&> Hx = boost::none);
};

/**
 *  ######  ########  ##       #### ########
 * ##    ## ##     ## ##        ##     ##
 * ##       ##     ## ##        ##     ##
 *  ######  ########  ##        ##     ##
 *       ## ##        ##        ##     ##
 * ##    ## ##        ##        ##     ##
 *  ######  ##        ######## ####    ##
 */
/// @brief Implementation of a Biased Prior using a SPLIT Constraint Function
template <class VAR_TYPE>
class SplitBiasedPrior : public BiasedPrior<VAR_TYPE> {
 public:
  /// @brief Constructor for SplitBiasedPrior. For params see BiasedPrior
  SplitBiasedPrior(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
                   const gtsam::SharedNoiseModel& noise_model)
      : BiasedPrior<VAR_TYPE>(key, vals_ptr, noise_model) {}

  /// @brief The non-squared biased prior error $\sqrt{\beta} q(\theta) - \frac{\lambda}{\beta}$
  gtsam::Vector evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const;

  /// @brief The constraint function error $q(x)$
  static gtsam::Vector constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                       boost::optional<gtsam::Matrix&> Hx = boost::none);
  /// @brief The constraint function error $q(x)$ that accepts a generic value
  static gtsam::Vector genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                              boost::optional<gtsam::Matrix&> Hx = boost::none);
};

/**
 * #### ##    ## ######## ######## ########  ########   #######  ##          ###    ######## ########
 *  ##  ###   ##    ##    ##       ##     ## ##     ## ##     ## ##         ## ##      ##    ##
 *  ##  ####  ##    ##    ##       ##     ## ##     ## ##     ## ##        ##   ##     ##    ##
 *  ##  ## ## ##    ##    ######   ########  ########  ##     ## ##       ##     ##    ##    ######
 *  ##  ##  ####    ##    ##       ##   ##   ##        ##     ## ##       #########    ##    ##
 *  ##  ##   ###    ##    ##       ##    ##  ##        ##     ## ##       ##     ##    ##    ##
 * #### ##    ##    ##    ######## ##     ## ##         #######  ######## ##     ##    ##    ########
 */

/** @brief Interpolates a Lie Group Object spherically.
 * @param pa: The start object of interpolation
 * @param pb: The end object of interpolation
 * @param alpha: The interpolation factor
 * @returns The interpolated object
 */
template <class LIE_TYPE>
LIE_TYPE baseInterpolateSLERP(const LIE_TYPE& pa, const LIE_TYPE& pb, double alpha);

/** @brief Interpolates a POSE Object: linearly for its translation and spherically for its rotation
 * @param pa: The start pose of interpolation
 * @param pb: The end pose of interpolation
 * @param alpha: The interpolation factor
 * @returns The interpolated pose
 */
template <class POSE_TYPE>
POSE_TYPE baseInterpolateSPLIT(const POSE_TYPE& start_pose, const POSE_TYPE& end_pose, double alpha);

/** @brief Interpolates the generic values to compute a new shared estimate.
 * Actual implementation handled in template specialization functions
 * @param this_estimate: The current solution to the variable held by this agent
 * @param other_estimate: The current solution to the variable held by the other agent
 * @returns The new shared estimate (z) held jointly by both agents
 */
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_vector(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point2(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point3(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose2(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose3(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate);
/**
 * ########    ###     ######  ########  #######  ########  ##    ##
 * ##         ## ##   ##    ##    ##    ##     ## ##     ##  ##  ##
 * ##        ##   ##  ##          ##    ##     ## ##     ##   ####
 * ######   ##     ## ##          ##    ##     ## ########     ##
 * ##       ######### ##          ##    ##     ## ##   ##      ##
 * ##       ##     ## ##    ##    ##    ##     ## ##    ##     ##
 * ##       ##     ##  ######     ##     #######  ##     ##    ##
 */

/** @brief Constructs a biased prior factor.
 * Factors are templatized, but we want to construct biased priors without templatizing the agent class.
 * So that the class can handle generic shared measurements of any type. therefore we need to deduce the type at runtime
 * Note: no macro usage to make things easier to read (though much longer code wise)
 * @param biased_prior_noise_model_sigmas: (rotation, translation) the weights with which to construct the noise model
 * of the biased prior. If none the noise model defaults to identity. Also note the weights are only used for POSE
 * biased priors.
 */
gtsam::NonlinearFactor::shared_ptr factory(
    const BiasedPriorType& type, const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas = std::nullopt);

/// @brief Constructs a biased prior factor templatized
template <template <typename> typename BIASED_PRIOR>
gtsam::NonlinearFactor::shared_ptr factory(
    const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas = std::nullopt);

/** @brief Constructs a shared estimate interpolator function
 * Each shared variable needs its own interpolator function as the interpolation depends on type so we use this factory
 * to construct the corresponding interpolator for the given biased prior
 */
SharedEstimateInterpolator interpolator_factory(const boost::shared_ptr<gtsam::Value>& val);

/** @brief Constructs the appropriate constraint function for the value
 * To update the dual variable we need access to the constraint function q.
 */
ConstraintFunction constraint_func_factory(const BiasedPriorType& type, const boost::shared_ptr<gtsam::Value>& val);

};  // namespace biased_priors

#include "imesa/biased_prior-inl.h"