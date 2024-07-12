/** @brief Implementation of Split and Geodesic Biased Priors.
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */

#include "imesa/biased_prior.h"

namespace biased_priors {
/*********************************************************************************************************************/
BiasedPriorType BPTypeFromString(const std::string& str) {
  if (str == "GEODESIC") {
    return biased_priors::BiasedPriorType::GEODESIC;
  } else if (str == "SPLIT") {
    return biased_priors::BiasedPriorType::SPLIT;
  } else {
    throw std::runtime_error("BPTypeFromString: invalid pose_biased_prior_type");
  }
}
/*********************************************************************************************************************/
std::string BPTypeToString(const BiasedPriorType& type) {
  if (type == BiasedPriorType::GEODESIC) {
    return "GEODESIC";
  } else if (type == BiasedPriorType::SPLIT) {
    return "SPLIT";
  } else {
    throw std::runtime_error("BPTypeToString: invalid pose_biased_prior_type");
  }
}

/*********************************************************************************************************************/
BiasedPriorInfo::BiasedPriorInfo(const BiasedPriorType& type, const gtsam::Value& init_estimate, double init_penalty) {
  this->shared_estimate = init_estimate.clone();
  this->shared_estimate_lin_point = init_estimate.clone();
  this->dual = gtsam::Vector::Zero(init_estimate.dim());
  this->dual_lin_point = gtsam::Vector::Zero(init_estimate.dim());
  this->penalty = init_penalty;
  this->interpolate = interpolator_factory(this->shared_estimate);
  this->constraint_func = constraint_func_factory(type, this->shared_estimate);
}

/*********************************************************************************************************************/
std::ostream& operator<<(std::ostream& o, BiasedPriorInfo const& bpi) {
  o << "Biased Prior Info | Dual: [" << bpi.dual(0) << ", " << bpi.dual(1) << ", " << bpi.dual(2)
    << "] Penalty: " << bpi.penalty;
  return o;
}

/*********************************************************************************************************************/
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_vector(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate) {
  gtsam::Vector new_shared_estimate =
      baseInterpolateSLERP(this_estimate.cast<gtsam::Vector>(), other_estimate.cast<gtsam::Vector>(), 0.5);
  return gtsam::GenericValue<gtsam::Vector>(new_shared_estimate).clone();
}

boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point2(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate) {
  gtsam::Point2 new_shared_estimate =
      baseInterpolateSLERP(this_estimate.cast<gtsam::Point2>(), other_estimate.cast<gtsam::Point2>(), 0.5);
  return gtsam::GenericValue<gtsam::Point2>(new_shared_estimate).clone();
}

boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point3(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate) {
  gtsam::Point3 new_shared_estimate =
      baseInterpolateSLERP(this_estimate.cast<gtsam::Point3>(), other_estimate.cast<gtsam::Point3>(), 0.5);
  return gtsam::GenericValue<gtsam::Point3>(new_shared_estimate).clone();
}

boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose2(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate) {
  gtsam::Pose2 new_shared_estimate =
      baseInterpolateSPLIT(this_estimate.cast<gtsam::Pose2>(), other_estimate.cast<gtsam::Pose2>(), 0.5);
  return gtsam::GenericValue<gtsam::Pose2>(new_shared_estimate).clone();
}

boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose3(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate) {
  gtsam::Pose3 new_shared_estimate =
      baseInterpolateSPLIT(this_estimate.cast<gtsam::Pose3>(), other_estimate.cast<gtsam::Pose3>(), 0.5);
  return gtsam::GenericValue<gtsam::Pose3>(new_shared_estimate).clone();
}

/*********************************************************************************************************************/
gtsam::NonlinearFactor::shared_ptr factory(
    const BiasedPriorType& type, const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas) {
  if (type == BiasedPriorType::GEODESIC) {
    return factory<GeodesicBiasedPrior>(key, vals_ptr, biased_prior_noise_model_sigmas);
  } else if (type == BiasedPriorType::SPLIT) {
    return factory<SplitBiasedPrior>(key, vals_ptr, biased_prior_noise_model_sigmas);
  } else {
    throw std::runtime_error("Invalid BiasedPriorType type passed to Biased Prior factory");
  }
}

/*********************************************************************************************************************/
SharedEstimateInterpolator interpolator_factory(const boost::shared_ptr<gtsam::Value>& val) {
  if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Vector>)) {
    return &interpolate_shared_estimate_vector;
  } else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Point2>)) {
    return &interpolate_shared_estimate_point2;
  } else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Point3>)) {
    return &interpolate_shared_estimate_point3;
  } else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
    return &interpolate_shared_estimate_pose2;
  } else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
    return &interpolate_shared_estimate_pose3;
  } else {
    throw std::runtime_error("Invalid type passed in vals_ptr to interpolator_factory");
  }
}

/*********************************************************************************************************************/
ConstraintFunction constraint_func_factory(const BiasedPriorType& type, const boost::shared_ptr<gtsam::Value>& val) {
  /** VECTOR **/
  if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Vector>)) {
    // Always use geodesic for linear variables
    return &GeodesicBiasedPrior<gtsam::Vector>::genericConstraintError;
  }
  /** POINT2 **/
  else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Point2>)) {
    // Always use geodesic for linear variables
    return &GeodesicBiasedPrior<gtsam::Point2>::genericConstraintError;
  }
  /** POINT3 **/
  else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Point3>)) {
    // Always use geodesic for linear variables
    return &GeodesicBiasedPrior<gtsam::Point3>::genericConstraintError;
  }
  /** POSE2 **/
  else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
    if (type == BiasedPriorType::GEODESIC) {
      return &GeodesicBiasedPrior<gtsam::Pose2>::genericConstraintError;
    } else if (type == BiasedPriorType::SPLIT) {
      return &SplitBiasedPrior<gtsam::Pose2>::genericConstraintError;
    }
  }
  /** POSE3 **/
  else if (typeid(*val) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
    if (type == BiasedPriorType::GEODESIC) {
      return &GeodesicBiasedPrior<gtsam::Pose3>::genericConstraintError;
    } else if (type == BiasedPriorType::SPLIT) {
      return &SplitBiasedPrior<gtsam::Pose3>::genericConstraintError;
    }
  }
  throw std::runtime_error("Invalid type or value type passed in vals_ptr to constraint_func_factory");
}

}  // namespace biased_priors