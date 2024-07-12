#pragma once
/** @brief Implementation of Split and Geodesic Biased Priors.
 *
 * @author Dan McGann
 * @date October, 6th 2023
 */

#include "imesa/biased_prior.h"

namespace biased_priors {

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector GeodesicBiasedPrior<VAR_TYPE>::constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                                             boost::optional<gtsam::Matrix&> Hx) {
  return gtsam::traits<VAR_TYPE>::Local(z, x, boost::none, Hx);
}

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector GeodesicBiasedPrior<VAR_TYPE>::genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                                                    boost::optional<gtsam::Matrix&> Hx) {
  return constraintError(x.template cast<VAR_TYPE>(), z.template cast<VAR_TYPE>(), Hx);
}

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector GeodesicBiasedPrior<VAR_TYPE>::evaluateError(const VAR_TYPE& val,
                                                           boost::optional<gtsam::Matrix&> H) const {
  gtsam::Vector err_vec =
      constraintError(val, this->vals_ptr_->shared_estimate_lin_point->template cast<VAR_TYPE>(), H);
  if (H) (*H) = sqrt(this->vals_ptr_->penalty) * (*H);
  return sqrt(this->vals_ptr_->penalty) * (err_vec + (this->vals_ptr_->dual_lin_point / this->vals_ptr_->penalty));
}

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector SplitBiasedPrior<VAR_TYPE>::constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                                          boost::optional<gtsam::Matrix&> Hx) {
  // Translation Error
  gtsam::Matrix dtrans_dx, dlocal_dtrans;
  gtsam::Vector translation_error = gtsam::traits<typename VAR_TYPE::Translation>::Local(
      z.translation(), x.translation(&dtrans_dx), boost::none, &dlocal_dtrans);

  // Rotation Error
  gtsam::Matrix drot_dx, dlocal_drot;
  gtsam::Vector rotation_error =
      gtsam::traits<typename VAR_TYPE::Rotation>::Local(z.rotation(), x.rotation(&drot_dx), boost::none, &dlocal_drot);

  // Construct the result
  gtsam::Vector constraint_satisfaction_gap = gtsam::Vector::Zero(translation_error.size() + rotation_error.size());
  constraint_satisfaction_gap << translation_error, rotation_error;
  if (Hx) {
    gtsam::Matrix dtrans_err_dx = dlocal_dtrans * dtrans_dx;
    gtsam::Matrix drot_err_dx = dlocal_drot * drot_dx;

    // Vertical concatenation
    *Hx = gtsam::Matrix(dtrans_err_dx.rows() + drot_err_dx.rows(), dtrans_err_dx.cols());
    *Hx << dtrans_err_dx, drot_err_dx;
  }

  return constraint_satisfaction_gap;
}

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector SplitBiasedPrior<VAR_TYPE>::genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                                                 boost::optional<gtsam::Matrix&> Hx) {
  return constraintError(x.template cast<VAR_TYPE>(), z.template cast<VAR_TYPE>(), Hx);
}

/*********************************************************************************************************************/
template <class VAR_TYPE>
gtsam::Vector SplitBiasedPrior<VAR_TYPE>::evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H) const {
  gtsam::Vector err_vec =
      constraintError(val, this->vals_ptr_->shared_estimate_lin_point->template cast<VAR_TYPE>(), H);
  if (H) (*H) = sqrt(this->vals_ptr_->penalty) * (*H);
  return sqrt(this->vals_ptr_->penalty) * (err_vec + (this->vals_ptr_->dual_lin_point / this->vals_ptr_->penalty));
}

/*********************************************************************************************************************/
template <class LIE_TYPE>
LIE_TYPE baseInterpolateSLERP(const LIE_TYPE& pa, const LIE_TYPE& pb, double alpha) {
  return gtsam::traits<LIE_TYPE>::Retract(pa, alpha * gtsam::traits<LIE_TYPE>::Local(pa, pb));
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
POSE_TYPE baseInterpolateSPLIT(const POSE_TYPE& start_pose, const POSE_TYPE& end_pose, double alpha) {
  typename POSE_TYPE::Rotation rs = start_pose.rotation();
  typename POSE_TYPE::Rotation re = end_pose.rotation();
  typename POSE_TYPE::Rotation interp_rot = rs.compose(gtsam::traits<typename POSE_TYPE::Rotation>::Expmap(
      alpha * gtsam::traits<typename POSE_TYPE::Rotation>::Logmap(rs.inverse().compose(re))));
  typename POSE_TYPE::Translation interp_trans =
      start_pose.translation() + (alpha * (end_pose.translation() - start_pose.translation()));
  return POSE_TYPE(interp_rot, interp_trans);
}

/*********************************************************************************************************************/
/// @brief Constructs a biased prior factor templatized
template <template <typename> typename BIASED_PRIOR>
gtsam::NonlinearFactor::shared_ptr factory(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
                                           const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas) {
  /** VECTOR **/
  if (typeid(*vals_ptr->shared_estimate) == typeid(gtsam::GenericValue<gtsam::Vector>)) {
    // Always use geodesic for linear variables
    return boost::make_shared<GeodesicBiasedPrior<gtsam::Vector>>(
        key, vals_ptr, gtsam::noiseModel::Unit::Create(vals_ptr->shared_estimate->dim()));
  }
  /** POINT2 **/
  else if (typeid(*vals_ptr->shared_estimate) == typeid(gtsam::GenericValue<gtsam::Vector2>)) {
    // Always use geodesic for linear variables
    return boost::make_shared<GeodesicBiasedPrior<gtsam::Point2>>(
        key, vals_ptr, gtsam::noiseModel::Unit::Create(vals_ptr->shared_estimate->dim()));
  }
  /** POINT3 **/
  else if (typeid(*vals_ptr->shared_estimate) == typeid(gtsam::GenericValue<gtsam::Vector3>)) {
    // Always use geodesic for linear variables
    return boost::make_shared<GeodesicBiasedPrior<gtsam::Point3>>(
        key, vals_ptr, gtsam::noiseModel::Unit::Create(vals_ptr->shared_estimate->dim()));
  }
  /** POSE2 **/
  else if (typeid(*vals_ptr->shared_estimate) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
    gtsam::SharedNoiseModel nm = gtsam::noiseModel::Unit::Create(vals_ptr->shared_estimate->dim());
    if (biased_prior_noise_model_sigmas) {
      auto [rs, ts] = *biased_prior_noise_model_sigmas;
      nm = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << ts, ts, rs).finished());
    }
    return boost::make_shared<BIASED_PRIOR<gtsam::Pose2>>(key, vals_ptr, nm);
  }
  /** POSE3 **/
  else if (typeid(*vals_ptr->shared_estimate) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
    gtsam::SharedNoiseModel nm = gtsam::noiseModel::Unit::Create(vals_ptr->shared_estimate->dim());
    if (biased_prior_noise_model_sigmas) {
      auto [rs, ts] = *biased_prior_noise_model_sigmas;
      nm = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << rs, rs, rs, ts, ts, ts).finished());
    }
    return boost::make_shared<BIASED_PRIOR<gtsam::Pose3>>(key, vals_ptr, nm);
  } else {
    throw std::runtime_error("Invalid shared_estimate type passed in vals_ptr to Biased Prior factory");
  }
}

}  // namespace biased_priors