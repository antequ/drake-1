#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

// Set both to false to replicate the behavior in master.
constexpr bool COMPUTE_TWO_JACOBIANS = true;
constexpr bool use_conservative_initial_guesses = false;

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
  num_half_vie_jacobian_reforms_ = 0;
  num_half_vie_iter_factorizations_ = 0;
  num_half_vie_function_evaluations_ = 0;
  num_half_vie_jacobian_function_evaluations_ = 0;
  num_half_vie_nr_iterations_ = 0;
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  const double kDefaultAccuracy = 1e-1;  // Good for this particular integrator.
  const double kLoosestAccuracy = 5e-1;  // Loosest accuracy is quite loose.

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that the maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set for VelocityImplicitEulerIntegrator.");

    this->request_initial_step_size_target(this->get_maximum_step_size());
  }

  // Sets the working accuracy to a good value.
  double working_accuracy = this->get_target_accuracy();

  // If the user asks for accuracy that is looser than the loosest this
  // integrator can provide, use the integrator's loosest accuracy setting
  // instead.
  if (isnan(working_accuracy))
    working_accuracy = kDefaultAccuracy;
  else if (working_accuracy > kLoosestAccuracy)
    working_accuracy = kLoosestAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->Jy_vie_.resize(0, 0);
  this->Jfyy_vie_.resize(0, 0);
  this->Jfyq_vie_.resize(0, 0);
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::
    ComputeAndFactorImplicitEulerIterationMatrix(
        const MatrixX<T>& J, const MatrixX<T>& Jfyy, const MatrixX<T>& Jfyq,
        const T& h,
        typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  // TODO(edrumwri) Investigate using a move-type operation below.
  // We form the iteration matrix in this particular way to avoid an O(n^2)
  // subtraction as would be the case with:
  // MatrixX<T>::Identity(n, n) - h * J.
  if (COMPUTE_TWO_JACOBIANS && (Jfyy.rows() > 0)) {
    const int n = Jfyy.rows();
    //std::cerr << "Factorizing, step size " << h << " rows: " << n << std::endl;
    iteration_matrix->SetAndFactorIterationMatrix(-h * (Jfyy + h * Jfyq) +
                                                  MatrixX<T>::Identity(n, n));
  } else {
    const int n = J.rows();
    iteration_matrix->SetAndFactorIterationMatrix(-h * J +
                                                  MatrixX<T>::Identity(n, n));
  }
  if (!iteration_matrix->matrix_factored()) {
    throw std::runtime_error("Iteration matrix failed to be factorized!");
  }
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::CalcVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, MatrixX<T>* Jy, MatrixX<T>* Jfyy, MatrixX<T>* Jfyq) {
  // Note: Unlike ImplicitIntegrator<T>::CalcJacobian(), we neither save the
  // context or change it back, because our implementation of
  // StepVelocityImplicitEuler() does not require the context to be restored.
  this->increment_jacobian_evaluations();

  // Get the existing number of ODE evaluations.
  int64_t existing_ODE_evals = this->get_num_derivative_evaluations();

  // Compute the Jacobian using the selected computation scheme.
  if (this->get_jacobian_computation_scheme() ==
          ImplicitIntegrator<
              T>::JacobianComputationScheme::kForwardDifference ||
      this->get_jacobian_computation_scheme() ==
          ImplicitIntegrator<
              T>::JacobianComputationScheme::kCentralDifference) {
    // Compute the Jacobian using numerical differencing.
    DRAKE_ASSERT(qdot_ != nullptr && qdot_->size() == qn.size());

    const math::NumericalGradientOption numerical_gradient_method(
        (this->get_jacobian_computation_scheme() ==
         ImplicitIntegrator<T>::JacobianComputationScheme::kCentralDifference)
            ? math::NumericalGradientMethod::kCentral
            : math::NumericalGradientMethod::kForward);

    if (COMPUTE_TWO_JACOBIANS && !(this->get_use_full_newton()) &&
        (qn.size() > 0)) {
      int ny = y.size();
      int nv = this->get_context().get_continuous_state().num_v();
      // Define the lambda fy_of_y to evaluate f(y)
      std::function<void(const VectorX<T>&, VectorX<T>*)> fy_of_y =
          [&t, &qn, this](const VectorX<T>& y_state, VectorX<T>* f_result) {
            *f_result = this->ComputeFyOfQY(t, y_state, qn);
          };
      // Define the lambda fy_of_q to evaluate f(q)
      std::function<void(const VectorX<T>&, VectorX<T>*)> fy_of_q =
          [&t, &y, this](const VectorX<T>& q_state, VectorX<T>* f_result) {
            *f_result = this->ComputeFyOfQY(t, y, q_state);
          };

      // Compute the two Jacobians
      auto Jfyq_temp = math::ComputeNumericalGradient(
          fy_of_q, qn, numerical_gradient_method);

      DRAKE_DEMAND(Jfyq_temp.rows() == ny);
      DRAKE_DEMAND(Jfyq_temp.cols() == qn.size());
      *Jfyy =
          math::ComputeNumericalGradient(fy_of_y, y, numerical_gradient_method);

      DRAKE_DEMAND(Jfyy->cols() == ny);
      DRAKE_DEMAND(Jfyy->rows() == ny);
      *Jfyq = MatrixX<T>::Zero(ny, ny);
      // the final time and position should be correct (set to qn), since
      // ComputeFyOfQY sets it so.
      this->get_mutable_context()
          ->get_mutable_continuous_state()
          .get_mutable_generalized_position()
          .SetFromVector(qn);
      VectorX<T> v_vector = VectorX<T>(nv);
      // compute Jq * N(q)
      for (int i = 0; i < nv; i++) {
        v_vector = VectorX<T>::Unit(nv, i);
        this->get_system().MapVelocityToQDot(this->get_context(), v_vector,
                                             qdot_.get());
        Jfyq->col(i) = Jfyq_temp * qdot_->get_value();
      }
      if (false) {
        std::cout << "Jfyy:\n" << *Jfyy << std::endl;
        std::cout << "Jfyq:\n" << *Jfyq << std::endl;
        std::cout << "Jfyy + h Jfyq:\n" << *Jfyy + h * (*Jfyq) << std::endl;
        // check
        // Define the lambda l_of_y to evaluate l(y).
        std::function<void(const VectorX<T>&, VectorX<T>*)> l_of_y =
            [&qk, &t, &qn, &h, this](const VectorX<T>& y_state,
                                     VectorX<T>* l_result) {
              *l_result =
                  this->ComputeLOfY(t, y_state, qk, qn, h, this->qdot_.get());
            };

        // Compute Jy by passing l(y) to math::ComputeNumericalGradient().
        // TODO(antequ): Right now we modify the context twice each time we call
        // l(y): once when we calculate qⁿ + h N(qₖ) v
        // (SetTimeAndContinuousState()), and once when we calculate l(y)
        // (get_mutable_generalized_position()). However, this is only necessary
        // for each y that modifies a velocity (v). For all but one of the
        // miscellaneous states (z), we can reuse the position so that the
        // context needs only one modification. Investigate how to refactor this
        // logic to achieve this performance benefit while maintaining code
        // readability.
        *Jy = math::ComputeNumericalGradient(l_of_y, y,
                                             numerical_gradient_method);
        std::cout << "Jy:\n" << *Jy << std::endl;

        if (this->IsBadJacobian(*Jfyy)) {
          std::cerr << y << std::endl;
          std::cerr << "retrying!" << std::endl;
        }
      }
    } else {
      // Define the lambda l_of_y to evaluate l(y).
      std::function<void(const VectorX<T>&, VectorX<T>*)> l_of_y =
          [&qk, &t, &qn, &h, this](const VectorX<T>& y_state,
                                   VectorX<T>* l_result) {
            *l_result =
                this->ComputeLOfY(t, y_state, qk, qn, h, this->qdot_.get());
          };

      // Compute Jy by passing l(y) to math::ComputeNumericalGradient().
      // TODO(antequ): Right now we modify the context twice each time we call
      // l(y): once when we calculate qⁿ + h N(qₖ) v
      // (SetTimeAndContinuousState()), and once when we calculate l(y)
      // (get_mutable_generalized_position()). However, this is only necessary
      // for each y that modifies a velocity (v). For all but one of the
      // miscellaneous states (z), we can reuse the position so that the context
      // needs only one modification. Investigate how to refactor this logic to
      // achieve this performance benefit while maintaining code readability.
      *Jy =
          math::ComputeNumericalGradient(l_of_y, y, numerical_gradient_method);
    }
  } else if (this->get_jacobian_computation_scheme() ==
             ImplicitIntegrator<T>::JacobianComputationScheme::kAutomatic) {
    this->ComputeAutoDiffVelocityJacobian(t, h, y, qk, qn, Jy);
  } else {
    throw new std::logic_error("Invalid Jacobian computation scheme.");
  }

  // Use the new number of ODE evaluations to determine the number of ODE
  // evaluations used in computing Jacobians.
  this->increment_jacobian_computation_derivative_evaluations(
      this->get_num_derivative_evaluations() - existing_ODE_evals);
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::ComputeAutoDiffVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, MatrixX<T>* Jy) {
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator ComputeAutoDiffVelocityJacobian "
      "{}-Jacobian t={}", y.size(), t);
  DRAKE_LOGGER_DEBUG("  computing from qk {}, y {}", qk.transpose(),
                     y.transpose());
  // TODO(antequ): Investigate how to refactor this method to use
  // math::jacobian(), if possible.

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  const System<T>& system = this->get_system();
  if (system_ad_ == nullptr) {
    system_ad_ = system.ToAutoDiffXd();
    context_ad_ = system_ad_->AllocateContext();
  }
  const Context<T>& context = this->get_context();
  context_ad_->SetTimeStateAndParametersFrom(context);
  system_ad_->FixInputPortsFrom(system, context, context_ad_.get());

  if (qdot_ad_ == nullptr || qdot_ad_->size() != qn.size()) {
    qdot_ad_ = std::make_unique<BasicVector<AutoDiffXd>>(qn.size());
  }

  // Initialize an AutoDiff version of the variable y.
  const int ny = y.size();
  VectorX<AutoDiffXd> y_ad = y;
  for (int i = 0; i < ny; ++i) {
    y_ad(i).derivatives() = VectorX<T>::Unit(ny, i);
  }

  // Evaluate the AutoDiff system with y_ad.
  const VectorX<AutoDiffXd> result = this->ComputeLOfY(
      t, y_ad, qk, qn, h, this->qdot_ad_.get(),
      *(this->system_ad_), this->context_ad_.get());

  *Jy = math::autoDiffToGradientMatrix(result);

  // Sometimes ℓ(y) does not depend on, for example, when ℓ(y) is a constant or
  // when ℓ(y) depends only on t. In this case, make sure that the Jacobian
  // isn't a n ✕ 0 matrix (this will cause a segfault when forming Newton
  // iteration matrices); if it is, we set it equal to an n x n zero matrix.
  if (Jy->cols() == 0) {
    *Jy = MatrixX<T>::Zero(ny, ny);
  }

  DRAKE_ASSERT(Jy->rows() == ny);
  DRAKE_ASSERT(Jy->cols() == ny);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::MaybeFreshenVelocityMatrices(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk, const VectorX<T>& qn,
    const T& h, int trial,
    const std::function<void(const MatrixX<T>&, const MatrixX<T>&,
                             const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy, MatrixX<T>* Jfyy, MatrixX<T>* Jfyq) {
  DRAKE_DEMAND(Jy != nullptr && Jfyy != nullptr && Jfyq != nullptr);
  DRAKE_DEMAND(iteration_matrix != nullptr);
  MatrixX<T>* Jac_In_Use =
      COMPUTE_TWO_JACOBIANS && !(this->get_use_full_newton()) && (qn.size() > 0)
          ? Jfyy
          : Jy;
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.
  if (!this->get_reuse() || Jac_In_Use->rows() == 0 ||
      this->IsBadJacobian(*Jac_In_Use)) {
    CalcVelocityJacobian(t, h, y, qk, qn, Jy, Jfyy, Jfyq);
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jy, *Jfyy, *Jfyq, h, iteration_matrix);
    if (trial > 1) {
      std::cerr << this->IsBadJacobian(*Jac_In_Use)
                << " , rows: " << Jac_In_Use->rows() << std::endl;
      std::cerr << *Jac_In_Use << std::endl;
    }
    DRAKE_DEMAND(trial <= 2);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jy, *Jfyy, *Jfyq, h, iteration_matrix);
    DRAKE_DEMAND(trial == 1);
    return true;  // Indicate success.
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing: this will cause the Newton-Raphson
      // process to use the last computed (and already factored) iteration
      // matrix. This matrix may be from a previous time-step or a previously-
      // attempted step size.
      return true;  // Indicate success.

    case 2: {
      // For the second trial, we know the first trial, which uses the last
      // computed iteration matrix, has already failed. We perform the (likely)
      // next least expensive operation, which is re-constructing and factoring
      // the iteration matrix, using the last computed Jacobian. The last
      // computed Jacobian may be from a previous time-step or a previously-
      // attempted step size.
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jy, *Jfyy, *Jfyq, h,
                                          iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, we know that the first two trials, which
      // exhausted all our options short of recomputing the Jacobian, have
      // failed. We recompute the Jacobian matrix and refactor the iteration
      // matrix.

      // If the Jacobians are already fresh, give up.
      if (((COMPUTE_TWO_JACOBIANS && !(this->get_use_full_newton())) ||
           (qn.size() == 0)) &&
          this->get_jacobian_is_fresh()) {
        return false;
      }

      // Note: Based on a few simple experimental tests, we found that the
      // optimization to abort this trial when matrices are already fresh in
      // ImplicitIntegrator<T>::MaybeFreshenMatrices() does not significantly
      // help here, especially because our Jacobian depends on step size h.
      CalcVelocityJacobian(t, h, y, qk, qn, Jy, Jfyy, Jfyq);
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jy, *Jfyy, *Jfyq, h,
                                          iteration_matrix);
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        throw std::domain_error("Unexpected trial number.");
    }
  }
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::FreshenVelocityMatricesIfFullNewton(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk, const VectorX<T>& qn,
    const T& h,
    const std::function<void(const MatrixX<T>&, const MatrixX<T>&,
                             const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy, MatrixX<T>* Jfyy, MatrixX<T>* Jfyq) {
  DRAKE_DEMAND(iteration_matrix != nullptr);
  DRAKE_DEMAND(Jy != nullptr && Jfyy != nullptr && Jfyq != nullptr);

  // Return immediately if full-Newton is not in use.
  if (!this->get_use_full_newton()) return;

  // Compute the initial Jacobian and iteration matrices and factor them.
  CalcVelocityJacobian(t, h, y, qk, qn, Jy, Jfyy, Jfyq);
  this->increment_num_iter_factorizations();
  compute_and_factor_iteration_matrix(*Jy, *Jfyy, *Jfyq, h, iteration_matrix);
}

template <class T>
VectorX<T> VelocityImplicitEulerIntegrator<T>::ComputeResidualR(
    const T& t, const VectorX<T>& y, const VectorX<T>& qk, const VectorX<T>& qn,
    const VectorX<T>& yn, const T& h, BasicVector<T>* qdot) {
  // Compute ℓ(y), which also sets the time and y states of the context.
  const VectorX<T> l_of_y = ComputeLOfY(t, y, qk, qn, h, qdot);

  // Evaluate R(y).
  return y - yn - h * l_of_y;
}

template <class T>
template <typename U>
VectorX<U> VelocityImplicitEulerIntegrator<T>::ComputeLOfY(
    const T& t, const VectorX<U>& y, const VectorX<T>& qk,
    const VectorX<T>& qn, const T& h, BasicVector<U>* qdot,
    const System<U>& system, Context<U>* context) {
  DRAKE_DEMAND(qdot != nullptr);
  DRAKE_DEMAND(context != nullptr);
  int nq = qn.size();
  int ny = y.size();

  // Set the context to (t, qₖ, y)
  // TODO(antequ): Optimize this procedure to both (1) remove unnecessary heap
  // allocations, like in the VectorX<T> constructions of x and q and the return
  // statement, and (2) reduce unnecessary cache invalidations since
  // MapVelocityToQDot() doesn't set any caches.
  VectorX<T> x(nq + ny);
  x.head(nq) = qk;
  x.tail(ny) = y;
  context->SetTimeAndContinuousState(t, x);

  // Compute q = qⁿ + h N(qₖ) v.
  system.MapVelocityToQDot(
      *context, context->get_continuous_state().get_generalized_velocity(),
      &*qdot);
  const VectorX<T> q = qn + h * qdot->get_value();

  // Evaluate ℓ = f_y(t, q, v, z).
  // TODO(antequ): Right now this invalidates the entire cache that depends on
  // any of the continuous state. Investigate invalidating less of the cache
  // once we have a Context method for modifying just the generalized position.
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q);
  const ContinuousState<U>& xc_deriv =
      this->EvalTimeDerivatives(system, *context);
  return xc_deriv.CopyToVector().tail(ny);
}

template <class T>
VectorX<T> VelocityImplicitEulerIntegrator<T>::ComputeFyOfQY(
    const T& t, const VectorX<T>& y, const VectorX<T>& q) {
  Context<T>* context = this->get_mutable_context();
  int nq = q.size();
  int ny = y.size();

  // Set the context to (t, q, y)
  // TODO(antequ): Optimize this procedure to both (1) remove unnecessary heap
  // allocations, like in the VectorX<T> constructions of x and q and the return
  // statement, and (2) reduce unnecessary cache invalidations since
  // MapVelocityToQDot() doesn't set any caches.
  VectorX<T> x(nq + ny);
  x.head(nq) = q;
  x.tail(ny) = y;
  context->SetTimeAndContinuousState(t, x);

  // Evaluate l = f_y(t, q, v, z).
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  return xc_deriv.CopyToVector().tail(ny);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::StepVelocityImplicitEuler(
    const T& t0, const T& h, const VectorX<T>& xn,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy, MatrixX<T>* Jfyy, MatrixX<T>* Jfyq, int trial) {
  using std::abs;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator::StepVelocityImplicitEuler(h={}, t={})",
      h, t0);

  const System<T>& system = this->get_system();
  // Verify xtplus. We also verify the size to make sure we're not making
  // unnecessary heap allocations.
  DRAKE_ASSERT(xtplus != nullptr && xtplus->size() == xn.size() &&
               xtplus_guess.size() == xn.size());

  // Initialize xtplus to the guess
  *xtplus = xtplus_guess;

  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  int nq = cstate.num_q();
  int nv = cstate.num_v();
  int nz = cstate.num_z();
  const Eigen::VectorBlock<const VectorX<T>> qn = xn.head(nq);
  const Eigen::VectorBlock<const VectorX<T>> yn = xn.tail(nv + nz);

  // Define references to q, y, and v portions of xtplus for readability.
  Eigen::VectorBlock<VectorX<T>> qtplus = xtplus->head(nq);
  Eigen::VectorBlock<VectorX<T>> ytplus = xtplus->tail(nv + nz);
  const Eigen::VectorBlock<VectorX<T>> vtplus = xtplus->segment(nq, nv);

  // Set last_qtplus to qk. This will be used in computing dx to determine
  // convergence.
  VectorX<T> last_qtplus = qtplus;

  // Verify the size of qdot_.
  DRAKE_ASSERT(qdot_ != nullptr && qdot_->size() == nq);

  // We compute our residuals at tf = t0 + h.
  const T tf = t0 + h;

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  VectorX<T> dx(xn.size());
  T last_dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed,
  // around (t0, xn). We do not do this calculation if full Newton is in use;
  // the calculation will be performed at the beginning of the loop instead.
  if (!this->get_use_full_newton() &&
      !this->MaybeFreshenVelocityMatrices(
          t0, yn, qn, qn, h, trial,
          ComputeAndFactorImplicitEulerIterationMatrix, iteration_matrix, Jy,
          Jfyy, Jfyq)) {
    return false;
  }

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    DRAKE_LOGGER_DEBUG(
        "VelocityImplicitEulerIntegrator::StepVelocityImplicitEuler() entered "
        "for t={}, h={}, trial={}",
        t0, h, trial);

    this->FreshenVelocityMatricesIfFullNewton(
        tf, ytplus, qtplus, qn, h, ComputeAndFactorImplicitEulerIterationMatrix,
        iteration_matrix, Jy, Jfyy, Jfyq);

    // Update the number of Newton-Raphson iterations.
    ++num_nr_iterations_;

    // Evaluate the residual error, which is defined above as R(yₖ):
    //     R(yₖ) = yₖ - yⁿ - h l(yₖ).
    VectorX<T> residual =
        ComputeResidualR(tf, ytplus, qtplus, qn, yn, h, qdot_.get());

    // Compute the state update using the equation A*y = -R(), where A is the
    // iteration matrix.
    const VectorX<T> dy = iteration_matrix->Solve(-residual);

    // Update the y portion of xtplus to yₖ₊₁.
    ytplus += dy;

    // Update the q portion of xtplus to qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁. Note that
    // currently, qtplus is set to qₖ, while vtplus is set to vₖ₊₁.
    // TODO(antequ): Optimize this so that the context doesn't invalidate the
    // position state cache an unnecessary number of times, because evaluating
    // N(q) does not set any cache.
    // TODO(antequ): Right now this invalidates the entire cache that depends
    // on any of the continuous state. Investigate invalidating less of the
    // cache once we have a Context method for modifying just the generalized
    // position.
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(qtplus);
    system.MapVelocityToQDot(*context, vtplus, qdot_.get());
    qtplus = qn + h * qdot_->get_value();
    dx << qtplus - last_qtplus, dy;

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);

    // TODO(antequ): Replace this with CalcStateChangeNorm() when error
    // control has been implemented.
    // Get the norm of the update vector.
    T dx_norm = dx_state_->CopyToVector().norm();

    // Check for Newton-Raphson convergence.
    typename ImplicitIntegrator<T>::ConvergenceStatus status =
        this->CheckNewtonConvergence(i, *xtplus, dx, dx_norm, last_dx_norm);

    // If it converged, we're done.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConverged)
      return true;
    // If it diverged, we have to abort and try again.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kDiverged) break;
    // Otherwise, continue to the next Newton-Raphson iteration.
    DRAKE_DEMAND(status ==
                 ImplicitIntegrator<T>::ConvergenceStatus::kNotConverged);

    last_dx_norm = dx_norm;
    last_qtplus = qtplus;
  }

  DRAKE_LOGGER_DEBUG(
      "Velocity-Implicit Euler integrator convergence failed"
      "for t={}, h={}, trial={}",
      t0, h, trial);

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try; otherwise, the following code will recurse
  // into this function again, and freshen computations as helpful. Note that
  // get_reuse() returns false if "full Newton-Raphson" mode is activated (see
  // ImplicitIntegrator::get_use_full_newton()).
  if (!this->get_reuse()) return false;

  // Try StepVelocityImplicitEuler() again. This method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  return StepVelocityImplicitEuler(t0, h, xn, xtplus_guess, xtplus,
                                   iteration_matrix, Jy, Jfyy, Jfyq, trial + 1);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::StepHalfVelocityImplicitEulers(
    const T& t0, const T& h, const VectorX<T>& xn,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jy, MatrixX<T>* Jfyy, MatrixX<T>* Jfyq) {
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator::StepHalfVelocityImplicitEulers(h={}, "
      "t={})",
      h, t0);

  // Store statistics before "error control". The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the half-sized-step-specific statistics.
  const int64_t stored_num_jacobian_evaluations =
      this->get_num_jacobian_evaluations();
  const int64_t stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  const int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  const int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  const int64_t stored_num_nr_iterations =
      this->get_num_newton_raphson_iterations();

  // We set our guess for the state after a half-step to the average of the
  // guess for the final state, xtplus_guess, and the initial state, xt0.
  VectorX<T> xtmp =
      use_conservative_initial_guesses ? xn : 0.5 * (xn + xtplus_guess);
  const VectorX<T>& xthalf_guess = xtmp;
  bool success = StepVelocityImplicitEuler(
      t0, 0.5 * h, xn, xthalf_guess, xtplus, iteration_matrix, Jy, Jfyy, Jfyq);
  if (!success) {
    DRAKE_LOGGER_DEBUG("First Half VIE convergence failed.");
  } else {
    // Swap the current output, xtplus, into xthalf, which functions as the new
    // xⁿ.
    std::swap(xtmp, *xtplus);
    const VectorX<T>& xthalf = xtmp;
    // set jacobian isn't fresh, since the previous half-step succeeded.
    this->set_jacobian_is_not_fresh();
    unused(xtplus_guess);
    success = StepVelocityImplicitEuler(
        t0 + 0.5 * h, 0.5 * h, xthalf,
        use_conservative_initial_guesses ? xthalf : xtplus_guess, xtplus,
        iteration_matrix, Jy, Jfyy, Jfyq);
    if (!success) {
      DRAKE_LOGGER_DEBUG("Second Half VIE convergence failed.");
    }
  }

  // Move statistics into half-sized-steps-specific statistics.
  num_half_vie_jacobian_reforms_ +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  num_half_vie_iter_factorizations_ +=
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  num_half_vie_function_evaluations_ +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  num_half_vie_jacobian_function_evaluations_ +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  num_half_vie_nr_iterations_ +=
      this->get_num_newton_raphson_iterations() - stored_num_nr_iterations;

  return success;
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::AttemptStepPaired(
    const T& t0, const T& h, const VectorX<T>& xn, VectorX<T>* xtplus_vie,
    VectorX<T>* xtplus_hvie) {
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator::AttemptStepPaired(h={}, "
      "t={})",
      h, t0);
  using std::abs;
  DRAKE_ASSERT(xtplus_vie != nullptr);
  DRAKE_ASSERT(xtplus_hvie != nullptr);

  // Use the current state as the candidate value for the next state.
  // [Hairer 1996] validates this choice (p. 120).
  const VectorX<T>& xtplus_guess = xn;

  // Do the large Velocity-Implicit Euler step.
  if (!StepVelocityImplicitEuler(t0, h, xn, xtplus_guess, xtplus_vie,
                                 &iteration_matrix_vie_, &Jy_vie_, &Jfyy_vie_,
                                 &Jfyq_vie_)) {
    DRAKE_LOGGER_DEBUG(
        "Velocity-Implicit Euler full-step approach did not converge for "
        "step size {}",
        h);
    return false;
  }

  // Do the half Velocity-Implicit Euler steps. We reuse the Jacobian and
  // iteration Matrices from the big step because they work quite well,
  // based on a few empirical tests.
  if (!StepHalfVelocityImplicitEulers(t0, h, xn, *xtplus_vie, xtplus_hvie,
                                      &iteration_matrix_vie_, &Jy_vie_,
                                      &Jfyy_vie_, &Jfyq_vie_)) {
    DRAKE_LOGGER_DEBUG(
        "Velocity-Implicit Euler half-step approach failed with a step size "
        "that succeeded for the full step, {}",
        h);
    return false;
  }

  return true;
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  DRAKE_LOGGER_DEBUG(
      "VelocityImplicitEulerIntegrator::"
      "DoImplicitIntegratorStep(h={}) t={}",
      h, t0);

  xn_ = context->get_continuous_state().CopyToVector();
  xtplus_vie_.resize(xn_.size());
  xtplus_hvie_.resize(xn_.size());
  int nq = context->get_continuous_state().num_q();
  if (qdot_ == nullptr || qdot_->size() != nq) {
    qdot_ = std::make_unique<BasicVector<T>>(nq);
  }

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_DEBUG(
        "-- requested step too small, taking explicit step instead, at t={}, "
        "h={}, minimum_h={}",
        t0, h, this->get_working_minimum_step_size());

    // The error estimation process for explicit Euler uses two half-sized
    // steps so that the order of the asymptotic term matches that used
    // for estimating the error of the velocity-implicit Euler integrator.

    // Compute the explicit Euler step.
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_vie_ = xn_ + h * xdot_;

    // Compute the two explicit Euler steps
    xtplus_hvie_ = xn_ + 0.5 * h * xdot_;
    context->SetTimeAndContinuousState(t0 + 0.5 * h, xtplus_hvie_);
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_hvie_ += 0.5 * h * xdot_;
  } else {
    // Try taking the requested step.
    const bool success =
        AttemptStepPaired(t0, h, xn_, &xtplus_vie_, &xtplus_hvie_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      DRAKE_LOGGER_DEBUG(
          "Velocity-Implicit Euler paired approach did not converge for "
          "time t={}, step size h={}",
          t0, h);
      context->SetTimeAndContinuousState(t0, xn_);
      return false;
    }
  }

  // Compute and update the error estimate. IntegratorBase will use the norm of
  // this vector to adjust step size.
  err_est_vec_ = (xtplus_vie_ - xtplus_hvie_);

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().SetFromVector(
      err_est_vec_);

  // Set the state to the computed state from the half-steps.
  context->SetTimeAndContinuousState(t0 + h, xtplus_hvie_);

  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::VelocityImplicitEulerIntegrator)
