#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

namespace internal {
#ifndef DRAKE_DOXYGEN_CXX
__attribute__((noreturn)) inline void EmitNoErrorEstimatorStatAndMessage() {
  throw std::logic_error(
      "No error estimator is currently implemented, so "
      "query error estimator statistics is not yet supported.");
}
#endif
}  // namespace internal

/**
 * A first-order, fully implicit integrator optimized for second-order systems.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * The velocity-implicit Euler integrator implements first-order implicit Euler
 * by alternating between steps from Newton's method for solving the
 * generalized velocity and miscellaneous states with fixed-point iterations
 * for solving the position states.
 *
 * This integrator requires a system of ordinary differential equations in
 * state x = (q,v,z) to be expressible in the following form:
 *     q̇ = N(q) v;                          (1)
 *     ẏ = fᵥ(t,q,y),                       (2)
 * where q̇ and v are linearly related via the matrix N that is dependent only
 * on the position q, y = (v,z), and fᵥ is a function that can depend on time
 * and state.
 *
 * Implicit Euler uses the following update rule at time step n:
 *     qⁿ⁺¹ = qⁿ + h N(qⁿ⁺¹) vⁿ⁺¹;          (3)
 *     yⁿ⁺¹ = yⁿ + h f(tⁿ⁺¹,qⁿ⁺¹,yⁿ⁺¹).     (4)
 *
 * To solve the nonlinear system, the velocity-implicit Euler integrator
 * iteratively solves for (qⁿ⁺¹,yⁿ⁺¹) with Newton's method: At iteration k,
 * find (qₖ₊₁,yₖ₊₁) that satisfies
 *     yₖ₊₁ = yⁿ + h f(tⁿ⁺¹,qₖ₊₁,yₖ₊₁);       (5)
 *     qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁.              (6)
 *
 * In this notation, the n's index timesteps, while the k's index the specific
 * Newton-Raphson iterations within each time step.
 *
 * To solve (5-6), first define
 *     l(y) = f(tⁿ⁺¹,qⁿ + h N(qₖ) v,y),      (7)
 *     Jₗ(y) = ∂l(y) / ∂y.                   (8)
 *
 * Next, solve the following linear equation for Δy:
 *     (I - h Jₗ) Δy = - R(yₖ),               (9)
 * where R(y) = y - yⁿ - h l(y) and Δy = yₖ₊₁ - yₖ. We then directly use yₖ₊₁
 * in (6) to get qₖ₊₁.
 *
 * This implementation uses Newton-Raphson (NR) and relies upon the obvious
 * convergence to a solution for y in `R(y) = 0` where
 * `R(y) = y - yⁿ - h l(y)` as `h` becomes sufficiently small.
 * General implementational details were gleaned from [Hairer, 1996].
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
 *
 * @note This integrator uses the integrator accuracy setting, even when run
 *       in fixed-step mode, to limit the error in the underlying Newton-Raphson
 *       process. See IntegratorBase::set_target_accuracy() for more info.
 * @see ImplicitIntegrator class documentation for information about implicit
 *      integration methods in general.
 * @see ImplicitEulerIntegrator class documentation for information about
 *      the "implicit Euler" integration method.
 */
template <class T>
class VelocityImplicitEulerIntegrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityImplicitEulerIntegrator)

  ~VelocityImplicitEulerIntegrator() override = default;

  explicit VelocityImplicitEulerIntegrator(const System<T>& system,
                                           Context<T>* context = nullptr)
      : ImplicitIntegrator<T>(system, context) {}

  // The integrator does not support error estimation.
  bool supports_error_estimation() const final { return false; }
  int get_error_estimate_order() const final { return 0; }

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    internal::EmitNoErrorEstimatorStatAndMessage();
  }

  void DoResetImplicitIntegratorStatistics() final;
  static void ComputeAndFactorImplicitEulerIterationMatrix(
      const MatrixX<T>& J, const T& h,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);

  void DoInitialize() final;

  bool DoImplicitIntegratorStep(const T& h) final;

  // Steps the system forward by a single step of at most h using the
  // Velocity-Implicit Euler method.
  // @param t0 the time at the left end of the integration interval.
  // @param h the maximum time increment to step forward.
  // @param xt0, xⁿ, the continuous state at t0.
  // @param xtplus_guess the starting guess for xⁿ⁺¹.
  // @param [out] xtplus the computed value for xⁿ⁺¹ on successful return.
  // @param [in, out] iteration_matrix the cached iteration matrix, which is
  //        only updated if the Newton-Raphson fails to converge on the first
  //        try.
  // @param [in, out] Jv the cached velocity Jacobian, which is only updated if
  //        the Newton-Raphson fails to converge on the second try.
  // @param trial the attempt for this approach (1-4). StepImplicitEuler() uses
  //        more computationally expensive methods as the trial numbers
  //        increase.
  // @returns `true` if the step of size `h` was successful, `false` otherwise.
  // @note The time and continuous state in the context are indeterminate upon
  //       exit.
  bool StepImplicitEuler(
      const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv, int trial = 1);

  // Velocity Jacobians for implicit Euler.
  MatrixX<T>& get_mutable_velocity_jacobian_implicit_euler() { return Jv_ie_; }

  // Compute the partial derivatives of the ordinary differential equations with
  // respect to the y variables of a given x(t). In particular, we compute the
  // Jacobian, Jₗ(y), of the function l(y), used in this integrator's
  // residual computation, with respect to y, where y = (v,z) and x = (q,v,z).
  // This Jacobian is then defined as:
  //     l(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (7)
  //     Jₗ(y) = ∂l(y)/∂y                      (8)
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y)
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y)
  // @param x is (qₖ, y), the continuous state around which to evaluate Jₗ(y)
  // @param qt0 refers to qⁿ, the initial position used in l(y)
  // @param [out] Jv is the Jacobian matrix, Jₗ(y).
  // @post the context's time and continuous state will be temporarily set
  //       during this call (and then reset to their original values) on
  //       return.
  void CalcVelocityJacobian(const T& tf, const T& h, const VectorX<T>& xtplus,
                            const VectorX<T>& qt0, MatrixX<T>* Jv);

  // Uses first-order forward differencing to compute the Jacobian, Jₗ(y), of
  // the function l(y), used in this integrator's residual computation, with
  // respect to y, where y = (v,z). This Jacobian is then defined as:
  //     l(y)  = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y)    (7)
  //     Jₗ(y) = ∂l(y)/∂y                      (8)
  //
  // In this method, we compute the Jacobian Jₗ(y) using a first-order forward
  // difference (i.e. numerical differentiation),
  //     Jₗ(y)ᵢⱼ = (l(y')ᵢ - l(y)ᵢ )/ δy(j),
  // where y' = y + δy(j) eⱼ, δy(j) = (√ε) max(1,|yⱼ|), and eⱼ is the j-th
  // standard Cartesian basis vector.
  // In the code we hereby refer to y as "the baseline" and y' as "prime".
  // @param t refers to tⁿ⁺¹, the time used in the definition of l(y)
  // @param h is the timestep size parameter, h, used in the definition of
  //        l(y)
  // @param x is (qₖ, y), the continuous state around which to evaluate Jₗ(y)
  // @param qt0 refers to qⁿ, the initial position used in l(y)
  // @param context the Context of the system, at time and continuous state
  //        unknown.
  // @param [out] Jv is the Jacobian matrix, Jₗ(y).
  // @note The continuous state will be indeterminate on return.
  void ComputeForwardDiffVelocityJacobian(const T& t, const T& h,
                                          const VectorX<T>& xt,
                                          const VectorX<T>& qt0,
                                          Context<T>* context, MatrixX<T>* Jv);

  // Computes necessary matrices (Jacobian and iteration matrix) for
  // Newton-Raphson (NR) iterations, as necessary. This method is based off of
  // ImplicitIntegrator<T>::MaybeFreshenMatrices. We implement our own version
  // here to use a specialized velocity Jacobian. The aformentioned method was
  // designed for use in DoImplicitIntegratorStep() processes that follow this
  // model:
  // 1. DoImplicitIntegratorStep(h) is called;
  // 2. One or more NR iterations is performed until either (a) convergence is
  //    identified, (b) the iteration is found to diverge, or (c) too many
  //    iterations were taken. In the case of (a), DoImplicitIntegratorStep(h)
  //    will return success. Otherwise, the Newton-Raphson process is attempted
  //    again with (i) a recomputed and refactored iteration matrix and (ii) a
  //    recomputed Jacobian and a recomputed an refactored iteration matrix, in
  //    that order. The process stage of that NR algorithm is indicated by the
  //    `trial` parameter below. In this model, DoImplicitIntegratorStep()
  //    returns failure if the NR iterations reach a fourth trial.
  //
  // We provide our own method to execute the same logic, but with the
  // following differences:
  // 1. We use the velocity Jacobian instead of the full Jacobian.
  // 2. We no longer use the get_reuse() logic to reuse a Jacobian
  //    when the time-step size (h) shrinks, because the velocity Jacobian
  //    depends on h.
  // These changes allow the velocity-implicit Euler method to use the smaller
  // velocity Jacobian in its Newton solves.
  //
  // @param t the time at which to compute the Jacobian.
  // @param xt the continuous state at which the Jacobian is computed.
  // @param qt0 the generalized position at the beginning of the step
  // @param h the integration step size
  // @param trial which trial (1-4) the Newton-Raphson process is in when
  //        calling this method.
  // @param compute_and_factor_iteration_matrix a function pointer for
  //        computing and factoring the iteration matrix.
  // @param [out] iteration_matrix the updated and factored iteration matrix on
  //            return.
  // @param [out] Jv the updated and factored velocity Jacobian matrix on
  //             return.
  // @returns `false` if the calling stepping method should indicate failure;
  //          `true` otherwise.
  // @pre 1 <= `trial` <= 4.
  // @post the state in the internal context may or may not be altered on
  //       return; if altered, it will be set to (t, xt).
  bool MaybeFreshenVelocityMatrices(
      const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
      int trial,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv);

  // Computes necessary matrices (Jacobian and iteration matrix) for full
  // Newton-Raphson (NR) iterations, if full Newton-Raphson method is activated
  // (if it's not activated, this method is a no-op).
  // @param t the time at which to compute the Jacobian.
  // @param xt the continuous state at which the Jacobian is computed.
  // @param h the integration step size (for computing iteration matrices).
  // @param compute_and_factor_iteration_matrix a function pointer for
  //        computing and factoring the iteration matrix.
  // @param[out] iteration_matrix the updated and factored iteration matrix on
  //             return.
  // @post the state in the internal context will be set to (t, xt) and this
  //       will store the updated Jacobian matrix, on return.
  void FreshenVelocityMatricesIfFullNewton(
      const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
      const std::function<
          void(const MatrixX<T>& J, const T& h,
               typename ImplicitIntegrator<T>::IterationMatrix*)>&
          compute_and_factor_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      MatrixX<T>* Jv);

  // This helper method evaluates the Newton-Raphson residual R(y), defined as
  // the following:
  //     R(y)  = y - yⁿ - h l(y),
  //     l(y) = f(tⁿ⁺¹, qⁿ + h N(qₖ) v, y),    (7)
  //  with tⁿ⁺¹, qₖ, y derived from the context and qⁿ, yⁿ, h passed in.
  // @param qt0 is qⁿ, the generalized position at the beginning of the step
  // @param yt0 is yⁿ, the generalized velocity and miscellaneous states at the
  //        beginning of the step
  // @param h is the step size
  // @param [out] result is set to R(y)
  // @post The position of the context is first altered and then restored to
  //       the original position. This might invalidate some caches that depend
  //       on the position.
  VectorX<T> ComputeResidualR(const VectorX<T>& qt0, const VectorX<T>& yt0,
                              const T& h);

  // The last computed iteration matrix and factorization; the _ie_ is for
  // the large step and the _hie_ is for the small step
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_ie_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Variables to avoid heap allocations.
  VectorX<T> xt0_, xdot_, xtplus_ie_;

  // Various statistics.
  int64_t num_nr_iterations_{0};

  // The last computed velocity+misc Jacobian matrices.
  MatrixX<T> Jv_ie_;
};

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::DoInitialize() {
  using std::isnan;

  // Allocate storage for changes to state variables during Newton-Raphson.
  dx_state_ = this->get_system().AllocateTimeDerivatives();

  // Verify that the maximum step size has been set.
  if (isnan(this->get_maximum_step_size()))
    throw std::logic_error("Maximum step size has not been set!");

  // Reset the Jacobian matrix (so that recomputation is forced).
  this->get_mutable_velocity_jacobian_implicit_euler().resize(0, 0);

  this->set_accuracy_in_use(1e-6);
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::
    ComputeAndFactorImplicitEulerIterationMatrix(
        const MatrixX<T>& J, const T& h,
        typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix) {
  const int n = J.rows();
  // TODO(edrumwri) Investigate using a move-type operation below.
  // We form the iteration matrix in this particular way to avoid an O(n^2)
  // subtraction as would be the case with:
  // MatrixX<T>::Identity(n, n) - J * h.
  iteration_matrix->SetAndFactorIterationMatrix(J * -h +
                                                MatrixX<T>::Identity(n, n));
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::ComputeForwardDiffVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& x, const VectorX<T>& qt0,
    Context<T>* context, MatrixX<T>* Jv) {
  using std::abs;
  using std::max;

  // TODO(antequ): Refactor this to use drake::math::ComputeNumericalGradient()
  // once it supports all scalar types (right now it only supports double).

  // Set the finite difference tolerance, used to compute δy in the notation
  // above, to √ε, the square root of machine precision.
  const double sqrt_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of q, v, y state variables in x.
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const int nq = cstate.num_q();
  const int nv = cstate.num_v();
  const int ny = nv + cstate.num_z();

  DRAKE_LOGGER_TRACE(
      "  ImplicitEulerIntegrator Compute ForwardDiffVelocityJacobian "
      "{}-Jacobian t={}",
      ny, t);
  DRAKE_LOGGER_TRACE("  computing from state {}", x.transpose());

  // Initialize the Jacobian.
  Jv->resize(ny, ny);

  // qk, v, y refer to the baseline state from x.
  const Eigen::VectorBlock<const VectorX<T>> qk = x.head(nq);
  const Eigen::VectorBlock<const VectorX<T>> v = x.segment(nq, nv);
  const Eigen::VectorBlock<const VectorX<T>> y = x.tail(ny);

  // Evaluate qt0_plus_hNv = qⁿ + h N(qₖ) v with (qₖ,v) from x.
  VectorX<T> qt0_plus_hNv(nq);
  BasicVector<T> qdot(nq);
  context->SetTimeAndContinuousState(t, x);
  this->get_system().MapVelocityToQDot(*context, v, &qdot);
  qt0_plus_hNv = qt0 + h * qdot.get_value();

  // Compute the Jacobian.

  // Define x' = (qⁿ + h N(qₖ) v, y), to compute the baseline l(y), and then
  // reuse x' = (qⁿ + h N(qₖ) v', y') to compute each prime l(y').
  VectorX<T> x_prime = x;
  Eigen::VectorBlock<VectorX<T>> q_prime = x_prime.head(nq);
  Eigen::VectorBlock<VectorX<T>> v_prime = x_prime.segment(nq, nv);
  Eigen::VectorBlock<VectorX<T>> y_prime = x_prime.tail(ny);
  q_prime = qt0_plus_hNv;

  // Initialize the finite-difference baseline, l(y), by evaluating the
  // context at l(y) = f(t, qⁿ + h N(qₖ) v, y) = f(t,x).
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q_prime);
  const VectorX<T> l_of_y = this->EvalTimeDerivatives(*context).CopyToVector();

  // Now evaluate each l(y') where y' modifies one value of y at a time
  for (int j = 0; j < ny; ++j) {
    // Compute a good increment, δy(j), to dimension j of y using approximately
    // log(1/√ε) digits of precision. Note that if |yⱼ| is large, the increment
    // will be large as well. If |yⱼ| is small, the increment will be no smaller
    // than √ε.
    const T abs_yj = abs(y(j));
    T dyj = sqrt_eps * max(T(1), abs_yj);

    // Update y', minimizing the effect of roundoff error by ensuring that
    // y and y' differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    y_prime(j) = y(j) + dyj;
    if (j < nv) {
      // Set q' = qⁿ + h N(qₖ) v' with v' from y'
      context->get_mutable_continuous_state()
          .get_mutable_generalized_position()
          .SetFromVector(qk);
      this->get_system().MapVelocityToQDot(*context, v_prime, &qdot);
      q_prime = qt0 + h * qdot.get_value();
    } else {
      // In this scenario, v' = v, and so q' is unchanged.
      q_prime = qt0_plus_hNv;
    }
    dyj = y_prime(j) - y(j);

    // Compute l(y') and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, x_prime);
    Jv->col(j) =
        (this->EvalTimeDerivatives(*context).CopyToVector() - l_of_y).tail(ny) /
        dyj;

    // Reset y' to y.
    y_prime(j) = y(j);
  }
}

template <class T>
void VelocityImplicitEulerIntegrator<T>::CalcVelocityJacobian(
    const T& t, const T& h, const VectorX<T>& x, const VectorX<T>& qt0,
    MatrixX<T>* Jv) {
  // Just like ImplicitIntegrator<T>::CalcJacobian, we change the context but
  // will change it back. The user should assume all caches are dirty after it
  // finishes.
  Context<T>* context = this->get_mutable_context();

  // Get the current time and state.
  const T t_current = context->get_time();
  const ContinuousState<T>& cstate = context->get_continuous_state();
  const VectorX<T> x_current = cstate.CopyToVector();

  // Update the time and state.
  context->SetTimeAndContinuousState(t, x);
  this->increment_jacobian_evaluations();

  // Get the current number of ODE evaluations.
  int64_t current_ODE_evals = this->get_num_derivative_evaluations();

  //// Get the system.
  // const System<T>& system = this->get_system();
  // forward diff the Jacobian
  switch (this->get_jacobian_computation_scheme()) {
    case VelocityImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kForwardDifference:
      ComputeForwardDiffVelocityJacobian(t, h, x, qt0, &*context, Jv);
      break;
    case VelocityImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kCentralDifference:
      throw std::runtime_error("Central difference not supported yet!");
      break;
    case VelocityImplicitEulerIntegrator<
        T>::JacobianComputationScheme::kAutomatic:
      throw std::runtime_error("AutoDiff'd Jacobian not supported yet!");
      break;
    default:
      throw new std::logic_error("Invalid Jacobian computation scheme!");
  }

  // Use the new number of ODE evaluations to determine the number of ODE
  // evaluations used in computing Jacobians.
  this->increment_jacobian_computation_derivative_evaluations(
      this->get_num_derivative_evaluations() - current_ODE_evals);

  // Reset the time and state.
  context->SetTimeAndContinuousState(t_current, x_current);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::MaybeFreshenVelocityMatrices(
    const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
    int trial,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv) {
  DRAKE_DEMAND(Jv != nullptr);
  DRAKE_DEMAND(iteration_matrix != nullptr);
  // Compute the initial Jacobian and iteration matrices and factor them, if
  // necessary.

  if (!this->get_reuse() || Jv->rows() == 0 || this->IsBadJacobian(*Jv)) {
    CalcVelocityJacobian(t, h, xt, qt0, Jv);
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
    return true;  // Indicate success.
  }

  // Reuse is activated, Jacobian is fully sized, and Jacobian is not "bad".
  // If the iteration matrix has not been set and factored, do only that.
  if (!iteration_matrix->matrix_factored()) {
    this->increment_num_iter_factorizations();
    compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
    return true;  // Indicate success.
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing: this will cause the Newton-Raphson
      // process to use the last computed (and already factored) iteration
      // matrix.
      return true;  // Indicate success.

    case 2: {
      // For the second trial, we perform the (likely) next least expensive
      // operation, re-constructing and factoring the iteration matrix.
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
      return true;
    }

    case 3: {
      // For the third trial, we reform the Jacobian matrix and refactor the
      // iteration matrix.

      // note: Based on a few simple experimental tests, we found that the
      // optimization when matrices are already fresh in
      // ImplicitIntegrator<T>::MaybeFreshenMatrices does not significantly help
      // here, especially because our Jacobian depends on step size h.
      CalcVelocityJacobian(t, h, xt, qt0, Jv);
      this->increment_num_iter_factorizations();
      compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
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
    const T& t, const VectorX<T>& xt, const VectorX<T>& qt0, const T& h,
    const std::function<void(const MatrixX<T>&, const T&,
                             typename ImplicitIntegrator<T>::IterationMatrix*)>&
        compute_and_factor_iteration_matrix,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv) {
  DRAKE_DEMAND(iteration_matrix != nullptr);
  DRAKE_DEMAND(Jv != nullptr);

  // Return immediately if full-Newton is not in use.
  if (!this->get_use_full_newton()) return;

  // Compute the initial Jacobian and iteration matrices and factor them.
  CalcVelocityJacobian(t, h, xt, qt0, Jv);
  this->increment_num_iter_factorizations();
  compute_and_factor_iteration_matrix(*Jv, h, iteration_matrix);
}

template <class T>
VectorX<T> VelocityImplicitEulerIntegrator<T>::ComputeResidualR(
    const VectorX<T>& qt0, const VectorX<T>& yt0, const T& h) {
  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  // Save the initial qk, yk to reset at the end of this method.
  const VectorX<T> xk = cstate.CopyToVector();
  int nq = qt0.rows();
  int ny = yt0.rows();

  const Eigen::VectorBlock<const VectorX<T>> qk = xk.head(nq);
  const Eigen::VectorBlock<const VectorX<T>> yk = xk.tail(ny);

  // Suppose context has state (qk, v, z).
  // Compute q = qt0 + h N(qk) v
  BasicVector<T> qdot(nq);
  this->get_system().MapVelocityToQDot(
      *context, cstate.get_generalized_velocity(), &qdot);
  const VectorX<T> q = qt0 + h * qdot.get_value();

  // Evaluate l = f(q, v, z)
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(q);
  const ContinuousState<T>& xc_deriv = this->EvalTimeDerivatives(*context);
  const VectorX<T> l_of_y = xc_deriv.CopyToVector().tail(ny);

  // reset the context back.
  context->get_mutable_continuous_state()
      .get_mutable_generalized_position()
      .SetFromVector(qk);
  return (yk - yt0 - h * l_of_y).eval();
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::StepImplicitEuler(
    const T& t0, const T& h, const VectorX<T>& xt0,
    const VectorX<T>& xtplus_guess, VectorX<T>* xtplus,
    typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
    MatrixX<T>* Jv, int trial) {
  using std::abs;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);
  DRAKE_LOGGER_TRACE("StepImplicitEuler(h={}) t={}", h, t0);

  const System<T>& system = this->get_system();
  // Verify xtplus
  DRAKE_ASSERT(xtplus != nullptr && xtplus->size() == xt0.size() &&
               xtplus_guess.size() == xt0.size());

  // Initialize xtplus to the guess
  *xtplus = xtplus_guess;

  Context<T>* context = this->get_mutable_context();
  const systems::ContinuousState<T>& cstate = context->get_continuous_state();
  int nq = cstate.num_q();
  int nv = cstate.num_v();
  int nz = cstate.num_z();
  const Eigen::VectorBlock<const VectorX<T>> qt0 = xt0.head(nq);
  const Eigen::VectorBlock<const VectorX<T>> yt0 = xt0.tail(nv + nz);

  // Define references to q, y, v, and z portions of xtplus for readibility.
  Eigen::VectorBlock<VectorX<T>> qtplus = xtplus->head(nq);
  Eigen::VectorBlock<VectorX<T>> ytplus = xtplus->tail(nv + nz);
  const Eigen::VectorBlock<VectorX<T>> vtplus = xtplus->segment(nq, nv);
  const Eigen::VectorBlock<VectorX<T>> ztplus = xtplus->tail(nz);
  unused(ztplus);

  // Set last_qtplus to qk. This will be used in computing dx to determine
  // convergence.
  VectorX<T> last_qtplus = qtplus;

  // Initialize the vector for qdot.
  BasicVector<T> qdot(nq);

  // Advance the context time and state to compute derivatives at t0 + h.
  const T tf = t0 + h;
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Declare the state update vector and initialize the current and last state
  // update norms; these will be used to detect convergence.
  VectorX<T> dx(xt0.size());
  T last_dx_norm = std::numeric_limits<double>::infinity();
  T dx_norm = std::numeric_limits<double>::infinity();

  // Calculate Jacobian and iteration matrices (and factorizations), as needed,
  // around (t0, xt0). We do not do this calculation if full Newton is in use;
  // the calculation will be performed at the beginning of the loop instead.
  if (!this->get_use_full_newton() &&
      !this->MaybeFreshenVelocityMatrices(
          t0, xt0, qt0, h, trial, ComputeAndFactorImplicitEulerIterationMatrix,
          iteration_matrix, Jv)) {
    return false;
  }

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < this->max_newton_raphson_iterations(); ++i) {
    DRAKE_LOGGER_TRACE("Newton-Raphson iteration {}", i);

    // Check for convergence.
    typename ImplicitIntegrator<T>::ConvergenceStatus status =
        this->CheckNewtonConvergence(i, *xtplus, dx, dx_norm, last_dx_norm);
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConverged)
      return true;  // We win.
    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kDiverged)
      break;  // Try something else.

    DRAKE_DEMAND(
        status == ImplicitIntegrator<T>::ConvergenceStatus::kNotConverged ||
        status ==
            ImplicitIntegrator<T>::ConvergenceStatus::kConvergesInOneMore);

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
    last_qtplus = qtplus;

    this->FreshenVelocityMatricesIfFullNewton(
        tf, *xtplus, qt0, h, ComputeAndFactorImplicitEulerIterationMatrix,
        iteration_matrix, Jv);

    // Update the number of Newton-Raphson iterations.
    num_nr_iterations_++;
    // Evaluate the residual error, which is defined as the following:
    //     R(y)  = y - yⁿ - h l(y),
    VectorX<T> residual = ComputeResidualR(qt0, yt0, h);
    // Compute the state update using the equation A*y = -R(), where A is the
    // iteration matrix.
    const VectorX<T> dy = iteration_matrix->Solve(-residual);

    // Update the y portion of xtplus to yₖ₊₁.
    ytplus += dy;

    // Update the q portion of xtplus to qₖ₊₁. Note that at this point, the
    // context has its position q equal to qtplus = qₖ, because
    // ComputeResidualR was the last function to modify the context. This means
    // that we can directly call MapVelocityToQDot on the context, which will
    // evaluate N(qₖ).
    system.MapVelocityToQDot(*context, vtplus, &qdot);
    qtplus = qt0 + h * qdot.get_value();
    dx << qtplus - last_qtplus, dy;

    // Get the infinity norm of the weighted update vector.
    dx_state_->get_mutable_vector().SetFromVector(dx);

    DRAKE_LOGGER_TRACE("dx: {}", dx.transpose());

    // TODO(antequ): Replace this with CalcStateChangeNorm() when error
    // control has been implemented.
    // Get the norm of the update vector.
    dx_norm = dx_state_->CopyToVector().norm();

    context->SetTimeAndContinuousState(tf, *xtplus);

    if (status == ImplicitIntegrator<T>::ConvergenceStatus::kConvergesInOneMore)
      return true;  // We win.
  }

  DRAKE_LOGGER_TRACE("VIE convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try; otherwise, the following code will recurse
  // into this function again, and freshen computations as helpful. Note that
  // get_reuse() returns false if "full Newton-Raphson" mode is activated (see
  // ImplicitIntegrator::get_use_full_newton()).
  if (!this->get_reuse()) return false;

  // Try StepImplicitEuler again. That method will
  // freshen Jacobians and iteration matrix factorizations as necessary.
  return StepImplicitEuler(t0, h, xt0, xtplus_guess, xtplus, iteration_matrix,
                           Jv, trial + 1);
}

template <class T>
bool VelocityImplicitEulerIntegrator<T>::DoImplicitIntegratorStep(const T& h) {
  // Save the current time and state.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  DRAKE_LOGGER_TRACE("IE DoStep(h={}) t={}", h, t0);

  xt0_ = context->get_continuous_state().CopyToVector();
  xtplus_ie_.resize(xt0_.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using an explicit Euler step.
  if (h < this->get_working_minimum_step_size()) {
    DRAKE_LOGGER_TRACE(
        "-- requested step too small, taking explicit "
        "step instead");

    // Compute the explicit Euler step.
    xdot_ = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie_ = xt0_ + h * xdot_;
  } else {
    // Use the current state as the candidate value for the next state.
    // [Hairer 1996] validates this choice (p. 120).
    const VectorX<T>& xtplus_guess = xt0_;
    bool success = StepImplicitEuler(t0, h, xt0_, xtplus_guess, &xtplus_ie_,
                                     &iteration_matrix_ie_, &Jv_ie_);

    // If the step was not successful, reset the time and state.
    if (!success) {
      DRAKE_LOGGER_TRACE(
          "SO Implicit Euler approach did not converge for "
          "step size {}",
          h);
      context->SetTimeAndContinuousState(t0, xt0_);
      return false;
    }
  }

  // Set the state to the computed state.
  context->SetTimeAndContinuousState(t0 + h, xtplus_ie_);

  return true;
}

}  // namespace systems
}  // namespace drake
