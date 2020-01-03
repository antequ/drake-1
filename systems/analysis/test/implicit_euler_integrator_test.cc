#include "drake/systems/analysis/implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Tests the Jacobian and iteration matrix reuse strategies using a test
// problem and integrator for which we have knowledge of the convergence
// behavior from the initial state.
GTEST_TEST(ImplicitEulerIntegratorTest, Reuse) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the Euler integrator.
  ImplicitEulerIntegrator<double> euler(*robertson, context.get());

  euler.set_maximum_step_size(1e-2);  // Maximum step that will be attempted.
  euler.set_throw_on_minimum_step_size_violation(false);
  euler.set_fixed_step_mode(true);
  euler.set_reuse(true);    // The whole point of this.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed. There
  // should be two Jacobian matrix evaluations- once at trial 1 and another
  // at trial 3. There should be three iteration matrix factorizations: once
  // at trial 1, another at trial 2, and the third at trial 3.
  euler.Initialize();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e-2));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(), 3);
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 2);

  // Now integrate again but with a smaller size. Again, past experience
  // indicates that this step size should be sufficiently small for the
  // integrator to converge. The Jacobian matrix will be "fresh"; we assume no
  // knowledge of the number of iteration matrix factorizations.
  euler.ResetStatistics();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 0);

  // Again try taking a large step, which we expect will be too large to
  // converge. There should be one Jacobian matrix evaluation- once at trial 3.
  // There should be two iteration matrix factorizations: one at trial 2 and
  // another at trial 3.
  euler.ResetStatistics();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e-2));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(), 2);
  EXPECT_EQ(euler.get_num_jacobian_evaluations(), 1);
}

// Tests that the full-Newton approach computes a Jacobian matrix and factorizes
// the iteration matrix on every Newton-Raphson iteration.
GTEST_TEST(ImplicitEulerIntegratorTest, FullNewton) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
    std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the Euler integrator.
  ImplicitEulerIntegrator<double> euler(*robertson, context.get());

  euler.request_initial_step_size_target(1e0);
  euler.set_throw_on_minimum_step_size_violation(false);
  euler.set_fixed_step_mode(true);
  euler.set_use_full_newton(true);    // The whole point of this test.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed.
  euler.Initialize();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());

  // Now integrate again but with a smaller size. Again, past experience tells
  // us that this step size should be sufficiently small for the integrator to
  // converge.
  euler.ResetStatistics();
  ASSERT_TRUE(euler.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());

  // Again try taking a large step, which we expect will be too large to
  // converge.
  euler.ResetStatistics();
  ASSERT_FALSE(euler.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(euler.get_num_iteration_matrix_factorizations(),
            euler.get_num_newton_raphson_iterations());
  EXPECT_EQ(euler.get_num_jacobian_evaluations(),
            euler.get_num_newton_raphson_iterations());
}

// Test Euler integrator.
typedef ::testing::Types<ImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

