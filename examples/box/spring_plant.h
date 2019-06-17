#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace examples {
namespace box {

/// A model of the spring interaction force between
///   two boxes. The output is the force to be applied
///   on box 2; box one should receive the opposite.
/// Box 1's state is passed into the normal input port.
/// Box 2's state is passed into second_box_input.
/// @f[ u_2 = -k (q2 - q1) @f]
///
/// @system{SpringPlant,
///    @input_port{q1, q-dot1},
///    @input_port{q2, q-dot2},
///    @output_port{u2}
/// }
///
/// @params: stiffness (k), damping (d), rest length (l),
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class SpringPlant final : public systems::VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringPlant);

  /// Constructs a default plant.
  SpringPlant();

  /// constructs a plant with stiffness (k), damping (d), rest length (l),
  SpringPlant(T k, T d, T l);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SpringPlant(const SpringPlant<U>&);

  ~SpringPlant() final;

  /// Returns the port to output force.
  const systems::OutputPort<T>& get_force_output_port() const
  {
    return this->get_output_port();
  }
  const systems::InputPort<T>& get_first_box_input_port() const;
  const systems::InputPort<T>& get_second_box_input_port() const;
  

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_u2(const systems::Context<T>& context) const
  {
    return this->get_force_output_port().Eval(context)(0);
  }

  const Eigen::VectorBlock< const VectorX<T> > get_parameters(
      const systems::Context<T>& context) const {
    auto& param_vector = this->GetNumericParameter(context, 0);
    return param_vector.get_value();
  }

 private:
  void DoCalcVectorOutput(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const final;
  void DoValidateAllocatedLeafContext(
      const systems::LeafContext<T>& context) const override {
    // N.B. The DRAKE_THROW_UNLESS conditions can be triggered by subclass
    // mistakes, so are part of our unit tests.  The DRAKE_DEMAND conditions
    // should be invariants guaranteed by the framework, so are asserted.

    // Exactly one input and output.
    DRAKE_THROW_UNLESS(this->num_input_ports() == 2);
    DRAKE_THROW_UNLESS(this->num_output_ports() <= 1);

    // At most one of either continuous or discrete state.
    DRAKE_THROW_UNLESS(context.num_abstract_states() == 0);
    const int continuous_size = context.num_continuous_states();
    const int num_discrete_groups = context.num_discrete_state_groups();
    DRAKE_DEMAND(continuous_size >= 0);
    DRAKE_DEMAND(num_discrete_groups >= 0);
    DRAKE_THROW_UNLESS(num_discrete_groups <= 1);
    DRAKE_THROW_UNLESS((continuous_size == 0) || (num_discrete_groups == 0));
  } 
  T k_ = {1.0}; /* stiffness */
  T d_ = {0.0}; /* damping */
  T l_ = {1.0}; /* rest length */
};

}  // namespace box
}  // namespace examples
}  // namespace drake
