#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace box {

/// A model of a simple box, where q is the center point
/// @f[ m \ddot q = u @f]
///
/// @system{BoxPlant,
///    @input_port{u},
///    @output_port{q}
/// }
///
/// @params: inverse mass (1/m), length (l), velocity damping (d)
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class BoxPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BoxPlant);

  /// Constructs a default plant.
  BoxPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BoxPlant(const BoxPlant<U>&);

  ~BoxPlant() final;

  /// Returns the input port to the externally applied force.
  const systems::InputPort<T>& get_input_port() const;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_state_output_port() const;

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_u(const systems::Context<T>& context) const {
    return this->get_input_port().Eval(context)(0);
  }

  static const systems::BasicVector<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
  }

  static const systems::BasicVector<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static systems::BasicVector<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<systems::BasicVector<T>&>(cstate->get_mutable_vector());
  }

  static systems::BasicVector<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const systems::BasicVector<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<systems::BasicVector>(context, 0);
  }

  systems::BasicVector<T>& get_mutable_parameters(
      systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<systems::BasicVector>(
        context, 0);
  }

 private:
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const final;
};

}  // namespace box
}  // namespace examples
}  // namespace drake
