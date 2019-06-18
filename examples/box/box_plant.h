#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"

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
class BoxPlant final : public systems::VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BoxPlant);

  /// Constructs a default plant.
  BoxPlant();

  /// Constructs a plant with inv mass m, length l, and damping d
  BoxPlant(T m, T l, T d);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BoxPlant(const BoxPlant<U>&);

  ~BoxPlant() final;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_state_output_port() const
  {
    return this->get_output_port();
  }

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_u(const systems::Context<T>& context) const {
    return this->get_input_port().Eval(context)(0);
  }

  T get_length() const {
    return l_;
  }

  T get_inv_mass() const {
    return m_;
  }

  Eigen::VectorBlock<const VectorX<T>> GetBoxState(const systems::Context<T>& context) const {
    return this->GetVectorState(context);
  }

  static void set_initial_state(systems::Context<T>* context,
                                const Eigen::Ref< const VectorX<T> >& z0);


 private:
  void DoCalcVectorOutput(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const final;

  void DoCalcVectorTimeDerivatives(const systems::Context< T > &context, 
      const Eigen::VectorBlock< const VectorX< T >> &input, 
      const Eigen::VectorBlock< const VectorX< T >> &state, 
      Eigen::VectorBlock< VectorX< T >> *derivatives) const final;

  T m_;
  T l_;
  T d_;
};

}  // namespace box
}  // namespace examples
}  // namespace drake
