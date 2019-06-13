#include "drake/examples/box/box_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace box {

template <typename T>
BoxPlant<T>::BoxPlant()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<box::BoxPlant>{}) {
  this->DeclareVectorInputPort("u", drake::systems::BasicVector<T>(1)/* input has size 1 */);
  this->DeclareVectorOutputPort("state", drake::systems::BasicVector<T>(2),
                                &BoxPlant::CopyStateOut,
                                {this->all_state_ticket()});
  drake::systems::BasicVector<T> state (2); 
  state[0] = T(0.0); /* q */
  state[1] = T(0.0); /* q-dot */
  this->DeclareContinuousState(state, 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
  drake::systems::BasicVector<T> params (3);
  params[0] = T(1.0); /* inverse mass */
  params[1] = T(1.0); /* length */
  params[2] = T(0.0); /* velocity damping */
  this->DeclareNumericParameter(params);
}

template <typename T>
template <typename U>
BoxPlant<T>::BoxPlant(const BoxPlant<U>&) : BoxPlant() {}

template <typename T>
BoxPlant<T>::~BoxPlant() = default;

template <typename T>
const systems::InputPort<T>& BoxPlant<T>::get_input_port() const {
  DRAKE_DEMAND(systems::LeafSystem<T>::num_input_ports() == 1);
  return systems::LeafSystem<T>::get_input_port(0);
}

template <typename T>
const systems::OutputPort<T>& BoxPlant<T>::get_state_output_port() const {
  DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
  return systems::LeafSystem<T>::get_output_port(0);
}

template <typename T>
void BoxPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                 drake::systems::BasicVector<T>* output) const {
  output -> SetFrom(get_state(context));
}

template <typename T>
T BoxPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const drake::systems::BasicVector<T>& state = get_state(context);
  const drake::systems::BasicVector<T>& params = get_parameters(context);
  // Kinetic energy = 1/2 m q-dot^2
  T kinetic_energy = T(0);
  if (params[0] != 0 )
      kinetic_energy += 0.5  * pow(state[1], 2) / params[0];
  // no spring, so no potential energy
  return kinetic_energy;
}

// Compute the actual physics.
template <typename T>
void BoxPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const drake::systems::BasicVector<T>& state = get_state(context);
  const drake::systems::BasicVector<T>& params = get_parameters(context);
  drake::systems::BasicVector<T>& derivative_vector = get_mutable_state(derivatives);

  derivative_vector[0] = state[1];
  derivative_vector[1] = (get_u(context) /* force */ - params[2] * state[1] /* damping */) * params[0] /* inv mass */ ;
  std::cout << derivative_vector[1] << std::endl;
}

}  // namespace box
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::box::BoxPlant)
