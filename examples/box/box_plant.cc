#include "drake/examples/box/box_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace box {

template <typename T>
BoxPlant<T>::BoxPlant(double m, double l, double d)
    :  systems::VectorSystem<T>(systems::SystemTypeTag<box::BoxPlant>{},1 /* input size */, 2 /* output size */),
    m_(m) /* inverse mass */, l_(l) /* length */, d_(d) /* velocity damping */
{
  this->DeclareContinuousState(drake::systems::BasicVector<T>(2) /* state size */, 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
}

template <typename T>
BoxPlant<T>::BoxPlant() : BoxPlant(1.0 /* inv mass */, 
    1.0 /* length */, 0.0 /* vel damp */) {

}

template <typename T>
template <typename U>
BoxPlant<T>::BoxPlant(const BoxPlant<U>& other) : BoxPlant(other.m_, other.l_, other.d_) {}

template <typename T>
BoxPlant<T>::~BoxPlant() = default;

template <typename T>
void BoxPlant<T>::DoCalcVectorOutput(
      const systems::Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const 
      {
        unused(input);
        unused(context);
        *output = state;
      }

template <typename T>
T BoxPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const VectorX<T>& state = this->GetVectorState(context);
  // Kinetic energy = 1/2 m q-dot^2
  T kinetic_energy = 0.;
  if (m_ != 0. )
      kinetic_energy += 0.5  * pow(state(1), 2) / m_;
  // no spring, so no potential energy
  return kinetic_energy;
}

template <typename T>
void BoxPlant<T>::set_initial_state(
    systems::Context<T>* context, const Eigen::Ref< const VectorX<T> >& z0) {
  systems::VectorBase<T>& state_vector = context->get_mutable_continuous_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(z0.rows() == state_vector.size() && z0.cols() == 1);
  state_vector.SetFromVector(z0);
}

// Compute the actual physics.
template <typename T>
void BoxPlant<T>::DoCalcVectorTimeDerivatives(const systems::Context< T > &context, 
      const Eigen::VectorBlock< const VectorX< T >> &input, 
      const Eigen::VectorBlock< const VectorX< T >> &state, 
      Eigen::VectorBlock< VectorX< T >> *derivatives) const  {
  unused(context);
  (*derivatives)[0] = state[1];
  (*derivatives)[1] = (input[0] /* force */ - d_ * state[1] /* damping */) * m_ /* inv mass */ ;
}

}  // namespace box
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::box::BoxPlant)
