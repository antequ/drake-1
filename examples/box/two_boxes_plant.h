
#pragma once
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/gain.h"
#include "drake/examples/box/box_plant.h"
#include "drake/examples/box/spring_plant.h"
#include <experimental/memory>

namespace drake{
namespace examples{
namespace box{


/// A model of a system with two identical boxes. The outputs
///   are the states of box 1 and box 2, and the input is the force
///   on box 1.
/// @f[ u_2 = -k (q2 - q1) @f]
///
/// @system{SpringPlant,
///    @input_port{q1, q-dot1},
///    @input_port{q2, q-dot2},
///    @output_port{u2}
/// }
///
/// @params: box mass (m), box viscosity (d), box length (l),
///             spring stiffness (sk), damping (sd),
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class TwoBoxesPlant final : public systems::Diagram<T> {
    public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoBoxesPlant);

    /// Constructs a default plant.
    TwoBoxesPlant();
    /// constructs a plant with box mass (m), box visc (d), box length (l),
    ///               spring stiffness (sk), spring damping (sd)
    TwoBoxesPlant(T m, T d, T l, T sk, T sd);

    /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
    template <typename U>
    explicit TwoBoxesPlant(const TwoBoxesPlant<U>&) : TwoBoxesPlant() {};

    ~TwoBoxesPlant() final = default;

    /// pass in length-4 vec
    void SetInitialStates(systems::Context<T>* context,
                            const drake::VectorX<T>& initState);

    /// pass in length-2 vec for the states
    void SetBox1State(systems::Context<T>* context, const drake::VectorX<T>& state);
    void SetBox2State(systems::Context<T>* context, const drake::VectorX<T>& state);
    
    drake::VectorX<T> GetState(const systems::Context<T>& context) const;
    drake::VectorX<T> GetBox1State(const systems::Context<T>& context) const;
    drake::VectorX<T> GetBox2State(const systems::Context<T>& context) const;
    T GetBox1Position(const systems::Context<T>& context) const;
    T GetBox1Velocity(const systems::Context<T>& context) const;
    T GetBox2Position(const systems::Context<T>& context) const;
    T GetBox2Velocity(const systems::Context<T>& context) const;

    BoxPlant<T>&    Box1()   { return *pBox1; }
    BoxPlant<T>&    Box2()   { return *pBox2; }
    SpringPlant<T>& Spring() { return *pSpring; }

    const BoxPlant<T>&    Box1()   const { return *pBox1; }
    const BoxPlant<T>&    Box2()   const { return *pBox2; }
    const SpringPlant<T>& Spring() const { return *pSpring; }

    T CalcBoxesTotalEnergy(const systems::Context<T>& context) const;

    const systems::OutputPort<T>& get_box1_output() const {
        DRAKE_DEMAND(box1output >= 0);
        return this->get_output_port(box1output);
    }
    const systems::OutputPort<T>& get_box2_output() const {
        DRAKE_DEMAND(box2output >= 0);
        return this->get_output_port(box2output);
    }

    private:

    systems::Context<T>& GetMutableBox1Context(systems::Context<T>* context);
    systems::Context<T>& GetMutableBox2Context(systems::Context<T>* context);
    const systems::Context<T>& GetBox1Context(const systems::Context<T>& context) const;
    const systems::Context<T>& GetBox2Context(const systems::Context<T>& context) const;

    /* these are observer pointers; the diagram (parent) owns all of these */
    BoxPlant<T>*       pBox1;
    BoxPlant<T>*       pBox2;
    SpringPlant<T>*    pSpring;
    systems::Adder<T>* pAdder1;
    systems::Adder<T>* pAdder2;
    systems::Gain<T>*  pNegater;
    int box1output {-1};
    int box2output {-1};
};

void AddGeometryToBuilder(systems::DiagramBuilder<double>* builder, 
                          const TwoBoxesPlant<double>& plant, 
                          geometry::SceneGraph<double>* scene_graph);

} // namespace box
} // namespace examples
} // namespace drake
