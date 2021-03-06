#include "drake/bindings/pydrake/systems/framework_py_values.h"

#include <sstream>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"

using std::string;

namespace drake {
namespace pydrake {

void DefineFrameworkPyValues(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_common_scalar_types = [m, &doc](auto dummy) {
    using T = decltype(dummy);
    // Value types.
    DefineTemplateClassWithDefault<VectorBase<T>>(
        m, "VectorBase", GetPyParam<T>(), doc.VectorBase.doc)
        .def("__str__",
            [](const VectorBase<T>& vec) {
              std::ostringstream oss;
              oss << vec;
              return oss.str();
            })
        .def("CopyToVector", &VectorBase<T>::CopyToVector,
            doc.VectorBase.CopyToVector.doc)
        .def("SetAtIndex", &VectorBase<T>::SetAtIndex,
            doc.VectorBase.SetAtIndex.doc)
        .def("SetFromVector", &VectorBase<T>::SetFromVector,
            doc.VectorBase.SetFromVector.doc)
        .def("size", &VectorBase<T>::size, doc.VectorBase.size.doc);

    // TODO(eric.cousineau): Make a helper function for the Eigen::Ref<>
    // patterns.
    auto basic_vector =
        DefineTemplateClassWithDefault<BasicVector<T>, VectorBase<T>>(
            m, "BasicVector", GetPyParam<T>(), doc.BasicVector.doc);
    DefClone(&basic_vector);
    basic_vector
        // N.B. Place `init<VectorX<T>>` `init<int>` so that we do not
        // implicitly convert scalar-size `np.array` objects to `int` (since
        // this is normally permitted).
        .def(py::init<VectorX<T>>(), py::arg("data"),
            doc.BasicVector.ctor.doc_1args_vec)
        .def(py::init<int>(), py::arg("size"),
            doc.BasicVector.ctor.doc_1args_size)
        .def(
            "get_value",
            [](const BasicVector<T>* self) -> Eigen::Ref<const VectorX<T>> {
              return self->get_value();
            },
            py_reference_internal, doc.BasicVector.get_value.doc)
        // TODO(eric.cousineau): Remove this once `get_value` is changed, or
        // reference semantics are changed for custom dtypes.
        .def("_get_value_copy",
            [](const BasicVector<T>* self) -> VectorX<T> {
              return self->get_value();
            })
        .def(
            "get_mutable_value",
            [](BasicVector<T>* self) -> Eigen::Ref<VectorX<T>> {
              return self->get_mutable_value();
            },
            py_reference_internal, doc.BasicVector.get_mutable_value.doc)
        .def(
            "GetAtIndex",
            [](BasicVector<T>* self, int index) -> T& {
              return self->GetAtIndex(index);
            },
            py_reference_internal, doc.VectorBase.GetAtIndex.doc)
        .def("SetZero", &BasicVector<T>::SetZero, doc.BasicVector.SetZero.doc);

    DefineTemplateClassWithDefault<Supervector<T>, VectorBase<T>>(
        m, "Supervector", GetPyParam<T>(), doc.Supervector.doc);

    DefineTemplateClassWithDefault<Subvector<T>, VectorBase<T>>(
        m, "Subvector", GetPyParam<T>(), doc.Subvector.doc);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});

  // Add `Value<>` instantiations for basic vectors templated on common scalar
  // types.
  auto bind_abstract_basic_vectors = [m](auto dummy) {
    using T = decltype(dummy);
    auto cls = AddValueInstantiation<BasicVector<T>>(m);
    cls  // BR
        .def(
            "set_value",
            [](Value<BasicVector<T>>* self, const T& value) {
              self->set_value(BasicVector<T>{value});
            },
            py::arg("value"))
        .def(
            "set_value",
            [](Value<BasicVector<T>>* self,
                const Eigen::Ref<const Eigen::VectorXd>& value) {
              self->set_value(BasicVector<T>(value));
            },
            py::arg("value"));
  };
  type_visit(bind_abstract_basic_vectors, CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
