#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"

#include "examples/Cassie/cassie_utils.h"

namespace py = pybind11;

namespace dairlib {

PYBIND11_MODULE(cassie_utils, m) {
  m.doc() = "Utility functions related to Cassie";

  m.def("makeFixedBaseCassieTree", []() {
    auto instance = std::make_unique<RigidBodyTree<double>>();
    buildFixedBaseCassieTree(*instance.get());
    return instance;
  });
}

}  // namespace dairlib
