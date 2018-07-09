#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"

#include "systems/primitives/vector_aggregator.h"

namespace py = pybind11;

namespace dairlib {
namespace systems {

PYBIND11_MODULE(vector_aggregator, m) {
  m.doc() = "System for aggregating vectors for LCM log playback";

  py::module::import("pydrake.systems.framework");

  py::class_<VectorAggregator>(m, "VectorAggregator")
    .def(py::init<int>(),
       py::arg("vector_length"))
  .def("get_received_vectors", &VectorAggregator::get_received_vectors)
  .def("get_received_timestamps", &VectorAggregator::get_received_timestamps)
  .def("BuildTimestampVector", &VectorAggregator::BuildTimestampVector)
  .def("BuildMatrixFromVectors", &VectorAggregator::BuildMatrixFromVectors);
}

}  // namespace systems
}  // namespace dairlib
