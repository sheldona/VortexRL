#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

// Pybind11 wrapper for the Vortex cart pole environment.
//
// The cart pole environment is implemented in C++
// using the Vortex SDK, and a concise interface is 
// defined for specific methods that are relevant for
// reinforcement learning.
//

#include "CartPoleEnvironment.h"

namespace py = pybind11;

PYBIND11_MODULE(cartPoleEnv, m) {
    py::class_<CartPoleEnvironment>(m, "CartPoleEnvironment")
        .def(py::init<const std::string&,bool>())
        .def("step", &CartPoleEnvironment::step)
        .def("reset", &CartPoleEnvironment::reset);
}
