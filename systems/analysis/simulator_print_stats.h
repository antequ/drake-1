#pragma once

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {

/// This method prints relevant simulation statistics for a simulator that
/// advanced the state of a system forward in time.
/// @param[in] simulator
///   The simulator to print statistics for.
void PrintSimulatorStatistics(const Simulator<double>& simulator);

}  // namespace systems
}  // namespace drake
