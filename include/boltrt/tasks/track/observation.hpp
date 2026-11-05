// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "boltrt/observation/observation.hpp"
#include "boltrt/tasks/track/context.hpp"
namespace boltrt {
void register_tracking_proprioception(ObservationRegistry &);
void register_mimic_observations(ObservationRegistry &);
void register_external_observations(ObservationRegistry &);
} // namespace boltrt
