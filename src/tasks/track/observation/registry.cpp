// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/observation.hpp"
#include "common.hpp"
namespace boltrt {
void register_external_observations(ObservationRegistry &r) {
  register_sonic_observations(r);
  register_heft_observations(r);
  register_twist2_observations(r);
  register_scalebfm_observations(r);
  register_teleopit_observations(r);
  register_humanoid_gpt_observations(r);
  register_holomotion_observations(r);
  register_bfm_zero_observations(r);
  register_grit_observations(r);
}
} // namespace boltrt
