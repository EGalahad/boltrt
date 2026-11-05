// SPDX-License-Identifier: Apache-2.0
#include "boltrt/tasks/track/register.hpp"
#include "boltrt/tasks/track/loop.hpp"
#include "boltrt/tasks/track/observation.hpp"
namespace boltrt {
void register_tracking_task(TaskRegistry &tasks) {
  tasks.add(
      "tracking",
      [](TaskServices services) -> std::unique_ptr<PolicyLoop> {
        const auto &policy = services.policy;
        const auto &args = services.arguments;
        auto reference_robot = services.robot;
        const auto config =
            policy.yaml["motion"] ? policy.yaml["motion"] : YAML::Node(YAML::NodeType::Map);
        if (config["body_names"] && config["body_names"].IsSequence())
          reference_robot.bodies = config["body_names"].as<Names>();
        if (config["mjcf_path"])
          reference_robot.model =
              resolve_policy_asset(config["mjcf_path"].as<std::string>(), policy).string();
        const auto future = reference_offsets(policy);
        const auto storage =
            config["storage_dtype"] ? config["storage_dtype"].as<std::string>() : "float16";
        if (storage != "float16" && storage != "float32")
          throw std::runtime_error("motion.storage_dtype must be float16 or float32");
        const bool smpl =
            config["motion_backend"] && config["motion_backend"].as<std::string>() == "smpl_zmq";
        if (smpl && !args.count("motion"))
          throw std::runtime_error("SMPL policies require --motion");
        const auto rate = services.options.rate;
        std::unique_ptr<MotionSource> source;
        if (!args.count("motion"))
          source = pose_motion(reference_robot, policy.default_q, future, rate);
        else if (smpl)
          source = smpl_motion(args.at("motion"), reference_robot, future, rate);
        else if (args.at("motion").rfind("tcp://", 0) == 0)
          source = zmq_motion(args.at("motion"), reference_robot, future, rate);
        else
          source = file_motion(std::filesystem::absolute(args.at("motion")), reference_robot,
                               future, rate, storage == "float16");
        services.options.task_name =
            args.count("motion") ? std::filesystem::path(args.at("motion")).filename().string()
                                 : "default pose";
        return std::make_unique<TrackingLoop>(services.robot, policy, services.backend,
                                              std::move(source), services.inference,
                                              services.observations, std::move(services.options));
      },
      [](ObservationRegistry &observations) {
        register_tracking_proprioception(observations);
        register_mimic_observations(observations);
        register_external_observations(observations);
      },
      {"motion"});
}
} // namespace boltrt
