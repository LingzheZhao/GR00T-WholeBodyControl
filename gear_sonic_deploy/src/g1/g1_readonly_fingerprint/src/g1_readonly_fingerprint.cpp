#include "fingerprint_analysis.hpp"

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <net/if.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/dds_wrapper/common/crc.h>

namespace {

using LowState = unitree_hg::msg::dds_::LowState_;
using IMUState = unitree_hg::msg::dds_::IMUState_;

struct Options {
  std::string interface;
  int domain_id = 0;
  int duration_seconds = 3;
  std::size_t minimum_samples = 50;
};

long ParseLong(const std::string& text, const std::string& option) {
  errno = 0;
  char* end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0') {
    throw std::invalid_argument(option + " requires an integer");
  }
  return value;
}

Options ParseOptions(int argc, char** argv) {
  if (argc < 2) {
    throw std::invalid_argument("network interface is required");
  }
  Options options;
  options.interface = argv[1];
  const std::regex interface_pattern("^[A-Za-z0-9_.:-]+$");
  if (!std::regex_match(options.interface, interface_pattern)) {
    throw std::invalid_argument(
        "network interface contains unsupported characters");
  }
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::invalid_argument("network interface does not exist");
  }

  for (int index = 2; index < argc; ++index) {
    const std::string option = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument(option + " requires a value");
    }
    const std::string value = argv[++index];
    if (option == "--domain-id") {
      const long parsed = ParseLong(value, option);
      if (parsed < 0 || parsed > 232) {
        throw std::invalid_argument("--domain-id must be between 0 and 232");
      }
      options.domain_id = static_cast<int>(parsed);
    } else if (option == "--duration-seconds") {
      const long parsed = ParseLong(value, option);
      if (parsed < 1 || parsed > 30) {
        throw std::invalid_argument(
            "--duration-seconds must be between 1 and 30");
      }
      options.duration_seconds = static_cast<int>(parsed);
    } else if (option == "--min-samples") {
      const long parsed = ParseLong(value, option);
      if (parsed < 5 || parsed > 10000) {
        throw std::invalid_argument(
            "--min-samples must be between 5 and 10000");
      }
      options.minimum_samples = static_cast<std::size_t>(parsed);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  return options;
}

g1_fingerprint::Snapshot MakeSnapshot(const LowState& state) {
  g1_fingerprint::Snapshot snapshot;
  snapshot.version = state.version();
  snapshot.mode_pr = state.mode_pr();
  snapshot.mode_machine = state.mode_machine();
  snapshot.tick = state.tick();
  snapshot.imu_temperature = state.imu_state().temperature();
  for (std::size_t index = 0; index < g1_fingerprint::kMotorSlots; ++index) {
    const auto& motor = state.motor_state()[index];
    snapshot.motor_modes[index] = motor.mode();
    snapshot.motor_status[index] = motor.motorstate();
    snapshot.motor_sensors[index] = motor.sensor();
    snapshot.motor_temperatures[index] = motor.temperature();
  }
  return snapshot;
}

std::string ErrorJson(const std::string& error) {
  return std::string{"{\"schema\":\"motionlcm.g1.readonly-fingerprint.v1\""}
      + ",\"status\":\"error\",\"receive_only\":true"
      + ",\"deployment_authorized\":false,\"profile_inference\":null"
      + ",\"issues\":[\"runtime_error\"],\"error\":"
      + g1_fingerprint::JsonString(error) + '}';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    std::mutex mutex;
    std::vector<g1_fingerprint::Snapshot> samples;
    std::size_t invalid_crc_samples = 0;
    std::size_t secondary_imu_samples = 0;
    std::array<float, 29> latest_q{};
    std::array<float, 29> latest_dq{};

    unitree::robot::ChannelFactory::Instance()->Init(options.domain_id,
                                                      options.interface);
    auto receiver = std::make_shared<
        unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    receiver->InitChannel(
        [&](const void* message) {
          LowState state = *static_cast<const LowState*>(message);
          const auto received_crc = state.crc();
          const auto calculated_crc =
              crc32_core(reinterpret_cast<std::uint32_t*>(&state),
                         (sizeof(LowState) >> 2) - 1);
          std::lock_guard<std::mutex> lock(mutex);
          if (received_crc != calculated_crc) {
            ++invalid_crc_samples;
            return;
          }
          for (std::size_t index = 0; index < latest_q.size(); ++index) {
            latest_q[index] = state.motor_state()[index].q();
            latest_dq[index] = state.motor_state()[index].dq();
          }
          samples.push_back(MakeSnapshot(state));
        },
        64);
    auto secondary_imu_receiver = std::make_shared<
        unitree::robot::ChannelSubscriber<IMUState>>("rt/secondary_imu");
    secondary_imu_receiver->InitChannel(
        [&](const void*) {
          std::lock_guard<std::mutex> lock(mutex);
          ++secondary_imu_samples;
        },
        64);

    const auto started = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(options.duration_seconds));
    receiver->CloseChannel();
    secondary_imu_receiver->CloseChannel();
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();

    std::vector<g1_fingerprint::Snapshot> captured;
    std::size_t captured_invalid_crc = 0;
    std::size_t captured_secondary_imu = 0;
    std::array<float, 29> captured_q{};
    std::array<float, 29> captured_dq{};
    {
      std::lock_guard<std::mutex> lock(mutex);
      captured = samples;
      captured_invalid_crc = invalid_crc_samples;
      captured_secondary_imu = secondary_imu_samples;
      captured_q = latest_q;
      captured_dq = latest_dq;
    }
    std::cerr << "G1_READONLY_DIAGNOSTIC: {\"secondary_imu_sample_count\":"
              << captured_secondary_imu << ",\"latest_q\":";
    const auto append_array = [](const auto& values) {
      std::cerr << '[';
      for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) std::cerr << ',';
        std::cerr << values[index];
      }
      std::cerr << ']';
    };
    append_array(captured_q);
    std::cerr << ",\"latest_dq\":";
    append_array(captured_dq);
    std::cerr << "}" << std::endl;
    const auto report = g1_fingerprint::BuildReport(
        captured,
        {.interface = options.interface,
         .domain_id = options.domain_id,
         .elapsed_seconds = elapsed,
         .minimum_samples = options.minimum_samples,
         .invalid_crc_samples = captured_invalid_crc});
    std::cout << report.json << std::endl;
    return report.exit_code;
  } catch (const std::exception& error) {
    std::cout << ErrorJson(error.what()) << std::endl;
    return 1;
  } catch (...) {
    std::cout << ErrorJson("unknown failure") << std::endl;
    return 1;
  }
}
