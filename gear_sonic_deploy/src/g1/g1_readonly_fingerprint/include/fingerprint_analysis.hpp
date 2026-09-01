#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace g1_fingerprint {

constexpr std::size_t kMotorSlots = 35;
constexpr std::uint64_t kUint32Modulus = std::uint64_t{1} << 32;
constexpr std::uint64_t kUint32HalfRange = std::uint64_t{1} << 31;

struct Snapshot {
  std::array<std::uint32_t, 2> version{};
  std::uint8_t mode_pr{};
  std::uint8_t mode_machine{};
  std::uint32_t tick{};
  std::array<std::uint8_t, kMotorSlots> motor_modes{};
  std::array<std::uint32_t, kMotorSlots> motor_status{};
  std::array<std::array<std::uint32_t, 2>, kMotorSlots> motor_sensors{};
  std::array<std::array<std::int16_t, 2>, kMotorSlots> motor_temperatures{};
  std::int16_t imu_temperature{};
  std::array<float, 29> q{};
  std::array<float, 29> dq{};
};

struct ReportInput {
  std::string interface;
  int domain_id{};
  double elapsed_seconds{};
  std::size_t minimum_samples{};
  std::size_t invalid_crc_samples{};
};

struct ReportResult {
  std::string json;
  int exit_code{};
};

inline std::string JsonString(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20) {
          constexpr char digits[] = "0123456789abcdef";
          output << "\\u00" << digits[character >> 4] << digits[character & 0xf];
        } else {
          output << character;
        }
    }
  }
  output << '"';
  return output.str();
}

template <typename T>
inline void AppendNumericArray(std::ostringstream& output,
                               const T& values) {
  output << '[';
  bool first = true;
  for (const auto value : values) {
    if (!first) output << ',';
    first = false;
    output << +value;
  }
  output << ']';
}

inline void AppendIssues(std::ostringstream& output,
                         const std::vector<std::string>& issues) {
  output << '[';
  for (std::size_t index = 0; index < issues.size(); ++index) {
    if (index != 0) output << ',';
    output << JsonString(issues[index]);
  }
  output << ']';
}

inline ReportResult BuildReport(const std::vector<Snapshot>& samples,
                                const ReportInput& input) {
  std::vector<std::string> issues;
  if (input.invalid_crc_samples != 0) {
    issues.emplace_back("invalid_crc_sample_received");
  }
  if (samples.size() < input.minimum_samples) {
    issues.emplace_back("insufficient_samples");
  }

  std::set<unsigned int> mode_machines;
  std::set<unsigned int> mode_prs;
  std::set<std::array<std::uint32_t, 2>> versions;
  for (const auto& sample : samples) {
    mode_machines.insert(sample.mode_machine);
    mode_prs.insert(sample.mode_pr);
    versions.insert(sample.version);
  }
  if (mode_machines.size() > 1) issues.emplace_back("mode_machine_changed");
  if (mode_prs.size() > 1) issues.emplace_back("mode_pr_changed");
  if (versions.size() > 1) issues.emplace_back("version_changed");

  std::size_t advancing = 0;
  std::size_t duplicate = 0;
  std::size_t regressing = 0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const std::uint64_t previous = samples[index - 1].tick;
    const std::uint64_t current = samples[index].tick;
    const std::uint64_t delta =
        (current + kUint32Modulus - previous) % kUint32Modulus;
    if (delta == 0) {
      ++duplicate;
    } else if (delta < kUint32HalfRange) {
      ++advancing;
    } else {
      ++regressing;
    }
  }
  if (!samples.empty() &&
      advancing < std::max<std::size_t>(1, input.minimum_samples - 1)) {
    issues.emplace_back("tick_not_sufficiently_advancing");
  }
  if (regressing != 0) issues.emplace_back("tick_regressed");

  std::ostringstream output;
  output << "{\"schema\":\"motionlcm.g1.readonly-lowstate-diagnostic.v1\""
         << ",\"receive_only\":true"
         << ",\"topic\":\"rt/lowstate\""
         << ",\"interface\":" << JsonString(input.interface)
         << ",\"domain_id\":" << input.domain_id
         << ",\"elapsed_seconds\":" << input.elapsed_seconds
         << ",\"sample_count\":" << samples.size()
         << ",\"minimum_sample_count\":" << input.minimum_samples
         << ",\"invalid_crc_sample_count\":" << input.invalid_crc_samples
         << ",\"diagnostic_findings\":";
  AppendIssues(output, issues);
  output << ",\"observations\":{";

  output << "\"mode_machine\":";
  if (mode_machines.size() == 1) output << *mode_machines.begin();
  else output << "null";
  output << ",\"mode_machine_values\":";
  AppendNumericArray(output, mode_machines);

  output << ",\"mode_pr\":";
  if (mode_prs.size() == 1) output << *mode_prs.begin();
  else output << "null";
  output << ",\"mode_pr_values\":";
  AppendNumericArray(output, mode_prs);

  output << ",\"version\":";
  if (versions.size() == 1) AppendNumericArray(output, *versions.begin());
  else output << "null";
  output << ",\"version_values\":[";
  bool first_version = true;
  for (const auto& version : versions) {
    if (!first_version) output << ',';
    first_version = false;
    AppendNumericArray(output, version);
  }
  output << ']';

  output << ",\"tick_first\":";
  if (samples.empty()) output << "null";
  else output << samples.front().tick;
  output << ",\"tick_last\":";
  if (samples.empty()) output << "null";
  else output << samples.back().tick;
  output << ",\"tick_advancing_transitions\":" << advancing
         << ",\"tick_duplicate_transitions\":" << duplicate
         << ",\"tick_regressing_transitions\":" << regressing
         << ",\"motor_state_slot_count\":";

  if (samples.empty()) {
    output << "null,\"motor_mode_vector\":null"
           << ",\"motor_status_vector\":null"
           << ",\"motor_sensor_vector\":null"
           << ",\"motor_temperature_ranges\":null"
           << ",\"imu_temperature_range\":null"
           << ",\"latest_q\":null"
           << ",\"latest_dq\":null";
  } else {
    const auto& latest = samples.back();
    std::array<std::uint32_t, kMotorSlots> motor_status_or{};
    for (const auto& sample : samples) {
      for (std::size_t index = 0; index < kMotorSlots; ++index) {
        motor_status_or[index] |= sample.motor_status[index];
      }
    }
    output << kMotorSlots << ",\"motor_mode_vector\":";
    AppendNumericArray(output, latest.motor_modes);
    output << ",\"motor_status_vector\":";
    // Report the bitwise OR across the whole capture so a transient motor
    // fault cannot disappear merely because the final sample recovered.
    AppendNumericArray(output, motor_status_or);
    output << ",\"motor_sensor_vector\":[";
    for (std::size_t index = 0; index < kMotorSlots; ++index) {
      if (index != 0) output << ',';
      AppendNumericArray(output, latest.motor_sensors[index]);
    }
    output << "],\"motor_temperature_ranges\":[";
    for (std::size_t motor = 0; motor < kMotorSlots; ++motor) {
      if (motor != 0) output << ',';
      output << '[';
      for (std::size_t sensor = 0; sensor < 2; ++sensor) {
        if (sensor != 0) output << ',';
        auto minimum = samples.front().motor_temperatures[motor][sensor];
        auto maximum = minimum;
        for (const auto& sample : samples) {
          minimum = std::min(minimum, sample.motor_temperatures[motor][sensor]);
          maximum = std::max(maximum, sample.motor_temperatures[motor][sensor]);
        }
        output << '[' << minimum << ',' << maximum << ']';
      }
      output << ']';
    }
    auto imu_minimum = samples.front().imu_temperature;
    auto imu_maximum = imu_minimum;
    for (const auto& sample : samples) {
      imu_minimum = std::min(imu_minimum, sample.imu_temperature);
      imu_maximum = std::max(imu_maximum, sample.imu_temperature);
    }
    output << "],\"imu_temperature_range\":[" << imu_minimum << ','
           << imu_maximum << "]"
           << ",\"latest_q\":";
    AppendNumericArray(output, latest.q);
    output << ",\"latest_dq\":";
    AppendNumericArray(output, latest.dq);
  }
  output << "}}";
  return {output.str(), issues.empty() ? 0 : 2};
}

}  // namespace g1_fingerprint
