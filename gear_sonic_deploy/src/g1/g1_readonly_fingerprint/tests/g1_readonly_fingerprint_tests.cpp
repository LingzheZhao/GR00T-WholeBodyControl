#include "fingerprint_analysis.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

g1_fingerprint::Snapshot MakeSnapshot(std::uint32_t tick,
                                      std::uint8_t mode_machine = 11,
                                      std::uint8_t mode_pr = 0) {
  g1_fingerprint::Snapshot snapshot;
  snapshot.version = {1, 2};
  snapshot.mode_pr = mode_pr;
  snapshot.mode_machine = mode_machine;
  snapshot.tick = tick;
  snapshot.motor_modes.fill(1);
  snapshot.imu_temperature = 32;
  snapshot.q[0] = 0.25F;
  snapshot.dq[0] = -0.5F;
  for (auto& temperature : snapshot.motor_temperatures) {
    temperature = {30, 31};
  }
  return snapshot;
}

g1_fingerprint::ReportResult Analyze(
    const std::vector<g1_fingerprint::Snapshot>& samples,
    std::size_t invalid_crc_samples = 0) {
  return g1_fingerprint::BuildReport(
      samples,
      {.interface = "lo",
       .domain_id = 0,
       .elapsed_seconds = 1.0,
       .minimum_samples = 5,
       .invalid_crc_samples = invalid_crc_samples});
}

void RequireContains(const std::string& text, const std::string& expected) {
  if (text.find(expected) == std::string::npos) {
    throw std::runtime_error("missing expected report fragment: " + expected);
  }
}

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
  std::vector<g1_fingerprint::Snapshot> stable;
  for (std::uint32_t tick = 100; tick < 105; ++tick) {
    stable.push_back(MakeSnapshot(tick));
  }
  auto report = Analyze(stable);
  Require(report.exit_code == 0, "clean capture produced diagnostic findings");
  RequireContains(
      report.json,
      "\"schema\":\"motionlcm.g1.readonly-lowstate-diagnostic.v1\"");
  RequireContains(report.json, "\"receive_only\":true");
  RequireContains(report.json, "\"diagnostic_findings\":[]");
  RequireContains(report.json, "\"mode_machine\":11");
  RequireContains(report.json, "\"mode_pr\":0");
  RequireContains(report.json, "\"latest_q\":[0.25");
  RequireContains(report.json, "\"latest_dq\":[-0.5");

  auto transient_fault = stable;
  transient_fault[1].motor_status[3] = 4;
  report = Analyze(transient_fault);
  RequireContains(report.json,
                  "\"motor_status_vector\":[0,0,0,4");

  auto changed = stable;
  changed.back().mode_machine = 5;
  report = Analyze(changed);
  Require(report.exit_code == 2, "mode change was not reported");
  RequireContains(report.json, "mode_machine_changed");
  RequireContains(report.json, "\"mode_machine\":null");

  auto pr_changed = stable;
  pr_changed.back().mode_pr = 1;
  report = Analyze(pr_changed);
  Require(report.exit_code == 2, "mode_pr change was not reported");
  RequireContains(report.json, "mode_pr_changed");

  auto stale = stable;
  for (auto& sample : stale) sample.tick = 100;
  report = Analyze(stale);
  Require(report.exit_code == 2, "stale ticks were not reported");
  RequireContains(report.json, "tick_not_sufficiently_advancing");

  std::vector<g1_fingerprint::Snapshot> wrapping;
  for (const auto tick :
       {0xfffffffdU, 0xfffffffeU, 0xffffffffU, 0U, 1U}) {
    wrapping.push_back(MakeSnapshot(tick));
  }
  report = Analyze(wrapping);
  Require(report.exit_code == 0, "uint32 tick rollover produced a finding");
  RequireContains(report.json, "\"tick_advancing_transitions\":4");

  report = Analyze(stable, 1);
  Require(report.exit_code == 2, "invalid CRC sample was not reported");
  RequireContains(report.json, "invalid_crc_sample_received");

  std::cout << "g1_readonly_fingerprint_tests: PASS\n";
  return 0;
}
