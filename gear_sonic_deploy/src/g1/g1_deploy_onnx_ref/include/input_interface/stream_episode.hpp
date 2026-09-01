#ifndef SONIC_STREAM_EPISODE_HPP
#define SONIC_STREAM_EPISODE_HPP

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace sonic::stream_episode {

inline constexpr std::string_view kHeaderKey = "episode";
inline constexpr std::string_view kExpectationEnv =
    "SONIC_EXPECTED_STREAM_EPISODE";
inline constexpr std::size_t kMaxLength = 128;

// Keep this grammar identical to scripts/sonic_pose_replay.py.  Restricting
// the run identifier to printable ASCII makes environment, JSON and log
// representations byte-for-byte identical across Python and C++.
inline bool IsValidIdentifier(std::string_view value) {
  if (value.empty() || value.size() > kMaxLength) {
    return false;
  }
  for (std::size_t index = 0; index < value.size(); ++index) {
    const unsigned char character =
        static_cast<unsigned char>(value[index]);
    const bool alpha =
        (character >= static_cast<unsigned char>('A') &&
         character <= static_cast<unsigned char>('Z')) ||
        (character >= static_cast<unsigned char>('a') &&
         character <= static_cast<unsigned char>('z'));
    const bool digit = character >= static_cast<unsigned char>('0') &&
                       character <= static_cast<unsigned char>('9');
    if ((!alpha && !digit && index == 0) ||
        (!alpha && !digit && character != static_cast<unsigned char>('.') &&
        character != static_cast<unsigned char>('_') &&
        character != static_cast<unsigned char>('-'))) {
      return false;
    }
  }
  return true;
}

// No expectation is the deliberate simulation/backward-compatible mode.
// Once physical startup supplies an expectation, missing and non-identical
// packet episodes both fail closed.
inline bool MatchesExpected(
    std::string_view incoming,
    const std::optional<std::string>& expected) {
  return !expected.has_value() || incoming == *expected;
}

}  // namespace sonic::stream_episode

#endif  // SONIC_STREAM_EPISODE_HPP
