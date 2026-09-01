#include <gtest/gtest.h>

#include "../include/input_interface/stream_episode.hpp"

#include <optional>
#include <string>

namespace episode = sonic::stream_episode;

TEST(StreamEpisode, IdentifierGrammarMatchesReplayWireContract) {
  EXPECT_TRUE(episode::IsValidIdentifier("episode-a"));
  EXPECT_TRUE(episode::IsValidIdentifier("real-bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"));
  EXPECT_TRUE(episode::IsValidIdentifier("A_Z.0-9"));
  EXPECT_TRUE(episode::IsValidIdentifier(std::string(episode::kMaxLength, 'x')));

  EXPECT_FALSE(episode::IsValidIdentifier(""));
  EXPECT_FALSE(episode::IsValidIdentifier(".episode"));
  EXPECT_FALSE(episode::IsValidIdentifier("_episode"));
  EXPECT_FALSE(episode::IsValidIdentifier("-episode"));
  EXPECT_FALSE(episode::IsValidIdentifier(
      std::string(episode::kMaxLength + 1, 'x')));
  EXPECT_FALSE(episode::IsValidIdentifier("with space"));
  EXPECT_FALSE(episode::IsValidIdentifier("path/segment"));
  EXPECT_FALSE(episode::IsValidIdentifier("line\nbreak"));
  EXPECT_FALSE(episode::IsValidIdentifier(std::string("\xC3\xA9")));
}

TEST(StreamEpisode, PhysicalExpectationRequiresByteExactMatch) {
  const std::optional<std::string> unbound;
  EXPECT_TRUE(episode::MatchesExpected("", unbound));
  EXPECT_TRUE(episode::MatchesExpected("legacy-sim", unbound));

  const std::optional<std::string> expected("episode-a");
  EXPECT_TRUE(episode::MatchesExpected("episode-a", expected));
  EXPECT_FALSE(episode::MatchesExpected("", expected));
  EXPECT_FALSE(episode::MatchesExpected("episode-b", expected));
  EXPECT_FALSE(episode::MatchesExpected("Episode-a", expected));
}

TEST(StreamEpisode, PublishedContractNamesArePinned) {
  EXPECT_EQ(episode::kHeaderKey, "episode");
  EXPECT_EQ(episode::kExpectationEnv, "SONIC_EXPECTED_STREAM_EPISODE");
}
