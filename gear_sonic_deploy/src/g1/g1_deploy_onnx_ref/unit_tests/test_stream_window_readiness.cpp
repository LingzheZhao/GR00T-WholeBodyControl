#include <gtest/gtest.h>

#include "../include/stream_window_readiness.hpp"

#include <atomic>
#include <chrono>
#include <future>

namespace readiness = sonic::stream_window_readiness;

TEST(StreamWindowReadiness, AcceptedTimeIsInvisibleUntilActivationCompletes) {
  readiness::CommitGate gate;
  const auto generation = gate.BeginGeneration();
  std::atomic<bool> active{false};
  std::promise<void> activation_entered;
  std::promise<void> release_activation;
  const auto release = release_activation.get_future().share();

  auto commit = std::async(std::launch::async, [&]() {
    return gate.CommitAfterActivation(generation, [&]() {
      activation_entered.set_value();
      release.wait();
      active.store(true, std::memory_order_release);
    });
  });
  activation_entered.get_future().wait();

  std::promise<void> reader_started;
  auto reader = std::async(std::launch::async, [&]() {
    reader_started.set_value();
    return gate.CurrentAcceptedTime();
  });
  reader_started.get_future().wait();

  // The read cannot pass the in-progress activation and observe a CURRENT
  // timestamp paired with the previous controller input.
  EXPECT_EQ(reader.wait_for(std::chrono::milliseconds(10)),
            std::future_status::timeout);

  release_activation.set_value();
  EXPECT_TRUE(commit.get());
  EXPECT_TRUE(reader.get().has_value());
  EXPECT_TRUE(active.load(std::memory_order_acquire));
}

TEST(StreamWindowReadiness, ResetRejectsDecodedResultFromOldGeneration) {
  readiness::CommitGate gate;
  const auto old_generation = gate.BeginGeneration();
  const auto current_generation = gate.BeginGeneration();
  bool stale_motion_activated = false;

  EXPECT_FALSE(gate.CommitAfterActivation(old_generation, [&]() {
    stale_motion_activated = true;
  }));
  EXPECT_FALSE(stale_motion_activated);
  EXPECT_FALSE(gate.CurrentAcceptedTime().has_value());

  bool current_motion_activated = false;
  EXPECT_TRUE(gate.CommitAfterActivation(current_generation, [&]() {
    current_motion_activated = true;
  }));
  EXPECT_TRUE(current_motion_activated);
  EXPECT_TRUE(gate.CurrentAcceptedTime().has_value());
}

TEST(StreamWindowReadiness, ConcurrentResetWinsBeforeLateDecodeCommit) {
  readiness::CommitGate gate;
  const auto decoded_generation = gate.BeginGeneration();
  std::promise<void> commit_thread_ready;
  std::promise<void> release_commit;
  const auto release = release_commit.get_future().share();
  std::atomic<bool> stale_motion_activated{false};

  auto late_commit = std::async(std::launch::async, [&]() {
    commit_thread_ready.set_value();
    release.wait();
    return gate.CommitAfterActivation(decoded_generation, [&]() {
      stale_motion_activated.store(true, std::memory_order_release);
    });
  });
  commit_thread_ready.get_future().wait();

  const auto next_generation = gate.BeginGeneration();
  EXPECT_NE(next_generation, decoded_generation);
  release_commit.set_value();

  EXPECT_FALSE(late_commit.get());
  EXPECT_FALSE(stale_motion_activated.load(std::memory_order_acquire));
  EXPECT_FALSE(gate.CurrentAcceptedTime().has_value());
}
