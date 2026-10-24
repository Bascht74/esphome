#include <gtest/gtest.h>

#include <string>

#include "esphome/components/cover/cover.h"

namespace esphome::cover::testing {

// A cover with a position that remembers what control() was given.
class SpeedCover : public Cover {
 public:
  CoverTraits get_traits() override {
    CoverTraits traits;
    traits.set_supports_position(true);
    traits.set_supports_stop(true);
    return traits;
  }

  bool controlled{false};
  bool last_stop{false};
  optional<float> last_position;
  StringRef last_speed;

 protected:
  void control(const CoverCall &call) override {
    this->controlled = true;
    this->last_stop = call.get_stop();
    this->last_position = call.get_position();
    this->last_speed = call.get_speed();
  }
};

static constexpr const char *const SPEEDS[] = {"slow", "fast"};

class CoverSpeedTest : public ::testing::Test {
 protected:
  void SetUp() override { this->cover_.set_supported_speeds_static(SPEEDS, 2); }

  SpeedCover cover_;
};

TEST_F(CoverSpeedTest, OpenWithAKnownSpeedPassesItToControl) {
  this->cover_.make_call().set_command_open().set_speed("fast").perform();
  ASSERT_TRUE(this->cover_.controlled);
  ASSERT_TRUE(this->cover_.last_position.has_value());
  EXPECT_EQ(*this->cover_.last_position, COVER_OPEN);
  EXPECT_EQ(this->cover_.last_speed, "fast");
  // The call keeps the list entry, not the caller's string
  EXPECT_EQ(this->cover_.last_speed.c_str(), SPEEDS[1]);
}

TEST_F(CoverSpeedTest, CallKeepsTheSpeedWhenTheCallersStringChanges) {
  std::string requested = "slow";
  auto call = this->cover_.make_call();
  call.set_speed(requested);
  requested = "gone";
  EXPECT_EQ(call.get_speed(), "slow");
}

TEST_F(CoverSpeedTest, SpeedFromABufferWithoutTerminatorIsFound) {
  // The API passes a pointer and a length into its receive buffer, with more bytes after the name
  const char buffer[] = "fast-and-more";
  auto call = this->cover_.make_call();
  call.set_speed(buffer, 4);
  EXPECT_EQ(call.get_speed(), "fast");
}

TEST_F(CoverSpeedTest, UnknownSpeedIsDroppedAndTheCoverStillMoves) {
  // Not in the list, only the start of a name, a longer name, another case
  for (const char *unknown : {"turbo", "fas", "fastest", "Fast"}) {
    SpeedCover cover;
    cover.set_supported_speeds_static(SPEEDS, 2);
    cover.make_call().set_position(0.4f).set_speed(unknown).perform();
    ASSERT_TRUE(cover.last_position.has_value()) << unknown;
    EXPECT_EQ(*cover.last_position, 0.4f) << unknown;
    EXPECT_TRUE(cover.last_speed.empty()) << unknown;
  }
}

TEST_F(CoverSpeedTest, EmptySpeedMeansTheDefaultSpeed) {
  this->cover_.make_call().set_command_close().set_speed("").perform();
  ASSERT_TRUE(this->cover_.last_position.has_value());
  EXPECT_EQ(*this->cover_.last_position, COVER_CLOSED);
  EXPECT_TRUE(this->cover_.last_speed.empty());
}

TEST_F(CoverSpeedTest, SpeedWithoutAMoveIsDropped) {
  // Stop has no position, so there is nothing the speed could apply to
  this->cover_.make_call().set_command_stop().set_speed("slow").perform();
  EXPECT_TRUE(this->cover_.last_stop);
  EXPECT_TRUE(this->cover_.last_speed.empty());
}

TEST(CoverWithoutSpeedsTest, EverySpeedIsDropped) {
  SpeedCover cover;
  EXPECT_TRUE(cover.get_supported_speeds().empty());
  cover.make_call().set_command_open().set_speed("slow").perform();
  ASSERT_TRUE(cover.last_position.has_value());
  EXPECT_EQ(*cover.last_position, COVER_OPEN);
  EXPECT_TRUE(cover.last_speed.empty());
}

}  // namespace esphome::cover::testing
