#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "stm32_bridge/protocol.hpp"

namespace {

using stm32_bridge::ControlCmd;
using stm32_bridge::ControlProtocol;
using stm32_bridge::EncoderFeedback;
using stm32_bridge::FeedbackProtocol;
using stm32_bridge::packControl;
using stm32_bridge::unpackFeedback;

TEST(Protocol, PacksControlFrameExpectedByStm32) {
  ControlCmd cmd;
  cmd.v_mm_s = 150;
  cmd.w_mrad_s = 0;

  const std::vector<uint8_t> expected{
      0xAA, 0x55, 0x00, 0x96, 0x00, 0x00, 0x96, 0xCC};
  EXPECT_EQ(packControl(cmd, ControlProtocol{}), expected);
}

TEST(Protocol, UnpacksEncoderFrameSentByStm32) {
  const std::vector<uint8_t> frame{
      0xAA, 0x55, 0x04, 0xD2, 0x16, 0x2E, 0x1A, 0x0D, 0x0A};
  EncoderFeedback feedback;

  ASSERT_TRUE(unpackFeedback(frame, feedback, FeedbackProtocol{}));
  EXPECT_TRUE(feedback.valid);
  EXPECT_EQ(feedback.left, 1234);
  EXPECT_EQ(feedback.right, 5678);
}

TEST(Protocol, RejectsBadChecksumOrTail) {
  EncoderFeedback feedback;
  std::vector<uint8_t> frame{
      0xAA, 0x55, 0x04, 0xD2, 0x16, 0x2E, 0x1A, 0x0D, 0x0A};

  frame[6] ^= 0x01;
  EXPECT_FALSE(unpackFeedback(frame, feedback, FeedbackProtocol{}));
  EXPECT_FALSE(feedback.valid);

  frame[6] ^= 0x01;
  frame[8] = 0xCC;
  EXPECT_FALSE(unpackFeedback(frame, feedback, FeedbackProtocol{}));
  EXPECT_FALSE(feedback.valid);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
