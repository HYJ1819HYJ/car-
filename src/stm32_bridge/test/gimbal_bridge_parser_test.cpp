#include <gtest/gtest.h>

#include <stm32_bridge/gimbal_status.hpp>

#include <cstdio>

using stm32_bridge::GimbalStartup;
using stm32_bridge::GimbalStatus;
using stm32_bridge::GimbalStatusParser;

namespace {
const std::string kLegacy =
    "STAT,mode=0,ready=1,found=0,lock=0,search=1,x=559,y=294,rx=153,cmd=0,bearing10=-21,tilt10=-135";

std::string checked(const std::string& payload) {
  unsigned int checksum = 0;
  for (unsigned char ch : payload) checksum ^= ch;
  char suffix[4];
  std::snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return payload + suffix;
}
}

TEST(GimbalStatus, LegacyAndChecksumUpgrade) {
  GimbalStatusParser parser;
  GimbalStatus status{};
  ASSERT_TRUE(parser.parse(kLegacy, &status));
  EXPECT_FALSE(status.enabled);
  EXPECT_TRUE(status.ready);
  EXPECT_TRUE(status.searching);
  EXPECT_DOUBLE_EQ(status.pan_deg, -2.1);
  EXPECT_DOUBLE_EQ(status.tilt_deg, -13.5);
  EXPECT_FALSE(parser.parse(kLegacy + "*zz", &status));
  ASSERT_TRUE(parser.parse(checked(kLegacy), &status));
  EXPECT_FALSE(parser.parse(kLegacy, &status));

  GimbalStatusParser strict(true);
  EXPECT_FALSE(strict.parse(kLegacy, &status));
  EXPECT_TRUE(strict.parse(checked(kLegacy), &status));
}

TEST(GimbalStatus, RejectsBadFramesWithoutPartialUpdates) {
  GimbalStatusParser parser;
  GimbalStatus status{};
  ASSERT_TRUE(parser.parse(kLegacy, &status));
  const auto prefix = kLegacy.substr(0, kLegacy.find("bearing10="));
  for (const auto& broken : {kLegacy + "MCSTAT,mode=1", kLegacy + "\x80",
                            kLegacy + "*00", kLegacy + ",mode=1",
                            prefix + "bearing10=1801,tilt10=0",
                            prefix + "bearing10=0,tilt10=-901",
                            prefix + "bearing10=0", prefix + "bearing10=0,tilt10=1junk"}) {
    EXPECT_FALSE(parser.parse(broken, &status)) << broken;
    EXPECT_DOUBLE_EQ(status.tilt_deg, -13.5);
    EXPECT_FALSE(status.enabled);
  }
}

TEST(GimbalStartup, RecoversWhileTheSerialPortRemainsOpen) {
  using Action = GimbalStartup::Action;
  GimbalStartup startup;
  startup.reset(false);
  EXPECT_EQ(Action::None, startup.next(1.0, false, true, false, -13.5));
  EXPECT_EQ(Action::Center, startup.next(1.1, true, true, false, -13.5));
  EXPECT_EQ(Action::None, startup.next(1.8, true, true, false, -13.5));
  EXPECT_EQ(Action::Enable, startup.next(2.0, true, true, false, 0.0));
  EXPECT_EQ(Action::None, startup.next(3.0, true, true, false, 0.0));
  EXPECT_EQ(Action::None, startup.next(4.0, true, true, true, 0.0));
  EXPECT_EQ(Action::Enable, startup.next(5.0, true, true, false, 0.0));
  startup.requestCenter();
  EXPECT_EQ(Action::Center, startup.next(6.0, true, false, true, 0.0));
  EXPECT_EQ(Action::None, startup.next(7.0, true, false, true, 0.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
