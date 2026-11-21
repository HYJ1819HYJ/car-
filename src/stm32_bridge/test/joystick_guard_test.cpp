#include "stm32_bridge/joystick_guard.hpp"
#include <gtest/gtest.h>
#include <fcntl.h>

TEST(JoystickGuard, NonblockingIdleKeepsCurrentState) {
  int fds[2]; ASSERT_EQ(pipe(fds), 0);
  ASSERT_EQ(fcntl(fds[0], F_SETFL, O_NONBLOCK), 0);
  double axes[8]{}; int buttons[11]{};
  js_event axis{}; axis.type=JS_EVENT_AXIS|JS_EVENT_INIT; axis.number=1; axis.value=-16384;
  js_event lb{}; lb.type=JS_EVENT_BUTTON|JS_EVENT_INIT; lb.number=4; lb.value=1;
  ASSERT_EQ(write(fds[1], &axis, sizeof(axis)), sizeof(axis));
  ASSERT_EQ(write(fds[1], &lb, sizeof(lb)), sizeof(lb));
  ASSERT_TRUE(stm32_bridge::readJoystickSafely(fds[0], axes, buttons));
  EXPECT_LT(axes[1], -0.49); EXPECT_EQ(buttons[4], 1);
  EXPECT_TRUE(stm32_bridge::readJoystickSafely(fds[0], axes, buttons));
  EXPECT_EQ(buttons[4], 1); EXPECT_LT(axes[1], -0.49);
  close(fds[1]); close(fds[0]);
}

TEST(JoystickGuard, DisconnectClearsDeadmanAndAxes) {
  int fds[2]; ASSERT_EQ(pipe(fds), 0);
  ASSERT_EQ(fcntl(fds[0], F_SETFL, O_NONBLOCK), 0);
  double axes[8]{}; int buttons[11]{}; axes[0]=0.5; axes[1]=-1; buttons[4]=1;
  close(fds[1]);
  EXPECT_FALSE(stm32_bridge::readJoystickSafely(fds[0], axes, buttons));
  EXPECT_EQ(buttons[4], 0); EXPECT_DOUBLE_EQ(axes[0], 0); EXPECT_DOUBLE_EQ(axes[1], 0);
  close(fds[0]);
}

TEST(JoystickGuard, FailedReadClearsState) {
  double axes[8]{}; int buttons[11]{}; axes[1]=1; buttons[4]=1;
  EXPECT_FALSE(stm32_bridge::readJoystickSafely(-1, axes, buttons));
  EXPECT_EQ(buttons[4], 0); EXPECT_DOUBLE_EQ(axes[1], 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
