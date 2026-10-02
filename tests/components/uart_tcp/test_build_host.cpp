#include <gtest/gtest.h>

// The host config includes a modbus bridge, so this file exists to compile
// UartTcpModbus. The framing itself is covered by MbapTest.
TEST(UartTcpModbusBuild, HostConfigCompiles) { EXPECT_TRUE(true); }
