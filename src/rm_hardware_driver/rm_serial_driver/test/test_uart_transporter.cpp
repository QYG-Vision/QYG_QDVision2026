#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>

#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"
#include "gtest/gtest.h"

namespace {

class UartTransporterTest: public ::testing::Test {
protected:
    /** @brief 创建伪终端串口，使回归测试无需连接真实电控。 */
    void SetUp() override {
        master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(master_fd_, 0);
        ASSERT_EQ(grantpt(master_fd_), 0);
        ASSERT_EQ(unlockpt(master_fd_), 0);
        const char* slave_path = ptsname(master_fd_);
        ASSERT_NE(slave_path, nullptr);
        slave_fd_ = ::open(slave_path, O_RDWR | O_NOCTTY);
        ASSERT_GE(slave_fd_, 0);
        transporter_ = std::make_shared<qd::serial_driver::UartTransporter>(slave_path, 921600);
    }

    /** @brief 关闭串口和伪终端，包括断言提前退出后的资源。 */
    void TearDown() override {
        if (transporter_) {
            transporter_->close();
        }
        if (slave_fd_ >= 0) {
            ::close(slave_fd_);
        }
        if (master_fd_ >= 0) {
            ::close(master_fd_);
        }
    }

    int master_fd_ { -1 };
    int slave_fd_ { -1 };
    std::shared_ptr<qd::serial_driver::UartTransporter> transporter_;
};

TEST_F(UartTransporterTest, preserves_binary_frame_with_inherited_parmrk) {
    struct termios options {};
    ASSERT_EQ(tcgetattr(slave_fd_, &options), 0);
    // 复现设备继承 PARMRK 的状态，确保有效 0xff 不会被终端重复。
    options.c_iflag |= PARMRK;
    options.c_iflag &= ~ISTRIP;
    ASSERT_EQ(tcsetattr(slave_fd_, TCSANOW, &options), 0);

    ASSERT_TRUE(transporter_->open()) << transporter_->errorMessage();
    ASSERT_EQ(tcgetattr(slave_fd_, &options), 0);
    EXPECT_EQ(options.c_iflag & PARMRK, 0U);

    const std::array<uint8_t, 16> frame { 0xff, 0x00, 0x00, 0x23, 0x08, 0x0d, 0xff, 0xf4,
                                          0x05, 0xdc, 0x00, 0x02, 0xd2, 0xc5, 0x00, 0x0d };
    ASSERT_EQ(::write(master_fd_, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));

    struct pollfd readable {
        slave_fd_, POLLIN, 0
    };
    ASSERT_EQ(::poll(&readable, 1, 1000), 1);
    ASSERT_NE(readable.revents & POLLIN, 0);
    std::array<uint8_t, 32> received {};
    ASSERT_EQ(transporter_->read(received.data(), received.size()), static_cast<int>(frame.size()));
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.begin()));
}

TEST_F(UartTransporterTest, receives_first_packet_without_failed_read_to_open_device) {
    ASSERT_FALSE(transporter_->isOpen());
    qd::serial_driver::FixedPacketTool<24> packet_tool(transporter_);
    ASSERT_TRUE(transporter_->isOpen());

    const std::array<uint8_t, 24> frame { 0xfe, 0x00, 0x00, 0x25, 0x08, 0x0f, 0xff, 0xf4,
                                          0x05, 0xdc, 0x00, 0x09, 0xae, 0x33, 0x3f, 0x80, 0, 0, 0xbf, 0x80, 0, 0, 0, 0x0d };
    ASSERT_EQ(::write(master_fd_, frame.data(), frame.size()), static_cast<ssize_t>(frame.size()));
    struct pollfd readable {
        slave_fd_, POLLIN, 0
    };
    ASSERT_EQ(::poll(&readable, 1, 1000), 1);
    ASSERT_NE(readable.revents & POLLIN, 0);

    qd::serial_driver::FixedPacket<24> received;
    ASSERT_TRUE(packet_tool.recvPacket(received));
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.buffer()));
}

} // namespace
