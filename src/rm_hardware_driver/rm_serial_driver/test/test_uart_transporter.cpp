#include <chrono>
#include <future>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include "rm_serial_driver/uart_transporter.hpp"

namespace qd::serial_driver {

TEST(UartTransporter, ReadTimesOutWhenNoByteIsAvailable) {
    const int master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_NE(master_fd, -1);
    ASSERT_EQ(grantpt(master_fd), 0);
    ASSERT_EQ(unlockpt(master_fd), 0);
    const char* const slave_path = ptsname(master_fd);
    ASSERT_NE(slave_path, nullptr);

    UartTransporter transporter(slave_path, 115200);
    ASSERT_TRUE(transporter.open());

    auto read_future = std::async(std::launch::async, [&transporter]() {
        char byte = 0;
        return transporter.read(&byte, sizeof(byte));
    });
    const auto status = read_future.wait_for(std::chrono::milliseconds(250));

    if (status != std::future_status::ready) {
        ::close(master_fd);
    }
    ASSERT_EQ(status, std::future_status::ready);
    EXPECT_EQ(read_future.get(), 0);

    transporter.close();
    if (status == std::future_status::ready) {
        ::close(master_fd);
    }
}

} // namespace qd::serial_driver
