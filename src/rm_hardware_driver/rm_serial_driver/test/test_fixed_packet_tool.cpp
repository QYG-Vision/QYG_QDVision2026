// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "dummy_transporter.hpp"
#include "gtest/gtest.h"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/float_protocol.hpp"

using namespace qd;
TEST(FixedPacketTool, construct_with_nullptr) {
  EXPECT_THROW(serial_driver::FixedPacketTool<32>(nullptr), std::invalid_argument);
}

TEST(FixedPacketTool, send_and_recv) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  serial_driver::FixedPacket<32> packet1, packet2;
  // send
  int a = 10;
  packet1.loadData(a, 10);
  bool send_ret = packet_tool1->sendPacket(packet1);
  ASSERT_TRUE(send_ret);
  // recv
  int b;
  bool recv_ret = packet_tool2->recvPacket(packet2);
  ASSERT_TRUE(recv_ret);
  packet2.unloadData<int>(b, 10);
  EXPECT_EQ(a, b);
}

TEST(FixedPacketTool, realtime_send) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  packet_tool1->enbaleRealtimeSend(true);
  serial_driver::FixedPacket<32> packet1, packet2;
  // recv
  auto t = std::thread([&]() {
    int b;
    for (int i = 0; i < 10; i++) {
      bool recv_ret = packet_tool2->recvPacket(packet2);
      ASSERT_TRUE(recv_ret);
      packet2.unloadData<int>(b, 10);
      EXPECT_EQ(i, b);
    }
  });
  // send
  for (int i = 0; i < 10; i++) {
    packet1.loadData(i, 10);
    bool send_ret = packet_tool1->sendPacket(packet1);
    ASSERT_TRUE(send_ret);
  }
  t.join();
}

namespace {

class ChunkTransporter: public serial_driver::TransporterInterface {
public:
    std::deque<std::vector<uint8_t>> chunks;
    bool open() override { return true; }
    void close() override {}
    bool isOpen() override { return true; }
    std::string errorMessage() override { return {}; }
    int write(const void*, size_t size) override { return static_cast<int>(size); }

    int read(void* buffer, size_t size) override {
        if (chunks.empty()) {
            return 0;
        }
        auto& chunk = chunks.front();
        const auto count = std::min(size, chunk.size());
        std::copy_n(chunk.begin(), count, static_cast<uint8_t*>(buffer));
        chunk.erase(chunk.begin(), chunk.begin() + count);
        if (chunk.empty()) {
            chunks.pop_front();
        }
        return static_cast<int>(count);
    }
};

TEST(FixedPacketTool, preservesPartialFramesAndResynchronizesAfterNoise) {
    auto transporter = std::make_shared<ChunkTransporter>();
    serial_driver::FixedPacketTool<24> tool(transporter);
    auto expected = serial_driver::wire::make_frame();
    // 载荷内的帧头尾字节不能截断物理帧。
    expected[5] = 0xff;
    expected[6] = 0x0d;
    transporter->chunks.push_back({0x77, 0x88, 0x99});
    transporter->chunks.emplace_back(expected.begin(), expected.begin() + 5);
    transporter->chunks.emplace_back(expected.begin() + 5, expected.begin() + 11);
    transporter->chunks.emplace_back(expected.begin() + 11, expected.end());
    serial_driver::FixedPacket<24> actual;
    EXPECT_FALSE(tool.recvPacket(actual));
    EXPECT_FALSE(tool.recvPacket(actual));
    EXPECT_FALSE(tool.recvPacket(actual));
    ASSERT_TRUE(tool.recvPacket(actual));
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.buffer()));
    EXPECT_FALSE(tool.recvPacket(actual));
}

TEST(FixedPacketTool, receivesBackToBack24ByteFramesAcrossArbitraryReads) {
    auto transporter = std::make_shared<ChunkTransporter>();
    serial_driver::FixedPacketTool<24> tool(transporter);
    std::array<serial_driver::wire::Frame, 3> frames{};
    for (size_t i = 0; i < frames.size(); ++i) {
        serial_driver::wire::Feedback feedback;
        feedback.yaw = static_cast<float>(i + 1);
        frames[i] = serial_driver::wire::encode_feedback(feedback);
    }
    std::vector<uint8_t> stream{0x66, 0x77, 0x88};
    for (const auto& frame : frames) {
        stream.insert(stream.end(), frame.begin(), frame.end());
    }
    transporter->chunks.push_back(stream);
    serial_driver::FixedPacket<24> actual;
    size_t received = 0;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (tool.recvPacket(actual)) {
            ASSERT_LT(received, frames.size());
            const auto& expected = frames[received++];
            EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.buffer()));
        }
    }
    EXPECT_EQ(received, frames.size());
}

} // namespace
