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

#include <memory>
#include <algorithm>
#include <array>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "dummy_transporter.hpp"
#include "gtest/gtest.h"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"

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
class ChunkTransporter : public qd::serial_driver::TransporterInterface {
public:
    bool open() override { return true; }
    void close() override {}
    bool isOpen() override { return true; }
    std::string errorMessage() override { return {}; }
    int read(void* buffer, size_t length) override {
        ++reads;
        if (chunks.empty()) return 0;
        auto& chunk = chunks.front();
        const auto count = std::min(length, chunk.size());
        std::memcpy(buffer, chunk.data(), count);
        chunk.erase(chunk.begin(), chunk.begin() + count);
        if (chunk.empty()) chunks.pop_front();
        return static_cast<int>(count);
    }
    int write(const void* buffer, size_t length) override {
        const auto* bytes = static_cast<const uint8_t*>(buffer);
        sent.assign(bytes, bytes + length);
        return static_cast<int>(length);
    }
    std::deque<std::vector<uint8_t>> chunks;
    std::vector<uint8_t> sent;
    int reads = 0;
};

TEST(FixedPacketTool, v2SplitStickyNoiseAndEmbeddedDelimiters) {
    auto transport = std::make_shared<ChunkTransporter>();
    serial_driver::FixedPacketTool<24> tool(transport);
    serial_driver::FixedPacket<24> received;
    std::vector<uint8_t> frame(24, 0);
    frame[0] = 0xfe;
    frame[5] = 0xfe;
    frame[6] = 0x0d;
    frame[23] = 0x0d;
    transport->chunks.push_back({0x55, 0x66, 0xff});
    transport->chunks.emplace_back(frame.begin(), frame.begin() + 7);
    transport->chunks.emplace_back(frame.begin() + 7, frame.end());
    EXPECT_FALSE(tool.recvPacket(received));
    EXPECT_FALSE(tool.recvPacket(received));
    ASSERT_TRUE(tool.recvPacket(received));
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.buffer()));
    std::vector<uint8_t> sticky = frame;
    sticky.insert(sticky.end(), frame.begin(), frame.end());
    transport->chunks.push_back(sticky);
    ASSERT_TRUE(tool.recvPacket(received));
    ASSERT_TRUE(tool.recvPacket(received));
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.buffer()));
}

TEST(FixedPacketTool, rejectsV1AndBadTailThenResynchronizes) {
    auto transport = std::make_shared<ChunkTransporter>();
    serial_driver::FixedPacketTool<24> tool(transport);
    serial_driver::FixedPacket<24> received;
    std::vector<uint8_t> old(16, 0);
    old[0] = 0xff;
    old[15] = 0x0d;
    transport->chunks.push_back(old);
    EXPECT_FALSE(tool.recvPacket(received));
    std::vector<uint8_t> bad(24, 0);
    bad[0] = 0xfe;
    transport->chunks.push_back(bad);
    EXPECT_FALSE(tool.recvPacket(received));
    bad[23] = 0x0d;
    transport->chunks.push_back(bad);
    ASSERT_TRUE(tool.recvPacket(received));
    EXPECT_TRUE(std::equal(bad.begin(), bad.end(), received.buffer()));
}

TEST(FixedPacketTool, sends32BytesWhileReceiving24OnSameTransport) {
    auto transport = std::make_shared<ChunkTransporter>();
    serial_driver::FixedPacketTool<24> tool(transport);
    serial_driver::FixedPacket<32> outgoing;
    ASSERT_TRUE(tool.sendPacket(outgoing));
    ASSERT_EQ(transport->sent.size(), 32U);
    EXPECT_EQ(transport->sent[0], 0xfe);
    EXPECT_EQ(transport->sent[31], 0x0d);
}
} // namespace
