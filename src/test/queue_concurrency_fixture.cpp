// The MIT License (MIT)
//
// Copyright (c) 2026 RM2027 Navigation contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "comm/ldq.h"

namespace
{

using livox_ros::LidarDataQueue;
using livox_ros::PointPacket;
using livox_ros::PointXyzlt;
using livox_ros::StoragePacket;

void Require(bool condition)
{
  if (!condition) {
    std::cerr << "queue concurrency fixture assertion failed" << std::endl;
    std::abort();
  }
}

uint64_t NowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void Push(LidarDataQueue * queue, uint64_t sequence)
{
  PointXyzlt point{};
  point.x = static_cast<float>(sequence);
  point.offset_time = NowNs();
  PointPacket packet{};
  packet.points_num = 1;
  packet.points = &point;
  Require(
    livox_ros::QueuePushAny(
      queue, reinterpret_cast<uint8_t *>(&packet),
      sequence) == 1);
}

void CheckEmptyAndFull()
{
  LidarDataQueue queue{};
  Require(livox_ros::InitQueue(&queue, 4));
  Require(livox_ros::QueueIsEmpty(&queue));
  Require(!livox_ros::QueueIsFull(&queue));
  Require(livox_ros::QueueUsedSize(&queue) == 0);
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    Push(&queue, sequence);
  }
  Require(livox_ros::QueueIsFull(&queue));
  Require(livox_ros::QueueUnusedSize(&queue) == 0);
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    StoragePacket packet;
    Require(livox_ros::QueuePop(&queue, &packet));
    Require(packet.base_time == sequence);
    Require(packet.points_num == 1);
    Require(packet.points.size() == 1);
    Require(packet.points[0].x == static_cast<float>(sequence));
  }
  Require(livox_ros::QueueIsEmpty(&queue));
  StoragePacket empty_packet;
  Require(!livox_ros::QueuePop(&queue, &empty_packet));
  Require(livox_ros::DeInitQueue(&queue));
}

void RunStress(
  const std::string & name, uint64_t total,
  uint32_t producer_pause_every,
  uint32_t consumer_pause_every)
{
  LidarDataQueue queue{};
  Require(livox_ros::InitQueue(&queue, 1024));
  std::atomic<bool> failed{false};
  std::atomic<bool> producer_done{false};
  std::vector<uint64_t> latencies;
  latencies.reserve(total);
  const auto begin = std::chrono::steady_clock::now();

  std::thread producer([&] {
      for (uint64_t sequence = 1; sequence <= total; ++sequence) {
        while (livox_ros::QueueIsFull(&queue)) {
          if (failed.load(std::memory_order_acquire)) {
            producer_done.store(true, std::memory_order_release);
            return;
          }
          std::this_thread::yield();
        }
        Push(&queue, sequence);
        if (producer_pause_every != 0 && sequence % producer_pause_every == 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
      }
      producer_done.store(true, std::memory_order_release);
    });

  std::thread consumer([&] {
      uint64_t expected = 1;
      while (expected <= total) {
        StoragePacket packet;
        if (!livox_ros::QueuePop(&queue, &packet)) {
          if (producer_done.load(std::memory_order_acquire) &&
          livox_ros::QueueIsEmpty(&queue))
          {
            failed.store(true, std::memory_order_release);
            break;
          }
          std::this_thread::yield();
          continue;
        }
        if (packet.base_time != expected || packet.points_num != 1 ||
        packet.points.size() != 1 ||
        packet.points[0].x != static_cast<float>(expected))
        {
          failed.store(true, std::memory_order_release);
          break;
        }
        latencies.push_back(NowNs() - packet.points[0].offset_time);
        if (consumer_pause_every != 0 && expected % consumer_pause_every == 0) {
          std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        ++expected;
      }
    });

  producer.join();
  consumer.join();
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  Require(!failed.load(std::memory_order_acquire));
  Require(latencies.size() == total);
  Require(livox_ros::QueueIsEmpty(&queue));
  std::sort(latencies.begin(), latencies.end());
  const double seconds = std::chrono::duration<double>(elapsed).count();
  const double throughput = static_cast<double>(total) / seconds;
  std::cout << "QUEUE_PASS " << name << " total=" << total
            << " throughput_packets_s=" << static_cast<uint64_t>(throughput)
            << " latency_p50_ns=" << latencies[latencies.size() / 2]
            << " latency_p99_ns=" << latencies[latencies.size() * 99 / 100]
            << std::endl;
  Require(livox_ros::DeInitQueue(&queue));
}

}  // namespace

int main()
{
  CheckEmptyAndFull();
  RunStress("high_frequency", 200000, 0, 0);
  RunStress("slow_producer", 20000, 16, 0);
  RunStress("slow_consumer", 20000, 0, 16);
  return 0;
}
