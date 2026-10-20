//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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
//

#include <stdio.h>
#include <string.h>

#include "ldq.h"

namespace livox_ros {

/* for pointcloud queue process */
bool InitQueue(LidarDataQueue *queue, uint32_t queue_size) {
  if (queue == nullptr) {
    // ROS_WARN("RosDriver Queue: Initialization failed - invalid queue.");
    return false;
  }

  if (!IsPowerOf2(queue_size)) {
    queue_size = RoundupPowerOf2(queue_size);
    printf("Init queue, real query size:%u.\n", queue_size);
  }

  if (queue->storage_packet) {
    delete[] queue->storage_packet;
    queue->storage_packet = nullptr;
  }

  queue->storage_packet = new StoragePacket[queue_size];
  if (queue->storage_packet == nullptr) {
    // ROS_WARN("RosDriver Queue: Initialization failed - failed to allocate memory.");
    return false;
  }

  queue->rd_idx.store(0, std::memory_order_relaxed);
  queue->wr_idx.store(0, std::memory_order_relaxed);
  queue->size = queue_size;
  queue->mask = queue_size - 1;

  return true;
}

bool DeInitQueue(LidarDataQueue *queue) {
  if (queue == nullptr) {
    // ROS_WARN("RosDriver Queue: Deinitialization failed - invalid queue.");
    return false;
  }

  if (queue->storage_packet) {
    delete[] queue->storage_packet;
  }

  queue->rd_idx.store(0, std::memory_order_relaxed);
  queue->wr_idx.store(0, std::memory_order_relaxed);
  queue->size = 0;
  queue->mask = 0;

  return true;
}

void ResetQueue(LidarDataQueue *queue) {
  queue->rd_idx.store(0, std::memory_order_relaxed);
  queue->wr_idx.store(0, std::memory_order_relaxed);
}

bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (queue == nullptr || storage_packet == nullptr) {
    // ROS_WARN("RosDriver Queue: Invalid pointer parameters.");
    return false;
  }

  const uint32_t rd_idx = queue->rd_idx.load(std::memory_order_relaxed);
  const uint32_t wr_idx = queue->wr_idx.load(std::memory_order_acquire);
  if (rd_idx == wr_idx) {
    // ROS_WARN("RosDriver Queue: Pop failed, since the queue is empty.");
    return false;
  }

  const uint32_t storage_idx = rd_idx & queue->mask;

  storage_packet->base_time = queue->storage_packet[storage_idx].base_time;
  storage_packet->points_num = queue->storage_packet[storage_idx].points_num;
  storage_packet->points.resize(queue->storage_packet[storage_idx].points_num);

  memcpy(storage_packet->points.data(), queue->storage_packet[storage_idx].points.data(), (storage_packet->points_num) * sizeof(PointXyzlt));
  return true;
}

void QueuePopUpdate(LidarDataQueue *queue) {
  const uint32_t rd_idx = queue->rd_idx.load(std::memory_order_relaxed);
  queue->rd_idx.store(rd_idx + 1, std::memory_order_release);
}

bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (!QueuePrePop(queue, storage_packet)) {
    return false;
  }
  QueuePopUpdate(queue);

  return true;
}

uint32_t QueueUsedSize(LidarDataQueue *queue) {
  const uint32_t rd_idx = queue->rd_idx.load(std::memory_order_acquire);
  const uint32_t wr_idx = queue->wr_idx.load(std::memory_order_acquire);
  return wr_idx - rd_idx;
}

uint32_t QueueUnusedSize(LidarDataQueue *queue) {
  return (queue->size - QueueUsedSize(queue));
}

bool QueueIsFull(LidarDataQueue *queue) {
  const uint32_t wr_idx = queue->wr_idx.load(std::memory_order_relaxed);
  const uint32_t rd_idx = queue->rd_idx.load(std::memory_order_acquire);
  return ((wr_idx - rd_idx) > queue->mask);
}

bool QueueIsEmpty(LidarDataQueue *queue) {
  const uint32_t rd_idx = queue->rd_idx.load(std::memory_order_acquire);
  const uint32_t wr_idx = queue->wr_idx.load(std::memory_order_acquire);
  return (rd_idx == wr_idx);
}

uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time) {
  const uint32_t wr_idx = queue->wr_idx.load(std::memory_order_relaxed);
  const uint32_t storage_idx = wr_idx & queue->mask;
  PointPacket* lidar_point_data = reinterpret_cast<PointPacket*>(data);
  queue->storage_packet[storage_idx].base_time = base_time;
  queue->storage_packet[storage_idx].points_num = lidar_point_data->points_num;

  queue->storage_packet[storage_idx].points.clear();
  queue->storage_packet[storage_idx].points.resize(lidar_point_data->points_num);
  memcpy(queue->storage_packet[storage_idx].points.data(), lidar_point_data->points, sizeof(PointXyzlt) * (lidar_point_data->points_num));

  queue->wr_idx.store(wr_idx + 1, std::memory_order_release);
  return 1;
}

}  // namespace livox_ros
