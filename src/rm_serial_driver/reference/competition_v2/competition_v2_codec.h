#ifndef RMCV2_CODEC_H
#define RMCV2_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RMCV2_MAGIC0 UINT8_C(0x52)
#define RMCV2_MAGIC1 UINT8_C(0x4d)
#define RMCV2_VERSION UINT8_C(2)
#define RMCV2_HEADER_SIZE ((size_t)8)
#define RMCV2_CRC_SIZE ((size_t)2)
#define RMCV2_MAX_PAYLOAD_SIZE ((size_t)128)
#define RMCV2_MAX_FRAME_SIZE (RMCV2_HEADER_SIZE + RMCV2_MAX_PAYLOAD_SIZE + RMCV2_CRC_SIZE)

typedef enum {
  RMCV2_MSG_CHASSIS_COMMAND = 0x01,
  RMCV2_MSG_POSTURE_REQUEST = 0x02,
  RMCV2_MSG_HEARTBEAT = 0x03,
  RMCV2_MSG_POSTURE_STATE = 0x81,
  RMCV2_MSG_GIMBAL_STATE = 0x82,
  RMCV2_MSG_REFEREE_STATE = 0x83,
  RMCV2_MSG_OPERATOR_TARGET = 0x84
} rmcv2_message_type_t;

typedef enum {
  RMCV2_POSTURE_ATTACK = 0,
  RMCV2_POSTURE_MOVE = 1,
  RMCV2_POSTURE_DEFENSE = 2,
  RMCV2_POSTURE_ENHANCED_ATTACK = 3,
  RMCV2_POSTURE_ENHANCED_DEFENSE = 4,
  RMCV2_POSTURE_ENHANCED_MOVE = 5
} rmcv2_posture_t;

typedef enum {
  RMCV2_FRAME_UNKNOWN = 0,
  RMCV2_FRAME_REFEREE_FIELD = 1,
  RMCV2_FRAME_MAP = 2
} rmcv2_coordinate_frame_t;

typedef enum {
  RMCV2_ALLIANCE_UNKNOWN = 0,
  RMCV2_ALLIANCE_RED = 1,
  RMCV2_ALLIANCE_BLUE = 2
} rmcv2_alliance_t;

enum {
  RMCV2_CAP_CHASSIS_COMMAND = 1u << 0u,
  RMCV2_CAP_POSTURE = 1u << 1u,
  RMCV2_CAP_GIMBAL_STATE = 1u << 2u,
  RMCV2_CAP_REFEREE_STATE = 1u << 3u,
  RMCV2_CAP_OPERATOR_TARGET = 1u << 4u
};

typedef enum {
  RMCV2_OK = 0,
  RMCV2_NEED_MORE = 1,
  RMCV2_INVALID_ARGUMENT = -1,
  RMCV2_INVALID_FRAME = -2,
  RMCV2_INVALID_PAYLOAD = -3,
  RMCV2_OUTPUT_TOO_SMALL = -4
} rmcv2_result_t;

typedef struct {
  uint8_t version;
  uint8_t message_type;
  uint16_t sequence;
  uint16_t payload_length;
  uint8_t payload[RMCV2_MAX_PAYLOAD_SIZE];
} rmcv2_frame_t;

typedef struct {float vx_mps, vy_mps, wz_rad_s; bool enabled;} rmcv2_chassis_command_t;
typedef struct {
  uint32_t command_id;
  uint32_t requester_boot_id;
  uint8_t requested_posture;
  bool valid;
  uint8_t request_flags;
} rmcv2_posture_request_t;
typedef struct {
  uint32_t ack_command_id;
  uint32_t ack_requester_boot_id;
  uint8_t posture_target;
  uint8_t posture_actual;
  bool valid, online, transitioning, fault;
  uint16_t fault_code;
} rmcv2_posture_state_t;
typedef struct {
  float relative_yaw_rad, yaw_rate_rad_s;
  uint32_t sample_sequence, mcu_time_ms;
  bool valid, online;
} rmcv2_gimbal_state_t;
typedef struct {
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint8_t robot_id;
  uint16_t current_hp, red_outpost_hp, blue_outpost_hp;
  uint16_t projectile_allowance_17mm, remaining_gold_coin, diagnostic_flags;
  bool valid;
} rmcv2_referee_state_t;
typedef struct {
  uint32_t command_id;
  uint8_t coordinate_frame, alliance;
  float x_m, y_m, yaw_rad;
  uint32_t sample_time_ms;
  uint16_t ttl_ms;
  bool valid, has_yaw;
} rmcv2_operator_target_t;
typedef struct {
  uint32_t uptime_ms, boot_id, capabilities;
  bool ready;
} rmcv2_heartbeat_t;

uint16_t rmcv2_crc16_modbus(const uint8_t *data, size_t size);
bool rmcv2_posture_is_valid(uint8_t posture);
bool rmcv2_sequence_is_newer(uint16_t candidate, uint16_t reference);
bool rmcv2_command_id_is_newer(uint32_t candidate, uint32_t reference);

rmcv2_result_t rmcv2_encode_frame(
  uint8_t message_type, uint16_t sequence, const uint8_t *payload,
  uint16_t payload_length, uint8_t *output, size_t output_capacity, size_t *output_size);
rmcv2_result_t rmcv2_decode_frame(
  const uint8_t *data, size_t size, rmcv2_frame_t *frame, size_t *consumed);

rmcv2_result_t rmcv2_encode_chassis_command(
  const rmcv2_chassis_command_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_chassis_command(
  const rmcv2_frame_t *frame, rmcv2_chassis_command_t *message);
rmcv2_result_t rmcv2_encode_posture_request(
  const rmcv2_posture_request_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_posture_request(
  const rmcv2_frame_t *frame, rmcv2_posture_request_t *message);
rmcv2_result_t rmcv2_encode_posture_state(
  const rmcv2_posture_state_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_posture_state(
  const rmcv2_frame_t *frame, rmcv2_posture_state_t *message);
rmcv2_result_t rmcv2_encode_gimbal_state(
  const rmcv2_gimbal_state_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_gimbal_state(
  const rmcv2_frame_t *frame, rmcv2_gimbal_state_t *message);
rmcv2_result_t rmcv2_encode_referee_state(
  const rmcv2_referee_state_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_referee_state(
  const rmcv2_frame_t *frame, rmcv2_referee_state_t *message);
rmcv2_result_t rmcv2_encode_operator_target(
  const rmcv2_operator_target_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_operator_target(
  const rmcv2_frame_t *frame, rmcv2_operator_target_t *message);
rmcv2_result_t rmcv2_encode_heartbeat(
  const rmcv2_heartbeat_t *message, uint16_t sequence,
  uint8_t *output, size_t capacity, size_t *size);
rmcv2_result_t rmcv2_decode_heartbeat(
  const rmcv2_frame_t *frame, rmcv2_heartbeat_t *message);

#ifdef __cplusplus
}
#endif

#endif
