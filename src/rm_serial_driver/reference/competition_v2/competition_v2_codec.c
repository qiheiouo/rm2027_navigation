#include "competition_v2_codec.h"

#include <math.h>
#include <string.h>

#define RMCV2_PI_F 3.14159265358979323846f

static void put_u16(uint8_t * data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8u);
}

static void put_u32(uint8_t * data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8u);
  data[2] = (uint8_t)(value >> 16u);
  data[3] = (uint8_t)(value >> 24u);
}

static uint16_t get_u16(const uint8_t * data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8u));
}

static uint32_t get_u32(const uint8_t * data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8u) |
         ((uint32_t)data[2] << 16u) |
         ((uint32_t)data[3] << 24u);
}

static void put_f32(uint8_t * data, float value)
{
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  put_u32(data, bits);
}

static float get_f32(const uint8_t * data)
{
  const uint32_t bits = get_u32(data);
  float value = 0.0f;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static bool yaw_valid(float value)
{
  return isfinite(value) && value >= -RMCV2_PI_F && value < RMCV2_PI_F;
}

static bool type_and_size(const rmcv2_frame_t * frame, uint8_t type, uint16_t size)
{
  return frame != NULL && frame->version == RMCV2_VERSION &&
         frame->message_type == type && frame->payload_length == size;
}

uint16_t rmcv2_crc16_modbus(const uint8_t * data, size_t size)
{
  uint16_t crc = 0xffffu;
  size_t index;
  int bit;

  if (data == NULL && size != 0u) {
    return 0u;
  }
  for (index = 0; index < size; ++index) {
    crc ^= data[index];
    for (bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) != 0u ?
        (uint16_t)((crc >> 1u) ^ 0xa001u) : (uint16_t)(crc >> 1u);
    }
  }
  return crc;
}

bool rmcv2_posture_is_valid(uint8_t posture)
{
  return posture <= RMCV2_POSTURE_ENHANCED_MOVE;
}

bool rmcv2_sequence_is_newer(uint16_t candidate, uint16_t reference)
{
  const uint16_t delta = (uint16_t)(candidate - reference);
  return delta != 0u && delta < 0x8000u;
}

bool rmcv2_command_id_is_newer(uint32_t candidate, uint32_t reference)
{
  const uint32_t delta = candidate - reference;
  return delta != 0u && delta < UINT32_C(0x80000000);
}

rmcv2_result_t rmcv2_encode_frame(
  uint8_t message_type, uint16_t sequence, const uint8_t * payload,
  uint16_t payload_length, uint8_t * output, size_t output_capacity,
  size_t * output_size)
{
  const size_t total = RMCV2_HEADER_SIZE + (size_t)payload_length + RMCV2_CRC_SIZE;
  uint16_t crc;

  if (output == NULL || output_size == NULL ||
    (payload == NULL && payload_length != 0u) || payload_length > RMCV2_MAX_PAYLOAD_SIZE)
  {
    return RMCV2_INVALID_ARGUMENT;
  }
  if (output_capacity < total) {
    return RMCV2_OUTPUT_TOO_SMALL;
  }

  output[0] = RMCV2_MAGIC0;
  output[1] = RMCV2_MAGIC1;
  output[2] = RMCV2_VERSION;
  output[3] = message_type;
  put_u16(output + 4, payload_length);
  put_u16(output + 6, sequence);
  if (payload_length != 0u) {
    memcpy(output + RMCV2_HEADER_SIZE, payload, payload_length);
  }
  crc = rmcv2_crc16_modbus(output + 2, 6u + payload_length);
  put_u16(output + RMCV2_HEADER_SIZE + payload_length, crc);
  *output_size = total;
  return RMCV2_OK;
}

rmcv2_result_t rmcv2_decode_frame(
  const uint8_t * data, size_t size, rmcv2_frame_t * frame, size_t * consumed)
{
  uint16_t payload_length;
  uint16_t expected_crc;
  uint16_t received_crc;
  size_t total;

  if (data == NULL || frame == NULL || consumed == NULL) {
    return RMCV2_INVALID_ARGUMENT;
  }
  *consumed = 0;
  if (size < 2u) {
    return RMCV2_NEED_MORE;
  }
  if (data[0] != RMCV2_MAGIC0 || data[1] != RMCV2_MAGIC1) {
    *consumed = 1;
    return RMCV2_INVALID_FRAME;
  }
  if (size < RMCV2_HEADER_SIZE) {
    return RMCV2_NEED_MORE;
  }
  if (data[2] != RMCV2_VERSION) {
    *consumed = 1;
    return RMCV2_INVALID_FRAME;
  }

  payload_length = get_u16(data + 4);
  if (payload_length > RMCV2_MAX_PAYLOAD_SIZE) {
    *consumed = 1;
    return RMCV2_INVALID_FRAME;
  }
  total = RMCV2_HEADER_SIZE + payload_length + RMCV2_CRC_SIZE;
  if (size < total) {
    return RMCV2_NEED_MORE;
  }

  expected_crc = rmcv2_crc16_modbus(data + 2, 6u + payload_length);
  received_crc = get_u16(data + RMCV2_HEADER_SIZE + payload_length);
  if (expected_crc != received_crc) {
    *consumed = 1;
    return RMCV2_INVALID_FRAME;
  }

  frame->version = data[2];
  frame->message_type = data[3];
  frame->sequence = get_u16(data + 6);
  frame->payload_length = payload_length;
  if (payload_length != 0u) {
    memcpy(frame->payload, data + RMCV2_HEADER_SIZE, payload_length);
  }
  *consumed = total;
  return RMCV2_OK;
}

rmcv2_result_t rmcv2_encode_chassis_command(
  const rmcv2_chassis_command_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[13];
  if (message == NULL || !isfinite(message->vx_mps) ||
    !isfinite(message->vy_mps) || !isfinite(message->wz_rad_s))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  put_f32(payload, message->vx_mps);
  put_f32(payload + 4, message->vy_mps);
  put_f32(payload + 8, message->wz_rad_s);
  payload[12] = message->enabled ? 1u : 0u;
  return rmcv2_encode_frame(
    RMCV2_MSG_CHASSIS_COMMAND, sequence, payload, 13, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_chassis_command(
  const rmcv2_frame_t * frame, rmcv2_chassis_command_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_CHASSIS_COMMAND, 13)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->vx_mps = get_f32(frame->payload);
  message->vy_mps = get_f32(frame->payload + 4);
  message->wz_rad_s = get_f32(frame->payload + 8);
  message->enabled = (frame->payload[12] & 1u) != 0u;
  return isfinite(message->vx_mps) && isfinite(message->vy_mps) &&
         isfinite(message->wz_rad_s) ? RMCV2_OK : RMCV2_INVALID_PAYLOAD;
}

rmcv2_result_t rmcv2_encode_posture_request(
  const rmcv2_posture_request_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[11];
  if (message == NULL || !rmcv2_posture_is_valid(message->requested_posture)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  put_u32(payload, message->command_id);
  put_u32(payload + 4, message->requester_boot_id);
  payload[8] = message->requested_posture;
  payload[9] = message->valid ? 1u : 0u;
  payload[10] = message->request_flags;
  return rmcv2_encode_frame(
    RMCV2_MSG_POSTURE_REQUEST, sequence, payload, 11, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_posture_request(
  const rmcv2_frame_t * frame, rmcv2_posture_request_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_POSTURE_REQUEST, 11) ||
    !rmcv2_posture_is_valid(frame->payload[8]))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->command_id = get_u32(frame->payload);
  message->requester_boot_id = get_u32(frame->payload + 4);
  message->requested_posture = frame->payload[8];
  message->valid = (frame->payload[9] & 1u) != 0u;
  message->request_flags = frame->payload[10];
  return RMCV2_OK;
}

rmcv2_result_t rmcv2_encode_posture_state(
  const rmcv2_posture_state_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[13];
  uint8_t flags = 0;
  if (message == NULL || !rmcv2_posture_is_valid(message->posture_target) ||
    !rmcv2_posture_is_valid(message->posture_actual))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  put_u32(payload, message->ack_command_id);
  put_u32(payload + 4, message->ack_requester_boot_id);
  payload[8] = message->posture_target;
  payload[9] = message->posture_actual;
  if (message->valid) {
    flags |= 1u;
  }
  if (message->online) {
    flags |= 2u;
  }
  if (message->transitioning) {
    flags |= 4u;
  }
  if (message->fault) {
    flags |= 8u;
  }
  payload[10] = flags;
  put_u16(payload + 11, message->fault_code);
  return rmcv2_encode_frame(
    RMCV2_MSG_POSTURE_STATE, sequence, payload, 13, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_posture_state(
  const rmcv2_frame_t * frame, rmcv2_posture_state_t * message)
{
  uint8_t flags;
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_POSTURE_STATE, 13) ||
    !rmcv2_posture_is_valid(frame->payload[8]) ||
    !rmcv2_posture_is_valid(frame->payload[9]))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->ack_command_id = get_u32(frame->payload);
  message->ack_requester_boot_id = get_u32(frame->payload + 4);
  message->posture_target = frame->payload[8];
  message->posture_actual = frame->payload[9];
  flags = frame->payload[10];
  message->valid = (flags & 1u) != 0u;
  message->online = (flags & 2u) != 0u;
  message->transitioning = (flags & 4u) != 0u;
  message->fault = (flags & 8u) != 0u;
  message->fault_code = get_u16(frame->payload + 11);
  return RMCV2_OK;
}

rmcv2_result_t rmcv2_encode_gimbal_state(
  const rmcv2_gimbal_state_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[17];
  if (message == NULL || !yaw_valid(message->relative_yaw_rad) ||
    !isfinite(message->yaw_rate_rad_s))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  put_f32(payload, message->relative_yaw_rad);
  put_f32(payload + 4, message->yaw_rate_rad_s);
  put_u32(payload + 8, message->sample_sequence);
  put_u32(payload + 12, message->mcu_time_ms);
  payload[16] = (message->valid ? 1u : 0u) | (message->online ? 2u : 0u);
  return rmcv2_encode_frame(
    RMCV2_MSG_GIMBAL_STATE, sequence, payload, 17, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_gimbal_state(
  const rmcv2_frame_t * frame, rmcv2_gimbal_state_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_GIMBAL_STATE, 17)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->relative_yaw_rad = get_f32(frame->payload);
  message->yaw_rate_rad_s = get_f32(frame->payload + 4);
  message->sample_sequence = get_u32(frame->payload + 8);
  message->mcu_time_ms = get_u32(frame->payload + 12);
  message->valid = (frame->payload[16] & 1u) != 0u;
  message->online = (frame->payload[16] & 2u) != 0u;
  return yaw_valid(message->relative_yaw_rad) && isfinite(message->yaw_rate_rad_s) ?
         RMCV2_OK : RMCV2_INVALID_PAYLOAD;
}

rmcv2_result_t rmcv2_encode_referee_state(
  const rmcv2_referee_state_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[17];
  if (message == NULL) {
    return RMCV2_INVALID_ARGUMENT;
  }
  payload[0] = message->game_progress;
  put_u16(payload + 1, message->stage_remain_time);
  payload[3] = message->robot_id;
  put_u16(payload + 4, message->current_hp);
  put_u16(payload + 6, message->red_outpost_hp);
  put_u16(payload + 8, message->blue_outpost_hp);
  put_u16(payload + 10, message->projectile_allowance_17mm);
  put_u16(payload + 12, message->remaining_gold_coin);
  put_u16(payload + 14, message->diagnostic_flags);
  payload[16] = message->valid ? 1u : 0u;
  return rmcv2_encode_frame(
    RMCV2_MSG_REFEREE_STATE, sequence, payload, 17, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_referee_state(
  const rmcv2_frame_t * frame, rmcv2_referee_state_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_REFEREE_STATE, 17)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->game_progress = frame->payload[0];
  message->stage_remain_time = get_u16(frame->payload + 1);
  message->robot_id = frame->payload[3];
  message->current_hp = get_u16(frame->payload + 4);
  message->red_outpost_hp = get_u16(frame->payload + 6);
  message->blue_outpost_hp = get_u16(frame->payload + 8);
  message->projectile_allowance_17mm = get_u16(frame->payload + 10);
  message->remaining_gold_coin = get_u16(frame->payload + 12);
  message->diagnostic_flags = get_u16(frame->payload + 14);
  message->valid = (frame->payload[16] & 1u) != 0u;
  return RMCV2_OK;
}

rmcv2_result_t rmcv2_encode_operator_target(
  const rmcv2_operator_target_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[25];
  if (message == NULL || message->coordinate_frame > RMCV2_FRAME_MAP ||
    message->alliance > RMCV2_ALLIANCE_BLUE || !isfinite(message->x_m) ||
    !isfinite(message->y_m) || (message->has_yaw && !yaw_valid(message->yaw_rad)))
  {
    return RMCV2_INVALID_PAYLOAD;
  }
  put_u32(payload, message->command_id);
  payload[4] = message->coordinate_frame;
  payload[5] = message->alliance;
  payload[6] = (message->valid ? 1u : 0u) | (message->has_yaw ? 2u : 0u);
  put_f32(payload + 7, message->x_m);
  put_f32(payload + 11, message->y_m);
  put_f32(payload + 15, message->has_yaw ? message->yaw_rad : 0.0f);
  put_u32(payload + 19, message->sample_time_ms);
  put_u16(payload + 23, message->ttl_ms);
  return rmcv2_encode_frame(
    RMCV2_MSG_OPERATOR_TARGET, sequence, payload, 25, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_operator_target(
  const rmcv2_frame_t * frame, rmcv2_operator_target_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_OPERATOR_TARGET, 25)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->command_id = get_u32(frame->payload);
  message->coordinate_frame = frame->payload[4];
  message->alliance = frame->payload[5];
  message->valid = (frame->payload[6] & 1u) != 0u;
  message->has_yaw = (frame->payload[6] & 2u) != 0u;
  message->x_m = get_f32(frame->payload + 7);
  message->y_m = get_f32(frame->payload + 11);
  message->yaw_rad = get_f32(frame->payload + 15);
  message->sample_time_ms = get_u32(frame->payload + 19);
  message->ttl_ms = get_u16(frame->payload + 23);
  return message->coordinate_frame <= RMCV2_FRAME_MAP &&
         message->alliance <= RMCV2_ALLIANCE_BLUE && isfinite(message->x_m) &&
         isfinite(message->y_m) && (!message->has_yaw || yaw_valid(message->yaw_rad)) ?
         RMCV2_OK : RMCV2_INVALID_PAYLOAD;
}

rmcv2_result_t rmcv2_encode_heartbeat(
  const rmcv2_heartbeat_t * message, uint16_t sequence,
  uint8_t * output, size_t capacity, size_t * size)
{
  uint8_t payload[13];
  if (message == NULL) {
    return RMCV2_INVALID_ARGUMENT;
  }
  put_u32(payload, message->uptime_ms);
  put_u32(payload + 4, message->boot_id);
  put_u32(payload + 8, message->capabilities);
  payload[12] = message->ready ? 1u : 0u;
  return rmcv2_encode_frame(
    RMCV2_MSG_HEARTBEAT, sequence, payload, 13, output, capacity, size);
}

rmcv2_result_t rmcv2_decode_heartbeat(
  const rmcv2_frame_t * frame, rmcv2_heartbeat_t * message)
{
  if (message == NULL || !type_and_size(frame, RMCV2_MSG_HEARTBEAT, 13)) {
    return RMCV2_INVALID_PAYLOAD;
  }
  message->uptime_ms = get_u32(frame->payload);
  message->boot_id = get_u32(frame->payload + 4);
  message->capabilities = get_u32(frame->payload + 8);
  message->ready = (frame->payload[12] & 1u) != 0u;
  return RMCV2_OK;
}
