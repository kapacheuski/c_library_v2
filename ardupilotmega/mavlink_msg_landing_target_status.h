#pragma once
// MESSAGE LANDING_TARGET_STATUS PACKING

#define MAVLINK_MSG_ID_LANDING_TARGET_STATUS 11150

MAVPACKED(
typedef struct __mavlink_landing_target_status_t {
 uint64_t time_usec; /*< [us] Timestamp (micros since boot or Unix epoch)*/
 float offset_x; /*< [m] X-axis offset of the target from the center of vehicle*/
 float offset_y; /*< [m] Y-axis offset of the target from the center of vehicle*/
 float distance; /*< [m] Distance to the target from the center of vehicle*/
 uint8_t target_num; /*<  The ID of the target if multiple targets are present*/
 uint8_t frame; /*<  MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.*/
 uint8_t type; /*<  LANDING_TARGET_TYPE enum specifying the type of landing target*/
 uint8_t trust_level; /*<  Trust level of position (100 max) or default unkown position (0), for validation of positioning of the landing target*/
}) mavlink_landing_target_status_t;

#define MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN 24
#define MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN 24
#define MAVLINK_MSG_ID_11150_LEN 24
#define MAVLINK_MSG_ID_11150_MIN_LEN 24

#define MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC 94
#define MAVLINK_MSG_ID_11150_CRC 94



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LANDING_TARGET_STATUS { \
    11150, \
    "LANDING_TARGET_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_landing_target_status_t, time_usec) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_landing_target_status_t, target_num) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_landing_target_status_t, frame) }, \
         { "offset_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_landing_target_status_t, offset_x) }, \
         { "offset_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_landing_target_status_t, offset_y) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_landing_target_status_t, distance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_landing_target_status_t, type) }, \
         { "trust_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_landing_target_status_t, trust_level) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LANDING_TARGET_STATUS { \
    "LANDING_TARGET_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_landing_target_status_t, time_usec) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_landing_target_status_t, target_num) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_landing_target_status_t, frame) }, \
         { "offset_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_landing_target_status_t, offset_x) }, \
         { "offset_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_landing_target_status_t, offset_y) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_landing_target_status_t, distance) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_landing_target_status_t, type) }, \
         { "trust_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_landing_target_status_t, trust_level) }, \
         } \
}
#endif

/**
 * @brief Pack a landing_target_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (micros since boot or Unix epoch)
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param offset_x [m] X-axis offset of the target from the center of vehicle
 * @param offset_y [m] Y-axis offset of the target from the center of vehicle
 * @param distance [m] Distance to the target from the center of vehicle
 * @param type  LANDING_TARGET_TYPE enum specifying the type of landing target
 * @param trust_level  Trust level of position (100 max) or default unkown position (0), for validation of positioning of the landing target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t target_num, uint8_t frame, float offset_x, float offset_y, float distance, uint8_t type, uint8_t trust_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, offset_x);
    _mav_put_float(buf, 12, offset_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_uint8_t(buf, 20, target_num);
    _mav_put_uint8_t(buf, 21, frame);
    _mav_put_uint8_t(buf, 22, type);
    _mav_put_uint8_t(buf, 23, trust_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN);
#else
    mavlink_landing_target_status_t packet;
    packet.time_usec = time_usec;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.distance = distance;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.type = type;
    packet.trust_level = trust_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
}

/**
 * @brief Pack a landing_target_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (micros since boot or Unix epoch)
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param offset_x [m] X-axis offset of the target from the center of vehicle
 * @param offset_y [m] Y-axis offset of the target from the center of vehicle
 * @param distance [m] Distance to the target from the center of vehicle
 * @param type  LANDING_TARGET_TYPE enum specifying the type of landing target
 * @param trust_level  Trust level of position (100 max) or default unkown position (0), for validation of positioning of the landing target
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_landing_target_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t target_num,uint8_t frame,float offset_x,float offset_y,float distance,uint8_t type,uint8_t trust_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, offset_x);
    _mav_put_float(buf, 12, offset_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_uint8_t(buf, 20, target_num);
    _mav_put_uint8_t(buf, 21, frame);
    _mav_put_uint8_t(buf, 22, type);
    _mav_put_uint8_t(buf, 23, trust_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN);
#else
    mavlink_landing_target_status_t packet;
    packet.time_usec = time_usec;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.distance = distance;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.type = type;
    packet.trust_level = trust_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LANDING_TARGET_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
}

/**
 * @brief Encode a landing_target_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param landing_target_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_landing_target_status_t* landing_target_status)
{
    return mavlink_msg_landing_target_status_pack(system_id, component_id, msg, landing_target_status->time_usec, landing_target_status->target_num, landing_target_status->frame, landing_target_status->offset_x, landing_target_status->offset_y, landing_target_status->distance, landing_target_status->type, landing_target_status->trust_level);
}

/**
 * @brief Encode a landing_target_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param landing_target_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_landing_target_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_landing_target_status_t* landing_target_status)
{
    return mavlink_msg_landing_target_status_pack_chan(system_id, component_id, chan, msg, landing_target_status->time_usec, landing_target_status->target_num, landing_target_status->frame, landing_target_status->offset_x, landing_target_status->offset_y, landing_target_status->distance, landing_target_status->type, landing_target_status->trust_level);
}

/**
 * @brief Send a landing_target_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (micros since boot or Unix epoch)
 * @param target_num  The ID of the target if multiple targets are present
 * @param frame  MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 * @param offset_x [m] X-axis offset of the target from the center of vehicle
 * @param offset_y [m] Y-axis offset of the target from the center of vehicle
 * @param distance [m] Distance to the target from the center of vehicle
 * @param type  LANDING_TARGET_TYPE enum specifying the type of landing target
 * @param trust_level  Trust level of position (100 max) or default unkown position (0), for validation of positioning of the landing target
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_landing_target_status_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_num, uint8_t frame, float offset_x, float offset_y, float distance, uint8_t type, uint8_t trust_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, offset_x);
    _mav_put_float(buf, 12, offset_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_uint8_t(buf, 20, target_num);
    _mav_put_uint8_t(buf, 21, frame);
    _mav_put_uint8_t(buf, 22, type);
    _mav_put_uint8_t(buf, 23, trust_level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS, buf, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
#else
    mavlink_landing_target_status_t packet;
    packet.time_usec = time_usec;
    packet.offset_x = offset_x;
    packet.offset_y = offset_y;
    packet.distance = distance;
    packet.target_num = target_num;
    packet.frame = frame;
    packet.type = type;
    packet.trust_level = trust_level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS, (const char *)&packet, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
#endif
}

/**
 * @brief Send a landing_target_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_landing_target_status_send_struct(mavlink_channel_t chan, const mavlink_landing_target_status_t* landing_target_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_landing_target_status_send(chan, landing_target_status->time_usec, landing_target_status->target_num, landing_target_status->frame, landing_target_status->offset_x, landing_target_status->offset_y, landing_target_status->distance, landing_target_status->type, landing_target_status->trust_level);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS, (const char *)landing_target_status, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_landing_target_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_num, uint8_t frame, float offset_x, float offset_y, float distance, uint8_t type, uint8_t trust_level)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, offset_x);
    _mav_put_float(buf, 12, offset_y);
    _mav_put_float(buf, 16, distance);
    _mav_put_uint8_t(buf, 20, target_num);
    _mav_put_uint8_t(buf, 21, frame);
    _mav_put_uint8_t(buf, 22, type);
    _mav_put_uint8_t(buf, 23, trust_level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS, buf, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
#else
    mavlink_landing_target_status_t *packet = (mavlink_landing_target_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->offset_x = offset_x;
    packet->offset_y = offset_y;
    packet->distance = distance;
    packet->target_num = target_num;
    packet->frame = frame;
    packet->type = type;
    packet->trust_level = trust_level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LANDING_TARGET_STATUS, (const char *)packet, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE LANDING_TARGET_STATUS UNPACKING


/**
 * @brief Get field time_usec from landing_target_status message
 *
 * @return [us] Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_landing_target_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_num from landing_target_status message
 *
 * @return  The ID of the target if multiple targets are present
 */
static inline uint8_t mavlink_msg_landing_target_status_get_target_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field frame from landing_target_status message
 *
 * @return  MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
 */
static inline uint8_t mavlink_msg_landing_target_status_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field offset_x from landing_target_status message
 *
 * @return [m] X-axis offset of the target from the center of vehicle
 */
static inline float mavlink_msg_landing_target_status_get_offset_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field offset_y from landing_target_status message
 *
 * @return [m] Y-axis offset of the target from the center of vehicle
 */
static inline float mavlink_msg_landing_target_status_get_offset_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field distance from landing_target_status message
 *
 * @return [m] Distance to the target from the center of vehicle
 */
static inline float mavlink_msg_landing_target_status_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field type from landing_target_status message
 *
 * @return  LANDING_TARGET_TYPE enum specifying the type of landing target
 */
static inline uint8_t mavlink_msg_landing_target_status_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field trust_level from landing_target_status message
 *
 * @return  Trust level of position (100 max) or default unkown position (0), for validation of positioning of the landing target
 */
static inline uint8_t mavlink_msg_landing_target_status_get_trust_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Decode a landing_target_status message into a struct
 *
 * @param msg The message to decode
 * @param landing_target_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_landing_target_status_decode(const mavlink_message_t* msg, mavlink_landing_target_status_t* landing_target_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    landing_target_status->time_usec = mavlink_msg_landing_target_status_get_time_usec(msg);
    landing_target_status->offset_x = mavlink_msg_landing_target_status_get_offset_x(msg);
    landing_target_status->offset_y = mavlink_msg_landing_target_status_get_offset_y(msg);
    landing_target_status->distance = mavlink_msg_landing_target_status_get_distance(msg);
    landing_target_status->target_num = mavlink_msg_landing_target_status_get_target_num(msg);
    landing_target_status->frame = mavlink_msg_landing_target_status_get_frame(msg);
    landing_target_status->type = mavlink_msg_landing_target_status_get_type(msg);
    landing_target_status->trust_level = mavlink_msg_landing_target_status_get_trust_level(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN? msg->len : MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN;
        memset(landing_target_status, 0, MAVLINK_MSG_ID_LANDING_TARGET_STATUS_LEN);
    memcpy(landing_target_status, _MAV_PAYLOAD(msg), len);
#endif
}
