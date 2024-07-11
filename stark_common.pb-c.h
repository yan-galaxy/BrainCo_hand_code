/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: stark_common.proto */

#ifndef PROTOBUF_C_stark_5fcommon_2eproto__INCLUDED
#define PROTOBUF_C_stark_5fcommon_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _GenericReq GenericReq;


/* --- enums --- */

typedef enum _LedMode {
  LED_MODE__LED_MODE_UNUSED = 0,
  LED_MODE__LED_MODE_SHUTDOWN = 1,
  LED_MODE__LED_MODE_KEEP = 2,
  /*
   *1Hz freq on-off cycle, 50% duty cycle
   */
  LED_MODE__LED_MODE_BLINK = 3,
  /*
   *on-100ms, then off, no repeat, if repeatedly receive this, time counter will be reset at each time received (e.g., repeatedly receive interval <= 100ms equal always on)
   */
  LED_MODE__LED_MODE_ONE_SHOT = 4,
  /*
   *0.5Hz freq on-off cycle, 50% duty cycle
   */
  LED_MODE__LED_MODE_BLINK0_5HZ = 5,
  /*
   *2Hz freq on-off cycle, 50% duty cycle
   */
  LED_MODE__LED_MODE_BLINK2HZ = 6
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(LED_MODE)
} LedMode;
typedef enum _LedColor {
  /*
   *not equal to OFF, since majorly use LedMode to turn-off LED.
   */
  LED_COLOR__LED_COLOR_UNCHANGED = 0,
  LED_COLOR__LED_COLOR_R = 1,
  LED_COLOR__LED_COLOR_G = 2,
  LED_COLOR__LED_COLOR_RG = 3,
  LED_COLOR__LED_COLOR_B = 4,
  LED_COLOR__LED_COLOR_RB = 5,
  LED_COLOR__LED_COLOR_GB = 6,
  LED_COLOR__LED_COLOR_RGB = 7
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(LED_COLOR)
} LedColor;
typedef enum _HandType {
  HAND_TYPE__HANDTYPE_UNUSED = 0,
  HAND_TYPE__REGULAR_RIGHT = 1,
  HAND_TYPE__REGULAR_LEFT = 2,
  HAND_TYPE__MINI_RIGHT = 3,
  HAND_TYPE__MINI_LEFT = 4
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(HAND_TYPE)
} HandType;
typedef enum _FingerId {
  /*
   *_AUX is auxiliary movement, all other is open-close
   */
  FINGER_ID__FINGERID_UNUSED = 0,
  FINGER_ID__FINGERID_THUMB = 1,
  FINGER_ID__FINGERID_THUMB_AUX = 2,
  FINGER_ID__FINGERID_INDEX = 3,
  FINGER_ID__FINGERID_MIDDLE = 4,
  FINGER_ID__FINGERID_RING = 5,
  FINGER_ID__FINGERID_PINKY = 6
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(FINGER_ID)
} FingerId;

/* --- messages --- */

/*
 *A generic message as a placeholder to represent most requests without detailed attributes. Since the union message only contains "message" type, not support other data types, so use GenericReq to wrap the request.
 */
struct  _GenericReq
{
  ProtobufCMessage base;
  /*
   * enum Magic
   * {
   * 	UNUSED = 0;
   * 	PERFORM_REQ = 110;//magic number
   * } // enum 大概率不会修改，协议有必要采取改，如果有更多的value可以换为常规request，建议保持bool
   * Magic req = 1;	//Must be true if desire to request remote device information.
   */
  protobuf_c_boolean req;
};
#define GENERIC_REQ__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&generic_req__descriptor) \
    , 0 }


/* GenericReq methods */
void   generic_req__init
                     (GenericReq         *message);
size_t generic_req__get_packed_size
                     (const GenericReq   *message);
size_t generic_req__pack
                     (const GenericReq   *message,
                      uint8_t             *out);
size_t generic_req__pack_to_buffer
                     (const GenericReq   *message,
                      ProtobufCBuffer     *buffer);
GenericReq *
       generic_req__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   generic_req__free_unpacked
                     (GenericReq *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*GenericReq_Closure)
                 (const GenericReq *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCEnumDescriptor    led_mode__descriptor;
extern const ProtobufCEnumDescriptor    led_color__descriptor;
extern const ProtobufCEnumDescriptor    hand_type__descriptor;
extern const ProtobufCEnumDescriptor    finger_id__descriptor;
extern const ProtobufCMessageDescriptor generic_req__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_stark_5fcommon_2eproto__INCLUDED */
