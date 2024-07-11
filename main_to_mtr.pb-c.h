/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: main_to_mtr.proto */

#ifndef PROTOBUF_C_main_5fto_5fmtr_2eproto__INCLUDED
#define PROTOBUF_C_main_5fto_5fmtr_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif

#include "main_mtr_shared.pb-c.h"
#include "stark_common.pb-c.h"

typedef struct _MtrCmd MtrCmd;
typedef struct _LedCmd LedCmd;
typedef struct _UMainToMtr UMainToMtr;


/* --- enums --- */

typedef enum _MtrCmd__CtrlMode {
  MTR_CMD__CTRL_MODE__UNUSED = 0,
  MTR_CMD__CTRL_MODE__POSITION = 1,
  MTR_CMD__CTRL_MODE__SPEED = 2,
  MTR_CMD__CTRL_MODE__CURRENT = 3,
  MTR_CMD__CTRL_MODE__SUBSCRIBE = 127
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(MTR_CMD__CTRL_MODE)
} MtrCmd__CtrlMode;

/* --- messages --- */

/*
 *Motor Command
 */
struct  _MtrCmd
{
  ProtobufCMessage base;
  MtrCmd__CtrlMode ctrl_mode;
  FingerStatus *expect_status;
  FingerStatusSampleRate sample_rate;
  protobuf_c_boolean lock_fingers;
};
#define MTR_CMD__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&mtr_cmd__descriptor) \
    , MTR_CMD__CTRL_MODE__UNUSED, NULL, FINGER_STATUS_SAMPLE_RATE__RATE_UNUSED, 0 }


/*
 *LED Command
 */
struct  _LedCmd
{
  ProtobufCMessage base;
  /*
   *required field
   */
  LedMode set_mode;
  /*
   *optional field, and if = 0 mean unchanged
   */
  LedColor set_color;
};
#define LED_CMD__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&led_cmd__descriptor) \
    , LED_MODE__LED_MODE_UNUSED, LED_COLOR__LED_COLOR_UNCHANGED }


/*
 * Command Message: main_board --> driver_board
 */
struct  _UMainToMtr
{
  ProtobufCMessage base;
  MtrCmd *mtr_cmd;
  LedCmd *led_cmd;
};
#define U__MAIN_TO_MTR__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&u__main_to_mtr__descriptor) \
    , NULL, NULL }


/* MtrCmd methods */
void   mtr_cmd__init
                     (MtrCmd         *message);
size_t mtr_cmd__get_packed_size
                     (const MtrCmd   *message);
size_t mtr_cmd__pack
                     (const MtrCmd   *message,
                      uint8_t             *out);
size_t mtr_cmd__pack_to_buffer
                     (const MtrCmd   *message,
                      ProtobufCBuffer     *buffer);
MtrCmd *
       mtr_cmd__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   mtr_cmd__free_unpacked
                     (MtrCmd *message,
                      ProtobufCAllocator *allocator);
/* LedCmd methods */
void   led_cmd__init
                     (LedCmd         *message);
size_t led_cmd__get_packed_size
                     (const LedCmd   *message);
size_t led_cmd__pack
                     (const LedCmd   *message,
                      uint8_t             *out);
size_t led_cmd__pack_to_buffer
                     (const LedCmd   *message,
                      ProtobufCBuffer     *buffer);
LedCmd *
       led_cmd__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   led_cmd__free_unpacked
                     (LedCmd *message,
                      ProtobufCAllocator *allocator);
/* UMainToMtr methods */
void   u__main_to_mtr__init
                     (UMainToMtr         *message);
size_t u__main_to_mtr__get_packed_size
                     (const UMainToMtr   *message);
size_t u__main_to_mtr__pack
                     (const UMainToMtr   *message,
                      uint8_t             *out);
size_t u__main_to_mtr__pack_to_buffer
                     (const UMainToMtr   *message,
                      ProtobufCBuffer     *buffer);
UMainToMtr *
       u__main_to_mtr__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   u__main_to_mtr__free_unpacked
                     (UMainToMtr *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*MtrCmd_Closure)
                 (const MtrCmd *message,
                  void *closure_data);
typedef void (*LedCmd_Closure)
                 (const LedCmd *message,
                  void *closure_data);
typedef void (*UMainToMtr_Closure)
                 (const UMainToMtr *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor mtr_cmd__descriptor;
extern const ProtobufCEnumDescriptor    mtr_cmd__ctrl_mode__descriptor;
extern const ProtobufCMessageDescriptor led_cmd__descriptor;
extern const ProtobufCMessageDescriptor u__main_to_mtr__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_main_5fto_5fmtr_2eproto__INCLUDED */