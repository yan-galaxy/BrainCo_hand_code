/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: main_mtr_shared.proto */

#ifndef PROTOBUF_C_main_5fmtr_5fshared_2eproto__INCLUDED
#define PROTOBUF_C_main_5fmtr_5fshared_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _FingerStatus FingerStatus;


/* --- enums --- */

typedef enum _FingerStatusSampleRate {
  FINGER_STATUS_SAMPLE_RATE__RATE_UNUSED = 0,
  FINGER_STATUS_SAMPLE_RATE__RATE_1MS = 1,
  FINGER_STATUS_SAMPLE_RATE__RATE_2MS = 2,
  FINGER_STATUS_SAMPLE_RATE__RATE_5MS = 5,
  FINGER_STATUS_SAMPLE_RATE__RATE_10MS = 10,
  FINGER_STATUS_SAMPLE_RATE__RATE_20MS = 20,
  FINGER_STATUS_SAMPLE_RATE__RATE_50MS = 50,
  FINGER_STATUS_SAMPLE_RATE__RATE_100MS = 100,
  FINGER_STATUS_SAMPLE_RATE__RATE_200MS = 200,
  FINGER_STATUS_SAMPLE_RATE__RATE_500MS = 500,
  FINGER_STATUS_SAMPLE_RATE__RATE_1000MS = 1000,
  FINGER_STATUS_SAMPLE_RATE__UPON_EVENT = 16382,
  /*
   *max value if encode in 2 bytes, base-128
   */
  FINGER_STATUS_SAMPLE_RATE__RATE_OFF = 16383
    PROTOBUF_C__FORCE_ENUM_TO_BE_INT_SIZE(FINGER_STATUS_SAMPLE_RATE)
} FingerStatusSampleRate;

/* --- messages --- */

struct  _FingerStatus
{
  ProtobufCMessage base;
  /*
   **
   * positions:
   * 0 ~ 100 for each finger. 0 for fully open, 100 for fully close
   */
  ProtobufCBinaryData positions;
  /*
   **
   * speeds:
   * -100 ~ +100 for each finger. 0 for stop, positive number for close finger,
   * negative number for open finger.
   */
  ProtobufCBinaryData speeds;
  /*
   **
   * currents:
   * 0 ~ 100 for each finger. Fraction of max motor current, absolute number.
   * The max motor current is 600mA, in a word, 100.
   */
  ProtobufCBinaryData currents;
};
#define FINGER_STATUS__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&finger_status__descriptor) \
    , {0,NULL}, {0,NULL}, {0,NULL} }


/* FingerStatus methods */
void   finger_status__init
                     (FingerStatus         *message);
size_t finger_status__get_packed_size
                     (const FingerStatus   *message);
size_t finger_status__pack
                     (const FingerStatus   *message,
                      uint8_t             *out);
size_t finger_status__pack_to_buffer
                     (const FingerStatus   *message,
                      ProtobufCBuffer     *buffer);
FingerStatus *
       finger_status__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   finger_status__free_unpacked
                     (FingerStatus *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*FingerStatus_Closure)
                 (const FingerStatus *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCEnumDescriptor    finger_status_sample_rate__descriptor;
extern const ProtobufCMessageDescriptor finger_status__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_main_5fmtr_5fshared_2eproto__INCLUDED */
