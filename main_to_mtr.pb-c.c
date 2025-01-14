/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: main_to_mtr.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "main_to_mtr.pb-c.h"
void   mtr_cmd__init
                     (MtrCmd         *message)
{
  static const MtrCmd init_value = MTR_CMD__INIT;
  *message = init_value;
}
size_t mtr_cmd__get_packed_size
                     (const MtrCmd *message)
{
  assert(message->base.descriptor == &mtr_cmd__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t mtr_cmd__pack
                     (const MtrCmd *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &mtr_cmd__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t mtr_cmd__pack_to_buffer
                     (const MtrCmd *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &mtr_cmd__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
MtrCmd *
       mtr_cmd__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (MtrCmd *)
     protobuf_c_message_unpack (&mtr_cmd__descriptor,
                                allocator, len, data);
}
void   mtr_cmd__free_unpacked
                     (MtrCmd *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &mtr_cmd__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   led_cmd__init
                     (LedCmd         *message)
{
  static const LedCmd init_value = LED_CMD__INIT;
  *message = init_value;
}
size_t led_cmd__get_packed_size
                     (const LedCmd *message)
{
  assert(message->base.descriptor == &led_cmd__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t led_cmd__pack
                     (const LedCmd *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &led_cmd__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t led_cmd__pack_to_buffer
                     (const LedCmd *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &led_cmd__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LedCmd *
       led_cmd__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (LedCmd *)
     protobuf_c_message_unpack (&led_cmd__descriptor,
                                allocator, len, data);
}
void   led_cmd__free_unpacked
                     (LedCmd *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &led_cmd__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   u__main_to_mtr__init
                     (UMainToMtr         *message)
{
  static const UMainToMtr init_value = U__MAIN_TO_MTR__INIT;
  *message = init_value;
}
size_t u__main_to_mtr__get_packed_size
                     (const UMainToMtr *message)
{
  assert(message->base.descriptor == &u__main_to_mtr__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t u__main_to_mtr__pack
                     (const UMainToMtr *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &u__main_to_mtr__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t u__main_to_mtr__pack_to_buffer
                     (const UMainToMtr *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &u__main_to_mtr__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
UMainToMtr *
       u__main_to_mtr__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (UMainToMtr *)
     protobuf_c_message_unpack (&u__main_to_mtr__descriptor,
                                allocator, len, data);
}
void   u__main_to_mtr__free_unpacked
                     (UMainToMtr *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &u__main_to_mtr__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCEnumValue mtr_cmd__ctrl_mode__enum_values_by_number[5] =
{
  { "UNUSED", "MTR_CMD__CTRL_MODE__UNUSED", 0 },
  { "POSITION", "MTR_CMD__CTRL_MODE__POSITION", 1 },
  { "SPEED", "MTR_CMD__CTRL_MODE__SPEED", 2 },
  { "CURRENT", "MTR_CMD__CTRL_MODE__CURRENT", 3 },
  { "SUBSCRIBE", "MTR_CMD__CTRL_MODE__SUBSCRIBE", 127 },
};
static const ProtobufCIntRange mtr_cmd__ctrl_mode__value_ranges[] = {
{0, 0},{127, 4},{0, 5}
};
static const ProtobufCEnumValueIndex mtr_cmd__ctrl_mode__enum_values_by_name[5] =
{
  { "CURRENT", 3 },
  { "POSITION", 1 },
  { "SPEED", 2 },
  { "SUBSCRIBE", 4 },
  { "UNUSED", 0 },
};
const ProtobufCEnumDescriptor mtr_cmd__ctrl_mode__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "MtrCmd.CtrlMode",
  "CtrlMode",
  "MtrCmd__CtrlMode",
  "",
  5,
  mtr_cmd__ctrl_mode__enum_values_by_number,
  5,
  mtr_cmd__ctrl_mode__enum_values_by_name,
  2,
  mtr_cmd__ctrl_mode__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCFieldDescriptor mtr_cmd__field_descriptors[4] =
{
  {
    "ctrl_mode",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(MtrCmd, ctrl_mode),
    &mtr_cmd__ctrl_mode__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "expect_status",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(MtrCmd, expect_status),
    &finger_status__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "sample_rate",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(MtrCmd, sample_rate),
    &finger_status_sample_rate__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "lock_fingers",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BOOL,
    0,   /* quantifier_offset */
    offsetof(MtrCmd, lock_fingers),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned mtr_cmd__field_indices_by_name[] = {
  0,   /* field[0] = ctrl_mode */
  1,   /* field[1] = expect_status */
  3,   /* field[3] = lock_fingers */
  2,   /* field[2] = sample_rate */
};
static const ProtobufCIntRange mtr_cmd__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
const ProtobufCMessageDescriptor mtr_cmd__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "MtrCmd",
  "MtrCmd",
  "MtrCmd",
  "",
  sizeof(MtrCmd),
  4,
  mtr_cmd__field_descriptors,
  mtr_cmd__field_indices_by_name,
  1,  mtr_cmd__number_ranges,
  (ProtobufCMessageInit) mtr_cmd__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor led_cmd__field_descriptors[2] =
{
  {
    "set_mode",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(LedCmd, set_mode),
    &led_mode__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "set_color",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(LedCmd, set_color),
    &led_color__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned led_cmd__field_indices_by_name[] = {
  1,   /* field[1] = set_color */
  0,   /* field[0] = set_mode */
};
static const ProtobufCIntRange led_cmd__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
const ProtobufCMessageDescriptor led_cmd__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "LedCmd",
  "LedCmd",
  "LedCmd",
  "",
  sizeof(LedCmd),
  2,
  led_cmd__field_descriptors,
  led_cmd__field_indices_by_name,
  1,  led_cmd__number_ranges,
  (ProtobufCMessageInit) led_cmd__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor u__main_to_mtr__field_descriptors[2] =
{
  {
    "mtr_cmd",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(UMainToMtr, mtr_cmd),
    &mtr_cmd__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "led_cmd",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(UMainToMtr, led_cmd),
    &led_cmd__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned u__main_to_mtr__field_indices_by_name[] = {
  1,   /* field[1] = led_cmd */
  0,   /* field[0] = mtr_cmd */
};
static const ProtobufCIntRange u__main_to_mtr__number_ranges[1 + 1] =
{
  { 2, 0 },
  { 0, 2 }
};
const ProtobufCMessageDescriptor u__main_to_mtr__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "U_MainToMtr",
  "UMainToMtr",
  "UMainToMtr",
  "",
  sizeof(UMainToMtr),
  2,
  u__main_to_mtr__field_descriptors,
  u__main_to_mtr__field_indices_by_name,
  1,  u__main_to_mtr__number_ranges,
  (ProtobufCMessageInit) u__main_to_mtr__init,
  NULL,NULL,NULL    /* reserved[123] */
};
