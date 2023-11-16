// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mavsdk_options.proto
// Protobuf C++ Version: 4.25.0

#ifndef GOOGLE_PROTOBUF_INCLUDED_mavsdk_5foptions_2eproto_2epb_2eh
#define GOOGLE_PROTOBUF_INCLUDED_mavsdk_5foptions_2eproto_2epb_2eh

#include <limits>
#include <string>
#include <type_traits>
#include <utility>

#include "google/protobuf/port_def.inc"
#if PROTOBUF_VERSION < 4025000
#error "This file was generated by a newer version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please update"
#error "your headers."
#endif  // PROTOBUF_VERSION

#if 4025000 < PROTOBUF_MIN_PROTOC_VERSION
#error "This file was generated by an older version of protoc which is"
#error "incompatible with your Protocol Buffer headers. Please"
#error "regenerate this file with a newer version of protoc."
#endif  // PROTOBUF_MIN_PROTOC_VERSION
#include "google/protobuf/port_undef.inc"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/arena.h"
#include "google/protobuf/arenastring.h"
#include "google/protobuf/generated_message_tctable_decl.h"
#include "google/protobuf/generated_message_util.h"
#include "google/protobuf/metadata_lite.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/repeated_field.h"  // IWYU pragma: export
#include "google/protobuf/extension_set.h"  // IWYU pragma: export
#include "google/protobuf/generated_enum_reflection.h"
#include "google/protobuf/descriptor.pb.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"

#define PROTOBUF_INTERNAL_EXPORT_mavsdk_5foptions_2eproto

namespace google {
namespace protobuf {
namespace internal {
class AnyMetadata;
}  // namespace internal
}  // namespace protobuf
}  // namespace google

// Internal implementation detail -- do not use these members.
struct TableStruct_mavsdk_5foptions_2eproto {
  static const ::uint32_t offsets[];
};
extern const ::google::protobuf::internal::DescriptorTable
    descriptor_table_mavsdk_5foptions_2eproto;
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google

namespace mavsdk {
namespace options {
enum AsyncType : int {
  ASYNC = 0,
  SYNC = 1,
  BOTH = 2,
  AsyncType_INT_MIN_SENTINEL_DO_NOT_USE_ =
      std::numeric_limits<::int32_t>::min(),
  AsyncType_INT_MAX_SENTINEL_DO_NOT_USE_ =
      std::numeric_limits<::int32_t>::max(),
};

bool AsyncType_IsValid(int value);
extern const uint32_t AsyncType_internal_data_[];
constexpr AsyncType AsyncType_MIN = static_cast<AsyncType>(0);
constexpr AsyncType AsyncType_MAX = static_cast<AsyncType>(2);
constexpr int AsyncType_ARRAYSIZE = 2 + 1;
const ::google::protobuf::EnumDescriptor*
AsyncType_descriptor();
template <typename T>
const std::string& AsyncType_Name(T value) {
  static_assert(std::is_same<T, AsyncType>::value ||
                    std::is_integral<T>::value,
                "Incorrect type passed to AsyncType_Name().");
  return AsyncType_Name(static_cast<AsyncType>(value));
}
template <>
inline const std::string& AsyncType_Name(AsyncType value) {
  return ::google::protobuf::internal::NameOfDenseEnum<AsyncType_descriptor,
                                                 0, 2>(
      static_cast<int>(value));
}
inline bool AsyncType_Parse(absl::string_view name, AsyncType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<AsyncType>(
      AsyncType_descriptor(), name, value);
}

// ===================================================================



// ===================================================================



static const int kDefaultValueFieldNumber = 50000;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::StringTypeTraits, 9, false >
  default_value;
static const int kEpsilonFieldNumber = 50001;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::FieldOptions,
    ::google::protobuf::internal::PrimitiveTypeTraits< double >, 1, false >
  epsilon;
static const int kAsyncTypeFieldNumber = 50000;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::MethodOptions,
    ::google::protobuf::internal::EnumTypeTraits< ::mavsdk::options::AsyncType, ::mavsdk::options::AsyncType_IsValid>, 14, false >
  async_type;
static const int kIsFiniteFieldNumber = 50001;
extern ::google::protobuf::internal::ExtensionIdentifier< ::google::protobuf::MethodOptions,
    ::google::protobuf::internal::PrimitiveTypeTraits< bool >, 8, false >
  is_finite;

// ===================================================================


#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)
}  // namespace options
}  // namespace mavsdk


namespace google {
namespace protobuf {

template <>
struct is_proto_enum<::mavsdk::options::AsyncType> : std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor<::mavsdk::options::AsyncType>() {
  return ::mavsdk::options::AsyncType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#include "google/protobuf/port_undef.inc"

#endif  // GOOGLE_PROTOBUF_INCLUDED_mavsdk_5foptions_2eproto_2epb_2eh
