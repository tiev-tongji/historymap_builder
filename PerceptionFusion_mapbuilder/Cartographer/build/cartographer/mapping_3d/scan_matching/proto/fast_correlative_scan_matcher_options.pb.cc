// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {
namespace proto {
class FastCorrelativeScanMatcherOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<FastCorrelativeScanMatcherOptions>
     _instance;
} _FastCorrelativeScanMatcherOptions_default_instance_;

namespace protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, branch_and_bound_depth_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, full_resolution_depth_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, min_rotational_score_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, min_low_resolution_score_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, linear_xy_search_window_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, linear_z_search_window_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(FastCorrelativeScanMatcherOptions, angular_search_window_),
  2,
  3,
  0,
  6,
  1,
  4,
  5,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 12, sizeof(FastCorrelativeScanMatcherOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_FastCorrelativeScanMatcherOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _FastCorrelativeScanMatcherOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_FastCorrelativeScanMatcherOptions_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\nWcartographer/mapping_3d/scan_matching/"
      "proto/fast_correlative_scan_matcher_opti"
      "ons.proto\022+cartographer.mapping_3d.scan_"
      "matching.proto\"\202\002\n!FastCorrelativeScanMa"
      "tcherOptions\022\036\n\026branch_and_bound_depth\030\002"
      " \001(\005\022\035\n\025full_resolution_depth\030\010 \001(\005\022\034\n\024m"
      "in_rotational_score\030\004 \001(\001\022 \n\030min_low_res"
      "olution_score\030\t \001(\001\022\037\n\027linear_xy_search_"
      "window\030\005 \001(\001\022\036\n\026linear_z_search_window\030\006"
      " \001(\001\022\035\n\025angular_search_window\030\007 \001(\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 395);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.proto", &protobuf_RegisterTypes);
}
} // anonymous namespace

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int FastCorrelativeScanMatcherOptions::kBranchAndBoundDepthFieldNumber;
const int FastCorrelativeScanMatcherOptions::kFullResolutionDepthFieldNumber;
const int FastCorrelativeScanMatcherOptions::kMinRotationalScoreFieldNumber;
const int FastCorrelativeScanMatcherOptions::kMinLowResolutionScoreFieldNumber;
const int FastCorrelativeScanMatcherOptions::kLinearXySearchWindowFieldNumber;
const int FastCorrelativeScanMatcherOptions::kLinearZSearchWindowFieldNumber;
const int FastCorrelativeScanMatcherOptions::kAngularSearchWindowFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

FastCorrelativeScanMatcherOptions::FastCorrelativeScanMatcherOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
}
FastCorrelativeScanMatcherOptions::FastCorrelativeScanMatcherOptions(const FastCorrelativeScanMatcherOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&min_rotational_score_, &from.min_rotational_score_,
    static_cast<size_t>(reinterpret_cast<char*>(&min_low_resolution_score_) -
    reinterpret_cast<char*>(&min_rotational_score_)) + sizeof(min_low_resolution_score_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
}

void FastCorrelativeScanMatcherOptions::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&min_rotational_score_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&min_low_resolution_score_) -
      reinterpret_cast<char*>(&min_rotational_score_)) + sizeof(min_low_resolution_score_));
}

FastCorrelativeScanMatcherOptions::~FastCorrelativeScanMatcherOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  SharedDtor();
}

void FastCorrelativeScanMatcherOptions::SharedDtor() {
}

void FastCorrelativeScanMatcherOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* FastCorrelativeScanMatcherOptions::descriptor() {
  protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const FastCorrelativeScanMatcherOptions& FastCorrelativeScanMatcherOptions::default_instance() {
  protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

FastCorrelativeScanMatcherOptions* FastCorrelativeScanMatcherOptions::New(::google::protobuf::Arena* arena) const {
  FastCorrelativeScanMatcherOptions* n = new FastCorrelativeScanMatcherOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void FastCorrelativeScanMatcherOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 127u) {
    ::memset(&min_rotational_score_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&min_low_resolution_score_) -
        reinterpret_cast<char*>(&min_rotational_score_)) + sizeof(min_low_resolution_score_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool FastCorrelativeScanMatcherOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 branch_and_bound_depth = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_branch_and_bound_depth();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &branch_and_bound_depth_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double min_rotational_score = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {
          set_has_min_rotational_score();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &min_rotational_score_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double linear_xy_search_window = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {
          set_has_linear_xy_search_window();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_xy_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double linear_z_search_window = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(49u /* 49 & 0xFF */)) {
          set_has_linear_z_search_window();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_z_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double angular_search_window = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(57u /* 57 & 0xFF */)) {
          set_has_angular_search_window();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &angular_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 full_resolution_depth = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(64u /* 64 & 0xFF */)) {
          set_has_full_resolution_depth();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &full_resolution_depth_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double min_low_resolution_score = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(73u /* 73 & 0xFF */)) {
          set_has_min_low_resolution_score();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &min_low_resolution_score_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  return false;
#undef DO_
}

void FastCorrelativeScanMatcherOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 branch_and_bound_depth = 2;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->branch_and_bound_depth(), output);
  }

  // optional double min_rotational_score = 4;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->min_rotational_score(), output);
  }

  // optional double linear_xy_search_window = 5;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->linear_xy_search_window(), output);
  }

  // optional double linear_z_search_window = 6;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(6, this->linear_z_search_window(), output);
  }

  // optional double angular_search_window = 7;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(7, this->angular_search_window(), output);
  }

  // optional int32 full_resolution_depth = 8;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(8, this->full_resolution_depth(), output);
  }

  // optional double min_low_resolution_score = 9;
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(9, this->min_low_resolution_score(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
}

::google::protobuf::uint8* FastCorrelativeScanMatcherOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 branch_and_bound_depth = 2;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->branch_and_bound_depth(), target);
  }

  // optional double min_rotational_score = 4;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->min_rotational_score(), target);
  }

  // optional double linear_xy_search_window = 5;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->linear_xy_search_window(), target);
  }

  // optional double linear_z_search_window = 6;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(6, this->linear_z_search_window(), target);
  }

  // optional double angular_search_window = 7;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(7, this->angular_search_window(), target);
  }

  // optional int32 full_resolution_depth = 8;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(8, this->full_resolution_depth(), target);
  }

  // optional double min_low_resolution_score = 9;
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(9, this->min_low_resolution_score(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  return target;
}

size_t FastCorrelativeScanMatcherOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 127u) {
    // optional double min_rotational_score = 4;
    if (has_min_rotational_score()) {
      total_size += 1 + 8;
    }

    // optional double linear_xy_search_window = 5;
    if (has_linear_xy_search_window()) {
      total_size += 1 + 8;
    }

    // optional int32 branch_and_bound_depth = 2;
    if (has_branch_and_bound_depth()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->branch_and_bound_depth());
    }

    // optional int32 full_resolution_depth = 8;
    if (has_full_resolution_depth()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->full_resolution_depth());
    }

    // optional double linear_z_search_window = 6;
    if (has_linear_z_search_window()) {
      total_size += 1 + 8;
    }

    // optional double angular_search_window = 7;
    if (has_angular_search_window()) {
      total_size += 1 + 8;
    }

    // optional double min_low_resolution_score = 9;
    if (has_min_low_resolution_score()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void FastCorrelativeScanMatcherOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const FastCorrelativeScanMatcherOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const FastCorrelativeScanMatcherOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
    MergeFrom(*source);
  }
}

void FastCorrelativeScanMatcherOptions::MergeFrom(const FastCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 127u) {
    if (cached_has_bits & 0x00000001u) {
      min_rotational_score_ = from.min_rotational_score_;
    }
    if (cached_has_bits & 0x00000002u) {
      linear_xy_search_window_ = from.linear_xy_search_window_;
    }
    if (cached_has_bits & 0x00000004u) {
      branch_and_bound_depth_ = from.branch_and_bound_depth_;
    }
    if (cached_has_bits & 0x00000008u) {
      full_resolution_depth_ = from.full_resolution_depth_;
    }
    if (cached_has_bits & 0x00000010u) {
      linear_z_search_window_ = from.linear_z_search_window_;
    }
    if (cached_has_bits & 0x00000020u) {
      angular_search_window_ = from.angular_search_window_;
    }
    if (cached_has_bits & 0x00000040u) {
      min_low_resolution_score_ = from.min_low_resolution_score_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void FastCorrelativeScanMatcherOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void FastCorrelativeScanMatcherOptions::CopyFrom(const FastCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FastCorrelativeScanMatcherOptions::IsInitialized() const {
  return true;
}

void FastCorrelativeScanMatcherOptions::Swap(FastCorrelativeScanMatcherOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void FastCorrelativeScanMatcherOptions::InternalSwap(FastCorrelativeScanMatcherOptions* other) {
  using std::swap;
  swap(min_rotational_score_, other->min_rotational_score_);
  swap(linear_xy_search_window_, other->linear_xy_search_window_);
  swap(branch_and_bound_depth_, other->branch_and_bound_depth_);
  swap(full_resolution_depth_, other->full_resolution_depth_);
  swap(linear_z_search_window_, other->linear_z_search_window_);
  swap(angular_search_window_, other->angular_search_window_);
  swap(min_low_resolution_score_, other->min_low_resolution_score_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata FastCorrelativeScanMatcherOptions::GetMetadata() const {
  protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f3d_2fscan_5fmatching_2fproto_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// FastCorrelativeScanMatcherOptions

// optional int32 branch_and_bound_depth = 2;
bool FastCorrelativeScanMatcherOptions::has_branch_and_bound_depth() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_branch_and_bound_depth() {
  _has_bits_[0] |= 0x00000004u;
}
void FastCorrelativeScanMatcherOptions::clear_has_branch_and_bound_depth() {
  _has_bits_[0] &= ~0x00000004u;
}
void FastCorrelativeScanMatcherOptions::clear_branch_and_bound_depth() {
  branch_and_bound_depth_ = 0;
  clear_has_branch_and_bound_depth();
}
::google::protobuf::int32 FastCorrelativeScanMatcherOptions::branch_and_bound_depth() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.branch_and_bound_depth)
  return branch_and_bound_depth_;
}
void FastCorrelativeScanMatcherOptions::set_branch_and_bound_depth(::google::protobuf::int32 value) {
  set_has_branch_and_bound_depth();
  branch_and_bound_depth_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.branch_and_bound_depth)
}

// optional int32 full_resolution_depth = 8;
bool FastCorrelativeScanMatcherOptions::has_full_resolution_depth() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_full_resolution_depth() {
  _has_bits_[0] |= 0x00000008u;
}
void FastCorrelativeScanMatcherOptions::clear_has_full_resolution_depth() {
  _has_bits_[0] &= ~0x00000008u;
}
void FastCorrelativeScanMatcherOptions::clear_full_resolution_depth() {
  full_resolution_depth_ = 0;
  clear_has_full_resolution_depth();
}
::google::protobuf::int32 FastCorrelativeScanMatcherOptions::full_resolution_depth() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.full_resolution_depth)
  return full_resolution_depth_;
}
void FastCorrelativeScanMatcherOptions::set_full_resolution_depth(::google::protobuf::int32 value) {
  set_has_full_resolution_depth();
  full_resolution_depth_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.full_resolution_depth)
}

// optional double min_rotational_score = 4;
bool FastCorrelativeScanMatcherOptions::has_min_rotational_score() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_min_rotational_score() {
  _has_bits_[0] |= 0x00000001u;
}
void FastCorrelativeScanMatcherOptions::clear_has_min_rotational_score() {
  _has_bits_[0] &= ~0x00000001u;
}
void FastCorrelativeScanMatcherOptions::clear_min_rotational_score() {
  min_rotational_score_ = 0;
  clear_has_min_rotational_score();
}
double FastCorrelativeScanMatcherOptions::min_rotational_score() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.min_rotational_score)
  return min_rotational_score_;
}
void FastCorrelativeScanMatcherOptions::set_min_rotational_score(double value) {
  set_has_min_rotational_score();
  min_rotational_score_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.min_rotational_score)
}

// optional double min_low_resolution_score = 9;
bool FastCorrelativeScanMatcherOptions::has_min_low_resolution_score() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_min_low_resolution_score() {
  _has_bits_[0] |= 0x00000040u;
}
void FastCorrelativeScanMatcherOptions::clear_has_min_low_resolution_score() {
  _has_bits_[0] &= ~0x00000040u;
}
void FastCorrelativeScanMatcherOptions::clear_min_low_resolution_score() {
  min_low_resolution_score_ = 0;
  clear_has_min_low_resolution_score();
}
double FastCorrelativeScanMatcherOptions::min_low_resolution_score() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.min_low_resolution_score)
  return min_low_resolution_score_;
}
void FastCorrelativeScanMatcherOptions::set_min_low_resolution_score(double value) {
  set_has_min_low_resolution_score();
  min_low_resolution_score_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.min_low_resolution_score)
}

// optional double linear_xy_search_window = 5;
bool FastCorrelativeScanMatcherOptions::has_linear_xy_search_window() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_linear_xy_search_window() {
  _has_bits_[0] |= 0x00000002u;
}
void FastCorrelativeScanMatcherOptions::clear_has_linear_xy_search_window() {
  _has_bits_[0] &= ~0x00000002u;
}
void FastCorrelativeScanMatcherOptions::clear_linear_xy_search_window() {
  linear_xy_search_window_ = 0;
  clear_has_linear_xy_search_window();
}
double FastCorrelativeScanMatcherOptions::linear_xy_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.linear_xy_search_window)
  return linear_xy_search_window_;
}
void FastCorrelativeScanMatcherOptions::set_linear_xy_search_window(double value) {
  set_has_linear_xy_search_window();
  linear_xy_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.linear_xy_search_window)
}

// optional double linear_z_search_window = 6;
bool FastCorrelativeScanMatcherOptions::has_linear_z_search_window() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_linear_z_search_window() {
  _has_bits_[0] |= 0x00000010u;
}
void FastCorrelativeScanMatcherOptions::clear_has_linear_z_search_window() {
  _has_bits_[0] &= ~0x00000010u;
}
void FastCorrelativeScanMatcherOptions::clear_linear_z_search_window() {
  linear_z_search_window_ = 0;
  clear_has_linear_z_search_window();
}
double FastCorrelativeScanMatcherOptions::linear_z_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.linear_z_search_window)
  return linear_z_search_window_;
}
void FastCorrelativeScanMatcherOptions::set_linear_z_search_window(double value) {
  set_has_linear_z_search_window();
  linear_z_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.linear_z_search_window)
}

// optional double angular_search_window = 7;
bool FastCorrelativeScanMatcherOptions::has_angular_search_window() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void FastCorrelativeScanMatcherOptions::set_has_angular_search_window() {
  _has_bits_[0] |= 0x00000020u;
}
void FastCorrelativeScanMatcherOptions::clear_has_angular_search_window() {
  _has_bits_[0] &= ~0x00000020u;
}
void FastCorrelativeScanMatcherOptions::clear_angular_search_window() {
  angular_search_window_ = 0;
  clear_has_angular_search_window();
}
double FastCorrelativeScanMatcherOptions::angular_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.angular_search_window)
  return angular_search_window_;
}
void FastCorrelativeScanMatcherOptions::set_angular_search_window(double value) {
  set_has_angular_search_window();
  angular_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_3d.scan_matching.proto.FastCorrelativeScanMatcherOptions.angular_search_window)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)
