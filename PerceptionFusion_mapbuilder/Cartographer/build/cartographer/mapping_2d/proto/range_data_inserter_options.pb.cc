// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping_2d/proto/range_data_inserter_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "cartographer/mapping_2d/proto/range_data_inserter_options.pb.h"

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
namespace mapping_2d {
namespace proto {
class RangeDataInserterOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<RangeDataInserterOptions>
     _instance;
} _RangeDataInserterOptions_default_instance_;

namespace protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RangeDataInserterOptions, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RangeDataInserterOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RangeDataInserterOptions, hit_probability_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RangeDataInserterOptions, miss_probability_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RangeDataInserterOptions, insert_free_space_),
  0,
  1,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(RangeDataInserterOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_RangeDataInserterOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping_2d/proto/range_data_inserter_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  _RangeDataInserterOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_RangeDataInserterOptions_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\?cartographer/mapping_2d/proto/range_da"
      "ta_inserter_options.proto\022\035cartographer."
      "mapping_2d.proto\"h\n\030RangeDataInserterOpt"
      "ions\022\027\n\017hit_probability\030\001 \001(\001\022\030\n\020miss_pr"
      "obability\030\002 \001(\001\022\031\n\021insert_free_space\030\003 \001"
      "(\010"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 202);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping_2d/proto/range_data_inserter_options.proto", &protobuf_RegisterTypes);
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

}  // namespace protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RangeDataInserterOptions::kHitProbabilityFieldNumber;
const int RangeDataInserterOptions::kMissProbabilityFieldNumber;
const int RangeDataInserterOptions::kInsertFreeSpaceFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RangeDataInserterOptions::RangeDataInserterOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping_2d.proto.RangeDataInserterOptions)
}
RangeDataInserterOptions::RangeDataInserterOptions(const RangeDataInserterOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&hit_probability_, &from.hit_probability_,
    static_cast<size_t>(reinterpret_cast<char*>(&insert_free_space_) -
    reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping_2d.proto.RangeDataInserterOptions)
}

void RangeDataInserterOptions::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&hit_probability_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&insert_free_space_) -
      reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
}

RangeDataInserterOptions::~RangeDataInserterOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  SharedDtor();
}

void RangeDataInserterOptions::SharedDtor() {
}

void RangeDataInserterOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* RangeDataInserterOptions::descriptor() {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const RangeDataInserterOptions& RangeDataInserterOptions::default_instance() {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

RangeDataInserterOptions* RangeDataInserterOptions::New(::google::protobuf::Arena* arena) const {
  RangeDataInserterOptions* n = new RangeDataInserterOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void RangeDataInserterOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    ::memset(&hit_probability_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&insert_free_space_) -
        reinterpret_cast<char*>(&hit_probability_)) + sizeof(insert_free_space_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool RangeDataInserterOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double hit_probability = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_hit_probability();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &hit_probability_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double miss_probability = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_miss_probability();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &miss_probability_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool insert_free_space = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_insert_free_space();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &insert_free_space_)));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  return false;
#undef DO_
}

void RangeDataInserterOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double hit_probability = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->hit_probability(), output);
  }

  // optional double miss_probability = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->miss_probability(), output);
  }

  // optional bool insert_free_space = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(3, this->insert_free_space(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping_2d.proto.RangeDataInserterOptions)
}

::google::protobuf::uint8* RangeDataInserterOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double hit_probability = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->hit_probability(), target);
  }

  // optional double miss_probability = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->miss_probability(), target);
  }

  // optional bool insert_free_space = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(3, this->insert_free_space(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  return target;
}

size_t RangeDataInserterOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional double hit_probability = 1;
    if (has_hit_probability()) {
      total_size += 1 + 8;
    }

    // optional double miss_probability = 2;
    if (has_miss_probability()) {
      total_size += 1 + 8;
    }

    // optional bool insert_free_space = 3;
    if (has_insert_free_space()) {
      total_size += 1 + 1;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void RangeDataInserterOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const RangeDataInserterOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const RangeDataInserterOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping_2d.proto.RangeDataInserterOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping_2d.proto.RangeDataInserterOptions)
    MergeFrom(*source);
  }
}

void RangeDataInserterOptions::MergeFrom(const RangeDataInserterOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      hit_probability_ = from.hit_probability_;
    }
    if (cached_has_bits & 0x00000002u) {
      miss_probability_ = from.miss_probability_;
    }
    if (cached_has_bits & 0x00000004u) {
      insert_free_space_ = from.insert_free_space_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void RangeDataInserterOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RangeDataInserterOptions::CopyFrom(const RangeDataInserterOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping_2d.proto.RangeDataInserterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RangeDataInserterOptions::IsInitialized() const {
  return true;
}

void RangeDataInserterOptions::Swap(RangeDataInserterOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RangeDataInserterOptions::InternalSwap(RangeDataInserterOptions* other) {
  using std::swap;
  swap(hit_probability_, other->hit_probability_);
  swap(miss_probability_, other->miss_probability_);
  swap(insert_free_space_, other->insert_free_space_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata RangeDataInserterOptions::GetMetadata() const {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// RangeDataInserterOptions

// optional double hit_probability = 1;
bool RangeDataInserterOptions::has_hit_probability() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void RangeDataInserterOptions::set_has_hit_probability() {
  _has_bits_[0] |= 0x00000001u;
}
void RangeDataInserterOptions::clear_has_hit_probability() {
  _has_bits_[0] &= ~0x00000001u;
}
void RangeDataInserterOptions::clear_hit_probability() {
  hit_probability_ = 0;
  clear_has_hit_probability();
}
double RangeDataInserterOptions::hit_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.RangeDataInserterOptions.hit_probability)
  return hit_probability_;
}
void RangeDataInserterOptions::set_hit_probability(double value) {
  set_has_hit_probability();
  hit_probability_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.RangeDataInserterOptions.hit_probability)
}

// optional double miss_probability = 2;
bool RangeDataInserterOptions::has_miss_probability() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void RangeDataInserterOptions::set_has_miss_probability() {
  _has_bits_[0] |= 0x00000002u;
}
void RangeDataInserterOptions::clear_has_miss_probability() {
  _has_bits_[0] &= ~0x00000002u;
}
void RangeDataInserterOptions::clear_miss_probability() {
  miss_probability_ = 0;
  clear_has_miss_probability();
}
double RangeDataInserterOptions::miss_probability() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.RangeDataInserterOptions.miss_probability)
  return miss_probability_;
}
void RangeDataInserterOptions::set_miss_probability(double value) {
  set_has_miss_probability();
  miss_probability_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.RangeDataInserterOptions.miss_probability)
}

// optional bool insert_free_space = 3;
bool RangeDataInserterOptions::has_insert_free_space() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void RangeDataInserterOptions::set_has_insert_free_space() {
  _has_bits_[0] |= 0x00000004u;
}
void RangeDataInserterOptions::clear_has_insert_free_space() {
  _has_bits_[0] &= ~0x00000004u;
}
void RangeDataInserterOptions::clear_insert_free_space() {
  insert_free_space_ = false;
  clear_has_insert_free_space();
}
bool RangeDataInserterOptions::insert_free_space() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.RangeDataInserterOptions.insert_free_space)
  return insert_free_space_;
}
void RangeDataInserterOptions::set_insert_free_space(bool value) {
  set_has_insert_free_space();
  insert_free_space_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.RangeDataInserterOptions.insert_free_space)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping_2d
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)
