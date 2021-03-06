// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping_2d/proto/submaps_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"

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
class SubmapsOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<SubmapsOptions>
     _instance;
} _SubmapsOptions_default_instance_;

namespace protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SubmapsOptions, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SubmapsOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SubmapsOptions, resolution_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SubmapsOptions, num_range_data_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SubmapsOptions, range_data_inserter_options_),
  1,
  2,
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(SubmapsOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_SubmapsOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping_2d/proto/submaps_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  ::cartographer::mapping_2d::proto::protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::InitDefaults();
  _SubmapsOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_SubmapsOptions_default_instance_);_SubmapsOptions_default_instance_._instance.get_mutable()->range_data_inserter_options_ = const_cast< ::cartographer::mapping_2d::proto::RangeDataInserterOptions*>(
      ::cartographer::mapping_2d::proto::RangeDataInserterOptions::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n3cartographer/mapping_2d/proto/submaps_"
      "options.proto\022\035cartographer.mapping_2d.p"
      "roto\032\?cartographer/mapping_2d/proto/rang"
      "e_data_inserter_options.proto\"\232\001\n\016Submap"
      "sOptions\022\022\n\nresolution\030\001 \001(\001\022\026\n\016num_rang"
      "e_data\030\003 \001(\005\022\\\n\033range_data_inserter_opti"
      "ons\030\005 \001(\01327.cartographer.mapping_2d.prot"
      "o.RangeDataInserterOptions"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 306);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping_2d/proto/submaps_options.proto", &protobuf_RegisterTypes);
  ::cartographer::mapping_2d::proto::protobuf_cartographer_2fmapping_5f2d_2fproto_2frange_5fdata_5finserter_5foptions_2eproto::AddDescriptors();
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

}  // namespace protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SubmapsOptions::kResolutionFieldNumber;
const int SubmapsOptions::kNumRangeDataFieldNumber;
const int SubmapsOptions::kRangeDataInserterOptionsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SubmapsOptions::SubmapsOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping_2d.proto.SubmapsOptions)
}
SubmapsOptions::SubmapsOptions(const SubmapsOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_range_data_inserter_options()) {
    range_data_inserter_options_ = new ::cartographer::mapping_2d::proto::RangeDataInserterOptions(*from.range_data_inserter_options_);
  } else {
    range_data_inserter_options_ = NULL;
  }
  ::memcpy(&resolution_, &from.resolution_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_range_data_) -
    reinterpret_cast<char*>(&resolution_)) + sizeof(num_range_data_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping_2d.proto.SubmapsOptions)
}

void SubmapsOptions::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&range_data_inserter_options_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_range_data_) -
      reinterpret_cast<char*>(&range_data_inserter_options_)) + sizeof(num_range_data_));
}

SubmapsOptions::~SubmapsOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping_2d.proto.SubmapsOptions)
  SharedDtor();
}

void SubmapsOptions::SharedDtor() {
  if (this != internal_default_instance()) delete range_data_inserter_options_;
}

void SubmapsOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SubmapsOptions::descriptor() {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SubmapsOptions& SubmapsOptions::default_instance() {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

SubmapsOptions* SubmapsOptions::New(::google::protobuf::Arena* arena) const {
  SubmapsOptions* n = new SubmapsOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void SubmapsOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping_2d.proto.SubmapsOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (has_range_data_inserter_options()) {
    GOOGLE_DCHECK(range_data_inserter_options_ != NULL);
    range_data_inserter_options_->::cartographer::mapping_2d::proto::RangeDataInserterOptions::Clear();
  }
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 6u) {
    ::memset(&resolution_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&num_range_data_) -
        reinterpret_cast<char*>(&resolution_)) + sizeof(num_range_data_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool SubmapsOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping_2d.proto.SubmapsOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double resolution = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_resolution();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &resolution_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 num_range_data = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_num_range_data();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_range_data_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .cartographer.mapping_2d.proto.RangeDataInserterOptions range_data_inserter_options = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_range_data_inserter_options()));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping_2d.proto.SubmapsOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping_2d.proto.SubmapsOptions)
  return false;
#undef DO_
}

void SubmapsOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping_2d.proto.SubmapsOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double resolution = 1;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->resolution(), output);
  }

  // optional int32 num_range_data = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->num_range_data(), output);
  }

  // optional .cartographer.mapping_2d.proto.RangeDataInserterOptions range_data_inserter_options = 5;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, *this->range_data_inserter_options_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping_2d.proto.SubmapsOptions)
}

::google::protobuf::uint8* SubmapsOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping_2d.proto.SubmapsOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double resolution = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->resolution(), target);
  }

  // optional int32 num_range_data = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->num_range_data(), target);
  }

  // optional .cartographer.mapping_2d.proto.RangeDataInserterOptions range_data_inserter_options = 5;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        5, *this->range_data_inserter_options_, deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping_2d.proto.SubmapsOptions)
  return target;
}

size_t SubmapsOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping_2d.proto.SubmapsOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional .cartographer.mapping_2d.proto.RangeDataInserterOptions range_data_inserter_options = 5;
    if (has_range_data_inserter_options()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->range_data_inserter_options_);
    }

    // optional double resolution = 1;
    if (has_resolution()) {
      total_size += 1 + 8;
    }

    // optional int32 num_range_data = 3;
    if (has_num_range_data()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->num_range_data());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void SubmapsOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping_2d.proto.SubmapsOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const SubmapsOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SubmapsOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping_2d.proto.SubmapsOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping_2d.proto.SubmapsOptions)
    MergeFrom(*source);
  }
}

void SubmapsOptions::MergeFrom(const SubmapsOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping_2d.proto.SubmapsOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_range_data_inserter_options()->::cartographer::mapping_2d::proto::RangeDataInserterOptions::MergeFrom(from.range_data_inserter_options());
    }
    if (cached_has_bits & 0x00000002u) {
      resolution_ = from.resolution_;
    }
    if (cached_has_bits & 0x00000004u) {
      num_range_data_ = from.num_range_data_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SubmapsOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping_2d.proto.SubmapsOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SubmapsOptions::CopyFrom(const SubmapsOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping_2d.proto.SubmapsOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SubmapsOptions::IsInitialized() const {
  return true;
}

void SubmapsOptions::Swap(SubmapsOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SubmapsOptions::InternalSwap(SubmapsOptions* other) {
  using std::swap;
  swap(range_data_inserter_options_, other->range_data_inserter_options_);
  swap(resolution_, other->resolution_);
  swap(num_range_data_, other->num_range_data_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata SubmapsOptions::GetMetadata() const {
  protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_5f2d_2fproto_2fsubmaps_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// SubmapsOptions

// optional double resolution = 1;
bool SubmapsOptions::has_resolution() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void SubmapsOptions::set_has_resolution() {
  _has_bits_[0] |= 0x00000002u;
}
void SubmapsOptions::clear_has_resolution() {
  _has_bits_[0] &= ~0x00000002u;
}
void SubmapsOptions::clear_resolution() {
  resolution_ = 0;
  clear_has_resolution();
}
double SubmapsOptions::resolution() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.SubmapsOptions.resolution)
  return resolution_;
}
void SubmapsOptions::set_resolution(double value) {
  set_has_resolution();
  resolution_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.SubmapsOptions.resolution)
}

// optional int32 num_range_data = 3;
bool SubmapsOptions::has_num_range_data() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void SubmapsOptions::set_has_num_range_data() {
  _has_bits_[0] |= 0x00000004u;
}
void SubmapsOptions::clear_has_num_range_data() {
  _has_bits_[0] &= ~0x00000004u;
}
void SubmapsOptions::clear_num_range_data() {
  num_range_data_ = 0;
  clear_has_num_range_data();
}
::google::protobuf::int32 SubmapsOptions::num_range_data() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.SubmapsOptions.num_range_data)
  return num_range_data_;
}
void SubmapsOptions::set_num_range_data(::google::protobuf::int32 value) {
  set_has_num_range_data();
  num_range_data_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping_2d.proto.SubmapsOptions.num_range_data)
}

// optional .cartographer.mapping_2d.proto.RangeDataInserterOptions range_data_inserter_options = 5;
bool SubmapsOptions::has_range_data_inserter_options() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void SubmapsOptions::set_has_range_data_inserter_options() {
  _has_bits_[0] |= 0x00000001u;
}
void SubmapsOptions::clear_has_range_data_inserter_options() {
  _has_bits_[0] &= ~0x00000001u;
}
void SubmapsOptions::clear_range_data_inserter_options() {
  if (range_data_inserter_options_ != NULL) range_data_inserter_options_->::cartographer::mapping_2d::proto::RangeDataInserterOptions::Clear();
  clear_has_range_data_inserter_options();
}
const ::cartographer::mapping_2d::proto::RangeDataInserterOptions& SubmapsOptions::range_data_inserter_options() const {
  const ::cartographer::mapping_2d::proto::RangeDataInserterOptions* p = range_data_inserter_options_;
  // @@protoc_insertion_point(field_get:cartographer.mapping_2d.proto.SubmapsOptions.range_data_inserter_options)
  return p != NULL ? *p : *reinterpret_cast<const ::cartographer::mapping_2d::proto::RangeDataInserterOptions*>(
      &::cartographer::mapping_2d::proto::_RangeDataInserterOptions_default_instance_);
}
::cartographer::mapping_2d::proto::RangeDataInserterOptions* SubmapsOptions::mutable_range_data_inserter_options() {
  set_has_range_data_inserter_options();
  if (range_data_inserter_options_ == NULL) {
    range_data_inserter_options_ = new ::cartographer::mapping_2d::proto::RangeDataInserterOptions;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping_2d.proto.SubmapsOptions.range_data_inserter_options)
  return range_data_inserter_options_;
}
::cartographer::mapping_2d::proto::RangeDataInserterOptions* SubmapsOptions::release_range_data_inserter_options() {
  // @@protoc_insertion_point(field_release:cartographer.mapping_2d.proto.SubmapsOptions.range_data_inserter_options)
  clear_has_range_data_inserter_options();
  ::cartographer::mapping_2d::proto::RangeDataInserterOptions* temp = range_data_inserter_options_;
  range_data_inserter_options_ = NULL;
  return temp;
}
void SubmapsOptions::set_allocated_range_data_inserter_options(::cartographer::mapping_2d::proto::RangeDataInserterOptions* range_data_inserter_options) {
  delete range_data_inserter_options_;
  range_data_inserter_options_ = range_data_inserter_options;
  if (range_data_inserter_options) {
    set_has_range_data_inserter_options();
  } else {
    clear_has_range_data_inserter_options();
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping_2d.proto.SubmapsOptions.range_data_inserter_options)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping_2d
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)
