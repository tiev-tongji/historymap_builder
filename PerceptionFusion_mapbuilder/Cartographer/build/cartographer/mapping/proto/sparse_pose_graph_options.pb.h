// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/sparse_pose_graph_options.proto

#ifndef PROTOBUF_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h"
// @@protoc_insertion_point(includes)
namespace cartographer {
namespace mapping {
namespace proto {
class SparsePoseGraphOptions;
class SparsePoseGraphOptionsDefaultTypeInternal;
extern SparsePoseGraphOptionsDefaultTypeInternal _SparsePoseGraphOptions_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer

namespace cartographer {
namespace mapping {
namespace proto {

namespace protobuf_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto

// ===================================================================

class SparsePoseGraphOptions : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.SparsePoseGraphOptions) */ {
 public:
  SparsePoseGraphOptions();
  virtual ~SparsePoseGraphOptions();

  SparsePoseGraphOptions(const SparsePoseGraphOptions& from);

  inline SparsePoseGraphOptions& operator=(const SparsePoseGraphOptions& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SparsePoseGraphOptions(SparsePoseGraphOptions&& from) noexcept
    : SparsePoseGraphOptions() {
    *this = ::std::move(from);
  }

  inline SparsePoseGraphOptions& operator=(SparsePoseGraphOptions&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SparsePoseGraphOptions& default_instance();

  static inline const SparsePoseGraphOptions* internal_default_instance() {
    return reinterpret_cast<const SparsePoseGraphOptions*>(
               &_SparsePoseGraphOptions_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SparsePoseGraphOptions* other);
  friend void swap(SparsePoseGraphOptions& a, SparsePoseGraphOptions& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SparsePoseGraphOptions* New() const PROTOBUF_FINAL { return New(NULL); }

  SparsePoseGraphOptions* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SparsePoseGraphOptions& from);
  void MergeFrom(const SparsePoseGraphOptions& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(SparsePoseGraphOptions* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .cartographer.mapping.sparse_pose_graph.proto.ConstraintBuilderOptions constraint_builder_options = 3;
  bool has_constraint_builder_options() const;
  void clear_constraint_builder_options();
  static const int kConstraintBuilderOptionsFieldNumber = 3;
  const ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions& constraint_builder_options() const;
  ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* mutable_constraint_builder_options();
  ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* release_constraint_builder_options();
  void set_allocated_constraint_builder_options(::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* constraint_builder_options);

  // optional .cartographer.mapping.sparse_pose_graph.proto.OptimizationProblemOptions optimization_problem_options = 4;
  bool has_optimization_problem_options() const;
  void clear_optimization_problem_options();
  static const int kOptimizationProblemOptionsFieldNumber = 4;
  const ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions& optimization_problem_options() const;
  ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* mutable_optimization_problem_options();
  ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* release_optimization_problem_options();
  void set_allocated_optimization_problem_options(::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* optimization_problem_options);

  // optional int32 optimize_every_n_scans = 1;
  bool has_optimize_every_n_scans() const;
  void clear_optimize_every_n_scans();
  static const int kOptimizeEveryNScansFieldNumber = 1;
  ::google::protobuf::int32 optimize_every_n_scans() const;
  void set_optimize_every_n_scans(::google::protobuf::int32 value);

  // optional int32 max_num_final_iterations = 6;
  bool has_max_num_final_iterations() const;
  void clear_max_num_final_iterations();
  static const int kMaxNumFinalIterationsFieldNumber = 6;
  ::google::protobuf::int32 max_num_final_iterations() const;
  void set_max_num_final_iterations(::google::protobuf::int32 value);

  // optional double global_sampling_ratio = 5;
  bool has_global_sampling_ratio() const;
  void clear_global_sampling_ratio();
  static const int kGlobalSamplingRatioFieldNumber = 5;
  double global_sampling_ratio() const;
  void set_global_sampling_ratio(double value);

  // optional double matcher_translation_weight = 7;
  bool has_matcher_translation_weight() const;
  void clear_matcher_translation_weight();
  static const int kMatcherTranslationWeightFieldNumber = 7;
  double matcher_translation_weight() const;
  void set_matcher_translation_weight(double value);

  // optional double matcher_rotation_weight = 8;
  bool has_matcher_rotation_weight() const;
  void clear_matcher_rotation_weight();
  static const int kMatcherRotationWeightFieldNumber = 8;
  double matcher_rotation_weight() const;
  void set_matcher_rotation_weight(double value);

  // optional double global_constraint_search_after_n_seconds = 10;
  bool has_global_constraint_search_after_n_seconds() const;
  void clear_global_constraint_search_after_n_seconds();
  static const int kGlobalConstraintSearchAfterNSecondsFieldNumber = 10;
  double global_constraint_search_after_n_seconds() const;
  void set_global_constraint_search_after_n_seconds(double value);

  // optional bool log_residual_histograms = 9;
  bool has_log_residual_histograms() const;
  void clear_log_residual_histograms();
  static const int kLogResidualHistogramsFieldNumber = 9;
  bool log_residual_histograms() const;
  void set_log_residual_histograms(bool value);

  // optional bool use_odom = 11;
  bool has_use_odom() const;
  void clear_use_odom();
  static const int kUseOdomFieldNumber = 11;
  bool use_odom() const;
  void set_use_odom(bool value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.SparsePoseGraphOptions)
 private:
  void set_has_optimize_every_n_scans();
  void clear_has_optimize_every_n_scans();
  void set_has_constraint_builder_options();
  void clear_has_constraint_builder_options();
  void set_has_matcher_translation_weight();
  void clear_has_matcher_translation_weight();
  void set_has_matcher_rotation_weight();
  void clear_has_matcher_rotation_weight();
  void set_has_optimization_problem_options();
  void clear_has_optimization_problem_options();
  void set_has_max_num_final_iterations();
  void clear_has_max_num_final_iterations();
  void set_has_global_sampling_ratio();
  void clear_has_global_sampling_ratio();
  void set_has_log_residual_histograms();
  void clear_has_log_residual_histograms();
  void set_has_global_constraint_search_after_n_seconds();
  void clear_has_global_constraint_search_after_n_seconds();
  void set_has_use_odom();
  void clear_has_use_odom();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* constraint_builder_options_;
  ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* optimization_problem_options_;
  ::google::protobuf::int32 optimize_every_n_scans_;
  ::google::protobuf::int32 max_num_final_iterations_;
  double global_sampling_ratio_;
  double matcher_translation_weight_;
  double matcher_rotation_weight_;
  double global_constraint_search_after_n_seconds_;
  bool log_residual_histograms_;
  bool use_odom_;
  friend struct protobuf_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SparsePoseGraphOptions

// optional int32 optimize_every_n_scans = 1;
inline bool SparsePoseGraphOptions::has_optimize_every_n_scans() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SparsePoseGraphOptions::set_has_optimize_every_n_scans() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SparsePoseGraphOptions::clear_has_optimize_every_n_scans() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SparsePoseGraphOptions::clear_optimize_every_n_scans() {
  optimize_every_n_scans_ = 0;
  clear_has_optimize_every_n_scans();
}
inline ::google::protobuf::int32 SparsePoseGraphOptions::optimize_every_n_scans() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.optimize_every_n_scans)
  return optimize_every_n_scans_;
}
inline void SparsePoseGraphOptions::set_optimize_every_n_scans(::google::protobuf::int32 value) {
  set_has_optimize_every_n_scans();
  optimize_every_n_scans_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.optimize_every_n_scans)
}

// optional .cartographer.mapping.sparse_pose_graph.proto.ConstraintBuilderOptions constraint_builder_options = 3;
inline bool SparsePoseGraphOptions::has_constraint_builder_options() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SparsePoseGraphOptions::set_has_constraint_builder_options() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SparsePoseGraphOptions::clear_has_constraint_builder_options() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SparsePoseGraphOptions::clear_constraint_builder_options() {
  if (constraint_builder_options_ != NULL) constraint_builder_options_->::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions::Clear();
  clear_has_constraint_builder_options();
}
inline const ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions& SparsePoseGraphOptions::constraint_builder_options() const {
  const ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* p = constraint_builder_options_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.constraint_builder_options)
  return p != NULL ? *p : *reinterpret_cast<const ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions*>(
      &::cartographer::mapping::sparse_pose_graph::proto::_ConstraintBuilderOptions_default_instance_);
}
inline ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* SparsePoseGraphOptions::mutable_constraint_builder_options() {
  set_has_constraint_builder_options();
  if (constraint_builder_options_ == NULL) {
    constraint_builder_options_ = new ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.SparsePoseGraphOptions.constraint_builder_options)
  return constraint_builder_options_;
}
inline ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* SparsePoseGraphOptions::release_constraint_builder_options() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.SparsePoseGraphOptions.constraint_builder_options)
  clear_has_constraint_builder_options();
  ::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* temp = constraint_builder_options_;
  constraint_builder_options_ = NULL;
  return temp;
}
inline void SparsePoseGraphOptions::set_allocated_constraint_builder_options(::cartographer::mapping::sparse_pose_graph::proto::ConstraintBuilderOptions* constraint_builder_options) {
  delete constraint_builder_options_;
  constraint_builder_options_ = constraint_builder_options;
  if (constraint_builder_options) {
    set_has_constraint_builder_options();
  } else {
    clear_has_constraint_builder_options();
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.SparsePoseGraphOptions.constraint_builder_options)
}

// optional double matcher_translation_weight = 7;
inline bool SparsePoseGraphOptions::has_matcher_translation_weight() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SparsePoseGraphOptions::set_has_matcher_translation_weight() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SparsePoseGraphOptions::clear_has_matcher_translation_weight() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SparsePoseGraphOptions::clear_matcher_translation_weight() {
  matcher_translation_weight_ = 0;
  clear_has_matcher_translation_weight();
}
inline double SparsePoseGraphOptions::matcher_translation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.matcher_translation_weight)
  return matcher_translation_weight_;
}
inline void SparsePoseGraphOptions::set_matcher_translation_weight(double value) {
  set_has_matcher_translation_weight();
  matcher_translation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.matcher_translation_weight)
}

// optional double matcher_rotation_weight = 8;
inline bool SparsePoseGraphOptions::has_matcher_rotation_weight() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SparsePoseGraphOptions::set_has_matcher_rotation_weight() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SparsePoseGraphOptions::clear_has_matcher_rotation_weight() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SparsePoseGraphOptions::clear_matcher_rotation_weight() {
  matcher_rotation_weight_ = 0;
  clear_has_matcher_rotation_weight();
}
inline double SparsePoseGraphOptions::matcher_rotation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.matcher_rotation_weight)
  return matcher_rotation_weight_;
}
inline void SparsePoseGraphOptions::set_matcher_rotation_weight(double value) {
  set_has_matcher_rotation_weight();
  matcher_rotation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.matcher_rotation_weight)
}

// optional .cartographer.mapping.sparse_pose_graph.proto.OptimizationProblemOptions optimization_problem_options = 4;
inline bool SparsePoseGraphOptions::has_optimization_problem_options() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SparsePoseGraphOptions::set_has_optimization_problem_options() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SparsePoseGraphOptions::clear_has_optimization_problem_options() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SparsePoseGraphOptions::clear_optimization_problem_options() {
  if (optimization_problem_options_ != NULL) optimization_problem_options_->::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions::Clear();
  clear_has_optimization_problem_options();
}
inline const ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions& SparsePoseGraphOptions::optimization_problem_options() const {
  const ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* p = optimization_problem_options_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.optimization_problem_options)
  return p != NULL ? *p : *reinterpret_cast<const ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions*>(
      &::cartographer::mapping::sparse_pose_graph::proto::_OptimizationProblemOptions_default_instance_);
}
inline ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* SparsePoseGraphOptions::mutable_optimization_problem_options() {
  set_has_optimization_problem_options();
  if (optimization_problem_options_ == NULL) {
    optimization_problem_options_ = new ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.SparsePoseGraphOptions.optimization_problem_options)
  return optimization_problem_options_;
}
inline ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* SparsePoseGraphOptions::release_optimization_problem_options() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.SparsePoseGraphOptions.optimization_problem_options)
  clear_has_optimization_problem_options();
  ::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* temp = optimization_problem_options_;
  optimization_problem_options_ = NULL;
  return temp;
}
inline void SparsePoseGraphOptions::set_allocated_optimization_problem_options(::cartographer::mapping::sparse_pose_graph::proto::OptimizationProblemOptions* optimization_problem_options) {
  delete optimization_problem_options_;
  optimization_problem_options_ = optimization_problem_options;
  if (optimization_problem_options) {
    set_has_optimization_problem_options();
  } else {
    clear_has_optimization_problem_options();
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.SparsePoseGraphOptions.optimization_problem_options)
}

// optional int32 max_num_final_iterations = 6;
inline bool SparsePoseGraphOptions::has_max_num_final_iterations() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SparsePoseGraphOptions::set_has_max_num_final_iterations() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SparsePoseGraphOptions::clear_has_max_num_final_iterations() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SparsePoseGraphOptions::clear_max_num_final_iterations() {
  max_num_final_iterations_ = 0;
  clear_has_max_num_final_iterations();
}
inline ::google::protobuf::int32 SparsePoseGraphOptions::max_num_final_iterations() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.max_num_final_iterations)
  return max_num_final_iterations_;
}
inline void SparsePoseGraphOptions::set_max_num_final_iterations(::google::protobuf::int32 value) {
  set_has_max_num_final_iterations();
  max_num_final_iterations_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.max_num_final_iterations)
}

// optional double global_sampling_ratio = 5;
inline bool SparsePoseGraphOptions::has_global_sampling_ratio() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SparsePoseGraphOptions::set_has_global_sampling_ratio() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SparsePoseGraphOptions::clear_has_global_sampling_ratio() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SparsePoseGraphOptions::clear_global_sampling_ratio() {
  global_sampling_ratio_ = 0;
  clear_has_global_sampling_ratio();
}
inline double SparsePoseGraphOptions::global_sampling_ratio() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.global_sampling_ratio)
  return global_sampling_ratio_;
}
inline void SparsePoseGraphOptions::set_global_sampling_ratio(double value) {
  set_has_global_sampling_ratio();
  global_sampling_ratio_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.global_sampling_ratio)
}

// optional bool log_residual_histograms = 9;
inline bool SparsePoseGraphOptions::has_log_residual_histograms() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void SparsePoseGraphOptions::set_has_log_residual_histograms() {
  _has_bits_[0] |= 0x00000100u;
}
inline void SparsePoseGraphOptions::clear_has_log_residual_histograms() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void SparsePoseGraphOptions::clear_log_residual_histograms() {
  log_residual_histograms_ = false;
  clear_has_log_residual_histograms();
}
inline bool SparsePoseGraphOptions::log_residual_histograms() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.log_residual_histograms)
  return log_residual_histograms_;
}
inline void SparsePoseGraphOptions::set_log_residual_histograms(bool value) {
  set_has_log_residual_histograms();
  log_residual_histograms_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.log_residual_histograms)
}

// optional double global_constraint_search_after_n_seconds = 10;
inline bool SparsePoseGraphOptions::has_global_constraint_search_after_n_seconds() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void SparsePoseGraphOptions::set_has_global_constraint_search_after_n_seconds() {
  _has_bits_[0] |= 0x00000080u;
}
inline void SparsePoseGraphOptions::clear_has_global_constraint_search_after_n_seconds() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void SparsePoseGraphOptions::clear_global_constraint_search_after_n_seconds() {
  global_constraint_search_after_n_seconds_ = 0;
  clear_has_global_constraint_search_after_n_seconds();
}
inline double SparsePoseGraphOptions::global_constraint_search_after_n_seconds() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.global_constraint_search_after_n_seconds)
  return global_constraint_search_after_n_seconds_;
}
inline void SparsePoseGraphOptions::set_global_constraint_search_after_n_seconds(double value) {
  set_has_global_constraint_search_after_n_seconds();
  global_constraint_search_after_n_seconds_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.global_constraint_search_after_n_seconds)
}

// optional bool use_odom = 11;
inline bool SparsePoseGraphOptions::has_use_odom() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void SparsePoseGraphOptions::set_has_use_odom() {
  _has_bits_[0] |= 0x00000200u;
}
inline void SparsePoseGraphOptions::clear_has_use_odom() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void SparsePoseGraphOptions::clear_use_odom() {
  use_odom_ = false;
  clear_has_use_odom();
}
inline bool SparsePoseGraphOptions::use_odom() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.SparsePoseGraphOptions.use_odom)
  return use_odom_;
}
inline void SparsePoseGraphOptions::set_use_odom(bool value) {
  set_has_use_odom();
  use_odom_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.SparsePoseGraphOptions.use_odom)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace proto
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_2fproto_2fsparse_5fpose_5fgraph_5foptions_2eproto__INCLUDED
