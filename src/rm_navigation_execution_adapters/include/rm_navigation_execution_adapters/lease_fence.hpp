#pragma once
#include <optional>
#include "rm_navigation_execution_adapters/current_geometry.hpp"

namespace rm_navigation_execution_adapters
{
inline constexpr int64_t source_lease_ns = 75000000;
// Supplied by the original host's registered action/switch responsibility.
// This adapter consumes the fence; neither arrival nor identity grants motion.
struct ExecutionFence
{
  std::string authority_instance, producer_instance, producer_id, execution_id;
  std::string owner_instance, base_frame, clock_domain, clock_generation;
  std::string plan_revision, body_revision, limits_revision;
  uint64_t authority_epoch{};
  bool active{};
};
bool same_fence(const ExecutionFence & a, const ExecutionFence & b);
enum class ProposalKind {Normal, Revoke};
struct ProposalProvenance
{
  std::string input_digest, receipt_digest, path_digest, map_digest, limits_digest, body_digest;
};
struct ProposalPacket
{
  ExecutionFence fence;
  uint64_t cycle_sequence{};
  ProposalKind kind{ProposalKind::Normal};
  Twist command;
  int64_t acquired_epoch_ns{}, sent_epoch_ns{}, source_elapsed_ns{}, remaining_ns{};
  std::optional<ProposalProvenance> provenance{};  // Atomic R4 identity; native sources may omit.
};
struct TransportWitness
{
  // Local original receiver evidence, never a producer-provided trust flag.
  std::string owner_instance, local_clock_instance, clock_domain, clock_generation;
  int64_t receipt_epoch_ns{}, receipt_steady_ns{};
  int64_t clock_offset_bound_ns{}, source_clock_drift_bound_ns{}, transport_delay_bound_ns{};
  bool verified{};  // Default false; live verification is a host obligation.
};
class ReceivedProposal
{
public:
  const ProposalPacket & source() const {return packet_;}
  const std::string & local_clock_instance() const {return local_clock_;}
  int64_t deadline_steady_ns() const {return deadline_;}
  int64_t receipt_steady_ns() const {return receipt_;}
private:
  ProposalPacket packet_;
  std::string local_clock_;
  int64_t deadline_{}, receipt_{};
  ReceivedProposal(ProposalPacket p, std::string clock, int64_t deadline, int64_t receipt);
  friend class ProposalReceiptGate;
};
struct ProposalReceipt
{
  std::optional<ReceivedProposal> proposal;
  bool revoked{};
  std::string reason;
};
// Original input callback serializes calls. No Twist cache, grant issuance,
// motion selection, scheduler, publisher or automatic restart registration.
class ProposalReceiptGate
{
public:
  ProposalReceiptGate(std::string owner_instance, std::string local_clock_instance,
    std::string registered_authority_instance);
  bool observe_fence(const ExecutionFence & fence);
  ProposalReceipt receive(const ProposalPacket & packet, const TransportWitness & witness);
  bool permits(const ReceivedProposal & proposal) const;
  const std::optional<ExecutionFence> & observed_fence() const {return fence_;}
private:
  std::string owner_, local_clock_, authority_;
  std::optional<ExecutionFence> fence_;
  uint64_t high_water_{};
  int64_t last_acquired_epoch_{}, last_receipt_steady_{};
  bool revoked_{};
};
struct SendContext
{
  std::string owner_instance, local_clock_instance, candidate_identity;
  std::string map_revision, body_revision, limits_revision, current_frame;
  uint64_t output_sequence{};
  Twist actual_candidate;  // Original final smoothing/deadband/encoding result.
  int64_t inspection_epoch_ns{}, inspection_steady_ns{}, now_steady_ns{}, send_budget_ns{};
  bool local_clock_verified{};
};
struct AdmittedCommand
{
  ProposalPacket source;
  Twist actual_command;
  std::string owner_instance, candidate_identity, map_revision, local_clock_instance;
  uint64_t output_sequence{};
  int64_t valid_until_steady_ns{};
};
struct SendAdmission {std::optional<AdmittedCommand> command; std::string reason;};
// Geometry provenance + active fence + original deadline, for the original
// final send branch. No expiry extension or velocity generation/selection.
SendAdmission admit_for_send(const ReceivedProposal & proposal, const ProposalReceiptGate & gate,
  const GeometryResult & geometry, const SendContext & context);
}  // namespace rm_navigation_execution_adapters
