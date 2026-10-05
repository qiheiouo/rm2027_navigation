#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include "fixtures.hpp"
using namespace r4_test;

TEST(Prediction, ImmutableOnceAdvancedCoastingAndIntegerEpoch)
{
  auto e = envelope(); const auto snap = PredictionSnapshot::freeze(e, source + 50000000, "map", body());
  TemporalSoftField field(snap); const auto initial = field.translated_cells(0, 0);
  EXPECT_EQ(snap.stage_epoch(0), source + 50000000);
  EXPECT_EQ(snap.stage_epoch(30), source + 1550000000);
  EXPECT_NEAR(initial.front().x, 1., 1e-12);
  EXPECT_NEAR(field.translated_cells(0, 30).front().x - initial.front().x, .75, 1e-12);
  e.tracks[0].local_endpoints[0].x = 2.; e.prediction.tracks[0].position.x = 8.;
  EXPECT_DOUBLE_EQ(field.translated_cells(0, 0).front().x, initial.front().x);
  const auto reused = PredictionSnapshot::freeze(envelope(), source + 100000000, "map", body());
  EXPECT_EQ(reused.receipt_digest(), snap.receipt_digest());
  EXPECT_THROW(field.translated_cells(0, 31), ContractError);
}
TEST(Prediction, TentativeSupportRemainsWithoutFutureMotion)
{
  auto e = envelope(); e.prediction.tracks[0].state = e.prediction.tracks[0].STATE_TENTATIVE;
  auto s = PredictionSnapshot::freeze(e, source + 50000000, "map", body()); TemporalSoftField f(s);
  EXPECT_EQ(f.translated_cells(0, 0).front().x, f.translated_cells(0, 30).front().x);
  EXPECT_FALSE(s.tracks()[0].local_cell_centres.empty());
}
TEST(Prediction, PublicContractAndIncompleteInputCannotBecomeClearance)
{
  auto invalid = [](auto change) {
      auto e = envelope(); change(e);
      EXPECT_THROW(PredictionSnapshot::freeze(e, source + 50000000, "map", body()), ContractError);
    };
  invalid([](auto & e) {e.complete = false;});
  invalid([](auto & e) {e.prediction.complete = false;});
  invalid([](auto & e) {e.prediction.schema = e.prediction.SCHEMA;});
  invalid([](auto & e) {e.prediction.authority = "control";});
  invalid([](auto & e) {e.prediction.header.frame_id = "odom";});
  invalid([](auto & e) {e.prediction.total_track_count = 0;});
  invalid([](auto & e) {e.prediction.prediction_dt = .2;});
  invalid([](auto & e) {e.prediction.tracks[0].prediction[3].x += .1;});
  invalid([](auto & e) {e.prediction.tracks[0].velocity.x = std::numeric_limits<double>::quiet_NaN();});
  EXPECT_THROW(PredictionSnapshot::freeze(envelope(), source + 401000000, "map", body()), ContractError);
  EXPECT_THROW(PredictionSnapshot::freeze(envelope(), source - 1, "map", body()), ContractError);
}
TEST(Prediction, MembersAreBoundToObservationAndActualAssignment)
{
  auto invalid = [](auto change) {
      auto e = envelope(); change(e);
      EXPECT_THROW(PredictionSnapshot::freeze(e, source + 50000000, "map", body()), ContractError);
    };
  invalid([](auto & e) {e.tracks.clear();});
  invalid([](auto & e) {e.tracks[0].track_id += 1;});
  invalid([](auto & e) {e.tracks[0].last_observation_stamp.nanosec += 1;});
  invalid([](auto & e) {e.tracks[0].association_sequence = e.sequence + 1;});
  invalid([](auto & e) {e.tracks[0].source_member_ids[1] = e.tracks[0].source_member_ids[0];});
  invalid([](auto & e) {e.tracks[0].local_endpoints.pop_back();});
  invalid([](auto & e) {e.tracks[0].local_endpoints[0].x = .5;});
  invalid([](auto & e) {e.tracks[0].centroid_at_observation.x += .1;});
  invalid([](auto & e) {e.prediction.tracks[0].state = e.prediction.tracks[0].STATE_CONFIRMED;});
  ConsumptionPolicy p; p.max_members = 2;
  EXPECT_THROW(PredictionSnapshot::freeze(envelope(), source, "map", body(), p), ContractError);
  auto e = envelope(); e.prediction.header.stamp = stamp(source + 350000000); align(e);
  EXPECT_THROW(PredictionSnapshot::freeze(e, source + 350000000, "map", body()), ContractError);
}
TEST(Receipt, ReuseMutationAndFailureDoNotRestoreOldWarmInput)
{
  auto e = envelope(); ReceiptGate gate;
  EXPECT_TRUE(gate.accept(PredictionSnapshot::freeze(e, source, "map", body())));
  EXPECT_FALSE(gate.accept(PredictionSnapshot::freeze(e, source + 50000000, "map", body())));
  auto mutated = e; mutated.tracks[0].centroid_at_observation.x += .1; align(mutated);
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(mutated, source + 50000000, "map", body())), ContractError);
  EXPECT_FALSE(gate.usable());
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(e, source + 50000000, "map", body())), ContractError);
  e.sequence += 1; e.prediction.header.stamp = stamp(source + 100000000); align(e);
  EXPECT_TRUE(gate.accept(PredictionSnapshot::freeze(e, source + 100000000, "map", body())));
  EXPECT_TRUE(gate.usable());
}
TEST(Receipt, MemberOrderAndAllPublicMetadataBelongToIdentity)
{
  auto e = envelope(); const auto before = PredictionSnapshot::freeze(e, source, "map", body());
  std::swap(e.tracks[0].local_endpoints[0], e.tracks[0].local_endpoints[1]);
  std::swap(e.tracks[0].source_member_ids[0], e.tracks[0].source_member_ids[1]);
  EXPECT_NE(PredictionSnapshot::freeze(e, source, "map", body()).receipt_digest(), before.receipt_digest());
  e = envelope(); e.prediction.tracks[0].observation_count += 1;
  EXPECT_NE(PredictionSnapshot::freeze(e, source, "map", body()).receipt_digest(), before.receipt_digest());
}
TEST(Receipt, GenerationRestartAndPolicyChangesInvalidateWarm)
{
  auto e = envelope(); ReceiptGate gate; gate.accept(PredictionSnapshot::freeze(e, source, "map", body()));
  auto changed_body = body(); changed_body.padding += .01;
  EXPECT_TRUE(gate.accept(PredictionSnapshot::freeze(e, source + 50000000, "map", changed_body)));
  e.producer_generation = 1;
  EXPECT_TRUE(gate.accept(PredictionSnapshot::freeze(e, source + 100000000, "map", body())));
  e.producer_generation = 0;
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(e, source + 150000000, "map", body())), ContractError);
  e.producer_id = "other-producer";
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(e, source + 200000000, "map", body())), ContractError);
  gate.reset(); EXPECT_TRUE(gate.accept(PredictionSnapshot::freeze(e, source, "map", body())));
}
TEST(Receipt, OlderSequencesAndDiscontinuousAnchorsAreRejected)
{
  auto e = envelope(); ReceiptGate gate; gate.accept(PredictionSnapshot::freeze(e, source, "map", body()));
  e.sequence = 3; e.prediction.header.stamp = stamp(source + 50000000);
  e.tracks[0].centroid_at_observation.x = 3.; align(e);
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(e, source + 50000000, "map", body())), ContractError);
  EXPECT_FALSE(gate.usable());
  e = envelope();
  EXPECT_THROW(gate.accept(PredictionSnapshot::freeze(e, source, "map", body())), ContractError);
}
TEST(SoftField, GradientMatchesFiniteDifferenceAndFuturePlateauIsOnlySoft)
{
  TemporalSoftField f(PredictionSnapshot::freeze(envelope(), source + 50000000, "map", body()));
  const Vec2 p{1.65, .15}; const auto sample = f.sample(p, 0); const double eps = 1e-6;
  EXPECT_GT(sample.residual, 0.); EXPECT_FALSE(sample.plateau);
  EXPECT_NEAR(sample.gradient.x, (f.sample({p.x + eps, p.y}, 0).residual -
    f.sample({p.x - eps, p.y}, 0).residual) / (2 * eps), 1e-6);
  const auto future = f.translated_cells(0, 30).front();
  EXPECT_DOUBLE_EQ(f.sample(future, 30).residual, 8.);
  EXPECT_TRUE(f.sample(future, 30).plateau);
  EXPECT_DOUBLE_EQ(f.sample({10., 10.}, 30).residual, 0.);
}
TEST(SoftField, ActualFootprintYawAndExplicitMotionErrorAreUsed)
{
  auto b = body(); b.yaw = std::acos(-1.) / 2; const auto support = b.support();
  EXPECT_NEAR(support.xmax, .29, 1e-12); EXPECT_NEAR(support.ymax, .34, 1e-12);
  ConsumptionPolicy policy; policy.motion_error_speed = .2;
  TemporalSoftField f(PredictionSnapshot::freeze(envelope(), source, "map", b, policy));
  auto future = f.translated_cells(0, 30).back(); future.x += .6;
  EXPECT_GT(f.sample(future, 30).residual, 0.);
  b.padding = -.01; EXPECT_THROW(b.support(), ContractError);
  b = body(); b.padding = 0.; EXPECT_THROW(b.support(), ContractError);
  b = body(); b.static_clearance = .049; EXPECT_THROW(b.support(), ContractError);
}
TEST(SoftField, EmptyCompleteSceneDoesNotInventShapeFromExtent)
{
  auto e = envelope(); e.prediction.tracks.clear(); e.tracks.clear(); e.prediction.total_track_count = 0;
  TemporalSoftField f(PredictionSnapshot::freeze(e, source, "map", body()));
  EXPECT_DOUBLE_EQ(f.sample({0., 0.}, 10).residual, 0.);
}
TEST(Follow, IndependentProgressAndTerminalZeroWithoutForcedStop)
{
  const auto route = PreparedCorridor::prepare(path(), grid(), body(), 1);
  TemporalSoftField f(PredictionSnapshot::freeze(envelope(), source, "map", body()));
  const auto free = free_progress_residual(route, f, {0., .1}, {.2, 0.}, 1., .1, 0, .6);
  EXPECT_NEAR(free.contour, .1, 1e-12); EXPECT_NEAR(free.projection, .1, 1e-12);
  EXPECT_NEAR(free.cruise, -.5, 1e-12);
  const double eps = 1e-6;
  const auto forward = free_progress_residual(route, f, {0., .1}, {.2, 0.}, 1. + eps, .1, 0, .6);
  const auto backward = free_progress_residual(route, f, {0., .1}, {.2, 0.}, 1. - eps, .1, 0, .6);
  EXPECT_NEAR(free.progress_gradient, (forward.cost - backward.cost) / (2 * eps), 1e-6);
  EXPECT_DOUBLE_EQ(free_progress_residual(route, f, {1., 0.}, {.5, 0.}, 2., .5, 30, .6).cost, 0.);
}
TEST(Prediction, CanonicalProducerCdrCrossLanguage)
{
  const char * folder = std::getenv("R4_PRODUCER_CDR_FIXTURES");
  if (!folder) {GTEST_SKIP() << "canonical producer CDR fixture directory not provided";}
  rclcpp::Serialization<rm_r4_interfaces::msg::ObservedPredictionEnvelope> serializer;
  for (const std::string name : {"observed", "coasting", "reset"}) {
    std::ifstream stream(std::string(folder) + "/" + name + ".cdr", std::ios::binary);
    ASSERT_TRUE(stream.good());
    std::vector<char> bytes((std::istreambuf_iterator<char>(stream)), {});
    rclcpp::SerializedMessage message(bytes.size()); auto & buffer = message.get_rcl_serialized_message();
    std::copy(bytes.begin(), bytes.end(), buffer.buffer); buffer.buffer_length = bytes.size();
    rm_r4_interfaces::msg::ObservedPredictionEnvelope e; serializer.deserialize_message(&message, &e);
    const auto & stamp = e.prediction.header.stamp;
    const auto now = static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
    EXPECT_NO_THROW(PredictionSnapshot::freeze(e, now + 50000000, "map", body()));
    EXPECT_EQ(e.producer_generation, name == "reset" ? 1u : 0u);
  }
}

TEST(Receipt, SingleConsumptionBoundaryInvalidatesOnFreezeFailureAndClockRollback)
{
  ReceiptGate gate; auto e = envelope();
  EXPECT_TRUE(gate.consume(e, source, "map", body()).reset_warm);
  EXPECT_FALSE(gate.consume(e, source + 100000000, "map", body()).reset_warm);
  EXPECT_THROW(gate.consume(e, source + 50000000, "map", body()), ContractError);
  EXPECT_FALSE(gate.usable());
  EXPECT_THROW(gate.consume(e, source + 50000000, "map", body()), ContractError);
  gate.reset(); gate.consume(e, source, "map", body());
  e.complete = false;
  EXPECT_THROW(gate.consume(e, source + 50000000, "map", body()), ContractError);
  EXPECT_FALSE(gate.usable());
}
TEST(Follow, FrameAndActualBodyPolicyMustMatchPreparedStaticValues)
{
  const auto route = PreparedCorridor::prepare(path(), grid(), body(), 1);
  auto other_body = body(); other_body.padding += .01;
  TemporalSoftField f(PredictionSnapshot::freeze(envelope(), source, "map", other_body));
  EXPECT_THROW(free_progress_residual(route, f, {0., 0.}, {.2, 0.}, 1., .2, 0, .6), ContractError);
}
