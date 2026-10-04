import time
from r4_hws.consumer import CycleConsumer
from r4_hws.observed_shape import ObservedShapeTracker
from r4_hws.tracker_core import Point2D


def values(route, epoch, public, shapes):
    return dict(cycle_id=epoch, epoch_ns=epoch, acquired_steady_ns=time.perf_counter_ns(), state_ns=epoch,
                tf_ns=epoch, state=(0., 0., 0., 0., 0., 0.), last_command=(0., 0., 0.), last_command_ns=epoch,
                route=route, public=public, shapes=shapes)


def test_control_cycles_can_reuse_same_receipt_but_cannot_mutate_it(route):
    consumer = CycleConsumer()
    public, shapes, _ = ObservedShapeTracker().update([], 1_000_000_000, 1)
    first = consumer.compute(**values(route, 1_000_000_000, public, shapes))
    second = consumer.compute(**values(route, 1_050_000_000, public, shapes))
    assert first.status == second.status == 'follow'
    assert first.snapshot.digest != second.snapshot.digest
    # Same epoch with changed association evidence is a corrupted receipt.
    tracker = ObservedShapeTracker()
    public, shapes, _ = tracker.update([Point2D(1., -.1), Point2D(1., 0.), Point2D(1., .1)], 1_100_000_000, 2)
    assert consumer.compute(**values(route, 1_100_000_000, public, shapes)).status == 'follow'
    shapes['tracks'][0]['member_ids'] = [99, 100, 101]
    invalid = consumer.compute(**values(route, 1_150_000_000, public, shapes))
    assert invalid.status == 'stop' and 'mutated' in invalid.reason
    assert consumer.arbiter.offer is None


def test_invalid_prediction_clears_live_offer_and_requires_new_receipt(route):
    consumer = CycleConsumer()
    tracker = ObservedShapeTracker()
    public, shapes, _ = tracker.update([], 1_000_000_000, 1)
    assert consumer.compute(**values(route, 1_000_000_000, public, shapes)).status == 'follow'
    public['complete'] = False
    assert consumer.compute(**values(route, 1_050_000_000, public, shapes)).status == 'stop'
    assert consumer.arbiter.offer is None and consumer.mpc.previous is None
    public['complete'] = True
    assert consumer.compute(**values(route, 1_100_000_000, public, shapes)).status == 'stop'
    public, shapes, _ = tracker.update([], 1_150_000_000, 2)
    assert consumer.compute(**values(route, 1_150_000_000, public, shapes)).status == 'follow'


def test_last_sent_command_and_control_clock_are_part_of_entering_gate(route):
    consumer = CycleConsumer()
    public, shapes, _ = ObservedShapeTracker().update([], 1_000_000_000, 1)
    assert consumer.compute(**values(route, 1_000_000_000, public, shapes)).status == 'follow'
    duplicate = consumer.compute(**values(route, 1_000_000_000, public, shapes))
    assert duplicate.status == 'stop' and 'clock order' in duplicate.reason
    public, shapes, _ = ObservedShapeTracker().update([], 1_100_000_000, 2)
    mismatched = values(route, 1_100_000_000, public, shapes)
    mismatched['last_command'] = (.1, 0., 0.)
    output = consumer.compute(**mismatched)
    assert output.status == 'stop' and 'actually sent' in output.reason
