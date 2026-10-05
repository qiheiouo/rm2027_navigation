"""Readout of the existing assignment; no tracking, prediction or rasterizer.

Only associated measured endpoints are retained during coasting. A failed batch
clears the private cache without changing the public tracker/prediction output.
"""
from dataclasses import dataclass
import math

from .core import Point2D


@dataclass(frozen=True)
class MemberRecord:
    track_id: int
    observation_ns: int
    association_sequence: int
    detection_index: int
    centroid: Point2D
    local_endpoints: tuple[Point2D, ...]
    source_member_ids: tuple[int, ...]


@dataclass(frozen=True)
class MemberBatch:
    complete: bool
    reason: str
    records: tuple[MemberRecord, ...] = ()


class ObservedMemberStore:
    def __init__(self, max_tracks=64, max_points=4096):
        if (type(max_tracks) is not int or not 1 <= max_tracks <= 256
                or type(max_points) is not int or not 1 <= max_points <= 4096):
            raise ValueError("invalid observed member budget")
        self.max_tracks, self.max_points = max_tracks, max_points
        self._records = {}
        self._last_source = self._last_sequence = None

    def _invalid(self, reason):
        self._records.clear()
        return MemberBatch(False, reason)

    def update(self, update, detections, members, points, source_member_ids,
               source_ns, sequence):
        if update.time_reset:
            self._records.clear()
            self._last_source = None
        if (type(source_ns) is not int or source_ns <= 0
                or type(sequence) is not int or not 0 <= sequence < 2**64
                or (self._last_source is not None and source_ns <= self._last_source)
                or (self._last_sequence is not None and sequence <= self._last_sequence)):
            return self._invalid("source_identity")
        self._last_source, self._last_sequence = source_ns, sequence
        if len(update.tracks) > self.max_tracks or len(points) > self.max_points:
            return self._invalid("members_budget")
        if (len(detections) != len(members) or len(points) != len(source_member_ids)
                or any(type(i) is not int or not 0 <= i < 2**32 for i in source_member_ids)
                or len(set(source_member_ids)) != len(source_member_ids)):
            return self._invalid("members_invalid")
        live_ids = {t.track_id for t in update.tracks}
        assignments = update.associations
        if (len(live_ids) != len(update.tracks)
                or len(assignments) != len(detections)
                or len({tid for tid, _ in assignments}) != len(assignments)
                or {i for _, i in assignments} != set(range(len(detections)))
                or any(tid not in live_ids for tid, _ in assignments)):
            return self._invalid("association_incomplete")
        records = {tid: record for tid, record in self._records.items() if tid in live_ids}
        used = set()
        for tid, index in assignments:
            component, centroid = members[index], detections[index].centroid
            if (not component or any(type(i) is not int or not 0 <= i < len(points)
                                     for i in component)
                    or len(set(component)) != len(component) or used.intersection(component)
                    or not all(math.isfinite(v) for v in (centroid.x, centroid.y))):
                return self._invalid("members_invalid")
            used.update(component)
            offsets = tuple(Point2D(points[i].x-centroid.x, points[i].y-centroid.y)
                            for i in component)
            if not all(math.isfinite(v) for p in offsets for v in (p.x, p.y)):
                return self._invalid("members_invalid")
            records[tid] = MemberRecord(tid, source_ns, sequence, index, centroid,
                offsets, tuple(source_member_ids[i] for i in component))
        if set(records) != live_ids:
            return self._invalid("members_missing")
        if sum(len(r.local_endpoints) for r in records.values()) > self.max_points:
            return self._invalid("members_budget")
        self._records = records
        return MemberBatch(True, "ok", tuple(records[tid] for tid in sorted(records)))


def make_envelope(prediction, batch, producer_id, generation, sequence):
    """Embed the once-built public value; never rebuild its prediction math."""
    from geometry_msgs.msg import Point
    from rclpy.time import Time
    from rm_r4_interfaces.msg import ObservedPredictionEnvelope, ObservedTrackMembers

    output = ObservedPredictionEnvelope()
    output.schema = output.SCHEMA
    output.producer_id = producer_id
    output.producer_generation = generation
    output.sequence = sequence
    output.prediction = prediction
    output.complete, output.reason = batch.complete, batch.reason
    if (prediction.schema != prediction.SCHEMA_OBSERVATION_ANCHOR
            or prediction.authority != prediction.AUTHORITY_SHADOW_ONLY):
        output.complete, output.reason = False, "public_contract"
    elif not prediction.complete:
        output.complete, output.reason = False, "public_incomplete"
    records = {r.track_id: r for r in batch.records}
    public_ids = {t.track_id for t in prediction.tracks}
    if output.complete and (len(public_ids) != len(prediction.tracks)
            or set(records) != public_ids or prediction.total_track_count != len(public_ids)):
        output.complete, output.reason = False, "track_identity"
    if output.complete:
        for track in prediction.tracks:
            r = records[track.track_id]
            if (Time.from_msg(track.last_observation_stamp).nanoseconds != r.observation_ns
                    or r.observation_ns > Time.from_msg(prediction.header.stamp).nanoseconds
                    or r.association_sequence > sequence):
                output.complete, output.reason = False, "observation_identity"
                output.tracks = []
                break
            item = ObservedTrackMembers()
            item.track_id = r.track_id
            item.last_observation_stamp = track.last_observation_stamp
            item.association_sequence = r.association_sequence
            item.detection_index = r.detection_index
            item.centroid_at_observation = Point(x=r.centroid.x, y=r.centroid.y, z=0.)
            item.local_endpoints = [Point(x=p.x, y=p.y, z=0.) for p in r.local_endpoints]
            item.source_member_ids = list(r.source_member_ids)
            output.tracks.append(item)
    return output
