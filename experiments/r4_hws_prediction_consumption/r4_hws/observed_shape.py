"""R4 sidecar from the exact detection-to-track association and its members.

Latest associated raster is retained during coasting. Unlike HWS there is no
history union/decay here: this first slice changes consumption without adding a
second shape-memory model. Public anchor and velocity remain frozen R3 v2 CV.
"""
import math
from .contracts import V2, SHAPE_V1, ContractError, integer, vector
from .tracker_core import Detection, Point2D, MultiObjectTracker, cluster_point_indices


class ObservedShapeTracker:
    def __init__(self):
        self.tracker = MultiObjectTracker(public_anchor_mode='last_observation_cv',
                                         velocity_decay_tau=0., max_prediction_speed=0.,
                                         min_displacement_to_confirm=.15)
        self.shapes = {}
        self.last_source = self.last_sequence = None

    def update(self, points, source_ns, sequence, member_ids=None, occupancy_map=None,
               static_distance=.25):
        """Input is source-time map-frame candidate endpoints, not future truth.

        Static extraction/TF belongs to the existing producer before this call.
        The optional centroid filter preserves each accepted component's IDs.
        """
        integer(source_ns, "source epoch")
        integer(sequence, "source sequence")
        if self.last_source is not None and (source_ns <= self.last_source or sequence <= self.last_sequence):
            self.shapes.clear()
            raise ContractError("producer time reset requires a new tracker instance")
        if len(points) > 4096:
            raise ContractError("candidate endpoint budget")
        points = [Point2D(*vector((p.x, p.y), 2, "candidate endpoint")) for p in points]
        ids = tuple(range(len(points))) if member_ids is None else tuple(member_ids)
        if len(ids) != len(points) or len(set(ids)) != len(ids):
            raise ContractError("candidate IDs must be unique and aligned")
        for value in ids:
            integer(value, "candidate source ID", upper=2**32-1)
        detections, members = [], []
        for component in cluster_point_indices(points, .20, 3, 1.50):
            xs, ys = [points[i].x for i in component], [points[i].y for i in component]
            detection = Detection(Point2D(sum(xs)/len(xs), sum(ys)/len(ys)),
                                  max(max(xs)-min(xs), .20), max(max(ys)-min(ys), .20), len(component))
            if occupancy_map is not None and static_distance > 0.:
                distance = occupancy_map.distance_to_occupied(detection.centroid, static_distance)
                if distance is not None and distance <= static_distance:
                    continue
            detections.append(detection)
            members.append(component)
        update = self.tracker.update(detections, source_ns/1e9, source_stamp_ns=source_ns)
        for track_id, detection_index in update.associations:
            detection, component = detections[detection_index], members[detection_index]
            offsets = [(points[i].x-detection.centroid.x, points[i].y-detection.centroid.y) for i in component]
            resolution = .05
            origin = tuple(math.floor(min(o[a] for o in offsets)/resolution)*resolution for a in range(2))
            cells = tuple(sorted(set((math.floor((o[0]-origin[0])/resolution),
                                      math.floor((o[1]-origin[1])/resolution)) for o in offsets)))
            if len(cells) > 512:
                raise ContractError("observed raster budget")
            self.shapes[track_id] = dict(track_id=track_id, observation_ns=source_ns,
                                        resolution=resolution, origin=origin, cells=cells,
                                        association_sequence=sequence, detection_index=detection_index,
                                        member_ids=tuple(ids[i] for i in component),
                                        provenance='associated_observed_endpoints:uncertified')
        live_ids = {t.track_id for t in update.tracks}
        self.shapes = {tid: shape for tid, shape in self.shapes.items() if tid in live_ids}
        public_tracks, shape_tracks = [], []
        for track in update.tracks:
            if track.track_id not in self.shapes:
                raise ContractError("associated observed shape missing; no extent proxy fallback")
            shape = self.shapes[track.track_id]
            position = (track.position.x, track.position.y)
            public_tracks.append(dict(track_id=track.track_id, state=track.state.value,
                                      observation_ns=shape['observation_ns'], position=position,
                                      velocity=(track.velocity.x, track.velocity.y),
                                      size=(track.size_x, track.size_y),
                                      prediction=tuple((p.x, p.y) for p in track.prediction)))
            shape_tracks.append(dict(shape, anchor=position))
        self.last_source, self.last_sequence = source_ns, sequence
        public = dict(schema=V2, source_ns=source_ns, sequence=sequence, frame='map',
                      authority='shadow_only', complete=True, prediction_steps=15, prediction_dt=.1,
                      total_track_count=len(public_tracks), tracks=public_tracks)
        sidecar = dict(schema=SHAPE_V1, source_ns=source_ns, sequence=sequence,
                       frame='map', complete=True, tracks=shape_tracks)
        return public, sidecar, update
