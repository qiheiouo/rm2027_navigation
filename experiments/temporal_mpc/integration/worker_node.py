#!/usr/bin/env python3
"""Asynchronous QP proposals only; no command publisher and no static search."""
import json
import math
import time
import subprocess
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from rm_temporal_mpc_msgs.msg import Plan, StateRequest, Proposal
from temporal_mpc.contracts import PublicAdapter, ContractError, stamp_ns, predict, causal_snapshot
from temporal_mpc.frontend import prepare_route
from temporal_mpc.realtime_qp import RealtimeMPC
from temporal_mpc.candidates import CandidateMPC
from temporal_mpc.diagnostics import input_identity, lateral_state
from temporal_mpc.timing import request_clock_status, raw_request


class Worker(Node):
    def __init__(self):
        super().__init__("temporal_mpc_worker")
        binary = self.declare_parameter("frontend", "").value
        if not binary:
            raise ValueError("explicit frozen T-DT frontend executable required")
        self.binary = binary
        self.corridor_range=self.declare_parameter('corridor_range',6.).value
        mode = self.declare_parameter("geometry_mode", "nominal_diameter").value
        self.adapter = PublicAdapter(geometry_mode=mode)
        self.strategy=self.declare_parameter('strategy','single').value
        if self.strategy not in ('single','portfolio'):raise ValueError('unregistered strategy')
        self.mpc = RealtimeMPC() if self.strategy=='single' else CandidateMPC()
        self.plan = self.map = self.route = self.snapshot = None
        self.snapshots = deque(maxlen=4)
        self.generation = self.map_revision = None
        self.last_request_ns = -1
        self.fault = ""
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(Proposal, "temporal_mpc/proposal", 1)
        self.diagnostic = self.create_publisher(String, "temporal_mpc/solver_diagnostic", 1)
        self.timing = self.create_publisher(String, "temporal_mpc/worker_timing", 100)
        self.callback_sequence=0
        self.create_subscription(Plan, "temporal_mpc/plan", self.on_plan, qos)
        self.create_subscription(OccupancyGrid, "map", self.on_map, qos)
        self.create_subscription(DynamicObstaclePredictionArray, "dynamic_obstacle_predictions", self.on_prediction, 1)
        self.create_subscription(StateRequest, "temporal_mpc/state", self.solve, 1)
        # Experimental fault injection; launch/test scope only, no formal endpoint.
        self.create_subscription(String, "temporal_mpc/test_fault", self.on_fault, 1)

    def on_fault(self, message):
        if message.data in ("", "silent", "timeout", "infeasible", "exception", "nan"):
            self.fault = message.data

    def on_map(self, message):
        self.route = None
        self.mpc.reset()
        self.map = None
        i = message.info
        q = i.origin.orientation
        try:
            revision = stamp_ns(message.header.stamp)
            if (message.header.frame_id != "map" or not 3 <= i.width <= 1000 or not 3 <= i.height <= 1000
                    or i.width*i.height > 100000 or len(message.data) != i.width*i.height
                    or not .01 <= i.resolution <= .2 or (q.x, q.y, q.z, q.w) != (0., 0., 0., 1.)
                    or not np.isfinite([i.origin.position.x,i.origin.position.y]).all()):
                raise ContractError("invalid static map")
            data = np.asarray(message.data, int).reshape(i.height, i.width)
            if np.any((data < -1) | (data > 100)):
                raise ContractError("invalid occupancy bytes")
            # Explicit OccupancyGrid -> raw Nav2 lethal/unknown translation.
            costs = np.where(data < 0, 255, np.where(data >= 65, 254, 0)).astype(np.uint8)
            self.map = (costs, i.resolution, (i.origin.position.x, i.origin.position.y))
            self.map_revision = revision
            self.prepare()
        except (ValueError, ContractError) as error:
            self.get_logger().warning(str(error))

    def on_plan(self, message):
        if self.generation == message.generation:
            return
        self.route = None
        self.mpc.reset()
        self.plan, self.generation = message.path, message.generation
        self.prepare()

    def prepare(self):
        timing=self.new_timing('frontend')
        try:self._prepare()
        finally:
            timing.update(generation=self.generation,map_revision=self.map_revision,
                          route_available=self.route is not None)
            self.emit_timing(timing)

    def _prepare(self):
        if self.plan is None or self.map is None:
            return
        try:
            if self.plan.header.frame_id != "map" or not 2 <= len(self.plan.poses) <= 4096:
                raise ContractError("planner path frame/count")
            xy = np.array([[p.pose.position.x,p.pose.position.y] for p in self.plan.poses])
            if not np.isfinite(xy).all():
                raise ContractError("invalid planner path")
            # Drop duplicate positions only. Never simplify a corner/alter topology.
            xy = xy[np.r_[True, np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-7]]
            # Remove exactly collinear interior positions from dense Nav2 paths.
            # Each replacement segment still passes the frozen raw-map bridge.
            keep=[0]
            for k in range(1,len(xy)-1):
                a,b=xy[k]-xy[keep[-1]],xy[k+1]-xy[k]
                if abs(a[0]*b[1]-a[1]*b[0])>1e-7 or np.dot(a,b)<0:
                    keep.append(k)
            if len(xy)>1:
                keep.append(len(xy)-1)
            xy=xy[keep]
            if len(xy)>256:
                raise ContractError("planner must compress path to <=256 distinct points")
            self.route = prepare_route(self.binary, *self.map, xy[0], xy[-1], supplied_path=xy,
                                       corridor_range=self.corridor_range)
        except (ValueError, ContractError, OSError, TimeoutError, subprocess.TimeoutExpired) as error:
            self.route = None
            self.get_logger().warning(f"planner/corridor rejected: {error}")

    def on_prediction(self, message):
        timing=self.new_timing('prediction')
        try:timing['source_ns']=stamp_ns(message.header.stamp)
        except ContractError:timing['source_ns']=None
        self.snapshot = None
        now=self.get_clock().now().nanoseconds
        timing['decision_clock_ns']=now
        try:
            self.snapshot = self.adapter.consume(message, now)
            self.snapshots.append(self.snapshot)
            timing['disposition']='accepted'
        except ContractError as error:
            self.snapshots.clear()
            self.get_logger().debug(str(error))
            timing.update(disposition='rejected',reason=str(error))
        finally:self.emit_timing(timing)

    def new_timing(self, kind):
        self.callback_sequence+=1
        return dict(schema='temporal_mpc_worker_timing/v1',kind=kind,
                    callback_id=self.callback_sequence,
                    callback_start_monotonic_ns=time.monotonic_ns(),
                    callback_entry_clock_ns=self.get_clock().now().nanoseconds,
                    disposition='not_completed',proposal_published=False,
                    dds_source_timestamp_ns=None,dds_received_timestamp_ns=None,
                    dds_info_scope='Installed Humble executor supplies one message argument; metadata unavailable.')

    def emit_timing(self, timing):
        timing['callback_body_end_monotonic_ns']=time.monotonic_ns()
        timing['callback_exit_clock_ns']=self.get_clock().now().nanoseconds
        self.timing.publish(String(data=json.dumps(timing,allow_nan=False)))

    def solve(self, request):
        started = time.perf_counter()
        timing=self.new_timing('request');timing.update(raw_request(request))
        try:self.solve_request(request,timing,started)
        finally:self.emit_timing(timing)

    def solve_request(self, request, timing, started):
        if self.fault == "silent":
            timing['disposition']='injected_silent'
            return
        result = Proposal()
        result.header = request.header
        result.generation = request.generation
        result.map_revision = self.map_revision or 0
        result.period = .05
        result.fallback_requested = True
        result.reason = "missing plan/map/prediction"
        iterations = 0
        constraint_min = None
        solver_status = "not_run"
        snapshot=x=window=solved=None;validated=False
        before=lateral_state(self.mpc.lateral)
        try:
            epoch = stamp_ns(request.header.stamp)
            # Duplicated timer/compute requests are not queued as extra QPs.
            if epoch <= self.last_request_ns:
                timing['disposition']='duplicate'
                return
            self.last_request_ns = epoch
            now = self.get_clock().now().nanoseconds
            clock_status=request_clock_status(epoch,request.header.frame_id,now)
            timing.update(decision_clock_ns=now,request_signed_age_ns=now-epoch,
                          request_clock_status=clock_status)
            if clock_status!='current':
                raise ContractError('state request '+clock_status)
            if self.fault == "timeout":
                # Delay only the worker, while native control continues braking.
                time.sleep(.2)
                raise ContractError("injected timeout")
            if self.fault == "infeasible":
                raise ContractError("injected infeasible")
            if self.fault == "exception":
                raise RuntimeError("injected solver exception")
            if self.route is None or self.snapshot is None or request.generation != self.generation:
                raise ContractError(result.reason)
            q = request.pose.orientation
            if (abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3):
                raise ContractError("nonplanar pose")
            yaw = 2*math.atan2(q.z,q.w)
            x = np.array([request.pose.position.x,request.pose.position.y,yaw,
                          request.velocity.linear.x,request.velocity.linear.y,request.velocity.angular.z])
            snapshot=causal_snapshot(self.snapshots,epoch)
            if snapshot is None:
                # A newer scan may arrive before an older state request callback.
                # Do not backdate it or issue a failure for this ordering alone.
                timing['disposition']='no_causal_snapshot'
                return
            predict(snapshot,epoch,self.mpc.times)
            window = self.route.window(x,epoch,self.mpc.times)
            validated=True
            timing['solver_start_monotonic_ns']=time.monotonic_ns()
            try:solved = self.mpc.solve(x,epoch,snapshot,window)
            finally:timing['solver_end_monotonic_ns']=time.monotonic_ns()
            result.fixed_yaw = yaw
            result.centre_bounds = list(window.centre_bounds)
            result.accelerations = [Vector3(x=float(a[0]),y=float(a[1]),z=float(a[2])) for a in solved.controls]
            result.model_feasible = solved.model_feasible
            result.fallback_requested = solved.fallback_id is not None
            result.reason = solved.reason
            iterations = solved.iterations
            constraint_min = solved.constraint_min
            solver_status = solved.solver_status
            if self.fault == "nan":
                result.accelerations[0].x = float("nan")
        except Exception as error:
            # This boundary contains unexpected solver/data errors. Native plugin
            # independently handles missing/malformed/stale proposals even if this
            # process is killed before publishing this result.
            self.mpc.reset()
            result.reason = f"{type(error).__name__}: {error}"[:200]
            timing['exception_reason']=result.reason
        elapsed = time.perf_counter()-started
        if elapsed>.04:
            result.model_feasible=False
            result.fallback_requested=True
            result.reason="worker cycle deadline"
        timing.update(disposition='proposal',cycle_budget_elapsed_s=elapsed,
                      proposal_feasible=result.model_feasible,proposal_fallback=result.fallback_requested,
                      proposal_reason=result.reason)
        timing['proposal_publish_monotonic_ns']=time.monotonic_ns()
        self.output.publish(result)
        timing['proposal_publish_return_monotonic_ns']=time.monotonic_ns()
        timing['proposal_published']=True
        epoch=timing['request_epoch_ns']
        identity=input_identity(epoch,snapshot,x,window,request.generation,self.map_revision,validated)
        self.diagnostic.publish(String(data=json.dumps({**identity,"epoch_ns":epoch,
            "timing_callback_id":timing['callback_id'],
            "elapsed_s":elapsed,"iterations":iterations,"feasible":result.model_feasible,
            "strategy":self.strategy,"solver_s":0. if solved is None else solved.solver_s,
            "candidates":[] if solved is None else solved.candidate_trace,
            "chosen_candidate":None if solved is None else solved.chosen_candidate,
            "local_reference_before":before,"local_reference_after":lateral_state(self.mpc.lateral),
            "local_reference_mode":self.mpc.lateral.mode,"local_reference_track":self.mpc.lateral.track,
            "fallback":result.fallback_requested,"reason":result.reason,"constraint_min":constraint_min,"solver_status":solver_status},allow_nan=False)))


def main():
    rclpy.init()
    node = Worker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
