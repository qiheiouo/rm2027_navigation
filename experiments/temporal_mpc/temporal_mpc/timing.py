"""Recorded clocks have distinct domains; this module never repairs timestamps."""
import math
from .contracts import stamp_ns, ContractError


def request_clock_status(epoch, frame, now):
    if type(epoch) is not int or epoch < 0:
        return 'invalid_epoch'
    if frame != 'map':
        return 'frame'
    if now < epoch:
        return 'future'
    if now-epoch > 100_000_000:
        return 'stale'
    return 'current'


def raw_request(request):
    try: epoch=stamp_ns(request.header.stamp)
    except ContractError: epoch=None
    p=request.pose;v=request.velocity
    values=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,
            p.orientation.z,p.orientation.w,v.linear.x,v.linear.y,v.linear.z,
            v.angular.x,v.angular.y,v.angular.z]
    return dict(request_epoch_ns=epoch,request_frame=request.header.frame_id,
                generation=int(request.generation),
                raw_request_fields=[float(x) if math.isfinite(x) else None for x in values],
                raw_request_invalid_indices=[i for i,x in enumerate(values) if not math.isfinite(x)])
