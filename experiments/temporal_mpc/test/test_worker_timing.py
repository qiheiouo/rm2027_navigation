import json
from types import SimpleNamespace as N
import pytest
from temporal_mpc.timing import request_clock_status, raw_request


@pytest.mark.parametrize('epoch,frame,now,status',[
    (None,'map',0,'invalid_epoch'),(-1,'map',0,'invalid_epoch'),
    (0,'odom',0,'frame'),(1_000_000_001,'map',1_000_000_000,'future'),
    (1_000_000_000,'map',1_100_000_001,'stale'),
    (1_000_000_000,'map',1_100_000_000,'current'),(1_000_000_000,'map',1_000_000_000,'current')])
def test_strict_request_clock_domain(epoch,frame,now,status):
    assert request_clock_status(epoch,frame,now)==status


def test_raw_invalid_request_is_retained_without_claiming_consumption():
    v=N(x=0.,y=0.,z=0.);p=N(position=N(x=float('nan'),y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))
    r=N(header=N(stamp=N(sec=-1,nanosec=0),frame_id='odom'),generation=2,pose=p,velocity=N(linear=v,angular=v))
    d=raw_request(r);assert d['request_epoch_ns'] is None and d['raw_request_invalid_indices']==[0]
    assert d['raw_request_fields'][0] is None and d['request_frame']=='odom'
    json.dumps(d,allow_nan=False)
