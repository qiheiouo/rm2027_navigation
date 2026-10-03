"""Real native C++ output fixture, corruption and complete-grid enforcement."""
from pathlib import Path
import struct
import pytest
from native_witness_io import records

FIXTURE=Path(__file__).resolve().parents[1]/'test/fixtures/native_witness/cycle_0.bin'


def test_real_cpp_output():
    output=list(records(FIXTURE));assert len(output)==1
    first=output[0];assert first['row_count']==11 and first['steps']==30
    assert first['raw_velocity_pose_exact'] and first['aggregate_SG_exact'] and first['actual_command_double_bit_exact']
    for name in ['vx','vy','wz']:
        assert len({struct.pack('<f',first['blocks'][name][row*30]) for row in range(11)})==1


@pytest.mark.parametrize('kind',['header','grid','budget','truncated','ordinal','mask','flag','yaw','tensor','trailing'])
def test_corruption_rejected(tmp_path,kind):
    data=bytearray(FIXTURE.read_bytes())
    if kind=='header':data[0]=0
    elif kind=='grid':struct.pack_into('<I',data,16,10)
    elif kind=='budget':struct.pack_into('<I',data,12,10000)
    elif kind=='truncated':data.pop()
    elif kind=='ordinal':struct.pack_into('<I',data,24,1)
    elif kind=='mask':struct.pack_into('<I',data,28,64)
    elif kind=='flag':struct.pack_into('<I',data,32,2)
    elif kind=='yaw':struct.pack_into('<d',data,40,float('nan'))
    elif kind=='tensor':struct.pack_into('<f',data,48,float('inf'))
    else:data.append(0)
    path=tmp_path/'corrupt.bin';path.write_bytes(data)
    with pytest.raises(ValueError):list(records(path))
