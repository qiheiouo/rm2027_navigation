"""Decode immutable per-cycle snapshots; reject corrupt/truncated array records."""
from pathlib import Path
import json
import numpy as np

def read_cycle(path):
 path=Path(path);m=json.loads(path.read_text());raw=path.with_suffix('.bin').read_bytes();arrays=[]
 for e in m['events']:
  if e['kind']!='array':continue
  dtype=np.dtype('<'+e['dtype']);offset=e['offset'];size=e['bytes'];shape=e['shape']
  if offset<0 or size!=int(np.prod(shape))*dtype.itemsize or offset+size>len(raw):raise ValueError('invalid trace array bounds')
  arrays.append((e['name'],np.frombuffer(raw,dtype=dtype,count=size//dtype.itemsize,offset=offset).reshape(shape)))
 return m,arrays

def last(arrays,name):return next(v for n,v in reversed(arrays) if n==name)
