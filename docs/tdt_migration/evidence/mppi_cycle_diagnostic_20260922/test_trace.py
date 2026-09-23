import json,struct,tempfile,unittest,os
from pathlib import Path
import numpy as np
from read_trace import read_cycle,last
class TraceTests(unittest.TestCase):
 def fixture(self,directory,**updates):
  p=Path(directory)/'cycle_0.json';e={'kind':'array','name':'sample','offset':0,'bytes':8,'dtype':'f4','shape':[2]};e.update(updates)
  p.write_text(json.dumps({'events':[e]}));p.with_suffix('.bin').write_bytes(struct.pack('<ff',.25,.75));return p
 def test_float_array(self):
  with tempfile.TemporaryDirectory() as d:
   _,a=read_cycle(self.fixture(d));np.testing.assert_array_equal(last(a,'sample'),[.25,.75])
 def test_truncated_payload_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=self.fixture(d);p.with_suffix('.bin').write_bytes(b'\0')
   with self.assertRaises(ValueError):read_cycle(p)
 def test_shape_and_size_mismatch_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(ValueError):read_cycle(self.fixture(d,shape=[3]))
 def test_negative_offset_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(ValueError):read_cycle(self.fixture(d,offset=-1))
if __name__=='__main__':unittest.main()
