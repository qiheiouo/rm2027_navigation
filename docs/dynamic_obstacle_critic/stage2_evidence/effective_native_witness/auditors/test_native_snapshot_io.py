"""Independent real-C++ fixture decoding and rejection of corrupt evidence."""
import json
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
from native_snapshot_io import read_snapshot,measured_prefix_matches


FIXTURE=Path(__file__).resolve().parents[1]/'test/fixtures/native_snapshot'


class NativeSnapshotIOTest(unittest.TestCase):
    def test_actual_cpp_fixture_complete_grid_costs_and_raw_bytes(self):
        meta,blocks=read_snapshot(FIXTURE/'cycle_0.json')
        self.assertEqual(len(blocks),14);self.assertEqual(blocks['x']['shape'],[2,30])
        self.assertEqual(list(blocks['x']['values']),[float(100*i+k)/8 for i in range(2) for k in range(30)])
        self.assertEqual(list(blocks['y']['values']),[-float(100*i+k)/8 for i in range(2) for k in range(30)])
        self.assertEqual(list(blocks['critic_costs_before_control_regularization']['values']),[12.25,-3.5])
        self.assertEqual(blocks['raw_costmap']['values'],bytes(range(200,206)))
        self.assertTrue(measured_prefix_matches(meta,blocks))
        self.assertIn('SG_history',meta['unavailable'])

    def corrupt(self,mutate):
        with tempfile.TemporaryDirectory() as path:
            root=Path(path);shutil.copytree(FIXTURE,root,dirs_exist_ok=True)
            meta=json.loads((root/'cycle_0.json').read_text());blob=bytearray((root/'cycle_0.bin').read_bytes())
            mutate(meta,blob);(root/'cycle_0.json').write_text(json.dumps(meta));(root/'cycle_0.bin').write_bytes(blob)
            with self.assertRaises(ValueError):read_snapshot(root/'cycle_0.json')

    def test_truncated_payload(self):self.corrupt(lambda m,b:b.pop())
    def test_overlapping_blocks(self):self.corrupt(lambda m,b:m['blocks'][1].update(offset=0))
    def test_duplicate_block(self):self.corrupt(lambda m,b:m['blocks'][1].update(name='x'))
    def test_wrong_dtype(self):self.corrupt(lambda m,b:m['blocks'][0].update(dtype='>f4'))
    def test_shape_does_not_match_byte_count(self):self.corrupt(lambda m,b:m['blocks'][0].update(shape=[2,29]))
    def test_nan_tensor(self):self.corrupt(lambda m,b:b.__setitem__(slice(0,4),struct.pack('<f',float('nan'))))
    def test_map_budget_or_shape(self):self.corrupt(lambda m,b:m['map'].update(size=[999,999]))


if __name__=='__main__':unittest.main()
