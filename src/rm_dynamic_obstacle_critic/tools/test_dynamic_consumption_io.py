"""Decode real C++ plugin fixtures and reject malformed private evidence."""
import gzip
import json
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
from dynamic_consumption_io import read_consumption, cost_relation_exact

FIXTURE = Path(__file__).resolve().parents[1]/'test/fixtures/dynamic_consumption'
MOVING = 'MovingInputWithCaptureAndBudgetExhaustionIsBitExact'
STATIC = 'StaticInputHasIdenticalCostsWithAndWithoutRecording'
EMPTY = 'EmptyInputIsRecordedAndStaleReplacementStillFailsClosed'


class DynamicConsumptionIOTest(unittest.TestCase):
    def test_actual_moving_fixture_has_consumed_fields_and_exact_float_relation(self):
        meta, blocks = read_consumption(FIXTURE/MOVING/'score_0.json')
        self.assertEqual(meta['score_stamp'], 100)
        self.assertEqual(meta['source_age_used'], 0)
        self.assertEqual(meta['input_used']['tracks'][0]['vxy'], [0, 1])
        self.assertEqual(blocks['x']['shape'], [2, 30])
        self.assertEqual(list(blocks['costs_before_dynamic']['values']), [10, 20])
        self.assertGreater(blocks['dynamic_risk_double']['values'][0], 1000)
        self.assertTrue(cost_relation_exact(blocks))
        self.assertNotIn('prediction', meta['input_used']['tracks'][0])

    def test_actual_static_fixture_uses_zero_velocity(self):
        meta, blocks = read_consumption(FIXTURE/STATIC/'score_0.json')
        self.assertEqual(meta['input_used']['tracks'][0]['vxy'], [0, 0])
        self.assertTrue(cost_relation_exact(blocks))

    def test_actual_empty_input_has_zero_risk_and_no_stale_record(self):
        meta, blocks = read_consumption(FIXTURE/EMPTY/'score_1.json')
        self.assertEqual(meta['score_stamp'], 101)
        self.assertEqual(meta['input_used']['tracks'], [])
        self.assertEqual(list(blocks['dynamic_risk_double']['values']), [0, 0])
        self.assertEqual(blocks['costs_after_dynamic']['bytes'], blocks['costs_before_dynamic']['bytes'])
        self.assertEqual(len(list((FIXTURE/EMPTY).glob('score_*.json'))), 2)

    def corrupt(self, mutate):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); shutil.copytree(FIXTURE/MOVING, root, dirs_exist_ok=True)
            meta = json.loads((root/'score_0.json').read_text()); blob = bytearray((root/'score_0.bin').read_bytes())
            mutate(meta, blob)
            (root/'score_0.json').write_text(json.dumps(meta)); (root/'score_0.bin').write_bytes(blob)
            with self.assertRaises(ValueError): read_consumption(root/'score_0.json')

    def test_truncation(self): self.corrupt(lambda m, b: b.pop())
    def test_trailing_payload(self): self.corrupt(lambda m, b: b.append(0))
    def test_overlap(self): self.corrupt(lambda m, b: m['blocks'][1].update(offset=0))
    def test_duplicate_block(self): self.corrupt(lambda m, b: m['blocks'][1].update(name='x'))
    def test_wrong_double_dtype(self): self.corrupt(lambda m, b: m['blocks'][4].update(dtype='<f4'))
    def test_wrong_shape(self): self.corrupt(lambda m, b: m['blocks'][0].update(shape=[2, 29]))
    def test_nonfinite_tensor(self): self.corrupt(lambda m, b: b.__setitem__(slice(0, 4), struct.pack('<f', float('nan'))))
    def test_nonfinite_risk(self): self.corrupt(lambda m, b: b.__setitem__(slice(728, 736), struct.pack('<d', float('inf'))))
    def test_false_source_age(self): self.corrupt(lambda m, b: m.update(source_age_used=.001))
    def test_stale_used_message(self): self.corrupt(lambda m, b: m['input_used'].update(stamp_sec=98))
    def test_false_track_count(self): self.corrupt(lambda m, b: m['input_used'].update(total_track_count=2))
    def test_grid_mismatch(self): self.corrupt(lambda m, b: m['input_used'].update(prediction_steps=29))
    def test_negative_extent(self): self.corrupt(lambda m, b: m['input_used']['tracks'][0].update(size_xy=[-.1, .2]))
    def test_boolean_ordinal(self): self.corrupt(lambda m, b: m.update(ordinal=True))
    def test_one_ulp_after_cost_is_rejected_by_exact_relation(self):
        meta, blocks = read_consumption(FIXTURE/MOVING/'score_0.json')
        raw = bytearray(blocks['costs_after_dynamic']['bytes'])
        raw[:4] = struct.pack('<I', struct.unpack('<I', raw[:4])[0]+1)
        blocks['costs_after_dynamic']['bytes'] = bytes(raw)
        self.assertFalse(cost_relation_exact(blocks))

    def test_duplicate_json_key(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); shutil.copytree(FIXTURE/MOVING, root, dirs_exist_ok=True)
            p = root/'score_0.json'; p.write_text(p.read_text().replace('"schema":1', '"schema":1,"schema":1', 1))
            with self.assertRaises(ValueError): read_consumption(p)

    def test_compressed_payload_has_identical_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); shutil.copytree(FIXTURE/MOVING, root, dirs_exist_ok=True)
            p = root/'score_0.bin'; raw = p.read_bytes()
            with gzip.open(str(p)+'.gz', 'wb') as output: output.write(raw)
            p.unlink(); meta, blocks = read_consumption(root/'score_0.json')
            self.assertTrue(cost_relation_exact(blocks))


if __name__ == '__main__': unittest.main()
