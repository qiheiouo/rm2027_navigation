"""Evidence-join guard tests: a repeated older hash must never mask a newer map."""
import unittest
import numpy as np
from analyze_map_age import fnv64, latest_update, pair_updates

class JoinTest(unittest.TestCase):
    def test_known_fnv(self):
        self.assertEqual(fnv64(b''), 'cbf29ce484222325')
        self.assertEqual(fnv64(b'hello'), 'a430d84680aabd0b')

    def test_selected_stamp_belongs_to_completed_update(self):
        events = [
            {'kind':'marking_selected','steady_ns':10,'stamp_ns':1},
            {'kind':'master_complete','steady_ns':11,'fnv64':'a'},
            {'kind':'marking_selected','steady_ns':20,'stamp_ns':2},
            {'kind':'master_complete','steady_ns':21,'fnv64':'b'}]
        rows=pair_updates(events)
        self.assertEqual([r['marking_stamps_ns'] for r in rows], [[1],[2]])

    def test_latest_map_required_even_if_older_hash_repeats(self):
        raw=np.array([[1,2]],dtype=np.uint8)
        meta={'frame':'odom','width':2,'height':1,'resolution':.05,'origin':[0.,0.]}
        base={**meta,'kind':'master_complete'}
        older={**base,'steady_ns':10,'fnv64':fnv64(raw.tobytes())}
        newer={**base,'steady_ns':20,'fnv64':fnv64(b'other')}
        with self.assertRaisesRegex(ValueError,'latest completed map'):
            latest_update([older,newer],[10,20],21,raw,meta)
        self.assertEqual(latest_update([older,newer],[10,20],15,raw,meta),older)

if __name__=='__main__':unittest.main()
