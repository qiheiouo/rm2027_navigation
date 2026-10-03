"""Version/geometry/epoch contract tests; synthetic v2 is not runtime evidence."""
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from dynamic_consumption_io import read_consumption, V2_SCHEMA
from prepare_dynamic_consumption_replay import prepare

FIXTURE = Path(__file__).resolve().parents[1]/'test/fixtures/dynamic_consumption/MovingInputWithCaptureAndBudgetExhaustionIsBitExact'
CPP_V2 = Path(__file__).resolve().parents[1]/'test/fixtures/observation_consumption'


def v2(meta):
    meta['schema'] = 2; meta['kind'] = 'dynamic_consumption_fields_v2'
    meta['input_used']['schema'] = V2_SCHEMA
    meta['limits'].update(input_schema=V2_SCHEMA, obstacle_radius_mode='visible_extent',
                          nominal_obstacle_diameter=0., geometry_prior_source='')
    return meta


class ObservationConsumptionIO(unittest.TestCase):
    def test_actual_cpp_v2_visible_fixture_has_recorded_selected_geometry(self):
        meta, blocks = read_consumption(CPP_V2/'visible/score_0.json')
        self.assertEqual(meta['kind'], 'dynamic_consumption_fields_v2')
        self.assertEqual(meta['limits']['obstacle_radius_mode'], 'visible_extent')
        self.assertLess(blocks['dynamic_risk_double']['values'][1], 1.)

    def test_actual_cpp_v2_nominal_fixture_changes_wait_cost_and_preserves_float_relation(self):
        from dynamic_consumption_io import cost_relation_exact
        meta, blocks = read_consumption(CPP_V2/'nominal/score_0.json')
        self.assertEqual(meta['limits']['nominal_obstacle_diameter'], .8)
        self.assertEqual(meta['limits']['geometry_prior_source'], 'synthetic:CPP-fixture-only')
        self.assertGreater(blocks['dynamic_risk_double']['values'][1], 100000.)
        self.assertTrue(cost_relation_exact(blocks))
    def exercise(self, mutation, reject=True):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); shutil.copytree(FIXTURE, root, dirs_exist_ok=True)
            path = root/'score_0.json'; meta = v2(json.loads(path.read_text()))
            mutation(meta); path.write_text(json.dumps(meta))
            if reject:
                with self.assertRaises(ValueError): read_consumption(path)
            else:
                return read_consumption(path)[0]

    def test_explicit_visible_v2_is_decoded_without_reinterpreting_v1(self):
        meta = self.exercise(lambda m: None, False)
        self.assertEqual(meta['limits']['nominal_obstacle_diameter'], 0.)
        self.assertEqual(read_consumption(FIXTURE/'score_0.json')[0]['schema'], 1)

    def test_nominal_model_metadata_is_explicit_but_not_a_physical_certificate(self):
        def change(meta):
            meta['limits'].update(obstacle_radius_mode='nominal_diameter',
                                 nominal_obstacle_diameter=1.6970562748477142,
                                 geometry_prior_source='synthetic:metadata-only/not-runtime-geometry')
        meta = self.exercise(change, False)
        self.assertEqual(meta['schema'], 2)
        # Unchanged risk payload is intentionally not a claimed nominal CPP fixture.

    def test_versions_and_schema_selection_cannot_be_disguised(self):
        for change in [lambda m: m.update(schema=True), lambda m: m.update(schema=1),
                       lambda m: m.update(kind='dynamic_consumption_fields_v1'),
                       lambda m: m['limits'].update(input_schema='rm_dynamic_obstacle_predictions/v1'),
                       lambda m: m['input_used'].update(schema='rm_dynamic_obstacle_predictions/v1')]:
            self.exercise(change)

    def test_unknown_and_missing_fields_are_rejected_at_each_v2_boundary(self):
        for slot in ('limits', 'cost_parameters', 'input_used'):
            self.exercise(lambda m, slot=slot: m[slot].update(unknown=1))
        self.exercise(lambda m: m.update(unknown=1))
        self.exercise(lambda m: m['input_used']['tracks'][0].update(covariance=1))
        self.exercise(lambda m: m['blocks'][0].update(unrecorded=1))
        self.exercise(lambda m: m['limits'].pop('geometry_prior_source'))

    def test_unused_nominal_values_and_unknown_modes_are_refused(self):
        for change in [lambda m: m['limits'].update(obstacle_radius_mode='complete'),
                       lambda m: m['limits'].update(nominal_obstacle_diameter=.8),
                       lambda m: m['limits'].update(geometry_prior_source='stray')]:
            self.exercise(change)

    def test_nominal_range_and_provenance_are_required(self):
        for value in (0., -.1, 3.1, float('inf'), float('nan'), True):
            self.exercise(lambda m, value=value: m['limits'].update(obstacle_radius_mode='nominal_diameter',
                nominal_obstacle_diameter=value, geometry_prior_source='synthetic'))
        for prior in ('', ' \t\n', '\f\v', 'x'*4097):
            self.exercise(lambda m, prior=prior: m['limits'].update(obstacle_radius_mode='nominal_diameter',
                nominal_obstacle_diameter=.8, geometry_prior_source=prior))

    def test_observed_and_coasting_epoch_rules_are_exact(self):
        self.exercise(lambda m: m['input_used']['tracks'][0].update(observed_sec=99, observed_nanosec=900000000))
        self.exercise(lambda m: m['input_used']['tracks'][0].update(state=3))
        def coast(meta):
            meta['input_used']['tracks'][0].update(state=3, observed_sec=99, observed_nanosec=900000000)
        self.exercise(coast, False)

    def test_future_one_nanosecond_cannot_hide_in_float_wall_epoch(self):
        def future(meta):
            meta['score_stamp'] = 1700000000.
            meta['input_used'].update(stamp_sec=1700000000, stamp_nanosec=0)
            meta['input_used']['tracks'][0].update(state=3, observed_sec=1700000000, observed_nanosec=1)
        self.exercise(future)

    def test_mixed_trace_refuses_to_create_a_reinterpreted_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); source = root/'source'
            shutil.copytree(FIXTURE, source)
            meta = v2(json.loads((source/'score_0.json').read_text())); meta['ordinal'] = 1
            (source/'score_1.json').write_text(json.dumps(meta))
            shutil.copyfile(source/'score_0.bin', source/'score_1.bin')
            with self.assertRaises(ValueError): prepare(source, root/'prepared')
            self.assertFalse((root/'prepared').exists())


if __name__ == '__main__': unittest.main()
