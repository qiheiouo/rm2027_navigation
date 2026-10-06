"""Actual assignments and evidence refusal do not change tracker values."""
from dataclasses import asdict
import json
import math

import pytest
from rm_dynamic_obstacle_tracking.core import Detection, MultiObjectTracker, Point2D
from rm_dynamic_obstacle_tracking.surface_member_evidence import SurfaceMemberEvidence, encoded_range


def detection(x):
    return Detection(Point2D(x,0),.2,.2,3)


def test_duplicate_positions_have_distinct_created_ids_and_original_input_order():
    tracker=MultiObjectTracker(min_hits_to_confirm=1)
    first=tracker.update([detection(1),detection(1)],1,capture_assignments=True)
    assert first.detection_track_ids==(1,2)
    second=tracker.update([detection(1.5),detection(.8)],1.1,capture_assignments=True)
    assert second.detection_track_ids==(2,1)
    assert {t.track_id for t in second.tracks}=={1,2}


def test_coasting_reset_and_metadata_disabled_preserve_numeric_snapshots():
    observed=MultiObjectTracker(min_hits_to_confirm=1)
    ordinary=MultiObjectTracker(min_hits_to_confirm=1)
    for time,detections in [(1,[detection(1)]),(1.1,[]),(1.1,[detection(2)]),(2,[]),(2.1,[detection(3)])]:
        a=asdict(observed.update(detections,time,capture_assignments=True))
        b=asdict(ordinary.update(detections,time))
        assert len(a.pop('detection_track_ids'))==len(detections)
        assert b.pop('detection_track_ids')==()
        assert a==b


def test_new_directory_and_record_collision_are_not_overwritten(tmp_path):
    root=tmp_path/'evidence';writer=SurfaceMemberEvidence(str(root))
    with pytest.raises(FileExistsError):SurfaceMemberEvidence(str(root))
    occupied=root/'scan_000000.json';occupied.write_text('preserved')
    assert not writer.write({'status':'accepted'}) and not writer.complete
    assert occupied.read_text()=='preserved'
    assert json.loads((root/'incomplete.json').read_text())['complete'] is False


def test_record_budget_rejects_whole_stream_without_truncation(tmp_path):
    writer=SurfaceMemberEvidence(str(tmp_path/'evidence'));writer.MAX_RECORDS=1
    assert writer.write({'status':'accepted','values':[1,2]})
    assert not writer.write({'status':'accepted','values':[3,4]})
    assert len(list(writer.root.glob('scan_*.json')))==1 and not writer.complete
    assert not writer.write({'status':'accepted'})


@pytest.mark.parametrize('record',[{'nonfinite':math.nan},{'oversize':'x'*300}])
def test_nonfinite_or_byte_budget_never_produces_complete_partial_record(tmp_path,record):
    writer=SurfaceMemberEvidence(str(tmp_path/'evidence'));writer.MAX_RECORD_BYTES=200
    assert not writer.write(record) and not writer.complete
    assert not list(writer.root.glob('scan_*.json'))


def test_missing_returns_retain_kind_and_are_not_free_values():
    assert [encoded_range(v) for v in [math.nan,math.inf,-math.inf,1.]]==['NaN','Infinity','-Infinity',1.]


def test_relative_sink_is_rejected():
    with pytest.raises(ValueError,match='absolute'):SurfaceMemberEvidence('relative')


def test_close_records_complete_count_and_does_not_allow_later_writes(tmp_path):
    writer=SurfaceMemberEvidence(str(tmp_path/'evidence'))
    assert writer.write({'status':'map','data':[0]})
    assert writer.write({'status':'accepted'})
    writer.close()
    summary=json.loads((writer.root/'summary.json').read_text())
    assert summary['complete'] and summary['records_written']==2
    assert [t['ordinal'] for t in summary['write_timings']]==[0,1]
    writer.close()
    assert not writer.write({'status':'accepted'}) and not writer.complete
