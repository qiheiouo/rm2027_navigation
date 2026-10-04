import json
from audit_public_replay import audit


def test_capture_missing_authority_and_tf_is_not_relabelled(tmp_path):
    p=tmp_path/'real.jsonl'
    records=[dict(source_t=t,processing_t=t+.01,receive_ros_t=t+.02,frame='odom',
                  schema='rm_dynamic_obstacle_predictions/v1',complete=True,total_track_count=0,
                  prediction_dt=.1,prediction_steps=15,tracks=[]) for t in (1.,1.1)]
    p.write_text('\n'.join(map(json.dumps,records))+'\n')
    report=audit(p)
    assert report['missing_fields']['authority']==2
    assert not report['live_consumer_acceptance']
    assert report['frames']=={'odom':2}
    assert report['nonincreasing_sources']==0
    assert len(report['blockers'])==4
    assert 'authority' not in report['samples'][0]
