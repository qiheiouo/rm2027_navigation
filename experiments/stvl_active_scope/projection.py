"""Existing adapter scope only; prediction/tracking and all active fields unchanged."""
import copy,time,json,hashlib
def project(payload,kind,log):
 start=time.monotonic_ns();assert kind in ('full','active')
 source=payload['tracks'];retained=[t for t in source if kind=='full' or t['state']!=1];ignored=[t for t in source if kind=='active' and t['state']==1]
 result=dict(payload,consumption_scope='all_tracks' if kind=='full' else 'confirmed_coasting',source_complete=payload['complete'],source_track_count=len(source),ignored_tentative_tracks=ignored,active_empty=not any(t['state'] in (2,3) for t in retained),tracks=retained)
 # Duplicate/unknown active information cannot be made valid by projection.
 if len({t['id'] for t in source})!=len(source):result['complete']=False
 assert sorted(retained+ignored,key=lambda t:t['id'])==sorted(source,key=lambda t:t['id'])
 cost=(time.monotonic_ns()-start)/1e6
 log.write(json.dumps(dict(source_ns=payload['source_ns'],publish_ns=payload['publish_ns'],kind=kind,source_count=len(source),retained_ids=[t['id'] for t in retained],ignored_ids=[t['id'] for t in ignored],active_empty=result['active_empty'],projection_ms=cost,active_fields_unchanged=True,source_reconstructable=True))+'\n')
 return result
