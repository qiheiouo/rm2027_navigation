"""Exact immutable input identities for experimental read-only replay."""
import numpy as np


def lateral_state(lateral):
    return dict(mode=lateral.mode,track=lateral.track,side=lateral.side,
                normal=None if lateral.normal is None else lateral.normal.tolist())


def input_identity(epoch,snapshot,initial,window,generation,map_revision,validated):
    finite=initial is not None and np.shape(initial)==(6,) and np.isfinite(initial).all()
    return dict(schema='temporal_mpc_solver_diagnostic/v2_inputs',generation=int(generation),
                map_revision=map_revision,input_validated=validated,
                input_source_ns=None if snapshot is None else snapshot.source_ns,
                source_age_s=None if snapshot is None else (epoch-snapshot.source_ns)*1e-9,
                track_observation_ns=[] if snapshot is None else [[t.track_id,t.observation_ns] for t in snapshot.tracks],
                initial_state=None if not finite else np.asarray(initial,float).tolist(),
                initial_state_invalid=initial is not None and not finite,
                centre_bounds=None if window is None else list(window.centre_bounds))
