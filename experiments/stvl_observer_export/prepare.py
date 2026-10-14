"""Generate a narrow observer overlay from an immutable research commit."""
import gzip, hashlib, json, pathlib, shutil, struct, subprocess

ROOT = pathlib.Path(__file__).resolve().parents[2]
BASE = pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
REV = 'a880c7dc01cb424ea865e1fa0f33732396078670'
CACHE = ROOT / 'build/observer_export_cache'
OUT = ROOT / 'experiments/stvl_observer_export/evidence'

def source(path):
    return subprocess.check_output(['git', 'show', REV + ':' + path], cwd=BASE, text=True)

def replace_once(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after)

def header():
    original = source('experiments/stvl_c83_endtoend/shadow.hpp')
    text = replace_once(original, '#include "../stvl_execution_realtime/optimized.hpp"',
                        '#include "' + str(BASE / 'experiments/stvl_execution_realtime/optimized.hpp') + '"')
    text = replace_once(text, 'bool enabled=false;', 'bool enabled=false,candidate_export_enabled=true;')
    text = replace_once(text, 'std::string map,log;node->get_parameter',
        'if(!node->has_parameter(name+".candidate_export_enabled"))node->declare_parameter(name+".candidate_export_enabled",true);node->get_parameter(name+".candidate_export_enabled",s.candidate_export_enabled);\n std::string map,log;node->get_parameter')
    text = replace_once(text, 's.snapshots.open(log+".raw",std::ios::binary);',
                        'if(s.candidate_export_enabled)s.snapshots.open(log+".raw",std::ios::binary);')
    text = replace_once(text, 's.attempts++;s.raw.clear();s.raw.reserve',
                        's.attempts++;s.raw.clear();if(!s.enabled&&!s.candidate_export_enabled)return;s.raw.reserve')
    export = 'uint64_t count=s.raw.size(),cycle=s.cycle;s.snapshots.write(reinterpret_cast<char*>(&cycle),8);s.snapshots.write(reinterpret_cast<char*>(&count),8);for(auto&t:s.raw)for(size_t k=0;k<30;k++)for(auto v:{t.poses[k],t.velocities[k]})s.snapshots.write(reinterpret_cast<char*>(v.data()),24);\n s.snapshots.flush();'
    text = replace_once(text, 'inline void finish(',
                        'inline void export_candidates(){if(!s.candidate_export_enabled)return;' + export + '}\ninline void finish(')
    # Move the exact old binary writer without changing the enabled format.
    text = replace_once(text, 'auto export_begin=Clock::now();' + export,
                        'auto export_begin=Clock::now();export_candidates();')
    text = replace_once(text, 'record["shadow_enabled"]=s.enabled;',
                        'record["shadow_enabled"]=s.enabled;record["candidate_export_enabled"]=s.candidate_export_enabled;record["raw_candidates_retained"]=s.raw.size();')
    text = replace_once(text, 's.context["shadow_enabled"]=s.enabled;',
                        's.context["shadow_enabled"]=s.enabled;s.context["candidate_export_enabled"]=s.candidate_export_enabled;')
    return original, text

def main():
    CACHE.mkdir(exist_ok=True); OUT.mkdir(exist_ok=True)
    assert shutil.disk_usage(CACHE).free > 3.03 * 1024**3
    original, modified = header()
    (CACHE / 'shadow_export.hpp').write_text(modified)
    p = ROOT / 'build/active_scope_cache/full/runs/narrow_C_83_aligned'
    records = {x['cycle_id']: x for x in map(json.loads, gzip.open(p / 'control.jsonl.gz', 'rt'))}
    wanted = (0, 24); raw = {}
    # Two original cycle slices, lossless doubles, from the existing unique stream.
    with gzip.open(p / 'paired_timing.jsonl.raw.gz', 'rb') as f:
        while len(raw) < len(wanted):
            pre = f.read(16)
            assert len(pre) == 16
            cycle, count = struct.unpack('=QQ', pre)
            data = f.read(count * 30 * 6 * 8)
            assert len(data) == count * 30 * 6 * 8
            if cycle in wanted:
                assert count == 300
                raw[cycle] = data
                (OUT / f'cycle_{cycle}.bin.gz').write_bytes(gzip.compress(data, mtime=0))
                (CACHE / f'cycle_{cycle}.bin').write_bytes(data)
    cases = []
    for cycle in wanted:
        rec = records[cycle]
        cases.append(dict(cycle=cycle, costs=rec['native_final_costs_after_gamma'],
            original_command=rec['command'], original_safe_candidates=rec['prediction']['safe_candidates'],
            raw_sha256=hashlib.sha256(raw[cycle]).hexdigest(), raw_bytes=len(raw[cycle]),
            input_shape=[300,30,6], provenance=str(p), fixture_control_inputs='cv=propagated velocity; not original noised controls'))
    (OUT / 'inputs.json').write_text(json.dumps(cases, indent=2)+'\n')
    (OUT / 'preparation.json').write_text(json.dumps(dict(frozen_commit=REV,
        original_shadow_sha256=hashlib.sha256(original.encode()).hexdigest(),
        generated_shadow_sha256=hashlib.sha256(modified.encode()).hexdigest(),
        base_algorithm='installed Nav2 1.1.20 weighted update',
        edits=['include-only adaptation', 'explicit export parameter default true',
               'skip unused raw read only when shadow and export both disabled',
               'guard/move unchanged binary writer', 'explicit availability metadata']), indent=2)+'\n')
    print('two frozen slices; owned generated observer, original untouched')

if __name__ == '__main__': main()
