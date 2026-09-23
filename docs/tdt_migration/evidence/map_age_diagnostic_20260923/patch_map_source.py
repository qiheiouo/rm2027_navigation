#!/usr/bin/env python3
"""Extract pinned Nav2 costmap and add passive in-lock trace only."""
from pathlib import Path
import difflib,hashlib,json,tarfile
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
WORK=ROOT/'build/tdt_p2b';ARCHIVE=WORK/'mppi_cycle_diagnostic_v1/navigation2-1.1.20.tar.gz'
SOURCE=WORK/'map_age_diagnostic_v1/source';PKG=SOURCE/'nav2_costmap_2d'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(ARCHIVE)=='c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6'
SOURCE.mkdir(parents=True,exist_ok=False)
with tarfile.open(ARCHIVE) as tar:
 for member in tar.getmembers():
  parts=Path(member.name).parts
  if len(parts)<2 or parts[1]!='nav2_costmap_2d':continue
  assert not member.issym() and not member.islnk()
  relative=Path(*parts[1:]);dest=SOURCE/relative
  if member.isdir():dest.mkdir(parents=True,exist_ok=True)
  elif member.isfile():
   dest.parent.mkdir(parents=True,exist_ok=True)
   with tar.extractfile(member) as src, dest.open('wb') as out:out.write(src.read())
   dest.chmod(member.mode)
assert PKG.is_dir()
before={str(p.relative_to(PKG)):p.read_text() for p in PKG.rglob('*') if p.is_file() and p.suffix in ('.cpp','.hpp')}
def insert(rel,old,new):
 p=PKG/rel;s=p.read_text();assert s.count(old)==1,(rel,old,s.count(old));p.write_text(s.replace(old,new))
for rel in ('src/layered_costmap.cpp','plugins/obstacle_layer.cpp'):
 p=PKG/rel;p.write_text('#include "nav2_costmap_2d/tdt_map_trace.hpp"\n'+p.read_text())
insert('src/layered_costmap.cpp','  initialized_ = true;\n}', '''  initialized_ = true;
  const auto update_complete_ns = tdt_map_trace::enabled() ? tdt_map_trace::steady_ns() : 0;
  if (tdt_map_trace::enabled()) {
    const auto width = combined_costmap_.getSizeInCellsX();
    const auto height = combined_costmap_.getSizeInCellsY();
    const auto hash = tdt_map_trace::fnv64(
      combined_costmap_.getCharMap(),
      static_cast<size_t>(width) * height);
    tdt_map_trace::emit(
      "{\\"kind\\":\\"master_complete\\",\\"steady_ns\\":%llu,\\"pid\\":%ld,\\"frame\\":\\"%s\\",\\"width\\":%u,\\"height\\":%u,\\"resolution\\":%.17g,\\"origin\\":[%.17g,%.17g],\\"fnv64\\":\\"%016llx\\"}",
      static_cast<unsigned long long>(update_complete_ns),
      static_cast<long>(::getpid()), global_frame_.c_str(), width, height,
      combined_costmap_.getResolution(), combined_costmap_.getOriginX(),
      combined_costmap_.getOriginY(), static_cast<unsigned long long>(hash));
  }
}''')
insert('plugins/obstacle_layer.cpp','  // update the global current status\n  current_ = current;','''  if (tdt_map_trace::enabled()) {
    for (const auto & obs : observations) {
      const auto & stamp = obs.cloud_->header.stamp;
      const auto stamp_ns = static_cast<int64_t>(stamp.sec) * INT64_C(1000000000) + stamp.nanosec;
      tdt_map_trace::emit(
        "{\\"kind\\":\\"marking_selected\\",\\"steady_ns\\":%llu,\\"pid\\":%ld,\\"layer\\":\\"%s\\",\\"stamp_ns\\":%lld,\\"cloud_bytes\\":%zu}",
        static_cast<unsigned long long>(tdt_map_trace::steady_ns()),
        static_cast<long>(::getpid()), name_.c_str(), static_cast<long long>(stamp_ns),
        obs.cloud_->data.size());
    }
  }
  // update the global current status
  current_ = current;''')
(PKG/'include/nav2_costmap_2d/tdt_map_trace.hpp').write_bytes((HERE/'map_trace.hpp').read_bytes())
patch=[]
for name,old in before.items():
 new=(PKG/name).read_text()
 if new!=old:patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
(HERE/'observer.patch').write_text(''.join(patch))
inputs={'schema':'tdt_costmap_update_trace_build/v1','upstream_archive_sha256':sha(ARCHIVE),
        'unmodified_source_sha256':{name:hashlib.sha256(s.encode()).hexdigest() for name,s in before.items()},
        'patched_source_sha256':{str(p.relative_to(PKG)):sha(p) for p in PKG.rglob('*') if p.is_file() and p.suffix in ('.cpp','.hpp')},
        'trace_header_sha256':sha(HERE/'map_trace.hpp'),'patch_sha256':sha(HERE/'observer.patch')}
with (HERE/'build_inputs.json').open('x') as f:json.dump(inputs,f,indent=2);f.write('\n')
print('pinned archive extracted; modified two source files and added one diagnostic header')
