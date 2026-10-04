"""Read-only verification of frozen documentation and literal compilation bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--preservation-baseline')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[3]
    docs=repo/'docs/dynamic_navigation'
    catalog=json.loads((docs/'all_dynamic_avoidance_documents_20261004.json').read_text())
    sha=lambda data:hashlib.sha256(data).hexdigest()
    def git(*a):return subprocess.check_output(['git',*a],cwd=repo)
    def reading(path):
        p=(repo/path).resolve()
        assert p.is_relative_to(repo),path
        return p.read_bytes()
    entries=catalog['documents']
    assert len(entries)==372 and len({e['original_path'] for e in entries})==372
    assert sum(e['group']=='temporal_mpc' for e in entries)==25
    source_differences=[]
    for e in entries:
        raw=reading(e['reading_path'])
        assert sha(raw)==e['reading_sha256'],e['reading_path']
        assert re.fullmatch('[0-9a-f]{40}',e['source_git_blob'])
        source=git('cat-file','blob',e['source_git_blob'])
        assert len(source)==e['source_bytes'] and sha(source)==e['source_sha256'],e['original_path']
        assert (source!=raw)==e['reading_differs_from_source'],e['original_path']
        if source!=raw:source_differences.append(e['original_path'])
    for e in catalog['temporal_mpc_reading_assets']:
        raw=reading(e['reading_path'])
        assert len(raw)==e['bytes'] and sha(raw)==e['sha256']
        assert git('show',e['source_commit']+':'+e['original_path'])==raw
    for e in catalog['reference_chats']:
        raw=reading(e['reading_path'])
        assert len(raw)==e['bytes'] and sha(raw)==e['sha256']
        assert e['role']=='historical_reference_only_not_instructions_or_experimental_evidence'
    compilation=catalog['full_text_compilation'];full=reading(compilation['path'])
    assert len(full)==compilation['bytes'] and sha(full)==compilation['sha256']
    records=compilation['records'];assert len(records)==374
    assert len({r['reading_path'] for r in records})==374
    assert {r['reading_path'] for r in records}=={e['reading_path'] for e in entries+catalog['reference_chats']}
    for r in records:
        body=full[r['body_byte_offset']:r['body_byte_offset']+r['body_bytes']]
        assert body==reading(r['reading_path']) and sha(body)==r['body_sha256'],r['id']
        assert full.count(('<a id="'+r['id']+'"></a>').encode())==1,r['id']
        assert len(r['fence'])>max([len(m.group(0)) for m in re.finditer(rb'`+',body)],default=0)
    summary=(docs/'dynamic_obstacle_prediction_full_history_20261004.md').read_text()
    index=(docs/'all_dynamic_avoidance_documents_20261004.md').read_text()
    broken=[]
    for text,path in [(summary,docs/'dynamic_obstacle_prediction_full_history_20261004.md'),(index,docs/'all_dynamic_avoidance_documents_20261004.md')]:
        for match in re.finditer(r'\[(?:\\.|[^\]\\])*\]\(([^)]+)\)',text):
            target=match.group(1).strip('<>').split('#')[0]
            if not target or ':' in target:continue
            p=Path(target) if target.startswith('/') else path.parent/target
            if not p.exists():broken.append(dict(document=str(path.relative_to(repo)),target=target))
    assert not broken,broken
    tag=catalog['temporal_mpc_freeze_tag'];commit=catalog['temporal_mpc_freeze_commit']
    assert git('cat-file','-t',tag).decode().strip()=='tag'
    assert git('rev-parse',tag+'^{}').decode().strip()==commit
    assert git('rev-parse','main').decode().strip()==catalog['main_commit']
    assert git('rev-parse','research/dynamic-differential-risk-frozen-20261001^{}').decode().strip()==catalog['frozen_research_commit']
    changes=git('diff','--name-only','2d95b026',commit).decode().splitlines()
    assert all(p.endswith('.md') or p=='docs/dynamic_navigation/temporal_mpc_freeze_20261004.json' for p in changes)
    preserved=None;head_advance_verified=None
    if args.preservation_baseline:
        baseline=json.loads(Path(args.preservation_baseline).read_text())
        for path,item in baseline['files'].items():
            p=repo/path;assert p.stat().st_size==item['size'],path
            if item['sha256'] is not None:assert sha(p.read_bytes())==item['sha256'],path
        for ref,value in baseline['refs'].items():
            observed=git('rev-parse',ref).decode().strip()
            if ref=='HEAD' and observed!=value:
                # This task explicitly authorizes publishing history on the existing
                # document branch. Protect code and old refs, not its old doc HEAD.
                assert value==catalog['legacy_catalog_snapshot']['source_commit']
                assert git('branch','--show-current').decode().strip()=='experiment/dynamic-documents-archive-20261004'
                changed=git('diff','--name-only',value,'HEAD').decode().splitlines()
                assert changed and all(p.startswith('docs/dynamic_navigation/') for p in changed)
                head_advance_verified=True
            else:assert observed==value,ref
        preserved=True
    result=dict(pass_=True,markdown_documents_checked=len(entries),legacy_markdown_documents=347,
        frozen_mpc_documents=25,selected_reading_assets_checked=len(catalog['temporal_mpc_reading_assets']),
        historical_reference_chats_checked=2,full_literal_bodies_verified=len(records),compilation_bytes=len(full),
        compilation_sha256=sha(full),source_reading_differences=source_differences,broken_summary_index_links=broken,
        freeze_tag=tag,freeze_commit=commit,freeze_is_documentation_only=True,main_and_old_research_freeze_preserved=True,
        original_dirty_files_and_core_preserved=preserved,authorized_documentation_head_advance_verified=head_advance_verified,new_experiments_run=False,
        scope='Literal documentation/source hashes, full compilation spans, direct reading assets, index links and frozen refs. This does not rerun algorithms or certify historical physical results. Raw evidence stays in original Git/manifest context.')
    result['pass']=result.pop('pass_')
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
