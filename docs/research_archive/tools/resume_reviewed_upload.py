#!/usr/bin/env python3
"""Human-authenticated upload of pinned, reviewed refs. Default: local preview."""
import argparse
import datetime
import json
import os
import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path('/home/qihei/rm2027_navigation')
RELATIVE = pathlib.Path('docs/research_archive/tools/resume_reviewed_upload.py')
PLAN_PATH = 'docs/research_archive/resume_upload_plan.json'


def git_read(*args, data=None):
    return subprocess.check_output(['git', *args], cwd=REPO, input=data, text=True).strip()


def validate_pin(commit):
    if len(commit) != 40 or any(c not in '0123456789abcdef' for c in commit):
        raise RuntimeError('Expected a complete, reviewed archive commit SHA')
    expected_script = git_read('show', commit + ':' + RELATIVE.as_posix())
    if pathlib.Path(__file__).read_text().strip() != expected_script:
        raise RuntimeError('Script differs from the reviewed commit; stop')
    plan = json.loads(git_read('show', commit + ':' + PLAN_PATH))
    if git_read('remote', 'get-url', 'origin') != plan['origin'] or git_read('remote', 'get-url', '--push', 'origin') != plan['origin']:
        raise RuntimeError('Origin differs from the reviewed repository; stop')
    if git_read('rev-parse', plan['archive_ref']) != commit:
        raise RuntimeError('Local archive tip changed since review; stop')
    subprocess.run(['git', 'merge-base', '--is-ancestor', plan['remote_archive_base'], commit], cwd=REPO, check=True)
    changed = git_read('diff', '--name-only', plan['remote_archive_base'], commit).splitlines()
    if any(not p.startswith('docs/research_archive/') for p in changed):
        raise RuntimeError('Archive changes outside the reviewed administrative scope; stop')
    return plan


def local_footprint(jobs, remote):
    known = sorted(set(oid for ref, oid in remote.items() if not ref.endswith('^{}')))
    for oid in known:
        subprocess.run(['git', 'cat-file', '-e', oid], cwd=REPO, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    data = ''.join(j['oid'] + '\n' for j in jobs) + ''.join('^' + oid + '\n' for oid in known)
    objects = git_read('rev-list', '--objects', '--stdin', data=data).splitlines()
    if not objects:
        return {'objects': 0, 'bytes': 0, 'largest_blob_bytes': 0}
    sizes = git_read('cat-file', '--batch-check=%(objectname) %(objecttype) %(objectsize)',
                     data=''.join(line.split()[0] + '\n' for line in objects)).splitlines()
    rows = [line.split() for line in sizes]
    return {'objects': len(rows), 'bytes': sum(int(s[2]) for s in rows),
            'largest_blob_bytes': max((int(s[2]) for s in rows if s[1] == 'blob'), default=0)}


def confirm_number(value, prompt):
    value = float(input(prompt)) if value is None else value
    if not (value >= 0 and value < 1000000):
        raise RuntimeError('Invalid quota value')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-commit', required=True)
    parser.add_argument('--scope', choices=['index', 'bounded', 'full'], default='bounded')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--free-mb', type=float, help='Confirmed current available repository capacity in MB')
    parser.add_argument('--single-file-mb', type=float, help='Confirmed current single-file limit in MB; required for full scope')
    args = parser.parse_args()
    plan = validate_pin(args.archive_commit)
    jobs = [{'ref': plan['archive_ref'], 'oid': args.archive_commit}]
    if args.scope != 'index':
        jobs += plan['bounded_jobs']
    if args.scope == 'full':
        jobs += plan['full_extra_jobs']
    assert len(jobs) == {'index': 1, 'bounded': 10, 'full': 18}[args.scope]
    for job in jobs:
        if job['ref'] == 'refs/heads/main' or git_read('rev-parse', job['ref']) != job['oid']:
            raise RuntimeError('Reviewed local ref changed: ' + job['ref'])
    print('范围:', args.scope, '；审核 refs:', len(jobs), '；main 不在推送清单。')
    for job in jobs:
        print(job['oid'], job['ref'])
    if args.scope == 'index':
        print('仅归档管理文本更新；新增对象总量必须小于2MB。服务端若拒绝即停止。')
    else:
        print('需先确认剩余空间至少 %d MB。仅 full 需单文件额度至少 200 MB。' % plan['minimum_confirmed_free_MB'][args.scope])
    if not args.execute:
        print('仅预览；没有访问远端或执行 push。')
        return
    if not sys.stdin.isatty():
        raise RuntimeError('Run in your own interactive terminal for quota confirmation and authentication')
    free_mb = None
    if args.scope != 'index':
        free_mb = confirm_number(args.free_mb, 'Gitee 管理页面当前剩余仓库空间（MB；GB保守按1000换算）: ')
        if free_mb < plan['minimum_confirmed_free_MB'][args.scope]:
            raise RuntimeError('Available capacity below the reviewed safety margin; no push attempted')
    single_mb = 100.0
    if args.scope == 'full':
        single_mb = confirm_number(args.single_file_mb, 'Gitee 已生效的单文件额度（MB）: ')
        if single_mb < plan['full_minimum_single_file_limit_MB']:
            raise RuntimeError('Single-file quota still rejects the original history; no push attempted')
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    log = REPO / 'build/research_archive_audit_20261006' / ('resume_results_' + stamp + '.json')
    log.parent.mkdir(parents=True, exist_ok=True)
    report = {'archive_commit': args.archive_commit, 'scope': args.scope,
              'user_confirmed_free_MB': free_mb, 'user_confirmed_single_file_MB': single_mb,
              'results': [], 'main_excluded': True, 'automatic_retries': 0}

    def save():
        log.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    env = os.environ.copy()
    env.pop('GIT_ASKPASS', None)
    env.pop('SSH_ASKPASS', None)
    env['GIT_TERMINAL_PROMPT'] = '1'
    with tempfile.TemporaryDirectory(prefix='rm2027-resume-auth-') as authdir:
        socket = str(pathlib.Path(authdir) / 'socket')
        git = ['git', '-c', 'credential.helper=', '-c',
               'credential.helper=cache --timeout=3600 --socket=' + socket,
               '-c', 'core.askPass=', '-c', 'push.followTags=false']

        def refs():
            value = subprocess.check_output(git + ['ls-remote', 'origin', 'refs/heads/*', 'refs/tags/*'],
                                            cwd=REPO, env=env, text=True)
            return {line.split()[1]: line.split()[0] for line in value.splitlines()}

        try:
            before = refs()
            report['remote_before'] = before
            save()
            if before.get('refs/heads/main') != plan['remote_main']:
                raise RuntimeError('Remote main changed since audit; stop without pushing')
            footprint = local_footprint(jobs, before)
            report['missing_object_footprint'] = footprint
            save()
            if footprint['largest_blob_bytes'] > single_mb * 1000000:
                raise RuntimeError('A pending blob exceeds the confirmed single-file quota')
            if args.scope == 'index' and footprint['bytes'] > 2 * 1000000:
                raise RuntimeError('Index-only update exceeded the reviewed 2 MB ceiling')
            if args.scope == 'bounded' and footprint['bytes'] > 84 * 1000000:
                raise RuntimeError('Bounded object footprint exceeded the reviewed 84 MB ceiling')
            if args.scope == 'full' and footprint['bytes'] > 2160 * 1000000:
                raise RuntimeError('Full object footprint exceeded the reviewed 2160 MB ceiling')
            for job in jobs:
                previous = before.get(job['ref'])
                if previous and previous != job['oid']:
                    if job['ref'].startswith('refs/tags/'):
                        raise RuntimeError('Remote tag differs; no replacement: ' + job['ref'])
                    subprocess.run(['git', 'merge-base', '--is-ancestor', previous, job['oid']], cwd=REPO, check=True)
            for n, job in enumerate(jobs, 1):
                row = dict(job, state='already_remote_verified' if before.get(job['ref']) == job['oid'] else 'pushing')
                report['results'].append(row)
                save()
                print('\n[%d/%d] %s' % (n, len(jobs), job['ref']), flush=True)
                if row['state'] != 'already_remote_verified':
                    rc = subprocess.run(git + ['push', '--no-force', '--no-follow-tags', '--no-mirror',
                                              '--no-prune', '--recurse-submodules=no', 'origin',
                                              job['oid'] + ':' + job['ref']], cwd=REPO, env=env).returncode
                    row['return_code'] = rc
                    if rc:
                        row['state'] = 'push_failed'
                        save()
                        raise RuntimeError('Push rejected; stop all remaining refs without retry: ' + job['ref'])
                after = refs()
                row['remote_oid'] = after.get(job['ref'])
                if row['remote_oid'] != job['oid']:
                    row['state'] = 'verification_failed'
                elif row['state'] != 'already_remote_verified':
                    row['state'] = 'pushed_verified'
                save()
                if row['state'] == 'verification_failed':
                    raise RuntimeError('Remote SHA verification failed')
                if any(after.get(ref) != oid for ref, oid in before.items() if ref not in {j['ref'] for j in jobs}):
                    raise RuntimeError('A protected remote ref changed; stop and review')
            after = refs()
            report['remote_after'] = after
            report['all_selected_refs_verified'] = all(after.get(j['ref']) == j['oid'] for j in jobs)
            report['remote_main_unchanged'] = after.get('refs/heads/main') == before.get('refs/heads/main')
            save()
            if not report['all_selected_refs_verified'] or not report['remote_main_unchanged']:
                raise RuntimeError('Final remote verification failed')
            print('\n已上传并核实本轮全部所选 refs；main 保持。完整历史尚未上传的范围请按 scope 查看文档。')
        except BaseException as exc:
            report['stopped'] = True
            report['stop_reason'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc)
            save()
            raise
        finally:
            subprocess.run(['git', 'credential-cache', '--socket=' + socket, 'exit'], cwd=REPO, env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print('结果记录（不含凭据）:', log)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n用户停止；检查结果记录后再继续。', file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print('STOP:', str(exc), file=sys.stderr)
        sys.exit(1)
