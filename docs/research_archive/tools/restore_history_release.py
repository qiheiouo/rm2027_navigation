#!/usr/bin/env python3
"""Verify Release bundle parts and restore to a separate bare mirror; no network."""
import argparse
import gzip
import hashlib
import json
import pathlib
import subprocess

ORIGINAL_GIT = pathlib.Path('/home/qihei/rm2027_navigation/.git').resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=pathlib.Path)
    parser.add_argument('parts_directory', type=pathlib.Path)
    parser.add_argument('--repo', type=pathlib.Path, help='Separate bare mirror, already containing prerequisite commits')
    args = parser.parse_args()
    m = json.loads(args.manifest.read_text())
    if m['format'] != 'rm2027-incremental-git-bundle-parts-v1':
        raise RuntimeError('Unknown manifest format')
    digest, total = hashlib.sha256(), 0
    for n, item in enumerate(m['parts'], 1):
        name = item['name']
        if pathlib.Path(name).name != name or item['number'] != n:
            raise RuntimeError('Unsafe part name/order')
        payload = (args.parts_directory / name).read_bytes()
        if len(payload) != item['bytes'] or hashlib.sha256(payload).hexdigest() != item['sha256']:
            raise RuntimeError('Part checksum mismatch: ' + name)
        raw = gzip.decompress(payload)
        if len(raw) != item['raw_bytes'] or hashlib.sha256(raw).hexdigest() != item['raw_sha256']:
            raise RuntimeError('Raw part checksum mismatch: ' + name)
        digest.update(raw)
        total += len(raw)
    if total != m['bundle_bytes'] or digest.hexdigest() != m['bundle_sha256']:
        raise RuntimeError('Complete bundle checksum mismatch')
    print('全部分片及完整bundle校验通过:', total, 'bytes')
    if args.repo is None:
        return
    repo = args.repo.resolve()

    def git(*command, **kwargs):
        return subprocess.check_output(['git', '-c', 'gc.auto=0', *command], cwd=repo, text=True, **kwargs).strip()

    common = pathlib.Path(git('rev-parse', '--git-common-dir'))
    if not common.is_absolute():
        common = repo / common
    if common.resolve() == ORIGINAL_GIT or git('rev-parse', '--is-bare-repository') != 'true':
        raise RuntimeError('Restore only to a separate bare mirror; original repository/worktrees forbidden')
    for oid in m['prerequisite_commits']:
        git('cat-file', '-e', oid + '^{commit}')
    git('rev-list', '--objects', '--missing=error', '--stdin', input='\n'.join(m['prerequisite_commits']) + '\n')
    current = {}
    for ref, oid in m['bundled_refs'].items():
        if ref == 'refs/heads/main' or not ref.startswith(('refs/heads/', 'refs/tags/')):
            raise RuntimeError('Bundle contains an unapproved restoration ref')
        result = subprocess.run(['git', 'rev-parse', '--verify', ref], cwd=repo, capture_output=True, text=True)
        current[ref] = result.stdout.strip() if result.returncode == 0 else '0' * 40
    process = subprocess.Popen(['git', '-c', 'gc.auto=0', 'bundle', 'unbundle', '-'], cwd=repo,
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for item in m['parts']:
            process.stdin.write(gzip.decompress((args.parts_directory / item['name']).read_bytes()))
        process.stdin.close()
        output = process.stdout.read().decode()
        error = process.stderr.read().decode()
        if process.wait() != 0:
            raise RuntimeError('Unbundle failed: ' + error[:300])
        actual = {line.split()[1]: line.split()[0] for line in output.splitlines()}
        if actual != m['bundled_refs']:
            raise RuntimeError('Restored bundle refs differ from the manifest')
        for ref, oid in actual.items():
            previous = current[ref]
            if previous != '0' * 40 and previous != oid:
                if ref.startswith('refs/tags/'):
                    raise RuntimeError('Existing destination tag differs; no overwrite')
                subprocess.run(['git', 'merge-base', '--is-ancestor', previous, oid], cwd=repo, check=True)
        updates = 'start\n' + ''.join('update %s %s %s\n' % (ref, oid, current[ref]) for ref, oid in actual.items())
        git('update-ref', '--stdin', input=updates + 'prepare\ncommit\n')
        for ref, oid in actual.items():
            if git('rev-parse', ref) != oid:
                raise RuntimeError('Restored ref verification failed')
        print('独立mirror恢复并核实全部归档refs；原工作区保持。')
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)


if __name__ == '__main__':
    main()
