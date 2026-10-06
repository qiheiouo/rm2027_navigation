#!/usr/bin/env python3
"""Stream immutable Git history to Gitee Release attachments; default is preview."""
import argparse
import datetime
import getpass
import gzip
import hashlib
import http.client
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import urllib.parse
import uuid

REPO = pathlib.Path('/home/qihei/rm2027_navigation')
ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = 'docs/research_archive/tools/upload_history_release.py'
ORIGIN = 'https://gitee.com/qiheiovo/rm2027_navigation.git'
API = '/api/v5/repos/qiheiovo/rm2027_navigation'
TAG = 'research-archive-history-20261006'
BASE_ARCHIVE = 'ac197e4e1d89e51e4799ac6d8fbafe34bc1ca2df'
CHUNK_BYTES = 48_000_000  # gzip overhead remains below a 50 MB single-file ceiling
AUDIT = REPO / 'build/research_archive_audit_20261006'


def local(*args, data=None):
    return subprocess.check_output(['git', '-c', 'gc.auto=0', *args], cwd=REPO, input=data, text=True).strip()


def pinned_jobs(commit):
    if len(commit) != 40 or any(c not in '0123456789abcdef' for c in commit):
        raise RuntimeError('Use the complete reviewed archive commit SHA')
    if pathlib.Path(__file__).read_text().strip() != local('show', commit + ':' + SCRIPT):
        raise RuntimeError('Upload tool differs from the reviewed commit')
    plan = json.loads(local('show', commit + ':docs/research_archive/resume_upload_plan.json'))
    if local('remote', 'get-url', 'origin') != ORIGIN or local('remote', 'get-url', '--push', 'origin') != ORIGIN:
        raise RuntimeError('Origin changed; stop')
    if local('rev-parse', plan['archive_ref']) != commit:
        raise RuntimeError('Local archive branch changed since review; stop')
    if any(not p.startswith('docs/research_archive/') for p in local('diff', '--name-only', BASE_ARCHIVE, commit).splitlines()):
        raise RuntimeError('Archive changed outside the reviewed administrative scope')
    jobs = [{'ref': plan['archive_ref'], 'oid': commit}] + plan['bounded_jobs'] + plan['full_extra_jobs']
    for j in jobs:
        if j['ref'] == 'refs/heads/main' or local('rev-parse', j['ref']) != j['oid']:
            raise RuntimeError('An audited local ref changed: ' + j['ref'])
    return jobs


def remote_refs():
    data = subprocess.check_output(['git', '-c', 'credential.helper=', '-c', 'core.askPass=',
                                    '-c', 'gc.auto=0', 'ls-remote', ORIGIN,
                                    'refs/heads/*', 'refs/tags/*'], cwd=REPO, text=True)
    return {line.split()[1]: line.split()[0] for line in data.splitlines()}


def bundle_header(data):
    stop = data.find(b'\n\n')
    if stop < 0 or not data.startswith((b'# v2 git bundle\n', b'# v3 git bundle\n')):
        raise RuntimeError('Git did not produce a valid bundle header')
    prerequisites, refs = [], {}
    for line in data[:stop].decode().splitlines()[1:]:
        if line.startswith('-'):
            prerequisites.append(line[1:41])
        elif line and not line.startswith('@'):
            oid, ref = line.split(' ', 1)
            refs[ref] = oid
    return {'prerequisite_commits': prerequisites, 'bundled_refs': refs}


class Gitee:
    def __init__(self, token):
        self.token = token

    def json_request(self, method, path, body=None, content_type='application/json', allow_404=False):
        if not path.startswith(API + '/'):
            raise RuntimeError('Unexpected API destination')
        headers = {'Authorization': 'Bearer ' + self.token, 'Accept': 'application/json',
                   'User-Agent': 'rm2027-research-archive'}
        if isinstance(body, dict):
            body = json.dumps(body, ensure_ascii=False).encode()
        if body is not None:
            headers['Content-Type'] = content_type
        connection = http.client.HTTPSConnection('gitee.com', timeout=600)
        try:
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            data = response.read(4_000_000)
            if allow_404 and response.status == 404:
                return None
            if not 200 <= response.status < 300:
                try:
                    value = json.loads(data)
                    reason = str(value.get('message') or value.get('error') or 'API request rejected')
                except (ValueError, AttributeError):
                    reason = 'API request rejected'
                raise RuntimeError('Gitee HTTP %d: %s' % (response.status, reason.replace(self.token, '[redacted]')[:300]))
            return json.loads(data)
        finally:
            connection.close()

    def attachments(self, release_id):
        values = []
        page = 1
        while True:
            batch = self.json_request('GET', API + '/releases/%d/attach_files?per_page=100&page=%d' % (release_id, page))
            if not isinstance(batch, list):
                raise RuntimeError('Unexpected attachment listing')
            values += batch
            if len(batch) < 100:
                return values
            page += 1

    def upload(self, release_id, name, payload):
        boundary = 'rm2027' + uuid.uuid4().hex
        prefix = ('--' + boundary + '\r\nContent-Disposition: form-data; name="file"; filename="' + name +
                  '"\r\nContent-Type: application/octet-stream\r\n\r\n').encode()
        body = prefix + payload + ('\r\n--' + boundary + '--\r\n').encode()
        return self.json_request('POST', API + '/releases/%d/attach_files' % release_id,
                                 body, 'multipart/form-data; boundary=' + boundary)

    def verify_download(self, release_id, attachment_id, expected_sha, expected_bytes):
        url = 'https://gitee.com' + API + '/releases/%d/attach_files/%d/download' % (release_id, attachment_id)
        for _ in range(6):
            parsed = urllib.parse.urlsplit(url)
            if parsed.scheme != 'https' or parsed.username or parsed.password:
                raise RuntimeError('Unsafe attachment download destination')
            connection = http.client.HTTPSConnection(parsed.netloc, timeout=600)
            headers = {'User-Agent': 'rm2027-research-archive'}
            # Never forward the token to a redirected object-storage host or non-API route.
            if parsed.netloc == 'gitee.com' and parsed.path.startswith(API + '/'):
                headers['Authorization'] = 'Bearer ' + self.token
            try:
                connection.request('GET', urllib.parse.urlunsplit(('', '', parsed.path, parsed.query, '')), headers=headers)
                response = connection.getresponse()
                if response.status in (301, 302, 303, 307, 308):
                    target = response.getheader('Location')
                    if not target:
                        raise RuntimeError('Missing attachment download redirect')
                    url = urllib.parse.urljoin(url, target)
                    continue
                if response.status != 200:
                    raise RuntimeError('Attachment verification download failed: HTTP %d' % response.status)
                digest, count = hashlib.sha256(), 0
                while chunk := response.read(1024 * 1024):
                    digest.update(chunk)
                    count += len(chunk)
                if count != expected_bytes or digest.hexdigest() != expected_sha:
                    raise RuntimeError('Uploaded attachment checksum/size mismatch; preserve evidence and stop')
                return
            finally:
                connection.close()
        raise RuntimeError('Too many attachment download redirects')

    def preserve_attachment(self, release_id, existing, name, payload):
        matches = [a for a in existing if a.get('name') == name]
        if len(matches) > 1:
            raise RuntimeError('Duplicate remote attachment names; stop without deleting anything')
        item = matches[0] if matches else self.upload(release_id, name, payload)
        expected = hashlib.sha256(payload).hexdigest()
        self.verify_download(release_id, int(item['id']), expected, len(payload))
        if not matches:
            existing.append(item)
        return {'name': name, 'attachment_id': int(item['id']), 'bytes': len(payload),
                'sha256': expected, 'remote_verified': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-commit', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    jobs = pinned_jobs(args.archive_commit)
    print('发行版附件方案：18项已审核refs，原提交身份保持；不执行 git push。')
    print('目标:', ORIGIN, '，发行版tag:', TAG)
    print('完整历史包约1.6GB；附件独立额度未经账号侧核实，不能保证足够。')
    print('每片原数据48MB，gzip后小于50MB；逐片远端下载校验；本地不保存完整bundle。')
    if not args.execute:
        print('仅本地预览；没有联网、创建发行版或上传。')
        return
    if not sys.stdin.isatty():
        raise RuntimeError('Run in your own interactive terminal; never put tokens in command arguments')
    AUDIT.mkdir(parents=True, exist_ok=True)
    ledger = AUDIT / ('release_history_' + args.archive_commit[:12] + '.json')
    if ledger.exists():
        report = json.loads(ledger.read_text())
        if report['archive_commit'] != args.archive_commit or report['jobs'] != jobs:
            raise RuntimeError('Existing upload ledger belongs to different reviewed assets')
        baseline = report['baseline_remote_refs']
    else:
        baseline = remote_refs()
        if baseline.get('refs/heads/main') != 'd735ee12bd950dca0e691cdf2f2c61f35cef8ffc':
            raise RuntimeError('Remote main changed since review')
        report = {'archive_commit': args.archive_commit, 'origin': ORIGIN, 'release_tag': TAG,
                  'baseline_remote_refs': baseline, 'jobs': jobs, 'parts': [], 'complete': False,
                  'created_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  'automatic_retries': 0, 'git_history_rewritten': False}

    def save():
        ledger.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    save()
    pending = [j for j in jobs if baseline.get(j['ref']) != j['oid']]
    if not pending:
        print('审核refs均已在远端，不需要另存历史包。')
        return
    known = sorted(set(oid for ref, oid in baseline.items() if not ref.endswith('^{}')))
    for oid in known:
        local('cat-file', '-e', oid)
    token = getpass.getpass('Gitee 私人令牌（projects 权限；不回显、不保存）: ').strip()
    if not token:
        raise RuntimeError('Empty token; no upload')
    client = Gitee(token)
    producer = None
    try:
        release = client.json_request('GET', API + '/releases/tags/' + TAG, allow_404=True)
        tag_ref = 'refs/tags/' + TAG
        current_remote = remote_refs()
        report['remote_refs_at_this_start'] = current_remote
        save()
        existing_tag = current_remote.get(tag_ref + '^{}', current_remote.get(tag_ref))
        if existing_tag and existing_tag != BASE_ARCHIVE:
            raise RuntimeError('Archive release tag already points elsewhere; no replacement or release creation')
        if shutil.disk_usage(AUDIT).free < 160_000_000:
            raise RuntimeError('Need at least 160 MB local free space for one temporary attachment')
        data = ''.join(j['ref'] + '\n' for j in pending) + ''.join('^' + oid + '\n' for oid in known)
        producer = subprocess.Popen(['git', '-c', 'gc.auto=0', '-c', 'pack.threads=1',
                                     '-c', 'pack.windowMemory=128m', 'bundle', 'create', '--quiet', '-', '--stdin'],
                                    cwd=REPO, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    start_new_session=True)
        producer.stdin.write(data.encode())
        producer.stdin.close()
        raw = producer.stdout.read(CHUNK_BYTES)
        header = bundle_header(raw)
        if any(header['bundled_refs'].get(j['ref']) != j['oid'] for j in pending):
            raise RuntimeError('Bundle header omitted an audited pending ref')
        report.update(header)
        if release is None:
            release = client.json_request('POST', API + '/releases', {
                'tag_name': TAG, 'target_commitish': BASE_ARCHIVE,
                'name': 'RM2027 immutable research history archive 2026-10-06',
                'body': 'Incremental Git bundle attachments. Restore with the existing Gitee mirror and the SHA256 manifest. '
                        'Original research refs/history remain unchanged. Incomplete until the final manifest is uploaded and verified.',
                'prerelease': True})
        rid = int(release['id'])
        remote = remote_refs()
        if remote.get(tag_ref + '^{}', remote.get(tag_ref)) != BASE_ARCHIVE:
            raise RuntimeError('Archive release tag points to an unexpected commit; stop without modifying it')
        report['release_id'] = rid
        save()
        existing = client.attachments(rid)
        digest, total, parts, number = hashlib.sha256(), 0, [], 0
        with tempfile.TemporaryDirectory(prefix='history-upload-', dir=AUDIT) as scratch:
            while raw:
                number += 1
                digest.update(raw)
                total += len(raw)
                payload = gzip.compress(raw, compresslevel=1, mtime=0)
                if len(payload) >= 50_000_000:
                    raise RuntimeError('Temporary gzip part exceeds the reviewed single-file ceiling')
                name = 'history-' + args.archive_commit[:12] + '.part-%04d.gz' % number
                temporary = pathlib.Path(scratch) / name
                temporary.write_bytes(payload)
                print('上传/核实分片 %d，%.2fMB' % (number, len(payload) / 1e6), flush=True)
                item = client.preserve_attachment(rid, existing, name, payload)
                item.update({'raw_bytes': len(raw), 'raw_sha256': hashlib.sha256(raw).hexdigest(), 'number': number})
                parts.append(item)
                report['parts'] = parts
                report['stream_bytes_verified_so_far'] = total
                save()
                temporary.unlink()  # Only our derived, remote-verified temporary file; never original evidence.
                raw = producer.stdout.read(CHUNK_BYTES)
        error = producer.stderr.read().decode(errors='replace')
        if producer.wait() != 0:
            raise RuntimeError('Git bundle creation failed: ' + error[:300])
        manifest = {'format': 'rm2027-incremental-git-bundle-parts-v1', 'archive_commit': args.archive_commit,
                    'origin': ORIGIN, 'release_tag': TAG, 'release_id': rid,
                    'baseline_remote_refs': baseline, **header, 'parts': parts,
                    'bundle_bytes': total, 'bundle_sha256': digest.hexdigest(),
                    'restore': 'Clone the existing Gitee repository as a separate bare mirror, then restore these verified parts. '
                               'All prerequisite commits must exist; do not apply to original worktrees.'}
        name = 'history-' + args.archive_commit[:12] + '.manifest.json'
        manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode()
        manifest_path = AUDIT / name
        manifest_path.write_bytes(manifest_bytes)
        result = client.preserve_attachment(rid, existing, name, manifest_bytes)
        report.update({'complete': True, 'manifest': result, 'bundle_bytes': total,
                       'bundle_sha256': digest.hexdigest(), 'stop_reason': None})
        final = remote_refs()
        report['original_remote_refs_unchanged'] = all(final.get(ref) == oid for ref, oid in current_remote.items())
        save()
        if not report['original_remote_refs_unchanged']:
            raise RuntimeError('A baseline remote ref changed during attachment upload; review separately')
        print('全部分片及manifest已远端下载校验；历史在发行版附件中，不代表原超限分支已直接push。')
        print('恢复manifest:', manifest_path)
    except BaseException as exc:
        report['complete'] = False
        report['stop_reason'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc).replace(token, '[redacted]')
        save()
        raise
    finally:
        if producer is not None and producer.poll() is None:
            os.killpg(producer.pid, signal.SIGTERM)
            try:
                producer.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(producer.pid, signal.SIGKILL)
                producer.wait()
        print('上传记录（不含令牌）:', ledger)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('用户停止；已上传附件保留，原证据不变。', file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        # Do not echo arbitrary exceptions from transports that might contain authentication headers.
        print('STOP:', str(exc) if isinstance(exc, RuntimeError) else type(exc).__name__, file=sys.stderr)
        sys.exit(1)
