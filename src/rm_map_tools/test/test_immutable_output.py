from __future__ import annotations

import errno
from pathlib import Path

import pytest

from rm_map_tools import immutable_output


def test_directory_publish_fail_closed_when_atomic_primitive_is_unavailable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "staging"
    destination = tmp_path / "candidate"
    source.mkdir()
    (source / "manifest.yaml").write_text("complete\n", encoding="utf-8")
    monkeypatch.setattr(immutable_output.sys, "platform", "unsupported")

    with pytest.raises(OSError) as caught:
        immutable_output.publish_new_directory(source, destination, "map bundle")

    assert caught.value.errno == errno.ENOTSUP
    assert source.is_dir()
    assert not destination.exists()


def test_file_writer_failure_never_publishes_and_cleans_staging(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "output.json"

    def failing_writer(path: Path) -> None:
        path.write_bytes(b"partial")
        raise OSError("injected writer verification failure")

    with pytest.raises(OSError, match="injected writer verification failure"):
        immutable_output.publish_new_file(destination, "output", failing_writer)

    assert not destination.exists()
    assert not tuple(tmp_path.glob(".*.tmp"))


def test_final_symlink_race_is_not_followed_during_output_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    destination = tmp_path / "output.json"
    redirected = tmp_path / "redirected.json"
    original_is_symlink = Path.is_symlink
    injected = False

    def create_after_check(path: Path) -> bool:
        nonlocal injected
        if path == destination and not injected:
            injected = True
            path.symlink_to(redirected)
            return False
        return original_is_symlink(path)

    monkeypatch.setattr(Path, "is_symlink", create_after_check)
    with pytest.raises(ValueError, match="already exists"):
        immutable_output.new_output_path(destination, "output")

    assert original_is_symlink(destination)
    assert not redirected.exists()
