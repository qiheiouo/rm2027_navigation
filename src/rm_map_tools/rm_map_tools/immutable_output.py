"""Helpers for publishing complete files without replacing existing paths."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import ctypes
import errno
import os
from pathlib import Path
import platform
import sys
import tempfile
from typing import TypeVar


OUTPUT_FILE_MODE = 0o644
_AT_FDCWD = -100
_RENAME_NOREPLACE = 1
_Result = TypeVar("_Result")


def path_entry_exists(path: Path) -> bool:
    """Return True for every directory entry, including dangling symlinks."""
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    return True


def fsync_file(path: Path) -> None:
    """Synchronize a completed regular file before publication."""
    with path.open("rb") as stream:
        os.fsync(stream.fileno())


def fsync_directory(path: Path) -> None:
    """Synchronize directory entry changes."""
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _renameat2_function():
    library = ctypes.CDLL(None, use_errno=True)
    try:
        function = library.renameat2
    except AttributeError:
        return None
    function.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    function.restype = ctypes.c_int
    # Keep the library alive for the duration of the C function wrapper.
    function._rm_map_tools_library = library
    return function


def _renameat2_syscall_number() -> int | None:
    # Used only when an older libc does not expose renameat2. Unknown Linux
    # architectures fail closed instead of falling back to replacing rename(2).
    return {
        "aarch64": 276,
        "arm64": 276,
        "armv7l": 382,
        "i386": 353,
        "i686": 353,
        "ppc64": 357,
        "ppc64le": 357,
        "riscv64": 276,
        "s390x": 347,
        "x86_64": 316,
    }.get(platform.machine().lower())


def _linux_rename_no_replace(source: Path, destination: Path) -> None:
    """Atomically rename a directory while refusing every existing target."""
    if not sys.platform.startswith("linux"):
        raise OSError(
            errno.ENOTSUP,
            "atomic directory no-replace publication requires Linux renameat2",
            destination,
        )

    source_bytes = os.fsencode(source)
    destination_bytes = os.fsencode(destination)
    function = _renameat2_function()
    if function is not None:
        ctypes.set_errno(0)
        result = function(
            _AT_FDCWD,
            source_bytes,
            _AT_FDCWD,
            destination_bytes,
            _RENAME_NOREPLACE,
        )
        error_number = ctypes.get_errno()
    else:
        syscall_number = _renameat2_syscall_number()
        if syscall_number is None:
            raise OSError(
                errno.ENOTSUP,
                "renameat2 is unavailable; refusing unsafe directory publication",
                destination,
            )
        library = ctypes.CDLL(None, use_errno=True)
        syscall = library.syscall
        syscall.restype = ctypes.c_long
        ctypes.set_errno(0)
        result = syscall(
            ctypes.c_long(syscall_number),
            ctypes.c_int(_AT_FDCWD),
            ctypes.c_char_p(source_bytes),
            ctypes.c_int(_AT_FDCWD),
            ctypes.c_char_p(destination_bytes),
            ctypes.c_uint(_RENAME_NOREPLACE),
        )
        error_number = ctypes.get_errno()

    if result != 0:
        raise OSError(
            error_number or errno.EIO,
            os.strerror(error_number or errno.EIO),
            destination,
        )


def resolved_output_path(value: str | Path, label: str) -> Path:
    """Resolve an output path while refusing an indirect final symlink."""
    requested_path = Path(value).expanduser()
    if requested_path.is_symlink():
        raise ValueError(
            f"{label} is a symlink; refusing indirect output: {requested_path}"
        )
    # Resolve only the parent. Resolving the whole requested path would follow
    # a final symlink created between the check above and resolve(), redirecting
    # publication away from the requested basename. The atomic no-replace
    # operation below remains the authority for a concurrent final entry.
    return requested_path.parent.resolve() / requested_path.name


def new_output_path(value: str | Path, label: str) -> Path:
    """Resolve a new output path and create its parent."""
    path = resolved_output_path(value, label)
    if path_entry_exists(path):
        raise ValueError(f"{label} already exists; refusing overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def require_distinct_paths(paths: Mapping[str, Path]) -> None:
    """Reject output/input roles that resolve to the same filesystem path."""
    labels = tuple(paths)
    for index, left_label in enumerate(labels):
        for right_label in labels[index + 1:]:
            if paths[left_label] == paths[right_label]:
                raise ValueError(
                    f"{left_label} and {right_label} paths must be distinct: "
                    f"{paths[left_label]}"
                )


def publish_new_file(
    destination: Path,
    label: str,
    writer: Callable[[Path], _Result],
) -> _Result:
    """Publish a complete file atomically without replacing an existing path.

    A parent-directory fsync error is propagated after publication. The fully
    written destination is deliberately retained: deleting by pathname would
    introduce a new race with an independently replaced entry.
    """
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".tmp",
        dir=destination.parent,
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        writer_result = writer(temporary_path)
        os.chmod(temporary_path, OUTPUT_FILE_MODE)
        fsync_file(temporary_path)
        try:
            # A hard link is an atomic no-replace publication on the same
            # filesystem. The sibling staging file guarantees that condition.
            os.link(temporary_path, destination)
        except FileExistsError as exc:
            raise ValueError(
                f"{label} appeared while writing; refusing overwrite: "
                f"{destination}"
            ) from exc
        fsync_directory(destination.parent)
        return writer_result
    finally:
        temporary_path.unlink(missing_ok=True)


def publish_new_directory(source: Path, destination: Path, label: str) -> None:
    """Atomically publish a complete directory without replacing any entry.

    There is deliberately no fallback to ``os.rename``/``os.replace`` because
    both permit replacement races for directories. Linux renameat2 is used
    directly (including a raw-syscall fallback for older libc); unsupported
    systems fail closed and leave ``source`` unpublished for caller cleanup.
    If the post-rename parent fsync fails, the complete destination is retained
    and the error is propagated; pathname rollback would not be race-safe.
    """
    try:
        _linux_rename_no_replace(source, destination)
    except OSError as exc:
        if exc.errno in {errno.EEXIST, errno.ENOTEMPTY}:
            raise ValueError(
                f"{label} appeared while writing; refusing overwrite: "
                f"{destination}"
            ) from exc
        raise
    fsync_directory(destination.parent)


def write_new_text(destination: Path, label: str, content: str) -> None:
    """Atomically publish UTF-8 text with canonical Unix newlines."""

    def write_staged(path: Path) -> None:
        path.write_text(content, encoding="utf-8", newline="\n")

    publish_new_file(destination, label, write_staged)
