"""Read original or archived trial logs without altering historical evidence."""
import gzip
import json


def rows(root, name):
    path = root / name
    compressed = not path.is_file()
    if compressed:
        path = root / (name + ".gz")
    opener = gzip.open if compressed else open
    with opener(path, "rt", encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line)
