"""Inspect/extract a source archive without links, special files or path traversal."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile

p = argparse.ArgumentParser()
p.add_argument('archive', type=Path)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
manifest = []
total = 0
try:
    with tarfile.open(a.archive, 'r:gz') as tar:
        members = tar.getmembers()
        if len(members) > 100000:
            raise ValueError('Unexpected number of files')
        seen = set()
        for m in members:
            name = PurePosixPath(m.name)
            if name.is_absolute() or '..' in name.parts or '\\' in m.name:
                raise ValueError('Unsafe archive path: ' + m.name)
            if not (m.isdir() or m.isfile() or m.issym() or m.islnk()):
                raise ValueError('Special file rejected: ' + m.name)
            if m.size < 0 or m.size > 512 * 1024**2:
                raise ValueError('Unexpected file size: ' + m.name)
            total += m.size
            if total > 2 * 1024**3:
                raise ValueError('Archive expands beyond the inspection budget')
            if m.isfile() and str(name) in seen:
                raise ValueError('Duplicate file: ' + m.name)
            seen.add(str(name))
        # Never create archive symlinks/hardlinks, preserve owners, or execute files.
        for m in members:
            target = a.output.joinpath(*PurePosixPath(m.name).parts)
            entry = dict(path=m.name, bytes=m.size, type='directory' if m.isdir() else 'file')
            if m.issym() or m.islnk():
                entry.update(type='skipped_link', link_target=m.linkname)
            elif m.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif m.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(m) as src, target.open('xb') as dst:
                    h = hashlib.sha256()
                    while block := src.read(1024 * 1024):
                        dst.write(block)
                        h.update(block)
                entry['sha256'] = h.hexdigest()
            manifest.append(entry)
    result = {'archive': str(a.archive), 'sha256': hashlib.sha256(a.archive.read_bytes()).hexdigest(),
              'total_bytes': total, 'members': manifest}
    (a.output / 'archive_manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != 'members'}, ensure_ascii=False))
    print('members', len(manifest), 'links_skipped', sum(m['type']=='skipped_link' for m in manifest))
except Exception:
    print('Inspection stopped; partial output retained at', a.output)
    raise
