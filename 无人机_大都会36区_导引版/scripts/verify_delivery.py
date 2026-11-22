"""Record first-version provenance without writing anything to the baseline."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
BASELINE=ROOT.parent/'无人机_仅算法1'
OUT=ROOT/'docs'/'verification_metropolis36'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    original=json.loads((OUT/'baseline_sha256.json').read_text(encoding='utf-8'))
    mismatches=[]
    changed=[]
    removed=[]
    for relative,expected in original.items():
        source=BASELINE/relative
        actual=digest(source) if source.is_file() else None
        if actual!=expected:
            mismatches.append(dict(path=relative,expected=expected,actual=actual))
        modified=ROOT/relative
        if not modified.is_file():
            removed.append(relative)
        elif digest(modified)!=expected:
            changed.append(dict(path=relative,baseline_sha256=expected,new_sha256=digest(modified)))
    excluded={'.git','__pycache__','.pytest_cache','.vscode','artifacts'}
    added=[]
    for path in ROOT.rglob('*'):
        relative=path.relative_to(ROOT)
        if not path.is_file() or excluded.intersection(relative.parts):
            continue
        if relative.as_posix() not in original and path.suffix not in ('.pyc','.nbc','.nbi','.pkl'):
            added.append(relative.as_posix())
    integrity=dict(baseline_directory=str(BASELINE),delivery_directory=str(ROOT),
                   checked_files=len(original),unchanged=not mismatches,mismatches=mismatches)
    (OUT/'baseline_integrity.json').write_text(json.dumps(integrity,ensure_ascii=False,indent=2),encoding='utf-8')
    changes=dict(changed=changed,removed=removed,added=sorted(added))
    (OUT/'source_changes.json').write_text(json.dumps(changes,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(checked_files=len(original),baseline_unchanged=not mismatches,
                         changed_files=len(changed),removed_files=len(removed),added_files=len(added))))
    if mismatches:
        raise SystemExit(1)


if __name__=='__main__':
    main()
