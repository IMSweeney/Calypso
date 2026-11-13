#!/usr/bin/env python3
r"""Merge wrapped comment lines into one thought per line.

AGENTS.md asks that no sentence be wrapped across two lines. This rewrites comments so a sentence
sits on one line, and changes nothing else.

Usage:
    unwrap_comments.py [--check] <file-or-dir>...

    --check   report what would change, write nothing
    default   rewrite in place

Joins a comment line into the one above while the one above does not end a sentence. Leaves alone:
blank comment lines (paragraph breaks), markdown lists and tables, code fences, `#` outside shell
scripts (a Rust attribute is not a comment), and multi-line string literals -- anything that follows
or ends with a trailing backslash, which is how a generator's `"...\n\` continuation looks.

Skips generated trees (`target`, `.dart_tool`, `protocol`, `gatt.rs`, `gatt_client.dart`).

After a run, prove only comments moved: strip comment lines from a pre-run copy and from the tree and
diff them. Formatting differences are expected; anything else is a bug.
"""

import re, sys, pathlib

FINAL = re.compile(r'[.!?:;][`"\'\u201d)\]]*$')
ABBREV = re.compile(r'\b(e\.g|i\.e|etc|vs|cf|resp|approx)\.$')
MARKER = re.compile(r'^([-*+|#>]|\d+\.|```)')
LINE = re.compile(r'^(\s*)(///|//!|//|#)?( ?)(.*)$')

def parts(line, suffix):
    """(indent, marker, text), or None if the line is not a comment."""
    m = LINE.match(line)
    indent, marker, _, text = m.groups()
    if marker is None:
        return None
    if marker == '#' and suffix != '.sh':
        return None
    if marker == '#' and text.startswith('!'):
        return None
    if marker == '#':
        # shell: `#` must be followed by a space or be bare
        if text and not line[len(indent) + 1:].startswith(' '):
            return None
    return indent, marker, text

def sentence_final(text):
    t = text.rstrip()
    if not t or ABBREV.search(t):
        return False
    return bool(FINAL.search(t))

def unwrap(lines, suffix, report):
    out, fence, i = [], False, 0
    while i < len(lines):
        line = lines[i]
        # A line that continues a Rust string literal, or one that does, is not a comment:
        # the `//` at its start is text the generator is writing.
        if out and out[-1].rstrip().endswith('\\'):
            out.append(line)
            i += 1
            continue
        if line.rstrip().endswith('\\'):
            fence_string = True
        else:
            fence_string = False
        p = parts(line, suffix)
        if p is None:
            fence = False
            out.append(line)
            i += 1
            continue
        indent, marker, text = p
        if fence_string:
            out.append(line)
            i += 1
            continue
        if text.strip().startswith('```'):
            fence = not fence
            out.append(line)
            i += 1
            continue
        if fence:
            out.append(line)
            i += 1
            continue
        merged = False
        while i + 1 < len(lines):
            if not text.strip():
                break
            q = parts(lines[i + 1], suffix)
            if q is None or q[0] != indent or q[1] != marker:
                break
            ntext = q[2]
            if not ntext.strip() or MARKER.match(ntext.strip()):
                break
            if sentence_final(text):
                break
            report.append((text.strip(), ntext.strip(), text.rstrip() + ' ' + ntext.strip()))
            text = text.rstrip() + ' ' + ntext.strip()
            merged = True
            i += 1
        out.append(indent + marker + (' ' if text else '') + text if merged else line)
        i += 1
    return out

check = '--check' in sys.argv
roots = [a for a in sys.argv[1:] if not a.startswith('--')]
targets = []
for root in roots:
    p = pathlib.Path(root)
    files = [p] if p.is_file() else [f for f in p.rglob('*') if f.is_file()]
    for f in files:
        if f.suffix not in ('.rs', '.dart', '.sh', '.proto'):
            continue
        if {'target', '.dart_tool', 'protocol'} & set(f.parts):
            continue
        if f.name in ('gatt_client.dart', 'gatt.rs'):
            continue
        head = f.read_text()[:200]
        if 'GENERATED' in head:
            continue
        targets.append(f)

total, samples = 0, []
for f in targets:
    s = f.read_text()
    out = unwrap(s.split('\n'), f.suffix, samples)
    if out != s.split('\n'):
        total += 1
        if not check:
            f.write_text('\n'.join(out))
print(f"{total} files would change; {len(samples)} lines merged")
for a, b, r in samples[:12]:
    print(f"  …{a[-50:]!r}\n  +{b[:60]!r}\n  ={r[:90]!r}")
