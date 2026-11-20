#!/usr/bin/env python3
"""Scan a folder before it is made public.

Looks for private keys, tokens and passwords, e-mail addresses, personal home paths,
names of writing/coding tools, and files above 50 MB. Hits in frozen package files are
listed separately because those files are never edited.

usage: python3 tools/prepublish_check.py [FOLDER]
"""
import json
import re
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
OK_EMAILS = {'d131gp038@mail2.mcut.edu.tw', 'iggy@mail.mcut.edu.tw', 'jnlin@mail.mcut.edu.tw',
             'rlshen@mail.ntpu.edu.tw'}
TOOLS = re.compile(r'\b(claude|anthropic|chatgpt|openai|copilot|gpt-?[345]\w*|llm)\b', re.I)
SECRET = re.compile(r'(-----BEGIN [A-Z ]*PRIVATE KEY-----|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}'
                    r'|xox[bap]-[A-Za-z0-9-]{10,}|AKIA[0-9A-Z]{16}|\bpsk\s*=|wpa_passphrase'
                    r'|(?<![a-z])pass(word|wd)\s*[:=]\s*\S+|api[_-]?key\s*[:=]\s*\S+)', re.I)
EMAIL = re.compile(r'[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}')
HOME = re.compile(r'/home/(?!ubuntu\b|user\b|USER\b|\$)[A-Za-z0-9_.-]+|/Users/[A-Za-z0-9_.-]+|C:\\\\Users\\\\\w+')
SKIP = {'.git', '__pycache__', 'build', 'install', 'log'}
BIG = 50 * 1024 * 1024


def frozen_files():
    out = set()
    for fz in ROOT.rglob('*_FREEZE_HASHES.json'):
        try:
            out.update((fz.parent / f).resolve() for f in json.loads(fz.read_text())['sha256'])
        except (ValueError, KeyError):
            pass
    return out


def main():
    frozen = frozen_files()
    hits = {k: [] for k in ('secret', 'email', 'home path', 'tool name', 'tool name (frozen file)', 'large file')}
    for p in sorted(ROOT.rglob('*')):
        rel = p.relative_to(ROOT)
        if not p.is_file() or SKIP.intersection(rel.parts) or rel.name == 'prepublish_check.py':
            continue
        tool_key = 'tool name (frozen file)' if p.resolve() in frozen else 'tool name'
        if p.stat().st_size > BIG:
            hits['large file'].append(f'{rel} ({p.stat().st_size / 1e6:.0f} MB)')
        if TOOLS.search(rel.as_posix()):
            hits[tool_key].append(f'{rel} (file name)')
        try:
            text = p.read_text(encoding='utf-8')
        except (UnicodeDecodeError, OSError):
            continue
        for n, line in enumerate(text.splitlines(), 1):
            if SECRET.search(line):
                hits['secret'].append(f'{rel}:{n}')
            for e in EMAIL.findall(line):
                if e.lower() not in OK_EMAILS and not e.lower().endswith(('@example.com', '@users.noreply.github.com')):
                    hits['email'].append(f'{rel}:{n}: {e}')
            if HOME.search(line):
                hits['home path'].append(f'{rel}:{n}')
            if TOOLS.search(line):
                hits[tool_key].append(f'{rel}:{n}')
    for k, v in hits.items():
        print(f'{k}: {len(v)}')
        for x in v[:40]:
            print('   ', x)
        if len(v) > 40:
            print(f'    ... and {len(v) - 40} more')
    fail = hits['secret'] or hits['large file'] or hits['tool name']
    print('PREPUBLISH_CHECK', 'FAIL' if fail else 'PASS')
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
