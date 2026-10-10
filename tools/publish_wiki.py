#!/usr/bin/env python3

"""
Publish docs/wiki/ to the repository's GitHub wiki.

The GitHub wiki lives in another repository (`<repo>.wiki.git`) and doesn't
understand the links as they are written in docs/wiki/:

- its pages have no extension: `Simulation.md` gives a 404, you have to link
  to the page name;
- its page names are the titles in this script, not the file names;
- it can't resolve paths relative to the code (`../../src/...`), because the
  code is in another repository;
- it shows the page name as the title, so the leading `#` of each file would
  appear twice.

This script does those four conversions and pushes the result. The wiki is
never edited by hand: change docs/wiki/ and publish again.

Usage:
    python3 tools/publish_wiki.py --dry-run     # only generate and report
    python3 tools/publish_wiki.py               # generate, commit and push
    python3 tools/publish_wiki.py --output DIR  # leave the result in DIR
"""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'docs' / 'wiki'

# Title of each page in the wiki. The key is the file in docs/wiki/.
PAGES = {
    'Home.md': 'Home',
    'Installation-and-build.md': 'Installation and build',
    'Simulation.md': 'Running the simulation',
    'Architecture.md': 'Architecture and data flow',
    'Nodes-and-topics.md': 'Nodes, topics and diagnostics',
    'Guidance-modes.md': 'Guidance modes',
    'Code-walkthrough.md': 'Code walkthrough',
    'Line-by-line-project-and-build-files.md': 'Project and build files line by line',
    'Line-by-line-odometry-nodes.md': 'Odometry and tf2 nodes line by line',
    'Line-by-line-guidance-modes.md': 'Guidance modes line by line',
    'Workspace-file-map.md': 'Workspace map and dependencies',
    'Quick-reference-and-troubleshooting.md': 'Quick reference and troubleshooting',
}

# Suggested reading order, for the sidebar.
READING_ORDER = [
    'Home.md',
    'Installation-and-build.md',
    'Simulation.md',
    'Architecture.md',
    'Nodes-and-topics.md',
    'Guidance-modes.md',
    'Code-walkthrough.md',
    'Line-by-line-project-and-build-files.md',
    'Line-by-line-odometry-nodes.md',
    'Line-by-line-guidance-modes.md',
    'Workspace-file-map.md',
    'Quick-reference-and-troubleshooting.md',
]

PAGE_LINK = re.compile(r'\]\(([A-Za-z0-9._-]+\.md)(#[^)]*)?\)')
REPO_LINK = re.compile(r'\]\(\.\./\.\./([^)]+)\)')
TITLE = re.compile(r'\A#\s+[^\n]*\n+')


def slug(page_name):
    """Return a page name as used in a URL: spaces become hyphens."""
    return page_name.replace(' ', '-')


def repository_url():
    url = subprocess.run(
        ['git', 'remote', 'get-url', 'origin'],
        cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    if url.startswith('git@github.com:'):
        url = 'https://github.com/' + url[len('git@github.com:'):]
    return url[:-len('.git')] if url.endswith('.git') else url


def convert(text, file_name, repo_base, warnings):
    def page(match):
        target, anchor = match.group(1), match.group(2) or ''
        if target not in PAGES:
            warnings.append(f'{file_name}: links to {target}, which is not a wiki page')
            return match.group(0)
        return f']({slug(PAGES[target])}{anchor})'

    def repo(match):
        path = match.group(1)
        target = ROOT / path
        if not target.exists():
            warnings.append(f'{file_name}: links to {path}, which does not exist in the repository')
        kind = 'tree' if target.is_dir() else 'blob'
        return f']({repo_base}/{kind}/main/{path})'

    text = PAGE_LINK.sub(page, text)
    text = REPO_LINK.sub(repo, text)
    # GitHub already shows the page name as the title.
    return TITLE.sub('', text, count=1)


def sidebar():
    lines = ['### Reading order', '']
    for file_name in READING_ORDER:
        name = PAGES[file_name]
        lines.append(f'- [{name}]({slug(name)})')
    return '\n'.join(lines) + '\n'


def generate(destination, repo_base):
    warnings = []
    files = sorted(p.name for p in SOURCE.glob('*.md'))
    missing = [f for f in files if f not in PAGES]
    if missing:
        sys.exit(f'These docs/wiki/ files have no page title in PAGES: {missing}')

    for file_name in files:
        text = (SOURCE / file_name).read_text(encoding='utf-8')
        output = destination / f'{slug(PAGES[file_name])}.md'
        output.write_text(convert(text, file_name, repo_base, warnings), encoding='utf-8')
    (destination / '_Sidebar.md').write_text(sidebar(), encoding='utf-8')
    return files, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dry-run', action='store_true',
                        help='generate the pages and show the diff, without pushing anything')
    parser.add_argument('--output', metavar='DIR',
                        help='leave the generated pages in DIR instead of publishing them')
    args = parser.parse_args()

    repo_base = repository_url()

    if args.output:
        destination = Path(args.output).resolve()
        destination.mkdir(parents=True, exist_ok=True)
        files, warnings = generate(destination, repo_base)
        for warning in warnings:
            print(f'WARNING {warning}')
        print(f'{len(files)} pages generated in {destination}')
        return

    temporary = Path(tempfile.mkdtemp(prefix='wiki-'))
    clone = temporary / 'wiki'
    try:
        subprocess.run(['git', 'clone', '--quiet', f'{repo_base}.wiki.git', str(clone)], check=True)
        for previous in clone.glob('*.md'):
            previous.unlink()
        files, warnings = generate(clone, repo_base)
        for warning in warnings:
            print(f'WARNING {warning}')

        subprocess.run(['git', 'add', '-A'], cwd=clone, check=True)
        status = subprocess.run(['git', 'status', '--porcelain'],
                                cwd=clone, capture_output=True, text=True, check=True).stdout
        if not status.strip():
            print('The wiki is already up to date; nothing to push.')
            return

        print(subprocess.run(['git', 'diff', '--cached', '--stat'],
                             cwd=clone, capture_output=True, text=True, check=True).stdout)
        if args.dry_run:
            print(f'--dry-run: nothing is pushed. Result in {clone}')
            temporary = None  # don't delete it, so it can be inspected
            return

        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                                cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
        subprocess.run(['git', 'commit', '--quiet', '-m',
                        f'Publish docs/wiki/ from {commit}'], cwd=clone, check=True)
        subprocess.run(['git', 'push', '--quiet'], cwd=clone, check=True)
        print(f'{len(files)} pages published to {repo_base}/wiki')
    finally:
        if temporary is not None:
            shutil.rmtree(temporary, ignore_errors=True)


if __name__ == '__main__':
    main()
