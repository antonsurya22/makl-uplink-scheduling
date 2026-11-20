#!/usr/bin/env bash
# Put the GitHub URL and the Zenodo DOI into the docs, then rewrite the manifest.
# usage: bash tools/fill_links.sh https://github.com/OWNER/REPO 10.5281/zenodo.NNNNNNN
set -euo pipefail
[ $# -eq 2 ] || { echo "usage: $0 REPO_URL DOI" >&2; exit 2; }
URL="${1%.git}"
DOI="$2"
cd "$(dirname "$0")/.."
for f in README.md CITATION.cff docs/*.md; do
  sed -i -e "s#REPOSITORY_URL#${URL}#g" -e "s#ZENODO_DOI#${DOI}#g" "$f"
done
python3 tools/make_manifest.py
if grep -rn "REPOSITORY_URL\|ZENODO_DOI" README.md CITATION.cff docs; then
  echo "some placeholders are still there"
  exit 1
fi
