#!/bin/bash
# Check that every relative markdown link in the docs points at a real file.
# The document map in CLAUDE.md is how anything gets found (R9); a dead link
# there loses knowledge as effectively as deleting it.
#
# P12.0 (2026-08-25) closed two holes:
#
#   * The document set was a hand-written glob list, which is the R34 shape and had
#     already drifted: .claude/agents/ (14 files) and .claude/skills/ were outside it
#     entirely, so a dead link in any of them was invisible. The set is now derived.
#
#   * A link was accepted if it resolved EITHER against the containing file's directory
#     OR against the repo root. Only the first is how markdown renders, so the second
#     was an amnesty: a README linking "docs/ops-playbook.md" is broken when rendered
#     (it means src/<pkg>/docs/...) yet passed because the root path exists. Measured
#     before removing it -- 344 of 344 links resolved relatively and none relied on the
#     fallback, so this tightens the rule without hiding anything that was already true.
#
# Usage: bash scripts/check_doc_links.sh [root]
#        The optional root exists so the positive control can point this at a
#        deliberately-broken copy. Same reason check_sim_real_parity.py takes one.
set -o pipefail

ROOT=${1:-/mnt/c/code/PX4_ROS2}
cd "$ROOT" || exit 1

broken=0
checked=0
docs=0

while IFS= read -r doc; do
  docs=$((docs + 1))
  dir=$(dirname "$doc")
  while IFS= read -r target; do
    [ -z "$target" ] && continue
    case "$target" in
      http*|\#*|mailto:*) continue ;;
    esac
    path=${target%%#*}
    [ -z "$path" ] && continue
    checked=$((checked + 1))
    # Relative to the file that contains it. That is the only resolution a renderer does.
    if [ ! -e "$dir/$path" ]; then
      echo "  BROKEN  $doc -> $target"
      broken=$((broken + 1))
    fi
  done < <(grep -oE '\]\([^)]+\)' "$doc" | sed 's/^](//; s/)$//')
done < <(
  find . -name '*.md' \
    -not -path './build/*' -not -path './install/*' -not -path './log/*' \
    -not -path './.git/*' -not -path './*/__pycache__/*' \
  | sed 's|^\./||' | sort
)

echo
echo "scanned $docs documents, checked $checked links, $broken broken"
[ "$broken" -eq 0 ] || exit 1
