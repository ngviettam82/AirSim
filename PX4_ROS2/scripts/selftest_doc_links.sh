#!/bin/bash
# Positive control for check_doc_links.sh (P12.0, 2026-08-25).
#
# This gate reported "0 broken" for months. That is a comfortable number to read and
# nobody had ever seen the script print anything else, which is exactly the condition
# under which a gate quietly stops meaning anything. Two holes were found the first
# time someone asked it to refuse:
#
#   * case 4: the document set was a hand-written glob list (the R34 shape). It had
#     already drifted -- .claude/agents/ and .claude/skills/ were outside it, so a dead
#     link anywhere in them was invisible rather than broken.
#   * case 3: a link was accepted if it resolved against the REPO ROOT even when it did
#     not resolve against its own directory. Renderers only do the second, so the first
#     was an amnesty for links that are broken everywhere a human would click them.
#
# Cases 1, 5, 6 are the other half: a clean tree must stay green, and external links and
# bare anchors must not be dragged in. A checker that fails everything proves nothing.
#
# Usage: bash scripts/selftest_doc_links.sh
set -o pipefail

CANON=/mnt/c/code/PX4_ROS2
CHECKER="$CANON/scripts/check_doc_links.sh"
fails=0

run_case() {
  local name=$1 want=$2 setup=$3
  local tmp
  tmp=$(mktemp -d)
  mkdir -p "$tmp/docs" "$tmp/src/uav_navigation" "$tmp/.claude/agents"
  echo 'real target' > "$tmp/docs/ops-playbook.md"
  "$setup" "$tmp"

  bash "$CHECKER" "$tmp" > "$tmp/out.txt" 2>&1
  local got=$?
  if [ "$got" -eq "$want" ]; then
    echo "  ok    $name (exit $got as expected)"
  else
    echo "  FAIL  $name: expected exit $want, got $got"
    sed 's/^/          /' "$tmp/out.txt"
    fails=$((fails + 1))
  fi
  rm -rf "$tmp"
}

clean() {
  printf 'see [the playbook](ops-playbook.md)\n' > "$1/docs/guide.md"
}

broken_link() {
  clean "$1"
  printf 'see [a ghost](does-not-exist.md)\n' > "$1/docs/guide.md"
}

root_only_link() {
  # Resolves as docs/ops-playbook.md from the repo root, but the file lives at
  # src/uav_navigation/ so a reader clicking it lands nowhere. The old fallback
  # called this fine.
  printf 'see [the playbook](docs/ops-playbook.md)\n' > "$1/src/uav_navigation/README.md"
}

link_outside_the_old_hand_list() {
  clean "$1"
  printf 'see [a ghost](../../docs/nope.md)\n' > "$1/.claude/agents/some-agent.md"
}

external_links_ignored() {
  printf 'see [docs](https://example.com/x.md) and [mail](mailto:a@b.c)\n' \
    > "$1/docs/guide.md"
}

anchors_ignored() {
  printf 'jump to [a section](#somewhere) and [a real file](ops-playbook.md#part-2)\n' \
    > "$1/docs/guide.md"
}

anchor_on_a_missing_file() {
  printf 'jump to [a ghost](nope.md#part-2)\n' > "$1/docs/guide.md"
}

echo "=== selftest: check_doc_links.sh ==="
run_case "1 a clean tree passes (shield is not amnesty)"          0 clean
run_case "2 a plainly broken link is caught"                      1 broken_link
run_case "3 a root-only link is caught (removed amnesty)"         1 root_only_link
run_case "4 a link outside the old hand-list is caught (R34)"     1 link_outside_the_old_hand_list
run_case "5 http and mailto links are not dragged in"             0 external_links_ignored
run_case "6 bare anchors are ignored, file+anchor is checked"     0 anchors_ignored
run_case "7 an anchor on a missing file is still caught"          1 anchor_on_a_missing_file

echo
if [ "$fails" -eq 0 ]; then
  echo "SELFTEST PASS (7/7)"
  exit 0
fi
echo "SELFTEST FAIL ($fails case(s))"
exit 1
