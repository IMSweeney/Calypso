#!/usr/bin/env bash
# A capture is recorded data rather than source: the bytes live in a bucket and the repository keeps
# what describes them. This moves them, and nothing about it belongs to the repository — the store is
# whoever's account holds it, so it arrives as CAPTURES_REMOTE, an `mc` alias and bucket:
#
#   export CAPTURES_REMOTE=r2/calypso-captures
#
# Usage: tools/captures.sh ls | push [name...] | pull <name>...
set -euo pipefail

: "${CAPTURES_REMOTE:?set CAPTURES_REMOTE, e.g. r2/calypso-captures}"
remote=${CAPTURES_REMOTE%/}
cd "$(dirname "$0")/.."

# The kinds of file that are a recording. A note or a plot is reviewable and stays in the repository.
is_capture() {
  case $1 in
    *.pblog | *.bin | *.log) return 0 ;;
    *) return 1 ;;
  esac
}

push_one() {
  local name=$1 path=captures/$1
  is_capture "$name" || die "$name: not a capture"
  [[ -f $path ]] || die "$path: no such capture"
  mc cp --quiet "$path" "$remote/$name"
  printf '%s -> %s/%s\n' "$path" "$remote" "$name"
}

pull_one() {
  local name=$1
  is_capture "$name" || die "$name: not a capture"
  mc cp --quiet "$remote/$name" "captures/$name"
  printf '%s/%s -> captures/%s\n' "$remote" "$name" "$name"
}

die() {
  echo "$*" >&2
  exit 1
}

case ${1:-ls} in
  ls)
    mc ls --recursive "$remote/"
    ;;
  push)
    shift
    if (($#)); then
      for name in "$@"; do push_one "$name"; done
    else
      for path in captures/*.pblog captures/*.bin captures/*.log; do
        if [[ -e $path ]]; then push_one "${path#captures/}"; fi
      done
    fi
    ;;
  pull)
    shift
    (($#)) || die "pull needs a capture name: $(cd captures && ls ./*.pblog 2>/dev/null | head -3)"
    for name in "$@"; do pull_one "$name"; done
    ;;
  *)
    die "usage: $0 ls | push [name...] | pull <name>..."
    ;;
esac
