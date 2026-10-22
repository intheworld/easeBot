set -eo pipefail
root=$1
if test -e "$root"; then
  test -d "$root" && test ! -L "$root" || exit 1
  if test -f "$root/.hei-sync-root"; then
    test "$(cat "$root/.hei-sync-root")" = 'hei-remote-dev-v1' || exit 1
    test ! -e "$root/.git" || exit 1
    echo "Already initialized: $root"
    exit 0
  fi
  test -z "$(find "$root" -mindepth 1 -maxdepth 1 -print -quit)" || {
    echo 'init requires a new or empty directory; choose a dedicated development path.' >&2; exit 1;
  }
fi
mkdir -p -- "$root"
printf '%s
' hei-remote-dev-v1 > "$root/.hei-sync-root"
echo "Initialized: $root"
