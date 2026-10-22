set -eo pipefail
root=$1
test -d "$root" && test ! -L "$root" || exit 1
test -f "$root/.hei-sync-root" || { echo 'Run init for this target first.' >&2; exit 1; }
test "$(cat "$root/.hei-sync-root")" = 'hei-remote-dev-v1' || exit 1
test ! -e "$root/.git" || { echo 'Refusing to overwrite a Git checkout.' >&2; exit 1; }
cd -- "$root"
