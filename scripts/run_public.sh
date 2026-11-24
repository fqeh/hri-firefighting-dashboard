#!/usr/bin/env bash
# Share the dashboard with remote teammates through a Cloudflare quick tunnel.
# Requires the local stack (scripts/run_dashboard.sh + the gui container).
#   run_public.sh           build the UI, serve it with `vite preview` on 127.0.0.1:4173
#                           (same /api and /gazebo proxies as the dev server) and print a
#                           https://*.trycloudflare.com URL
#   run_public.sh rebuild   rebuild the UI and reload the preview; the public URL is kept
# The URL is public and has no password: anyone holding it can control the run.
set -Eeuo pipefail
cd "$(dirname "$0")/.."
PIDFILE=results/public-preview.pid
start_preview() { (cd dashboard && exec node node_modules/vite/bin/vite.js preview --host 127.0.0.1 --port 4173 --strictPort) & echo "$!" > "$PIDFILE"; }
if [[ "${1:-}" == rebuild ]]; then
  npm --prefix dashboard run build
  [[ -f "$PIDFILE" ]] && kill -TERM "$(cat "$PIDFILE")" 2>/dev/null || true   # the running tunnel script restarts it
  echo 'UI rebuilt; the preview reloads in a few seconds and the public URL is unchanged.'
  exit 0
fi
CLOUDFLARED="${CLOUDFLARED:-$(command -v cloudflared || echo "$HOME/bin/cloudflared")}"
[[ -x "$CLOUDFLARED" ]] || { echo "cloudflared not found (set CLOUDFLARED=/path/to/cloudflared)" >&2; exit 2; }
npm --prefix dashboard run build
# Stop the whole process group on exit: wrappers do not forward signals to vite or cloudflared.
cleanup() { trap - EXIT; rm -f "$PIDFILE"; kill -TERM 0 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start_preview
"$CLOUDFLARED" tunnel --no-autoupdate --url http://127.0.0.1:4173 2>&1 | while IFS= read -r line; do
  echo "$line"
  [[ "$line" =~ (https://[a-z0-9-]+\.trycloudflare\.com) ]] && echo "PUBLIC DASHBOARD: ${BASH_REMATCH[1]}"
done &
tunnel=$!
# Keep the tunnel (and its URL) alive; bring the preview back after a rebuild.
while kill -0 "$tunnel" 2>/dev/null; do
  kill -0 "$(cat "$PIDFILE")" 2>/dev/null || { sleep 1; start_preview; }
  sleep 1
done
exit 1
