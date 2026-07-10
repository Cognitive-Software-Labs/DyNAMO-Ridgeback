#!/usr/bin/env bash
# Query the local Isaac Sim MCP server (streamable HTTP, port 9904) from
# the shell — used during the Isaac port to resolve exact 6.0 extension /
# API / OmniGraph-node names without a native MCP client.
#
#   tools/isaac/mcp_query.sh list
#   tools/isaac/mcp_query.sh call <tool_name> '<json-arguments>'
#
# Server setup: docs in tools/isaac/PORT_PLAN.md (P1). Container:
#   docker run -d --name isaacsim-mcp -p 127.0.0.1:9904:9904 \
#     --env-file ~/.config/nvidia/mcp.env isaacsim-mcp:latest
set -euo pipefail

URL="${MCP_URL:-http://localhost:9904/mcp}"
HDR=(-H "Content-Type: application/json" -H "Accept: application/json, text/event-stream")

_post() {  # $1 = body, $2 = extra header (optional)
    if [ -n "${2:-}" ]; then
        curl -s -D /tmp/mcp_headers.$$ -X POST "$URL" "${HDR[@]}" -H "$2" -d "$1"
    else
        curl -s -D /tmp/mcp_headers.$$ -X POST "$URL" "${HDR[@]}" -d "$1"
    fi
}

_data() { grep '^data:' | sed 's/^data: //' ; }

# --- open a session -----------------------------------------------------
init_body='{"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"mcp_query.sh","version":"1"}}}'
_post "$init_body" >/dev/null
SESSION=$(awk -v IGNORECASE=1 '/^mcp-session-id:/ {print $2}' /tmp/mcp_headers.$$ | tr -d '\r')
[ -n "$SESSION" ] || { echo "no mcp-session-id from $URL" >&2; exit 1; }
SESS_HDR="mcp-session-id: $SESSION"
_post '{"jsonrpc":"2.0","method":"notifications/initialized"}' "$SESS_HDR" >/dev/null

# --- dispatch ------------------------------------------------------------
case "${1:-list}" in
    list)
        _post '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' "$SESS_HDR" \
            | _data | python3 -c '
import json, sys
for t in json.load(sys.stdin)["result"]["tools"]:
    print(f"{t['"'"'name'"'"']}: {t.get('"'"'description'"'"',"" ).strip().splitlines()[0][:110]}")'
        ;;
    call)
        tool="$2"; args="${3:-{\}}"
        body=$(python3 -c "
import json, sys
print(json.dumps({'jsonrpc':'2.0','id':2,'method':'tools/call',
                  'params':{'name':sys.argv[1],'arguments':json.loads(sys.argv[2])}}))
" "$tool" "$args")
        _post "$body" "$SESS_HDR" | _data | python3 -c '
import json, sys
r = json.load(sys.stdin)
if "error" in r:
    print("ERROR:", r["error"].get("message"), file=sys.stderr); sys.exit(1)
for item in r["result"]["content"]:
    print(item.get("text", item))'
        ;;
    *)
        echo "usage: $0 list | call <tool> '<json args>'" >&2; exit 2;;
esac
rm -f /tmp/mcp_headers.$$
