#!/usr/bin/env bash
set -euo pipefail
exec >/home/beano/kendrick_home_automation/boot-tmux.log 2>&1

SESSION="startup"

# If session exists, do nothing
/usr/bin/tmux has-session -t "$SESSION" 2>/dev/null && exit 0

# Start tmux and one window (add more new-window lines later)
/usr/bin/tmux new-session -d -s "$SESSION" -n "A" "cd /home/beano/kendrick_home_automation; ./npm_launch.sh"
/usr/bin/tmux new-window  -t "$SESSION:" -n "B" "cd /home/beano/kendrick_home_automation; ./lighting_controller.sh"
# For now, just a shell so we can verify it works:
 /usr/bin/tmux new-session -d -s "$SESSION" -n "Shell" "bash"

exit 0
