#!/usr/bin/env bash
# Install/enable for the NEXT boot. This script never starts or restarts a mission.
set -euo pipefail
mission_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
service_name=rover-mission.service
case "${1:---enable}" in
  --enable)
    if [[ "$mission_root" != "$HOME/nav_explore_ws" ]]; then
      echo "This service expects the workspace at $HOME/nav_explore_ws" >&2
      exit 1
    fi
    if [[ "$(loginctl show-user "$(id -un)" -p Linger --value)" != yes ]]; then
      echo "Boot without login requires: sudo loginctl enable-linger $(id -un)" >&2
      exit 1
    fi
    service_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
    mkdir -p "$service_dir"
    if [[ -e "$service_dir/$service_name" ]]; then
      backup_dir="$mission_root/data/change_backups/mission_autostart_$(date +%Y%m%d_%H%M%S)"
      mkdir -p "$backup_dir"
      cp -a "$service_dir/$service_name" "$backup_dir/"
    fi
    systemd-analyze --user verify "$mission_root/scripts/systemd/$service_name"
    install -m 644 "$mission_root/scripts/systemd/$service_name" "$service_dir/$service_name"
    systemctl --user daemon-reload
    systemctl --user enable "$service_name"
    echo "Enabled for next boot. No mission was started or restarted."
    ;;
  --disable)
    systemctl --user disable "$service_name"
    echo "Autostart disabled. To stop a running mission: systemctl --user stop $service_name"
    ;;
  --status)
    systemctl --user is-enabled "$service_name" || true
    systemctl --user status "$service_name" --no-pager || true
    ;;
  *) echo "Usage: $0 [--enable|--disable|--status]" >&2; exit 2 ;;
esac
