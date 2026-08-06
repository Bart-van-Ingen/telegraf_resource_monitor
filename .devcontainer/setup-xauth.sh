#!/usr/bin/env bash
# Ensures /tmp/.docker.xauth exists as a file (not a directory) before the
# devcontainer's docker compose stack starts. Docker auto-creates missing
# bind-mount sources as directories, which breaks the X11 auth mount on
# every reboot once /tmp is cleared. Run via devcontainer.json's
# "initializeCommand" so it happens on the host before "docker compose up".
set -e

XAUTH=/tmp/.docker.xauth

if [ -d "$XAUTH" ]; then
  rmdir "$XAUTH" 2>/dev/null || {
    echo "warning: $XAUTH is a non-empty directory owned by another user." >&2
    echo "Run: sudo rm -rf $XAUTH" >&2
    exit 0
  }
fi

if [ ! -f "$XAUTH" ]; then
  touch "$XAUTH"
  xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
  chmod 644 "$XAUTH"
fi
