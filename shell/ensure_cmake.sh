#!/usr/bin/env bash
set -euo pipefail

REQUIRED="${1:-3.18.0}"

# Return success if cmake exists and is >= REQUIRED
version_ok() { command -v cmake >/dev/null 2>&1 && printf "%s\n%s\n" "$REQUIRED" "$(cmake --version | head -1 | awk '{print $3}')" | sort -V | head -1 | grep -qx "$REQUIRED"; }

# Skip if version is sufficient
if version_ok; then
  echo "[cmake] sufficient version detected: $(cmake --version | head -1)"
  exit 0
fi

# Install newer cmake from Kitware APT
echo "[cmake] installing from Kitware APT..."
. /etc/os-release
install -d /usr/share/keyrings
curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/kitware.list
apt-get update
apt-get install -y --no-install-recommends cmake
hash -r
echo "[cmake] installed: $(cmake --version | head -1)"