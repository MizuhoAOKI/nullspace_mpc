#!/usr/bin/env bash
set -euo pipefail

PREFIX="${PREFIX:-/usr/local}"
CORES="${CORES:-$(nproc)}"
WORKDIR="${WORKDIR:-/tmp/build-osqp}"

# Check if already installed
if pkg-config --exists osqp 2>/dev/null || [[ -f "${PREFIX}/lib/libosqp.so" ]]; then
  echo "[OSQP] already installed at ${PREFIX}"
  exit 0
fi

echo "[OSQP] building from source..."
rm -rf "${WORKDIR}"
mkdir -p "${WORKDIR}"
cd "${WORKDIR}"

git clone --recursive https://github.com/osqp/osqp.git
cd osqp
mkdir -p build && cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" ..
cmake --build . -- -j"${CORES}"
sudo cmake --build . --target install
sudo ldconfig

echo "[OSQP] installed to ${PREFIX}"
