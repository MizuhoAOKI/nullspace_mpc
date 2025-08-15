#!/usr/bin/env bash
set -euo pipefail

PREFIX="${PREFIX:-/usr/local}"
CORES="${CORES:-$(nproc)}"
WORKDIR="${WORKDIR:-/tmp/build-osqp-eigen}"

# Check if already installed
if [[ -f "${PREFIX}/include/OsqpEigen/OsqpEigen.h" ]]; then
  echo "[osqp-eigen] already installed at ${PREFIX}"
  exit 0
fi

echo "[osqp-eigen] building from source..."
rm -rf "${WORKDIR}"
mkdir -p "${WORKDIR}"
cd "${WORKDIR}"

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir -p build && cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" ..
cmake --build . -- -j"${CORES}"
sudo cmake --build . --target install
sudo ldconfig

echo "[osqp-eigen] installed to ${PREFIX}"
