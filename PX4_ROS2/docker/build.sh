#!/bin/bash
# P11.1 -- build the aircraft and sim images from the ONE Dockerfile, on both
# architectures, and say plainly which of them was actually proven.
#
# WHY arm64 IS SLOW HERE. There is no arm64 machine in this project yet (owner, to be
# confirmed in a later session), so arm64 is built under QEMU emulation. Compiling ~12
# ROS 2 C++ packages emulated is roughly an order of magnitude slower than native. That
# is a property of the measurement rig, not of the image, and it is the reason this
# script separates two very different claims:
#
#   DEPS      the arm64 base image and every apt/rosdep dependency RESOLVE and install.
#             This is where portability actually breaks -- a package with no arm64 build.
#   FULL      the workspace also COMPILES for arm64. Architecture-independent C++, so
#             this is far less likely to surprise, and far more expensive to prove.
#
# 🔴 Neither one is the same as "it runs on the aircraft". Nothing here has executed a
# single node on real arm64 hardware. P11.5 (Jetson-in-the-loop) is the gate for that and
# it is BLOCKED until a companion computer exists. Do not let a green build here be read
# as flight readiness.
#
# Usage:
#   bash docker/build.sh              # amd64 aircraft + amd64 sim + arm64 DEPS
#   bash docker/build.sh full-arm64   # also compile the workspace for arm64 (slow)
set -o pipefail

cd "$(dirname "$0")/.." || exit 1
MODE=${1:-deps-arm64}
TAG=uav
fails=""

echo "=== buildx builder ==="
docker buildx inspect uavbuilder >/dev/null 2>&1 || \
  docker buildx create --name uavbuilder --use >/dev/null
docker buildx use uavbuilder
docker buildx inspect --bootstrap 2>&1 | grep -E 'Name|Platforms' | head -4

timed() {
  local label=$1; shift
  local start=$SECONDS
  echo
  echo "############ $label ############"
  if "$@"; then
    echo "  --> $label OK in $((SECONDS - start))s"
  else
    echo "  --> $label FAILED after $((SECONDS - start))s"
    fails="$fails $label"
  fi
}

build() {
  local platform=$1 profile=$2 target=$3 tag=$4
  docker buildx build \
    --platform "$platform" \
    --build-arg PROFILE="$profile" \
    ${target:+--target "$target"} \
    -f docker/Dockerfile \
    -t "$tag" \
    --load \
    . 2>&1 | tail -25
}

timed "amd64 / aircraft" build linux/amd64 aircraft "" "$TAG:aircraft-amd64"
timed "amd64 / sim"      build linux/amd64 sim      "" "$TAG:sim-amd64"

# Emulation for the arm64 half. Without binfmt registered, buildx cannot run arm64
# RUN steps at all and reports "exec format error".
echo
echo "=== registering QEMU binfmt for arm64 ==="
docker run --privileged --rm tonistiigi/binfmt --install arm64 2>&1 | tail -3

if [ "$MODE" = "full-arm64" ]; then
  timed "arm64 / aircraft (FULL compile, emulated)" build linux/arm64 aircraft "" "$TAG:aircraft-arm64"
else
  echo
  echo "############ arm64 / aircraft (DEPS only) ############"
  echo "  Proving the arm64 base image and every dependency resolve. Re-run with"
  echo "  'bash docker/build.sh full-arm64' to also compile the workspace."
  # Stop before the colcon step by building only up to the rosdep layer. There is one
  # stage, so this is done by asking for a copy of the tree with the deps installed --
  # the layer cache makes the eventual full build reuse it.
  start=$SECONDS
  if DOCKER_BUILDKIT=1 docker buildx build \
      --platform linux/arm64 \
      --build-arg PROFILE=aircraft \
      -f docker/Dockerfile \
      --target deps \
      -o type=cacheonly \
      . 2>&1 | grep -vE '^#[0-9]+ (sha256|extracting|transferring)' | tail -30; then
    echo "  --> arm64 deps OK in $((SECONDS - start))s"
  else
    echo "  --> arm64 deps FAILED after $((SECONDS - start))s"
    fails="$fails arm64-deps"
  fi
fi

echo
echo "############ SUMMARY ############"
docker image ls "$TAG" --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' 2>/dev/null
echo
if [ -n "$fails" ]; then
  echo "  FAILED:$fails"
  exit 1
fi
echo "  All requested builds succeeded."
echo "  🔴 Reminder: no node has run on real arm64 hardware. P11.5 is the gate for that,"
echo "     and it stays BLOCKED until a companion computer exists."
