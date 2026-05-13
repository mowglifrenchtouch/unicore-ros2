#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Multi-arch build with buildx + ccache + registry cache
# =============================================================================

TAG="${1:-ghcr.io/mowglifrenchtouch/unicore-ros2:mowgli}"
PLATFORMS="${2:-linux/amd64,linux/arm64}"
BUILDER="mowgli-builder"
CACHE_IMAGE="${TAG}-mowgli"

echo "TAG        : $TAG"
echo "PLATFORMS  : $PLATFORMS"
echo "BUILDER    : $BUILDER"
echo "CACHE IMG  : $CACHE_IMAGE"

# -----------------------------------------------------------------------------
# Create or reuse builder
# -----------------------------------------------------------------------------
if ! docker buildx inspect "$BUILDER" >/dev/null 2>&1; then
  docker buildx create \
    --name "$BUILDER" \
    --driver docker-container \
    --use \
    --bootstrap
else
  docker buildx use "$BUILDER"
fi

# -----------------------------------------------------------------------------
# Enable QEMU for cross-build (important for ARM)
# -----------------------------------------------------------------------------
docker run --rm --privileged tonistiigi/binfmt --install all >/dev/null 2>&1 || true

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
docker buildx build \
  --platform "$PLATFORMS" \
  --tag "$TAG" \
  --build-arg CCACHE_DIR=/root/.ccache \
  --build-arg BUILDKIT_INLINE_CACHE=1 \
  --cache-from "type=registry,ref=$CACHE_IMAGE" \
  --cache-to "type=registry,ref=$CACHE_IMAGE,mode=max" \
  --push \
  .

echo "Build terminé"