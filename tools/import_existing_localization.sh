#!/usr/bin/env bash
set -euo pipefail

# Copy the already-validated localization packages WITHOUT editing their source.
# Default source matches the previous project workspace.
SOURCE_WS="${1:-$HOME/ros2_icp_ekf_localization}"
TARGET_WS="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

SOURCE_SRC="$SOURCE_WS/src"
TARGET_SRC="$TARGET_WS/src"

PACKAGES=(
  robot_description
  icp_localization
  ekf_localization
)

if [[ ! -d "$SOURCE_SRC" ]]; then
  echo "[ERROR] Source workspace not found: $SOURCE_SRC" >&2
  exit 1
fi

mkdir -p "$TARGET_SRC"

for pkg in "${PACKAGES[@]}"; do
  if [[ ! -f "$SOURCE_SRC/$pkg/package.xml" ]]; then
    echo "[ERROR] Missing package: $SOURCE_SRC/$pkg" >&2
    exit 1
  fi

  rm -rf "$TARGET_SRC/$pkg"
  cp -a "$SOURCE_SRC/$pkg" "$TARGET_SRC/$pkg"
  echo "[OK] Copied unchanged: $pkg"
done

echo
echo "Localization packages imported from:"
echo "  $SOURCE_WS"
echo "into:"
echo "  $TARGET_WS"
echo
echo "Next:"
echo "  cd $TARGET_WS"
echo "  source /opt/ros/humble/setup.bash"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install"
