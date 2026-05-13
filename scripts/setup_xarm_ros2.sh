#!/bin/bash
# ==============================================================================
# Script for cloning and setting up xarm_ros2 with patches for ROS 2 Humble
# ==============================================================================

set -e  # Stop on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Define directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$WS_DIR/src"
XARM_DIR="$SRC_DIR/xarm_ros2"
PATCHES_DIR="$WS_DIR/patches"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║       Setting up xarm_ros2 for ROS 2 Humble                ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if src directory exists
if [ ! -d "$SRC_DIR" ]; then
    echo -e "${YELLOW}⚠️  Creating src directory...${NC}"
    mkdir -p "$SRC_DIR"
fi

# Check if xarm_ros2 already exists
if [ -d "$XARM_DIR" ]; then
    echo -e "${YELLOW}⚠️  xarm_ros2 directory already exists!${NC}"
    read -p "Delete and clone again? [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}🗑️  Removing old directory...${NC}"
        rm -rf "$XARM_DIR"
    else
        echo -e "${BLUE}ℹ️  Skipping clone, applying patches only...${NC}"
        SKIP_CLONE=true
    fi
fi

# Clone xarm_ros2
if [ "$SKIP_CLONE" != "true" ]; then
    echo -e "${GREEN}📦 Cloning xarm_ros2 (humble branch)...${NC}"
    cd "$SRC_DIR"
    git clone -b humble https://github.com/xArm-Developer/xarm_ros2.git
    echo -e "${GREEN}✅ Cloning completed!${NC}"
    echo ""
fi

# Apply patches
echo -e "${GREEN}🔧 Applying patches for Gazebo...${NC}"

PATCH_FILE="$PATCHES_DIR/xarm_ros2_gazebo_humble_fixes.patch"

if [ ! -f "$PATCH_FILE" ]; then
    echo -e "${RED}❌ ERROR: Patch file not found: $PATCH_FILE${NC}"
    exit 1
fi

cd "$XARM_DIR"

# Check if patch is already applied
if git apply --check "$PATCH_FILE" 2>/dev/null; then
    echo -e "${BLUE}   → Applying patch...${NC}"
    git apply "$PATCH_FILE"
    echo -e "${GREEN}✅ Patch successfully applied!${NC}"
else
    echo -e "${YELLOW}⚠️  Patch already applied or files modified${NC}"
    echo -e "${BLUE}ℹ️  Checking changes...${NC}"
    
    # Show change status
    if git diff --quiet xarm_gazebo/launch/xarm7_beside_table_gazebo.launch.py xarm_gazebo/worlds/table.world; then
        echo -e "${YELLOW}⚠️  Files not modified, trying to apply forcefully...${NC}"
        git apply --reject --whitespace=fix "$PATCH_FILE" || true
    else
        echo -e "${GREEN}✅ Files already contain necessary changes!${NC}"
    fi
fi

echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    Applied changes:                        ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}✓${NC} xarm_gazebo/launch/xarm7_beside_table_gazebo.launch.py"
echo -e "  ${BLUE}→${NC} Added ros2_control_plugin parameter for Gazebo"
echo ""
echo -e "${GREEN}✓${NC} xarm_gazebo/worlds/table.world"
echo -e "  ${BLUE}→${NC} Added gazebo_ros_state plugin for ROS 2 services"
echo ""
echo -e "${GREEN}✅ Setup completed successfully!${NC}"
echo ""
echo -e "${YELLOW}📝 Next steps:${NC}"
echo -e "   1. cd $WS_DIR"
echo -e "   2. rosdep install --from-paths src --ignore-src -r -y"
echo -e "   3. colcon build --symlink-install"
echo -e "   4. source install/setup.bash"
echo -e "   5. ros2 launch xarm_gazebo xarm7_beside_table_gazebo.launch.py"
echo ""
