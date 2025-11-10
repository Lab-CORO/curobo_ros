#!/bin/bash
# Script pour exécuter les tests d'intégration CuRobo ROS

set -e

# Couleurs pour output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  CuRobo ROS Integration Tests${NC}"
echo -e "${BLUE}======================================${NC}\n"

# Vérifier que ROS 2 est sourcé
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS 2 not sourced!${NC}"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 distro: $ROS_DISTRO${NC}"

# Vérifier que workspace est sourcé
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo -e "${YELLOW}Warning: Workspace not sourced${NC}"
    echo "Run: source install/setup.bash"
fi

# Vérifier que pytest est installé
if ! command -v pytest &> /dev/null; then
    echo -e "${RED}Error: pytest not installed${NC}"
    echo "Install with: pip install -r requirements_test.txt"
    exit 1
fi

echo -e "${GREEN}✓ pytest installed${NC}\n"

# Parse arguments
MODE="all"
MARKERS=""
VERBOSE="-v"
EXTRA_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            MODE="quick"
            MARKERS="-m 'not slow and not hardware and not requires_rosbag'"
            shift
            ;;
        --perception)
            MODE="perception"
            MARKERS="-m perception"
            shift
            ;;
        --stress)
            MODE="stress"
            MARKERS="-m 'stress or robustness'"
            shift
            ;;
        --planning)
            MODE="planning"
            EXTRA_ARGS="test/integration/test_e2e_planning.py"
            shift
            ;;
        --no-hardware)
            MARKERS="$MARKERS -m 'not hardware'"
            shift
            ;;
        --no-rosbag)
            MARKERS="$MARKERS -m 'not requires_rosbag'"
            shift
            ;;
        -vv|--very-verbose)
            VERBOSE="-vv -s"
            shift
            ;;
        --html)
            EXTRA_ARGS="$EXTRA_ARGS --html=test_report.html --self-contained-html"
            shift
            ;;
        --cov)
            EXTRA_ARGS="$EXTRA_ARGS --cov=curobo_ros --cov-report=html --cov-report=term"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --quick         Run quick tests only (no slow, hardware, rosbag tests)"
            echo "  --perception    Run perception tests only"
            echo "  --stress        Run stress and robustness tests only"
            echo "  --planning      Run planning tests only"
            echo "  --no-hardware   Skip hardware tests"
            echo "  --no-rosbag     Skip tests requiring rosbags"
            echo "  -vv             Very verbose output"
            echo "  --html          Generate HTML report"
            echo "  --cov           Generate coverage report"
            echo "  -h, --help      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --quick           # Quick smoke test"
            echo "  $0 --perception      # Test perception pipeline"
            echo "  $0 --stress          # Stress and robustness tests"
            echo "  $0 --html --cov      # Generate reports"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Afficher configuration
echo -e "${BLUE}Test Configuration:${NC}"
echo "  Mode: $MODE"
if [ -n "$MARKERS" ]; then
    echo "  Markers: $MARKERS"
fi
echo -e "  Verbose: $VERBOSE\n"

# Démarrer les nœuds ROS nécessaires (si pas déjà lancés)
# Note: Ceci devrait être fait dans un terminal séparé ou via launch file
echo -e "${YELLOW}Note: Make sure required ROS nodes are running:${NC}"
echo "  - curobo_gen_traj"
echo "  - curobo_ik"
echo "  - curobo_fk (optionnel)"
echo "  - robot_segmentation (pour tests perception)"
echo ""

read -p "Press Enter to continue with tests, or Ctrl+C to cancel..."

# Exécuter les tests
echo -e "\n${BLUE}Running tests...${NC}\n"

if [ -z "$EXTRA_ARGS" ]; then
    EXTRA_ARGS="test/integration/"
fi

# Commande pytest complète
CMD="pytest $EXTRA_ARGS $VERBOSE $MARKERS"

echo -e "${BLUE}Command: $CMD${NC}\n"

# Exécuter
if eval $CMD; then
    echo -e "\n${GREEN}======================================${NC}"
    echo -e "${GREEN}  All tests passed! ✓${NC}"
    echo -e "${GREEN}======================================${NC}"
    exit 0
else
    echo -e "\n${RED}======================================${NC}"
    echo -e "${RED}  Some tests failed! ✗${NC}"
    echo -e "${RED}======================================${NC}"
    exit 1
fi
