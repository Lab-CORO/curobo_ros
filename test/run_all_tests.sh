#!/bin/bash
# Auto-generated test runner for launch_test
# DO NOT EDIT - Changes will be overwritten

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="${SCRIPT_DIR}"

echo "========================================="
echo "Running all launch_test tests"
echo "========================================="
echo ""

TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
FAILED_TEST_LIST=()


echo "Running test_test_robot_strategy..."
if timeout 60 launch_test "${TEST_DIR}/test_test_robot_strategy.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_robot_strategy PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_robot_strategy FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_robot_strategy")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "Running test_test_trajectory..."
if timeout 60 launch_test "${TEST_DIR}/test_test_trajectory.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_trajectory PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_trajectory FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_trajectory")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "Running test_test_kinematics..."
if timeout 60 launch_test "${TEST_DIR}/test_test_kinematics.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_kinematics PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_kinematics FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_kinematics")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "Running test_test_planners..."
if timeout 60 launch_test "${TEST_DIR}/test_test_planners.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_planners PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_planners FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_planners")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "Running test_test_collision..."
if timeout 60 launch_test "${TEST_DIR}/test_test_collision.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_collision PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_collision FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_collision")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "Running test_test_object..."
if timeout 60 launch_test "${TEST_DIR}/test_test_object.py" > /dev/null 2>&1; then
    echo "  ✓ test_test_object PASSED"
    ((PASSED_TESTS++)) || true
else
    echo "  ✗ test_test_object FAILED"
    ((FAILED_TESTS++)) || true
    FAILED_TEST_LIST+=("test_test_object")
fi
((TOTAL_TESTS++)) || true
echo ""

echo "========================================="
echo "Test Summary"
echo "========================================="
echo "Total tests:  $TOTAL_TESTS"
echo "Passed:       $PASSED_TESTS"
echo "Failed:       $FAILED_TESTS"

if [ $FAILED_TESTS -gt 0 ]; then
    echo ""
    echo "Failed tests:"
    for test in "${FAILED_TEST_LIST[@]}"; do
        echo "  - $test"
    done
    exit 1
fi

echo ""
echo "All tests passed! ✓"
exit 0
