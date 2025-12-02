#!/usr/bin/env python3
"""
Quick test to verify Strategy Pattern implementation.

Run with: python3 test_strategy_pattern.py
"""

def test_imports():
    """Test that all modules can be imported."""
    print("Testing imports...")

    try:
        from curobo_ros.planners import (
            TrajectoryPlanner,
            PlannerResult,
            ClassicPlanner,
            MPCPlanner,
            PlannerFactory,
            PlannerManager
        )
        print("✅ All imports successful")
        return True
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False


def test_factory():
    """Test PlannerFactory functionality."""
    print("\nTesting PlannerFactory...")

    try:
        from curobo_ros.planners import PlannerFactory

        # Test get_available_planners
        available = PlannerFactory.get_available_planners()
        print(f"  Available planners: {available}")

        if 'classic' in available and 'mpc' in available:
            print("✅ Factory registry correct")
            return True
        else:
            print("❌ Missing planners in registry")
            return False

    except Exception as e:
        print(f"❌ Factory test error: {e}")
        return False


def test_planner_result():
    """Test PlannerResult dataclass."""
    print("\nTesting PlannerResult...")

    try:
        from curobo_ros.planners import PlannerResult

        # Create result
        result = PlannerResult(
            success=True,
            message="Test successful",
            trajectory=None,
            metadata={'test': 123}
        )

        assert result.success == True
        assert result.message == "Test successful"
        assert result.metadata['test'] == 123

        print("✅ PlannerResult works correctly")
        return True

    except Exception as e:
        print(f"❌ PlannerResult test error: {e}")
        return False


def test_execution_mode():
    """Test ExecutionMode enum."""
    print("\nTesting ExecutionMode...")

    try:
        from curobo_ros.planners.trajectory_planner import ExecutionMode

        assert ExecutionMode.OPEN_LOOP.value == "open_loop"
        assert ExecutionMode.CLOSED_LOOP.value == "closed_loop"

        print("✅ ExecutionMode enum correct")
        return True

    except Exception as e:
        print(f"❌ ExecutionMode test error: {e}")
        return False


def test_class_hierarchy():
    """Test that planners inherit correctly."""
    print("\nTesting class hierarchy...")

    try:
        from curobo_ros.planners import TrajectoryPlanner, ClassicPlanner, MPCPlanner

        # Check inheritance
        assert issubclass(ClassicPlanner, TrajectoryPlanner)
        assert issubclass(MPCPlanner, TrajectoryPlanner)

        # Check abstract methods are implemented
        required_methods = ['plan', 'execute', 'get_planner_name', '_get_execution_mode']

        for planner_class in [ClassicPlanner, MPCPlanner]:
            for method in required_methods:
                assert hasattr(planner_class, method), f"{planner_class.__name__} missing {method}"

        print("✅ Class hierarchy correct")
        return True

    except Exception as e:
        print(f"❌ Hierarchy test error: {e}")
        return False


def test_planner_registration():
    """Test custom planner registration."""
    print("\nTesting custom planner registration...")

    try:
        from curobo_ros.planners import PlannerFactory, TrajectoryPlanner, ExecutionMode, PlannerResult

        class TestPlanner(TrajectoryPlanner):
            def _get_execution_mode(self):
                return ExecutionMode.OPEN_LOOP

            def get_planner_name(self):
                return "Test Planner"

            def plan(self, start, goal, config):
                return PlannerResult(success=True, message="Test")

            def execute(self, robot_context, goal_handle=None):
                return True

        # Register
        PlannerFactory.register_planner('test_custom', TestPlanner)

        # Verify registration
        assert 'test_custom' in PlannerFactory._PLANNER_REGISTRY

        print("✅ Custom planner registration works")
        return True

    except Exception as e:
        print(f"❌ Registration test error: {e}")
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("Strategy Pattern Implementation Tests")
    print("=" * 60)

    tests = [
        test_imports,
        test_factory,
        test_planner_result,
        test_execution_mode,
        test_class_hierarchy,
        test_planner_registration,
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ Test crashed: {e}")
            results.append(False)

    print("\n" + "=" * 60)
    print(f"Results: {sum(results)}/{len(results)} tests passed")
    print("=" * 60)

    if all(results):
        print("\n🎉 All tests passed! Strategy Pattern is ready to use.")
        print("\nNext steps:")
        print("1. Test with: ros2 run curobo_ros unified_planner")
        print("2. Read: ARCHITECTURE.md for details")
        print("3. Check: examples/planner_usage_example.py")
        return 0
    else:
        print("\n⚠️  Some tests failed. Please check the errors above.")
        return 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
