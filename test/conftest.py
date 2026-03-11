"""
Pytest configuration for launch_test integration.
Allows running launch_test tests via pytest/colcon test.
"""

import subprocess
import pytest
from pathlib import Path


def pytest_collection_modifyitems(config, items):
    """
    Mark all test items as launch_test items.
    This prevents pytest from trying to collect launch_test tests directly.
    """
    # Remove all collected items - we'll run them via launch_test
    items.clear()


def pytest_sessionstart(session):
    """
    Run launch_test tests at the start of pytest session.
    """
    test_dir = Path(__file__).parent
    test_files = sorted(test_dir.glob('test_*.py'))

    # Filter out the runner scripts
    test_files = [f for f in test_files if f.stem not in ['test_runner', 'run_all_tests']]

    if not test_files:
        return

    print("\n" + "=" * 70)
    print("Running launch_test tests")
    print("=" * 70)

    failed_tests = []

    for test_file in test_files:
        print(f"\nRunning {test_file.name}...")

        try:
            result = subprocess.run(
                ['launch_test', str(test_file)],
                timeout=60,
                capture_output=True,
                text=True
            )

            if result.returncode == 0:
                print(f"  ✓ {test_file.name} PASSED")
            else:
                print(f"  ✗ {test_file.name} FAILED")
                failed_tests.append(test_file.name)

        except subprocess.TimeoutExpired:
            print(f"  ✗ {test_file.name} TIMEOUT")
            failed_tests.append(test_file.name)
        except Exception as e:
            print(f"  ✗ {test_file.name} ERROR: {e}")
            failed_tests.append(test_file.name)

    print("\n" + "=" * 70)

    if failed_tests:
        print(f"\nFailed tests: {', '.join(failed_tests)}")
        pytest.exit("launch_test tests failed", returncode=1)
    else:
        print("\nAll launch_test tests passed! ✓")
        pytest.exit("All tests passed", returncode=0)
