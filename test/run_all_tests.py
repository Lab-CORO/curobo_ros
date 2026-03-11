#!/usr/bin/env python3
"""
Auto-generated test runner for launch_test
DO NOT EDIT - Changes will be overwritten
"""

import subprocess
import sys
from pathlib import Path


def run_test(test_file: str) -> bool:
    """
    Run a single launch_test test.

    Args:
        test_file: Path to test file

    Returns:
        True if test passed, False otherwise
    """
    test_path = Path(__file__).parent / test_file
    print(f"Running {test_file}...")

    try:
        result = subprocess.run(
            ['launch_test', str(test_path)],
            timeout=60,
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            print(f"  ✓ {test_file} PASSED")
            return True
        else:
            print(f"  ✗ {test_file} FAILED")
            if result.stderr:
                print(f"    Error: {result.stderr[:200]}")
            return False
    except subprocess.TimeoutExpired:
        print(f"  ✗ {test_file} TIMEOUT")
        return False
    except Exception as e:
        print(f"  ✗ {test_file} ERROR: {e}")
        return False


def main():
    """Run all tests and report results."""
    test_files = [
        "test_test_object.py",
    ]

    print("=" * 50)
    print("Running all launch_test tests")
    print("=" * 50)
    print()

    results = []
    for test_file in test_files:
        passed = run_test(test_file)
        results.append((test_file, passed))
        print()

    # Summary
    total = len(results)
    passed = sum(1 for _, p in results if p)
    failed = total - passed

    print("=" * 50)
    print("Test Summary")
    print("=" * 50)
    print(f"Total tests:  {total}")
    print(f"Passed:       {passed}")
    print(f"Failed:       {failed}")

    if failed > 0:
        print()
        print("Failed tests:")
        for test_file, passed in results:
            if not passed:
                print(f"  - {test_file}")
        sys.exit(1)

    print()
    print("All tests passed! ✓")
    sys.exit(0)


if __name__ == '__main__':
    main()
