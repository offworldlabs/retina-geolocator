#!/bin/bash
# Run all ADS-B feature tests
# Usage: ./run_all_tests.sh

set -e  # Exit on first failure

echo "============================================================"
echo "Running Complete ADS-B Test Suite"
echo "============================================================"
echo

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

run_test() {
    local test_file=$1
    local test_name=$2

    echo "-----------------------------------------------------------"
    echo "Running: $test_name"
    echo "-----------------------------------------------------------"

    if python3 "$test_file"; then
        echo -e "${GREEN}✓ $test_name PASSED${NC}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo -e "${RED}✗ $test_name FAILED${NC}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi

    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    echo
}

# Run all test files
run_test "test_parse_adsb.py" "ADS-B Metadata Parsing (Issue #2)"
run_test "test_adsb_initial_guess.py" "ADS-B Initial Guess Generator (Issue #3)"
run_test "test_dual_mode_selection.py" "Dual-Mode Selection (Issue #4)"
run_test "test_adsb_config.py" "ADS-B Configuration (Issue #5)"
run_test "test_enhanced_output.py" "Enhanced Output Metadata (Issue #6)"
run_test "test_adsb_features.py" "Comprehensive ADS-B Features (Issue #7)"

# Summary
echo "============================================================"
echo "Test Suite Summary"
echo "============================================================"
echo "Total test files: $TOTAL_TESTS"
echo -e "Passed: ${GREEN}$PASSED_TESTS${NC}"
echo -e "Failed: ${RED}$FAILED_TESTS${NC}"
echo

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
    echo "============================================================"
    exit 0
else
    echo -e "${RED}❌ SOME TESTS FAILED${NC}"
    echo "============================================================"
    exit 1
fi
