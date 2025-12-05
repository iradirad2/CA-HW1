#!/bin/bash
bad_tests_count=0
for filename in CompArchHw1Tests/test*.trc; do
    test_num=$(echo "$filename" | cut -d'.' -f1)
    dos2unix -q "$filename" 
    ./bp_main "$filename" > "${test_num}Yours.out"
    echo "Comparing $test_num.out" > /dev/null
    diff "${test_num}.out" "${test_num}Yours.out" > /dev/null
    if [ $? -ne 0 ]; then
	((bad_tests_count++))
        echo "Difference found for ${test_num}! Total bad tests: ${bad_tests_count}"
    fi
done
