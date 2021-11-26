#!/bin/sh
TEST=$1
TEST_UPPER=$(echo $TEST | tr '[:lower:]' '[:upper:]')
TEST_CAMEL=$(echo $TEST | sed 's/\([a-z]\)\([a-zA-Z0-9]*\)/\u\1\2/g')

export LD_LIBRARY_PATH=./:${LD_LIBRARY_PATH}
echo "${TEST_CAMEL} test" > valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./${TEST}_test --valgrind 2>>valgrind.tmp
export VALGRIND_OUT=$?

echo
echo
echo "==============================================="
echo "================ VALGRIND OUTPUT =============="
echo "==============================================="
echo 
echo
cat valgrind.tmp
\rm valgrind.tmp

echo
echo
echo "======================================="
echo "================ SUMMARY =============="
echo "======================================="
echo 
echo


export EXIT_CODE=0

if [ $VALGRIND_OUT -eq 99 ]
then
	echo "\e[1;31m[ ${TEST_UPPER} NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ ${TEST_UPPER} OK ]\e[0m"
fi

if [ $EXIT_CODE -eq 99 ]
then
	echo "\e[1;31m[  FAILED  ]\e[0m"
else
	echo "\e[32m[  PASSED  ]\e[0m"
fi

exit $EXIT_CODE
