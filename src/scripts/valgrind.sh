#!/bin/sh
export LD_LIBRARY_PATH=./build/lib:${LD_LIBRARY_PATH}
echo "(1) Frame test:" > valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/frame/frame_test 2>>valgrind.tmp
export FRAME_OUT=$?
echo "\n(2) Math test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/math/math_test 2>>valgrind.tmp
export MATH_OUT=$?
echo "\n(3) Tire test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/tire/tire_test 2>>valgrind.tmp
export TIRE_OUT=$?
echo "\n(4) Chassis test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/chassis/chassis_test 2>>valgrind.tmp
export CHASSIS_OUT=$?
echo "\n(5) Propagators test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/propagators/propagators_test 2>>valgrind.tmp
export PROPAGATORS_OUT=$?
echo "\n(6) Vehicles test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/vehicles/vehicles_test 2>>valgrind.tmp
export VEHICLES_OUT=$?
echo "\n(7) Actuators test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/actuators/actuators_test 2>>valgrind.tmp
export ACTUATORS_OUT=$?
echo "\n(8) Applications test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/applications/applications_test 2>>valgrind.tmp
export APPLICATIONS_OUT=$?
echo "\n(9) Foundation test:" >> valgrind.tmp
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./build/test/foundation/foundation_test 2>>valgrind.tmp
export FOUNDATION_OUT=$?

echo "\n(10) IO test:" >> valgrind.tmp
cd ./build/test/io/
valgrind --trace-children=yes --leak-check=full --error-exitcode=99 ./io_test 2>>../../../valgrind.tmp
export IO_OUT=$?
cd ../../../

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

if [ $FRAME_OUT -eq 99 ]
then
	echo "\e[1;31m[ FRAME \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ FRAME \t     OK ]\e[0m"
fi

if [ $MATH_OUT -eq 99 ]
then
	echo "\e[1;31m[ MATH \t\t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ MATH \t\t     OK ]\e[0m"
fi

if [ $TIRE_OUT -eq 99 ]
then
	echo "\e[1;31m[ TIRE \t\t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ TIRE \t\t     OK ]\e[0m"
fi

if [ $CHASSIS_OUT -eq 99 ]
then
	echo "\e[1;31m[ CHASSIS \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ CHASSIS \t     OK ]\e[0m"
fi

if [ $PROPAGATORS_OUT -eq 99 ]
then
	echo "\e[1;31m[ PROPAGATORS \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ PROPAGATORS \t     OK ]\e[0m"
fi

if [ $VEHICLES_OUT -eq 99 ]
then
	echo "\e[1;31m[ VEHICLES \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ VEHICLES \t     OK ]\e[0m"
fi

if [ $ACTUATORS_OUT -eq 99 ]
then
	echo "\e[1;31m[ ACTUATORS \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ ACTUATORS \t     OK ]\e[0m"
fi



if [ $APPLICATIONS_OUT -eq 99 ]
then
	echo "\e[1;31m[ APPLICATIONS \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ APPLICATIONS \t     OK ]\e[0m"
fi

if [ $IO_OUT -eq 99 ]
then
	echo "\e[1;31m[ IO \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ IO \t\t     OK ]\e[0m"
fi

if [ $FOUNDATION_OUT -eq 99 ]
then
	echo "\e[1;31m[ FOUNDATION \t NOT OK ]\e[0m"
	export EXIT_CODE=99
else
	echo "\e[32m[ FOUNDATION \t     OK ]\e[0m"
fi


echo
if [ $EXIT_CODE -eq 99 ]
then
	echo "\e[1;31m[  FAILED  ]\e[0m"
else
	echo "\e[32m[  PASSED  ]\e[0m"
fi

exit $EXIT_CODE

