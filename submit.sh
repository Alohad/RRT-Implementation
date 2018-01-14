#!/bin/bash

echo "Trying to compile assignment6.cpp ..."
clang++ -std=c++11 assignment8.cpp assignment8_testing.cpp -lpthread -lgtest -lX11 -o run_tests > /dev/null 2>&1
error=$?
if [ $error = 0 ]; then
	tar -czf assignment8.tar.gz assignment8.cpp assignment8.h assignment8_testing.cpp
	echo "Success! You may now submit assignment8.tar.gz to Moodle."
else
	echo "ERROR: Compilation failed, you are not ready to submit."
fi
