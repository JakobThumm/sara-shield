mkdir build && cd build
cmake -DENABLE_COVERAGE=ON ..
ctest && make coverage
# lcov --directory . --capture --output-file coverage.info --exclude "/usr/*" --exclude "*external/*" --exclude "*tests/*"
# genhtml --demangle-cpp -o coverage coverage.info