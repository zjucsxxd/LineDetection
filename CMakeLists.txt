cmake_minimum_required(VERSION 2.8) 
project( line )             #write folders name here instead of "project"
find_package( OpenCV REQUIRED ) 
add_executable( linetuning linetuning.cpp )   # a is executable .  canny. cpp is ur cpp file.
target_link_libraries( linetuning ${OpenCV_LIBS} )
