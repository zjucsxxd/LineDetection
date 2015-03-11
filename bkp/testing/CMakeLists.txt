cmake_minimum_required(VERSION 2.8) 
project( testing )             #write folders name here instead of "project"
find_package( OpenCV REQUIRED ) 
add_executable( testing testing.cpp )   # a is executable .  canny. cpp is ur cpp file.
target_link_libraries( testing ${OpenCV_LIBS} )
