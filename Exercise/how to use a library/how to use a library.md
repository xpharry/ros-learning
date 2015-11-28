1. in package.xml

	add

	<build_depend>library_name</build_depend>
	<run_depend>library_name</run_depend>

2. in CMakeList.txt

	put the class file name like plane_fitter which is name of the head file and cpp file

	target_link_libraries(plane_fitter_external_main plane_fitter ${catkin_LIBRARIES})
