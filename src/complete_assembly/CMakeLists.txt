cmake_minimum_required(VERSION 3.8)

project(complete_assembly)

find_package(ament_cmake  REQUIRED)



foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir}
		
		)
endforeach()

ament_package()

