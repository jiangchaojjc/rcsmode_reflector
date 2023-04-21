cmake_minimum_required(VERSION 3.5)
PROJECT(rcsmode_reflector)

set(CMAKE_CXX_STANDARD 11)
#add_compile_options(-std=c++11 -g)

# get version
string(TIMESTAMP UVERSION "%y.%m.%d")
add_definitions(-DUVERSION="${UVERSION}")
message(STATUS "version = ${UVERSION}")

add_definitions("-DULIN")

FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()


############################## funcs ##############################
# set include dir
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif ()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach (sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub})
        endif ()
    endforeach ()
endfunction()
# get all matched files
function(get_all_files_recursively dir files match)
    file(GLOB_RECURSE tempfiles ${dir} ${dir}/${match})
    set(${files} ${tempfiles} PARENT_SCOPE)
endfunction()

############################## longto set ##############################
add_definitions("-DULIN")
if (ARM)
    set(PLATFORM armhf)
elseif (AARCH64)
    set(PLATFORM aarch64)
else()
    set(PLATFORM x86_64)
endif()

############################## set pre ##############################
# set(RCSRootDir /home/ucore/clion_ws/rcs_project/rcsalgo/rcsplugin/rcsmode)
# set(RCS_PROXY_PRE /usr/lib/longto)


############################## set pre ##############################
set(RCSRootDir /home/jc/rcsLongto/rcsmode)
#set(RCSRootDir /home/tzy/work/tzy/project/longto_project)

set(RCS_PROXY_PRE /usr/lib/longto)

############################## longto mq lib ##############################
#set(RCSLtLibDir ${RCS_PROXY_PRE})
#include_directories(${RCSLtLibDir}/include)
set(RCSLtLibDir ${RCSRootDir}/longto)
include_sub_directories_recursively(${RCSLtLibDir}/include)
#get_all_files_recursively(${RCSLtLibDir}/include LONGTOIncs *.h*)

############################## longto mq lib ##############################
set(RCSLtLibDir ${RCS_PROXY_PRE})
include_directories(${RCSLtLibDir}/include)

# ############################## rcsbase ##############################
# set(RCSBaseDir ${RCSRootDir}/rcsbase)
# include_sub_directories_recursively(${RCSBaseDir}/inc)

# ############################## rcsmsg ##############################
# set(RCSMsgDir ${RCSRootDir}/rcsmsg)
# include_sub_directories_recursively(${RCSMsgDir}/inc)
# include_directories(${RCSMsgDir}/msgproxy/inc)


############################## rcsbase ##############################
set(RCSBaseDir ${RCSRootDir}/rcsbase)
include_sub_directories_recursively(${RCSBaseDir}/inc)
get_all_files_recursively(${RCSBaseDir}/inc BASEIncs *.h*)
include_directories(${RCSBaseDir}/common/eigen3)

############################## rcsmsg ##############################
set(RCSMsgDir ${RCSRootDir}/rcsmsg)
include_sub_directories_recursively(${RCSMsgDir}/inc)
get_all_files_recursively(${RCSMsgDir}/inc MSGIncs *.h*)
include_directories(${RCSMsgDir}/msgproxy/inc)

############################## rcstask ##############################
set(RCSTaskDir ${RCSRootDir}/rcstask)
include_sub_directories_recursively(${RCSTaskDir}/inc)
# get_all_files_recursively(${RCSTaskDir}/inc TASKIncs *.h*)
# include_directories(${RCSTaskDir}/msgproxy/inc)

# ############################# rcscommon ##############################
set(RCSCommonDir ${RCSRootDir}/rcscommon)
include_sub_directories_recursively(${RCSCommonDir}/inc)

############################## this project ##############################
include_sub_directories_recursively(${PROJECT_SOURCE_DIR}/inc)
get_all_files_recursively(${PROJECT_SOURCE_DIR}/inc Incs *.h)
get_all_files_recursively(${PROJECT_SOURCE_DIR}/src SRC *.c*)
include_directories(${PROJECT_SOURCE_DIR}/common)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/${PLATFORM})

link_directories(${RCSBaseDir}/lib/${PLATFORM})
link_directories(${RCSMsgDir}/lib/${PLATFORM})
link_directories(${RCSTaskDir}/lib/${PLATFORM})
link_directories(${RCSLtLibDir}/lib/${PLATFORM})
link_directories(lib/${PLATFORM})


add_library(${PROJECT_NAME} SHARED ${Incs} ${SRC})
target_link_libraries(${PROJECT_NAME} pthread dl rcsmsg rcsproxy rcsbase rcstask longto_mq_lib longto_file_log  z pthread zmq longto_tinyxml protobuf)

