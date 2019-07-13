
set(CMAKE_VERBOSE_MAKEFILE on)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})


#set(CPPSRC ff-node.cpp)
#set(CPPHDR relax.h search.h)

set (CMAKE_C_FLAGS "--std=gnu99 -Wall -g -O6 ${CMAKE_C_FLAGS}")


BISON_TARGET(ScanOps scan-ops_pddl.y ${PROJECT_BINARY_DIR}/src/ff/scan-ops_pddl.tab.c COMPILE_FLAGS "-pops_pddl -bscan-ops_pddl")
FLEX_TARGET(LexOps lex-ops_pddl.l ${PROJECT_BINARY_DIR}/src/ff/lex.ops_pddl.c COMPILE_FLAGS "-Pops_pddl")
ADD_FLEX_BISON_DEPENDENCY(LexOps ScanOps)


BISON_TARGET(ScanFct scan-fct_pddl.y ${PROJECT_BINARY_DIR}/src/ff/scan-fct_pddl.tab.c COMPILE_FLAGS "-pfct_pddl -bscan-fct_pddl")
FLEX_TARGET(LexFct lex-fct_pddl.l ${PROJECT_BINARY_DIR}/src/ff/lex.fct_pddl.c COMPILE_FLAGS "-Pfct_pddl")
ADD_FLEX_BISON_DEPENDENCY(LexFct ScanFct )



set( FFSRC  memory.c output.c parse.c inst_pre.c inst_easy.c inst_hard.c inst_final.c orderings.c relax.c search.c main.c)
set( HDR ff.h inst_final.h inst_hard.h inst_pre.h memory.h orderings.h output.h parse.h relax.h search.h)




set( PDDL_PARSER_SRC ${PROJECT_BINARY_DIR}/scan-fct_pddl.tab.c
                     ${PROJECT_BINARY_DIR}/scan-ops_pddl.tab.c
                     #${PROJECT_BINARY_DIR}/scan-probname.tab.c
                     ${PROJECT_BINARY_DIR}/lex.fct_pddl.c
                     ${PROJECT_BINARY_DIR}/lex.ops_pddl.c  )

set( PDDL_PARSER_OBJ_C scan-fct_pddl.tab.c scan-ops_pddl.tab.c)


#${PDDL_PARSER_OBJ_C} )

add_library( fflib SHARED ${FFSRC} ${BISON_ScanFct_OUTPUTS} ${CPPSRC}
 #   ${FLEX_LexOps_OUTPUTS}
 #   ${FLEX_LexFct_OUTPUTS}
    ${HDR}
    ${CPPHDR}
    ${BISON_ScanOps_OUTPUTS} )
target_link_libraries( fflib ${catkin_LIBRARIES} ${FLEX_LIBRARIES} ${BISON_LIBRARIES})
add_dependencies(fflib ${catkin_EXPORTED_TARGETS} )


#add_executable( ff ${FFSRC} ${BISON_ScanFct_OUTPUTS}
#                 #  ${PROJECT_SOURCE_DIR}/src/mp/mp.h
#                 #  ${PROJECT_SOURCE_DIR}/src/mp/mp.cpp
#                     #${FLEX_LexOps_OUTPUTS}
#                     #${FLEX_LexFct_OUTPUTS}
#                     ${BISON_ScanOps_OUTPUTS}  )
# target_link_libraries( ff fflib ${catkin_LIBRARIES} )
# add_dependencies(ff ${catkin_EXPORTED_TARGETS})
