add_library(QpSolverCollection
  QpSolverCollection.cpp
  QpSolverQld.cpp
  QpSolverQuadprog.cpp
  QpSolverLssol.cpp
  QpSolverJrlqp.cpp
  QpSolverQpoases.cpp
  QpSolverOsqp.cpp
  QpSolverNasoq.cpp
  QpSolverHpipm.cpp
  QpSolverProxqp.cpp
  QpSolverQpmad.cpp
)

target_compile_features(QpSolverCollection PUBLIC cxx_std_17)

# This warning is included in the header of qpOASES
set_source_files_properties(QpSolverQpoases.cpp PROPERTIES
  COMPILE_FLAGS -Wno-unused-parameter)

if(DEFINED CATKIN_DEVEL_PREFIX)
  target_include_directories(QpSolverCollection PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(QpSolverCollection PUBLIC
    ${catkin_LIBRARIES}
  )
else()
  target_link_libraries(QpSolverCollection PUBLIC Eigen3::Eigen)

  target_compile_definitions(QpSolverCollection PUBLIC QP_SOLVER_COLLECTION_STANDALONE)

  install(TARGETS QpSolverCollection
          EXPORT ${PROJECT_NAME}
          LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  )

  install(FILES
      "${PROJECT_SOURCE_DIR}/include/qp_solver_collection/QpSolverCollection.h"
      "${QP_SOLVER_OPTIONS_HEADER_FILE}"
    DESTINATION
      "${CMAKE_INSTALL_INCLUDEDIR}/qp_solver_collection"
  )
endif()

target_include_directories(QpSolverCollection PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>)

if(${ENABLE_QLD})
  target_link_libraries(QpSolverCollection PRIVATE
    eigen-qld::eigen-qld)
endif()
if(${ENABLE_QUADPROG})
  target_link_libraries(QpSolverCollection PRIVATE
    eigen-quadprog::eigen-quadprog)
endif()
if(${ENABLE_LSSOL})
  target_link_libraries(QpSolverCollection PRIVATE
    eigen-lssol::eigen-lssol)
endif()
if(${ENABLE_JRLQP})
  target_link_libraries(QpSolverCollection PRIVATE
    jrl-qp::jrl-qp)
endif()
if(${ENABLE_QPOASES})
  target_include_directories(QpSolverCollection PRIVATE
    ${QPOASES_INCLUDE_DIR})
  if(COMMAND target_link_directories)
    target_link_directories(QpSolverCollection PRIVATE
      ${QPOASES_LIBRARY_DIR})
  endif()
  target_link_libraries(QpSolverCollection PRIVATE
    qpOASES)
endif()
if(${ENABLE_OSQP})
  target_link_libraries(QpSolverCollection PRIVATE
    OsqpEigen::OsqpEigen)
endif()
if(${ENABLE_NASOQ})
  target_link_libraries(QpSolverCollection PRIVATE
    nasoq)
endif()
if(${ENABLE_HPIPM})
  target_link_libraries(QpSolverCollection PRIVATE
    blasfeo hpipm)
endif()
if(${ENABLE_PROXQP})
  target_link_libraries(QpSolverCollection PRIVATE
    proxsuite::proxsuite)
endif()
if(${ENABLE_QPMAD})
  target_include_directories(QpSolverCollection PRIVATE
    ${qpmad_INCLUDE_DIRS})
endif()
