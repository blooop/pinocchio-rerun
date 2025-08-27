include(${PROJECT_SOURCE_DIR}/cmake-module/stubs.cmake)
include(${PROJECT_SOURCE_DIR}/cmake-module/python.cmake)

find_package(eigenpy REQUIRED)

file(GLOB py_SRC CONFIGURE_DEPENDS "*.cpp")

set(PYLIB "${LIB_NAME}_pywrap")
python3_add_library(${PYLIB} MODULE WITH_SOABI ${py_SRC})

target_link_libraries(${PYLIB} PUBLIC ${LIB_NAME} eigenpy::eigenpy)
target_compile_definitions(${PYLIB} PRIVATE MODULE_NAME=${PYLIB})

set(PYLIB_INSTALL_DIR "${PYTHON_SITELIB}/${LIB_NAME}")
GET_RELATIVE_RPATH("${CMAKE_INSTALL_PREFIX}/${PYLIB_INSTALL_DIR}" PYLIB_INSTALL_RPATH)

set_target_properties(
  ${PYLIB}
  PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${LIB_NAME}
    INSTALL_RPATH "${PYLIB_INSTALL_RPATH}"
)

# Ensure the runtime python package directory (build tree) contains __init__.py alongside the extension
# so that `from .pinocchio_rerun_pywrap import *` resolves within the same directory rather than the
# source tree (which otherwise masks the built extension causing AttributeError in examples).
add_custom_command(
  TARGET ${PYLIB} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/pinocchio_rerun/__init__.py
          $<TARGET_FILE_DIR:${PYLIB}>/__init__.py
  VERBATIM
)

install(TARGETS ${PYLIB} DESTINATION ${PYLIB_INSTALL_DIR})

PYTHON_INSTALL_ON_SITE(${LIB_NAME} __init__.py)

# Generate stubs

if(GENERATE_PYTHON_STUBS)
  LOAD_STUBGEN()
  GENERATE_STUBS(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${LIB_NAME}
    ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}
    ${PYLIB}
  )
endif(GENERATE_PYTHON_STUBS)
