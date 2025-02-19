set(OrbbecSDK_ROOT_DIR $ENV{ORBBEC_SDK_PATH}/SDK)
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)

include_directories(${OrbbecSDK_INCLUDE_DIR})
link_directories(${OrbbecSDK_LIBRARY_DIRS})

if(EXISTS ${OrbbecSDK_ROOT_DIR})
  set(OrbbecSDK_FOUND YES)
else()
  set(OrbbecSDK_FOUND NO)
endif()
