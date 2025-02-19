
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_util/internal.h"

#include "libobsensor/hpp/Pipeline.hpp"

void helloWorld() {
    ob::Pipeline pipe;
}

//
// These static variables only exist to ensure the initializer is called, which registers our camera type.
//
//int _cwipc_dummy_orbbec_initializer = _cwipc_register_capturer("orbbec", OrbbecCapture::count_devices, cwipc_orrbec);
//int _cwipc_dummy_orbbec_offline_initializer = _cwipc_register_capturer("orbbec_offline", nullptr, cwipc_orbbec_offline);
