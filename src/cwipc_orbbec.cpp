
#if defined(WIN32) || defined(_WIN32)
#define _CWIPC_KINECT_EXPORT __declspec(dllexport)
#endif

#include <inttypes.h>

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_util/internal.h"
#include "cwipc_orbbec/api.h"

#include "OrbbecCapture.hpp"

class cwipc_source_orbbec_impl : public cwipc_tiledsource {
protected:
    OrbbecCapture *grabber;

    cwipc_source_orbbec_impl(OrbbecCapture *obj) : grabber(obj) {
    }

public:
    cwipc_source_orbbec_impl(const char *configFilename=NULL) : grabber(OrbbecCapture::factory()) {
        grabber->configReload(configFilename);
    }

    ~cwipc_source_orbbec_impl() {
        free();
    }

    void free() {
        delete grabber;
        grabber = 0;
    }

    bool eof() {
        return grabber->eof;
    }

    bool available(bool wait) {
        if (grabber == 0) {
            return false;
        }

        return grabber->pointcouldAvailable(wait);
    }

    cwipc* get() {
        if (grabber == 0) {
            return 0;
        }

        return grabber->getPointcloud();
    }

    int maxtile() {
        if (grabber == 0) {
            return 0;
        }

        int n = grabber->configuration.all_camera_configs.size();

        if (n <= 1) {
            return n;
        }

        return 1 << n;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo* tileinfo) = 0;
};

class cwipc_source_orbbec_offline_impl : public cwipc_tiledsource {
protected:
public:
    cwipc_source_orbbec_offline_impl(const char* configFilename = NULL) {
    }

    ~cwipc_source_orbbec_offline_impl() {
    }
};

//
// C-compatible entry points
//

cwipc_tiledsource* cwipc_orbbec(const char *configFilename, char **errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_kinect: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }

        return NULL;
    }
    if (errorMessage) {
        static char *tmp = (char *)"cwipc_orbbec() not yet implemented";
        *errorMessage = tmp;
    }
    return NULL;
}

cwipc_tiledsource* cwipc_orbbec_offline(const char* configFilename, char** errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_k4aoffline: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }

        return NULL;
    }
    if (errorMessage) {
        static char *tmp = (char *)"cwipc_orbbec_offline() not yet implemented";
        *errorMessage = tmp;
    }

    return NULL;
}

//
// These static variables only exist to ensure the initializer is called, which registers our camera type.
//
int _cwipc_dummy_orbbec_initializer = _cwipc_register_capturer("orbbec", OrbbecCapture::countDevices, cwipc_orbbec);
int _cwipc_dummy_orbbec_offline_initializer = _cwipc_register_capturer("orbbec_offline", nullptr, cwipc_orbbec_offline);
