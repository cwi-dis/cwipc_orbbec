#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>

#include "cwipc_util/api.h"
#include "cwipc_orbbec/api.h"

#define DEBUG_AUXDATA
#define DEBUG_CONFIG

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " count directory [configfile]" << std::endl;
        std::cerr << "Creates COUNT pointclouds from an Orbbec camera and stores the PLY files in the given DIRECTORY" << std::endl;
        std::cerr << "If directory is - then drop the pointclouds on the floor" << std::endl;

        return 2;
    }

    int countWanted = atoi(argv[1]);
    char filename[500];
    char *error = NULL;
    cwipc_tiledsource *generator;
    char *outputdir = argv[2];
    char *configFile = NULL;

    if (argc == 4) {
        configFile = argv[3];
    }

    generator = cwipc_orbbec(configFile, &error, CWIPC_API_VERSION);

    if (generator == NULL) {
        std::cerr << argv[0] << ": creating orbbec grabber failed: " << error << std::endl;

        if (getenv("CWIPC_ORBBEC_TESTING") != NULL) {
            return 0; // No failure while running tests, so we can at least test linking, etc.
        }

        return 1;
    }

    if (error) {
        std::cerr << argv[0] << ": warning while creating orbbec grabber: " << error << std::endl;
    }

#ifdef DEBUG_AUXDATA
    generator->request_auxiliary_data("rgb");
    generator->request_auxiliary_data("depth");
    generator->request_auxiliary_data("skeleton");
#endif

#ifdef DEBUG_CONFIG
    size_t configSize = generator->get_config(nullptr, 0);
    char* configBuf = (char *)malloc(configSize + 1);
    memset(configBuf, 0, configSize + 1);
    generator->get_config(configBuf, configSize);

    std::cerr << "cameraconfig as json:\n=================\n" << configBuf << "\n======================\n";
#endif

    int ok = 0;
    int framenum = 0;
    int nGrabbedSuccessfully = 0;

    while (!generator->eof()) {
        if (countWanted != 0 && framenum >= countWanted) {
            break;
        }

        cwipc* pc = NULL;
        pc = generator->get();

        if (pc == NULL) {
            error = (char*)"grabber returned NULL pointcloud";
            ok = -1;
            break;
        }
        if (pc->count() <= 0) {
            std::cerr << argv[0] << ": warning: empty pointcloud, grabbing again" << std::endl;
            pc->free();
            continue;
        }
        std::cerr << argv[0] << ": got pc with " << std::to_string(pc->count()) << " points" << std::endl;

#ifdef DEBUG_AUXDATA
        cwipc_auxiliary_data* ap = pc->access_auxiliary_data();

        if (ap == nullptr) {
            std::cerr << argv[0] << ": access_auxiliary_data: returned null pointer" << std::endl;
        } else {
            std::cerr << argv[0] << ": auxdata: " << ap->count() << " items:" << std::endl;

            for (int i = 0; i < ap->count(); i++) {
                void* ptr = ap->pointer(i);
                std::cerr << argv[0] << "auxdata: item " << i << " name=" << ap->name(i) << ", size=" << (int)ap->size(i) << ", descr=" << ap->description(i) << ", pointer=0x" << std::hex << (uint64_t)ptr << std::dec << std::endl;
            }
        }
#endif
        framenum++;

        if (strcmp(outputdir, "-") != 0) {
            snprintf(filename, sizeof(filename), "%s/pointcloud-%" PRIu64 ".ply", outputdir, pc->timestamp());
            std::cout << "-> Writing frame " << framenum << " with " << pc->count() << " points to "<< filename << std::endl;
            ok = cwipc_write(filename, pc, &error);
        } else {
            std::cout << "-> Dropping frame " << framenum << " with " << pc->count() << " points" << std::endl;
        }

        pc->free();
        nGrabbedSuccessfully++;
    }

    generator->free();

    if (ok < 0) {
        std::cerr << "Error: " << error << std::endl;
        return 1;
    }

    if (countWanted != 0 && nGrabbedSuccessfully != countWanted) {
        std::cerr << "cwipc_k4aoffline: Wanted " << countWanted << " pointclouds but got only " << nGrabbedSuccessfully << std::endl;
        return 1;
    }

    return 0;
}

