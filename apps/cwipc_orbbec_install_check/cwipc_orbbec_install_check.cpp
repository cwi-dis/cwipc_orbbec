#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>
#include "cwipc_util/api.h"
#include "cwipc_orbbec/api.h"

int main(int argc, char** argv) {
    char *error = NULL;
    cwipc_source *generator = cwipc_orbbec("auto", &error, CWIPC_API_VERSION);
    if (generator == NULL) {
        char* expectedError = strstr(error, "no orbbec cameras found");
        if (expectedError == NULL) {
            expectedError = strstr(error, "cwipc_orbbec() failed");
        }
        if (expectedError == NULL) {
            // Any other error is unexpected.
            std::cerr << argv[0] << ": Error: " << error << std::endl;
            return 1;
        }
    }
    else {
        generator->free();
    }
    return 0;
}

