#include <spot/exception.h>
#include <iostream>

static void log::logError(const char* s) {
    freopen( "log.txt", "w", stderr );
    std::cerr << s << std::endl;
}