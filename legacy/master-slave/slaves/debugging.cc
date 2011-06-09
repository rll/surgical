#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "debugging.h"

void debugprintf(const char* format, ...){
    va_list ap;
    va_start(ap,format);
    vprintf(format, ap);
    char logdata[128];

}

