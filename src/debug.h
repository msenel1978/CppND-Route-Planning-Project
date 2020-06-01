#ifndef DEBUG_H
#define DEBUG_H


#ifdef DEBUG_BUILD
#include <iostream>

#define DEBUG(...) do { fprintf(stderr, __VA_ARGS__); } while (0) 
#else
#define DEBUG(...)
#endif

#endif
