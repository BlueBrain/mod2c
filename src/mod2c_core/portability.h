#ifndef mod2c_portability_h
#define mod2c_portability_h

#ifdef MINGW
// mkdir doesn't accept permission parameter on mingw
#define mkdir(directory, permission) mkdir(directory)

// use strchr instead of index on mingw
#define index strchr

#endif

#endif
