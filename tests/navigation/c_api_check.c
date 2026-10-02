#include "navigation.h"
// Compile the public header as C, as required by the H7 application boundary.
size_t nav_c_header_check(void) { return sizeof(NavConfig)+sizeof(NavOutput)+sizeof(NavStats); }
