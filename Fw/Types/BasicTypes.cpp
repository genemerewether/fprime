/**
 * \file
 * \author T. Canham
 * \brief Defines ISF basic types
 *
 * \copyright
 * Copyright 2009-2016, by the California Institute of Technology.
 * ALL RIGHTS RESERVED.  United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 *
 * This software may be subject to U.S. export control laws and
 * regulations.  By accepting this document, the user agrees to comply
 * with all U.S. export laws and regulations.  User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */

#include <Fw/Types/BasicTypes.hpp>

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#endif

// VxWorks and DSPAL don't have strnlen
#if defined __VXWORKS__ || defined BUILD_DSPAL || defined BUILD_TIR5
NATIVE_INT_TYPE strnlen(const char *s, NATIVE_INT_TYPE maxlen) {
    // walk buffer looking for NULL
    for (NATIVE_INT_TYPE index = 0; index < maxlen; index++) {
        if (0 == s[index]) {
            return index+1;
        }
    }
    return maxlen;
}
#endif

#ifdef BUILD_DSPAL
int fputc(int c, FILE *stream)
{
  FARF(ALWAYS, "fputc called with %d", c);
  return c;
}

int fprintf(FILE *stream, const char *format, ...)
{
  FARF(ALWAYS, "fprintf called with format string %s", format);
  return 0;
}
#endif
