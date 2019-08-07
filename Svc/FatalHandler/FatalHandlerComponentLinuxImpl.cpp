// ====================================================================== 
// \title  FatalHandlerImpl.cpp
// \author tcanham
// \brief  cpp file for FatalHandler component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 

#include <stdlib.h>
#include <signal.h>
#include <Os/Log.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#ifdef BUILD_DSPAL
extern volatile bool terminate;
#endif

#ifndef BUILD_DSPAL
#include <execinfo.h>
#endif // BUILD_DSPAL
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <Os/Task.hpp>

namespace Svc
{

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void FatalHandlerComponentImpl::FatalReceive_handler(
            const NATIVE_INT_TYPE portNum,
            FwEventIdType Id)
    {

        int j, nptrs;
        void *buffer[100];
        char **strings;
#ifndef BUILD_DSPAL
        nptrs = backtrace(buffer, 100);
        //printf("backtrace() returned %d addresses\n", nptrs);

        /* The call backtrace_symbols_fd(buffer, nptrs, STDOUT_FILENO)
         would produce similar output to the following: */

        strings = backtrace_symbols(buffer, nptrs);
        if (strings != NULL)
        {
            for (j = 0; j < nptrs; j++) {
                printf("%s\n", strings[j]);
            }
            free(strings);
        } else {
            printf("No backtrace!\n");
        }
#endif // BUILD_DSPAL

        if (not this->m_disableAssert) {
            // for **nix, delay then exit with error code
            Os::Log::logMsg("FATAL %d handled.\n", (U32) Id, 0, 0, 0, 0, 0);
#ifdef BUILD_DSPAL
            terminate = true;
#endif
            (void) Os::Task::delay(1000);
            //Os::Log::logMsg("Exiting.\n", 0, 0, 0, 0, 0, 0);
            //assert(0);
#ifndef BUILD_DSPAL
            (void) raise(SIGTERM);
#endif
            pthread_exit(0);
        }
    }

} // end namespace Svc
