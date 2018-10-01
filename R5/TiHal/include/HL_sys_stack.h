// ====================================================================== 
// \title  HL_sys_stack.h
// \author kubiak
// \brief  h file for stack monitoring functions
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
// 
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ====================================================================== 

#ifndef __HL_SYS_STACK_H
#define __HL_SYS_STACK_H

#ifdef __cplusplus
extern "C" {
#endif

/** @fn void _initStackPattern_(uint32_t pattern)
*   @brief Initializes the unused portion of the stack with pattern
*/
void initStackPattern(uint32_t pattern);

/** @fn uint32_t _checkStackPattern_(void)
*   @brief Checks the number of patterns still present in stack space
*/
uint32_t checkStackPattern(uint32_t pattern);

#ifdef __cplusplus
}
#endif

#endif
