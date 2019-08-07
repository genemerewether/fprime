// ====================================================================== 
// \title  rtiDefs.cpp
// \author kubiak
// \brief  Rti peripheral definitions
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


#ifndef __RTI_DEFS_H__
#define __RTI_DEFS_H__

#define RTI_BIT(x) (1 << x)

// RTI COMPCTRL Register
#define RTI_COMPCTRL_COMPSEL3 RTI_BIT(12)
#define RTI_COMPCTRL_COMPSEL2 RTI_BIT(8)
#define RTI_COMPCTRL_COMPSEL1 RTI_BIT(4)
#define RTI_COMPCTRL_COMPSEL0 RTI_BIT(0)

// RTI INTFLAG Register
#define RTI_INTFLAG_OVL1INT RTI_BIT(18)
#define RTI_INTFLAG_OVL0INT RTI_BIT(17)
#define RTI_INTFLAG_TBINT   RTI_BIT(16)
#define RTI_INTFLAG_INT3  RTI_BIT(3)
#define RTI_INTFLAG_INT2  RTI_BIT(2)
#define RTI_INTFLAG_INT1  RTI_BIT(1)
#define RTI_INTFLAG_INT0  RTI_BIT(0)

#endif
