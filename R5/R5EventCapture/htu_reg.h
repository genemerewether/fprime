
#ifndef __TMS570_HTU_H__
#define __TMS570_HTU_H__

// Why doesn't TI provide these defines...

#define HTU_BIT(x) (1 << x)

// Global Control Register
#define HTU_GC_VBUSHOLD HTU_BIT(24)
#define HTU_GC_HTUEN HTU_BIT(16)
#define HTU_GC_DEBM HTU_BIT(8)
#define HTU_GC_HTURES HTU_BIT(0)

// Control Packet Enable Register
#define HTU_CPENA_CPENA_ENA(x) (1 << (x*2))
#define HTU_CPENA_CPENA_ENB(x) (2 << (x*2))
#define HTU_CPENA_CPENA_DIS(x) (3 << (x*2))

// Control Packet Busy
#define HTU_BUSY0_BUSY0A HTU_BIT(24)
#define HTU_BUSY0_BUSY0B HTU_BIT(16)
#define HTU_BUSY0_BUSY1A HTU_BIT(8)
#define HTU_BUSY0_BUSY1B HTU_BIT(0)

#define HTU_BUSY1_BUSY2A HTU_BIT(24)
#define HTU_BUSY1_BUSY2B HTU_BIT(16)
#define HTU_BUSY1_BUSY3A HTU_BIT(8)
#define HTU_BUSY1_BUSY3B HTU_BIT(0)

#define HTU_BUSY2_BUSY4A HTU_BIT(24)
#define HTU_BUSY2_BUSY4B HTU_BIT(16)
#define HTU_BUSY2_BUSY5A HTU_BIT(8)
#define HTU_BUSY2_BUSY5B HTU_BIT(0)

#define HTU_BUSY3_BUSY6A HTU_BIT(24)
#define HTU_BUSY3_BUSY6B HTU_BIT(16)
#define HTU_BUSY3_BUSY7A HTU_BIT(8)
#define HTU_BUSY0_BUSY7B HTU_BIT(0)

// Active Control Packet and Error Register
#define HTU_ACPE_ERRF HTU_BIT(31)
#define HTU_ACPE_ERRETC_MASK (0x1F)
#define HTU_ACPE_ERRETC_SHIFT (24)
#define HTU_ACPE_ERRCPN_MASK (0xF)
#define HTU_ACPE_ERRCPN_SHIFT (16)
#define HTU_ACPE_TIPF HTU_BIT(15)
#define HTU_ACPE_BUSBUSY HTU_BIT(14)
#define HTU_ACPE_CETCOUNT_MASK (0x1F)
#define HTU_ACPE_CETCOUNT_SHIFT (8)
#define HTU_ACPE_NACP_MASK (0xF)
#define HTU_ACPE_NACP_SHIFT (0)

// Request Lost and Bus Error Control Register
#define HTU_RLBECTRL_BERINTENA HTU_BIT(16)
#define HTU_RLBECTRL_CORL HTU_BIT(8)
#define HTU_RLBECTRL_RLINTENA HTU_BIT(0)

// Buffer Full Interrupt Enable Set Register

// Buffer Full Interrupt Enable Clear Register

// Interrupt Mapping Register

// Interrupt Offset Register

// Buffer Initialization Mode Register
#define HTU_BIM_BIM(x) (1 << x)

// Request Lost Flag Register
#define HTU_RLOSTFL_CPRLFL(x) (1 << x)

// Buffer Full Interrupt Flag Register
#define HTU_BFINTFL_BFINTFL(x) (1 << x)

// BER Interrupt Flag Register
#define HTU_BER_BERINTFL(x) (1 << x)


// DCP Configuration Memory
#define DCP_IHADDRCT_DIR HTU_BIT(23)
#define DCP_IHADDRCT_SIZE HTU_BIT(22)
#define DCP_IHADDRCT_ADDMH HTU_BIT(21)
#define DCP_IHADDRCT_ADDFM HTU_BIT(20)
#define DCP_IHADDRCT_TMBA_1 HTU_BIT(19)
#define DCP_IHADDRCT_TMBA_0 HTU_BIT(18)
#define DCP_IHADDRCT_TMBB_1 HTU_BIT(17)
#define DCP_IHADDRCT_TMBB_0 HTU_BIT(16)
#define DCP_IHADDRCT_IHADDR_MASK (0x7FF)
#define DCP_IHADDRCT_IHADDR_SHIFT (2)

// DCP Initial Transfer Count Register
#define DCP_ITCOUNT_IETCOUNT_MASK (0x1F)
#define DCP_ITCOUNT_IETCOUNT_SHIFT (16)
#define DCP_ITCOUNT_IFTCOUNT_MASK (0xFF)
#define DCP_ITCOUNT_IFTCOUNT_SHIFT (0)

// DCP Current Frame Counter Register
#define CDCP_CFCOUNT_CFTCTA_MASK (0xFF)
#define CDCP_CFCOUNT_CFTCTA_SHIFT (16)
#define CDCP_CFCOUNT_CFTCTB_MASK (0xFF)
#define CDCP_CFCOUNT_CFTCTB_SHIFT (0)



#endif

