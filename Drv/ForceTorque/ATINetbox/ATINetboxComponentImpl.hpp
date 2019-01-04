#ifndef _ATINETBOX_COMPONENT_IMPL_HPP_
#define _ATINETBOX_COMPONENT_IMPL_HPP_

#include <arpa/inet.h>

#include <Os/Task.hpp>
#include <Drv/ForceTorque/ATINetbox/ATINetboxComponentAc.hpp>
#include <Drv/ForceTorque/ATINetbox/ATINetboxComponentImplCfg.hpp>

namespace Drv {

  class ATINetboxComponentImpl : public ATINetboxComponentBase {
  public:

    typedef enum {
      ATINET_ENABLED  = 0,
      ATINET_DISABLED = 1,
      ATINET_CLOSED   = 2,
    } ATINET_states_t;

    typedef enum {
      RDT_CMD_STOP        = 0x0000,
      RDT_CMD_START_RT    = 0x0002,
      RDT_CMD_START_BUF   = 0x0003,
      RDT_CMD_START_MULTI = 0x0004,
      RDT_CMD_RESET_LATCH = 0x0041,
      RDT_CMD_SET_BIAS    = 0x0042,
    } RDT_request_cmd_t;

    typedef struct {
      U16 header;
      U16 command;
      U32 scount;
    } RDT_request_t;

    typedef struct {
      RDT_request_t req;
      U32 ipaddr_dest;
      U16 port_dest;
    } RDT_request_ext_t;

    typedef struct {
      U32 rdt_seq;
      U32 ft_seq;
      U32 stat;

      I32 Fx;
      I32 Fy;
      I32 Fz;
      I32 Tx;
      I32 Ty;
      I32 Tz;

    } RDT_record_t;

#if FW_OBJECT_NAMES == 1
    explicit ATINetboxComponentImpl (const char * compName);
#else
    explicit ATINetboxComponentImpl (void);
#endif

    virtual ~ATINetboxComponentImpl (void);

    void init (void);
    void open (const char * local_ip, const char * remote_ip, U16 remote_port);
    void stop  (void);
    void set_thread_attr (NATIVE_INT_TYPE ident,
			  NATIVE_INT_TYPE prio,
			  NATIVE_INT_TYPE stack_size,
			  bool            realtime,
			  int             affinity);

    static const U16 ATINETBOX_RDP_PORT;

  protected:
  private:

    void start (void);
    void close (void);

    void ATINET_BIAS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq);
    void ATINET_ENABLE_cmdHandler(FwOpcodeType opCode , U32 cmdSeq,
			       ATINetboxComponentBase::ATINetboxEnableMode);
    void ATINET_FLOW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
              ATINetboxComponentBase::ATINetboxFlowMode mode);
    void ATINET_DEBUG_cmdHandler(
            FwOpcodeType opCode, //!< The opcode
            U32 cmdSeq);

    void cfg_request     (RDT_request_t     & req, RDT_request_cmd_t cmd, U32 count);
    void cfg_request_ext (RDT_request_ext_t & req,
			  RDT_request_cmd_t   cmd, U32 count, U32 ip, U16 port);

    void write_request   (const RDT_request_t     & req);
    void write_request   (const RDT_request_ext_t & req);
    void write_socket    (void * buf, NATIVE_UINT_TYPE len);

    bool read_record (RDT_record_t & rec);
    bool wait_readable (void);

    void send_telem (const RDT_record_t & rec, const Fw::Time & ts);

    void bias (void);

    static void ATINetboxTaskFunc (void * ptr);

    F32 m_cpf; // "Counts per Force"
    F32 m_cpt; // "Counts per Torque"

    NATIVE_INT_TYPE m_fd;
    struct sockaddr_in m_saddr; // My address information
    struct sockaddr_in m_raddr; // Their information

    Os::Task        m_task;
    NATIVE_INT_TYPE m_run;
    NATIVE_INT_TYPE m_mode;
    bool            m_bias;

    // Whether to flow F/T packets
    NATIVE_INT_TYPE m_flow;

    NATIVE_INT_TYPE m_thread_ident;
    NATIVE_INT_TYPE m_thread_prio;
    NATIVE_INT_TYPE m_thread_stack_sz;
    bool m_thread_realtime;
    int  m_thread_affinity;

    // coefficients for averaging. See ticket #45
    F32 m_a1;
    F32 m_a2;
    F32 m_b0;
    F32 m_b1;
    F32 m_b2;

    // coefficients for anti-aliasing. See ticket #523
    // numerator first, then denominator
    F64 m_aa[FT_AA_FILT_SIZE];

    // antialiasing low-pass filters
    // TODO(mereweth)

    // Transform matrices for trimming

    // Force matrix
    F32 m_mR11;
    F32 m_mR12;
    F32 m_mR13;
    F32 m_mR21;
    F32 m_mR22;
    F32 m_mR23;
    F32 m_mR31;
    F32 m_mR32;
    F32 m_mR33;

    // Force tares
    F32 m_mBFX;
    F32 m_mBFY;
    F32 m_mBFZ;

    // Torque tares
    F32 m_mBTX;
    F32 m_mBTY;
    F32 m_mBTZ;

    // cross-product vector
    F32 m_PX;
    F32 m_PY;
    F32 m_PZ;

    // enumeration for storing value history
    enum ValueHistories {
        HIST_FORCE_X,
        HIST_FORCE_Y,
        HIST_FORCE_Z,
        HIST_TORQUE_X,
        HIST_TORQUE_Y,
        HIST_TORQUE_Z,
        HIST_NIM_HIST
    };

    // value history for averaging
    struct HistElem {
        F32 y1;
        F32 y2;
        F32 u1;
        F32 u2;
    }  m_hist[HIST_NIM_HIST];

    // update history
    F32 filterVal(F32 curr, ValueHistories history);

    void parameterUpdated(FwPrmIdType id);

    void setParameterVariable(U32 id, bool sendEvent = false);

    void logWarning(const char *msg);

    void logDebug(const char *msg);

    void ATINET_TARE_cmdHandler(FwOpcodeType opCode,U32 cmdSeq,U32 cycles);

    bool m_doTare;
    U32 m_tareCycles;
    U32 m_tareCyclesLeft;

    // Force tare averages
    F32 m_forceXAve;
    F32 m_forceYAve;
    F32 m_forceZAve;

    // Torque tare averages
    F32 m_torqueXAve;
    F32 m_torqueYAve;
    F32 m_torqueZAve;

    // Throttles
    bool m_throttleErrors;

    U32 m_numPackets; // numbers of packets processed

  };

}

#endif //_ATINETBOX_COMPONENT_IMPL_HPP_
