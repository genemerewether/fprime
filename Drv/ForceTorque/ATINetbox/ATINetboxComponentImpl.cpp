#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>

#include <Fw/Types/Assert.hpp>
#include <Fw/Time/Time.hpp>
#include <Drv/ForceTorque/ATINetbox/ATINetboxComponentImpl.hpp>
#include <Drv/ForceTorque/ATINetbox/ATINetboxComponentImplCfg.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

namespace Drv {

  const U16 ATINetboxComponentImpl::ATINETBOX_RDP_PORT = 49152;

  ATINetboxComponentImpl::ATINetboxComponentImpl(const char* compName) :
    ATINetboxComponentBase(compName) {

    this->m_fd = -1;
    this->m_run = false;
    this->m_mode = ATINET_DISABLED;
    this->m_bias = true;
    this->m_flow = ATINET_NOFLOW;

    this->m_cpf = 1.0;
    this->m_cpt = 1.0;

    this->m_a1 = 0.0;
    this->m_a2 = 0.0;
    this->m_b0 = 0.0;
    this->m_b1 = 0.0;
    this->m_b2 = 0.0;

    for (int i = 0; i < FT_AA_FILT_SIZE; i++) {
      this->m_aa[i] = 0.0;
    }
    this->m_aa[FT_AA_FILT_NUMER_SIZE] = 1.0f;

    this->m_thread_ident = 0;
    this->m_thread_prio = 10;
    this->m_thread_stack_sz = 20 * 1024;
    this->m_thread_realtime = 1;
    this->m_thread_affinity = -1;

    // clear average history
    for (NATIVE_UINT_TYPE hist = 0; hist < HIST_NIM_HIST; hist++) {
      this->m_hist[hist].y1 = 0.0;
      this->m_hist[hist].y2 = 0.0;
      this->m_hist[hist].u1 = 0.0;
      this->m_hist[hist].u2 = 0.0;
    }

    // tare variables
    this->m_doTare = false;
    this->m_tareCycles = 0;
    this->m_tareCyclesLeft = 0;

    this->m_forceXAve = 0.0;
    this->m_forceYAve = 0.0;
    this->m_forceZAve = 0.0;

    // Torque tare averages
    this->m_torqueXAve = 0.0;
    this->m_torqueYAve = 0.0;
    this->m_torqueZAve = 0.0;

    this->m_throttleErrors = false;

    this->m_numPackets = 0;
  }

  ATINetboxComponentImpl::~ATINetboxComponentImpl(void) {
  }

  void ATINetboxComponentImpl::cfg_request(RDT_request_t & req,
                                           RDT_request_cmd_t cmd, U32 count) {
    req.header = htons((U16) 0x1234);
    req.command = htons((U16) cmd);
    req.scount = htonl(count);
  }

  void ATINetboxComponentImpl::cfg_request_ext(RDT_request_ext_t & req,
                                               RDT_request_cmd_t cmd, U32 count, U32 ip, U16 port) {
    cfg_request(req.req, cmd, count);
    req.ipaddr_dest = htonl(ip);
    req.port_dest = htons(port);
  }

  void ATINetboxComponentImpl::write_request(const RDT_request_t & req) {
    this->write_socket((void *) &req, sizeof(req));
  }

  void ATINetboxComponentImpl::write_request(
                                             const RDT_request_ext_t & req) {
    this->write_socket((void *) &req, sizeof(req));
  }

  void ATINetboxComponentImpl::write_socket(void * buf,
                                            NATIVE_UINT_TYPE len) {
    int ret;
    int flags = 0;

    sockaddr * saddr_ptr = (sockaddr *) &m_raddr;
    socklen_t socklen = sizeof(m_raddr);

    if ((ret = sendto(m_fd, buf, len, flags, saddr_ptr, socklen)) < 0) {
      // Issue failure EVR
      if (not this->m_throttleErrors) {
        printf("error from sendto: %d, %s\n", errno, strerror(errno));
        this->m_throttleErrors = true;
      }
    } else {
      if ((unsigned int) ret != len) {
        // Issue failure EVR
        printf(
               "unable to write all data to UDP packet. Wrote %d, expected %d\n",
               ret, len);
      }
    }
  }

  bool ATINetboxComponentImpl::read_record(RDT_record_t & rec) {
    int ret;
    int flags = 0;

    sockaddr raddr;
    socklen_t socklen;

    if ((ret = recvfrom(m_fd, (void *) &rec, sizeof(rec), flags, &raddr,
                        &socklen)) < 0) {
      // Issue failure EVR
      if (not this->m_throttleErrors) {
        printf("error from recvfrom: %d, %s\n", errno, strerror(errno));
        this->m_throttleErrors = true;
      }
    }

    if (ret == sizeof(rec)) {
      rec.rdt_seq = ntohl(rec.rdt_seq);
      rec.ft_seq = ntohl(rec.ft_seq);
      rec.stat = ntohl(rec.stat);
      rec.Fx = ntohl(rec.Fx);
      rec.Fy = ntohl(rec.Fy);
      rec.Fz = ntohl(rec.Fz);
      rec.Tx = ntohl(rec.Tx);
      rec.Ty = ntohl(rec.Ty);
      rec.Tz = ntohl(rec.Tz);

      return true;
    } else {
      memset(&rec, 0, sizeof(rec));
      return false;
    }
  }

  void ATINetboxComponentImpl::init() {
    ATINetboxComponentBase::init();
  }

  void ATINetboxComponentImpl::open(const char * local_ip,
                                    const char * remote_ip, U16 remote_port) {

    // FIXME : make re-entrant

    if (this->m_fd >= 0) {
      ::close(this->m_fd);
    }

    // Create our socket to talk with the device
    if ((this->m_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
      log_WARNING_HI_ATINET_SocketCreateFail();
      return;
    }

    // Clear structs
    memset((char *) &m_saddr, 0, sizeof(m_saddr));
    memset((char *) &m_raddr, 0, sizeof(m_raddr));

    m_saddr.sin_family = AF_INET;
    m_raddr.sin_family = AF_INET;

    inet_pton(AF_INET, local_ip, &(m_saddr.sin_addr.s_addr));
    inet_pton(AF_INET, remote_ip, &(m_raddr.sin_addr.s_addr));

    m_saddr.sin_port = htons(0);
    m_raddr.sin_port = htons(remote_port);

    // Bind our local socket
    if (bind(this->m_fd, (struct sockaddr *) &m_saddr, sizeof(m_saddr)) < 0) {
      log_WARNING_HI_ATINET_SocketBindFail();
      return;
    }

    log_ACTIVITY_LO_ATINET_SocketCreated();
  }

  void ATINetboxComponentImpl::set_thread_attr(NATIVE_INT_TYPE ident,
                                               NATIVE_INT_TYPE prio, NATIVE_INT_TYPE stack_size, bool realtime,
                                               int affinity) {
    this->m_thread_ident = ident;
    this->m_thread_prio = prio;
    this->m_thread_stack_sz = stack_size;
    this->m_thread_realtime = realtime;
    this->m_thread_affinity = affinity;
  }

  void ATINetboxComponentImpl::start(void) {

    if (!__sync_bool_compare_and_swap(&m_mode, ATINET_DISABLED, ATINET_ENABLED)) {
      return;
    }

    // load parameters into local variables
    this->setParameterVariable(PARAMID_COUNTSPERFORCE);
    this->setParameterVariable(PARAMID_COUNTSPERTORQUE);

    this->setParameterVariable(PARAMID_FTAVEA1);
    this->setParameterVariable(PARAMID_FTAVEA2);
    this->setParameterVariable(PARAMID_FTAVEB0);
    this->setParameterVariable(PARAMID_FTAVEB1);
    this->setParameterVariable(PARAMID_FTAVEB2);

    //NOTE(mereweth) - make sure the filter size corresponds to the number of fprime params
    FW_ASSERT(FT_AA_FILT_SIZE == 12);

    this->setParameterVariable(PARAMID_FTANTIALIAS1);
    this->setParameterVariable(PARAMID_FTANTIALIAS2);
    this->setParameterVariable(PARAMID_FTANTIALIAS3);
    this->setParameterVariable(PARAMID_FTANTIALIAS4);
    this->setParameterVariable(PARAMID_FTANTIALIAS5);
    this->setParameterVariable(PARAMID_FTANTIALIAS6);
    this->setParameterVariable(PARAMID_FTANTIALIAS7);
    this->setParameterVariable(PARAMID_FTANTIALIAS8);
    this->setParameterVariable(PARAMID_FTANTIALIAS9);
    this->setParameterVariable(PARAMID_FTANTIALIAS10);
    this->setParameterVariable(PARAMID_FTANTIALIAS11);
    this->setParameterVariable(PARAMID_FTANTIALIAS12);

    this->setParameterVariable(PARAMID_R11);
    this->setParameterVariable(PARAMID_R12);
    this->setParameterVariable(PARAMID_R13);
    this->setParameterVariable(PARAMID_R21);
    this->setParameterVariable(PARAMID_R22);
    this->setParameterVariable(PARAMID_R23);
    this->setParameterVariable(PARAMID_R31);
    this->setParameterVariable(PARAMID_R32);
    this->setParameterVariable(PARAMID_R33);

    this->setParameterVariable(PARAMID_BTX);
    this->setParameterVariable(PARAMID_BTY);
    this->setParameterVariable(PARAMID_BTZ);
    this->setParameterVariable(PARAMID_BFX);
    this->setParameterVariable(PARAMID_BFY);
    this->setParameterVariable(PARAMID_BFZ);

    this->setParameterVariable(PARAMID_PX);
    this->setParameterVariable(PARAMID_PY);
    this->setParameterVariable(PARAMID_PZ);

    // TODO(Mereweth) - init filters
		
    this->m_run = true;

    Os::TaskString task_name("eft_worker");
    Os::Task::TaskStatus stat = this->m_task.start(task_name,
                                                   this->m_thread_ident, this->m_thread_prio, this->m_thread_stack_sz,
                                                   ATINetboxTaskFunc, (void *) this, this->m_thread_affinity);

    FW_ASSERT(Os::Task::TASK_OK == stat, stat);
  }

  void ATINetboxComponentImpl::stop() {

    this->m_run = false;

    //this->m_task.cleanup();

    this->m_mode = ATINET_DISABLED;
  }

  void ATINetboxComponentImpl::close() {

    this->m_run = false;

    //this->m_task.cleanup();

    this->m_mode = ATINET_CLOSED;

    // Shutdown socket here
  }

  bool ATINetboxComponentImpl::wait_readable(void) {
    fd_set rfds;
    struct timespec tv;
    int ret;

    FD_ZERO(&rfds);
    FD_SET(m_fd, &rfds);

    tv.tv_sec = 0;
    tv.tv_nsec = 100000000; // 100ms timeout

    ret = pselect(this->m_fd + 1, &rfds, NULL, NULL, &tv, NULL);

    switch (ret) {
    case -1: { // We had some error
      printf("Error in select: %d, %s", errno, strerror(errno));
      return false;
    }
    case 0: { // We timed out, so no packet
      return false;
    }
    default: { // We got a readable state, packet's there!
      return true;
    }
    }
  }

  void ATINetboxComponentImpl::send_telem(const RDT_record_t & rec,
                                          const Fw::Time & ts) {

    Svc::ActiveFileLoggerPacket packet;

    F32 Fx, Fy, Fz, Tx, Ty, Tz;

    //    char msg[40];
    //    Fw::LogStringArg arg;
    //
    //    snprintf(msg,sizeof(msg),"%f %f",m_cpf,m_cpt);
    //    arg = msg;
    //    this->log_ACTIVITY_HI_ATINetboxMsg(arg);

    Fx = rec.Fx / this->m_cpf;
    Fy = rec.Fy / this->m_cpf;
    Fz = rec.Fz / this->m_cpf;
    Tx = rec.Tx / this->m_cpt;
    Ty = rec.Ty / this->m_cpt;
    Tz = rec.Tz / this->m_cpt;

    // antialias filter only the data we send every cycle
    // TODO(mereweth)
	
    // send subset of packets
    if (this->m_flow == ATINET_FLOW) {
      if ((this->m_numPackets % FT_DECIMATE) == 0) {

        if (this->isConnected_ATINetboxData_OutputPort(0)) {
          //this->ATINetboxData_out(0, ftsAA, ts);
        }

        /* TODO(mereweth@jpl.nasa.gov) - what format for FTS antialiasing log?
         * and does it need to be optional?
         */
        packet.resetSer();
        //packet.serialize((U8) AFL_FORCE_TORQUE_ANTIALIAS);

        if (isConnected_ATINetboxLog_OutputPort(0)) {
          this->ATINetboxLog_out(0, packet);
        }
      }
    }

    this->tlmWrite_ATINET_ForceXRaw(Fx);
    this->tlmWrite_ATINET_ForceYRaw(Fy);
    this->tlmWrite_ATINET_ForceZRaw(Fz);
    this->tlmWrite_ATINET_TorqueXRaw(Tx);
    this->tlmWrite_ATINET_TorqueYRaw(Ty);
    this->tlmWrite_ATINET_TorqueZRaw(Tz);

    // Compute filtered force value
    F32 fXAve = filterVal(Fx, HIST_FORCE_X);
    F32 fYAve = filterVal(Fy, HIST_FORCE_Y);
    F32 fZAve = filterVal(Fz, HIST_FORCE_Z);

    F32 netFx = fXAve - this->m_mBFX;
    F32 netFy = fYAve - this->m_mBFY;
    F32 netFz = fZAve - this->m_mBFZ;

    // Do transformations
    F32 tranFx = this->m_mR11 * netFx + this->m_mR12 * netFy
      + this->m_mR13 * netFz;
    F32 tranFy = this->m_mR21 * netFx + this->m_mR22 * netFy
      + this->m_mR23 * netFz;
    F32 tranFz = this->m_mR31 * netFx + this->m_mR32 * netFy
      + this->m_mR33 * netFz;

    // write channels and log values
    this->tlmWrite_ATINET_ForceX(tranFx);
    this->tlmWrite_ATINET_ForceY(tranFy);
    this->tlmWrite_ATINET_ForceZ(tranFz);

    // Compute filtered torque value
    F32 tXAve = this->filterVal(Tx, HIST_TORQUE_X);
    F32 tYAve = this->filterVal(Ty, HIST_TORQUE_Y);
    F32 tZAve = this->filterVal(Tz, HIST_TORQUE_Z);

    F32 netTx = tXAve - this->m_mBTX;
    F32 netTy = tYAve - this->m_mBTY;
    F32 netTz = tZAve - this->m_mBTZ;

    // Do transformations
    F32 tranTx = this->m_mR11
      * (-1.0 * netFz * this->m_PY + netFy * this->m_PZ + netTx)
      + this->m_mR12 * (netFz * this->m_PX - netFx * this->m_PZ + netTy)
      + this->m_mR13
      * (-1.0 * netFy * this->m_PX + netFx * this->m_PY + netTz);

    F32 tranTy = this->m_mR21
      * (-1.0 * netFz * this->m_PY + netFy * this->m_PZ + netTx)
      + this->m_mR22 * (netFz * this->m_PX - netFx * this->m_PZ + netTy)
      + this->m_mR23
      * (-1.0 * netFy * this->m_PX + netFx * this->m_PY + netTz);

    F32 tranTz = this->m_mR31
      * (-1.0 * netFz * this->m_PY + netFy * this->m_PZ + netTx)
      + this->m_mR32 * (netFz * this->m_PX - netFx * this->m_PZ + netTy)
      + this->m_mR33
      * (-1.0 * netFy * this->m_PX + netFx * this->m_PY + netTz);

    // do taring if requested
    if (this->m_doTare) {
      if (this->m_tareCyclesLeft-- > 0) {
        this->m_forceXAve += Fx;
        this->m_forceYAve += Fy;
        this->m_forceZAve += Fz;
        this->m_torqueXAve += Tx;
        this->m_torqueYAve += Ty;
        this->m_torqueZAve += Tz;
      } else { // zero
        // do average and assign
        this->m_mBFX = this->m_forceXAve / (F32) this->m_tareCycles;
        this->m_mBFY = this->m_forceYAve / (F32) this->m_tareCycles;
        this->m_mBFZ = this->m_forceZAve / (F32) this->m_tareCycles;

        this->m_mBTX = this->m_torqueXAve / (F32) this->m_tareCycles;
        this->m_mBTY = this->m_torqueYAve / (F32) this->m_tareCycles;
        this->m_mBTZ = this->m_torqueZAve / (F32) this->m_tareCycles;
        // send EVR
        this->log_ACTIVITY_HI_ATINET_TareComplete(this->m_mBFX, this->m_mBFY,
                                                  this->m_mBFZ, this->m_mBTX, this->m_mBTY, this->m_mBTZ);
        this->m_doTare = false;
      }
    }

    // write channels and log values
    this->tlmWrite_ATINET_TorqueX(tranTx);
    this->tlmWrite_ATINET_TorqueY(tranTy);
    this->tlmWrite_ATINET_TorqueZ(tranTz);

    // write health word
    U32 health = rec.stat;
    this->tlmWrite_ATINET_Health(health);

    packet.resetSer();
    //packet.serialize(health);

    if (isConnected_ATINetboxLog_OutputPort(0)) {
      //this->ATINetboxLog_out(0, packet);
    }


  }

  void ATINetboxComponentImpl::ATINET_BIAS_cmdHandler(FwOpcodeType opCode,
                                                      U32 cmdSeq) {
    this->m_bias = true;
    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void ATINetboxComponentImpl::ATINET_ENABLE_cmdHandler(FwOpcodeType opCode,
                                                        U32 cmdSeq, ATINetboxComponentBase::ATINetboxEnableMode mode) {
    switch (mode) {
    case ATINET_ENABLE: {
      this->start();
      break;
    }
    case ATINET_DISABLE: {
      this->stop();
      break;
    }
    default: {
      FW_ASSERT(false, false);
      return;
    }
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void ATINetboxComponentImpl::ATINET_FLOW_cmdHandler(FwOpcodeType opCode,
                                                          U32 cmdSeq, ATINetboxComponentBase::ATINetboxFlowMode mode) {
    switch (mode) {
    case ATINET_FLOW: {
      this->m_flow = ATINET_FLOW;
      break;
    }
    case ATINET_NOFLOW: {
      this->m_flow = ATINET_NOFLOW;
      break;
    }
    default: {
      FW_ASSERT(false, false);
      return;
    }
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void ATINetboxComponentImpl::bias(void) {
    RDT_request_t req;

    if (this->m_bias) {
      this->cfg_request(req, RDT_CMD_SET_BIAS, 0);
      this->write_request(req);
      this->log_ACTIVITY_HI_ATINET_Biased();
      this->m_bias = false;
    }
  }

  void ATINetboxComponentImpl::ATINetboxTaskFunc(void * ptr) {
    Fw::Time ts;
    ATINetboxComponentImpl * comp = static_cast<ATINetboxComponentImpl*>(ptr);
    RDT_request_t req;
    RDT_record_t rec;

    comp->log_ACTIVITY_LO_ATINET_WorkerStarted();

    // Bias the system
    // comp->bias(); // TKC - let bias calibration procedure use own biases

    // Begin transmission in RDT (UDP realtime) mode (forever: count = 0)
    comp->cfg_request(req, RDT_CMD_START_RT, 0);
    comp->write_request(req);

    while (comp->m_run) {              // While the system is running ...

      //        comp->bias();

      if (comp->wait_readable()) { // Wait until a packet arrives (with timeout)
        if (comp->read_record(rec)) { // If that packet was from the F/T ...

          // Acquire time stamp
          ts = comp->getTime();

          // Check sequence numbers

          comp->send_telem(rec, ts); // Send telemetry updates to the system
          comp->tlmWrite_ATINET_Packets(++comp->m_numPackets);
        }
      }
    }

    comp->cfg_request(req, RDT_CMD_STOP, 0);
    comp->write_request(req);

    comp->log_ACTIVITY_LO_ATINET_WorkerStopped();
  }

  F32 ATINetboxComponentImpl::filterVal(F32 u, ValueHistories history) {
    F32 y = -1.0 * this->m_a1 * this->m_hist[history].y1
      + -1.0 * this->m_a2 * this->m_hist[history].y2
      + 1.0 * this->m_b0 * u + 1.0 * this->m_b1 * this->m_hist[history].u1
      + 1.0 * this->m_b2 * this->m_hist[history].u2;
    // update history
    this->m_hist[history].u2 = this->m_hist[history].u1;
    this->m_hist[history].u1 = u;

    this->m_hist[history].y2 = this->m_hist[history].y1;
    this->m_hist[history].y1 = y;

    return y;

  }

  void ATINetboxComponentImpl::parameterUpdated(FwPrmIdType id) {
    this->setParameterVariable(id, true);
  }

  void ATINetboxComponentImpl::setParameterVariable(U32 id, bool sendEvent) {

    Fw::ParamValid valid = Fw::PARAM_INVALID;

    switch (id) {
      // Transform matrix
    case PARAMID_R11:
      this->m_mR11 = this->paramGet_R11(valid);
      break;
    case PARAMID_R12:
      this->m_mR12 = this->paramGet_R12(valid);
      break;
    case PARAMID_R13:
      this->m_mR13 = this->paramGet_R13(valid);
      break;
    case PARAMID_R21:
      this->m_mR21 = this->paramGet_R21(valid);
      break;
    case PARAMID_R22:
      this->m_mR22 = this->paramGet_R22(valid);
      break;
    case PARAMID_R23:
      this->m_mR23 = this->paramGet_R23(valid);
      break;
    case PARAMID_R31:
      this->m_mR31 = this->paramGet_R31(valid);
      break;
    case PARAMID_R32:
      this->m_mR32 = this->paramGet_R32(valid);
      break;
    case PARAMID_R33:
      this->m_mR33 = this->paramGet_R33(valid);
      break;

      // Biases
    case PARAMID_BTX:
      this->m_mBTX = this->paramGet_BTX(valid);
      break;
    case PARAMID_BTY:
      this->m_mBTY = this->paramGet_BTY(valid);
      break;
    case PARAMID_BTZ:
      this->m_mBTZ = this->paramGet_BTZ(valid);
      break;
    case PARAMID_BFX:
      this->m_mBFX = this->paramGet_BFX(valid);
      break;
    case PARAMID_BFY:
      this->m_mBFY = this->paramGet_BFY(valid);
      break;
    case PARAMID_BFZ:
      this->m_mBFZ = this->paramGet_BFZ(valid);
      break;

      // Averaging
    case PARAMID_FTAVEA1:
      this->m_a1 = this->paramGet_FTAveA1(valid);
      break;
    case PARAMID_FTAVEA2:
      this->m_a2 = this->paramGet_FTAveA2(valid);
      break;
    case PARAMID_FTAVEB0:
      this->m_b0 = this->paramGet_FTAveB0(valid);
      break;
    case PARAMID_FTAVEB1:
      this->m_b1 = this->paramGet_FTAveB1(valid);
      break;
    case PARAMID_FTAVEB2:
      this->m_b2 = this->paramGet_FTAveB2(valid);
      break;

      //NOTE(mereweth) - make sure the filter size corresponds to the number of fprime params
      COMPILE_TIME_ASSERT(FT_AA_FILT_SIZE == 12, ATINETBOX_AA_PARM_ENOUGH);

    case PARAMID_FTANTIALIAS1:
      this->m_aa[0] = this->paramGet_FTAntiAlias1(valid);
      break;
    case PARAMID_FTANTIALIAS2:
      this->m_aa[1] = this->paramGet_FTAntiAlias2(valid);
      break;
    case PARAMID_FTANTIALIAS3:
      this->m_aa[2] = this->paramGet_FTAntiAlias3(valid);
      break;
    case PARAMID_FTANTIALIAS4:
      this->m_aa[3] = this->paramGet_FTAntiAlias4(valid);
      break;
    case PARAMID_FTANTIALIAS5:
      this->m_aa[4] = this->paramGet_FTAntiAlias5(valid);
      break;
    case PARAMID_FTANTIALIAS6:
      this->m_aa[5] = this->paramGet_FTAntiAlias6(valid);
      break;
    case PARAMID_FTANTIALIAS7:
      this->m_aa[6] = this->paramGet_FTAntiAlias7(valid);
      break;
    case PARAMID_FTANTIALIAS8:
      this->m_aa[7] = this->paramGet_FTAntiAlias8(valid);
      break;
    case PARAMID_FTANTIALIAS9:
      this->m_aa[8] = this->paramGet_FTAntiAlias9(valid);
      break;
    case PARAMID_FTANTIALIAS10:
      this->m_aa[9] = this->paramGet_FTAntiAlias10(valid);
      break;
    case PARAMID_FTANTIALIAS11:
      this->m_aa[10] = this->paramGet_FTAntiAlias11(valid);
      break;
    case PARAMID_FTANTIALIAS12:
      this->m_aa[11] = this->paramGet_FTAntiAlias12(valid);
      break;

      // Counts
    case PARAMID_COUNTSPERFORCE:
      this->m_cpf = this->paramGet_CountsPerForce(valid);
      break;
    case PARAMID_COUNTSPERTORQUE:
      this->m_cpt = this->paramGet_CountsPerTorque(valid);
      break;

      // cross-product vector
    case PARAMID_PX:
      this->m_PX = this->paramGet_PX(valid);
      break;
    case PARAMID_PY:
      this->m_PY = this->paramGet_PY(valid);
      break;
    case PARAMID_PZ:
      this->m_PZ = this->paramGet_PZ(valid);
      break;

    default:
      FW_ASSERT(0, id);
      break;

    }

    switch (valid) {
    case Fw::PARAM_VALID: // okay
    case Fw::PARAM_DEFAULT:
      break;
    default:
      this->log_WARNING_HI_ATINET_PrmLdErr(id);
      break;
    }

    if (sendEvent) {
      this->log_ACTIVITY_HI_ATINET_PrmUpdated(id);
    }

  }

  void ATINetboxComponentImpl::ATINET_DEBUG_cmdHandler(
                                                       FwOpcodeType opCode, //!< The opcode
                                                       U32 cmdSeq) {

    char msg[256];

    snprintf(msg, sizeof(msg), "a1: %f", this->m_a1);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "a2: %f", this->m_a2);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "b0: %f", this->m_b0);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "b1: %f", this->m_b1);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "b2: %f", this->m_b2);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "b2: %f", this->m_b2);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR11: %f", this->m_mR11);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR12: %f", this->m_mR12);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR13: %f", this->m_mR13);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR21: %f", this->m_mR21);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR22: %f", this->m_mR22);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR23: %f", this->m_mR23);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR31: %f", this->m_mR31);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR32: %f", this->m_mR32);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mR33: %f", this->m_mR33);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBFX: %f", this->m_mBFX);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBFY: %f", this->m_mBFY);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBFZ: %f", this->m_mBFZ);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBTX: %f", this->m_mBTX);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBTY: %f", this->m_mBTY);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "mBTZ: %f", this->m_mBTZ);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "cpf: %f", this->m_cpf);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "cpt: %f", this->m_cpt);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "PX: %f", this->m_PX);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "PY: %f", this->m_PY);
    this->logDebug(msg);

    snprintf(msg, sizeof(msg), "PZ: %f", this->m_PZ);
    this->logDebug(msg);

    for(int i = 0; i < FT_AA_FILT_SIZE; i++) {
      snprintf(msg, sizeof(msg), "aa[%d]: %.15e", i, this->m_aa[i]);
      this->logDebug(msg);
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void ATINetboxComponentImpl::ATINET_TARE_cmdHandler(FwOpcodeType opCode,
                                                      U32 cmdSeq, U32 cycles) {

    this->log_ACTIVITY_HI_ATINET_TareStarted(cycles, this->m_mBFX, this->m_mBFY,
                                             this->m_mBFZ, this->m_mBTX, this->m_mBTY, this->m_mBTZ);

    this->m_tareCycles = cycles;
    this->m_tareCyclesLeft = cycles;

    this->m_forceXAve = 0.0;
    this->m_forceYAve = 0.0;
    this->m_forceZAve = 0.0;

    this->m_torqueXAve = 0.0;
    this->m_torqueYAve = 0.0;
    this->m_torqueZAve = 0.0;

    this->m_doTare = true;

    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void ATINetboxComponentImpl::logWarning(const char *msg) {

    Fw::LogStringArg arg(msg);
    this->log_WARNING_HI_ATINET_Err_Msg(arg);
  }

  void ATINetboxComponentImpl::logDebug(const char *msg) {

    Fw::LogStringArg arg(msg);
    this->log_ACTIVITY_HI_ATINET_Msg(arg);
  }

} // close namespace
