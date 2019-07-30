/**
 * \file   SnapdragonHealthComponentImpl.cpp
 * \author Gorang Gandhi
 * \brief  Component responsible for monitoring Snapdragon health
 *
 * \copyright
 * Copyright 2009-2016, by the California Institute of Technology.
 * ALL RIGHTS RESERVED.  United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 * <br /><br />
 * This software may be subject to U.S. export control laws and
 * regulations.  By accepting this document, the user agrees to comply
 * with all U.S. export laws and regulations.  User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */

#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImpl.hpp>
#include <string.h>
#include <stdlib.h>
#include <Os/File.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

extern "C" {
    // Defined in version.c
    extern const unsigned int FSW_HASH;
    extern const char* FSW_BRANCH;
}

#if BUILD_UT
    const unsigned int FSW_HASH=0x1bf18d920;
    const char* FSW_BRANCH = "Rel3.0-AltMgr";
#endif

    // ----------------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------------

#if FW_OBJECT_NAMES == 1
    SnapdragonHealthComponentImpl::SnapdragonHealthComponentImpl(const char* name) :
        SnapdragonHealthComponentBase(name),
#else
        SnapdragonHealthComponentImpl::SnapdragonHealthComponentImpl() :
#endif
        temp_zone_stat(),
        temp_zone_temp(),
        gov_setpoint(),
        freq_setpoint(),
        l2_power_setpoint(L2_POWER_TYPE_4),
        total_tick_old(),
        total_cpu_util(0),
        cpu_util(),
        cpu_freq(),
        proc_file(NULL),
        fsw_version_sent(false),
        power_saver(SAVER_ON),
        m_boot_count(0u),
        m_schedCalls(0u)
    {
        memset(this->total_tick_old,0,sizeof(this->total_tick_old));
        memset(this->idle_old,0,sizeof(this->idle_old));

        // This will be overridden in Sched call
        for (NATIVE_UINT_TYPE i = 0; i < FW_NUM_ARRAY_ELEMENTS(this->gov_setpoint); i++) {
            this->gov_setpoint[i] = GOVERNOR_TYPE_USERSPACE;
        }

        for (NATIVE_UINT_TYPE i = 0; i < FW_NUM_ARRAY_ELEMENTS(this->freq_setpoint); i++) {
            this->freq_setpoint[i] = FREQ_TYPE_2GHZ;
        }
    }

    SnapdragonHealthComponentImpl::~SnapdragonHealthComponentImpl()
    {
        // Close the file if needed:
        if (this->proc_file != NULL) {
            (void) fclose(this->proc_file);
        }
    }

    void SnapdragonHealthComponentImpl::init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance)
    {
        SnapdragonHealthComponentBase::init(queueDepth, instance);
    }

    void SnapdragonHealthComponentImpl::preamble()
    {

        Os::File::Status stat = Os::File::OTHER_ERROR;

        // Open the stat file for CPU utilization:
        this->proc_file = fopen(health_file_path(PROC_FILE_TYPE), "r");

        if (this->proc_file == NULL) {

            this->log_WARNING_LO_SH_FileOpenError(PROC_FILE_TYPE,stat);
        }
    }

    void SnapdragonHealthComponentImpl::setBootCount(U32 count) {
        this->m_boot_count = count;
    }

    void SnapdragonHealthComponentImpl::setInitPowerState(SH_PowerSaverMode mode) {
        switch (mode) {
            case SH_SAVER_ON:
                this->power_saver = SAVER_ON;
                break;
            case SH_SAVER_DYNAMIC:
                this->power_saver = SAVER_DYNAMIC;
                break;
            case SH_SAVER_OFF:
                this->power_saver = SAVER_OFF;
                break;
            case SH_SAVER_RAPID:
                this->power_saver = SAVER_RAPID;
                break;
            default:
                FW_ASSERT(0, mode);
                break;
        }
    }
  
    // ----------------------------------------------------------------------
    // Helper Methods
    // ----------------------------------------------------------------------

    const char* SnapdragonHealthComponentImpl ::
      health_file_path(HealthFileType type) {
        const char* const CONST_PATH_ARRAY[HEALTH_FILE_TYPE_MAX
                                           - HEALTH_FILE_TYPE_MIN + 1] = {
            "/sys/devices/virtual/thermal/thermal_zone0/temp",
            "/sys/devices/virtual/thermal/thermal_zone1/temp",
            "/sys/devices/virtual/thermal/thermal_zone2/temp",
            "/sys/devices/virtual/thermal/thermal_zone3/temp",
            "/sys/devices/virtual/thermal/thermal_zone4/temp",
            "/sys/devices/virtual/thermal/thermal_zone5/temp",
            "/sys/devices/virtual/thermal/thermal_zone6/temp",
            "/sys/devices/virtual/thermal/thermal_zone7/temp",
            "/sys/devices/virtual/thermal/thermal_zone8/temp",
            "/sys/devices/virtual/thermal/thermal_zone9/temp",
            "/sys/devices/virtual/thermal/thermal_zone10/temp",
            "/sys/devices/virtual/thermal/thermal_zone11/temp",
            "/sys/devices/virtual/thermal/thermal_zone12/temp",
            "/sys/devices/virtual/thermal/thermal_zone13/temp",
            "/sys/devices/virtual/thermal/thermal_zone14/temp",
            "/sys/devices/virtual/thermal/thermal_zone15/temp",
            "/sys/devices/virtual/thermal/thermal_zone16/temp",

            "/sys/devices/virtual/thermal/thermal_zone0/mode",
            "/sys/devices/virtual/thermal/thermal_zone1/mode",
            "/sys/devices/virtual/thermal/thermal_zone2/mode",
            "/sys/devices/virtual/thermal/thermal_zone3/mode",
            "/sys/devices/virtual/thermal/thermal_zone4/mode",
            "/sys/devices/virtual/thermal/thermal_zone5/mode",
            "/sys/devices/virtual/thermal/thermal_zone6/mode",
            "/sys/devices/virtual/thermal/thermal_zone7/mode",
            "/sys/devices/virtual/thermal/thermal_zone8/mode",
            "/sys/devices/virtual/thermal/thermal_zone9/mode",
            "/sys/devices/virtual/thermal/thermal_zone10/mode",
            "/sys/devices/virtual/thermal/thermal_zone11/mode",
            "/sys/devices/virtual/thermal/thermal_zone12/mode",
            "/sys/devices/virtual/thermal/thermal_zone13/mode",
            "/sys/devices/virtual/thermal/thermal_zone14/mode",
            "/sys/devices/virtual/thermal/thermal_zone15/mode",
            "/sys/devices/virtual/thermal/thermal_zone16/mode",

            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
            "/sys/devices/system/cpu/cpu1/cpufreq/scaling_cur_freq",
            "/sys/devices/system/cpu/cpu2/cpufreq/scaling_cur_freq",
            "/sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq",

            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor",
            "/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor",
            "/sys/devices/system/cpu/cpu2/cpufreq/scaling_governor",
            "/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor",

            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed",
            "/sys/devices/system/cpu/cpu1/cpufreq/scaling_setspeed",
            "/sys/devices/system/cpu/cpu2/cpufreq/scaling_setspeed",
            "/sys/devices/system/cpu/cpu3/cpufreq/scaling_setspeed",

            "/proc/stat",

            "/sys/module/lpm_levels/enable_low_power/l2"
        };

        FW_ASSERT((type >= HEALTH_FILE_TYPE_MIN) &&
                  (type <= HEALTH_FILE_TYPE_MAX),
                  static_cast<NATIVE_INT_TYPE>(type));

        return CONST_PATH_ARRAY[type];
    }

    const char* SnapdragonHealthComponentImpl ::
      governor_string(GovernorType type) {
        const char* const CONST_PATH_ARRAY[GOVERNOR_TYPE_MAX
                                           - GOVERNOR_TYPE_MIN + 1] = {
            "performance",
            "powersave",
            "userspace",
            "conservative",
            "ondemand",
            "interactive"
        };

        FW_ASSERT((type >= GOVERNOR_TYPE_MIN) &&
                  (type <= GOVERNOR_TYPE_MAX),
                  static_cast<NATIVE_INT_TYPE>(type));

        return CONST_PATH_ARRAY[type];
    }

    const char* SnapdragonHealthComponentImpl ::
      freq_string(FreqType type) {
        const char* const CONST_PATH_ARRAY[FREQ_TYPE_MAX
                                           - FREQ_TYPE_MIN + 1] = {
            "300000",
            "2265600"
        };

        FW_ASSERT((type >= FREQ_TYPE_MIN) &&
                  (type <= FREQ_TYPE_MAX),
                  static_cast<NATIVE_INT_TYPE>(type));

        return CONST_PATH_ARRAY[type];
    }

    const char* SnapdragonHealthComponentImpl ::
      l2_power_string(L2PowerType type) {
        const char* const CONST_PATH_ARRAY[L2_POWER_TYPE_MAX
                                           - L2_POWER_TYPE_MIN + 1] = {
            "4",
            "0"
        };

        FW_ASSERT((type >= L2_POWER_TYPE_MIN) &&
                  (type <= L2_POWER_TYPE_MAX),
                  static_cast<NATIVE_INT_TYPE>(type));

        return CONST_PATH_ARRAY[type];
    }

    void SnapdragonHealthComponentImpl::load_from_file(HealthFileType type)
    {
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;

        FW_ASSERT(((type >= TEMP_FILE_TYPE_MIN)      &&
                   (type <= TEMP_FILE_TYPE_MAX))         ||
                  ((type >= TEMP_MODE_FILE_TYPE_MIN) &&
                   (type <= TEMP_MODE_FILE_TYPE_MAX))    ||
                  ((type >= FREQ_FILE_TYPE_MIN)      &&
                   (type <= FREQ_FILE_TYPE_MAX)),
                  static_cast<NATIVE_INT_TYPE>(type));

        DEBUG_PRINT("SnapHealth read FILE_TYPE %d, path %s\n",
                    type, health_file_path(type));

        // Open file
        stat = file.open(health_file_path(type), Os::File::OPEN_READ);

        if (stat != Os::File::OP_OK) {
            this->log_WARNING_LO_SH_FileOpenError(type,stat);
        }
        // Open was successful:
        else {
            // Read:
            char read_buff[FW_LOG_STRING_MAX_SIZE];
            NATIVE_INT_TYPE size = 10;
            stat = file.read(read_buff, size, false);

            // Make sure its null-terminated:
            read_buff[sizeof(read_buff)-1] = 0;

            if (stat != Os::File::OP_OK || size == 0) {
                this->log_WARNING_LO_SH_FileReadError(type,stat);
            }
            // Read was successful:
            else {
                if ((type >= TEMP_FILE_TYPE_MIN) &&
                    (type <= TEMP_FILE_TYPE_MAX)) {

                     NATIVE_INT_TYPE temp = atoi(read_buff);
                     FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->temp_zone_temp)
                               + TEMP_FILE_TYPE_MIN > type,
                               static_cast<NATIVE_INT_TYPE>(type));
                     this->temp_zone_temp[type - TEMP_FILE_TYPE_MIN] = temp;

                     // NOTE(mereweth) - unit test must reflect these transformations
#ifdef SOC_8096 //TODO(mereweth) - confirm temperature sensor locations on 8096
                     if (TEMP_0_FILE_TYPE == type) {
                         this->temp_zone_temp[type - TEMP_FILE_TYPE_MIN] *= 0.001;
                     }
                     else {
                         this->temp_zone_temp[type - TEMP_FILE_TYPE_MIN] *= 0.1;
                     }
#else // 8074
                     if ((TEMP_11_FILE_TYPE == type) ||
                         (TEMP_12_FILE_TYPE == type)) {
                         this->temp_zone_temp[type - TEMP_FILE_TYPE_MIN] *= 0.001;
                     }
#endif
                }
                else if ((type >= TEMP_MODE_FILE_TYPE_MIN) &&
                         (type <= TEMP_MODE_FILE_TYPE_MAX)) {
                    FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->temp_zone_stat)
                              + TEMP_MODE_FILE_TYPE_MIN > type,
                              static_cast<NATIVE_INT_TYPE>(type));
                    if (strncmp(read_buff,"enabled",7) == 0) {
                        this->temp_zone_stat[type - TEMP_MODE_FILE_TYPE_MIN] = true;
                    }
                    else if (strncmp(read_buff,"disabled",8) == 0) {
                        this->temp_zone_stat[type - TEMP_MODE_FILE_TYPE_MIN] = false;
                    }
                    // TODO may want to make it a error stat if
                    // doesnt match either above
                    else {
                        // For now just dont update the state
                        //(void) printf("<<< %s\n", read_buff);
                    }
                }
                else if ((type >= FREQ_FILE_TYPE_MIN) &&
                         (type <= FREQ_FILE_TYPE_MAX)) {

                     NATIVE_INT_TYPE temp = atoi(read_buff);
                     FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->cpu_freq)
                               + FREQ_FILE_TYPE_MIN > type,
                               static_cast<NATIVE_INT_TYPE>(type));
                     this->cpu_freq[type - FREQ_FILE_TYPE_MIN] = temp;
                }
                else {
                    //NOTE(mereweth) - assert above should keep us from getting here
                    FW_ASSERT(0, static_cast<NATIVE_INT_TYPE>(type));
                }
            }
        }

        // Close:
        file.close();
    }

    bool SnapdragonHealthComponentImpl::read_proc_stat_fields(U64 *fields)
    {
        const NATIVE_INT_TYPE BUF_MAX = 1024;
        char buffer[BUF_MAX];

        // Read a line of the /stat/proc file:
        if (fgets(buffer, BUF_MAX, this->proc_file) == NULL) {

            this->log_WARNING_LO_SH_FileReadError(PROC_FILE_TYPE,Os::File::OTHER_ERROR);
            return false;
        }

        // Line starts with c and a string. This is to handle cpu, cpu[0-9]+
        NATIVE_INT_TYPE retval = sscanf(buffer, "c%*s %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
                                        &fields[0],
                                        &fields[1],
                                        &fields[2],
                                        &fields[3],
                                        &fields[4],
                                        &fields[5],
                                        &fields[6],
                                        &fields[7],
                                        &fields[8],
                                        &fields[9]);

        // Make sure we read the correct number of fields:
        if (retval < MAX_NUM_FIELDS)
        {
            this->log_WARNING_LO_SH_FileReadError(PROC_FILE_TYPE,Os::File::OTHER_ERROR);
            return false;
        }

        return true;
    }


    void SnapdragonHealthComponentImpl::calculate_cpu_utilization()
    {

        U64 fields[MAX_NUM_FIELDS];
        U64 total_tick[MAX_NUM_CPUS];
        U64 idle[MAX_NUM_CPUS];

        // Clear data:
        memset(total_tick,0,sizeof(total_tick));
        memset(idle,0,sizeof(idle));
        memset(fields,0,sizeof(fields));

        // Couldnt open the file in the preamble():
        if (this->proc_file == NULL) {
            return;
        }

        // Seek the file to the start:
        NATIVE_INT_TYPE stat1 = fseek(this->proc_file, 0, SEEK_SET);
        NATIVE_INT_TYPE stat2 = fflush(this->proc_file);  // Doesnt refresh the file w/o the flush

        // If there was a error with the seek:
        if (stat1 != 0 || stat2 != 0) {

            this->log_WARNING_LO_SH_FileReadError(PROC_FILE_TYPE,stat1 != 0 ? stat1 : stat2);
            return;
        }

        // Loop through the fields of each core, where the first line is for
        // all of the CPU cores:
        U32 cpus = 0;
        while (cpus < MAX_NUM_CPUS && this->read_proc_stat_fields(fields))
        {
            // Get the total tick sum:
            for (U32 i = 0; i < MAX_NUM_CPUS; ++i)
            {
                total_tick[cpus] += fields[i];
            }
            idle[cpus] = fields[3]; // idle ticks index
            ++cpus;
        }

        // Sanity check:
        FW_ASSERT(cpus <= MAX_NUM_CPUS, cpus);

        // Calculate the CPU utilization over the last second for the all
        // cores and each core:
        F32 percent_usage;
        U64 del_total_tick;
        U64 del_idle;
        for (U32 count = 0; count < cpus; ++count)
        {
            // Get the deltas from the last cycle:
            // These should always be positive or zero, but wont assert on it
            // These values indicates the measure of time since the system was booted, which
            // is why they will stay the same or increase.
            del_total_tick = total_tick[count] - this->total_tick_old[count];
            del_idle = idle[count] - this->idle_old[count];

            // The delta can be zero, in the case that proc/stat file hasnt been updated since
            // the last cycle.
            // The other conditions should always hold, but not asserting on it in case its a
            // linux issue.
            if (del_total_tick > 0 && del_idle > 0 && del_total_tick >= del_idle) {

                percent_usage = ( (del_total_tick - del_idle)/(static_cast<F64>(del_total_tick)) ) * 100;
            }
            // If delta did not change, then didnt use the core at all:
            else if (del_total_tick == 0) {

                percent_usage = 0;
            }
            // Otherwise its a abnormal case
            else {
                continue;
            }

            if (count == 0) {
                this->total_cpu_util = percent_usage;
            }
            else if (count - 1 < FW_NUM_ARRAY_ELEMENTS(this->cpu_util)) {
                this->cpu_util[count - 1] = percent_usage;
            }
            else {
                FW_ASSERT(0, count);
            }

            // Save off the old values:
            this->total_tick_old[count] = total_tick[count];
            this->idle_old[count] = idle[count];
        }
    }

    void SnapdragonHealthComponentImpl::write_to_file(HealthFileType type,
                                                      const char* const string)
    {
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;

        DEBUG_PRINT("SnapHealth write FILE_TYPE %d, path %s\n, %s\n",
                    type, health_file_path(type), string);

        stat = file.open(health_file_path(type), Os::File::OPEN_WRITE);
        if (stat != Os::File::OP_OK) {
            this->log_WARNING_LO_SH_FileOpenError(type, stat);
        }
        // Open was successful:
        else {
            // Write:
            NATIVE_INT_TYPE size = strlen(string);
            NATIVE_INT_TYPE bytesWritten = size;
            stat = file.write(string, bytesWritten, true);
            if ((size != bytesWritten)
                || (stat != Os::File::OP_OK)) {
                this->log_WARNING_HI_SH_FileWriteError(type, stat);
            }
        }

        // Close:
        file.close();
    }

    // ----------------------------------------------------------------------
    // Handlers to implement for typed input ports
    // ----------------------------------------------------------------------


    void SnapdragonHealthComponentImpl ::
      SnapdragonHealth_SetPowerSaver_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          PowerSaverMode Mode
      )
    {
        this->power_saver = Mode;
        switch (this->power_saver) {
            case SAVER_ON:
                this->log_ACTIVITY_HI_SH_PowerSaver(SAVER_ON_EVR);
                break;
            case SAVER_DYNAMIC:
                this->log_ACTIVITY_HI_SH_PowerSaver(SAVER_DYNAMIC_EVR);
                break;
            case SAVER_OFF:
                this->log_ACTIVITY_HI_SH_PowerSaver(SAVER_OFF_EVR);
                break;
            case SAVER_RAPID:
                this->log_ACTIVITY_HI_SH_PowerSaver(SAVER_RAPID_EVR);
                break;
            default:
                FW_ASSERT(0, static_cast<NATIVE_INT_TYPE>(this->power_saver));
        }

        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
    }

    void SnapdragonHealthComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        this->pingOut_out(0,key);
    }

    void SnapdragonHealthComponentImpl::SchedIn_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context)
    {

        // TODO only open the temperature files once in preamble()? Perhaps, only if the number
        // of temperature zones does not vary.

        if (!this->fsw_version_sent) {

            Fw::LogStringArg branch(FSW_BRANCH);
            this->log_ACTIVITY_HI_SH_FswVersion(FSW_HASH, branch);
            this->fsw_version_sent = true;
        }

        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
        COMPILE_TIME_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->gov_setpoint) >= 4,
                            SH_ENOUGH_GOV_SETPOINTS);
        COMPILE_TIME_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->freq_setpoint) >= 4,
                            SH_ENOUGH_FREQ_SETPOINTS);
        #pragma GCC diagnostic pop

        if (SH_SCHED_COUNTDOWN == m_schedCalls) {
            m_schedCalls++;
        }

        //Fw::PolyType polySetVal;
        PowerSaverModeTlm saverModeTlm;
        switch (this->power_saver) {
            case SAVER_ON:
                saverModeTlm = SAVER_ON_TLM;
                //polySetVal = Common::COMMON_POLYDB_SAVER_ON;
                this->gov_setpoint[0] = GOVERNOR_TYPE_CONSERVATIVE;
                this->gov_setpoint[1] = GOVERNOR_TYPE_POWERSAVE;
                this->gov_setpoint[2] = GOVERNOR_TYPE_POWERSAVE;
                this->gov_setpoint[3] = GOVERNOR_TYPE_POWERSAVE;

                this->l2_power_setpoint = L2_POWER_TYPE_4;
                break;
            case SAVER_DYNAMIC:
                saverModeTlm = SAVER_DYNAMIC_TLM;
                //polySetVal = Common::COMMON_POLYDB_SAVER_DYNAMIC;
                this->gov_setpoint[0] = GOVERNOR_TYPE_CONSERVATIVE;
                this->gov_setpoint[1] = GOVERNOR_TYPE_CONSERVATIVE;
                this->gov_setpoint[2] = GOVERNOR_TYPE_CONSERVATIVE;
                this->gov_setpoint[3] = GOVERNOR_TYPE_CONSERVATIVE;

                this->l2_power_setpoint = L2_POWER_TYPE_0;
                break;
            case SAVER_OFF:
                saverModeTlm = SAVER_OFF_TLM;
                //polySetVal = Common::COMMON_POLYDB_SAVER_OFF;
                this->gov_setpoint[0] = GOVERNOR_TYPE_USERSPACE;
                this->gov_setpoint[1] = GOVERNOR_TYPE_USERSPACE;
                this->gov_setpoint[2] = GOVERNOR_TYPE_USERSPACE;
                this->gov_setpoint[3] = GOVERNOR_TYPE_USERSPACE;

                this->freq_setpoint[0] = FREQ_TYPE_2GHZ;
                this->freq_setpoint[1] = FREQ_TYPE_2GHZ;
                this->freq_setpoint[2] = FREQ_TYPE_2GHZ;
                this->freq_setpoint[3] = FREQ_TYPE_2GHZ;

                this->l2_power_setpoint = L2_POWER_TYPE_0;
                break;
            case SAVER_RAPID:
                saverModeTlm = SAVER_RAPID_TLM;
                this->tlmWrite_SnapdragonPowerSaver(SAVER_RAPID_TLM);
                //polySetVal = Common::COMMON_POLYDB_SAVER_OFF;
                this->gov_setpoint[0] = GOVERNOR_TYPE_PERFORMANCE;
                this->gov_setpoint[1] = GOVERNOR_TYPE_PERFORMANCE;
                this->gov_setpoint[2] = GOVERNOR_TYPE_PERFORMANCE;
                this->gov_setpoint[3] = GOVERNOR_TYPE_PERFORMANCE;

                this->l2_power_setpoint = L2_POWER_TYPE_0;
                break;
            default:
                FW_ASSERT(0, static_cast<NATIVE_INT_TYPE>(this->power_saver));
        }
        /*Svc::MeasurementStatus measureStat = Svc::MEASUREMENT_OK;
        Fw::Time polySetTime = this->getTime();
        if (isConnected_PowerReport_OutputPort(0)) {
            this->PowerReport_out(0, Common::COMMON_POLYDB_POWERSAVE_STATE,
                                  measureStat,
                                  polySetTime,
                                  polySetVal);
                                  }*/


        if (m_schedCalls >= SH_SCHED_COUNTDOWN) {
            this->tlmWrite_SnapdragonPowerSaver(saverModeTlm);

            for (NATIVE_UINT_TYPE i = GOV_FILE_TYPE_MIN; i <= GOV_FILE_TYPE_MAX; i++) {
                FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->gov_setpoint)
                          + GOV_FILE_TYPE_MIN > i,
                          static_cast<NATIVE_INT_TYPE>(i));
                FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->freq_setpoint)
                          + GOV_FILE_TYPE_MIN > i,
                          static_cast<NATIVE_INT_TYPE>(i));

                this->write_to_file(static_cast<HealthFileType>(i),
                                    governor_string(this->gov_setpoint[i - GOV_FILE_TYPE_MIN]));
                if (this->gov_setpoint[i - GOV_FILE_TYPE_MIN] == GOVERNOR_TYPE_USERSPACE) {
                    this->write_to_file(static_cast<HealthFileType>(i - GOV_FILE_TYPE_MIN + FREQ_SET_FILE_TYPE_MIN),
                                        freq_string(this->freq_setpoint[i - GOV_FILE_TYPE_MIN]));
                }
            }

#ifndef SOC_8096 // TODO(mereweth) - find L2 cache power level file on 8096
            this->write_to_file(L2_POWER_FILE_TYPE,
                                l2_power_string(this->l2_power_setpoint));
#endif
        }
        else {
            this->tlmWrite_SnapdragonPowerSaver(SAVER_UNSET_TLM);
            m_schedCalls++;
        }

        // Open files to read temperatures and modes:
        //
        // Note, the temperature files are not on the MAC OS
        //
        for (NATIVE_UINT_TYPE i = TEMP_FILE_TYPE_MIN; i <= TEMP_FILE_TYPE_MAX; i++) {
            this->load_from_file(static_cast<HealthFileType>(i));
        }
	/*
        for (NATIVE_UINT_TYPE i = TEMP_MODE_FILE_TYPE_MIN; i <= TEMP_MODE_FILE_TYPE_MAX; i++) {
            this->load_from_file(static_cast<HealthFileType>(i));
        }
	*/
        for (NATIVE_UINT_TYPE i = FREQ_FILE_TYPE_MIN; i <= FREQ_FILE_TYPE_MAX; i++) {
            this->load_from_file(static_cast<HealthFileType>(i));
        }

        // Calculate CPU utilization:
        //
        // Code based off of: https://phoxis.org/2013/09/05/finding-overall-and-per-core-cpu-utilization/
        //
        this->calculate_cpu_utilization();

        /*measureStat = Svc::MEASUREMENT_OK;
        polySetTime = this->getTime();
        polySetVal = (F32) FW_MAX(FW_MAX(this->temp_zone_temp[5],
                                         this->temp_zone_temp[6]),
                                  FW_MAX(this->temp_zone_temp[7],
                                         this->temp_zone_temp[8]));
        if (isConnected_PowerReport_OutputPort(0)) {
            this->PowerReport_out(0, Common::COMMON_POLYDB_SDTEMP,
                                  measureStat,
                                  polySetTime,
                                  polySetVal);
                                  }*/

        // Push telemetry
        this->tlmWrite_SnapTempZone0DspCpuStat(this->temp_zone_stat[0]);
        this->tlmWrite_SnapTempZone0DspCpuTemp(this->temp_zone_temp[0]);
        this->tlmWrite_SnapTempZone1DspModemStat(this->temp_zone_stat[1]);
        this->tlmWrite_SnapTempZone1DspModemTemp(this->temp_zone_temp[1]);
        this->tlmWrite_SnapTempZone2ModemStat(this->temp_zone_stat[2]);
        this->tlmWrite_SnapTempZone2ModemTemp(this->temp_zone_temp[2]);
        this->tlmWrite_SnapTempZone3VideoStat(this->temp_zone_stat[3]);
        this->tlmWrite_SnapTempZone3VideoTemp(this->temp_zone_temp[3]);
        this->tlmWrite_SnapTempZone4WlanCpuStat(this->temp_zone_stat[4]);
        this->tlmWrite_SnapTempZone4WlanCpuTemp(this->temp_zone_temp[4]);
        this->tlmWrite_SnapTempZone5Cpu0Stat(this->temp_zone_stat[5]);
        this->tlmWrite_SnapTempZone5Cpu0Temp(this->temp_zone_temp[5]);
        this->tlmWrite_SnapTempZone6Cpu1Stat(this->temp_zone_stat[6]);
        this->tlmWrite_SnapTempZone6Cpu1Temp(this->temp_zone_temp[6]);
        this->tlmWrite_SnapTempZone7Cpu2Stat(this->temp_zone_stat[7]);
        this->tlmWrite_SnapTempZone7Cpu2Temp(this->temp_zone_temp[7]);
        this->tlmWrite_SnapTempZone8Cpu3Stat(this->temp_zone_stat[8]);
        this->tlmWrite_SnapTempZone8Cpu3Temp(this->temp_zone_temp[8]);
        this->tlmWrite_SnapTempZone9GpuAStat(this->temp_zone_stat[9]);
        this->tlmWrite_SnapTempZone9GpuATemp(this->temp_zone_temp[9]);
        this->tlmWrite_SnapTempZone10GpuBStat(this->temp_zone_stat[10]);
        this->tlmWrite_SnapTempZone10GpuBTemp(this->temp_zone_temp[10]);

        this->tlmWrite_SnapTempZone11PM8841Stat(this->temp_zone_stat[11]);
        this->tlmWrite_SnapTempZone11PM8841Temp(0.001 *
                                                this->temp_zone_temp[11]);
        this->tlmWrite_SnapTempZone12PM8941Stat(this->temp_zone_stat[12]);
        this->tlmWrite_SnapTempZone12PM8941Temp(0.001 *
                                                this->temp_zone_temp[12]);

        /*this->tlmWrite_SnapTempZone13PA0Stat(this->temp_zone_stat[13]);
        this->tlmWrite_SnapTempZone13PA0Temp(this->temp_zone_temp[13]);
        this->tlmWrite_SnapTempZone14PA1Stat(this->temp_zone_stat[14]);
        this->tlmWrite_SnapTempZone14PA1Temp(this->temp_zone_temp[14]);*/
        this->tlmWrite_SnapTempZone15EmmcStat(this->temp_zone_stat[15]);
        this->tlmWrite_SnapTempZone15EmmcTemp(this->temp_zone_temp[15]);
        this->tlmWrite_SnapTempZone16MsmStat(this->temp_zone_stat[16]);
        this->tlmWrite_SnapTempZone16MsmTemp(this->temp_zone_temp[16]);

        this->tlmWrite_SnapdragonTotalCpuUtil(this->total_cpu_util);
        this->tlmWrite_SnapdragonTotalCpuUtilTrunc((U8) this->total_cpu_util);
        this->tlmWrite_SnapdragonCpu0Util(this->cpu_util[0]);
        this->tlmWrite_SnapdragonCpu1Util(this->cpu_util[1]);
        this->tlmWrite_SnapdragonCpu2Util(this->cpu_util[2]);
        this->tlmWrite_SnapdragonCpu3Util(this->cpu_util[3]);

        this->tlmWrite_SnapdragonCpu0Freq(this->cpu_freq[0]);
        this->tlmWrite_SnapdragonCpu1Freq(this->cpu_freq[1]);
        this->tlmWrite_SnapdragonCpu2Freq(this->cpu_freq[2]);
        this->tlmWrite_SnapdragonCpu3Freq(this->cpu_freq[3]);

        this->tlmWrite_SnapdragonBootCount(this->m_boot_count);
        this->tlmWrite_SH_FswVersion(FSW_HASH);

    }

} // end namespace
