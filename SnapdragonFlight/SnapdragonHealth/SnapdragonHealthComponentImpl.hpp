/**
 * \file   SnapdragonHealthComponentImpl.hpp
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


#ifndef SNAPDRAGONHEALTHCOMPONENTIMPL_HPP_
#define SNAPDRAGONHEALTHCOMPONENTIMPL_HPP_

#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImplCfg.hpp>
#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentAc.hpp>
#include <Os/File.hpp>


namespace SnapdragonFlight {

    //! \class SnapdragonHealthComponentImpl
    //! \brief Snapdragon temp component class
    //!
    //! This component is responsible for monitoring the temperature
    //! of the Snapdragon board.  It checks all of the temperature zones
    //! every second, and pushed them to telemetry.
    //!

    class SnapdragonHealthComponentImpl : public SnapdragonHealthComponentBase {

        public:

            //!  \brief Component constructor
            //!
            //!  The constructor initializes the state of the component.
            //!
            //!  Note: Making constructor explicit to prevent implicit
            //!        type conversion.
            //!
            //!  \param name the component instance name
#if FW_OBJECT_NAMES == 1
            explicit SnapdragonHealthComponentImpl(const char* name);
#else
            SnapdragonHealthComponentImpl();
#endif

            //!  \brief Component initialization routine
            //!
            //!  The initialization function calls the initialization
            //!  routine for the base class.
            //!
            //!  \param queueDepth the depth of the message queue for the component
            void init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance);

            //!  \brief Component destructor
            //!
            virtual ~SnapdragonHealthComponentImpl();

            //!  \brief Component preamble routine
            //!
            virtual void preamble();

            //!  \brief Set the boot count
            void setBootCount(U32 count);

            //! Set the power state to enter after sched call countdown
            //! Default is SAVER_ON
            void setInitPowerState(SH_PowerSaverMode mode);

        PROTECTED:

        PRIVATE:

            // ----------------------------------------------------------------------
            // Constants/Types
            // ----------------------------------------------------------------------

            /*
             * \brief File types
             * Note(mereweth) - from the standard:  If the first enumerator
             * has no initializer, the value of the corresponding constant is
             * zero. An enumerator-definition without an initializer gives the
             * enumerator the value obtained by increasing the value of the
             * previous enumerator by one.
             *
             * So we can use these in order, starting at 0
             */
            enum HealthFileType {
                HEALTH_FILE_TYPE_MIN,
                TEMP_FILE_TYPE_MIN = HEALTH_FILE_TYPE_MIN,
                TEMP_0_FILE_TYPE = TEMP_FILE_TYPE_MIN,
                TEMP_1_FILE_TYPE,
                TEMP_2_FILE_TYPE,
                TEMP_3_FILE_TYPE,
                TEMP_4_FILE_TYPE,
                TEMP_5_FILE_TYPE,
                TEMP_6_FILE_TYPE,
                TEMP_7_FILE_TYPE,
                TEMP_8_FILE_TYPE,
                TEMP_9_FILE_TYPE,
                TEMP_10_FILE_TYPE,
                TEMP_11_FILE_TYPE,
                TEMP_12_FILE_TYPE,
                TEMP_13_FILE_TYPE,
                TEMP_14_FILE_TYPE,
                TEMP_15_FILE_TYPE,
                TEMP_16_FILE_TYPE,
                TEMP_FILE_TYPE_MAX = TEMP_16_FILE_TYPE,

                TEMP_MODE_FILE_TYPE_MIN,
                TEMP_MODE_0_FILE_TYPE = TEMP_MODE_FILE_TYPE_MIN,
                TEMP_MODE_1_FILE_TYPE,
                TEMP_MODE_2_FILE_TYPE,
                TEMP_MODE_3_FILE_TYPE,
                TEMP_MODE_4_FILE_TYPE,
                TEMP_MODE_5_FILE_TYPE,
                TEMP_MODE_6_FILE_TYPE,
                TEMP_MODE_7_FILE_TYPE,
                TEMP_MODE_8_FILE_TYPE,
                TEMP_MODE_9_FILE_TYPE,
                TEMP_MODE_10_FILE_TYPE,
                TEMP_MODE_11_FILE_TYPE,
                TEMP_MODE_12_FILE_TYPE,
                TEMP_MODE_13_FILE_TYPE,
                TEMP_MODE_14_FILE_TYPE,
                TEMP_MODE_15_FILE_TYPE,
                TEMP_MODE_16_FILE_TYPE,
                TEMP_MODE_FILE_TYPE_MAX = TEMP_MODE_16_FILE_TYPE,

                FREQ_FILE_TYPE_MIN,
                FREQ_0_FILE_TYPE = FREQ_FILE_TYPE_MIN,
                FREQ_1_FILE_TYPE,
                FREQ_2_FILE_TYPE,
                FREQ_3_FILE_TYPE,
                FREQ_FILE_TYPE_MAX = FREQ_3_FILE_TYPE,

                GOV_FILE_TYPE_MIN,
                GOV_0_FILE_TYPE = GOV_FILE_TYPE_MIN,
                GOV_1_FILE_TYPE,
                GOV_2_FILE_TYPE,
                GOV_3_FILE_TYPE,
                GOV_FILE_TYPE_MAX = GOV_3_FILE_TYPE,

                FREQ_SET_FILE_TYPE_MIN,
                FREQ_SET_0_FILE_TYPE = FREQ_SET_FILE_TYPE_MIN,
                FREQ_SET_1_FILE_TYPE,
                FREQ_SET_2_FILE_TYPE,
                FREQ_SET_3_FILE_TYPE,
                FREQ_SET_FILE_TYPE_MAX = FREQ_SET_3_FILE_TYPE,

                PROC_FILE_TYPE,

                L2_POWER_FILE_TYPE,
                HEALTH_FILE_TYPE_MAX = L2_POWER_FILE_TYPE
            };

            enum GovernorType {
                GOVERNOR_TYPE_MIN,
                GOVERNOR_TYPE_PERFORMANCE = GOVERNOR_TYPE_MIN,
                GOVERNOR_TYPE_POWERSAVE,
                GOVERNOR_TYPE_USERSPACE,
                GOVERNOR_TYPE_CONSERVATIVE,
                GOVERNOR_TYPE_ONDEMAND,
                GOVERNOR_TYPE_INTERACTIVE,
                GOVERNOR_TYPE_MAX = GOVERNOR_TYPE_INTERACTIVE
            };

            enum FreqType {
                FREQ_TYPE_MIN,
                FREQ_TYPE_300MHZ = FREQ_TYPE_MIN,
                FREQ_TYPE_2GHZ,
                FREQ_TYPE_MAX = FREQ_TYPE_2GHZ
            };

            enum L2PowerType {
                L2_POWER_TYPE_MIN,
                L2_POWER_TYPE_4 = L2_POWER_TYPE_MIN,
                L2_POWER_TYPE_0,
                L2_POWER_TYPE_MAX = L2_POWER_TYPE_0
            };

            /*
             * \brief Max number of proc/stat fields
             *
             */
            static const I32 MAX_NUM_FIELDS = 10;

            /*
             * \brief Max number of CPU cores+1
             *
             */
            static const U32 MAX_NUM_CPUS = 5;

            // ----------------------------------------------------------------------
            // Helper Functions
            // ----------------------------------------------------------------------

            static const char* health_file_path(HealthFileType type);

            static const char* governor_string(GovernorType type);

            static const char* freq_string(FreqType type);

            static const char* l2_power_string(L2PowerType type);

            // ----------------------------------------------------------------------
            // Member Functions
            // ----------------------------------------------------------------------

            /*
             * \brief Open specified temperature file and process data
             *
             * \param type The file type - TEMP_FILE, TEMP_MODE_FILE, or FREQ_FILE
             */
            void load_from_file(HealthFileType type);

            void write_to_file(HealthFileType type, const char* const string);

            /*
             * \brief Calculate the CPU utilization since the last cycle
             *
             */
            void calculate_cpu_utilization();

            /*
             * \brief Read the proc/stat file
             *
             * \param fields Populates with each field read from the proc/stat file
             * \return T=true if there is good read/scan, otherwise false
             */
            bool read_proc_stat_fields(U64 *fields);

            // ----------------------------------------------------------------------
            // Handlers to implement for typed input ports
            // ----------------------------------------------------------------------
            void SnapdragonHealth_SetPowerSaver_cmdHandler(
                const FwOpcodeType opCode,
                const U32 cmdSeq,
                PowerSaverMode Mode
            );

            //! Handler implementation for pingIn
            //!
            void pingIn_handler(
                const NATIVE_INT_TYPE portNum, /*!< The port number*/
                U32 key /*!< Value to return to pinger*/
            );



            //! Handler for input port SchedIn
            //
            virtual void SchedIn_handler(
                NATIVE_INT_TYPE portNum, /*!< The port number*/
                NATIVE_UINT_TYPE context /*!< The call order*/
            );

            // ----------------------------------------------------------------------
            // Member Variables
            // ----------------------------------------------------------------------

            /*
             * \brief Temperature zone states (true=enabled, false=disabled)
             */
            bool temp_zone_stat[TEMP_MODE_FILE_TYPE_MAX - TEMP_MODE_FILE_TYPE_MIN + 1];

            /*
             * \brief Temperature zone temperatures
             */
            I32 temp_zone_temp[TEMP_FILE_TYPE_MAX - TEMP_FILE_TYPE_MIN + 1];

            GovernorType gov_setpoint[GOV_FILE_TYPE_MAX - GOV_FILE_TYPE_MIN + 1];

            FreqType freq_setpoint[FREQ_SET_FILE_TYPE_MAX - FREQ_SET_FILE_TYPE_MIN + 1];

            L2PowerType l2_power_setpoint;

            /*
             * \brief Previous total tick count
             */
            U64 total_tick_old[MAX_NUM_CPUS];

            /*
             * \brief Previous idle tick count
             */
            U64 idle_old[MAX_NUM_CPUS];

            /*
             * \brief CPU Utilization Percentages
             */
            F32 total_cpu_util;
            F32 cpu_util[MAX_NUM_CPUS - 1];

            /*
             * \brief CPU Scaling Frequencies
             */
            U32 cpu_freq[FREQ_FILE_TYPE_MAX - FREQ_FILE_TYPE_MIN + 1];

            /*
             * \brief /proc/stat file
             */
            FILE* proc_file;

            /*
             * \brief Flag to indicate if FSW version EVR was sent
             */
            bool fsw_version_sent;

            /*
             * \brief Whether power-saving features are on
             */
            PowerSaverMode power_saver;

            /*
             *\brief boot count
             */
            U32 m_boot_count;

            // Number of Sched calls - used for triggering switch into initial power state
            U32 m_schedCalls;
    };
}

#endif /* SNAPDRAGONHEALTHCOMPONENTIMPL_HPP_ */
