/**
 * \file
 * \author T.Canham
 * \brief Component for managing parameters
 *
 * \copyright
 * Copyright 2009-2015, by the California Institute of Technology.
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

#ifndef PRMDBIMPL_HPP_
#define PRMDBIMPL_HPP_

#include <iterator>
#include <Svc/PrmDb/PrmDbImplCfg.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <Os/Mutex.hpp>
#include <Fw/Prm/PrmGetPortAc.hpp>

namespace Svc {

    // anonymous namespace for buffer declaration
    namespace {
        class PrmDbWorkingBuffer : public Fw::SerializeBufferBase {
            public:
  
                NATIVE_UINT_TYPE getBuffCapacity(void) const {
                    return sizeof(m_buff);
                }

                U8* getBuffAddr(void) {
                    return m_buff;
                }

                const U8* getBuffAddr(void) const {
                    return m_buff;
                }

            private:
                // Set to max of parameter buffer + id
                U8 m_buff[FW_PARAM_BUFFER_MAX_SIZE + sizeof(FwPrmIdType)];
        };
    }

    //! \class PrmDbImpl
    //! \brief Component class for managing parameters
    //!
    //! This component supports storing, setting and saving of serialized parameters
    //! for components.
    //!

    class PrmDbImpl {


        private:

            struct t_dbStruct {
                bool used; //!< whether slot is being used
                FwPrmIdType id; //!< the id being stored in the slot
                Fw::ParamBuffer val; //!< the serialized value of the parameter
            } m_db[PRMDB_NUM_DB_ENTRIES];

        public:

            friend class PrmDbImplTester;
            friend class Tester;

            enum SetPrmStatus {
                SET_PRM_UPDATED,
                SET_PRM_ADDED,
                SET_PRM_FULL
            };

            enum SerializePrmStatus {
                SERIALIZE_PRM_OK,
                SERIALIZE_PRM_NOT_VALID
            };

            enum DeserializePrmStatus {
                DESERIALIZE_PRM_OK,

            };

            //!  \brief PrmDb constructor
            //!
            //!  The constructor for the PrmDbImpl class.
            //!   The constructor clears the database and stores
            //!   the file name for opening later.
            //!
            //!  \param name component instance name
            //!  \param file file where parameters are stored.
            PrmDbImpl();


            //!  \brief PrmDb destructor
            //!
            virtual ~PrmDbImpl();

            //!  \brief PrmDb parameter get handler
            //!
            //!  This function retrieves a parameter value from the loaded set of stored parameters
            //!
            //!  \param id identifier for parameter being used.
            //!  \param val buffer where value is placed.
            //!  \return status of retrieval. PARAM_VALID = successful read, PARAM_INVALID = unsuccessful read
            Fw::ParamValid getPrm(const FwPrmIdType id, Fw::ParamBuffer &val);

            //!  \brief PrmDb parameter set handler
            //!
            //!  This function updates the value of the parameter stored in RAM. The PRM_SAVE_FILE
            //!  must be called to save the value to a file.
            //!
            //!  \param id identifier for parameter being used.
            //!  \param val buffer where value to be saved is stored.
            //!  \return status of setting the parameter
            PrmDbImpl::SetPrmStatus setPrm(const FwPrmIdType id, Fw::ParamBuffer &val);

            //!  \brief PrmDb clear database function
            //!
            //!  This function clears all entries from the RAM database
            //!

            void clearDb(void); //!< clear the parameter database

            SerializePrmStatus serializePrmSize(const FwPrmIdType id, NATIVE_INT_TYPE& size);

            //!  \brief PrmDb parameter serialize helper function
            //!
            //!  This function serializes a parameter into a provided buffer
            //!
            //!  \param id identifier for the parameter being used
            //!  \param buff Buffer to serialize parameter into
            //!  \return status of parameter serialization
            SerializePrmStatus serializePrm(const FwPrmIdType id, Fw::SerializeBufferBase& buff);

            DeserializePrmStatus deserializePrm(Fw::SerializeBufferBase& buff, FwPrmIdType& id, Fw::ParamBuffer& prm);


            class PrmDbIterator : public std::iterator<
                                                std::input_iterator_tag,
                                                const FwPrmIdType,
                                                ptrdiff_t,
                                                const FwPrmIdType*,
                                                const FwPrmIdType> {

                friend class PrmDbImpl;

                public:

                    PrmDbIterator() :
                        db(nullptr),
                        entry(PRMDB_NUM_DB_ENTRIES) {
                    }

                    PrmDbIterator(const PrmDbIterator& iter) :
                        db(iter.db),
                        entry(iter.entry) {
                    }

                    ~PrmDbIterator() {};

                    PrmDbIterator& operator=(const PrmDbIterator& other) {
                        this->db = other.db;
                        this->entry = other.entry;
                        return *this;
                    }
                    PrmDbIterator& operator++() { //Prefix

                        this->advanceToNextEntry();

                        return *this;
                    }

                    PrmDbIterator operator++(int) { //Postfix

                        PrmDbIterator tmp(*this);
                        operator++();
                        return tmp;
                    }

                    FwPrmIdType operator*() const {
                        return this->db[this->entry].id;
                    }

                    const FwPrmIdType* operator->() const {
                        return &this->db[this->entry].id;
                    }

                    friend bool operator==(const PrmDbIterator& lhs, const PrmDbIterator& rhs) {
                        // Assumes that we are always pointing to the same PrmDb
                        return lhs.entry == rhs.entry &&
                               lhs.db == rhs.db;
                    }

                    friend bool operator!=(const PrmDbIterator& lhs, const PrmDbIterator& rhs) {
                        // Assumes that we are always pointing to the same PrmDb
                        return !(lhs == rhs);
                    }

                    bool valid(void) {
                        return this->db != nullptr &&
                               this->entry >= 0 &&
                               this->entry < PRMDB_NUM_DB_ENTRIES;
                    }

                private:

                    PrmDbIterator(const PrmDbImpl& prmDb, const NATIVE_INT_TYPE startEntry);

                    void advanceToNextEntry();

                    const struct t_dbStruct* db;
                    NATIVE_INT_TYPE entry;
            };

            PrmDbIterator begin() {
                return PrmDbIterator(*this, 0);
            };

            PrmDbIterator end() {
                return PrmDbIterator(*this, PRMDB_NUM_DB_ENTRIES);
            }
    };
}

#endif /* PRMDBIMPL_HPP_ */
