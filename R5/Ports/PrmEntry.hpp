/*
 * PrmEntry.hpp
 *
 *  Created on: Nov 15, 2017
 *      Author: tcanham
 */

#ifndef R5_PORTS_PRMENTRY_HPP_
#define R5_PORTS_PRMENTRY_HPP_

#include "Fw/Types/Serializable.hpp"
#include "Fw/Types/PolyType.hpp"
#include "Fw/Types/Assert.hpp"

namespace R5
{
    class PrmEntry: public Fw::Serializable
    {
    public:

        // Maximum number of parameters is found by solving this equation:
        // storageSize >= (# prms) * (4 * 3) + ceil((# prms)/8) + 4
        //   where a PolyType takes 3 words to store
        //         eight parameters bitmasks can be stored in a byte
        //         one word is used for the CRC32 for the parameters
        //
        // If storageSize is 4096 bytes, then the maximum number of parameters
        // is 337

        // Not serializing for now, but set the size if we decide to in the future
        static const NATIVE_UINT_TYPE SERIALIZED_SIZE = Fw::PolyType::SERIALIZED_SIZE + sizeof(U16);

        PrmEntry() : m_id(0) {}
        virtual ~PrmEntry() {}

        void set(U16 id, const Fw::PolyType& val) {
            this->m_id = id;
            this->m_val = val;
        }

        void get(U16 &id, Fw::PolyType& val) {
            id  = this->m_id;
            val = this->m_val;
        }

        Fw::SerializeStatus serialize(Fw::SerializeBufferBase& buffer) const {
            FW_ASSERT(0); // shouldn't be called
            return Fw::FW_DESERIALIZE_FORMAT_ERROR; // for compiler warnings
        }

        Fw::SerializeStatus deserialize(Fw::SerializeBufferBase& buffer) {
            FW_ASSERT(0); // shouldn't be called
            return Fw::FW_DESERIALIZE_FORMAT_ERROR; // for compiler warnings
        }

        Fw::PolyType& val(void) {
            return m_val;
        }

    PRIVATE:

        Fw::PolyType m_val;
        U16 m_id;
    };

} /* namespace R5 */

#endif /* R5_PORTS_PRMENTRY_HPP_ */
