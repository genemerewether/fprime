/*
 * PrmList.hpp
 *
 *  Created on: Feb 13, 2019
 *      Author: kubiak
 */

/*
 * Description:
 * This object contains the ParamList type, used for storing multiple parameters
 */
#ifndef FW_PRM_LIST_HPP
#define FW_PRM_LIST_HPP

#include <Fw/Cfg/Config.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Serializable.hpp>
#include <Fw/Cfg/SerIds.hpp>

namespace Fw {

    class ParamList : public SerializeBufferBase {
        public:

            enum {
                SERIALIZED_TYPE_ID = FW_TYPEID_PRM_BUFF,
                SERIALIZED_SIZE = FW_PARAM_LIST_MAX_SIZE + sizeof(FwBuffSizeType)
            };

            ParamList(const U8 *args, NATIVE_UINT_TYPE size);
            ParamList();
            ParamList(const ParamList& other);
            virtual ~ParamList();
            const ParamList& operator=(const ParamList& other);

            NATIVE_UINT_TYPE getBuffCapacity(void) const; // !< returns capacity, not current size, of buffer
            U8* getBuffAddr(void);
            const U8* getBuffAddr(void) const;

        private:
            U8 m_data[FW_PARAM_LIST_MAX_SIZE]; // command argument buffer
    };

}

#endif
