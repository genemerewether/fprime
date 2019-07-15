#include <Fw/Prm/PrmList.hpp>
#include <Fw/Types/Assert.hpp>

namespace Fw {

	ParamList::ParamList(const U8 *args, NATIVE_UINT_TYPE size) {
	    SerializeStatus stat = SerializeBufferBase::setBuff(args,size);
        FW_ASSERT(FW_SERIALIZE_OK == stat,static_cast<NATIVE_INT_TYPE>(stat));
    }
    
	ParamList::ParamList() {
    }

    ParamList::~ParamList() {
    }

    ParamList::ParamList(const ParamList& other) : Fw::SerializeBufferBase() {
        SerializeStatus stat = SerializeBufferBase::setBuff(other.m_data,other.getBuffLength());
        FW_ASSERT(FW_SERIALIZE_OK == stat,static_cast<NATIVE_INT_TYPE>(stat));
	}

	const ParamList& ParamList::operator=(const ParamList& other) {
	    SerializeStatus stat = SerializeBufferBase::setBuff(other.m_data,other.getBuffLength());
        FW_ASSERT(FW_SERIALIZE_OK == stat,static_cast<NATIVE_INT_TYPE>(stat));
        return other;
	}

    NATIVE_UINT_TYPE ParamList::getBuffCapacity(void) const {
        return sizeof(this->m_data);
    }

    const U8* ParamList::getBuffAddr(void) const {
        return this->m_data;
    }

    U8* ParamList::getBuffAddr(void) {
        return this->m_data;
    }

}

