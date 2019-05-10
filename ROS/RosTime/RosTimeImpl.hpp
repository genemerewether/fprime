#ifndef ROSTIMEIMPL_HPP_
#define ROSTIMEIMPL_HPP_

#include <Svc/Time/TimeComponentAc.hpp>

namespace ROS {

class RosTimeImpl: public Svc::TimeComponentBase {
    public:
#if FW_OBJECT_NAMES == 1
        RosTimeImpl(const char* compName);
#else
        RosTimeImpl();
#endif
        virtual ~RosTimeImpl();
        void init(NATIVE_INT_TYPE instance);
    protected:
        void timeGetPort_handler(
                NATIVE_INT_TYPE portNum, /*!< The port number*/
                Fw::Time &time /*!< The U32 cmd argument*/
            );
    private:
};

}

#endif /* ROSTIMEIMPL_HPP_ */
