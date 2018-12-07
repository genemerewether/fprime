#include <Fw/Cfg/Config.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/File.hpp>
#include <Fw/Types/Assert.hpp>

#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <string.h>

namespace Os {

    File::File() :m_fd(0),m_mode(OPEN_NO_MODE),m_lastError(0) {
    }

    File::~File() {
        if (this->m_mode != OPEN_NO_MODE) {
            this->close();
        }
    }

    File::Status File::open(const char* fileName, File::Mode mode) {
        return OTHER_ERROR;
    }

    File::Status File::seek(NATIVE_INT_TYPE offset, bool absolute) {
        return OTHER_ERROR;
    }

    File::Status File::read(void * buffer, NATIVE_INT_TYPE &size, bool waitForFull) {
        return OTHER_ERROR;
    }

    File::Status File::write(const void * buffer, NATIVE_INT_TYPE &size, bool waitForDone) {
        return OTHER_ERROR;
    }

    void File::close(void) {
    }

    NATIVE_INT_TYPE File::getLastError(void) {
        return this->m_lastError;
    }

    const char* File::getLastErrorString(void) {
        return strerror(this->m_lastError);
    }

}
