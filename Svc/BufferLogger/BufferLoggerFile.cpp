// ======================================================================
// \title  BufferLoggerFile.cpp
// \author bocchino, dinkel, mereweth
// \brief  Implementation for Svc::BufferLogger::BufferLoggerFile
//
// \copyright
// Copyright (C) 2015-2017 California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include "Svc/BufferLogger/BufferLogger.hpp"
#include "Svc/BufferLogger/BufferLoggerCfg.hpp"
#include "Os/ValidateFile.hpp"
#include "Os/ValidatedFile.hpp"

#include <stdio.h>
#include <sys/time.h>
//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Constructors and destructors
  // ----------------------------------------------------------------------

  BufferLogger::File ::
    File(
        BufferLogger& bufferLogger
    ) :
      bufferLogger(bufferLogger),
      prefix(""),
      suffix(""),
      baseName(""),
      baseNameBeenSet(false),
      initBeenCalled(false),
      fileCounter(0u),
      maxSize(0u),
      sizeOfSize(0u),
      mode(Mode::CLOSED),
      bytesWritten(0u),
      writeMode(BL_WRITE_MODE_MAX),
      closeMode(BL_CLOSE_SYNC),
      directChunkSize(0u),
      tempBuffer()
  {
  }

  BufferLogger::File ::
    ~File(void)
  {
    this->close();
  }

  // ----------------------------------------------------------------------
  // Public functions
  // ----------------------------------------------------------------------

  void BufferLogger::File ::
    init(
        const char *const logFilePrefix,
        const char *const logFileSuffix,
        const U32 maxFileSize,
        const U8 sizeOfSize,
        const BufferLoggerFileMode writeMode,
        const BufferLoggerCloseMode closeMode,
        const U32 directChunkSize
    )
  {
      //NOTE(mereweth) - only call this before opening the file
      FW_ASSERT(this->mode == File::Mode::CLOSED);

      this->initBeenCalled = true;

      this->prefix = logFilePrefix;
      this->suffix = logFileSuffix;
      this->maxSize = maxFileSize;
      this->sizeOfSize = sizeOfSize;
      this->writeMode = writeMode;
      this->closeMode = closeMode;
      this->directChunkSize = directChunkSize;

      FW_ASSERT(sizeOfSize <= sizeof(U32), sizeOfSize);
      FW_ASSERT(maxSize > sizeOfSize, maxSize);
  }

  void BufferLogger::File ::
    setBaseName(
        const Fw::EightyCharString& baseName
    )
  {
      if (this->mode == File::Mode::OPEN) {
          this->closeAndEmitEvent();
      }
      this->baseNameBeenSet = true;
      this->baseName = baseName;
      this->fileCounter = 0;
  }

  void BufferLogger::File ::
    logBuffer(
        const U8 *const data,
        const U32 size
    )
  {
    // Close the file if it will be too big
    if (this->mode == File::Mode::OPEN) {
        U32 projectedByteCount = 0;
        switch (this->writeMode) {
            case BL_DIRECT_WRITE:
            {
                // TODO(mereweth) - is there a more efficient way to write size of size?
                U32 ceilSizeOfSize = ((this->sizeOfSize / this->directChunkSize)
                                      + 1) * this->directChunkSize;
                U32 ceilSize = ((this->sizeOfSize / this->directChunkSize)
                                + 1) * this->directChunkSize;
                projectedByteCount = this->bytesWritten + ceilSizeOfSize + ceilSize;
                break;
            }
            case BL_BULK_WRITE:
            case BL_LOOPING_WRITE:
            case BL_REGULAR_WRITE:
                projectedByteCount =
                    this->bytesWritten + this->sizeOfSize + size;
                break;
            default:
                DEBUG_PRINT("Unhandled BL file mode %d in logBuffer\n", this->writeMode);
                FW_ASSERT(0, static_cast<U32>(this->writeMode));
                return;
        }
        if (projectedByteCount > this->maxSize) {
            this->closeAndEmitEvent();
        }
    }
    // Open a file if necessary
    if (this->mode == File::Mode::CLOSED) {
      this->open();
    }
    // Write to the file if it is open
    if (this->mode == File::Mode::OPEN) {
      (void) this->writeBuffer(data, size);
    }
  }

  void BufferLogger::File ::
    closeAndEmitEvent(void)
  {
    if (this->mode == File::Mode::OPEN) {
      this->flush();

      struct timeval tv;
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("BL closing file at time %f\n",
                  tv.tv_sec + tv.tv_usec / 1000000.0);

      this->close();

      gettimeofday(&tv,NULL);
      DEBUG_PRINT("BL done closing file at time %f\n",
                  tv.tv_sec + tv.tv_usec / 1000000.0);

      Fw::LogStringArg logStringArg(this->name.toChar());
      this->bufferLogger.log_DIAGNOSTIC_BL_LogFileClosed(logStringArg);
    }
  }

  // ----------------------------------------------------------------------
  // Private functions
  // ----------------------------------------------------------------------

  void BufferLogger::File ::
    open(void)
  {
    FW_ASSERT(this->mode == File::Mode::CLOSED);

    struct timeval tv;
    gettimeofday(&tv,NULL);
    DEBUG_PRINT("BL opening file at time %f\n",
                tv.tv_sec + tv.tv_usec / 1000000.0);

    // NOTE(mereweth) - check that file path has been set and that initLog has been called
    if ((!baseNameBeenSet)    ||
        (!initBeenCalled)) {
        this->bufferLogger.log_WARNING_HI_BL_NoLogFileOpenInitError();
        return;
    }

    this->name.format(
        "%s%s%07d%s",
        this->prefix.toChar(),
        this->baseName.toChar(),
        this->fileCounter,
        this->suffix.toChar()
    );

    Os::File::Mode fileMode;
    switch (this->writeMode) {
        case BL_DIRECT_WRITE:
            fileMode = Os::File::OPEN_SYNC_DIRECT_WRITE;
            break;
        case BL_BULK_WRITE:
        case BL_LOOPING_WRITE:
            fileMode = Os::File::OPEN_SYNC_WRITE;
            break;
        case BL_REGULAR_WRITE:
            fileMode = Os::File::OPEN_WRITE;
            break;
        default:
            DEBUG_PRINT("Unhandled BL file mode %d in open\n", this->writeMode);
            FW_ASSERT(0, static_cast<U32>(this->writeMode));
            return;
    }

    Os::File::Status status = this->osFile.open(
        this->name.toChar(),
        fileMode
    );
    if (status == Os::File::OP_OK) {
      this->fileCounter++;
      // Reset bytes written
      this->bytesWritten = 0;
      // Set mode
      this->mode = File::Mode::OPEN;
    }
    else {
      Fw::LogStringArg string(this->name.toChar());
      this->bufferLogger.log_WARNING_HI_BL_LogFileOpenError(status, string);
    }

#ifdef BL_PREALLOC
    if (status == Os::File::OP_OK) {
        status = this->osFile.prealloc(0, this->maxSize);
        // TODO(mereweth) - EVR
        //if (status != Os::File::OP_OK)
    }
#endif // BL_PREALLOC

    gettimeofday(&tv,NULL);
    DEBUG_PRINT("BL done opening file at time %f\n",
                tv.tv_sec + tv.tv_usec / 1000000.0);
  }

  bool BufferLogger::File ::
    writeBuffer(
        const U8 *const data,
        const U32 size
    )
  {
    bool status = this->writeSize(size);
    if (status) {
      status = this->writeBytes(data, size);
    }
    return status;
  }

  bool BufferLogger::File ::
    writeSize(const U32 size)
  {
    U32 sizeRegister = size;
    U8 sizeBuffer[this->sizeOfSize];
    for (U8 i = 0; i < this->sizeOfSize; ++i) {
      sizeBuffer[this->sizeOfSize - i - 1] = sizeRegister & 0xFF;
      sizeRegister >>= 8;
    }
    const bool status = this->writeBytes(
        sizeBuffer,
        sizeof(sizeBuffer)
    );
    return status;
  }

  bool BufferLogger::File ::
    writeBytes(
        const void *const data,
        const NATIVE_UINT_TYPE length
    )
  {
    FW_ASSERT(length > 0, length);

    switch (this->writeMode) {
        case BL_REGULAR_WRITE:
        case BL_LOOPING_WRITE:
        {
            NATIVE_UINT_TYPE chunkSize = 0;
            if (this->writeMode == BL_LOOPING_WRITE) { // multiple Os::File::write calls
                chunkSize = this->directChunkSize;
            }
            else if (this->writeMode == BL_REGULAR_WRITE) { // one Os::File::write call
                chunkSize = length;
            }
            for (NATIVE_UINT_TYPE idx = 0; idx < length; idx += chunkSize) {
                NATIVE_INT_TYPE size = chunkSize;
                // if we're on the last iteration and length isn't a multiple of chunkSize
                if (idx + chunkSize > length) {
                    size = length - idx;
                }
                const NATIVE_INT_TYPE toWrite = size;
                const Os::File::Status fileStatus = this->osFile.write(data, size);
                if (fileStatus == Os::File::OP_OK && size == static_cast<NATIVE_INT_TYPE>(toWrite)) {
                  this->bytesWritten += toWrite;
                }
                else {
                  Fw::LogStringArg string(this->name.toChar());

                  this->bufferLogger.log_WARNING_HI_BL_LogFileWriteError(fileStatus, size, toWrite, string);
                  return false;
                }
            }
            return true;

            break;
        }
        case BL_BULK_WRITE:
        {
            const NATIVE_UINT_TYPE chunkSize = static_cast<NATIVE_UINT_TYPE>(BL_CHUNK_SIZE);
            NATIVE_UINT_TYPE size = length;
            const Os::File::Status fileStatus = this->osFile.bulkWrite(data, size, chunkSize);
            bool status;
            if (fileStatus == Os::File::OP_OK && size == length) {
              this->bytesWritten += length;
              status = true;
            }
            else {
              Fw::LogStringArg string(this->name.toChar());

              this->bufferLogger.log_WARNING_HI_BL_LogFileWriteError(fileStatus, size, length, string);
              status = false;
            }
            return status;

            break;
        }
        case BL_DIRECT_WRITE:
        {
            // NOTE(mereweth) - init won't succeed if this is true; this is just to protect ourselves
            FW_ASSERT(BL_MAX_DIRECT_CHUNK_SIZE >= this->directChunkSize, this->directChunkSize);

            const NATIVE_UINT_TYPE alignedSize = (length / this->directChunkSize) * this->directChunkSize;
            const NATIVE_UINT_TYPE alignedRem = length - alignedSize;
            NATIVE_INT_TYPE size = 0;

            if (alignedSize) {
                size = alignedSize;
                const NATIVE_INT_TYPE toWrite = size;
                const Os::File::Status fileStatus = this->osFile.write(data, size);
                if (fileStatus == Os::File::OP_OK && size == static_cast<NATIVE_INT_TYPE>(toWrite)) {
                    this->bytesWritten += toWrite;
                }
                else {
                    Fw::LogStringArg string(this->name.toChar());

                    this->bufferLogger.log_WARNING_HI_BL_LogFileWriteError(fileStatus, size, toWrite, string);
                    return false;
                }
            }

            if (alignedRem) {
                BYTE* alignedBuff = (BYTE*) (((U64) this->tempBuffer
                                              | (this->directChunkSize - 1)) + 1);
                FW_ASSERT((this->tempBuffer + FW_NUM_ARRAY_ELEMENTS(this->tempBuffer))
                          >= (alignedBuff + alignedRem), (U64) alignedBuff, alignedRem);

                memset(alignedBuff, 0, alignedRem);
                memcpy(alignedBuff,
                       static_cast<const void*>(static_cast<const BYTE*>(data) + alignedSize),
                       alignedRem);

                size = this->directChunkSize;
                const NATIVE_INT_TYPE toWrite = size;
                const Os::File::Status fileStatus = this->osFile.write(alignedBuff, size);
                if (fileStatus == Os::File::OP_OK && size == static_cast<NATIVE_INT_TYPE>(toWrite)) {
                    this->bytesWritten += toWrite;
                }
                else {
                    Fw::LogStringArg string(this->name.toChar());

                    this->bufferLogger.log_WARNING_HI_BL_LogFileWriteError(fileStatus, size, toWrite, string);
                    return false;
                }
            }

            return true;

            break;
        }
        default:
            DEBUG_PRINT("Unhandled BL file mode %d in writeBytes\n", this->writeMode);
            return false;
    }
  }

  void BufferLogger::File ::
    writeHashFile(void)
  {
    Os::ValidatedFile validatedFile(this->name.toChar());
    const Os::ValidateFile::Status status =
      validatedFile.createHashFile();
    if (status !=  Os::ValidateFile::VALIDATION_OK) {
      const Fw::EightyCharString &hashFileName = validatedFile.getHashFileName();
      Fw::LogStringArg logStringArg(hashFileName.toChar());
      this->bufferLogger.log_WARNING_HI_BL_LogFileValidationError(
          logStringArg,
          status
      );
    }
  }

  bool BufferLogger::File ::
  flush(void)
  {
    bool status = true;
    if(this->mode == File::Mode::OPEN)
    {
      struct timeval tv;
      gettimeofday(&tv,NULL);
      DEBUG_PRINT("BL flushing file at time %f\n",
                  tv.tv_sec + tv.tv_usec / 1000000.0);

      const Os::File::Status fileStatus = this->osFile.flush();

      gettimeofday(&tv,NULL);
      DEBUG_PRINT("BL done flushing file at time %f\n",
                  tv.tv_sec + tv.tv_usec / 1000000.0);

      if(fileStatus == Os::File::OP_OK)
      {
        status = true;
      }
      else
      {
        status = false;
        Fw::LogStringArg string(this->name.toChar());
        this->bufferLogger.log_WARNING_HI_BL_FileFlushError(string, fileStatus);
      }
    }
    return status;
  }

  void BufferLogger::File ::
    close(void)
  {
    if (this->mode == File::Mode::OPEN) {
      // Close file
      if (this->closeMode == BL_CLOSE_SYNC) {
          this->osFile.close();
      }
      // Write out the hash file to disk
      this->writeHashFile();
      // Update mode
      this->mode = File::Mode::CLOSED;
    }
  }

}
