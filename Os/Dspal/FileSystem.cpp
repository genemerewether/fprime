#include <Fw/Cfg/Config.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/FileSystem.hpp>
#include <Os/File.hpp>
#include <Fw/Types/Assert.hpp>

#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <stdio.h> // Needed for rename
#include <string.h>
#include <limits>

namespace Os {

	namespace FileSystem {

		Status createDirectory(const char* path) {
		  return OTHER_ERROR;
		} // end createDirectory

		Status removeDirectory(const char* path) {
		  return OTHER_ERROR;
		} // end removeDirectory

		Status readDirectory(const char* path, const U32 maxNum,
							 Fw::EightyCharString fileArray[])
		{
		  return OTHER_ERROR;
		}

		Status removeFile(const char* path) {
		  return OTHER_ERROR;
		} // end moveFile

		Status handleFileError(File::Status fileStatus) {
			Status fileSystemStatus = OTHER_ERROR;

			switch(fileStatus) {
				case File::NO_SPACE:
					fileSystemStatus = NO_SPACE;
					break;
				case File::NO_PERMISSION:
					fileSystemStatus = NO_PERMISSION;
					break;
				case File::DOESNT_EXIST:
					fileSystemStatus = INVALID_PATH;
					break;
				default:
					fileSystemStatus = OTHER_ERROR;
			}
			return fileSystemStatus;
		} // end handleFileError

		Status copyFile(const char* originPath, const char* destPath) {
		  return OTHER_ERROR;
		} // end copyFile

		Status getFileSize(const char* path, U64& size) {
		  return OTHER_ERROR;
		} // end getFileSize

		Status changeWorkingDirectory(const char* path) {
		  return OTHER_ERROR;
		} // end changeWorkingDirectory

		
		// Public function to get the file count for a given directory.
		Status getFileCount (const char* directory, U32& fileCount) {
		  return OTHER_ERROR;
		} //end getFileCount

	} // end FileSystem namespace 

} // end Os namespace
