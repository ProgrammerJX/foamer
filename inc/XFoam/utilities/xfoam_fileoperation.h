#ifndef XFoam_FileOperation_H_
#define XFoam_FileOperation_H_

// 对标 OpenFOAM fileHandler：串行读 regIOobject 头与数据体（见 regIOobjectRead.C）。

#include "XFoam/utilities/xfoam_stream.h"

class XFoam_FileName;
class XFoam_RegIOobject;
class XFoam_Word;

class XFoam_API XFoam_FileHandler
{
public:
	bool readHeader(
		XFoam_RegIOobject& obj,
		const XFoam_FileName& fName,
		const XFoam_Word& typeName) const;

	bool read(
		XFoam_RegIOobject& obj,
		bool masterOnly,
		XFoam_IOstream::streamFormat defaultFormat,
		const XFoam_Word& typeNameArg) const;
};

inline XFoam_FileHandler& XFoam_fileHandler()
{
	static XFoam_FileHandler instance;
	return instance;
}

#endif
