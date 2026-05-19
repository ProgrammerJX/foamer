#ifndef XFoam_Time_H_
#define XFoam_Time_H_

// 对标 OpenFOAM db/Time/Time.H 的最小子集；Time 作为顶层 objectRegistry（与 OF 一致：Time 继承 objectRegistry）。
// 路径成员在 objectRegistry 基类构造之后由 XFoam_Time 构造函数体初始化。

#include "XFoam/utilities/xfoam_ioobject.h"

class XFoam_API XFoam_Time : public XFoam_ObjectRegistry
{
	XFoam_FileName rootPath_;
	XFoam_FileName caseName_;
	XFoam_FileName globalCaseName_;
	XFoam_FileName timeName_;
	XFoam_FileName constant_;
	XFoam_FileName system_;

public:
	XFoam_Time();

	const XFoam_FileName& rootPath() const { return rootPath_; }

	const XFoam_FileName& caseName() const { return caseName_; }

	const XFoam_FileName& globalCaseName() const { return globalCaseName_; }

	/// 当前时间目录名（对标 Time 目录字符串）；与 IOobject::name()（word）区分。
	const XFoam_FileName& timeName() const { return timeName_; }

	const XFoam_FileName& constant() const { return constant_; }

	const XFoam_FileName& system() const { return system_; }

	// 对标 OpenFOAM Time::globalPath()（用于 systemDictIO 等）。
	XFoam_FileName globalPath() const { return rootPath_ / globalCaseName_; }

	// 未移植：controlDict / runTimeModifiable 全量；恒 false。
	bool runTimeModifiable() const { return false; }

	XFoam_IOstream::streamFormat writeFormat() const { return XFoam_IOstream::ASCII; }

	XFoam_IOstream::compressionType writeCompression() const { return XFoam_IOstream::UNCOMPRESSED; }
};

#endif
