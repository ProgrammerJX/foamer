#ifndef TFoam_Common_H_
#define TFoam_Common_H_

#include <boost/filesystem.hpp>

/// XFoam 模块根目录（其下含 `data/`、`tmp/`）。在任意 `tests/<子目录>/*.cpp` 中传入 `__FILE__`。
/// 命名：foam_code.md 全局函数为 `XFoam_` + lowerCamel。
inline boost::filesystem::path XFoam_testsModuleRoot(const char* testSourceFile)
{
	return boost::filesystem::path(testSourceFile).parent_path().parent_path().parent_path().lexically_normal();
}

inline boost::filesystem::path XFoam_testsDataDir(const char* testSourceFile)
{
	return XFoam_testsModuleRoot(testSourceFile) / "data";
}

inline boost::filesystem::path XFoam_testsTmpDir(const char* testSourceFile)
{
	return XFoam_testsModuleRoot(testSourceFile) / "tmp";
}

#endif
