#ifndef TFoam_Common_H_
#define TFoam_Common_H_

#include <boost/filesystem.hpp>

/// XFoam 模块根目录（其下含 `data/`、`tmp/`）。在任意 `tests/<子目录>/*.cpp` 中传入 `__FILE__`。
inline boost::filesystem::path XFoamTests_moduleRoot(const char* testSourceFile)
{
	return boost::filesystem::path(testSourceFile).parent_path().parent_path().parent_path().lexically_normal();
}

inline boost::filesystem::path XFoamTests_dataDir(const char* testSourceFile)
{
	return XFoamTests_moduleRoot(testSourceFile) / "data";
}

inline boost::filesystem::path XFoamTests_tmpDir(const char* testSourceFile)
{
	return XFoamTests_moduleRoot(testSourceFile) / "tmp";
}

#endif
