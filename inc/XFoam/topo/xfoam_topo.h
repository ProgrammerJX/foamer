#ifndef XFoam_Topo_H_
#define XFoam_Topo_H_

// 便捷聚合头：把虚拓扑层与两套底层 brep 实现一次性引入。
// 单独使用时按需 include 三者中的任意子集即可（编译期更少）。

#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/topo/xfoam_vbrep.h"
#include "XFoam/topo/xfoam_mbrep.h"

#endif // XFoam_Topo_H_
