# 项目现状与缺口审计

本文档基于 `b638b01` 提交（foam-refactor 基线导入）整理。后续每一次重要重构请同步更新本文件，避免重复审计。

## 1. 模块完成度速览

| 模块 | 规模 | 状态 | 关键 TODO 数 |
|---|---|---|---|
| `utilities/` (28 头 + 16 源) | ~10 k 行 | OK 基本完整 | 散落多处，不阻塞 |
| `primitive/` (8 几何图元) | 中等 | OK 基本完整 | 1–2 |
| `mesh/Shape` | 1060 行 | OK 基本完整 | 1 |
| `mesh/PrimitiveMesh` | 188 行 | OK 完整 | 0 |
| `mesh/PolyMesh` | 1015 行 | 警告 第二构造函数 SIGSEGV（多块） | 1 |
| `mesh/PolyBoundaryMesh/Patch` | 628 行 | OK 基本完整 | 2 |
| `topo/`, `tools/` | 中等 | OK 完整 | 0 |
| `block/Block` | 930 行 | 警告 缺 `namedVertices` 变量展开 / `check()` 面外向 | 2 |
| `block/BlockEdge/Face/Vertex/Grading` | 中等 | 警告 曲线边/面按 density 均匀采样，未用 `expand_/lineDivide` | 4 |
| `block/BlockMesh` | 841 行 | 阻塞 8 处核心未移植，详见 §3 | 8 |
| `snappyhexmesh/` | 空 | 阻塞 完全未实现 | – |
| `cartesian/`, `foam/` | 空 | 阻塞 未实现 | – |
| `samples/foam-blockmesh/main.cpp` | 0 字节 | 阻塞 空文件 | – |
| `samples/foam-snappyhexmesh/main.cpp` | 0 字节 | 阻塞 空文件 | – |
| 顶层 CMake | 56 行 | 阻塞 依赖外部 `x_*` 宏 / `XCommon` / OCCT / boost / json / doctest，无法独立构建 | – |
| `data/` 测试夹具 | 不存在 | 阻塞 测试代码引用 `data/dict/blockMeshDict*` 等三个字典 | – |

## 2. 编程规范遵循情况

规范来源 `doc/foam_code.md`。当前代码全面遵循以下约束：

- 头文件 `inc/XFoam/<module>/xfoam_<name>.h`，源文件 `src/<module>/xfoam_<name>.cpp`
- 头保护宏 `XFoam_<Name>_H_`
- 公开符号 `XFoam_` + PascalCase（类型）/ lowerCamel（函数、变量、宏）
- 头文件禁用具名 namespace；`.cpp` 内匿名 namespace 仅作 TU 私有
- 非 utilities 模块只 `#include "XFoam/utilities/xfoam_common.h"`
- 头部三联注释 `// 移植源码:` / `// 命名规范:` / `// 移植规范:` 标注上游对照
- `// 未移植：` 注释明确标出尚未对齐 Foam 行为的位置

## 3. `block/BlockMesh` 8 处未移植清单

源文件 `src/block/xfoam_blockmesh.cpp`：

```
L282   IOdictionary::lookupOrDefault<Switch>、geometry IOobject
L314   polyPatch::clone + OStringStream 序列化字典
L635   createTopology 中 defaultPatch name/type / preservePatchTypes / polyMesh 装配
L674   由 tmpBlockCells / patch 数据装配完整 polyMesh
L697   calcMergeInfo：当前仅 identity 映射，无跨 block 重合点合并
L724   calcMergeInfoFast：空体
L778   printCellSizeRange/printCellSizeRanges：Foam::Info 输出占位
L845   inplacePointTransforms：缺，convertToMeters 仅当作均匀缩放
```

## 4. 测试明确告警的崩溃点

`tests/foam-mesh/tblockmesh.cpp`：

- **L240** `blockMesh.hex111` 之后：`由 points/cells/patches 装配的 XFoam_PolyMesh 第二构造函数在部分环境下仍 SIGSEGV；write/BDF 待该路径稳定后再测`
- **L254** `blockMesh.hex456` 之后：`XFoam_PolyMesh(points, cells, patches, ...) 多块拓扑路径仍易崩溃；面数级断言待该路径稳定后再启用`

两者根因相同：`XFoam_PolyMesh::setTopology` 在面匹配/边界拓扑阶段，对 1 个以上 cell 或 1 个以上 block 时存在访问越界或未初始化引用（具体细节待运行时验证）。

## 5. 缺口依赖图（推荐修复顺序）

```
A. 修复 XFoam_PolyMesh 第二构造函数 SIGSEGV
   (mesh/xfoam_polymesh.cpp L500..L940)
                |
                v
B. BlockMesh::calcMergeInfo 真正的跨块顶点合并
   + createPatches/createPatchFaces 装配
   + createTopology 接通 polyMesh
                |
                v
C1. samples/foam-blockmesh CLI 入口    C2. OpenFOAM polyMesh 五件套写出器
                                          (points/faces/owner/neighbour/boundary)
                |
                v
D. 独立 CMake（剥离 x_* 宏 + XCommon）
                |
                v
E. tests/foam-mesh/data/dict/* 三个 blockMeshDict 算例
                |
                v
F. snappyHexMesh：完全从零（castellation -> snap -> layer addition）
```

注：A 与 D 之间存在反向依赖：要修 A 需要能跑测试，故实际操作顺序为 **D -> A -> B -> C2 -> C1 -> E -> F**。

## 6. 当前决策

- **下一步动手点**：A（修复 `XFoam_PolyMesh` SIGSEGV）
- **构建策略**：独立 CMake 含 OCCT；依赖通过 `find_package` / vcpkg / 用户提供路径
- **本文档归档位置**：`doc/status.md`，跟随仓库一同提交

## 7. 已知外部依赖

| 库 | 用途 | 计划 |
|---|---|---|
| `doctest` | 单元测试框架 | `find_package` 或 vendored 单头 |
| `nlohmann/json` | dictionary / jform 解析 | `find_package` 或 FetchContent |
| `boost::filesystem` | 路径处理（tests/`tcommon.h`） | `find_package` |
| `OpenCASCADE (OCCT)` | `xfoam_searchablesurface` 等几何运算 | `find_package` |
| `XCommon` 等内部库 | 旧自有构建框架，仅在原项目使用 | **不引入**，本项目用纯 CMake 替代 |

