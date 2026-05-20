# myfoam

OpenFOAM 风格的网格库（XFoam），目标包含 blockMesh / snappyHexMesh 等工具的 C++ 重写。
本仓库以 `foam-refactor` 为基线，已剥离对 `XCommon` 与外部 CMake 框架的依赖，可独立构建。

## 约束

- 默认 **C++17**（OCCT 7.9 要求；早期版本守过 C++11，接 OCCT 时升级）。
- 所有第三方依赖由本仓库的 CMake 自动下载，禁止依赖系统已安装版本。

## 第三方依赖

由 `FetchContent` 在 configure 阶段自动下载：

| 库 | 版本 | 用途 | 默认 |
|---|---|---|---|
| Boost | 1.85.0（cmake 包）| `boost::filesystem` + `boost::system`，路径/文件操作 | 必选 |
| doctest | v2.4.11 | 单元测试框架 | 必选 |
| OCCT | V7_9_1 | 参数化 B-rep / STEP / IGES 解析（`XFoam_MBrep`） | `-DXFOAM_WITH_OCCT=ON` 才拉 |

> 首次 configure 大约下载 150 MB（Boost 源码包），仅编译 `filesystem` 与 `system` 两个子模块，速度可控。
> 下载到 `build/_deps/`，`.gitignore` 已排除。

### OCCT（可选）

`-DXFOAM_WITH_OCCT=ON` 时，CMake 会从 GitHub 拉 OCCT 源码并构建以下最小 CAD 子集：
`FoundationClasses` / `ModelingData` / `ModelingAlgorithms` / `DataExchange`。

不启用 Visualization / Draw / ApplicationFramework / FreeType / TBB / TCL，所以
不需要任何系统包；**首次配置 + 编译耗时大约 15–30 min（多核）**。把开关关掉时，
`XFoam_MBrep::readFromStep / readFromIges / tessellate` 会抛
`XFoam_Error("OCCT disabled at build time")`，整个工程仍可正常构建运行。

启用后会多出：

- `x-test-foam-topo`：OCCT 端到端单元测试（自造 BRepPrimAPI_MakeBox + STEP 回读）
- `x-sample-foam-occt`：CLI 把 STEP/IGES 转 STL，方便接 `x-sample-foam-snappyhexmesh`

## 编程规范

详见 `doc/foam_code.md`。重点：

- `inc/` 公开头文件，`src/` 私有实现，不混。
- 头文件 `.h`；不开命名空间，类型/函数全部 `XFoam_*` 前缀。
- 不写注释说明改动；可写"为什么"的注释，禁止"做了什么"流水账。

## 编译（Windows / VS 2019）

```powershell
"C:\workspace\setup\VS2019\VC\Auxiliary\Build\vcvars64.bat"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

可选：

- `-DXFOAM_BUILD_TESTS=OFF`：跳过测试。
- `-DXFOAM_BUILD_SAMPLES=ON`：构建 `samples/` 下的示例可执行文件。
- `-DXFOAM_WITH_OCCT=ON`：拉取 + 编译 OCCT 7.9（首次约 15–30 min），开启 STEP/IGES 路径。
- `-DXFOAM_SCALAR=FLOAT|DOUBLE|LONGDOUBLE`：标量精度宏（注意 `XFoam_Scalar` 实际仍硬编为 `double`，本宏只切换 legacy 检测分支）。

## 测试

```powershell
ctest --test-dir build --output-on-failure
```

或直接：

```powershell
build\bin\x-test-foam-core.exe -nb -d
build\bin\x-test-foam-mesh.exe -nb -d
```

当前已知失败：

- `foam-mesh` 的若干 `TEST_CASE` 需要 `data/dict/blockMeshDict*` 等输入文件。这些数据从原 `foam-refactor` 起就不在仓库里，待补。
- `XFoam_PolyMesh` 由 `points/faces/owner/neighbour/patch*` 装配的第二构造函数在多块用例下 SIGSEGV（`tblockmesh.cpp` 注释亦说明），待修。

## 目录结构

```
inc/XFoam/          公开头文件（utilities, primitive, mesh, block, topo, snap, tools）
src/                .cpp 实现，子目录与 inc/XFoam/ 对齐
tests/foam-core/    utilities / primitives / 核心 mesh 单元测试
tests/foam-mesh/    blockMesh + 完整 PolyMesh 装配 + 虚拓扑（VBrep / BDF）测试
tests/foam-snap/    snappyHexMesh（refine + snap + addLayers）测试
tests/foam-topo/    OCCT 路径端到端测试（仅 XFOAM_WITH_OCCT=ON 时编译）
samples/            示例可执行文件（blockMesh / snappyHexMesh / occt CAD 转 STL）
doc/                编程规范、设计说明
```
