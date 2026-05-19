# myfoam

OpenFOAM 风格的网格库（XFoam），目标包含 blockMesh / snappyHexMesh 等工具的 C++ 重写。
本仓库以 `foam-refactor` 为基线，已剥离对 `XCommon` 与外部 CMake 框架的依赖，可独立构建。

## 约束

- 仅允许 **C++11**。
- 所有第三方依赖由本仓库的 CMake 自动下载，禁止依赖系统已安装版本。

## 第三方依赖

由 `FetchContent` 在 configure 阶段自动下载：

| 库 | 版本 | 用途 |
|---|---|---|
| Boost | 1.85.0（cmake 包）| `boost::filesystem` + `boost::system`，路径/文件操作 |
| doctest | v2.4.11 | 单元测试框架 |

> 首次 configure 大约下载 150 MB（Boost 源码包），仅编译 `filesystem` 与 `system` 两个子模块，速度可控。
> 下载到 `build/_deps/`，`.gitignore` 已排除。

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
- `-DXFOAM_BUILD_SAMPLES=ON`：构建 `samples/` 下的示例可执行文件（当前为占位）。
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
inc/XFoam/          公开头文件（utilities, primitive, mesh, block, topo, tools）
src/                .cpp 实现，子目录与 inc/XFoam/ 对齐
tests/foam-core/    utilities / primitives / 核心 mesh 单元测试
tests/foam-mesh/    blockMesh + 完整 PolyMesh 装配测试
samples/            示例可执行文件（占位）
doc/                编程规范、设计说明
```
