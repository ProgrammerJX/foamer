## OpenFOAM 代码结构（参考）

#### 总体结构

```
├─ applications
│  ├─ test                      // 单元测试
│  └─ utilities
│      └─ mesh
│          ├─ blockMesh         // block 剖分
│          └─ snappyHexMesh
│
├─ src
│  └─ OpenFOAM                  // 基础库
```

---

## 头文件 `#include` 位置（XFoam 编程规范）

1. **默认规则**：在每个 `.h` / `.cpp` 源文件中，所有 `#include`（含 `<cstdio>` 等系统库与 `"XFoam/...` 工程头）必须写在文件**最前段**——紧接在头保护 `#ifndef … #define …`（或文件首行注释块）之后，且在**任何**非注释、非预处理指令的声明/定义/宏替换（如 `class` / `struct` / `template` / `namespace` / `typedef` / `const` 对象定义等）**之前**。同一文件内 `#include` 应**连续书写**，中间不插入其它代码；`.cpp` 中亦不得在首个函数/匿名命名空间之后再写 `#include`。
2. **唯一例外（OpenFOAM 移植惯用「声明 / 模板实现分离」）**：在**同一**主头文件（如 `xfoam_primitivepatch.h`、`xfoam_shape.h`）中，允许在**某一类或类模板的主声明体完整结束**（闭合 `};`）之后、本文件 `#endif` 之前，**仅**为引入与之配套的 `*I.h`、`*_templates.h` 而再写一行 `#include`；此类配套头文件自身仍须遵守第 1 条（其内部所有 `#include` 均在该文件顶部）。除上述「主声明已闭合后的配套实现头」外，**禁止**在文件中部或尾部插入其它工程头。

3. **实现细节（`xfoam_autoptr.h`）**：`XFoam_AutoPtr` 等模板成员内使用 `XFoam_FatalErrorIn` 等宏，因此 `xfoam_autoptr.h` 在 `#include "xfoam_types.h"` 之后**立即** `#include "xfoam_error.h"`；其它头文件若已包含 `xfoam_autoptr.h`，无需再为宏可见性单独调整 `xfoam_error.h` 顺序（重复 `#include` 由头保护消化即可）。

4. **非 utilities 代码引用 utilities**：凡**不在** `inc/XFoam/utilities/` 与 `src/utilities/` 下的头文件或 `.cpp`（如 `mesh/`、`block/`、`primitive/`、`topo/`、`tools/`、`tests/`、`samples/` 等），若需使用 `XFoam_String`、`XFoam_List`、`XFoam_Dictionary`、`XFoam_Error`、`XFoam_OStream` 等 **utilities 层类型或接口**，应**只**写一行 `#include "XFoam/utilities/xfoam_common.h"`，**禁止**再直接 `#include "XFoam/utilities/xfoam_types.h"`、`xfoam_stream.h` 等零散 utilities 子头（避免依赖顺序分叉与重复包含策略不一致）。`xfoam_common.h` 按约定顺序聚合 utilities 目录下各 `xfoam_*.h`。**兼容**：`xfoam_include.h` 仅转包含 `xfoam_common.h`，旧路径仍可编译，新代码请统一写 `xfoam_common.h`。utilities 目录**内部**实现仍保持各 `xfoam_*.h` 之间直接互相 `#include`，**不得**改为包含 `xfoam_common.h`（防止环依赖与无意义膨胀）。

## XFoam 命名规范

以下为 XFoam 模块内**建议遵循**的命名约定（与既有 OpenFOAM 对齐代码并存时，以可读、一致为先）。

| 范畴 | 规则 | 示例 |
|------|------|------|
| **全局变量** | `XFoam_` 前缀 + **首字母小写**的 camelCase（亦称 lowerCamel） | `XFoam_debug`、`XFoam_currentVersion` |
| **全局函数** | 同上 | `XFoam_hashBytes`、`XFoam_dictScalarEntry` |
| **全局 class 类型** | `XFoam_` 前缀 + **PascalCase**（每个成分首字母大写）；**唯一例外**：traits 模板类 **`XFoam_pTraits`**（`p` 小写，对标 `Foam::pTraits` 等传统 traits 命名） | `XFoam_BlockMesh`、`XFoam_ISstream`、`XFoam_pTraits<XFoam_Scalar>` |
| **全局 struct 类型** | 同上 | `XFoam_GradingDescriptors` |
| **全局 enum 类型** | 同上 | `XFoam_FileType`、`XFoam_Token::tokenType` |
| **全局 typedef / using 别名** | 同上 | `XFoam_String`、`XFoam_BlockVertList`、`XFoam_BlockEdgeList`、`XFoam_BlockFaceList` |
| **全局对象（与 OpenFOAM 同名形态对齐）** | `XFoam_` 前缀 + **PascalCase** 尾段，与上游全局实例命名一致 | `XFoam_FatalError`、`XFoam_FatalIOError`（分别为 `XFoam_Error`、`XFoam_IOerror` 的全局实例） |
| **预处理宏（`#define`）—对象宏 / 函数式宏** | `XFoam_` 前缀 + **首字母小写**的 camelCase（lowerCamel） | `XFoam_maxEntries`、`XFoam_hashRound`、`XFoam_declareRunTimeSelectionTable`（见 `xfoam_runtime.h`） |
| **预处理宏（`#define`）—与类相关的宏** | `XFoam` 前缀（**`Foam` 与后续成分之间无下划线**）+ **PascalCase** | `XFoamBlockMesh`、`XFoamDictionary` |
| **单元测试中的自定义函数**（`tests/` 下 doctest 等专用 helper，**非**库公开 API） | **`XFoamTests_`** 前缀 + **首字母小写**的 camelCase（lowerCamel） | `XFoamTests_moduleRoot`、`XFoamTests_dataDir`、`XFoamTests_tmpDir` |

- **`#define` 与 C++ 符号**：上表中「类相关宏」为 **`XFoam` 与后续 PascalCase 连写、中间无下划线**（如 `XFoamBlockMesh`），与 C++ 全局类型名 `XFoam_BlockMesh`（`XFoam_` + PascalCase）刻意区分，避免宏与类型同名引发预处理歧义。由 CMake、第三方或历史兼容引入的全大写宏（如 `XFOAM_SCALAR`）可保留，新写 XFoam 侧条件编译/特性开关时仍宜优先本表两条宏规则。
- **类内 / 函数内局部类型**（嵌套 class、struct、enum、using 等）：使用 **PascalCase**，**不加** `XFoam_` 前缀，以免与全局符号混淆；若需对外可见且跨翻译单元，仍宜提升为带 `XFoam_` 前缀的全局类型并放入头文件。
- **命名空间**：**禁止**在头文件（`.h`）中自定义 `namespace` 包裹 XFoam API（避免 ODR、依赖顺序与 include 污染）。在实现文件（`.cpp`）中可**有限**使用匿名命名空间 `namespace { ... }` 封装翻译单元私有的辅助函数或常量。
- **源文件名**：对外头文件、实现文件仍可采用 `xfoam_*.h` / `xfoam_*.cpp` 与 OpenFOAM 源树对照；符号名以本表为准。

## XFoam 移植 OpenFOAM 规范

从 OpenFOAM 迁代码或做行为对齐时，在「XFoam 命名规范」之外还须遵守：

1. **不得把一个函数拆分为多个函数（不允许拆分函数）**：以 OpenFOAM 侧**某一个**函数实现（如 `*.C` 中单独一块函数体）为移植对照单位，在 XFoam 中须在同一名下实现为**单个**函数；**不得**将该函数的控制流与数据流拆成多个函数来承担（包括拆成多个对外符号、多个平级 `static` 函数、或匿名命名空间内多个平级函数等），以免偏离「与上游逐函数对照、diff 可审」的粒度。若上游 OpenFOAM **本身**已将逻辑拆成多个子函数，则按**各自函数**分别移植，不在本条禁止之列。本条与「仅因文件内其它独立用途而写的工具函数」无冲突，但禁止以「工具」为名拆分某一正在移植的**单个**上游函数之整体逻辑。
2. **不允许拆分文件**：以 OpenFOAM 侧**单个** `.H`、`.C`（或惯例成对的声明/实现）为对齐单位，移植到 XFoam 时须落在**同一对** `xfoam_*.h` / `xfoam_*.cpp`（纯头内联则全部在同一 `xfoam_*.h`），不得为「模块化」或可读性再拆成多个平行头/源文件承载同一移植单元；上游若另有 `*I.H`、`*Templates.C` 等，并入所对标的 `xfoam_*.h`（或同一 `.cpp`）内，**禁止**新建额外碎片文件（如单独的 `xfoam_*I.h`）分散同一逻辑。
3. **不允许新增命名空间**：不得新增任何**具名** `namespace`；头文件与公开 API 的约定同上文「命名规范」；仅在实现文件（`.cpp`）中可使用**匿名**命名空间封装翻译单元私有符号。
4. **用 XFoam utilities 中已有类型替换**：与 OpenFOAM 语义对应的量，凡 **`inc/XFoam/utilities/`** 目录内各 `xfoam_*.h` 中**已定义的全部类型**（含别名、类、模板与 `xfoam_types.h` 中的 **`XFoam_Label`、`XFoam_Scalar`、`XFoam_String`、`XFoam_StreamSize`**、`XFoam_UInt8`～`XFoam_UInt64`、`XFoam_Int8`～`XFoam_Int64`、缓冲区等），一律改用上述已有定义，不为本移植块另起一套平行类型。
5. **移植溯源注释（与实现中三行注释块一致）**：对标某段 OpenFOAM 实现时，在 `.cpp` 中宜保留或补全下列**成组**注释，便于对照与评审：`// 移植源码:`（上游 **`src/...` 相对路径**及类/函数或片段名，**不写**本机绝对路径）、`// 命名规范: foam_code.md`、`// 移植规范: foam_code.md`。头文件中标注对标的 `.H` / 模块路径的说明可继续保留。
6. **保留代码中的 debug / 可观测性信息**：移植时**不得**为「精简」而整段删除上游与**调试开关、`Info`/`Warning`、verbose、诊断分支**等相关的语句；已有 XFoam 对等 API 的改为对等调用。尚无对等输出或宏时，**须将原语句保留为注释**（可与 `// 未移植：` 并列说明对标符号），以便与上游 diff 及后续补全；**禁止**无注释、无对标的空白占位替代可恢复的调试信息。后续若删除此类注释中的调试片段，须在评审中明确记录。

## XFoam 代码结构（本仓库 `src/Mesh/XFoam`）

XFoam 为对齐 OpenFOAM 语义的 C++ 网格/字典/IO 子库，产出动态库 **XFoam**（工程名 `MODULE_NAME XFoam`）；**库目标不链接 XCommon/XData**（测试可执行文件与 sample 仍可按需链 **XCommon** 与 **XFoam**）。头文件路径形如 `#include "XFoam/.../xfoam_*.h"`；公开类型与函数命名以 **`XFoam_`** 为前缀（源文件名仍为 `xfoam_*.h` / `xfoam_*.cpp`，便于与上游 OF 文件名对照）。

#### 类型与标准库（约定）

- `xfoam_types.h` 在既有 `XFoam_String`（`std::string`）、`XFoam_Size`、`XFoam_Label`、`XFoam_Scalar` 之外，提供固定宽度别名 **`XFoam_UInt8`～`XFoam_UInt64`、`XFoam_Int8`～`XFoam_Int64`**，以及 **`XFoam_StreamSize`**（`std::streamsize`）、**`XFoam_CharBuffer`**（`std::vector<char>`）。模块内新代码宜优先使用上述别名与 `XFoam_String`，减少对裸 `std::uint32_t` 等的直接书写。
- 哈希表、字典等仍以 STL 容器为底层实现；与 OpenFOAM 对齐的「自有类」与「std 别名」并存，逐步把热点路径从裸 std 类型迁到 `XFoam_*` 命名。

#### 仓库根目录

```
XFoam/
├─ CMakeLists.txt          // 模块聚合、标量精度宏、测试与 sample 目标
├─ inc/XFoam/              // 对外头文件（按子域分子目录）
├─ src/                    // 实现，子目录与 inc/XFoam 对应
├─ tests/
│  ├─ foam-core/           // doctest：类型、List、Dictionary、Stream、Field 等
│  └─ foam-mesh/           // doctest：blockMesh、PolyMesh、Shape 等
├─ samples/foam-core/      // 链 XFoam 的可执行样例入口
├─ data/                   // 字典样例、jform、stp 等测试夹具数据
└─ doc/
   └─ foam_code.md         // 本文
```

#### `inc/XFoam` 与 `src` 子域说明

| 子目录 | 职责概要 |
|--------|----------|
| **utilities** | 基础设施：`xfoam_types.h`（标量/字符串/词等）、`xfoam_list.h`（List / PtrList / DLList 等）、`xfoam_stream.h`（I/O 流与 Token）、`xfoam_dictionary.h`、`xfoam_field.h`、`xfoam_autoptr.h`、`xfoam_error.h`、`xfoam_hash.h`、`xfoam_regexp.h`、`xfoam_runtime.h`（run-time selection 宏）、`xfoam_ioobject.h`、`xfoam_time.h` 等。 |
| **mesh** | 网格核心：`xfoam_shape.h`（CellShape / Face）、`xfoam_polymesh.h`、`xfoam_primitivemesh.h`、`xfoam_primitivepatch.h`、`xfoam_polypatch.h` 等。 |
| **block** | blockMesh 相关：`xfoam_blockmesh.h`、`xfoam_block.h`、`xfoam_blockvert.h`、`xfoam_blockedge.h`、`xfoam_blockface.h` 等。 |
| **primitive** | 几何图元：直线、三角形、四面体、棱锥、平面等（`xfoam_line.h`、`xfoam_triangle.h` 等）。 |
| **topo** | 拓扑与 BRep 抽象：`xfoam_mbrep.h`、`xfoam_topo.h`、`xfoam_vbrep.h` 等。 |
| **tools** | 工具类：`xfoam_searchablesurface.h`（可搜索曲面列表，供 block 顶点等使用）。 |

`CMakeLists.txt` 中尚登记 **foam**、**snappyhexmesh**、**cartesian** 等子模块目录；当前树中若对应目录无源文件，则为预留扩展位，便于后续对齐 OpenFOAM 同名组件。

#### 构建与测试（摘录）

- **标量**：`XFOAM_SCALAR` 缓存变量（`FLOAT` / `DOUBLE` / `LONGDOUBLE`）映射为 `DXFOAM_SCALAR_*` 编译宏（见顶层 `CMakeLists.txt`）。
- **库目标**：`x_add_module2` 生成 **XFoam**（不链 XCommon/XData）；`x_add_testing_module2` 等目标仍通过 `x_add_lib(XCommon XFoam)` 链测试所需库。
- **测试可执行文件**：`x-test-foam-core` 与 `x-test-foam-mesh`（`TEST_MODULE_NAME` 为 `foam-core` 与 `foam-mesh` 两段配置）。

#### 与 OpenFOAM 的对应关系（阅读代码时）

- 头文件/类注释中常标注对标的 OpenFOAM 路径（如 `blockMesh/`、`Field/`、`dictionary/`），便于查阅上游实现。
- 已移植子集在注释中标「未移植」或占位行为，避免与完整 OF 行为混同；与 **debug / 可观测性** 相关的保留约定见上文移植规范第 **5、6** 条。
