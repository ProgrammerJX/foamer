#ifndef XFoam_Common_H_
#define XFoam_Common_H_

// 聚合本目录下全部 utilities 头文件（依赖顺序：先基础类型与流，再 list / dictionary 等）。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_error.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_unit.h"
#include "XFoam/utilities/xfoam_hash.h"
#include "XFoam/utilities/xfoam_runtime.h"
#include "XFoam/utilities/xfoam_randomgenerator.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_tuple.h"
#include "XFoam/utilities/xfoam_vector.h"
#include "XFoam/utilities/xfoam_tensor.h"
#include "XFoam/utilities/xfoam_list.h"
#include "XFoam/utilities/xfoam_regexp.h"
#include "XFoam/utilities/xfoam_field.h"
#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_ioobject.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_time.h"
#include "XFoam/utilities/xfoam_pointhit.h"
#include "XFoam/utilities/xfoam_circulator.h"

#endif
