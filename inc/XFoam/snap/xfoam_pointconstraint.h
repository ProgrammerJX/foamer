#ifndef XFoam_PointConstraint_H_
#define XFoam_PointConstraint_H_

// 对标 OpenFOAM-13: src/meshTools/pointConstraint/pointConstraint.H
// snappyHexMesh snap/smooth 阶段用来描述「一个点剩余的位移自由度」。
//
//   nConstraints == 0 : free        （3 DOF，displacement 不受限）
//   nConstraints == 1 : plane       （2 DOF，displacement ⟂ first 方向必须为 0）
//   nConstraints == 2 : line        （1 DOF，displacement 只能沿 first × second 方向）
//   nConstraints == 3 : fixed       （0 DOF，displacement ≡ 0）
//
// 用法：
//   * 在 snap 完成后给每个 boundary point 标一份初始约束：
//       feature vertex → fixed
//       feature edge   → line（用边的两个相邻面法向作为 first/second）
//       smooth surface → plane（surface 法向作 first）
//   * 之后任何 displacement Δ 经过 constrainDisplacement(Δ) 投影到允许子空间，
//     就不会把点拉离 feature / surface。
//   * combine(other) 用「取约束更严的那一个」语义：两个 plane 不同向就升级到 line；
//     plane + line 升级到 fixed；任意一方 fixed 都升级到 fixed。这样把"多个 surface
//     snap 到同一点"的多约束自然合并。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_vector.h"

/*---------------------------------------------------------------------------*\
                    Class XFoam_PointConstraint Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_PointConstraint
{
public:
	/// 两个法向夹角余弦 ≤ kParallelCos 视为「平行」，combine 时被吸收成同一个 plane。
	/// 0.99 ≈ 8.1° tolerance；与 OpenFOAM 默认的 pointConstraint 几乎一致。
	static constexpr XFoam_Scalar kParallelCos = static_cast<XFoam_Scalar>(0.99);

	XFoam_PointConstraint() = default;

	/// 单 plane 约束便捷构造：法向应为单位向量；非单位向量在 constrain 路径上不安全。
	static XFoam_PointConstraint plane(const XFoam_Vector3D& unitNormal);

	/// 由 edge 切向（unit）构造的 line 约束；内部用任意两个 ⟂ 平面法向表示。
	static XFoam_PointConstraint line(const XFoam_Vector3D& unitTangent);

	/// 0 DOF 锁死。
	static XFoam_PointConstraint fixed();

	/// 当前 DOF 约束数：0/1/2/3。
	int nConstraints() const { return nConstraints_; }

	/// nConstraints==1 时为法向；==2 时为 line 切向。其它情况未定义（仅作为内部存储）。
	const XFoam_Vector3D& first() const { return first_; }

	/// 仅 nConstraints==2 时使用：与 first 一起定义 line 方向。
	const XFoam_Vector3D& second() const { return second_; }

	/// 把任意位移 d 投影到允许的子空间。
	///   free  : 原样返回
	///   plane : d - (d·n) n
	///   line  : (d·t) t                 ← t = first（unit tangent）
	///   fixed : (0,0,0)
	XFoam_Vector3D constrainDisplacement(const XFoam_Vector3D& d) const;

	/// 合并另一个约束：取约束更严的那一个（详见 header 头注）。
	void combine(const XFoam_PointConstraint& other);

private:
	int nConstraints_ = 0;
	// nConstraints==1: first_ 是 plane 法向（unit）
	// nConstraints==2: first_ 是 line 切向（unit）；second_ 是该 line 的一个 ⟂ 平面法向（unit）
	// nConstraints==3: first_/second_ 不参与判定
	XFoam_Vector3D first_ = XFoam_Vector3D(0, 0, 0);
	XFoam_Vector3D second_ = XFoam_Vector3D(0, 0, 0);
};

#endif
