#include "XFoam/block/xfoam_blockmesh.h"
#include <cmath>

namespace
{
static XFoam_PolyMesh* XFoam_newSingleHexTopology(const XFoam_PointField& vtx)
{
	if (vtx.size() != 8)
	{
		return nullptr;
	}
	XFoam_PointField points(8);
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		points[i] = vtx[i];
	}
	const XFoam_Label f0[4] = {3, 7, 6, 2};
	const XFoam_Label f1[4] = {0, 4, 7, 3};
	const XFoam_Label f2[4] = {2, 6, 5, 1};
	const XFoam_Label f3[4] = {1, 5, 4, 0};
	const XFoam_Label f4[4] = {0, 3, 2, 1};
	const XFoam_Label f5[4] = {4, 5, 6, 7};
	XFoam_FaceList faces(6);
	for (int fi = 0; fi < 6; ++fi)
	{
		const XFoam_Label* src =
			(fi == 0   ? f0
			 : fi == 1 ? f1
			 : fi == 2 ? f2
			 : fi == 3 ? f3
			 : fi == 4 ? f4
				    : f5);
		XFoam_Face& f = faces[fi];
		f.setSize(4);
		for (int k = 0; k < 4; ++k)
		{
			f[k] = src[k];
		}
	}
	XFoam_LabelList owner(6, 0);
	XFoam_LabelList nei(0);

	XFoam_WordList patchNames(1);
	patchNames[0] = XFoam_Word("WALLS");
	XFoam_LabelList patchSizes(1);
	patchSizes[0] = 6;
	XFoam_WordList patchTypes(1);
	patchTypes[0] = XFoam_Word("patch");

	return new XFoam_PolyMesh(
		XFoam_move(points),
		XFoam_move(faces),
		XFoam_move(owner),
		XFoam_move(nei),
		patchNames,
		patchSizes,
		patchTypes);
}

static XFoam_IOerrorLocation XFoam_ioLocDictName(const XFoam_Dictionary& d)
{
	return XFoam_IOerrorLocation(static_cast<const XFoam_String&>(static_cast<const XFoam_FileName&>(d.name())));
}

static XFoam_TokenList XFoam_readBalancedBraceContent(XFoam_ITstream& is, const XFoam_IOerrorLocation& loc)
{
	XFoam_TokenList out;
	XFoam_Label depth = 1;
	XFoam_Token t;
	while (is >> t, t.good())
	{
		if (t == XFoam_Token::BEGIN_BLOCK || t == XFoam_Token::BEGIN_LIST)
		{
			++depth;
		}
		else if (t == XFoam_Token::END_BLOCK || t == XFoam_Token::END_LIST)
		{
			--depth;
			if (depth == 0)
			{
				return out;
			}
		}
		out.append(t);
	}
	XFoam_FatalIOErrorInFunction(loc) << "Unterminated boundary patch entry" << XFoam_exit(XFoam_FatalIOError, 1);
	return out;
}

static XFoam_Label XFoam_readLabelBD(
	XFoam_IStream& is,
	const XFoam_Dictionary& varDict,
	const XFoam_IOerrorLocation& loc)
{
	XFoam_Token t;
	is >> t;
	if (!t.good())
	{
		XFoam_FatalIOErrorInFunction(loc) << "unexpected EOF reading label" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (t.isLabel())
	{
		return t.labelToken();
	}
	if (t.isWord())
	{
		const XFoam_Word varName(t.wordToken());
		const XFoam_Entry* ePtr = varDict.lookupScopedEntryPtr(varName, true, true);
		if (!ePtr)
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "Undefined variable " << varName << ". Valid variables are " << varDict
				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
		XFoam_ITstream& es = ePtr->stream();
		es.rewind();
		XFoam_Token vt;
		es >> vt;
		if (!vt.isLabel())
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "Variable " << varName << " does not expand to a label" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		return vt.labelToken();
	}
	XFoam_FatalIOErrorInFunction(loc)
		<< "Illegal token when trying to read label" << XFoam_exit(XFoam_FatalIOError, 1);
	return 0;
}

static void XFoam_readOneFaceBD(
	XFoam_IStream& is,
	const XFoam_Dictionary& varDict,
	const XFoam_IOerrorLocation& loc,
	XFoam_Face& f)
{
	XFoam_Token firstTok;
	is >> firstTok;
	if (!firstTok.good())
	{
		XFoam_FatalIOErrorInFunction(loc) << "unexpected EOF reading face" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (firstTok.isLabel())
	{
		const XFoam_Label n = firstTok.labelToken();
		f.setSize(n);
		(void)is.readBeginList("face");
		for (XFoam_Label i = 0; i < n; ++i)
		{
			f[i] = XFoam_readLabelBD(is, varDict, loc);
		}
		is.readEndList("face");
		return;
	}
	is.putBack(firstTok);
	(void)is.readBeginList("face");
	XFoam_LabelList buf;
	for (;;)
	{
		XFoam_Token t;
		is >> t;
		if (!t.good())
		{
			XFoam_FatalIOErrorInFunction(loc) << "unexpected EOF in face list" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		if (t.isPunctuation() && t.pToken() == XFoam_Token::END_LIST)
		{
			is.putBack(t);
			break;
		}
		is.putBack(t);
		buf.append(XFoam_readLabelBD(is, varDict, loc));
	}
	is.readEndList("face");
	f = XFoam_Face(buf);
}

static void XFoam_readFaceListBD(
	XFoam_IStream& is,
	const XFoam_Dictionary& varDict,
	const XFoam_IOerrorLocation& loc,
	XFoam_List<XFoam_Face>& out)
{
	out.clear();
	XFoam_Token firstTok;
	is >> firstTok;
	if (!firstTok.good())
	{
		XFoam_FatalIOErrorInFunction(loc) << "unexpected EOF reading face list" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (firstTok.isLabel())
	{
		const XFoam_Label n = firstTok.labelToken();
		out.setSize(n);
		(void)is.readBeginList("faceList");
		for (XFoam_Label i = 0; i < n; ++i)
		{
			XFoam_readOneFaceBD(is, varDict, loc, out[i]);
		}
		is.readEndList("faceList");
		return;
	}
	is.putBack(firstTok);
	(void)is.readBeginList("faceList");
	for (;;)
	{
		XFoam_Token t;
		is >> t;
		if (!t.good())
		{
			XFoam_FatalIOErrorInFunction(loc) << "unexpected EOF in faceList" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		if (t.isPunctuation() && t.pToken() == XFoam_Token::END_LIST)
		{
			is.putBack(t);
			break;
		}
		is.putBack(t);
		const XFoam_Label j = out.size();
		out.setSize(j + 1);
		XFoam_readOneFaceBD(is, varDict, loc, out[j]);
	}
	is.readEndList("faceList");
}

static void XFoam_buildOneBoundaryPatch(
	const XFoam_Word& patchName,
	const XFoam_TokenList& innerTokens,
	const XFoam_Dictionary& varDict,
	XFoam_Dictionary& pdict,
	XFoam_List<XFoam_Face>& faceList,
	const XFoam_IOerrorLocation& loc)
{
	XFoam_ITstream innerIs(XFoam_KeyType(patchName), innerTokens);
	innerIs.rewind();
	pdict.clear();
	pdict.name() = XFoam_FileName(static_cast<const XFoam_String&>(patchName));
	if (!pdict.read(innerIs))
	{
		XFoam_FatalIOErrorInFunction(loc)
			<< "Error reading patch dictionary for patch " << patchName << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (!pdict.found(XFoam_Word("faces")))
	{
		XFoam_FatalIOErrorInFunction(loc)
			<< "Patch " << patchName << " has no faces entry" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	XFoam_ITstream& faceIs = pdict.lookup(XFoam_Word("faces"));
	faceList.clear();
	XFoam_readFaceListBD(faceIs, varDict, loc, faceList);
}
} // namespace

const char* const XFoam_BlockMesh::typeName = "blockMesh";

bool XFoam_BlockMesh::checkBlockFaceOrientation = true;

XFoam_BlockMesh::XFoam_BlockMesh(
	const XFoam_Dictionary& dict,
	const XFoam_FileName& meshPath,
	const XFoam_Word& regionName)
	: meshDict_(dict)
	, verboseOutput(true)
	, checkFaceCorrespondence_(true)
	, geometry_()
	, scaleFactor_(1.0)
	, blockVertices_(meshDict_.lookup(XFoam_Word("vertices")), XFoam_BlockVert::INew(meshDict_, geometry_))
	, vertices_(XFoam_vertices(blockVertices_))
	, edges_()
	, faces_()
	, topologyPtr_(createTopology(dict, meshPath, regionName))
	, nPoints_(0)
	, nCells_(0)
	, blockOffsets_()
	, mergeList_()
	, points_()
	, cells_()
	, patches_()
{
	(void)meshPath;
	(void)regionName;
	// 未移植：Foam::IOdictionary::lookupOrDefault<Switch>、geometry IOobject、
	// Foam::Switch fastMerge 与 calcMergeInfo / calcMergeInfoFast 的完整合并逻辑。
	calcMergeInfo();
}

XFoam_BlockMesh::~XFoam_BlockMesh()
{
	delete topologyPtr_;
	topologyPtr_ = nullptr;
}

void XFoam_BlockMesh::verbose(const bool on)
{
	verboseOutput = on;
}

const XFoam_PointField& XFoam_BlockMesh::vertices() const
{
	return vertices_;
}

const XFoam_PolyMesh& XFoam_BlockMesh::topology() const
{
	if (!topologyPtr_)
	{
		XFoam_FatalErrorInFunction << "topologyPtr_ not allocated" << XFoam_abort(XFoam_FatalError);
	}
	return *topologyPtr_;
}

XFoam_PtrListDictionary<XFoam_Dictionary> XFoam_BlockMesh::patchDicts() const
{
	// 未移植：Foam::polyPatch::clone + OStringStream 序列化字典（见 OF blockMesh.C）。
	return XFoam_PtrListDictionary<XFoam_Dictionary>();
}

XFoam_Scalar XFoam_BlockMesh::scaleFactor() const
{
	return scaleFactor_;
}

const XFoam_PointField& XFoam_BlockMesh::points() const
{
	if (points_.empty())
	{
		createPoints();
	}
	return points_;
}

const XFoam_CellShapeList& XFoam_BlockMesh::cells() const
{
	if (cells_.empty())
	{
		createCells();
	}
	return cells_;
}

const XFoam_FaceListList& XFoam_BlockMesh::patches() const
{
	if (patches_.empty())
	{
		createPatches();
	}
	return patches_;
}

XFoam_WordList XFoam_BlockMesh::patchNames() const
{
	if (!topologyPtr_)
	{
		return XFoam_WordList();
	}
	const XFoam_List<XFoam_String> n = topology().boundary().names();
	XFoam_WordList out(static_cast<XFoam_Label>(n.size()));
	for (XFoam_Label i = 0; i < n.size(); ++i)
	{
		out[i] = XFoam_Word(n[i]);
	}
	return out;
}

XFoam_Label XFoam_BlockMesh::numZonedBlocks() const
{
	XFoam_Label num = 0;
	for (XFoam_Label blocki = 0; blocki < size(); ++blocki)
	{
		if (this->set(blocki) && operator[](blocki).zoneName().size())
		{
			++num;
		}
	}
	return num;
}

void XFoam_BlockMesh::writeTopology(XFoam_OStream& os) const
{
	if (!topologyPtr_)
	{
		return;
	}
	const XFoam_UList<XFoam_Vector3D>& pts = topology().points();
	for (XFoam_Label pI = 0; pI < pts.size(); ++pI)
	{
		const XFoam_Vector3D& pt = pts[pI];
		os << "v " << pt.x() << ' ' << pt.y() << ' ' << pt.z() << '\n';
	}
	const XFoam_List<XFoam_Edge>& eds = topology().edges();
	for (XFoam_Label eI = 0; eI < eds.size(); ++eI)
	{
		const XFoam_Edge& e = eds[eI];
		os << "l " << e.start() + 1 << ' ' << e.end() + 1 << '\n';
	}
}

void XFoam_BlockMesh::readPatches(
	const XFoam_Dictionary& meshDescription,
	XFoam_FaceListList& tmpBlocksPatches,
	XFoam_WordList& patchNames,
	XFoam_WordList& patchTypes,
	XFoam_WordList& nbrPatchNames)
{
	(void)meshDescription;
	(void)tmpBlocksPatches;
	(void)patchNames;
	(void)patchTypes;
	(void)nbrPatchNames;
}

// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshTopology.C（readBoundary）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
void XFoam_BlockMesh::readBoundary(
	const XFoam_Dictionary& meshDescription,
	XFoam_WordList& patchNames,
	XFoam_FaceListList& tmpBlocksPatches,
	XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const
{
	XFoam_Dictionary varDict(meshDescription.subOrEmptyDict(XFoam_Word("namedVertices")));
	varDict.merge(meshDescription.subOrEmptyDict(XFoam_Word("namedBlocks")));

	const XFoam_Entry* ent = meshDescription.lookupEntryPtr(XFoam_Word("boundary"), false, true);
	if (!ent)
	{
		return;
	}
	const XFoam_IOerrorLocation loc(XFoam_ioLocDictName(meshDescription));

	if (ent->isDict())
	{
		readBoundaryFromSubDict(
			ent->dict(),
			varDict,
			patchNames,
			tmpBlocksPatches,
			patchDicts);
		return;
	}
	if (!ent->isStream())
	{
		XFoam_FatalIOErrorInFunction(loc)
			<< "boundary entry is neither dictionary nor stream" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	readBoundaryFromPatchEntryStream(
		ent->stream(),
		meshDescription,
		varDict,
		patchNames,
		tmpBlocksPatches,
		patchDicts);
}

void XFoam_BlockMesh::readBoundaryFromSubDict(
	const XFoam_Dictionary& bnd,
	const XFoam_Dictionary& varDict,
	XFoam_WordList& patchNames,
	XFoam_FaceListList& tmpBlocksPatches,
	XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const
{
	const XFoam_IOerrorLocation loc(XFoam_ioLocDictName(bnd));
	patchNames.clear();
	tmpBlocksPatches.clear();
	patchDicts.clear();
	const XFoam_WordList keys = bnd.sortedToc();
	for (XFoam_Label ki = 0; ki < keys.size(); ++ki)
	{
		const XFoam_Word& patchName = keys[ki];
		const XFoam_Entry* e = bnd.lookupEntryPtr(patchName, false, false);
		if (!e)
		{
			continue;
		}
		if (!e->isStream())
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "Entry " << patchName << " in boundary section is not a valid primitive stream"
				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
		XFoam_ITstream& ps = e->stream();
		ps.rewind();
		XFoam_Token t;
		ps >> t;
		if (!t.good())
		{
			continue;
		}
		if (t != XFoam_Token::BEGIN_BLOCK)
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "Patch " << patchName << ": expected '{' after patch name" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		const XFoam_TokenList inner = XFoam_readBalancedBraceContent(ps, loc);
		for (XFoam_Label i = 0; i < patchNames.size(); ++i)
		{
			if (patchNames[i] == patchName)
			{
				XFoam_FatalIOErrorInFunction(loc)
					<< "Duplicate patch " << patchName << XFoam_exit(XFoam_FatalIOError, 1);
			}
		}
		XFoam_Dictionary pdict;
		XFoam_List<XFoam_Face> faceList;
		XFoam_buildOneBoundaryPatch(patchName, inner, varDict, pdict, faceList, loc);
		checkPatchLabels(pdict, patchName, vertices_, faceList);
		patchNames.append(patchName);
		tmpBlocksPatches.append(faceList);
		patchDicts.append(patchName, new XFoam_Dictionary(pdict));
	}
}

void XFoam_BlockMesh::readBoundaryFromPatchEntryStream(
	XFoam_ITstream& patchStream,
	const XFoam_Dictionary& meshDescription,
	const XFoam_Dictionary& varDict,
	XFoam_WordList& patchNames,
	XFoam_FaceListList& tmpBlocksPatches,
	XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const
{
	const XFoam_IOerrorLocation loc(XFoam_ioLocDictName(meshDescription));
	patchNames.clear();
	tmpBlocksPatches.clear();
	patchDicts.clear();
	patchStream.rewind();
	XFoam_Token outer;
	patchStream >> outer;
	if (!outer.good())
	{
		return;
	}
	const bool listO = (outer == XFoam_Token::BEGIN_LIST);
	const bool braceO = (outer == XFoam_Token::BEGIN_BLOCK);
	if (!listO && !braceO)
	{
		XFoam_FatalIOErrorInFunction(loc)
			<< "boundary: expected '(' or '{' after keyword" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	for (;;)
	{
		XFoam_Token t;
		patchStream >> t;
		if (!t.good())
		{
			break;
		}
		if (braceO && t == XFoam_Token::END_BLOCK)
		{
			break;
		}
		if (listO && t == XFoam_Token::END_LIST)
		{
			break;
		}
		if (!t.isWord())
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "boundary: expected patch name (word)" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		const XFoam_Word patchName(t.wordToken());
		XFoam_Token ob;
		patchStream >> ob;
		if (!ob.good() || ob != XFoam_Token::BEGIN_BLOCK)
		{
			XFoam_FatalIOErrorInFunction(loc)
				<< "boundary: patch " << patchName << " expected '{'" << XFoam_exit(XFoam_FatalIOError, 1);
		}
		const XFoam_TokenList inner = XFoam_readBalancedBraceContent(patchStream, loc);
		for (XFoam_Label i = 0; i < patchNames.size(); ++i)
		{
			if (patchNames[i] == patchName)
			{
				XFoam_FatalIOErrorInFunction(loc)
					<< "Duplicate patch " << patchName << XFoam_exit(XFoam_FatalIOError, 1);
			}
		}
		XFoam_Dictionary pdict;
		XFoam_List<XFoam_Face> faceList;
		XFoam_buildOneBoundaryPatch(patchName, inner, varDict, pdict, faceList, loc);
		checkPatchLabels(pdict, patchName, vertices_, faceList);
		patchNames.append(patchName);
		tmpBlocksPatches.append(faceList);
		patchDicts.append(patchName, new XFoam_Dictionary(pdict));
	}
}

// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshTopology.C（createCellShapes）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
void XFoam_BlockMesh::createCellShapes(XFoam_CellShapeList& tmpBlockCells)
{
	const XFoam_BlockMesh& blocks = *this;
	tmpBlockCells.setSize(blocks.size());
	for (XFoam_Label blocki = 0; blocki < blocks.size(); ++blocki)
	{
		if (!blocks.set(blocki))
		{
			throw XFoam_Error(
				XFoam_String("XFoam_BlockMesh::createCellShapes: block index not set"));
		}
		tmpBlockCells[blocki] = blocks[blocki].blockShape();
	}
}

void XFoam_BlockMesh::defaultPatchError(const XFoam_Word& defaultPatchName, const XFoam_Dictionary& meshDescription) const
{
	(void)defaultPatchName;
	(void)meshDescription;
}

// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshTopology.C（createTopology）
// 命名规范：foam_code.md
// 移植规范：foam_code.md
XFoam_PolyMesh* XFoam_BlockMesh::createTopology(
	const XFoam_Dictionary& meshDict,
	const XFoam_FileName& meshPath,
	const XFoam_Word& regionName)
{
	(void)meshPath;
	(void)regionName;
	(void)verboseOutput;

	checkBlockFaceOrientation = meshDict.lookupOrDefault<bool>(
		XFoam_Word("checkBlockFaceOrientation"),
		checkBlockFaceOrientation);

	scaleFactor_ = static_cast<XFoam_Scalar>(1);
	if (!meshDict.readIfPresent(XFoam_Word("convertToMeters"), scaleFactor_))
	{
		(void)meshDict.readIfPresent(XFoam_Word("scale"), scaleFactor_);
	}

	if (meshDict.subDictPtr(XFoam_Word("defaultPatch")))
	{
		// 未移植：defaultPatch name/type 与 preservePatchTypes / polyMesh 装配（OF createTopology）。
	}

	if (meshDict.found(XFoam_Word("edges")))
	{
		XFoam_BlockEdgeList edgesTmp(
			meshDict.lookup(XFoam_Word("edges")),
			XFoam_BlockEdge::INew(meshDict, geometry_, vertices_));
		edges_.transfer(edgesTmp);
	}
	if (meshDict.found(XFoam_Word("faces")))
	{
		XFoam_BlockFaceList facesTmp(
			meshDict.lookup(XFoam_Word("faces")),
			XFoam_BlockFace::INew(meshDict, geometry_));
		faces_.transfer(facesTmp);
	}
	// 移植：OpenFOAM blockMeshTopology.C ~453-460（局部 blockList + transfer）
	{
		XFoam_BlockList blocks(
			meshDict.lookup(XFoam_Word("blocks")),
			XFoam_Block::INew(meshDict, vertices_, edges_, faces_));

		transfer(blocks);
	}

	// 暂时不处理patches，只处理boundary
	if (meshDict.found(XFoam_Word("boundary")) || meshDict.found(XFoam_Word("patches")))
	{
		XFoam_WordList patchNames;
		XFoam_FaceListList tmpBlocksPatches;
		XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts;

		readBoundary(meshDict, patchNames, tmpBlocksPatches, patchDicts);

		XFoam_info() << XFoam_endl << "Creating block mesh topology" << XFoam_endl;

		XFoam_CellShapeList tmpBlockCells;
		createCellShapes(tmpBlockCells);
		// 未移植：由 tmpBlockCells / patch 数据装配完整 polyMesh（OF polyMesh + IOobject 构造）
		(void)tmpBlockCells;
		(void)patchNames;
		(void)tmpBlocksPatches;
		(void)patchDicts;
	}

	XFoam_PointField scaledVerts(vertices_);
	for (XFoam_Label vi = 0; vi < scaledVerts.size(); ++vi)
	{
		scaledVerts[vi] *= scaleFactor_;
	}
	return XFoam_newSingleHexTopology(scaledVerts);
}

void XFoam_BlockMesh::check(const XFoam_PolyMesh&, const XFoam_Dictionary&) const
{}

void XFoam_BlockMesh::calcMergeInfo()
{
	// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshMerge.C（calcMergeInfo 中 mergeList / offsets 布局）
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	// 当前实现：各 block 内点槽位连续、mergeList_ 恒等映射（无跨 block 几何合并）；完整重合点合并仍 未移植。
	const XFoam_BlockMesh& blocks = *this;
	const XFoam_Label nBlock = blocks.size();
	blockOffsets_.setSize(nBlock);
	XFoam_Label sumPoints = 0;
	XFoam_Label sumCells = 0;
	for (XFoam_Label blocki = 0; blocki < nBlock; ++blocki)
	{
		blockOffsets_[blocki] = sumPoints;
		if (blocks.set(blocki))
		{
			const XFoam_Block& b = blocks[blocki];
			sumPoints += b.nPoints();
			sumCells += b.nCells();
		}
	}
	nPoints_ = sumPoints;
	nCells_ = sumCells;
	mergeList_.setSize(sumPoints);
	for (XFoam_Label i = 0; i < sumPoints; ++i)
	{
		mergeList_[i] = i;
	}
}

void XFoam_BlockMesh::calcMergeInfoFast()
{
	// 未移植：Foam::blockMesh::calcMergeInfoFast。
}

XFoam_List<XFoam_Face> XFoam_BlockMesh::createPatchFaces(const XFoam_PolyPatch& patchTopologyFaces) const
{
	(void)patchTopologyFaces;
	return XFoam_List<XFoam_Face>();
}

XFoam_Pair<XFoam_Scalar> XFoam_BlockMesh::xCellSizes(
	const XFoam_Block& b,
	const XFoam_PointField& blockPoints,
	XFoam_Label j,
	XFoam_Label k) const
{
	return XFoam_Pair<XFoam_Scalar>(
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(1, j, k)] - blockPoints[b.pointLabel(0, j, k)]).mag())),
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(b.density().x() - 1, j, k)] - blockPoints[b.pointLabel(b.density().x(), j, k)])
				.mag())));
}

XFoam_Pair<XFoam_Scalar> XFoam_BlockMesh::yCellSizes(
	const XFoam_Block& b,
	const XFoam_PointField& blockPoints,
	XFoam_Label i,
	XFoam_Label k) const
{
	return XFoam_Pair<XFoam_Scalar>(
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(i, 0, k)] - blockPoints[b.pointLabel(i, 1, k)]).mag())),
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(i, b.density().y() - 1, k)] - blockPoints[b.pointLabel(i, b.density().y(), k)])
				.mag())));
}

XFoam_Pair<XFoam_Scalar> XFoam_BlockMesh::zCellSizes(
	const XFoam_Block& b,
	const XFoam_PointField& blockPoints,
	XFoam_Label i,
	XFoam_Label j) const
{
	return XFoam_Pair<XFoam_Scalar>(
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(i, j, 0)] - blockPoints[b.pointLabel(i, j, 1)]).mag())),
		std::abs(static_cast<double>(
			(blockPoints[b.pointLabel(i, j, b.density().z() - 1)] - blockPoints[b.pointLabel(i, j, b.density().z())])
				.mag())));
}

void XFoam_BlockMesh::printCellSizeRange(const XFoam_Pair<XFoam_Scalar>& cellSizes) const
{
	(void)cellSizes;
	// 未移植：Foam::Info 输出。
}

void XFoam_BlockMesh::printCellSizeRanges(int d, const XFoam_FixedList<XFoam_Pair<XFoam_Scalar>, 4>& cellSizes) const
{
	(void)d;
	(void)cellSizes;
}

/// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshCreate.C（Foam::blockMesh::createPoints）
/// 命名规范: foam_code.md
/// 移植规范: foam_code.md
void XFoam_BlockMesh::createPoints() const
{
	const XFoam_BlockMesh& blocks = *this;
	if (nPoints_ <= 0)
	{
		points_.clear();
		return;
	}
	points_.setSize(nPoints_);
	const XFoam_Scalar sf = scaleFactor_;
	for (XFoam_Label blocki = 0; blocki < blocks.size(); ++blocki)
	{
		if (!blocks.set(blocki))
		{
			continue;
		}
		const XFoam_PointField& blockPoints = blocks[blocki].points();
		const XFoam_Label off = blockOffsets_[blocki];
		for (XFoam_Label bpI = 0; bpI < blockPoints.size(); ++bpI)
		{
			points_[mergeList_[off + bpI]] = blockPoints[bpI] * sf;
		}
		if (verboseOutput)
		{
			const XFoam_Block& b = blocks[blocki];
			const XFoam_Label v0 = b.pointLabel(0, 0, 0);
			const XFoam_Label nx = b.density().x();
			const XFoam_Label v1 = b.pointLabel(1, 0, 0);
			const XFoam_Label vn = b.pointLabel(nx, 0, 0);
			const XFoam_Label vn1 = b.pointLabel(nx - 1, 0, 0);
			const XFoam_Scalar cwBeg = static_cast<XFoam_Scalar>(
				(blockPoints[v1] - blockPoints[v0]).mag() * static_cast<double>(sf));
			const XFoam_Scalar cwEnd = static_cast<XFoam_Scalar>(
				(blockPoints[vn] - blockPoints[vn1]).mag() * static_cast<double>(sf));
			XFoam_info() << " Block " << blocki << " cell size :" << XFoam_endl;
			XFoam_info() << " i : " << cwBeg << " .. " << cwEnd << XFoam_endl;
			const XFoam_Label ny = b.density().y();
			const XFoam_Scalar cwyBeg = static_cast<XFoam_Scalar>(
				(blockPoints[b.pointLabel(0, 1, 0)] - blockPoints[b.pointLabel(0, 0, 0)]).mag()
				* static_cast<double>(sf));
			const XFoam_Scalar cwyEnd = static_cast<XFoam_Scalar>(
				(blockPoints[b.pointLabel(0, ny, 0)] - blockPoints[b.pointLabel(0, ny - 1, 0)]).mag()
				* static_cast<double>(sf));
			XFoam_info() << " j : " << cwyBeg << " .. " << cwyEnd << XFoam_endl;
			const XFoam_Label nz = b.density().z();
			const XFoam_Scalar cwzBeg = static_cast<XFoam_Scalar>(
				(blockPoints[b.pointLabel(0, 0, 1)] - blockPoints[b.pointLabel(0, 0, 0)]).mag()
				* static_cast<double>(sf));
			const XFoam_Scalar cwzEnd = static_cast<XFoam_Scalar>(
				(blockPoints[b.pointLabel(0, 0, nz)] - blockPoints[b.pointLabel(0, 0, nz - 1)]).mag()
				* static_cast<double>(sf));
			XFoam_info() << " k : " << cwzBeg << " .. " << cwzEnd << XFoam_endl;
			XFoam_info() << XFoam_endl;
		}
	}
	// 未移植：Foam::blockMesh::inplacePointTransforms(points_)；OF 中 prescaling_/scaling_ 与 convertToMeters 的向量分量。
}

/// 移植源码: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshCreate.C（Foam::blockMesh::createCells）
/// 命名规范: foam_code.md
/// 移植规范: foam_code.md
void XFoam_BlockMesh::createCells() const
{
	const XFoam_BlockMesh& blocks = *this;
	cells_.clear();
	if (nCells_ <= 0)
	{
		return;
	}
	cells_.setSize(nCells_);
	const XFoam_CellModel& hex = XFoam_CellModel::hex();
	XFoam_Label celli = 0;
	for (XFoam_Label blocki = 0; blocki < blocks.size(); ++blocki)
	{
		if (!blocks.set(blocki))
		{
			continue;
		}
		const XFoam_List<XFoam_FixedList<XFoam_Label, 8>> blockCells = blocks[blocki].cells();
		const XFoam_Label off = blockOffsets_[blocki];
		for (XFoam_Label ci = 0; ci < blockCells.size(); ++ci)
		{
			const XFoam_FixedList<XFoam_Label, 8>& bc = blockCells[ci];
			XFoam_LabelList cellPts(8);
			for (int vi = 0; vi < 8; ++vi)
			{
				cellPts[vi] = mergeList_[off + bc[static_cast<unsigned>(vi)]];
			}
			cells_[celli] = XFoam_CellShape(hex, cellPts, true);
			++celli;
		}
	}
}

/// 移植参考: OpenFOAM src/mesh/blockMesh/blockMesh/blockMeshCreate.C（Foam::blockMesh::createPatches）
/// 命名规范: foam_code.md
/// 移植规范: foam_code.md
///
/// 端到端 remap：把 dict 中按 dict-vertex 标号给出的边界面，按"哪个 block 的哪个模型面"
/// 拆分到 Block::boundaryPatches() 预先细分好的 sub-face 列表，再加上
/// blockOffsets_ + mergeList_ 映射到 mesh-global point 标号。
/// 单 hex (1 1 1) 等价于直接重标号（每面 1 个 sub-face）；多 cell（如 (4 5 6)）
/// 即按 OF 行为细分为 nj*nk / ni*nk / ni*nj 个子四边形。
void XFoam_BlockMesh::createPatches() const
{
	patches_.clear();
	if (!meshDict_.found(XFoam_Word("boundary")) && !meshDict_.found(XFoam_Word("patches")))
	{
		return;
	}

	XFoam_WordList patchNames;
	XFoam_FaceListList tmpBlocksPatches;
	XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts;
	readBoundary(meshDict_, patchNames, tmpBlocksPatches, patchDicts);

	const XFoam_BlockMesh& blocks = *this;
	patches_.setSize(tmpBlocksPatches.size());

	for (XFoam_Label patchi = 0; patchi < tmpBlocksPatches.size(); ++patchi)
	{
		const XFoam_FaceList& dictFaces = tmpBlocksPatches[patchi];
		XFoam_FaceList outFaces;

		for (XFoam_Label di = 0; di < dictFaces.size(); ++di)
		{
			const XFoam_Face& dictFace = dictFaces[di];
			bool matched = false;

			for (XFoam_Label blockI = 0; blockI < blocks.size() && !matched; ++blockI)
			{
				if (!blocks.set(blockI))
				{
					continue;
				}
				const XFoam_Block& b = blocks[blockI];
				const XFoam_CellShape& shape = b.blockShape();
				// 仅 hex 受支持：6 个模型面。每个模型面的 4 个顶点都是 dict-vertex 标号
				// （cellShape labels 就是 hex (v0..v7) 字段里的标号）。
				for (XFoam_Label modelFaceI = 0; modelFaceI < 6 && !matched; ++modelFaceI)
				{
					const XFoam_FixedList<XFoam_Label, 4> blockFaceDict =
						shape.faceVertexLabels(modelFaceI);
					XFoam_Face cmpFace(4);
					for (XFoam_Label k = 0; k < 4; ++k)
					{
						cmpFace[k] = blockFaceDict[static_cast<unsigned>(k)];
					}
					if (!XFoam_Face::sameVertices(cmpFace, dictFace))
					{
						continue;
					}

					const XFoam_List<XFoam_Face>& subFaces =
						b.boundaryPatches()[static_cast<unsigned>(modelFaceI)];
					const XFoam_Label off = blockOffsets_[blockI];
					const XFoam_Label outBase = outFaces.size();
					outFaces.setSize(outBase + subFaces.size());
					for (XFoam_Label si = 0; si < subFaces.size(); ++si)
					{
						XFoam_Face mapped = subFaces[si];
						for (XFoam_Label vi = 0; vi < mapped.size(); ++vi)
						{
							mapped[vi] = mergeList_[off + mapped[vi]];
						}
						outFaces[outBase + si] = mapped;
					}
					matched = true;
				}
			}

			if (!matched)
			{
				const XFoam_IOerrorLocation loc(XFoam_ioLocDictName(meshDict_));
				XFoam_FatalIOErrorInFunction(loc)
					<< "boundary patch " << patchi
					<< " face " << di
					<< " (size " << dictFace.size()
					<< ") does not match any hex block face"
					<< XFoam_exit(XFoam_FatalIOError, 1);
			}
		}

		patches_[patchi] = outFaces;
	}
}
