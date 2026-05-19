#include "XFoam/utilities/xfoam_ioobject.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"
#include "XFoam/utilities/xfoam_fileoperation.h"
#include "XFoam/utilities/xfoam_time.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <string>

#if defined(_MSC_VER) && _MSC_VER < 1920
#include <experimental/filesystem>
namespace xfoam_fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace xfoam_fs = std::filesystem;
#endif

#pragma push_macro("toc")
#undef toc

namespace
{
bool stripInvalidWord(XFoam_Word& w)
{
	bool bad = false;
	XFoam_String out;
	out.reserve(w.size());
	for (char c : w)
	{
		if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == ':' || c == '.'
			|| c == '-' || c == '+')
		{
			out += c;
		}
		else
		{
			bad = true;
		}
	}
	static_cast<std::string&>(w).swap(out);
	return bad;
}
} // namespace

template<>
const char* XFoam_NamedEnum<XFoam_IOobject::fileCheckTypes, 4>::names[4] = {
	"timeStamp",
	"timeStampMaster",
	"inotify",
	"inotifyMaster",
};

const XFoam_NamedEnum<XFoam_IOobject::fileCheckTypes, 4> XFoam_IOobject::fileCheckTypesNames;

XFoam_IOobject::fileCheckTypes XFoam_IOobject::fileModificationChecking = XFoam_IOobject::timeStamp;

bool XFoam_IOobject::fileNameComponents(
	const XFoam_FileName& path,
	XFoam_FileName& instance,
	XFoam_FileName& local,
	XFoam_Word& name)
{
	instance.clear();
	local.clear();
	name.clear();

	if (xfoam_fs::is_directory(xfoam_fs::path(static_cast<const std::string&>(path))))
	{
		return false;
	}

	if (path.isAbsolute())
	{
		const XFoam_String::size_type last = path.find_last_of("/\\");
		if (last == XFoam_String::npos)
		{
			return false;
		}
		instance = XFoam_FileName(path.substr(0, last));
		name = XFoam_Word(path.substr(last + 1));
	}
	else
	{
		const XFoam_String::size_type first = path.find_first_of("/\\");
		if (first == XFoam_String::npos)
		{
			name = XFoam_Word(static_cast<const std::string&>(path));
		}
		else
		{
			instance = XFoam_FileName(path.substr(0, first));
			const XFoam_String::size_type last = path.find_last_of("/\\");
			if (last > first)
			{
				local = XFoam_FileName(path.substr(first + 1, last - first - 1));
			}
			name = XFoam_Word(path.substr(last + 1));
		}
	}

	if (name.empty() || stripInvalidWord(name))
	{
		return false;
	}
	return true;
}

XFoam_Word XFoam_IOobject::group(const XFoam_Word& name)
{
	const XFoam_String::size_type i = name.find_last_of('.');
	if (i == XFoam_String::npos || i == 0)
	{
		return XFoam_Word();
	}
	return name.substr(i + 1);
}

XFoam_Word XFoam_IOobject::member(const XFoam_Word& name)
{
	const XFoam_String::size_type i = name.find_last_of('.');
	if (i == XFoam_String::npos || i == 0)
	{
		return name;
	}
	return name.substr(0, i);
}

XFoam_IOobject::XFoam_IOobject(
	const XFoam_Word& name,
	const XFoam_FileName& instance,
	const XFoam_ObjectRegistry& registry,
	readOption ro,
	writeOption wo,
	bool registerObject)
	: name_(name)
	, headerClassName_(typeName)
	, note_()
	, instance_(instance)
	, local_()
	, db_(registry)
	, rOpt_(ro)
	, wOpt_(wo)
	, registerObject_(registerObject)
	, objState_(GOOD)
{
}

XFoam_IOobject::XFoam_IOobject(
	const XFoam_Word& name,
	const XFoam_FileName& instance,
	const XFoam_FileName& local,
	const XFoam_ObjectRegistry& registry,
	readOption ro,
	writeOption wo,
	bool registerObject)
	: name_(name)
	, headerClassName_(typeName)
	, note_()
	, instance_(instance)
	, local_(local)
	, db_(registry)
	, rOpt_(ro)
	, wOpt_(wo)
	, registerObject_(registerObject)
	, objState_(GOOD)
{
}

XFoam_IOobject::XFoam_IOobject(
	const XFoam_FileName& path,
	const XFoam_ObjectRegistry& registry,
	readOption ro,
	writeOption wo,
	bool registerObject)
	: name_()
	, headerClassName_(typeName)
	, note_()
	, instance_()
	, local_()
	, db_(registry)
	, rOpt_(ro)
	, wOpt_(wo)
	, registerObject_(registerObject)
	, objState_(GOOD)
{
	if (!fileNameComponents(path, instance_, local_, name_))
	{
		throw XFoam_Error(XFoam_String("XFoam_IOobject: invalid path specification"));
	}
}

XFoam_IOobject::XFoam_IOobject(
	const XFoam_IOobject& io,
	const XFoam_ObjectRegistry& registry)
	: name_(io.name_)
	, headerClassName_(io.headerClassName_)
	, note_(io.note_)
	, instance_(io.instance_)
	, local_(io.local_)
	, db_(registry)
	, rOpt_(io.rOpt_)
	, wOpt_(io.wOpt_)
	, registerObject_(io.registerObject_)
	, objState_(io.objState_)
{
}

XFoam_IOobject::XFoam_IOobject(
	const XFoam_IOobject& io,
	const XFoam_Word& name)
	: name_(name)
	, headerClassName_(io.headerClassName_)
	, note_(io.note_)
	, instance_(io.instance_)
	, local_(io.local_)
	, db_(io.db_)
	, rOpt_(io.rOpt_)
	, wOpt_(io.wOpt_)
	, registerObject_(io.registerObject_)
	, objState_(io.objState_)
{
}

XFoam_IOobject::~XFoam_IOobject() = default;

const XFoam_Time& XFoam_IOobject::time() const
{
	return db_.time();
}

XFoam_Word XFoam_IOobject::group() const
{
	return group(name_);
}

XFoam_Word XFoam_IOobject::member() const
{
	return member(name_);
}

const XFoam_FileName& XFoam_IOobject::rootPath() const
{
	return time().rootPath();
}

const XFoam_FileName& XFoam_IOobject::caseName(const bool global) const
{
	if (global)
	{
		return time().globalCaseName();
	}
	return time().caseName();
}

XFoam_FileName& XFoam_IOobject::instance() const
{
	return instance_;
}

void XFoam_IOobject::updateInstance() const
{
	if (!instance_.isAbsolute() && instance_ != time().system() && instance_ != time().constant()
		&& instance_ != time().timeName())
	{
		char* endp = nullptr;
		(void)std::strtod(instance_.c_str(), &endp);
		if (endp != instance_.c_str())
		{
			instance_ = time().timeName();
		}
	}
}

void XFoam_IOobject::updateTimeInstance() const
{
	instance_ = time().timeName();
}

XFoam_FileName XFoam_IOobject::path(const bool global) const
{
	if (instance_.isAbsolute())
	{
		return instance_;
	}
	return rootPath() / caseName(global) / instance() / db_.dbDir() / local();
}

XFoam_FileName XFoam_IOobject::relativePath() const
{
	if (instance().isAbsolute())
	{
		return instance();
	}
	return instance() / db_.dbDir() / local();
}

// 移植源码: src/OpenFOAM/db/IOobject/IOobject.C Foam::IOobject::filePath（无 fileHandler 时退化为 objectPath）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
XFoam_FileName XFoam_IOobject::filePath(const bool global) const
{
	return objectPath(global);
}

// 移植源码: src/OpenFOAM/db/IOobject/IOobjectReadHeader.C Foam::IOobject::readHeader
// 命名规范: foam_code.md
// 移植规范: foam_code.md
bool XFoam_IOobject::readHeader(XFoam_IStream& is)
{
	if (!is.good())
	{
		if (readOpt() == MUST_READ || readOpt() == MUST_READ_IF_MODIFIED)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_IOobject::readHeader: stream not good for MUST_READ object ")
				+ static_cast<const std::string&>(name()));
		}
		objState_ = BAD;
		return false;
	}

	XFoam_Token firstToken;
	is >> firstToken;
	if (!is.good() || !firstToken.good() || !firstToken.isWord()
		|| firstToken.wordToken() != XFoam_Word(XFoam_IOobject::foamFile))
	{
		objState_ = BAD;
		return false;
	}

	XFoam_Dictionary headerDict;
	{
		XFoam_Token braceTok;
		is >> braceTok;
		if (!braceTok.good() || braceTok != XFoam_Token::BEGIN_BLOCK)
		{
			objState_ = BAD;
			return false;
		}
		for (;;)
		{
			// PrimitiveEntry 将分号放回流中（见 XFoam_PrimitiveEntry::read）；须与 Dictionary::read
			// 一样跳过 END_STATEMENT，否则会读到 ';' 而非下一关键字。
			XFoam_Token keyTok;
			do
			{
				is >> keyTok;
				if (!is.good() || !keyTok.good())
				{
					objState_ = BAD;
					return false;
				}
			} while (keyTok == XFoam_Token::END_STATEMENT);

			if (keyTok == XFoam_Token::END_BLOCK)
			{
				break;
			}
			if (!keyTok.isWord())
			{
				objState_ = BAD;
				return false;
			}
			const XFoam_KeyType kw(keyTok);
			XFoam_PrimitiveEntry* pe = nullptr;
			try
			{
				pe = new XFoam_PrimitiveEntry(kw, headerDict, is);
			}
			catch (const XFoam_Error&)
			{
				objState_ = BAD;
				return false;
			}
			catch (const std::exception&)
			{
				objState_ = BAD;
				return false;
			}
			catch (...)
			{
				objState_ = BAD;
				return false;
			}
			if (!headerDict.add(pe))
			{
				delete pe;
				objState_ = BAD;
				return false;
			}
		}
	}

	const XFoam_Entry* ev = headerDict.lookupEntryPtr("version", false, false);
	if (ev && ev->isStream())
	{
		XFoam_ITstream& vs = ev->stream();
		vs.rewind();
		is.version(XFoam_IOstream::versionNumber(vs));
	}
	else
	{
		is.version(XFoam_IOstream::currentVersion);
	}

	const XFoam_Entry* ef = headerDict.lookupEntryPtr("format", false, false);
	XFoam_Word fmtWord;
	if (!ef || !ef->isStream())
	{
		objState_ = BAD;
		return false;
	}
	{
		XFoam_ITstream& s = ef->stream();
		s.rewind();
		XFoam_Token t;
		s >> t;
		if (!t.good() || !t.isWord())
		{
			objState_ = BAD;
			return false;
		}
		fmtWord = t.wordToken();
	}
	is.format(fmtWord);

	const XFoam_Entry* ec = headerDict.lookupEntryPtr("class", false, false);
	XFoam_Word className;
	if (!ec || !ec->isStream())
	{
		objState_ = BAD;
		return false;
	}
	{
		XFoam_ITstream& s = ec->stream();
		s.rewind();
		XFoam_Token t;
		s >> t;
		if (!t.good() || !t.isWord())
		{
			objState_ = BAD;
			return false;
		}
		className = t.wordToken();
	}
	headerClassName_ = className;

	const XFoam_Entry* eo = headerDict.lookupEntryPtr("object", false, false);
	XFoam_Word objectName;
	if (!eo || !eo->isStream())
	{
		objState_ = BAD;
		return false;
	}
	{
		XFoam_ITstream& s = eo->stream();
		s.rewind();
		XFoam_Token t;
		s >> t;
		if (!t.good() || !t.isWord())
		{
			objState_ = BAD;
			return false;
		}
		objectName = t.wordToken();
	}
	(void)objectName;

	if (const XFoam_Entry* en = headerDict.lookupEntryPtr("note", false, false))
	{
		if (en->isStream())
		{
			XFoam_ITstream& ns = en->stream();
			ns.rewind();
			XFoam_Token nt;
			ns >> nt;
			if (nt.good())
			{
				if (nt.isString())
				{
					note_ = nt.stringToken();
				}
				else if (nt.isWord())
				{
					note_ = XFoam_String(static_cast<const std::string&>(nt.wordToken()));
				}
			}
		}
	}

	if (!is.good())
	{
		if (readOpt() == MUST_READ || readOpt() == MUST_READ_IF_MODIFIED)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_IOobject::readHeader: stream bad after header for object ")
				+ static_cast<const std::string&>(name()));
		}
		objState_ = BAD;
		return false;
	}

	objState_ = GOOD;
	return true;
}

bool XFoam_IOobject::headerOk()
{
	return false;
}

bool XFoam_IOobject::writeHeader(XFoam_OStream&) const
{
	return false;
}

bool XFoam_IOobject::writeHeader(XFoam_OStream&, const XFoam_Word&) const
{
	return false;
}

void XFoam_IOobject::operator=(const XFoam_IOobject& io)
{
	name_ = io.name_;
	headerClassName_ = io.headerClassName_;
	note_ = io.note_;
	instance_ = io.instance_;
	local_ = io.local_;
	rOpt_ = io.rOpt_;
	wOpt_ = io.wOpt_;
	objState_ = io.objState_;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_IOobject>& ip)
{
	return os << ip().name();
}

float XFoam_RegIOobject::fileModificationSkew = 0.0f;

XFoam_RegIOobject::XFoam_RegIOobject(const XFoam_IOobject& io, const bool isTime)
	: XFoam_IOobject(io)
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_()
	, eventNo_(isTime ? 0 : db().getEvent())
	, isPtr_()
{
	if (registerObject())
	{
		checkIn();
	}
}

XFoam_RegIOobject::XFoam_RegIOobject(const XFoam_RegIOobject& rio)
	: XFoam_IOobject(rio)
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_(rio.watchIndices_)
	, eventNo_(db().getEvent())
	, isPtr_()
{
}

XFoam_RegIOobject::XFoam_RegIOobject(XFoam_RegIOobject&& rio)
	: XFoam_IOobject(static_cast<XFoam_IOobject&&>(rio))
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_()
	, eventNo_(db().getEvent())
	, isPtr_()
{
	if (rio.registered_)
	{
		rio.checkOut();
		checkIn();
	}
}

XFoam_RegIOobject::XFoam_RegIOobject(const XFoam_RegIOobject& rio, bool registerCopy)
	: XFoam_IOobject(rio)
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_()
	, eventNo_(db().getEvent())
	, isPtr_()
{
	if (registerCopy)
	{
		if (rio.registered_)
		{
			const_cast<XFoam_RegIOobject&>(rio).checkOut();
		}
		checkIn();
	}
}

XFoam_RegIOobject::XFoam_RegIOobject(const XFoam_Word& newName, const XFoam_RegIOobject& rio, bool registerCopy)
	: XFoam_IOobject(newName, rio.instance(), rio.local(), rio.db())
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_()
	, eventNo_(db().getEvent())
	, isPtr_()
{
	if (registerCopy)
	{
		checkIn();
	}
}

XFoam_RegIOobject::XFoam_RegIOobject(const XFoam_IOobject& io, const XFoam_RegIOobject& rio)
	: XFoam_IOobject(io)
	, registered_(false)
	, ownedByRegistry_(false)
	, watchIndices_()
	, eventNo_(db().getEvent())
	, isPtr_()
{
	if (registerObject())
	{
		checkIn();
	}
}

XFoam_RegIOobject::~XFoam_RegIOobject()
{
	db().resetCacheTemporaryObject(*this);
	if (!ownedByRegistry_)
	{
		checkOut();
	}
}

const char* XFoam_RegIOobject::type() const
{
	return headerClassName().empty() ? typeName : headerClassName().c_str();
}

bool XFoam_RegIOobject::global() const
{
	return false;
}

bool XFoam_RegIOobject::globalFile() const
{
	return global();
}

const XFoam_FileName& XFoam_RegIOobject::caseName() const
{
	return XFoam_IOobject::caseName(globalFile());
}

XFoam_FileName XFoam_RegIOobject::path() const
{
	return XFoam_IOobject::path(globalFile());
}

XFoam_FileName XFoam_RegIOobject::filePath() const
{
	return XFoam_IOobject::filePath(globalFile());
}

bool XFoam_RegIOobject::headerOk()
{
	const XFoam_FileName fName(filePath());
	return XFoam_fileHandler().readHeader(*this, fName, XFoam_Word(type()));
}

bool XFoam_RegIOobject::checkIn()
{
	if (!registered_)
	{
		registered_ = db().checkIn(*this);
	}
	return registered_;
}

bool XFoam_RegIOobject::checkOut()
{
	if (registered_)
	{
		registered_ = false;
		watchIndices_.clear();
		return db().checkOut(*this);
	}
	return false;
}

void XFoam_RegIOobject::addWatch()
{
	// 未移植：Foam::fileHandler / runTimeModifiable / Pstream；无操作。
}

void XFoam_RegIOobject::store()
{
	checkIn();
	ownedByRegistry_ = true;
}

void XFoam_RegIOobject::release()
{
	ownedByRegistry_ = false;
}

bool XFoam_RegIOobject::upToDate(const XFoam_RegIOobject& a) const
{
	return a.eventNo() < eventNo_;
}

bool XFoam_RegIOobject::upToDate(const XFoam_RegIOobject& a, const XFoam_RegIOobject& b) const
{
	return a.eventNo() < eventNo_ && b.eventNo() < eventNo_;
}

bool XFoam_RegIOobject::upToDate(
	const XFoam_RegIOobject& a,
	const XFoam_RegIOobject& b,
	const XFoam_RegIOobject& c) const
{
	return a.eventNo() < eventNo_ && b.eventNo() < eventNo_ && c.eventNo() < eventNo_;
}

bool XFoam_RegIOobject::upToDate(
	const XFoam_RegIOobject& a,
	const XFoam_RegIOobject& b,
	const XFoam_RegIOobject& c,
	const XFoam_RegIOobject& d) const
{
	return a.eventNo() < eventNo_ && b.eventNo() < eventNo_ && c.eventNo() < eventNo_ && d.eventNo() < eventNo_;
}

void XFoam_RegIOobject::setUpToDate()
{
	eventNo_ = db().getEvent();
}

void XFoam_RegIOobject::rename(const XFoam_Word& newName)
{
	if (newName != name())
	{
		const bool ownedByRegistry0 = ownedByRegistry();
		release();
		checkOut();
		XFoam_IOobject::rename(newName);
		if (registerObject())
		{
			if (ownedByRegistry0)
			{
				store();
			}
			else
			{
				checkIn();
			}
		}
	}
}

bool XFoam_RegIOobject::readHeaderOk(const XFoam_IOstream::streamFormat defaultFormat, const XFoam_Word& typeNameArg)
{
	// 移植源码: src/OpenFOAM/db/regIOobject/regIOobjectRead.C regIOobject::readHeaderOk
	const bool masterOnly =
		global()
		&& (
			XFoam_IOobject::fileModificationChecking == XFoam_IOobject::timeStampMaster
			|| XFoam_IOobject::fileModificationChecking == XFoam_IOobject::inotifyMaster);

	bool isHeaderOk = false;
	if (readOpt() == XFoam_IOobject::READ_IF_PRESENT)
	{
		if (masterOnly)
		{
			// 未移植：Pstream::master() / Pstream::scatter(isHeaderOk)；串行中等价于仅本进程执行 headerOk()。
			isHeaderOk = headerOk();
		}
		else
		{
			isHeaderOk = headerOk();
		}
	}

	if (
		(readOpt() == XFoam_IOobject::MUST_READ || readOpt() == XFoam_IOobject::MUST_READ_IF_MODIFIED) || isHeaderOk)
	{
		return XFoam_fileHandler().read(*this, masterOnly, defaultFormat, typeNameArg);
	}

	return false;
}

XFoam_IStream& XFoam_RegIOobject::readStream(const bool read)
{
	if (readOpt() == XFoam_IOobject::NO_READ)
	{
		throw XFoam_Error(XFoam_String("XFoam_RegIOobject::readStream: NO_READ"));
	}
	if (!isPtr_)
	{
		isPtr_.reset(new XFoam_IStringStream(XFoam_String("")));
	}
	(void)read;
	return *isPtr_;
}

XFoam_IStream& XFoam_RegIOobject::readStream(const XFoam_Word&, const bool read)
{
	return readStream(read);
}

void XFoam_RegIOobject::close()
{
	isPtr_.reset();
}

bool XFoam_RegIOobject::readData(XFoam_IStream&)
{
	return false;
}

bool XFoam_RegIOobject::read()
{
	// 未移植：Foam::fileHandler::read 与 watch 管线；恒 false。
	return false;
}

bool XFoam_RegIOobject::modified() const
{
	return false;
}

bool XFoam_RegIOobject::readIfModified()
{
	return false;
}

bool XFoam_RegIOobject::writeObject(
	XFoam_IOstream::streamFormat fmt,
	XFoam_IOstream::versionNumber ver,
	XFoam_IOstream::compressionType cmp,
	const bool write) const
{
	(void)fmt;
	(void)ver;
	(void)cmp;
	(void)write;
	// 未移植：Foam::fileHandler::writeObject；恒 false。
	return false;
}

bool XFoam_RegIOobject::write(const bool write) const
{
	return writeObject(time().writeFormat(), XFoam_IOstream::currentVersion, time().writeCompression(), write);
}

int XFoam_ObjectRegistry::debug = 0;

bool XFoam_ObjectRegistry::parentNotTime() const
{
	return &parent_ != static_cast<const XFoam_ObjectRegistry*>(&time_);
}

void XFoam_ObjectRegistry::readCacheTemporaryObjects() const
{
	if (!cacheTemporaryObjectsSet_)
	{
		cacheTemporaryObjectsSet_ = true;
		// 未移植：time().controlDict() 中 cacheTemporaryObjects；表保持空。
	}
}

void XFoam_ObjectRegistry::deleteCachedObject(XFoam_RegIOobject& cachedOb) const
{
	cachedOb.release();
	cachedOb.checkOut();
	cachedOb.rename(XFoam_Word(static_cast<const std::string&>(cachedOb.name()) + "Cached"));
	delete &cachedOb;
}

XFoam_ObjectRegistry::XFoam_ObjectRegistry(const XFoam_Time& db, const XFoam_Label nIoObjects)
	: XFoam_RegIOobject(
		XFoam_IOobject(
			XFoam_Word("runTime"),
			XFoam_FileName(),
			static_cast<const XFoam_ObjectRegistry&>(db),
			XFoam_IOobject::NO_READ,
			XFoam_IOobject::AUTO_WRITE,
			false),
		true)
	, XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >(nIoObjects)
	, time_(db)
	, parent_(static_cast<const XFoam_ObjectRegistry&>(db))
	, dbDir_()
	, event_(1)
	, cacheTemporaryObjectsSet_(false)
	, dependents_()
{
}

XFoam_ObjectRegistry::XFoam_ObjectRegistry(const XFoam_IOobject& io, const XFoam_FileName& dbDir, const XFoam_Label nIoObjects)
	: XFoam_RegIOobject(io)
	, XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >(nIoObjects)
	, time_(io.time())
	, parent_(io.db())
	, dbDir_(dbDir)
	, event_(1)
	, cacheTemporaryObjectsSet_(false)
	, dependents_()
{
	writeOpt() = XFoam_IOobject::AUTO_WRITE;
}

XFoam_ObjectRegistry::XFoam_ObjectRegistry(const XFoam_IOobject& io, const XFoam_Label nIoObjects)
	: XFoam_ObjectRegistry(
		io,
		io.db().dbDir() / io.local() / XFoam_FileName(static_cast<const std::string&>(io.name())),
		nIoObjects)
{
}

XFoam_ObjectRegistry::~XFoam_ObjectRegistry()
{
	cacheTemporaryObjects_.clear();
	clear();
}

const char* XFoam_ObjectRegistry::type() const
{
	return typeName;
}

XFoam_FileName XFoam_ObjectRegistry::path(const XFoam_Word& instance, const XFoam_FileName& local) const
{
	return time().rootPath() / time().caseName() / XFoam_FileName(static_cast<const std::string&>(instance)) / dbDir_
		/ local;
}

XFoam_WordList XFoam_ObjectRegistry::toc(const XFoam_Word& className) const
{
	XFoam_WordList objectNames(static_cast<XFoam_Label>(size()));
	XFoam_Label count = 0;
	for (const_iterator iter = cbegin(); iter != cend(); ++iter)
	{
		const XFoam_RegIOobject* p = *iter;
		if (p && XFoam_Word(p->type()) == className)
		{
			objectNames[count++] = iter.key();
		}
	}
	objectNames.setSize(count);
	return objectNames;
}

XFoam_WordList XFoam_ObjectRegistry::sortedToc(const XFoam_Word& className) const
{
	XFoam_WordList sortedLst = toc(className);
	std::sort(sortedLst.begin(), sortedLst.end());
	return sortedLst;
}

const XFoam_ObjectRegistry& XFoam_ObjectRegistry::subRegistry(const XFoam_Word& name, const bool forceCreate) const
{
	if (forceCreate && !this->template foundObject<XFoam_ObjectRegistry>(name))
	{
		XFoam_IOobject io(
			name,
			time().constant(),
			const_cast<XFoam_ObjectRegistry&>(*this),
			XFoam_IOobject::NO_READ,
			XFoam_IOobject::NO_WRITE,
			false);
		XFoam_ObjectRegistry* fieldsCachePtr = new XFoam_ObjectRegistry(io, XFoam_FileName(), 128);
		fieldsCachePtr->store();
	}
	return this->template lookupObject<XFoam_ObjectRegistry>(name);
}

XFoam_Label XFoam_ObjectRegistry::getEvent() const
{
	XFoam_Label curEvent = event_++;
	if (event_ == std::numeric_limits<XFoam_Label>::max())
	{
		curEvent = 1;
		event_ = 2;
		for (iterator iter = const_cast<XFoam_ObjectRegistry&>(*this).begin();
			 iter != const_cast<XFoam_ObjectRegistry&>(*this).end();
			 ++iter)
		{
			XFoam_RegIOobject& io = *(*iter);
			if (io.eventNo() != 0)
			{
				io.eventNo() = curEvent;
			}
		}
	}
	return curEvent;
}

void XFoam_ObjectRegistry::rename(const XFoam_Word& newName)
{
	XFoam_RegIOobject::rename(newName);
	const XFoam_String::size_type i = dbDir_.rfind('/');
	if (i == XFoam_String::npos)
	{
		dbDir_ = XFoam_FileName(static_cast<const std::string&>(newName));
	}
	else
	{
		dbDir_.replace(i + 1, XFoam_String::npos, static_cast<const std::string&>(newName));
	}
}

bool XFoam_ObjectRegistry::checkIn(XFoam_RegIOobject& io) const
{
	readCacheTemporaryObjects();
	XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >& cacheTab =
		const_cast<XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >&>(cacheTemporaryObjects_);
	XFoam_ObjectRegistry& reg = const_cast<XFoam_ObjectRegistry&>(*this);
	if (cacheTemporaryObjects_.size())
	{
		typename XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >::iterator cacheIter =
			cacheTab.find(io.name());
		if (cacheIter != cacheTab.end())
		{
			iterator iter = reg.find(io.name());
			if (iter != reg.end() && *iter != &io && (*iter)->ownedByRegistry())
			{
				cacheIter().first() = false;
				deleteCachedObject(**iter);
			}
		}
	}
	return reg.insert(io.name(), &io);
}

bool XFoam_ObjectRegistry::checkOut(XFoam_RegIOobject& io) const
{
	XFoam_ObjectRegistry& reg = const_cast<XFoam_ObjectRegistry&>(*this);
	iterator iter = reg.find(io.name());
	if (iter != reg.end())
	{
		if (*iter != &io)
		{
			return false;
		}
		XFoam_RegIOobject* object = *iter;
		const bool hasErased = reg.erase(iter);
		if (io.ownedByRegistry())
		{
			delete object;
		}
		return hasErased;
	}
	return false;
}

void XFoam_ObjectRegistry::clear()
{
	XFoam_List<XFoam_RegIOobject*> myObjects(static_cast<XFoam_Label>(size()));
	XFoam_Label nMyObjects = 0;
	for (iterator iter = begin(); iter != end(); ++iter)
	{
		if ((*iter)->ownedByRegistry())
		{
			myObjects[nMyObjects++] = *iter;
		}
	}
	myObjects.setSize(nMyObjects);
	for (XFoam_Label i = 0; i < nMyObjects; ++i)
	{
		checkOut(*myObjects[i]);
	}
}

bool XFoam_ObjectRegistry::cacheTemporaryObject(const XFoam_Word& name) const
{
	const XFoam_ObjectRegistry& root = time_;
	readCacheTemporaryObjects();
	return root.cacheTemporaryObjects_.found(name);
}

void XFoam_ObjectRegistry::resetCacheTemporaryObject(const XFoam_RegIOobject& ob) const
{
	if (cacheTemporaryObjects_.size())
	{
		typename XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >::iterator iter =
			const_cast<XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >&>(cacheTemporaryObjects_)
				.find(ob.name());
		if (iter != cacheTemporaryObjects_.end())
		{
			iter().first() = false;
		}
	}
	if (this != &time_)
	{
		time_.resetCacheTemporaryObject(ob);
	}
}

bool XFoam_ObjectRegistry::checkCacheTemporaryObjects() const
{
	for (const_iterator iter = cbegin(); iter != cend(); ++iter)
	{
		const XFoam_ObjectRegistry* orPtr = dynamic_cast<const XFoam_ObjectRegistry*>(*iter);
		if (orPtr && orPtr != this)
		{
			orPtr->checkCacheTemporaryObjects();
		}
	}
	const XFoam_ObjectRegistry& root = time_;
	if (root.cacheTemporaryObjects_.empty())
	{
		return false;
	}
	if (this != &root)
	{
		const_cast<XFoam_ObjectRegistry*>(this)->cacheTemporaryObjects_.clear();
	}
	else
	{
		XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >& tab =
			const_cast<XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >&>(root.cacheTemporaryObjects_);
		for (typename XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> >::iterator it = tab.begin();
			 it != tab.end();
			 ++it)
		{
			it().second() = false;
		}
	}
	const_cast<XFoam_ObjectRegistry*>(this)->temporaryObjects_.clear();
	return true;
}

bool XFoam_ObjectRegistry::modified() const
{
	for (const_iterator iter = cbegin(); iter != cend(); ++iter)
	{
		if ((*iter)->modified())
		{
			return true;
		}
	}
	return false;
}

bool XFoam_ObjectRegistry::dependenciesModified() const
{
	dependents_.setSize(static_cast<XFoam_Label>(size()));
	XFoam_Label count = 0;
	for (const_iterator iter = cbegin(); iter != cend(); ++iter)
	{
		if ((*iter)->dependenciesModified())
		{
			dependents_[count++] = *iter;
		}
	}
	dependents_.setSize(count);
	return count != 0;
}

bool XFoam_ObjectRegistry::readIfModified()
{
	bool modifiedFlag = false;
	for (iterator iter = begin(); iter != end(); ++iter)
	{
		modifiedFlag = modifiedFlag || (*iter)->readIfModified();
	}
	return modifiedFlag;
}

bool XFoam_ObjectRegistry::read()
{
	bool readOk = true;
	for (XFoam_Label i = 0; i < dependents_.size(); ++i)
	{
		dependents_[i]->read();
	}
	(void)readOk;
	return readOk;
}

void XFoam_ObjectRegistry::readModifiedObjects()
{
	dependenciesModified();
	const bool mod = readIfModified();
	if (mod)
	{
		XFoam_ObjectRegistry::read();
	}
}

void XFoam_ObjectRegistry::printToc(XFoam_OStream& os) const
{
	const std::vector<XFoam_Word> keys = sortedToc();
	for (std::size_t i = 0; i < keys.size(); ++i)
	{
		const_iterator iter = find(keys[i]);
		if (iter != cend() && *iter != nullptr)
		{
			os << "    " << keys[i] << ' ' << (*iter)->type() << '\n';
		}
	}
}

bool XFoam_ObjectRegistry::writeData(XFoam_OStream&) const
{
	return false;
}

bool XFoam_ObjectRegistry::writeObject(
	XFoam_IOstream::streamFormat fmt,
	XFoam_IOstream::versionNumber ver,
	XFoam_IOstream::compressionType cmp,
	const bool write) const
{
	bool ok = true;
	for (const_iterator iter = cbegin(); iter != cend(); ++iter)
	{
		if ((*iter)->writeOpt() != XFoam_IOobject::NO_WRITE)
		{
			ok = (*iter)->writeObject(fmt, ver, cmp, write) && ok;
		}
	}
	return ok;
}

#pragma pop_macro("toc")
