#ifndef XFoam_IOobject_H_
#define XFoam_IOobject_H_

// 对标 OpenFOAM-13 src/OpenFOAM/db/IOobject/IOobject.H（及 IOobject.C / IOobjectI.H 核心逻辑）。
// 未移植：Foam::fileHandler、IFstream/OSspecific、NamedEnum 与 debug 开关的完全联动、
// readHeader/writeHeader 的完整字典头解析与 token 管线；相应接口在声明处标注。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_list.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_hash.h"
#include "XFoam/utilities/xfoam_tuple.h"
#include "XFoam/utilities/xfoam_regexp.h"

#include <cstring>
#include <memory>
#include <sstream>

class XFoam_IOobject;

class XFoam_Time;

class XFoam_RegIOobject;

class XFoam_ObjectRegistry;

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_IOobject>& ip);

class XFoam_API XFoam_IOobject
{
public:
	static constexpr const char* foamFile = "FoamFile";

	enum objectState
	{
		GOOD,
		BAD
	};

	enum readOption
	{
		MUST_READ,
		MUST_READ_IF_MODIFIED,
		READ_IF_PRESENT,
		NO_READ
	};

	enum writeOption
	{
		AUTO_WRITE = 0,
		NO_WRITE = 1
	};

	enum fileCheckTypes
	{
		timeStamp,
		timeStampMaster,
		inotify,
		inotifyMaster
	};

	static const XFoam_NamedEnum<fileCheckTypes, 4> fileCheckTypesNames;

private:
	XFoam_Word name_;
	XFoam_Word headerClassName_;
	XFoam_String note_;
	mutable XFoam_FileName instance_;
	XFoam_FileName local_;
	const XFoam_ObjectRegistry& db_;
	readOption rOpt_;
	writeOption wOpt_;
	bool registerObject_;
	objectState objState_;

protected:
	template<class Type>
	bool typeHeaderOk(const bool /*checkType*/)
	{
		// 未移植：typeGlobalFile 与头文件类型校验；恒 true。
		(void)sizeof(Type*);
		return true;
	}

	template<class Type>
	void warnNoRereading() const
	{
		(void)sizeof(Type*);
	}

public:
	static constexpr const char* typeName = "IOobject";

	static bool fileNameComponents(
		const XFoam_FileName& path,
		XFoam_FileName& instance,
		XFoam_FileName& local,
		XFoam_Word& name);

	template<class Name>
	static inline XFoam_Word groupName(Name name, const XFoam_Word& group)
	{
		if (!group.empty())
		{
			return XFoam_Word(XFoam_String(name) + '.' + XFoam_String(group));
		}
		return XFoam_Word(name);
	}

	static XFoam_Word group(const XFoam_Word& name);

	static XFoam_Word member(const XFoam_Word& name);

	static fileCheckTypes fileModificationChecking;

	XFoam_IOobject(
		const XFoam_Word& name,
		const XFoam_FileName& instance,
		const XFoam_ObjectRegistry& registry,
		readOption r = NO_READ,
		writeOption w = NO_WRITE,
		bool registerObject = true);

	XFoam_IOobject(
		const XFoam_Word& name,
		const XFoam_FileName& instance,
		const XFoam_FileName& local,
		const XFoam_ObjectRegistry& registry,
		readOption r = NO_READ,
		writeOption w = NO_WRITE,
		bool registerObject = true);

	XFoam_IOobject(
		const XFoam_FileName& path,
		const XFoam_ObjectRegistry& registry,
		readOption r = NO_READ,
		writeOption w = NO_WRITE,
		bool registerObject = true);

	XFoam_IOobject(
		const XFoam_IOobject& io,
		const XFoam_ObjectRegistry& registry);

	XFoam_IOobject(
		const XFoam_IOobject& io,
		const XFoam_Word& name);

	XFoam_IOobject(const XFoam_IOobject& io) = default;

	XFoam_AutoPtr<XFoam_IOobject> clone() const
	{
		return XFoam_AutoPtr<XFoam_IOobject>(new XFoam_IOobject(*this));
	}

	XFoam_AutoPtr<XFoam_IOobject> clone(const XFoam_ObjectRegistry& registry) const
	{
		return XFoam_AutoPtr<XFoam_IOobject>(new XFoam_IOobject(*this, registry));
	}

	virtual ~XFoam_IOobject();

	const XFoam_Time& time() const;

	const XFoam_ObjectRegistry& db() const { return db_; }

	const XFoam_Word& name() const { return name_; }

	const XFoam_Word& headerClassName() const { return headerClassName_; }

	XFoam_Word& headerClassName() { return headerClassName_; }

	XFoam_String& note() { return note_; }

	const XFoam_String& note() const { return note_; }

	virtual void rename(const XFoam_Word& newName) { name_ = newName; }

	bool& registerObject() { return registerObject_; }

	bool registerObject() const { return registerObject_; }

	readOption readOpt() const { return rOpt_; }

	readOption& readOpt() { return rOpt_; }

	writeOption writeOpt() const { return wOpt_; }

	writeOption& writeOpt() { return wOpt_; }

	XFoam_Word group() const;

	XFoam_Word member() const;

	const XFoam_FileName& rootPath() const;

	const XFoam_FileName& caseName(const bool global) const;

	XFoam_FileName& instance() const;

	void updateInstance() const;

	void updateTimeInstance() const;

	const XFoam_FileName& local() const { return local_; }

	XFoam_FileName path(const bool global) const;

	XFoam_FileName objectPath(const bool global) const { return path(global) / name_; }

	XFoam_FileName relativePath() const;

	XFoam_FileName relativeObjectPath() const { return relativePath() / name_; }

	// 未移植：Foam::fileHandler；返回空路径。
	XFoam_FileName filePath(const bool global) const;

	// 未移植：完整 Istream 头解析；恒 false。
	bool readHeader(XFoam_IStream&);

	bool headerOk();

	template<class Stream>
	static inline Stream& writeBanner(Stream& os, bool noHint = false)
	{
		if (noHint)
		{
			os << "/*-------------------------------------"
				  "------------------------------------*\\\n";
		}
		else
		{
			os << "/*--------------------------------*- C++ "
				  "-*----------------------------------*\\\n";
		}
		os << " ========= |\n"
		   << " \\\\ / F ield | XFoam (IO banner, OpenFOAM-compatible layout)\n"
		   << " \\\\ / O peration |\n"
		   << " \\\\ / A nd |\n"
		   << " \\\\/ M anipulation |\n"
		   << "\\*-----------------------------------------"
				  "----------------------------------*/\n";
		return os;
	}

	template<class Stream>
	static inline Stream& writeDivider(Stream& os)
	{
		os << "// * * * * * * * * * * * * * * * * * "
			  "* * * * * * * * * * * * * * * * * * * * //\n";
		return os;
	}

	template<class Stream>
	static inline Stream& writeEndDivider(Stream& os)
	{
		os << "\n\n"
		   << "// *****************************************"
			  "******************************** //\n";
		return os;
	}

	// 未移植：写标准 OpenFOAM 对象头；恒 false。
	bool writeHeader(XFoam_OStream&) const;

	bool writeHeader(XFoam_OStream&, const XFoam_Word& objectType) const;

	bool good() const { return objState_ == GOOD; }

	bool bad() const { return objState_ == BAD; }

	XFoam_InfoProxy<XFoam_IOobject> info() const { return XFoam_InfoProxy<XFoam_IOobject>(*this); }

	void operator=(const XFoam_IOobject&);
};

/*---------------------------------------------------------------------------*\
                        Class XFoam_RegIOobject Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_RegIOobject
	: public XFoam_IOobject
{
	bool registered_;
	bool ownedByRegistry_;
	mutable XFoam_List<XFoam_Label> watchIndices_;
	XFoam_Label eventNo_;
	mutable std::unique_ptr<XFoam_IStringStream> isPtr_;

	XFoam_IStream& readStream(const bool read = true);

protected:
	bool readHeaderOk(const XFoam_IOstream::streamFormat defaultFormat, const XFoam_Word& typeName);

public:
	static constexpr const char* typeName = "regIOobject";

	static float fileModificationSkew;

	XFoam_RegIOobject(const XFoam_IOobject& io, const bool isTime = false);

	XFoam_RegIOobject(const XFoam_RegIOobject&);

	XFoam_RegIOobject(XFoam_RegIOobject&&);

	XFoam_RegIOobject(const XFoam_RegIOobject&, bool registerCopy);

	XFoam_RegIOobject(const XFoam_Word& newName, const XFoam_RegIOobject&, bool registerCopy);

	XFoam_RegIOobject(const XFoam_IOobject&, const XFoam_RegIOobject&);

	virtual ~XFoam_RegIOobject();

	virtual const char* type() const;

	virtual bool global() const;

	virtual bool globalFile() const;

	using XFoam_IOobject::caseName;

	const XFoam_FileName& caseName() const;

	using XFoam_IOobject::path;

	XFoam_FileName path() const;

	using XFoam_IOobject::objectPath;

	XFoam_FileName objectPath() const { return path() / name(); }

	using XFoam_IOobject::filePath;

	XFoam_FileName filePath() const;

	bool checkIn();

	bool checkOut();

	void addWatch();

	bool registered() const { return registered_; }

	bool ownedByRegistry() const { return ownedByRegistry_; }

	void store();

	template<class Type>
	static Type& store(Type* tPtr)
	{
		if (!tPtr)
		{
			throw XFoam_Error(XFoam_String("XFoam_RegIOobject::store: null pointer"));
		}
		tPtr->XFoam_RegIOobject::checkIn();
		tPtr->XFoam_RegIOobject::ownedByRegistry_ = true;
		return *tPtr;
	}

	template<class Type>
	static Type& store(XFoam_AutoPtr<Type>& atPtr)
	{
		Type* tPtr = atPtr.ptr();
		return store(tPtr);
	}

	void release();

	XFoam_Label eventNo() const { return eventNo_; }

	XFoam_Label& eventNo() { return eventNo_; }

	bool upToDate(const XFoam_RegIOobject&) const;

	bool upToDate(const XFoam_RegIOobject&, const XFoam_RegIOobject&) const;

	bool upToDate(const XFoam_RegIOobject&, const XFoam_RegIOobject&, const XFoam_RegIOobject&) const;

	bool upToDate(
		const XFoam_RegIOobject&,
		const XFoam_RegIOobject&,
		const XFoam_RegIOobject&,
		const XFoam_RegIOobject&) const;

	void setUpToDate();

	virtual void rename(const XFoam_Word& newName);

	bool headerOk();

	XFoam_IStream& readStream(const XFoam_Word&, const bool read = true);

	void close();

	virtual bool readData(XFoam_IStream&);

	virtual bool read();

	const XFoam_List<XFoam_Label>& watchIndices() const { return watchIndices_; }

	XFoam_List<XFoam_Label>& watchIndices() { return watchIndices_; }

	virtual bool modified() const;

	virtual bool dependenciesModified() const { return false; }

	virtual bool readIfModified();

	virtual bool writeData(XFoam_OStream&) const = 0;

	virtual bool writeObject(
		XFoam_IOstream::streamFormat fmt,
		XFoam_IOstream::versionNumber ver,
		XFoam_IOstream::compressionType cmp,
		const bool write) const;

	virtual bool write(const bool write = true) const;

	void operator=(const XFoam_IOobject&) = delete;

	void operator=(const XFoam_RegIOobject&) = delete;
};

/*---------------------------------------------------------------------------*\
                       Class XFoam_ObjectRegistry Declaration
\*---------------------------------------------------------------------------*/

#pragma push_macro("toc")
#undef toc

class XFoam_API XFoam_ObjectRegistry
	: public XFoam_RegIOobject,
	  public XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >
{
	const XFoam_Time& time_;
	const XFoam_ObjectRegistry& parent_;
	XFoam_FileName dbDir_;
	mutable XFoam_Label event_{1};
	mutable XFoam_HashTable<XFoam_Pair<bool, bool>, XFoam_Word, std::hash<XFoam_String> > cacheTemporaryObjects_;
	mutable bool cacheTemporaryObjectsSet_{false};
	mutable XFoam_HashSet<XFoam_Word, std::hash<XFoam_String> > temporaryObjects_;
	mutable XFoam_List<XFoam_RegIOobject*> dependents_;

	bool parentNotTime() const;

	void readCacheTemporaryObjects() const;

	void deleteCachedObject(XFoam_RegIOobject& cachedOb) const;

public:
	static constexpr const char* typeName = "objectRegistry";

	static int debug;

	const char* type() const override;

	XFoam_ObjectRegistry(const XFoam_Time& db, const XFoam_Label nIoObjects = 128);

	XFoam_ObjectRegistry(const XFoam_IOobject& io, const XFoam_FileName& dbDir, const XFoam_Label nIoObjects = 128);

	XFoam_ObjectRegistry(const XFoam_IOobject& io, const XFoam_Label nIoObjects = 128);

	XFoam_ObjectRegistry(XFoam_ObjectRegistry&&) = default;

	XFoam_ObjectRegistry(const XFoam_ObjectRegistry&) = delete;

	virtual ~XFoam_ObjectRegistry();

	const XFoam_Time& time() const { return time_; }

	const XFoam_ObjectRegistry& parent() const { return parent_; }

	const XFoam_FileName& dbDir() const { return dbDir_; }

	XFoam_FileName path(const XFoam_Word& instance, const XFoam_FileName& local = XFoam_FileName()) const;

	using XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >::toc;

	using XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >::sortedToc;

	XFoam_WordList toc(const XFoam_Word& className) const;

	XFoam_WordList sortedToc(const XFoam_Word& className) const;

	template<class Type>
	XFoam_WordList toc() const;

	template<class Type>
	XFoam_WordList toc(const XFoam_WordRe& name) const;

	template<class Type>
	XFoam_WordList toc(const XFoam_WordReList& name) const;

	const XFoam_ObjectRegistry& subRegistry(const XFoam_Word& name, const bool forceCreate = false) const;

	template<class Type>
	XFoam_HashTable<const Type*, XFoam_Word, std::hash<XFoam_String> > lookupClass(const bool strict = false) const;

	template<class Type>
	XFoam_HashTable<Type*, XFoam_Word, std::hash<XFoam_String> > lookupClass(const bool strict = false);

	template<class Type>
	bool foundObject(const XFoam_Word& name) const;

	template<class Type>
	const Type& lookupObject(const XFoam_Word& name) const;

	template<class Type>
	Type& lookupObjectRef(const XFoam_Word& name) const;

	template<class Type>
	bool foundType(const XFoam_Word& group = XFoam_Word::null) const;

	template<class Type>
	const Type& lookupType(const XFoam_Word& group = XFoam_Word::null) const;

	XFoam_Label getEvent() const;

	const XFoam_ObjectRegistry& thisDb() const { return *this; }

	virtual void rename(const XFoam_Word& newName);

	bool checkIn(XFoam_RegIOobject&) const;

	bool checkOut(XFoam_RegIOobject&) const;

	void clear();

	bool cacheTemporaryObject(const XFoam_Word& name) const;

	template<class Object>
	bool cacheTemporaryObject(Object& ob) const;

	void resetCacheTemporaryObject(const XFoam_RegIOobject& ob) const;

	bool checkCacheTemporaryObjects() const;

	virtual bool modified() const;

	virtual bool dependenciesModified() const;

	virtual bool readIfModified();

	virtual bool read();

	void readModifiedObjects();

	void printToc(XFoam_OStream& os) const;

	virtual bool writeData(XFoam_OStream&) const;

	virtual bool writeObject(
		XFoam_IOstream::streamFormat fmt,
		XFoam_IOstream::versionNumber ver,
		XFoam_IOstream::compressionType cmp,
		const bool write) const;

	void operator=(const XFoam_ObjectRegistry&) = delete;
};

template<class Type>
inline XFoam_WordList XFoam_ObjectRegistry::toc() const
{
	XFoam_WordList out(static_cast<XFoam_Label>(size()));
	XFoam_Label n = 0;
	for (typename XFoam_HashTable<XFoam_RegIOobject*, XFoam_Word, std::hash<XFoam_String> >::const_iterator iter = cbegin();
		 iter != cend();
		 ++iter)
	{
		const XFoam_RegIOobject* p = *iter;
		if (p && p->headerClassName() == Type::typeName)
		{
			out[n++] = iter.key();
		}
	}
	out.setSize(n);
	return out;
}

template<class Type>
inline XFoam_WordList XFoam_ObjectRegistry::toc(const XFoam_WordRe& name) const
{
	(void)name;
	return XFoam_WordList();
}

template<class Type>
inline XFoam_WordList XFoam_ObjectRegistry::toc(const XFoam_WordReList& name) const
{
	(void)name;
	return XFoam_WordList();
}

template<class Type>
inline XFoam_HashTable<const Type*, XFoam_Word, std::hash<XFoam_String> > XFoam_ObjectRegistry::lookupClass(const bool strict) const
{
	(void)strict;
	return XFoam_HashTable<const Type*, XFoam_Word, std::hash<XFoam_String> >();
}

template<class Type>
inline XFoam_HashTable<Type*, XFoam_Word, std::hash<XFoam_String> > XFoam_ObjectRegistry::lookupClass(const bool strict)
{
	(void)strict;
	return XFoam_HashTable<Type*, XFoam_Word, std::hash<XFoam_String> >();
}

template<class Type>
inline bool XFoam_ObjectRegistry::foundObject(const XFoam_Word& name) const
{
	(void)name;
	return false;
}

template<class Type>
inline const Type& XFoam_ObjectRegistry::lookupObject(const XFoam_Word& name) const
{
	(void)name;
	static Type dummy;
	return dummy;
}

template<class Type>
inline Type& XFoam_ObjectRegistry::lookupObjectRef(const XFoam_Word& name) const
{
	(void)name;
	static Type dummy;
	return dummy;
}

template<class Type>
inline bool XFoam_ObjectRegistry::foundType(const XFoam_Word& group) const
{
	(void)group;
	return false;
}

template<class Type>
inline const Type& XFoam_ObjectRegistry::lookupType(const XFoam_Word& group) const
{
	(void)group;
	static Type dummy;
	return dummy;
}

template<class Object>
inline bool XFoam_ObjectRegistry::cacheTemporaryObject(Object& ob) const
{
	(void)ob;
	return false;
}

template<>
inline bool XFoam_ObjectRegistry::foundObject<XFoam_ObjectRegistry>(const XFoam_Word& name) const
{
	return found(name);
}

template<>
inline const XFoam_ObjectRegistry& XFoam_ObjectRegistry::lookupObject<XFoam_ObjectRegistry>(const XFoam_Word& name) const
{
	const_iterator iter = find(name);
	if (iter == cend() || *iter == nullptr)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_ObjectRegistry::lookupObject<objectRegistry> missing ")
			+ static_cast<const std::string&>(name));
	}
	return *static_cast<const XFoam_ObjectRegistry*>(*iter);
}

template<>
inline XFoam_ObjectRegistry& XFoam_ObjectRegistry::lookupObjectRef<XFoam_ObjectRegistry>(const XFoam_Word& name) const
{
	return const_cast<XFoam_ObjectRegistry&>(lookupObject<XFoam_ObjectRegistry>(name));
}

#pragma pop_macro("toc")

template<class Type>
struct XFoam_TypeGlobal
{
	static const bool global = false;
};

template<class Type>
struct XFoam_TypeGlobalFile
{
	static const bool global = XFoam_TypeGlobal<Type>::global;
};

inline XFoam_IOobject XFoam_unregister(const XFoam_IOobject& io)
{
	XFoam_IOobject uio(io);
	uio.registerObject() = false;
	return uio;
}

template<class Type>
class XFoam_TypeIOobject
	: public XFoam_IOobject
{
public:
	using XFoam_IOobject::XFoam_IOobject;

	explicit XFoam_TypeIOobject(const XFoam_IOobject& io)
		: XFoam_IOobject(io)
	{
	}

	bool headerOk() { return XFoam_IOobject::headerOk(); }

	using XFoam_IOobject::objectPath;

	inline XFoam_FileName objectPath() const
	{
		return objectPath(XFoam_TypeGlobalFile<Type>::global);
	}

	using XFoam_IOobject::filePath;

	inline XFoam_FileName filePath() const
	{
		return filePath(XFoam_TypeGlobalFile<Type>::global);
	}
};

#endif
