#ifndef XFoam_stream_H_
#define XFoam_stream_H_

// 对标 OpenFOAM db/IOstreams（IOstreams/、Sstreams/、StringStreams/）与 token。
// XFoam_IStream / XFoam_OStream：对标 Foam::Istream / Ostream（定义于此头；xfoam_types.h 仅前向声明）。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_autoptr.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

class XFoam_Token;
class XFoam_KeyType;
class XFoam_IStream;
class XFoam_OStream;
class XFoam_OSstream;
class XFoam_VerbatimString;
class XFoam_Dictionary;

/*---------------------------------------------------------------------------*\
                        Class XFoam_InfoProxy Declaration
\*---------------------------------------------------------------------------*/

template<class T>
class XFoam_InfoProxy
{
public:
	const T& t_;

	explicit XFoam_InfoProxy(const T& t)
		: t_(t)
	{
	}

	const T& operator()() const { return t_; }
};

/*---------------------------------------------------------------------------*\
                          Class XFoam_IOstream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_IOstream
{
public:
	enum streamAccess
	{
		OPENED,
		CLOSED
	};

	enum streamFormat
	{
		ASCII,
		BINARY
	};

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const streamFormat& sf);

	class versionNumber
	{
		XFoam_Scalar versionNumber_;
		int index_;

	public:
		explicit versionNumber(const XFoam_Scalar num)
			: versionNumber_(num)
			, index_(numberToIndex(num))
		{
		}

		versionNumber(XFoam_IStream& is);

		int numberToIndex(const XFoam_Scalar num) const { return int(10 * num + 1e-6); }

		int majorVersion() const { return int(versionNumber_); }

		int minorVersion() const { return int(10.0 * (versionNumber_ - majorVersion())); }

		XFoam_String str() const;

		bool operator==(const versionNumber& vn) const { return index_ == vn.index_; }
		bool operator!=(const versionNumber& vn) const { return index_ != vn.index_; }
		bool operator<(const versionNumber& vn) const { return index_ < vn.index_; }
		bool operator<=(const versionNumber& vn) const { return index_ <= vn.index_; }
		bool operator>(const versionNumber& vn) const { return index_ > vn.index_; }
		bool operator>=(const versionNumber& vn) const { return index_ >= vn.index_; }

		friend XFoam_OStream& operator<<(XFoam_OStream& os, const versionNumber& vn);
	};

	enum compressionType
	{
		UNCOMPRESSED,
		COMPRESSED
	};

	static const versionNumber currentVersion;
	static unsigned int precision_;

private:
	static XFoam_FileName name_;
	streamFormat format_;
	versionNumber version_;
	compressionType compression_;
	bool global_;
	streamAccess openClosed_;
	std::ios_base::iostate ioState_;

protected:
	XFoam_Label lineNumber_;

	void setOpened() { openClosed_ = OPENED; }
	void setClosed() { openClosed_ = CLOSED; }
	void setState(std::ios_base::iostate state) { ioState_ = state; }
	void setGood() { ioState_ = std::ios_base::iostate(0); }

public:
	XFoam_IOstream(
		const streamFormat format,
		const versionNumber version,
		const compressionType compression = UNCOMPRESSED,
		const bool global = false)
		: format_(format)
		, version_(version)
		, compression_(compression)
		, global_(global)
		, openClosed_(CLOSED)
		, ioState_(std::ios_base::iostate(0))
		, lineNumber_(0)
	{
		setBad();
	}

	virtual ~XFoam_IOstream() = default;

	virtual const XFoam_FileName& name() const { return name_; }
	virtual XFoam_FileName& name() { return name_; }

	virtual bool check(const char* operation) const;
	void fatalCheck(const char* operation) const;

	bool opened() const { return openClosed_ == OPENED; }
	bool closed() const { return openClosed_ == CLOSED; }
	bool good() const { return ioState_ == 0; }
	bool eof() const { return (ioState_ & std::ios_base::eofbit) != 0; }
	bool fail() const { return (ioState_ & (std::ios_base::badbit | std::ios_base::failbit)) != 0; }
	bool bad() const { return (ioState_ & std::ios_base::badbit) != 0; }

	operator void*() const
	{
		return fail() ? reinterpret_cast<void*>(0) : reinterpret_cast<void*>(-1);
	}
	bool operator!() const { return fail(); }

	static streamFormat formatEnum(const XFoam_Word&);
	streamFormat format() const { return format_; }
	streamFormat format(const streamFormat fmt)
	{
		streamFormat fmt0 = format_;
		format_ = fmt;
		return fmt0;
	}
	streamFormat format(const XFoam_Word& fmt)
	{
		streamFormat fmt0 = format_;
		format_ = formatEnum(fmt);
		return fmt0;
	}

	versionNumber version() const { return version_; }
	versionNumber version(const versionNumber ver)
	{
		versionNumber ver0 = version_;
		version_ = ver;
		return ver0;
	}

	static compressionType compressionEnum(const XFoam_Word&);
	compressionType compression() const { return compression_; }
	compressionType compression(const compressionType cmp)
	{
		compressionType cmp0 = compression_;
		compression_ = cmp;
		return cmp0;
	}
	compressionType compression(const XFoam_Word& cmp)
	{
		compressionType cmp0 = compression_;
		compression_ = compressionEnum(cmp);
		return cmp0;
	}

	bool global() const { return global_; }
	bool& global() { return global_; }

	XFoam_Label lineNumber() const { return lineNumber_; }
	XFoam_Label& lineNumber() { return lineNumber_; }
	XFoam_Label lineNumber(const XFoam_Label ln)
	{
		XFoam_Label ln0 = lineNumber_;
		lineNumber_ = ln;
		return ln0;
	}

	virtual std::ios_base::fmtflags flags() const = 0;

	static unsigned int defaultPrecision() { return precision_; }
	static unsigned int defaultPrecision(unsigned int p)
	{
		unsigned int precision0 = precision_;
		precision_ = p;
		return precision0;
	}

	void setEof() { ioState_ |= std::ios_base::eofbit; }
	void setFail() { ioState_ |= std::ios_base::failbit; }
	void setBad() { ioState_ |= std::ios_base::badbit; }

	virtual std::ios_base::fmtflags flags(const std::ios_base::fmtflags f) = 0;

	std::ios_base::fmtflags setf(const std::ios_base::fmtflags f) { return flags(flags() | f); }

	std::ios_base::fmtflags setf(const std::ios_base::fmtflags f, const std::ios_base::fmtflags mask)
	{
		return flags((flags() & ~mask) | (f & mask));
	}

	void unsetf(const std::ios_base::fmtflags uf) { flags(flags() & ~uf); }

	virtual void print(XFoam_OStream&) const;
	void print(XFoam_OStream&, const int streamState) const;

	XFoam_InfoProxy<XFoam_IOstream> info() const { return XFoam_InfoProxy<XFoam_IOstream>(*this); }
};

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOstream::streamFormat& sf);
XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOstream::versionNumber& vn);

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_IOstream>& ip);

typedef XFoam_IOstream& (*XFoam_IOstreamManip)(XFoam_IOstream&);

inline XFoam_IOstream& operator<<(XFoam_IOstream& io, XFoam_IOstreamManip f)
{
	return f(io);
}

inline XFoam_IOstream& dec(XFoam_IOstream& io)
{
	io.setf(std::ios_base::dec, std::ios_base::dec | std::ios_base::hex | std::ios_base::oct);
	return io;
}

inline XFoam_IOstream& hex(XFoam_IOstream& io)
{
	io.setf(std::ios_base::hex, std::ios_base::dec | std::ios_base::hex | std::ios_base::oct);
	return io;
}

inline XFoam_IOstream& oct(XFoam_IOstream& io)
{
	io.setf(std::ios_base::oct, std::ios_base::dec | std::ios_base::hex | std::ios_base::oct);
	return io;
}

inline XFoam_IOstream& fixed(XFoam_IOstream& io)
{
	io.setf(std::ios_base::fixed, std::ios_base::floatfield);
	return io;
}

inline XFoam_IOstream& scientific(XFoam_IOstream& io)
{
	io.setf(std::ios_base::scientific, std::ios_base::floatfield);
	return io;
}

/*---------------------------------------------------------------------------*\
                           Class XFoam_OStream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_OStream
	: public XFoam_IOstream
{
protected:
	static const unsigned short indentSize_ = 4;
	unsigned short indentLevel_;

public:
	XFoam_OStream(
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED,
		const bool global = false)
		: XFoam_IOstream(format, version, compression, global)
		, indentLevel_(0)
	{
	}

	~XFoam_OStream() override = default;

	virtual XFoam_OStream& write(const char) = 0;
	virtual XFoam_OStream& write(const char*) = 0;
	virtual XFoam_OStream& write(const XFoam_Word&) = 0;
	virtual XFoam_OStream& write(const XFoam_String&) = 0;
	virtual XFoam_OStream& write(const XFoam_VerbatimString&) = 0;
	virtual XFoam_OStream& writeQuoted(const std::string&, const bool quoted = true) = 0;
	virtual XFoam_OStream& write(const int32_t) = 0;
	virtual XFoam_OStream& write(const int64_t) = 0;
	virtual XFoam_OStream& write(const uint32_t) = 0;
	virtual XFoam_OStream& write(const uint64_t) = 0;
	virtual XFoam_OStream& write(const float) = 0;
	virtual XFoam_OStream& write(const double) = 0;
	virtual XFoam_OStream& write(const long double) = 0;
	virtual XFoam_OStream& write(const char*, XFoam_StreamSize) = 0;
	virtual void indent() = 0;

	unsigned short indentLevel() const { return indentLevel_; }
	unsigned short& indentLevel() { return indentLevel_; }
	void incrIndent() { indentLevel_++; }
	void decrIndent();

	XFoam_OStream& writeKeyword(const XFoam_KeyType&);

	virtual void flush() = 0;
	virtual void endl() = 0;
	virtual int width() const = 0;
	virtual int width(const int w) = 0;
	virtual int precision() const = 0;
	virtual int precision(const int p) = 0;

	XFoam_OStream& operator()() const { return const_cast<XFoam_OStream&>(*this); }
};

typedef XFoam_OStream& (*XFoam_OStreamManip)(XFoam_OStream&);

inline XFoam_OStream& operator<<(XFoam_OStream& os, XFoam_OStreamManip f)
{
	return f(os);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, XFoam_IOstreamManip f)
{
	f(os);
	return os;
}

inline XFoam_OStream& indent(XFoam_OStream& os)
{
	os.indent();
	return os;
}

inline XFoam_OStream& incrIndent(XFoam_OStream& os)
{
	os.incrIndent();
	return os;
}

inline XFoam_OStream& decrIndent(XFoam_OStream& os)
{
	os.decrIndent();
	return os;
}

inline XFoam_OStream& flush(XFoam_OStream& os)
{
	os.flush();
	return os;
}

inline XFoam_OStream& XFoam_endl(XFoam_OStream& os)
{
	os.endl();
	return os;
}

static const char XFoam_tab = '\t';
static const char XFoam_nl = '\n';

/*---------------------------------------------------------------------------*\
                      Class XFoam_VerbatimString Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_VerbatimString
	: public XFoam_String
{
public:
	XFoam_VerbatimString() = default;

	explicit XFoam_VerbatimString(const XFoam_String& s)
		: XFoam_String(s)
	{
	}

	explicit XFoam_VerbatimString(const char* s)
		: XFoam_String(s ? s : "")
	{
	}

	static const XFoam_VerbatimString null;
};

/*---------------------------------------------------------------------------*\
                        Class XFoam_Token Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Token
{
public:
	// XFoam：将 OpenFOAM token 中的 floatScalar / doubleScalar / longDoubleScalar 合并为单一 tokenType::SCALAR；
	// 数值均以 XFoam_Scalar 存储（默认与 double 主导的标量配置一致，见 XFOAM_SCALAR_* / xfoam_types.h）。
	enum tokenType : char
	{
		UNDEFINED = 0,

		PUNCTUATION = char(128),
		WORD,
		FUNCTIONNAME,
		VARIABLE,
		STRING,
		VERBATIMSTRING,
		LABEL,
		SCALAR,
		COMPOUND,

		ERROR
	};

	enum punctuationToken : char
	{
		NULL_TOKEN = '\0',
		SPACE = ' ',
		TAB = '\t',
		NL = '\n',

		END_STATEMENT = ';',
		BEGIN_LIST = '(',
		END_LIST = ')',
		BEGIN_SQR = '[',
		END_SQR = ']',
		BEGIN_BLOCK = '{',
		END_BLOCK = '}',
		COLON = ':',
		COMMA = ',',
		HASH = '#',

		BEGIN_STRING = '"',
		END_STRING = BEGIN_STRING,

		ASSIGN = '=',
		ADD = '+',
		SUBTRACT = '-',
		MULTIPLY = '*',
		DIVIDE = '/'
	};

	class compound
		: public XFoam_RefCount
	{
		bool empty_;

	public:
		static const char* const typeName;

		compound();

		compound(const compound&) = delete;

		static XFoam_AutoPtr<compound> New(const XFoam_Word& type, XFoam_IStream& is);

		virtual ~compound();

		static bool isCompound(const XFoam_Word& name);

		bool empty() const { return empty_; }

		bool& empty() { return empty_; }

		virtual XFoam_Label size() const = 0;

		virtual void write(XFoam_OStream& os) const = 0;

		virtual const char* type() const = 0;

		void operator=(const compound&) = delete;

		friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const compound& c);
	};

	template<class T>
	class Compound
		: public compound,
		  public T
	{
	public:
		static constexpr const char* typeName = "Compound";

		explicit Compound(XFoam_IStream& is)
			: T(is)
		{
		}

		XFoam_Label size() const override { return static_cast<XFoam_Label>(T::size()); }

		void write(XFoam_OStream& os) const override { os << static_cast<const T&>(*this); }

		const char* type() const override { return typeName; }
	};

	static XFoam_Token undefinedToken;

private:
	tokenType type_;

	union {
		punctuationToken punctuationToken_;
		XFoam_Word* wordTokenPtr_;
		XFoam_FunctionName* functionNameTokenPtr_;
		XFoam_Variable* variableTokenPtr_;
		XFoam_String* stringTokenPtr_;
		XFoam_VerbatimString* verbatimStringTokenPtr_;
		XFoam_Label labelToken_;
		XFoam_Scalar scalarToken_;
		mutable compound* compoundTokenPtr_;
	};

	XFoam_Label lineNumber_;

	void clear();

	void parseError(const char* expected) const;

public:
	static const char* const typeName;

	XFoam_Token();

	XFoam_Token(const XFoam_Token&);

	XFoam_Token(punctuationToken p, XFoam_Label lineNumber = 0);

	XFoam_Token(const XFoam_Word& w, XFoam_Label lineNumber = 0);

	XFoam_Token(const XFoam_String& s, XFoam_Label lineNumber = 0);

	XFoam_Token(const XFoam_VerbatimString& vs, XFoam_Label lineNumber = 0);

	XFoam_Token(const XFoam_Label l, XFoam_Label lineNumber = 0);

	/// 标量 token：类型恒为 \c SCALAR，值以 \c XFoam_Scalar 存储。
	XFoam_Token(XFoam_Scalar s, XFoam_Label lineNumber = 0);

	explicit XFoam_Token(XFoam_IStream& is);

	~XFoam_Token();

	tokenType type() const;

	tokenType& type();

	bool good() const;

	bool undefined() const;

	bool error() const;

	bool isPunctuation() const;

	punctuationToken pToken() const;

	bool isWord() const;

	const XFoam_Word& wordToken() const;

	bool isFunctionName() const;

	const XFoam_FunctionName& functionNameToken() const;

	bool isVariable() const;

	const XFoam_Variable& variableToken() const;

	bool isString() const;

	const XFoam_String& stringToken() const;

	bool isVerbatimString() const;

	const XFoam_VerbatimString& verbatimStringToken() const;

	bool isAnyString() const;

	const XFoam_String& anyStringToken() const;

	bool isLabel() const;

	XFoam_Label labelToken() const;

	bool isScalar() const;

	XFoam_Scalar scalarToken() const;

	bool isNumber() const;

	XFoam_Scalar number() const;

	bool isCompound() const;

	const compound& compoundToken() const;

	compound& transferCompoundToken(const XFoam_IStream& is);

	XFoam_Label lineNumber() const;

	XFoam_Label& lineNumber();

	void setBad();

	void operator=(const XFoam_Token&);

	void operator=(punctuationToken p);

	void operator=(XFoam_Word* wPtr);

	void operator=(const XFoam_Word& w);

	void operator=(XFoam_FunctionName* fnPtr);

	void operator=(const XFoam_FunctionName& fn);

	void operator=(XFoam_Variable* vPtr);

	void operator=(const XFoam_Variable& v);

	void operator=(XFoam_String* sPtr);

	void operator=(const XFoam_String& s);

	void operator=(XFoam_VerbatimString* vsPtr);

	void operator=(const XFoam_VerbatimString& vs);

	void operator=(const XFoam_Label l);

	void operator=(XFoam_Scalar s);

	void operator=(compound* cPtr);

	bool operator==(const XFoam_Token& t) const;

	bool operator==(punctuationToken p) const;

	bool operator==(const XFoam_Word& w) const;

	bool operator==(const XFoam_FunctionName& fn) const;

	bool operator==(const XFoam_Variable& v) const;

	bool operator==(const XFoam_String& s) const;

	bool operator==(const XFoam_VerbatimString& vs) const;

	bool operator==(const XFoam_Label l) const;

	bool operator==(XFoam_Scalar s) const;

	bool operator!=(const XFoam_Token& t) const;

	bool operator!=(punctuationToken p) const;

	bool operator!=(const XFoam_Word& w) const;

	bool operator!=(const XFoam_FunctionName& fn) const;

	bool operator!=(const XFoam_Variable& v) const;

	bool operator!=(const XFoam_String& s) const;

	bool operator!=(const XFoam_VerbatimString& vs) const;

	bool operator!=(const XFoam_Label l) const;

	bool operator!=(XFoam_Scalar s) const;

	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Token& t);

	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Token& t);

	/// \c XFoam_OStream 即 \c std::ostream；仅保留此重载，避免与 \c std::ostream& 版本签名重复。
	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, punctuationToken pt);

	XFoam_InfoProxy<XFoam_Token> info() const { return XFoam_InfoProxy<XFoam_Token>(*this); }
};

/*---------------------------------------------------------------------------*\
                           Class XFoam_IStream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_IStream
	: public XFoam_IOstream
{
	bool putBack_;
	XFoam_Token putBackToken_;

public:
	XFoam_IStream(
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED,
		const bool global = false)
		: XFoam_IOstream(format, version, compression, global)
		, putBack_(false)
	{
	}

	~XFoam_IStream() override = default;

	virtual void putBack(const XFoam_Token& t);
	bool getBack(XFoam_Token&);
	bool peekBack(XFoam_Token&);

	virtual XFoam_IStream& read(XFoam_Token&) = 0;
	virtual XFoam_IStream& read(char&) = 0;
	virtual XFoam_IStream& read(XFoam_Word&) = 0;
	virtual XFoam_IStream& read(XFoam_String&) = 0;
	virtual XFoam_IStream& read(int32_t&) = 0;
	virtual XFoam_IStream& read(int64_t&) = 0;
	virtual XFoam_IStream& read(uint32_t&) = 0;
	virtual XFoam_IStream& read(uint64_t&) = 0;
	virtual XFoam_IStream& read(float&) = 0;
	virtual XFoam_IStream& read(double&) = 0;
	virtual XFoam_IStream& read(long double&) = 0;
	virtual XFoam_IStream& read(char*, XFoam_StreamSize) = 0;
	virtual XFoam_IStream& rewind() = 0;

	virtual int peek() = 0;
	virtual XFoam_IStream& get(char& c) = 0;
	virtual XFoam_IStream& get() = 0;
	virtual XFoam_IStream& putback(const char c) = 0;

	XFoam_IStream& readBegin(const char* funcName);
	XFoam_IStream& readEnd(const char* funcName);
	XFoam_IStream& readEndBegin(const char* funcName);
	char readBeginList(const char* funcName);
	char readEndList(const char* funcName);

	XFoam_IStream& operator()() const;
};

typedef XFoam_IStream& (*XFoam_IStreamManip)(XFoam_IStream&);

inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_IStreamManip f)
{
	return f(is);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_IOstreamManip f)
{
	f(is);
	return is;
}

/*---------------------------------------------------------------------------*\
                          Class XFoam_ISstream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_ISstream
	: public XFoam_IStream
{
	static const int bufInitialCapacity = 1024;
	static const int bufErrorLength = 80;

	XFoam_FileName name_;
	std::istream& is_;
	XFoam_CharBuffer buf_;

	char nextValid();
	XFoam_IStream& readVerbatim(XFoam_VerbatimString&);
	XFoam_IStream& readVariable(XFoam_String&);
	XFoam_IStream& readDelimited(XFoam_String&, const char begin, const char end);
	void readWordToken(XFoam_Token&);

public:
	inline XFoam_ISstream(
		std::istream& is,
		const XFoam_String& name,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED);

	~XFoam_ISstream() override = default;

	const XFoam_FileName& name() const override { return name_; }
	XFoam_FileName& name() override { return name_; }

	std::ios_base::fmtflags flags() const override;

	int peek() override;
	XFoam_IStream& get(char& c) override;
	XFoam_IStream& get() override;
	XFoam_IStream& putback(const char c) override;

	XFoam_ISstream& getLine(XFoam_String&, const bool continuation = true);
	XFoam_IStream& readList(XFoam_String&);
	XFoam_IStream& readBlock(XFoam_String&);

	XFoam_IStream& read(XFoam_Token&) override;
	XFoam_IStream& read(char&) override;
	XFoam_IStream& read(XFoam_Word&) override;
	XFoam_IStream& read(XFoam_String&) override;
	XFoam_IStream& read(int32_t&) override;
	XFoam_IStream& read(int64_t&) override;
	XFoam_IStream& read(uint32_t&) override;
	XFoam_IStream& read(uint64_t&) override;
	XFoam_IStream& read(float&) override;
	XFoam_IStream& read(double&) override;
	XFoam_IStream& read(long double&) override;
	XFoam_IStream& read(char*, XFoam_StreamSize) override;
	XFoam_IStream& rewind() override;

	std::ios_base::fmtflags flags(const std::ios_base::fmtflags flags) override;

	std::istream& stdStream() { return is_; }
	const std::istream& stdStream() const { return is_; }

	void print(XFoam_OStream&) const override;

	void operator=(const XFoam_ISstream&) = delete;
};

/*---------------------------------------------------------------------------*\
                          Class XFoam_OSstream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_OSstream
	: public XFoam_OStream
{
	XFoam_FileName name_;
	std::ostream& os_;

public:
	XFoam_OSstream(
		std::ostream& os,
		const XFoam_String& name,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED);

	XFoam_OSstream(const XFoam_OSstream&) = default;

	const XFoam_FileName& name() const override { return name_; }
	XFoam_FileName& name() override { return name_; }

	std::ios_base::fmtflags flags() const override;

	XFoam_OStream& write(const char) override;
	XFoam_OStream& write(const char*) override;
	XFoam_OStream& write(const XFoam_Word&) override;
	XFoam_OStream& write(const XFoam_String&) override;
	XFoam_OStream& write(const XFoam_VerbatimString&) override;
	XFoam_OStream& writeQuoted(const std::string&, const bool quoted = true) override;
	XFoam_OStream& write(const int32_t) override;
	XFoam_OStream& write(const int64_t) override;
	XFoam_OStream& write(const uint32_t) override;
	XFoam_OStream& write(const uint64_t) override;
	XFoam_OStream& write(const float) override;
	XFoam_OStream& write(const double) override;
	XFoam_OStream& write(const long double) override;
	XFoam_OStream& write(const char*, XFoam_StreamSize) override;
	void indent() override;

	std::ios_base::fmtflags flags(const std::ios_base::fmtflags flags) override;
	void flush() override;
	void endl() override;
	int width() const override;
	int width(const int) override;
	int precision() const override;
	int precision(const int) override;

	std::ostream& stdStream() { return os_; }
	const std::ostream& stdStream() const { return os_; }

	void print(XFoam_OStream&) const override;

	void operator=(const XFoam_OSstream&) = delete;
};

/*---------------------------------------------------------------------------*\
        Class XFoam_IFstream / XFoam_OFstream Declaration（对标 Fstreams）
\*---------------------------------------------------------------------------*/
// 移植源码: src/OpenFOAM/db/IOstreams/Fstreams/IFstream.H
// 移植源码: src/OpenFOAM/db/IOstreams/Fstreams/OFstream.H
// 命名规范: foam_code.md
// 移植规范: foam_code.md
//
// std::ifstream / std::ofstream 必须先于 XFoam_ISstream / XFoam_OSstream 完成构造，
// 再绑定引用；不得写「: XFoam_ISstream(ifs_, …), ifs_(…)」（基类先于成员，属 UB）。
//
// OpenFOAM 的 IFstreamAllocator：先构造底层流（必要时 new ifstream / igzstream、.orig 回退），
// 再让 ISstream 绑定 *ifPtr_。XFoam 尚未移植压缩读与多态替换，故用内嵌 std::ifstream 的首基类
// 即可满足构造顺序；职责与 OF 的 Allocator 层一致，故沿用 Allocator 命名。

struct XFoam_IFstreamAllocator
{
	std::ifstream ifs_;

	explicit XFoam_IFstreamAllocator(const XFoam_FileName& path)
		: ifs_(static_cast<const XFoam_String&>(path), std::ios::binary)
	{
	}
};

class XFoam_API XFoam_IFstream
	: XFoam_IFstreamAllocator,
	  public XFoam_ISstream
{
	XFoam_FileName filePath_;
public:
	XFoam_IFstream(
		const XFoam_FileName& path,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion)
		: XFoam_IFstreamAllocator(path)
		, XFoam_ISstream(ifs_, static_cast<const XFoam_String&>(path), format, version)
		, filePath_(path)
	{
	}

	//- Return the name of the stream
	const XFoam_FileName& name() const
	{
		return filePath_;
	}

	//- Return non-const access to the name of the stream
	XFoam_FileName& name()
	{
		return filePath_;
	}
	// STL stream
	//- Access to underlying std::istream
	virtual std::istream& stdStream() { return ifs_; }
	//- Const access to underlying std::istream
	virtual const std::istream& stdStream() const { return ifs_; }	
};

struct XFoam_OFstreamAllocator
{
	std::ofstream ofs_;

	explicit XFoam_OFstreamAllocator(const XFoam_FileName& path)
		: ofs_(static_cast<const XFoam_String&>(path), std::ios::binary | std::ios::trunc)
	{
	}
};

class XFoam_API XFoam_OFstream
	: XFoam_OFstreamAllocator,
	  public XFoam_OSstream
{
	XFoam_FileName filePath_;
public:
	XFoam_OFstream(
		const XFoam_FileName& path,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED)
		: XFoam_OFstreamAllocator(path)
		, XFoam_OSstream(ofs_, static_cast<const XFoam_String&>(path), format, version, compression)
		, filePath_(path)
	{
	}

	//- Return the name of the stream
	const XFoam_FileName& name() const
	{
		return filePath_;
	}
	
	//- Return non-const access to the name of the stream
	XFoam_FileName& name()
	{
		return filePath_;
	}

	// STL stream
	//- Access to underlying std::ostream
	virtual std::ostream& stdStream() { return ofs_; }
	//- Const access to underlying std::ostream
	virtual const std::ostream& stdStream() const { return ofs_; }
};

/*---------------------------------------------------------------------------*\
                       Class XFoam_PrefixOSstream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_PrefixOSstream
	: public XFoam_OSstream
{
	bool printPrefix_;
	XFoam_String prefix_;

	void checkWritePrefix();

public:
	XFoam_PrefixOSstream(
		std::ostream& os,
		const XFoam_String& name,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion,
		const compressionType compression = UNCOMPRESSED);

	XFoam_OStream& write(const char) override;
	XFoam_OStream& write(const char*) override;
	XFoam_OStream& write(const XFoam_Word&) override;
	XFoam_OStream& write(const XFoam_String&) override;
	XFoam_OStream& write(const XFoam_VerbatimString&) override;
	XFoam_OStream& writeQuoted(const std::string&, const bool quoted = true) override;
	XFoam_OStream& write(const int32_t) override;
	XFoam_OStream& write(const int64_t) override;
	XFoam_OStream& write(const uint32_t) override;
	XFoam_OStream& write(const uint64_t) override;
	XFoam_OStream& write(const float) override;
	XFoam_OStream& write(const double) override;
	XFoam_OStream& write(const long double) override;
	XFoam_OStream& write(const char*, XFoam_StreamSize) override;
	void indent() override;

	void print(XFoam_OStream&) const override;
};

/*---------------------------------------------------------------------------*\
                        Class XFoam_MessageStream Declaration
	对标 OpenFOAM db/error/messageStream.H（流式 Serious / Warning / Info 等）。
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_MessageStream
{
public:
	enum class ErrorSeverity
	{
		Info,
		Warning,
		Serious,
		Fatal
	};

protected:
	XFoam_String title_;
	ErrorSeverity severity_;
	int maxErrors_;
	int errorCount_;

public:
	static int level;

	XFoam_MessageStream(const XFoam_String& title, const ErrorSeverity severity, const int maxErrors = 0);

	const XFoam_String& title() const { return title_; }

	int maxErrors() const { return maxErrors_; }

	int& maxErrors() { return maxErrors_; }

	XFoam_OSstream& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber = 0);

	XFoam_OSstream& operator()(
		const XFoam_String& functionName,
		const char* sourceFileName,
		int sourceFileLineNumber = 0);

	XFoam_OSstream& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber,
		const XFoam_String& ioFileName,
		XFoam_Label ioLineNumber = -1);

	XFoam_OSstream& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber,
		const XFoam_IOstream& ioStream);

	XFoam_OSstream& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber,
		const XFoam_Dictionary& dict);

	XFoam_OSstream& operator()(XFoam_Label communicator = -1);

	operator XFoam_OSstream&() { return operator()(); }

private:
	XFoam_OSstream& streamLeadIn_(const char* functionName, const char* sourceFileName, int sourceFileLineNumber);
};

extern XFoam_API XFoam_MessageStream XFoam_seriousError;
extern XFoam_API XFoam_MessageStream XFoam_info;
extern XFoam_API bool XFoam_writeInfoHeader;

/*---------------------------------------------------------------------------*\
                        Class XFoam_IStringStream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_IStringStream
	: public XFoam_ISstream
{
public:
	explicit XFoam_IStringStream(
		const XFoam_String& buffer,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion)
		: XFoam_ISstream(
			  *(new std::istringstream(buffer)),
			  "IStringStream.sourceFile",
			  format,
			  version)
	{
	}

	XFoam_IStringStream(
		const XFoam_String& name,
		const XFoam_String& buffer,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion)
		: XFoam_ISstream(*(new std::istringstream(buffer)), name, format, version)
	{
	}

	explicit XFoam_IStringStream(
		const char* buffer,
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion)
		: XFoam_ISstream(
			  *(new std::istringstream(buffer ? buffer : "")),
			  "IStringStream.sourceFile",
			  format,
			  version)
	{
	}

	~XFoam_IStringStream() override { delete &dynamic_cast<std::istringstream&>(stdStream()); }

	XFoam_String str() const { return dynamic_cast<const std::istringstream&>(stdStream()).str(); }

	void print(XFoam_OStream&) const override;

	XFoam_IStream& operator()() const { return const_cast<XFoam_IStringStream&>(*this); }
};

/*---------------------------------------------------------------------------*\
                        Class XFoam_OStringStream Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_OStringStream
	: public XFoam_OSstream
{
public:
	explicit XFoam_OStringStream(
		const streamFormat format = ASCII,
		const versionNumber version = XFoam_IOstream::currentVersion)
		: XFoam_OSstream(*(new std::ostringstream()), "OStringStream.sinkFile", format, version)
	{
	}

	XFoam_OStringStream(const XFoam_OStringStream& oss)
		: XFoam_OSstream(
			  *(
				  new std::ostringstream(
					  dynamic_cast<const std::ostringstream&>(oss.stdStream()).str())),
			  oss.name(),
			  oss.format(),
			  oss.version())
	{
	}

	~XFoam_OStringStream() override { delete &dynamic_cast<std::ostringstream&>(stdStream()); }

	XFoam_String str() const { return dynamic_cast<const std::ostringstream&>(stdStream()).str(); }

	void rewind() { stdStream().rdbuf()->pubseekpos(0); }

	void print(XFoam_OStream&) const override;
};

extern XFoam_API XFoam_ISstream XFoam_sin;
extern XFoam_API XFoam_PrefixOSstream XFoam_sout;
extern XFoam_API XFoam_OSstream XFoam_serr;
extern XFoam_API XFoam_PrefixOSstream XFoam_pout;
extern XFoam_API XFoam_PrefixOSstream XFoam_perr;

inline XFoam_ISstream::XFoam_ISstream(
	std::istream& is,
	const XFoam_String& name,
	const streamFormat format,
	const versionNumber version,
	const compressionType compression)
	: XFoam_IStream(format, version, compression)
	, name_(name)
	, is_(is)
	, buf_()
{
	buf_.reserve(static_cast<std::size_t>(bufInitialCapacity));
	if (is_.good())
	{
		setOpened();
		setGood();
	}
	else
	{
		setState(is_.rdstate());
	}
}

inline XFoam_IStream& XFoam_ISstream::get(char& c)
{
	is_.get(c);
	setState(is_.rdstate());
	if (c == '\n')
	{
		lineNumber_++;
	}
	return *this;
}

inline XFoam_IStream& XFoam_ISstream::get()
{
	char c;
	return get(c);
}

inline int XFoam_ISstream::peek()
{
	return is_.peek();
}

inline XFoam_IStream& XFoam_ISstream::putback(const char c)
{
	if (c == '\n')
	{
		lineNumber_--;
	}
	if (!is_.putback(c))
	{
		setBad();
	}
	setState(is_.rdstate());
	return *this;
}

template<class T>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<T>& ip)
{
	return os << ip.t_;
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_Token>& ip);

XFoam_API std::ostream& operator<<(std::ostream& os, const XFoam_InfoProxy<XFoam_Token>& ip);

XFoam_API std::ostream& operator<<(std::ostream& os, const XFoam_Token& t);

inline XFoam_OStream& operator<<(XFoam_OStream& os, const char* s)
{
	return os.write(s ? s : "");
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_String& s)
{
	return os.write(s);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, char c)
{
	return os.write(c);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, int32_t v)
{
	return os.write(v);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, unsigned int v)
{
	return os.write(static_cast<uint32_t>(v));
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, long long v)
{
	return os.write(static_cast<int64_t>(v));
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, unsigned long long v)
{
	return os.write(static_cast<uint64_t>(v));
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, float v)
{
	return os.write(v);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, double v)
{
	return os.write(v);
}

inline XFoam_OStream& operator<<(XFoam_OStream& os, long double v)
{
	return os.write(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, char& c)
{
	return is.read(c);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_String& s)
{
	return is.read(s);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Word& w)
{
	return is.read(w);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, int32_t& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, int64_t& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, uint32_t& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, uint64_t& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, float& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, double& v)
{
	return is.read(v);
}

inline XFoam_IStream& operator>>(XFoam_IStream& is, long double& v)
{
	return is.read(v);
}

XFoam_API bool XFoam_readTokenFromStream(XFoam_IStream& is, XFoam_Label& lineNo, XFoam_Token& tok);

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// 对标 messageStream.H 末尾便捷宏（doc/foam_code.md：对象/函数式宏为 XFoam_ + lowerCamel）。

#ifndef XFOAM_FUNCTION_NAME
#define XFoam_seriousErrorInFunction XFoam_seriousErrorIn(__func__)
#define XFoam_seriousIoErrorInFunction(ios) XFoam_seriousIoErrorIn(__func__, (ios))
#define XFoam_infoInFunction XFoam_infoIn(__func__)
#define XFoam_ioInfoInFunction(ios) XFoam_ioInfoIn(__func__, (ios))
#define XFoam_debugInFunction \
	if (debug) XFoam_infoInFunction
#else
#define XFoam_seriousErrorInFunction XFoam_seriousErrorIn(FUNCTION_NAME)
#define XFoam_seriousIoErrorInFunction(ios) XFoam_seriousIoErrorIn(FUNCTION_NAME, (ios))
#define XFoam_infoInFunction XFoam_infoIn(FUNCTION_NAME)
#define XFoam_ioInfoInFunction(ios) XFoam_ioInfoIn(FUNCTION_NAME, (ios))
#define XFoam_debugInFunction \
	if (debug) XFoam_infoInFunction
#endif

#define XFoam_seriousErrorIn(functionName) XFoam_seriousError((functionName), __FILE__, __LINE__)

#define XFoam_seriousIoErrorIn(functionName, ios) XFoam_seriousError((functionName), __FILE__, __LINE__, (ios))

#define XFoam_infoIn(functionName) XFoam_info((functionName), __FILE__, __LINE__)

#define XFoam_infoHeader \
	if (XFoam_writeInfoHeader) XFoam_info

#define XFoam_log \
	if (log) XFoam_info

#define XFoam_ioInfoIn(functionName, ios) XFoam_info((functionName), __FILE__, __LINE__, (ios))

#define XFoam_debugInfo \
	if (debug) XFoam_info

#define XFoam_debugVar(var)                                                                    \
	do                                                                                         \
	{                                                                                          \
		XFoam_pout << "[" << __FILE__ << ":" << __LINE__ << "] " << #var << " " << (var)      \
				   << XFoam_endl;                                                            \
	} while (0)

#endif
