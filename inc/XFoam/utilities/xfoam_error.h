#ifndef XFoam_Error_H_
#define XFoam_Error_H_

// 对齐 OpenFOAM db/error/error.H的核心概念：可流式拼接的 FatalError / FatalIOError 与全局实例。
// 未移植：dictionary 转换、jobInfo、并行 Pstream、栈回溯等。（messageStream 见 xfoam_stream.h 中 XFoam_MessageStream。）

#include "XFoam/utilities/xfoam_types.h"

class XFoam_Error;
class XFoam_IOerror;
class XFoam_OStream;

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Error& err);

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// 对标 OpenFOAM db/error/errorManip.H：exit / abort 流操纵子。

template<class Err>
class XFoam_ErrorManip
{
	friend class XFoam_Error;

	void (Err::*fPtr_)();
	Err& err_;

public:
	XFoam_ErrorManip(void (Err::*fPtr)(), Err& t)
		: fPtr_(fPtr)
		, err_(t)
	{}
};

template<class Err, class T>
class XFoam_ErrorManipArg
{
	friend class XFoam_Error;

	void (Err::*fPtr_)(const T);
	Err& err_;
	T arg_;

public:
	XFoam_ErrorManipArg(void (Err::*fPtr)(const T), Err& t, const T i)
		: fPtr_(fPtr)
		, err_(t)
		, arg_(i)
	{}
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

class XFoam_API XFoam_Error : public std::exception
{
protected:
	XFoam_String title_;
	XFoam_String functionName_;
	XFoam_String sourceFileName_;
	XFoam_Label sourceLine_{0};
	XFoam_String streamBuf_;
	XFoam_String immediateText_;
	mutable XFoam_String whatBuffer_;
	bool immediate_{false};

	void appendToStream(const XFoam_String& s);
	void buildPreamble(XFoam_String& out) const;

public:
	explicit XFoam_Error(const char* title);
	explicit XFoam_Error(const XFoam_String& completeMessage);

	XFoam_Error(const XFoam_Error&) = default;
	XFoam_Error& operator=(const XFoam_Error&) = default;
	XFoam_Error(XFoam_Error&&) noexcept = default;
	XFoam_Error& operator=(XFoam_Error&&) noexcept = default;

	~XFoam_Error() noexcept override = default;

	const XFoam_String& title() const { return title_; }
	XFoam_String message() const;

	const char* what() const noexcept override;

	void clearMessageContext() noexcept;

	XFoam_Error& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber = 0);

	void exit(int errNo = 1);
	void abort();

	template<class Err>
	XFoam_Error& operator<<(XFoam_ErrorManip<Err> m)
	{
		(m.err_.*m.fPtr_)();
		return *this;
	}

	template<class Err, class ArgT>
	XFoam_Error& operator<<(XFoam_ErrorManipArg<Err, ArgT> m)
	{
		(m.err_.*m.fPtr_)(m.arg_);
		return *this;
	}

	template<class T>
	XFoam_Error& operator<<(const T& t)
	{
		if (immediate_)
		{
			std::ostringstream oss;
			oss << t;
			immediateText_ += oss.str();
			return *this;
		}
		std::ostringstream oss;
		oss << t;
		appendToStream(oss.str());
		return *this;
	}
};

inline XFoam_Error& operator<<(XFoam_Error& e, const char* s)
{
	e << XFoam_String(s ? s : "");
	return e;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

class XFoam_API XFoam_IOerrorLocation
{
	XFoam_String ioFileName_;
	XFoam_Label ioStartLineNumber_{-1};
	XFoam_Label ioEndLineNumber_{-1};
	bool ioGlobal_{false};

public:
	XFoam_IOerrorLocation() = default;

	XFoam_IOerrorLocation(
		const XFoam_String& ioFileName,
		XFoam_Label ioStartLineNumber = -1,
		XFoam_Label ioEndLineNumber = -1,
		bool ioGlobal = false);

	const XFoam_String& ioFileName() const { return ioFileName_; }
	XFoam_Label ioStartLineNumber() const { return ioStartLineNumber_; }
	XFoam_Label ioEndLineNumber() const { return ioEndLineNumber_; }
	bool ioGlobal() const { return ioGlobal_; }
};

class XFoam_API XFoam_IOerror : public XFoam_Error, public XFoam_IOerrorLocation
{
public:
	explicit XFoam_IOerror(const char* title);

	XFoam_IOerror& operator()(
		const char* functionName,
		const char* sourceFileName,
		int sourceFileLineNumber,
		const XFoam_IOerrorLocation& location);

	const char* what() const noexcept override;
};

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOerror& err);

inline XFoam_ErrorManip<XFoam_Error> XFoam_abort(XFoam_Error& err)
{
	return XFoam_ErrorManip<XFoam_Error>(&XFoam_Error::abort, err);
}

inline XFoam_ErrorManipArg<XFoam_Error, int> XFoam_exit(XFoam_Error& err, const int errNo = 1)
{
	return XFoam_ErrorManipArg<XFoam_Error, int>(&XFoam_Error::exit, err, errNo);
}

inline XFoam_ErrorManip<XFoam_IOerror> XFoam_abort(XFoam_IOerror& err)
{
	return XFoam_ErrorManip<XFoam_IOerror>(&XFoam_IOerror::abort, err);
}

inline XFoam_ErrorManipArg<XFoam_IOerror, int> XFoam_exit(XFoam_IOerror& err, const int errNo = 1)
{
	return XFoam_ErrorManipArg<XFoam_IOerror, int>(&XFoam_IOerror::exit, err, errNo);
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

extern XFoam_API XFoam_Error XFoam_FatalError;
extern XFoam_API XFoam_IOerror XFoam_FatalIOError;

template<class... T>
constexpr bool XFoam_False = false;

#define XFoam_FatalErrorIn(functionName) \
	XFoam_FatalError.operator()((functionName), __FILE__, __LINE__)

#ifndef XFOAM_FUNCTION_NAME
#define XFoam_FatalErrorInFunction XFoam_FatalErrorIn(__func__)
#else
#define XFoam_FatalErrorInFunction XFoam_FatalErrorIn(FUNCTION_NAME)
#endif

#define XFoam_FatalIOErrorIn(functionName, location) \
	XFoam_FatalIOError.operator()((functionName), __FILE__, __LINE__, (location))

#ifndef XFOAM_FUNCTION_NAME
#define XFoam_FatalIOErrorInFunction(location) \
	XFoam_FatalIOError.operator()(__func__, __FILE__, __LINE__, (location))
#else
#define XFoam_FatalIOErrorInFunction(location) \
	XFoam_FatalIOError.operator()(FUNCTION_NAME, __FILE__, __LINE__, (location))
#endif

#endif
