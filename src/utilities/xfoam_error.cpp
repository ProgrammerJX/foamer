#include "XFoam/utilities/xfoam_error.h"
#include "XFoam/utilities/xfoam_stream.h"

void XFoam_Error::appendToStream(const XFoam_String& s)
{
	streamBuf_ += s;
}

void XFoam_Error::buildPreamble(XFoam_String& out) const
{
	if (immediate_)
	{
		out += immediateText_;
		return;
	}
	if (!title_.empty())
	{
		out += title_;
	}
	out += streamBuf_;
	if (!functionName_.empty())
	{
		out += "\n    From function ";
		out += functionName_;
		if (!sourceFileName_.empty())
		{
			out += "\n    in file ";
			out += sourceFileName_;
			out += " at line ";
			out += XFoam_String(std::to_string(sourceLine_));
		}
		out += '.';
	}
}

XFoam_Error::XFoam_Error(const char* title)
	: title_(title ? title : "")
{}

XFoam_Error::XFoam_Error(const XFoam_String& completeMessage)
	: immediate_(true)
	, immediateText_(completeMessage)
{}

XFoam_String XFoam_Error::message() const
{
	XFoam_String out;
	buildPreamble(out);
	return out;
}

const char* XFoam_Error::what() const noexcept
{
	try
	{
		whatBuffer_.clear();
		buildPreamble(whatBuffer_);
		return whatBuffer_.c_str();
	}
	catch (...)
	{
		return "XFoam_Error";
	}
}

XFoam_Error& XFoam_Error::operator()(
	const char* functionName,
	const char* sourceFileName,
	int sourceFileLineNumber)
{
	functionName_ = functionName ? functionName : "";
	sourceFileName_ = sourceFileName ? sourceFileName : "";
	sourceLine_ = sourceFileLineNumber;
	return *this;
}

void XFoam_Error::exit(int errNo)
{
	std::cerr << what() << std::endl;
	std::exit(errNo);
}

void XFoam_Error::abort()
{
	std::cerr << what() << std::endl;
	std::abort();
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Error& err)
{
	return os << err.what();
}

XFoam_IOerrorLocation::XFoam_IOerrorLocation(
	const XFoam_String& ioFileName,
	XFoam_Label ioStartLineNumber,
	XFoam_Label ioEndLineNumber,
	bool ioGlobal)
	: ioFileName_(ioFileName)
	, ioStartLineNumber_(ioStartLineNumber)
	, ioEndLineNumber_(ioEndLineNumber)
	, ioGlobal_(ioGlobal)
{}

XFoam_IOerror::XFoam_IOerror(const char* title)
	: XFoam_Error(title)
{}

XFoam_IOerror& XFoam_IOerror::operator()(
	const char* functionName,
	const char* sourceFileName,
	int sourceFileLineNumber,
	const XFoam_IOerrorLocation& location)
{
	XFoam_Error::operator()(functionName, sourceFileName, sourceFileLineNumber);
	static_cast<XFoam_IOerrorLocation&>(*this) = location;
	return *this;
}

const char* XFoam_IOerror::what() const noexcept
{
	try
	{
		whatBuffer_.clear();
		buildPreamble(whatBuffer_);
		if (!ioFileName().empty())
		{
			whatBuffer_ += "\n    Reading ";
			if (ioGlobal())
			{
				whatBuffer_ += "global ";
			}
			whatBuffer_ += "IOobject ";
			whatBuffer_ += ioFileName();
			if (ioStartLineNumber() >= 0)
			{
				whatBuffer_ += " at line ";
				whatBuffer_ += XFoam_String(std::to_string(ioStartLineNumber()));
				if (ioEndLineNumber() >= 0 && ioEndLineNumber() != ioStartLineNumber())
				{
					whatBuffer_ += " to ";
					whatBuffer_ += XFoam_String(std::to_string(ioEndLineNumber()));
				}
			}
		}
		return whatBuffer_.c_str();
	}
	catch (...)
	{
		return "XFoam_IOerror";
	}
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOerror& err)
{
	return os << err.what();
}

XFoam_Error XFoam_FatalError("--> FOAM FATAL ERROR: ");
XFoam_IOerror XFoam_FatalIOError("--> FOAM FATAL IO ERROR: ");
