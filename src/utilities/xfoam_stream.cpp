#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"
#include <cctype>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>

namespace
{
inline bool xf_scalar_equal(XFoam_Scalar a, XFoam_Scalar b)
{
	return std::abs(a - b) < 1e-12;
}

class XFoam_NullCompound
	: public XFoam_Token::compound
{
public:
	XFoam_Label size() const override { return 0; }

	void write(XFoam_OStream& os) const override { (void)os; }

	const char* type() const override { return "null"; }
};

const XFoam_NullCompound xf_nullCompound;

inline const char* xf_bufCStr(const XFoam_CharBuffer& b)
{
	return b.empty() ? "" : &b[0];
}

inline void xf_bufMarkParseError(XFoam_CharBuffer& buf, const int markIndex)
{
	if (static_cast<int>(buf.size()) <= markIndex)
	{
		buf.resize(static_cast<XFoam_Size>(markIndex) + 1, '\0');
	}
	if (!buf.empty())
	{
		buf.back() = '\0';
	}
	buf[static_cast<XFoam_Size>(markIndex)] = '\0';
}

static bool xf_parseLabel(const char* buf, XFoam_Label& val)
{
	if (!buf || !*buf)
	{
		return false;
	}
	char* end = nullptr;
	const long long v = std::strtoll(buf, &end, 10);
	if (end == buf || *end != '\0')
	{
		return false;
	}
	if (v > std::numeric_limits<XFoam_Label>::max() || v < std::numeric_limits<XFoam_Label>::min())
	{
		return false;
	}
	val = static_cast<XFoam_Label>(v);
	return true;
}

static bool xf_parseScalar(const char* buf, XFoam_Scalar& val)
{
	if (!buf || !*buf)
	{
		return false;
	}
	char* end = nullptr;
	val = static_cast<XFoam_Scalar>(std::strtod(buf, &end));
	return end != buf && *end == '\0';
}

} // namespace

const XFoam_IOstream::versionNumber XFoam_IOstream::currentVersion(2.0);
unsigned int XFoam_IOstream::precision_(6);
XFoam_FileName XFoam_IOstream::name_("IOstream");

XFoam_IOstream::versionNumber::versionNumber(XFoam_IStream& is)
	: versionNumber_(0)
	, index_(0)
{
	double s = 0;
	is.read(s);
	versionNumber_ = static_cast<XFoam_Scalar>(s);
	index_ = numberToIndex(versionNumber_);
}

XFoam_String XFoam_IOstream::versionNumber::str() const
{
	std::ostringstream os;
	os.precision(1);
	os.setf(std::ios_base::fixed, std::ios_base::floatfield);
	os << versionNumber_;
	return os.str();
}

XFoam_IOstream::streamFormat XFoam_IOstream::formatEnum(const XFoam_Word& format)
{
	if (format == "ascii")
	{
		return XFoam_IOstream::ASCII;
	}
	if (format == "binary")
	{
		return XFoam_IOstream::BINARY;
	}
	std::cerr << "--> XFoam Warning : bad format specifier '" << format << "', using 'ascii'" << std::endl;
	return XFoam_IOstream::ASCII;
}

XFoam_IOstream::compressionType XFoam_IOstream::compressionEnum(const XFoam_Word& compression)
{
	if (compression == "true" || compression == "on" || compression == "yes")
	{
		return XFoam_IOstream::COMPRESSED;
	}
	if (compression == "false" || compression == "off" || compression == "no")
	{
		return XFoam_IOstream::UNCOMPRESSED;
	}
	if (compression == "uncompressed")
	{
		return XFoam_IOstream::UNCOMPRESSED;
	}
	if (compression == "compressed")
	{
		return XFoam_IOstream::COMPRESSED;
	}
	std::cerr << "--> XFoam Warning : bad compression specifier '" << compression
			  << "', using 'uncompressed'" << std::endl;
	return XFoam_IOstream::UNCOMPRESSED;
}

bool XFoam_IOstream::check(const char* operation) const
{
	if (bad())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "error in IOstream " << name() << " for operation " << operation << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return !bad();
}

void XFoam_IOstream::fatalCheck(const char* operation) const
{
	if (bad())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "error in IOstream " << name() << " for operation " << operation << XFoam_exit(XFoam_FatalIOError, 1);
	}
}

void XFoam_IOstream::print(XFoam_OStream& os) const
{
	os << "IOstream: "
	   << "Version " << version_ << ", format ";
	switch (format_)
	{
	case ASCII:
		os << "ASCII";
		break;
	case BINARY:
		os << "BINARY";
		break;
	}
	os << ", line " << lineNumber();
	if (opened())
	{
		os << ", OPENED";
	}
	if (closed())
	{
		os << ", CLOSED";
	}
	if (good())
	{
		os << ", GOOD";
	}
	if (eof())
	{
		os << ", EOF";
	}
	if (fail())
	{
		os << ", FAIL";
	}
	if (bad())
	{
		os << ", BAD";
	}
	XFoam_endl(os);
}

void XFoam_IOstream::print(XFoam_OStream& os, const int streamState) const
{
	if (streamState == std::ios_base::goodbit)
	{
		os << "ios_base::goodbit set : the last operation on stream succeeded";
		XFoam_endl(os);
	}
	else if (streamState & std::ios_base::badbit)
	{
		os << "ios_base::badbit set : characters possibly lost";
		XFoam_endl(os);
	}
	else if (streamState & std::ios_base::failbit)
	{
		os << "ios_base::failbit set : some type of formatting error";
		XFoam_endl(os);
	}
	else if (streamState & std::ios_base::eofbit)
	{
		os << "ios_base::eofbit set : at end of stream";
		XFoam_endl(os);
	}
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOstream::streamFormat& sf)
{
	if (sf == XFoam_IOstream::ASCII)
	{
		os << "ascii";
	}
	else
	{
		os << "binary";
	}
	return os;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_IOstream::versionNumber& vn)
{
	os << vn.str().c_str();
	return os;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_IOstream>& ip)
{
	ip.t_.print(os);
	return os;
}

void XFoam_IStream::putBack(const XFoam_Token& t)
{
	if (bad())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Attempt to put back onto bad stream" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (putBack_)
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Attempt to put back another token" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	putBackToken_ = t;
	putBack_ = true;
}

bool XFoam_IStream::getBack(XFoam_Token& t)
{
	if (bad())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Attempt to get back from bad stream" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	if (putBack_)
	{
		t = putBackToken_;
		putBack_ = false;
		return true;
	}
	return false;
}

bool XFoam_IStream::peekBack(XFoam_Token& t)
{
	if (putBack_)
	{
		t = putBackToken_;
	}
	else
	{
		t = XFoam_Token::undefinedToken;
	}
	return putBack_;
}

XFoam_IStream& XFoam_IStream::readBegin(const char* funcName)
{
	XFoam_Token delimiter(*this);
	if (delimiter != XFoam_Token::BEGIN_LIST)
	{
		setBad();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Expected a '" << XFoam_Token::BEGIN_LIST << "' while reading " << funcName << ", found " << delimiter.info()
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
	return *this;
}

XFoam_IStream& XFoam_IStream::readEnd(const char* funcName)
{
	XFoam_Token delimiter(*this);
	if (delimiter != XFoam_Token::END_LIST)
	{
		setBad();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Expected a '" << XFoam_Token::END_LIST << "' while reading " << funcName << ", found " << delimiter.info()
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
	return *this;
}

XFoam_IStream& XFoam_IStream::readEndBegin(const char* funcName)
{
	readEnd(funcName);
	return readBegin(funcName);
}

char XFoam_IStream::readBeginList(const char* funcName)
{
	XFoam_Token delimiter(*this);
	if (delimiter != XFoam_Token::BEGIN_LIST && delimiter != XFoam_Token::BEGIN_BLOCK)
	{
		setBad();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Expected a '" << XFoam_Token::BEGIN_LIST << "' or a '" << XFoam_Token::BEGIN_BLOCK << "' while reading "
			<< funcName << ", found " << delimiter.info() << XFoam_exit(XFoam_FatalIOError, 1);
		return '\0';
	}
	return delimiter.pToken();
}

char XFoam_IStream::readEndList(const char* funcName)
{
	XFoam_Token delimiter(*this);
	if (delimiter != XFoam_Token::END_LIST && delimiter != XFoam_Token::END_BLOCK)
	{
		setBad();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Expected a '" << XFoam_Token::END_LIST << "' or a '" << XFoam_Token::END_BLOCK << "' while reading "
			<< funcName << ", found " << delimiter.info() << XFoam_exit(XFoam_FatalIOError, 1);
		return '\0';
	}
	return delimiter.pToken();
}

XFoam_IStream& XFoam_IStream::operator()() const
{
	if (!good())
	{
		check("Istream::operator()");
		XFoam_FatalError.exit();
	}
	return const_cast<XFoam_IStream&>(*this);
}

void XFoam_OStream::decrIndent()
{
	if (indentLevel_ == 0)
	{
		std::cerr << "Ostream::decrIndent() : attempt to decrement 0 indent level" << std::endl;
	}
	else
	{
		indentLevel_--;
	}
}

char XFoam_ISstream::nextValid()
{
	char c = 0;
	while (true)
	{
		while (get(c) && std::isspace(static_cast<unsigned char>(c)))
		{}
		if (bad() || std::isspace(static_cast<unsigned char>(c)))
		{
			break;
		}
		if (c == '/')
		{
			if (!get(c))
			{
				return '/';
			}
			if (c == '/')
			{
				while (get(c) && c != '\n')
				{}
			}
			else if (c == '*')
			{
				while (true)
				{
					if (get(c) && c == '*')
					{
						if (get(c))
						{
							if (c == '/')
							{
								break;
							}
							if (c == '*')
							{
								putback(c);
							}
						}
					}
					if (!good())
					{
						return 0;
					}
				}
			}
			else
			{
				putback(c);
				return '/';
			}
		}
		else
		{
			return c;
		}
	}
	return 0;
}

void XFoam_ISstream::readWordToken(XFoam_Token& t)
{
	XFoam_Word* wPtr = new XFoam_Word;

	if (read(*wPtr).bad())
	{
		delete wPtr;
		t.setBad();
	}
	else if (XFoam_Token::compound::isCompound(*wPtr))
	{
		t = XFoam_Token::compound::New(*wPtr, *this).ptr();
		delete wPtr;
	}
	else
	{
		t = wPtr;
	}
}

XFoam_IStream& XFoam_ISstream::read(XFoam_Token& t)
{
	if (XFoam_IStream::getBack(t))
	{
		return *this;
	}

	char c = nextValid();
	t.lineNumber() = lineNumber();

	if (!c)
	{
		t.setBad();
		return *this;
	}

	switch (c)
	{
	case XFoam_Token::END_STATEMENT:
	case XFoam_Token::BEGIN_LIST:
	case XFoam_Token::END_LIST:
	case XFoam_Token::BEGIN_SQR:
	case XFoam_Token::END_SQR:
	case XFoam_Token::BEGIN_BLOCK:
	case XFoam_Token::END_BLOCK:
	case XFoam_Token::COLON:
	case XFoam_Token::COMMA:
	case XFoam_Token::ASSIGN:
	case XFoam_Token::ADD:
	case XFoam_Token::MULTIPLY:
	case XFoam_Token::DIVIDE:
	{
		t = static_cast<XFoam_Token::punctuationToken>(c);
		return *this;
	}

	case XFoam_Token::BEGIN_STRING:
	{
		putback(c);
		XFoam_String* sPtr = new XFoam_String;
		if (read(*sPtr).bad())
		{
			delete sPtr;
			t.setBad();
		}
		else
		{
			t = sPtr;
		}
		return *this;
	}

	case XFoam_Token::HASH:
	{
		char nextC;
		if (read(nextC).bad())
		{
			t = XFoam_Token(XFoam_Word(XFoam_String(1, c)));
			return *this;
		}
		if (nextC == XFoam_Token::BEGIN_BLOCK)
		{
			XFoam_VerbatimString* vsPtr = new XFoam_VerbatimString;
			if (readVerbatim(*vsPtr).bad())
			{
				delete vsPtr;
				t.setBad();
			}
			else
			{
				t = vsPtr;
			}
			return *this;
		}
		putback(nextC);
		putback(c);
		XFoam_FunctionName* fnPtr = new XFoam_FunctionName;
		if (read(static_cast<XFoam_Word&>(*fnPtr)).bad())
		{
			delete fnPtr;
			t.setBad();
		}
		else
		{
			t = fnPtr;
		}
		return *this;
	}

	case '$':
	{
		char nextC;
		if (read(nextC).bad())
		{
			t = XFoam_Token(XFoam_Word(XFoam_String(1, c)));
			return *this;
		}
		putback(nextC);
		putback(c);
		XFoam_Variable* vPtr = new XFoam_Variable;
		if (readVariable(*vPtr).bad())
		{
			delete vPtr;
			t.setBad();
		}
		else
		{
			t = vPtr;
		}
		return *this;
	}

	case '-':
	case '.':
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
	{
		bool asLabel = (c != '.');
		buf_.clear();
		buf_.push_back(c);
		while (is_.get(c)
			   && (std::isdigit(static_cast<unsigned char>(c)) || c == '+' || c == '-' || c == '.' || c == 'E'
				   || c == 'e'))
		{
			if (asLabel)
			{
				asLabel = std::isdigit(static_cast<unsigned char>(c));
			}
			buf_.push_back(c);
		}
		buf_.push_back('\0');
		setState(is_.rdstate());
		if (is_.bad())
		{
			t.setBad();
		}
		else
		{
			is_.putback(c);
			if (buf_.size() == 2 && buf_[0] == '-')
			{
				t = XFoam_Token(XFoam_Token::SUBTRACT, lineNumber());
			}
			else if (asLabel)
			{
				XFoam_Label labelVal = 0;
				XFoam_Scalar scalarVal = 0;
				if (xf_parseLabel(xf_bufCStr(buf_), labelVal))
				{
					t = labelVal;
					t.lineNumber() = lineNumber();
				}
				else if (xf_parseScalar(xf_bufCStr(buf_), scalarVal))
				{
					t = XFoam_Token(scalarVal, lineNumber());
				}
				else
				{
					t.setBad();
				}
			}
			else
			{
				XFoam_Scalar scalarVal = 0;
				if (xf_parseScalar(xf_bufCStr(buf_), scalarVal))
				{
					t = XFoam_Token(scalarVal, lineNumber());
				}
				else
				{
					t.setBad();
				}
			}
		}
		return *this;
	}

	default:
		putback(c);
		readWordToken(t);
		return *this;
	}
}

XFoam_IStream& XFoam_ISstream::read(char& c)
{
	c = nextValid();
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(XFoam_Word& str)
{
	buf_.clear();
	int listDepth = 0;
	char c;
	while (get(c) && XFoam_Word::valid(c))
	{
		if (c == XFoam_Token::BEGIN_LIST)
		{
			listDepth++;
		}
		else if (c == XFoam_Token::END_LIST)
		{
			if (listDepth)
			{
				listDepth--;
			}
			else
			{
				break;
			}
		}
		buf_.push_back(c);
	}
	if (bad())
	{
		xf_bufMarkParseError(buf_, bufErrorLength);
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "problem while reading word '" << xf_bufCStr(buf_) << "...' after " << buf_.size() << " characters\n"
			<< XFoam_exit(XFoam_FatalIOError, 1);
		return *this;
	}
	if (buf_.empty())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "invalid first character found : " << c << XFoam_exit(XFoam_FatalIOError, 1);
	}
	buf_.push_back('\0');
	str = XFoam_Word(xf_bufCStr(buf_));
	putback(c);
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(XFoam_String& str)
{
	buf_.clear();
	char c;
	if (!get(c))
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "cannot read start of string" << XFoam_exit(XFoam_FatalIOError, 1);
		return *this;
	}
	if (c != XFoam_Token::BEGIN_STRING)
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "Incorrect start of string character found : " << c << XFoam_exit(XFoam_FatalIOError, 1);
		return *this;
	}
	bool escaped = false;
	while (get(c))
	{
		if (c == XFoam_Token::END_STRING)
		{
			if (escaped)
			{
				escaped = false;
				buf_.pop_back();
			}
			else
			{
				buf_.push_back('\0');
				str = xf_bufCStr(buf_);
				return *this;
			}
		}
		else if (c == XFoam_Token::NL)
		{
			if (escaped)
			{
				escaped = false;
				buf_.pop_back();
			}
			else
			{
				xf_bufMarkParseError(buf_, bufErrorLength);
				XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
					<< "found '\\n' while reading string \"" << xf_bufCStr(buf_) << "...\"" << XFoam_exit(XFoam_FatalIOError, 1);
				return *this;
			}
		}
		else if (c == '\\')
		{
			escaped = !escaped;
		}
		else
		{
			escaped = false;
		}
		buf_.push_back(c);
	}
	xf_bufMarkParseError(buf_, bufErrorLength);
	XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
		<< "problem while reading string \"" << xf_bufCStr(buf_) << "...\"" << XFoam_exit(XFoam_FatalIOError, 1);
	return *this;
}

XFoam_IStream& XFoam_ISstream::readVariable(XFoam_String& str)
{
	buf_.clear();
	char c;
	if (!get(c) || c != '$')
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "invalid first character found : " << c << XFoam_exit(XFoam_FatalIOError, 1);
	}
	buf_.push_back(c);
	if (peek() == static_cast<int>(XFoam_Token::BEGIN_BLOCK))
	{
		if (!get(c))
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
				<< "invalid first character found : " << c << XFoam_exit(XFoam_FatalIOError, 1);
		}
		buf_.push_back(c);
		int depth = 1;
		while (get(c))
		{
			buf_.push_back(c);
			if (c == XFoam_Token::BEGIN_BLOCK)
			{
				depth++;
			}
			else if (c == XFoam_Token::END_BLOCK)
			{
				depth--;
				if (depth <= 0)
				{
					break;
				}
			}
		}
		if (bad() || depth > 0)
		{
			xf_bufMarkParseError(buf_, bufErrorLength);
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
				<< "problem while reading variable '" << xf_bufCStr(buf_) << "...'" << XFoam_exit(XFoam_FatalIOError, 1);
		}
	}
	else
	{
		int listDepth = 0;
		while (get(c) && XFoam_Variable::valid(c))
		{
			if (c == XFoam_Token::BEGIN_LIST)
			{
				listDepth++;
			}
			else if (c == XFoam_Token::END_LIST)
			{
				if (listDepth)
				{
					listDepth--;
				}
				else
				{
					break;
				}
			}
			buf_.push_back(c);
		}
		if (bad())
		{
			xf_bufMarkParseError(buf_, bufErrorLength);
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
				<< "problem while reading string '" << xf_bufCStr(buf_) << "...' after " << buf_.size() << " characters\n"
				<< XFoam_exit(XFoam_FatalIOError, 1);
			return *this;
		}
		putback(c);
	}
	if (buf_.empty())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "invalid first character found : " << c << XFoam_exit(XFoam_FatalIOError, 1);
	}
	buf_.push_back('\0');
	str = xf_bufCStr(buf_);
	return *this;
}

XFoam_IStream& XFoam_ISstream::readVerbatim(XFoam_VerbatimString& str)
{
	buf_.clear();
	char c;
	while (get(c))
	{
		if (c == XFoam_Token::HASH)
		{
			char nextC;
			get(nextC);
			if (nextC == XFoam_Token::END_BLOCK)
			{
				buf_.push_back('\0');
				str = XFoam_VerbatimString(xf_bufCStr(buf_));
				return *this;
			}
			putback(nextC);
		}
		buf_.push_back(c);
	}
	xf_bufMarkParseError(buf_, bufErrorLength);
	XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
		<< "problem while reading string \"" << xf_bufCStr(buf_) << "...\"" << XFoam_exit(XFoam_FatalIOError, 1);
	return *this;
}

XFoam_ISstream& XFoam_ISstream::getLine(XFoam_String& s, const bool continuation)
{
	std::getline(is_, s);
	setState(is_.rdstate());
	lineNumber_++;
	if (continuation && s.size())
	{
		while (!s.empty() && s.back() == '\\')
		{
			XFoam_String contLine;
			std::getline(is_, contLine);
			setState(is_.rdstate());
			lineNumber_++;
			s.pop_back();
			s += contLine;
		}
	}
	return *this;
}

XFoam_IStream& XFoam_ISstream::readDelimited(XFoam_String& str, const char begin, const char end)
{
	str.clear();
	int listDepth = 0;
	char c;
	while (get(c))
	{
		str += c;
		if (c == begin)
		{
			listDepth++;
		}
		else if (c == end)
		{
			listDepth--;
			if (listDepth <= 0)
			{
				break;
			}
		}
	}
	if (bad() || listDepth != 0)
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "    problem while reading delimited string \n"
			<< str.c_str() << "\n    list depth = " << listDepth << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return *this;
}

XFoam_IStream& XFoam_ISstream::readList(XFoam_String& str)
{
	return readDelimited(str, XFoam_Token::BEGIN_LIST, XFoam_Token::END_LIST);
}

XFoam_IStream& XFoam_ISstream::readBlock(XFoam_String& str)
{
	return readDelimited(str, XFoam_Token::BEGIN_BLOCK, XFoam_Token::END_BLOCK);
}

XFoam_IStream& XFoam_ISstream::read(int32_t& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(int64_t& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(uint32_t& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(uint64_t& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(float& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(double& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(long double& val)
{
	is_ >> val;
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::read(char* buf, XFoam_StreamSize count)
{
	if (format() != BINARY)
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "stream format not binary" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	readBegin("binaryBlock");
	is_.read(buf, count);
	readEnd("binaryBlock");
	setState(is_.rdstate());
	return *this;
}

XFoam_IStream& XFoam_ISstream::rewind()
{
	stdStream().rdbuf()->pubseekpos(0);
	XFoam_Token t;
	XFoam_IStream::getBack(t);
	return *this;
}

std::ios_base::fmtflags XFoam_ISstream::flags() const
{
	return is_.flags();
}

std::ios_base::fmtflags XFoam_ISstream::flags(const std::ios_base::fmtflags f)
{
	return is_.flags(f);
}

void XFoam_ISstream::print(XFoam_OStream& os) const
{
	os << "ISstream " << name() << " : ";
	XFoam_IOstream::print(os);
}

XFoam_OSstream::XFoam_OSstream(
	std::ostream& os,
	const XFoam_String& name,
	const streamFormat format,
	const versionNumber version,
	const compressionType compression)
	: XFoam_OStream(format, version, compression)
	, name_(name)
	, os_(os)
{
	if (os_.good())
	{
		setOpened();
		setGood();
		os_.precision(precision_);
	}
	else
	{
		setState(os_.rdstate());
	}
}

XFoam_OStream& XFoam_OSstream::write(const char c)
{
	os_ << c;
	if (c == XFoam_Token::NL)
	{
		lineNumber_++;
	}
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const char* str)
{
	const XFoam_String s(str ? str : "");
	lineNumber_ += static_cast<XFoam_Label>(std::count(s.begin(), s.end(), XFoam_Token::NL));
	os_ << str;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const XFoam_Word& str)
{
	os_ << static_cast<const std::string&>(str);
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const XFoam_String& str)
{
	return writeQuoted(str);
}

XFoam_OStream& XFoam_OSstream::write(const XFoam_VerbatimString& vs)
{
	os_ << XFoam_Token::HASH << XFoam_Token::BEGIN_BLOCK;
	writeQuoted(static_cast<const XFoam_String&>(vs), false);
	os_ << XFoam_Token::HASH << XFoam_Token::END_BLOCK;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::writeQuoted(const std::string& str, const bool quoted)
{
	if (quoted)
	{
		os_ << XFoam_Token::BEGIN_STRING;
		int backslash = 0;
		for (std::string::const_iterator iter = str.begin(); iter != str.end(); ++iter)
		{
			char c = *iter;
			if (c == '\\')
			{
				backslash++;
				continue;
			}
			if (c == XFoam_Token::NL)
			{
				lineNumber_++;
				backslash++;
			}
			else if (c == XFoam_Token::END_STRING)
			{
				backslash++;
			}
			while (backslash)
			{
				os_ << '\\';
				backslash--;
			}
			os_ << c;
		}
		os_ << XFoam_Token::END_STRING;
	}
	else
	{
		lineNumber_ += static_cast<XFoam_Label>(std::count(str.begin(), str.end(), XFoam_Token::NL));
		os_ << str;
	}
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const int32_t val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const int64_t val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const uint32_t val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const uint64_t val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const float val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const double val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const long double val)
{
	os_ << val;
	setState(os_.rdstate());
	return *this;
}

XFoam_OStream& XFoam_OSstream::write(const char* buf, XFoam_StreamSize count)
{
	if (format() != BINARY)
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(name())))
			<< "stream format not binary" << XFoam_exit(XFoam_FatalIOError, 1);
	}
	os_ << XFoam_Token::BEGIN_LIST;
	os_.write(buf, count);
	os_ << XFoam_Token::END_LIST;
	setState(os_.rdstate());
	return *this;
}

void XFoam_OSstream::indent()
{
	for (unsigned short i = 0; i < indentLevel_ * indentSize_; i++)
	{
		os_ << ' ';
	}
}

void XFoam_OSstream::flush()
{
	os_.flush();
}

void XFoam_OSstream::endl()
{
	write('\n');
	os_.flush();
}

std::ios_base::fmtflags XFoam_OSstream::flags() const
{
	return os_.flags();
}

std::ios_base::fmtflags XFoam_OSstream::flags(const std::ios_base::fmtflags f)
{
	return os_.flags(f);
}

int XFoam_OSstream::width() const
{
	return os_.width();
}

int XFoam_OSstream::width(const int w)
{
	return os_.width(w);
}

int XFoam_OSstream::precision() const
{
	return os_.precision();
}

int XFoam_OSstream::precision(const int p)
{
	return os_.precision(p);
}

void XFoam_OSstream::print(XFoam_OStream& os) const
{
	os << "OSstream " << name() << " : ";
	XFoam_IOstream::print(os);
}

void XFoam_PrefixOSstream::checkWritePrefix()
{
	if (printPrefix_ && prefix_.size())
	{
		this->XFoam_OSstream::write(prefix_.c_str());
		printPrefix_ = false;
	}
}

XFoam_PrefixOSstream::XFoam_PrefixOSstream(
	std::ostream& os,
	const XFoam_String& name,
	const streamFormat format,
	const versionNumber version,
	const compressionType compression)
	: XFoam_OSstream(os, name, format, version, compression)
	, printPrefix_(true)
	, prefix_("")
{
}

void XFoam_PrefixOSstream::print(XFoam_OStream& os) const
{
	os << "prefixOSstream ";
	XFoam_OSstream::print(os);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const char c)
{
	checkWritePrefix();
	XFoam_OSstream::write(c);
	if (c == XFoam_Token::NL)
	{
		printPrefix_ = true;
	}
	return *this;
}

XFoam_OStream& XFoam_PrefixOSstream::write(const char* str)
{
	checkWritePrefix();
	XFoam_OSstream::write(str);
	const XFoam_Size len = str ? std::strlen(str) : 0;
	if (len && str[len - 1] == XFoam_Token::NL)
	{
		printPrefix_ = true;
	}
	return *this;
}

XFoam_OStream& XFoam_PrefixOSstream::write(const XFoam_Word& val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const XFoam_String& val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const XFoam_VerbatimString& vs)
{
	checkWritePrefix();
	return XFoam_OSstream::write(vs);
}

XFoam_OStream& XFoam_PrefixOSstream::writeQuoted(const std::string& val, const bool quoted)
{
	checkWritePrefix();
	return XFoam_OSstream::writeQuoted(val, quoted);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const int32_t val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const int64_t val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const uint32_t val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const uint64_t val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const float val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const double val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const long double val)
{
	checkWritePrefix();
	return XFoam_OSstream::write(val);
}

XFoam_OStream& XFoam_PrefixOSstream::write(const char* buf, XFoam_StreamSize count)
{
	checkWritePrefix();
	return XFoam_OSstream::write(buf, count);
}

void XFoam_PrefixOSstream::indent()
{
	checkWritePrefix();
	XFoam_OSstream::indent();
}

void XFoam_IStringStream::print(XFoam_OStream& os) const
{
	os << "IStringStream " << name() << " : "
	   << "buffer = \n"
	   << str();
	XFoam_endl(os);
	XFoam_ISstream::print(os);
}

void XFoam_OStringStream::print(XFoam_OStream& os) const
{
	os << "OStringStream " << name() << " : "
	   << "buffer = \n"
	   << str();
	XFoam_endl(os);
	XFoam_OSstream::print(os);
}

XFoam_ISstream XFoam_sin(std::cin, "sin");
XFoam_PrefixOSstream XFoam_sout(std::cout, "sout");
XFoam_OSstream XFoam_serr(std::cerr, "serr");
XFoam_PrefixOSstream XFoam_pout(std::cout, "pout");
XFoam_PrefixOSstream XFoam_perr(std::cerr, "perr");

const XFoam_VerbatimString XFoam_VerbatimString::null;

const char* const XFoam_Token::typeName = "token";

XFoam_Token XFoam_Token::undefinedToken;

const char* const XFoam_Token::compound::typeName = "compound";

XFoam_Token::compound::compound()
	: empty_(false)
{
}

XFoam_Token::compound::~compound() = default;

bool XFoam_Token::compound::isCompound(const XFoam_Word& name)
{
	(void)name;
	// XFoam：未实现 runTime 选择表；恒 false。
	return false;
}

XFoam_AutoPtr<XFoam_Token::compound> XFoam_Token::compound::New(const XFoam_Word& compoundType, XFoam_IStream& is)
{
	(void)compoundType;
	(void)is;
	XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String()))
		<< "XFoam_Token::compound::New: run-time compound table not implemented" << XFoam_exit(XFoam_FatalIOError, 1);
	return XFoam_AutoPtr<compound>(nullptr);
}

void XFoam_Token::clear()
{
	if (type_ == WORD)
	{
		delete wordTokenPtr_;
	}
	else if (type_ == FUNCTIONNAME)
	{
		delete functionNameTokenPtr_;
	}
	else if (type_ == VARIABLE)
	{
		delete variableTokenPtr_;
	}
	else if (type_ == STRING)
	{
		delete stringTokenPtr_;
	}
	else if (type_ == VERBATIMSTRING)
	{
		delete verbatimStringTokenPtr_;
	}
	else if (type_ == COMPOUND)
	{
		if (compoundTokenPtr_->unique())
		{
			delete compoundTokenPtr_;
		}
		else
		{
			compoundTokenPtr_->XFoam_RefCount::operator--();
		}
	}

	type_ = UNDEFINED;
}

void XFoam_Token::parseError(const char* expected) const
{
	std::cerr << "Parse error, expected a " << expected << ", found \n " << info() << std::endl;
}

XFoam_Token::XFoam_Token()
	: type_(UNDEFINED)
	, lineNumber_(0)
{
}

XFoam_Token::XFoam_Token(const XFoam_Token& t)
	: type_(t.type_)
	, lineNumber_(t.lineNumber_)
{
	switch (type_)
	{
	case UNDEFINED:
		break;

	case PUNCTUATION:
		punctuationToken_ = t.punctuationToken_;
		break;

	case WORD:
		wordTokenPtr_ = new XFoam_Word(*t.wordTokenPtr_);
		break;

	case FUNCTIONNAME:
		functionNameTokenPtr_ = new XFoam_FunctionName(*t.functionNameTokenPtr_);
		break;

	case VARIABLE:
		variableTokenPtr_ = new XFoam_Variable(*t.variableTokenPtr_);
		break;

	case STRING:
		stringTokenPtr_ = new XFoam_String(*t.stringTokenPtr_);
		break;

	case VERBATIMSTRING:
		verbatimStringTokenPtr_ = new XFoam_VerbatimString(*t.verbatimStringTokenPtr_);
		break;

	case LABEL:
		labelToken_ = t.labelToken_;
		break;

	case SCALAR:
		scalarToken_ = t.scalarToken_;
		break;

	case COMPOUND:
		compoundTokenPtr_ = t.compoundTokenPtr_;
		compoundTokenPtr_->XFoam_RefCount::operator++();
		break;

	case ERROR:
		break;
	}
}

XFoam_Token::XFoam_Token(punctuationToken p, XFoam_Label lineNumber)
	: type_(PUNCTUATION)
	, punctuationToken_(p)
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(const XFoam_Word& w, XFoam_Label lineNumber)
	: type_(WORD)
	, wordTokenPtr_(new XFoam_Word(w))
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(const XFoam_String& s, XFoam_Label lineNumber)
	: type_(STRING)
	, stringTokenPtr_(new XFoam_String(s))
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(const XFoam_VerbatimString& vs, XFoam_Label lineNumber)
	: type_(VERBATIMSTRING)
	, verbatimStringTokenPtr_(new XFoam_VerbatimString(vs))
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(const XFoam_Label l, XFoam_Label lineNumber)
	: type_(LABEL)
	, labelToken_(l)
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(XFoam_Scalar s, XFoam_Label lineNumber)
	: type_(SCALAR)
	, scalarToken_(s)
	, lineNumber_(lineNumber)
{
}

XFoam_Token::XFoam_Token(XFoam_IStream& is)
	: type_(UNDEFINED)
	, lineNumber_(0)
{
	is.read(*this);
}

XFoam_Token::~XFoam_Token()
{
	clear();
}

XFoam_Token::tokenType XFoam_Token::type() const
{
	return type_;
}

XFoam_Token::tokenType& XFoam_Token::type()
{
	return type_;
}

bool XFoam_Token::good() const
{
	return (type_ != ERROR && type_ != UNDEFINED);
}

bool XFoam_Token::undefined() const
{
	return (type_ == UNDEFINED);
}

bool XFoam_Token::error() const
{
	return (type_ == ERROR);
}

bool XFoam_Token::isPunctuation() const
{
	return (type_ == PUNCTUATION);
}

XFoam_Token::punctuationToken XFoam_Token::pToken() const
{
	if (type_ == PUNCTUATION)
	{
		return punctuationToken_;
	}
	parseError("punctuation character");
	return NULL_TOKEN;
}

bool XFoam_Token::isWord() const
{
	return (type_ == WORD);
}

const XFoam_Word& XFoam_Token::wordToken() const
{
	if (type_ == WORD)
	{
		return *wordTokenPtr_;
	}
	parseError("word");
	return XFoam_Word::null;
}

bool XFoam_Token::isFunctionName() const
{
	return (type_ == FUNCTIONNAME);
}

const XFoam_FunctionName& XFoam_Token::functionNameToken() const
{
	if (type_ == FUNCTIONNAME)
	{
		return *functionNameTokenPtr_;
	}
	parseError("functionName");
	return XFoam_FunctionName::null;
}

bool XFoam_Token::isVariable() const
{
	return (type_ == VARIABLE);
}

const XFoam_Variable& XFoam_Token::variableToken() const
{
	if (type_ == VARIABLE)
	{
		return *variableTokenPtr_;
	}
	parseError("variable");
	return XFoam_Variable::null;
}

bool XFoam_Token::isString() const
{
	return (type_ == STRING);
}

const XFoam_String& XFoam_Token::stringToken() const
{
	if (type_ == STRING)
	{
		return *stringTokenPtr_;
	}
	parseError("string");
	static const XFoam_String nullStr;
	return nullStr;
}

bool XFoam_Token::isVerbatimString() const
{
	return (type_ == VERBATIMSTRING);
}

const XFoam_VerbatimString& XFoam_Token::verbatimStringToken() const
{
	if (type_ == VERBATIMSTRING)
	{
		return *verbatimStringTokenPtr_;
	}
	parseError("verbatimString");
	return XFoam_VerbatimString::null;
}

bool XFoam_Token::isAnyString() const
{
	return (
		type_ == WORD || type_ == FUNCTIONNAME || type_ == VARIABLE || type_ == STRING || type_ == VERBATIMSTRING);
}

const XFoam_String& XFoam_Token::anyStringToken() const
{
	if (type_ == WORD)
	{
		return *wordTokenPtr_;
	}
	if (type_ == FUNCTIONNAME)
	{
		return *functionNameTokenPtr_;
	}
	if (type_ == VARIABLE)
	{
		return *variableTokenPtr_;
	}
	if (type_ == STRING)
	{
		return *stringTokenPtr_;
	}
	if (type_ == VERBATIMSTRING)
	{
		return *verbatimStringTokenPtr_;
	}
	parseError("string");
	static const XFoam_String nullStr;
	return nullStr;
}

bool XFoam_Token::isLabel() const
{
	return (type_ == LABEL);
}

XFoam_Label XFoam_Token::labelToken() const
{
	if (type_ == LABEL)
	{
		return labelToken_;
	}
	parseError("label");
	return 0;
}

bool XFoam_Token::isScalar() const
{
	return (type_ == SCALAR);
}

XFoam_Scalar XFoam_Token::scalarToken() const
{
	if (type_ == SCALAR)
	{
		return scalarToken_;
	}
	parseError("scalar");
	return 0.0;
}

bool XFoam_Token::isNumber() const
{
	return (type_ == LABEL || isScalar());
}

XFoam_Scalar XFoam_Token::number() const
{
	if (type_ == LABEL)
	{
		return static_cast<XFoam_Scalar>(labelToken_);
	}
	if (isScalar())
	{
		return scalarToken();
	}
	parseError("number (label or scalar)");
	return 0.0;
}

bool XFoam_Token::isCompound() const
{
	return (type_ == COMPOUND);
}

const XFoam_Token::compound& XFoam_Token::compoundToken() const
{
	if (type_ == COMPOUND)
	{
		return *compoundTokenPtr_;
	}
	parseError("compound");
	return xf_nullCompound;
}

XFoam_Token::compound& XFoam_Token::transferCompoundToken(const XFoam_IStream& is)
{
	if (type_ == COMPOUND)
	{
		if (compoundTokenPtr_->empty())
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String()))
				<< "compound has already been transferred from token\n " << info() << XFoam_exit(XFoam_FatalIOError, 1);
		}
		else
		{
			compoundTokenPtr_->empty() = true;
		}

		return *compoundTokenPtr_;
	}
	parseError("compound");
	(void)is;
	return const_cast<compound&>(static_cast<const compound&>(xf_nullCompound));
}

XFoam_Label XFoam_Token::lineNumber() const
{
	return lineNumber_;
}

XFoam_Label& XFoam_Token::lineNumber()
{
	return lineNumber_;
}

void XFoam_Token::setBad()
{
	clear();
	type_ = ERROR;
}

void XFoam_Token::operator=(const XFoam_Token& t)
{
	clear();
	type_ = t.type_;

	switch (type_)
	{
	case UNDEFINED:
		break;

	case PUNCTUATION:
		punctuationToken_ = t.punctuationToken_;
		break;

	case WORD:
		wordTokenPtr_ = new XFoam_Word(*t.wordTokenPtr_);
		break;

	case FUNCTIONNAME:
		functionNameTokenPtr_ = new XFoam_FunctionName(*t.functionNameTokenPtr_);
		break;

	case VARIABLE:
		variableTokenPtr_ = new XFoam_Variable(*t.variableTokenPtr_);
		break;

	case STRING:
		stringTokenPtr_ = new XFoam_String(*t.stringTokenPtr_);
		break;

	case VERBATIMSTRING:
		verbatimStringTokenPtr_ = new XFoam_VerbatimString(*t.verbatimStringTokenPtr_);
		break;

	case LABEL:
		labelToken_ = t.labelToken_;
		break;

	case SCALAR:
		scalarToken_ = t.scalarToken_;
		break;

	case COMPOUND:
		compoundTokenPtr_ = t.compoundTokenPtr_;
		compoundTokenPtr_->XFoam_RefCount::operator++();
		break;

	case ERROR:
		break;
	}

	lineNumber_ = t.lineNumber_;
}

void XFoam_Token::operator=(punctuationToken p)
{
	clear();
	type_ = PUNCTUATION;
	punctuationToken_ = p;
}

void XFoam_Token::operator=(XFoam_Word* wPtr)
{
	clear();
	type_ = WORD;
	wordTokenPtr_ = wPtr;
}

void XFoam_Token::operator=(const XFoam_Word& w)
{
	operator=(new XFoam_Word(w));
}

void XFoam_Token::operator=(XFoam_FunctionName* fnPtr)
{
	clear();
	type_ = FUNCTIONNAME;
	functionNameTokenPtr_ = fnPtr;
}

void XFoam_Token::operator=(const XFoam_FunctionName& fn)
{
	operator=(new XFoam_FunctionName(fn));
}

void XFoam_Token::operator=(XFoam_Variable* vPtr)
{
	clear();
	type_ = VARIABLE;
	variableTokenPtr_ = vPtr;
}

void XFoam_Token::operator=(const XFoam_Variable& v)
{
	operator=(new XFoam_Variable(v));
}

void XFoam_Token::operator=(XFoam_String* sPtr)
{
	clear();
	type_ = STRING;
	stringTokenPtr_ = sPtr;
}

void XFoam_Token::operator=(const XFoam_String& s)
{
	operator=(new XFoam_String(s));
}

void XFoam_Token::operator=(XFoam_VerbatimString* vsPtr)
{
	clear();
	type_ = VERBATIMSTRING;
	verbatimStringTokenPtr_ = vsPtr;
}

void XFoam_Token::operator=(const XFoam_VerbatimString& vs)
{
	operator=(new XFoam_VerbatimString(vs));
}

void XFoam_Token::operator=(const XFoam_Label l)
{
	clear();
	type_ = LABEL;
	labelToken_ = l;
}

void XFoam_Token::operator=(XFoam_Scalar s)
{
	clear();
	type_ = SCALAR;
	scalarToken_ = s;
}

void XFoam_Token::operator=(compound* cPtr)
{
	clear();
	type_ = COMPOUND;
	compoundTokenPtr_ = cPtr;
}

bool XFoam_Token::operator==(const XFoam_Token& t) const
{
	if (type_ != t.type_)
	{
		return false;
	}

	switch (type_)
	{
	case UNDEFINED:
		return true;

	case PUNCTUATION:
		return punctuationToken_ == t.punctuationToken_;

	case WORD:
		return *wordTokenPtr_ == *t.wordTokenPtr_;

	case FUNCTIONNAME:
		return *functionNameTokenPtr_ == *t.functionNameTokenPtr_;

	case VARIABLE:
		return *variableTokenPtr_ == *t.variableTokenPtr_;

	case STRING:
		return *stringTokenPtr_ == *t.stringTokenPtr_;

	case VERBATIMSTRING:
		return *verbatimStringTokenPtr_ == *t.verbatimStringTokenPtr_;

	case LABEL:
		return labelToken_ == t.labelToken_;

	case SCALAR:
		return xf_scalar_equal(scalarToken_, t.scalarToken_);

	case COMPOUND:
		return compoundTokenPtr_ == t.compoundTokenPtr_;

	case ERROR:
		return true;
	}

	return false;
}

bool XFoam_Token::operator==(punctuationToken p) const
{
	return (type_ == PUNCTUATION && punctuationToken_ == p);
}

bool XFoam_Token::operator==(const XFoam_Word& w) const
{
	return (type_ == WORD && wordToken() == w);
}

bool XFoam_Token::operator==(const XFoam_FunctionName& fn) const
{
	return (type_ == FUNCTIONNAME && functionNameToken() == fn);
}

bool XFoam_Token::operator==(const XFoam_Variable& v) const
{
	return (type_ == VARIABLE && variableToken() == v);
}

bool XFoam_Token::operator==(const XFoam_String& s) const
{
	return (type_ == STRING && stringToken() == s);
}

bool XFoam_Token::operator==(const XFoam_VerbatimString& vs) const
{
	return (type_ == VERBATIMSTRING && verbatimStringToken() == vs);
}

bool XFoam_Token::operator==(const XFoam_Label l) const
{
	return (type_ == LABEL && labelToken_ == l);
}

bool XFoam_Token::operator==(XFoam_Scalar s) const
{
	return isScalar() && xf_scalar_equal(scalarToken_, s);
}

bool XFoam_Token::operator!=(const XFoam_Token& t) const
{
	return !operator==(t);
}

bool XFoam_Token::operator!=(punctuationToken p) const
{
	return !operator==(p);
}

bool XFoam_Token::operator!=(const XFoam_Word& w) const
{
	return !operator==(w);
}

bool XFoam_Token::operator!=(const XFoam_FunctionName& fn) const
{
	return !operator==(fn);
}

bool XFoam_Token::operator!=(const XFoam_Variable& v) const
{
	return !operator==(v);
}

bool XFoam_Token::operator!=(const XFoam_String& s) const
{
	return !operator==(s);
}

bool XFoam_Token::operator!=(const XFoam_VerbatimString& vs) const
{
	return !operator==(vs);
}

bool XFoam_Token::operator!=(const XFoam_Label l) const
{
	return !operator==(l);
}

bool XFoam_Token::operator!=(XFoam_Scalar s) const
{
	return !operator==(s);
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Token& t)
{
	t.clear();
	return is.read(t);
}

XFoam_OStream& operator<<(XFoam_OStream& os, XFoam_Token::punctuationToken pt)
{
	return os << static_cast<char>(pt);
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Token& t)
{
	switch (t.type())
	{
	case XFoam_Token::UNDEFINED:
		os << "UNDEFINED";
		break;

	case XFoam_Token::PUNCTUATION:
		os << static_cast<char>(t.pToken());
		break;

	case XFoam_Token::WORD:
		os << t.wordToken();
		break;

	case XFoam_Token::FUNCTIONNAME:
		os << t.functionNameToken();
		break;

	case XFoam_Token::VARIABLE:
		os << t.variableToken();
		break;

	case XFoam_Token::STRING:
		os << t.stringToken();
		break;

	case XFoam_Token::VERBATIMSTRING:
		os << t.verbatimStringToken();
		break;

	case XFoam_Token::LABEL:
		os << t.labelToken();
		break;

	case XFoam_Token::SCALAR:
		os << t.scalarToken();
		break;

	case XFoam_Token::COMPOUND:
		os << t.compoundToken();
		break;

	case XFoam_Token::ERROR:
		os << "ERROR";
		break;

	default:
		os << "UNKNOWN";
		break;
	}
	return os;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Token::compound& ct)
{
	os << ct.type() << static_cast<char>(XFoam_Token::SPACE);
	ct.write(os);
	return os;
}

std::ostream& operator<<(std::ostream& os, const XFoam_Token& t)
{
	return os << XFoam_InfoProxy<XFoam_Token>(t);
}

std::ostream& operator<<(std::ostream& os, const XFoam_InfoProxy<XFoam_Token>& ip)
{
	const XFoam_Token& t = ip.t_;

	os << "on line " << t.lineNumber();

	switch (t.type())
	{
	case XFoam_Token::UNDEFINED:
		os << " an undefined token";
		break;

	case XFoam_Token::PUNCTUATION:
		os << " the punctuation token " << '\'' << static_cast<char>(t.pToken()) << '\'';
		break;

	case XFoam_Token::WORD:
		os << " the word " << '\'' << t.wordToken() << '\'';
		break;

	case XFoam_Token::STRING:
		os << " the string " << t.stringToken();
		break;

	case XFoam_Token::VERBATIMSTRING:
		os << " the verbatim string " << t.verbatimStringToken();
		break;

	case XFoam_Token::FUNCTIONNAME:
		os << " the functionName " << t.functionNameToken();
		break;

	case XFoam_Token::VARIABLE:
		os << " the variable " << t.variableToken();
		break;

	case XFoam_Token::LABEL:
		os << " the label " << t.labelToken();
		break;

	case XFoam_Token::SCALAR:
		os << " the scalar " << t.scalarToken();
		break;

	case XFoam_Token::COMPOUND:
		if (t.compoundToken().empty())
		{
			os << " the empty compound of type " << t.compoundToken().type();
		}
		else
		{
			os << " the compound of type " << t.compoundToken().type();
		}
		break;

	case XFoam_Token::ERROR:
		os << " an error";
		break;

	default:
		os << " an unknown token type " << '\'' << int(t.type()) << '\'';
	}

	return os;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_Token>& ip)
{
	std::ostringstream oss;
	oss << ip;
	return os << oss.str();
}

bool XFoam_readTokenFromStream(XFoam_IStream& is, XFoam_Label& lineNo, XFoam_Token& tok)
{
	lineNo = is.lineNumber();
	is.read(tok);
	lineNo = tok.lineNumber();
	return tok.good();
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_MessageStream（对标 OpenFOAM db/error/messageStream.C 行为子集，无并行分支）

int XFoam_MessageStream::level = 2;

XFoam_MessageStream::XFoam_MessageStream(
	const XFoam_String& title,
	const ErrorSeverity severity,
	const int maxErrors)
	: title_(title)
	, severity_(severity)
	, maxErrors_(maxErrors)
	, errorCount_(0)
{
}

XFoam_OSstream& XFoam_MessageStream::operator()(const XFoam_Label /*communicator*/)
{
	if (!level)
	{
		static std::ostringstream nullOss;
		static XFoam_OSstream snull(nullOss, "Snull");
		return snull;
	}

	XFoam_OSstream* osptr = (severity_ == ErrorSeverity::Serious || severity_ == ErrorSeverity::Fatal)
		? &XFoam_serr
		: static_cast<XFoam_OSstream*>(&XFoam_sout);

	if (!title_.empty())
	{
		*osptr << title_.c_str();
	}

	if (maxErrors_)
	{
		++errorCount_;
		if (errorCount_ >= maxErrors_)
		{
			XFoam_FatalErrorInFunction << "Too many errors" << XFoam_abort(XFoam_FatalError);
		}
	}

	return *osptr;
}

XFoam_OSstream& XFoam_MessageStream::streamLeadIn_(
	const char* functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber)
{
	XFoam_OSstream& os = operator()();
	os << XFoam_endl;
	if (functionName)
	{
		os << " From function " << functionName << XFoam_endl;
	}
	if (sourceFileName)
	{
		os << " in file " << sourceFileName << " at line " << sourceFileLineNumber << XFoam_endl << " ";
	}
	return os;
}

XFoam_OSstream& XFoam_MessageStream::operator()(
	const char* functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber)
{
	return streamLeadIn_(functionName, sourceFileName, sourceFileLineNumber);
}

XFoam_OSstream& XFoam_MessageStream::operator()(
	const XFoam_String& functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber)
{
	return operator()(functionName.c_str(), sourceFileName, sourceFileLineNumber);
}

XFoam_OSstream& XFoam_MessageStream::operator()(
	const char* functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber,
	const XFoam_String& ioFileName,
	const XFoam_Label ioLineNumber)
{
	XFoam_OSstream& os = streamLeadIn_(functionName, sourceFileName, sourceFileLineNumber);
	os << "Reading \"" << ioFileName << '"';
	if (ioLineNumber >= 0)
	{
		os << " at line " << ioLineNumber;
	}
	os << XFoam_endl << " ";
	return os;
}

XFoam_OSstream& XFoam_MessageStream::operator()(
	const char* functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber,
	const XFoam_IOstream& ioStream)
{
	return operator()(
		functionName,
		sourceFileName,
		sourceFileLineNumber,
		static_cast<const XFoam_String&>(ioStream.name()),
		ioStream.lineNumber());
}

XFoam_OSstream& XFoam_MessageStream::operator()(
	const char* functionName,
	const char* sourceFileName,
	const int sourceFileLineNumber,
	const XFoam_Dictionary& dict)
{
	XFoam_OSstream& os = streamLeadIn_(functionName, sourceFileName, sourceFileLineNumber);
	const XFoam_String& ioName = static_cast<const XFoam_String&>(dict.name());
	os << "Reading \"" << ioName << '"';
	const XFoam_Label s = dict.startLineNumber();
	const XFoam_Label e = dict.endLineNumber();
	if (s >= 0 && e >= 0 && s < e)
	{
		os << " from line " << s << " to line " << e;
	}
	else if (s >= 0)
	{
		os << " at line " << s;
	}
	os << XFoam_endl << " ";
	return os;
}

XFoam_MessageStream XFoam_seriousError(
	XFoam_String("--> FOAM Serious Error : "),
	XFoam_MessageStream::ErrorSeverity::Serious,
	100);
XFoam_MessageStream XFoam_info(XFoam_String(), XFoam_MessageStream::ErrorSeverity::Info, 0);
bool XFoam_writeInfoHeader = true;
