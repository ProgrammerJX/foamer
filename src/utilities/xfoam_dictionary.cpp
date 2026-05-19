#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_time.h"
#include "XFoam/utilities/xfoam_error.h"
#include <sstream>
#include <utility>
#include <vector>
#include <cctype>
#include <iostream>
#include <algorithm>

namespace
{
inline XFoam_IOerrorLocation xf_ioLocDict(const XFoam_Dictionary& d)
{
	return XFoam_IOerrorLocation(static_cast<const XFoam_String&>(static_cast<const XFoam_FileName&>(d.name())));
}

static XFoam_String xf_getEnvStub(const XFoam_String&)
{
	// XFoam ????Foam::getEnv/OSspecific??????
	return XFoam_String();
}

static std::string xf_patternRegexText(const XFoam_KeyType& kw)
{
	const std::string& raw = static_cast<const std::string&>(kw);
	if (kw.isPattern() && raw.size() > 1)
	{
		return raw.substr(1);
	}
	return raw;
}

static void xf_writeQuoted(XFoam_OStream& os, const std::string& str, const bool quoted)
{
	if (!quoted)
	{
		os << str;
		return;
	}
	os << '"';
	for (char c : str)
	{
		if (c == '"')
		{
			os << '\\' << '"';
		}
		else if (c == '\\')
		{
			os << '\\' << '\\';
		}
		else
		{
			os << c;
		}
	}
	os << '"';
}

} // namespace

const XFoam_KeyType XFoam_KeyType::null;

// * * * * * * * * * * * * * * XFoam_KeyType * * * * * * * * * * * * * * //

XFoam_KeyType::XFoam_KeyType()
	: XFoam_Variable()
	, type_(UNDEFINED)
{}

XFoam_KeyType::XFoam_KeyType(const XFoam_KeyType& k)
	: XFoam_Variable(k)
	, type_(k.type_)
{}

XFoam_KeyType::XFoam_KeyType(const XFoam_Word& w)
	: XFoam_Variable(w)
	, type_(WORD)
{}

XFoam_KeyType::XFoam_KeyType(const XFoam_FunctionName& fn)
	: XFoam_Variable(static_cast<const XFoam_Word&>(fn))
	, type_(FUNCTIONNAME)
{}

XFoam_KeyType::XFoam_KeyType(const XFoam_Variable& v)
	: XFoam_Variable(v)
	, type_(VARIABLE)
{}

XFoam_KeyType::XFoam_KeyType(const std::string& s)
	: XFoam_Variable(s, false)
	, type_(PATTERN)
{}

XFoam_KeyType::XFoam_KeyType(const char* s)
	: XFoam_Variable(s, true)
	, type_(WORD)
{}

XFoam_KeyType::XFoam_KeyType(const XFoam_Token& t)
	: XFoam_Variable()
	, type_(UNDEFINED)
{
	operator=(t);
}

XFoam_KeyType::XFoam_KeyType(XFoam_IStream& is)
	: XFoam_Variable()
	, type_(UNDEFINED)
{
	is >> *this;
}

bool XFoam_KeyType::match(const std::string& str, bool literalMatch) const
{
	if (literalMatch || !isPattern())
	{
		return str == static_cast<const std::string&>(*this);
	}
	return XFoam_RegExp(static_cast<const std::string&>(*this)).match(str);
}

void XFoam_KeyType::operator=(const XFoam_KeyType& k)
{
	XFoam_Variable::operator=(k);
	type_ = k.type_;
}

void XFoam_KeyType::operator=(const XFoam_FunctionName& fn)
{
	XFoam_Word::operator=(fn);
	type_ = FUNCTIONNAME;
}

void XFoam_KeyType::operator=(const XFoam_Variable& v)
{
	XFoam_Variable::operator=(v);
	type_ = VARIABLE;
}

void XFoam_KeyType::operator=(const XFoam_Word& w)
{
	XFoam_Variable::operator=(w);
	type_ = WORD;
}

void XFoam_KeyType::operator=(const std::string& s)
{
	static_cast<std::string&>(*this) = s;
	type_ = PATTERN;
}

void XFoam_KeyType::operator=(const char* s)
{
	XFoam_Variable::operator=(XFoam_Word(s ? s : ""));
	type_ = WORD;
}

void XFoam_KeyType::operator=(const XFoam_Token& t)
{
	if (t.isWord())
	{
		operator=(t.wordToken());
	}
	else if (t.isFunctionName())
	{
		operator=(t.functionNameToken());
	}
	else if (t.isVariable())
	{
		operator=(t.variableToken());
	}
	else if (t.isString())
	{
		operator=(t.stringToken());
		if (empty())
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String())) << "Empty pattern string"
																				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
	}
	else if (t.isLabel())
	{
		operator=(XFoam_Word(std::to_string(t.labelToken())));
	}
	else
	{
		XFoam_Variable::clear();
		type_ = UNDEFINED;
	}
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_KeyType& kw)
{
	XFoam_Label lineNo = 0;
	XFoam_Token t;
	if (!XFoam_readTokenFromStream(is, lineNo, t) || !t.good())
	{
		is.setFail();
		return is;
	}
	kw = t;
	if (kw.isUndefined())
	{
		is.setFail();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String()))
			<< "wrong token type - expected word or string, found (bad token)" << XFoam_exit(XFoam_FatalIOError, 1);
		return is;
	}
	return is;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_KeyType& kw)
{
	xf_writeQuoted(os, static_cast<const std::string&>(kw), kw.isPattern());
	return os;
}

XFoam_OStream& XFoam_writeKeyword(XFoam_OStream& os, const XFoam_KeyType& kw)
{
	static const unsigned short entryIndentation_ = 16;
	os << kw;
	XFoam_Label nSpaces = static_cast<XFoam_Label>(entryIndentation_) - static_cast<XFoam_Label>(kw.size());
	if (kw.isPattern())
	{
		nSpaces -= 2;
	}
	nSpaces = (std::max)(nSpaces, XFoam_Label(1));
	while (nSpaces--)
	{
		os.write(' ');
	}
	return os;
}

XFoam_OStream& XFoam_OStream::writeKeyword(const XFoam_KeyType& kw)
{
	return XFoam_writeKeyword(*this, kw);
}

// * * * * * * * * * * XFoam_ITstream * * * * * * * * * * * * * * * * * * //
// ???? OpenFOAM-13 ITstream.C / ITstream.H?read(char&) ? NotImplemented ???XFoam ????????????

XFoam_ITstream::XFoam_ITstream()
	: XFoam_IStream(XFoam_IOstream::ASCII, XFoam_IOstream::currentVersion, XFoam_IOstream::UNCOMPRESSED, false)
	, XFoam_TokenList()
	, name_()
	, tokenIndex_(0)
{
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(const XFoam_ITstream& its)
	: XFoam_IStream(XFoam_IOstream::ASCII, XFoam_IOstream::currentVersion, XFoam_IOstream::UNCOMPRESSED, false)
	, XFoam_TokenList(static_cast<const XFoam_TokenList&>(its))
	, name_(its.name_)
	, tokenIndex_(0)
{
	setOpened();
	setGood();
}

void XFoam_ITstream::operator=(const XFoam_ITstream& its)
{
	static_cast<XFoam_IStream&>(*this) = static_cast<const XFoam_IStream&>(its);
	static_cast<XFoam_TokenList&>(*this) = static_cast<const XFoam_TokenList&>(its);
	name_ = its.name_;
	tokenIndex_ = 0;
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(
	const XFoam_FileName& streamName,
	const XFoam_UList<XFoam_Token>& tokens,
	XFoam_IOstream::streamFormat format,
	const XFoam_IOstream::versionNumber& version,
	bool global)
	: XFoam_IStream(format, version, XFoam_IOstream::UNCOMPRESSED, global)
	, XFoam_TokenList(tokens.size())
	, name_(streamName)
	, tokenIndex_(0)
{
	for (XFoam_Label i = 0; i < tokens.size(); ++i)
	{
		(*this)[i] = tokens[i];
	}
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(
	const XFoam_FileName& streamName,
	XFoam_TokenList&& tokens,
	XFoam_IOstream::streamFormat format,
	const XFoam_IOstream::versionNumber& version,
	bool global)
	: XFoam_IStream(format, version, XFoam_IOstream::UNCOMPRESSED, global)
	, XFoam_TokenList(std::move(tokens))
	, name_(streamName)
	, tokenIndex_(0)
{
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(const XFoam_KeyType& keywordForName, const XFoam_UList<XFoam_Token>& tokens)
	: XFoam_IStream(XFoam_IOstream::ASCII, XFoam_IOstream::currentVersion, XFoam_IOstream::UNCOMPRESSED, false)
	, XFoam_TokenList(tokens.size())
	, name_(static_cast<const XFoam_String&>(keywordForName))
	, tokenIndex_(0)
{
	for (XFoam_Label i = 0; i < tokens.size(); ++i)
	{
		(*this)[i] = tokens[i];
	}
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(const XFoam_KeyType& keywordForName, XFoam_TokenList&& tokens)
	: XFoam_IStream(XFoam_IOstream::ASCII, XFoam_IOstream::currentVersion, XFoam_IOstream::UNCOMPRESSED, false)
	, XFoam_TokenList(std::move(tokens))
	, name_(static_cast<const XFoam_String&>(keywordForName))
	, tokenIndex_(0)
{
	setOpened();
	setGood();
}

XFoam_ITstream::XFoam_ITstream(const XFoam_KeyType& keywordForName, XFoam_Label reservedTokens)
	: XFoam_IStream(XFoam_IOstream::ASCII, XFoam_IOstream::currentVersion, XFoam_IOstream::UNCOMPRESSED, false)
	, XFoam_TokenList(reservedTokens)
	, name_(static_cast<const XFoam_String&>(keywordForName))
	, tokenIndex_(0)
{
	setOpened();
	setGood();
}

XFoam_ITstream::~XFoam_ITstream() = default;

void XFoam_ITstream::print(XFoam_OStream& os) const
{
	os << "ITstream : " << static_cast<const std::string&>(name()).c_str();
	if (size())
	{
		if (begin()->lineNumber() == rbegin()->lineNumber())
		{
			os << ", line " << begin()->lineNumber() << ", ";
		}
		else
		{
			os << ", lines " << begin()->lineNumber() << '-' << rbegin()->lineNumber() << ", ";
		}
	}
	else
	{
		os << ", line " << lineNumber() << ", ";
	}
	XFoam_IOstream::print(os);
}

std::ios_base::fmtflags XFoam_ITstream::flags() const
{
	return std::ios_base::fmtflags(0);
}

std::ios_base::fmtflags XFoam_ITstream::flags(const std::ios_base::fmtflags)
{
	return std::ios_base::fmtflags(0);
}

XFoam_IStream& XFoam_ITstream::read(XFoam_Token& t)
{
	if (XFoam_IStream::getBack(t))
	{
		lineNumber_ = t.lineNumber();
		setGood();
		return *this;
	}

	if (tokenIndex_ < size())
	{
		t = operator[](tokenIndex_++);
		lineNumber_ = t.lineNumber();
		// 读完最后一个 list token 时不 setEof：否则 good()==false 会令 XFoam_PtrList::read 的 while(is.good())
		// 在尚未读到 END_LIST 时提前退出，主字典会缺项（如 vertices）。
	}
	else
	{
		// 与 std::istream 一致：尾后多读返回非 good，供 Dictionary::read 正常结束，而非 Fatal。
		if (!eof())
		{
			setEof();
		}
		else
		{
			setFail();
		}

		t = XFoam_Token::undefinedToken;

		if (size())
		{
			t.lineNumber() = XFoam_TokenList::last().lineNumber();
		}
		else
		{
			t.lineNumber() = lineNumber();
		}
	}

	return *this;
}

XFoam_IStream& XFoam_ITstream::read(char& c)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isPunctuation())
	{
		c = static_cast<char>(t.pToken());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	c = '\0';
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(XFoam_Word& w)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isWord())
	{
		w = t.wordToken();
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(XFoam_String& s)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isString())
	{
		s = t.stringToken();
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(int32_t& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isLabel())
	{
		val = static_cast<int32_t>(t.labelToken());
		setGood();
		return *this;
	}
	if (t.good() && t.isScalar())
	{
		val = static_cast<int32_t>(t.scalarToken());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(int64_t& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isLabel())
	{
		val = static_cast<int64_t>(t.labelToken());
		setGood();
		return *this;
	}
	if (t.good() && t.isScalar())
	{
		val = static_cast<int64_t>(t.scalarToken());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(uint32_t& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isLabel())
	{
		val = static_cast<uint32_t>(t.labelToken());
		setGood();
		return *this;
	}
	if (t.good() && t.isScalar())
	{
		val = static_cast<uint32_t>(t.scalarToken());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(uint64_t& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isLabel())
	{
		val = static_cast<uint64_t>(t.labelToken());
		setGood();
		return *this;
	}
	if (t.good() && t.isScalar())
	{
		val = static_cast<uint64_t>(t.scalarToken());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(float& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isNumber())
	{
		val = static_cast<float>(t.number());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(double& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isNumber())
	{
		val = static_cast<double>(t.number());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(long double& val)
{
	XFoam_Token t;
	read(t);
	if (t.good() && t.isNumber())
	{
		val = static_cast<long double>(t.number());
		setGood();
		return *this;
	}
	if (t.good())
	{
		putBack(t);
	}
	setFail();
	return *this;
}

XFoam_IStream& XFoam_ITstream::read(char*, XFoam_StreamSize)
{
	return *this;
}

XFoam_IStream& XFoam_ITstream::rewind()
{
	tokenIndex_ = 0;

	if (size())
	{
		lineNumber_ = XFoam_TokenList::first().lineNumber();
	}

	setGood();

	XFoam_Token discard;
	(void)XFoam_IStream::getBack(discard);

	return *this;
}

int XFoam_ITstream::peek()
{
	XFoam_Token t;
	if (peekBack(t))
	{
		if (t.good() && t.isPunctuation())
		{
			return static_cast<int>(static_cast<unsigned char>(t.pToken()));
		}
		return -1;
	}
	if (tokenIndex_ < size())
	{
		const XFoam_Token& nt = operator[](tokenIndex_);
		if (nt.good() && nt.isPunctuation())
		{
			return static_cast<int>(static_cast<unsigned char>(nt.pToken()));
		}
		return -1;
	}
	return -1;
}

XFoam_IStream& XFoam_ITstream::get(char& c)
{
	return read(c);
}

XFoam_IStream& XFoam_ITstream::get()
{
	char c = '\0';
	return get(c);
}

XFoam_IStream& XFoam_ITstream::putback(const char c)
{
	putBack(XFoam_Token(static_cast<XFoam_Token::punctuationToken>(c), lineNumber()));
	return *this;
}

// * * * * * * * * * * XFoam_Entry * * * * * * * * * * * * * * * * * * * //

int XFoam_Entry::disableFunctionEntries = 0;

XFoam_Entry::XFoam_Entry(const XFoam_KeyType& keyword, const XFoam_Label lineNumber)
	: XFoam_DLListBase::link()
	, keyword_(keyword)
	, startLineNumber_(lineNumber)
{
}

XFoam_Entry::XFoam_Entry(const XFoam_Entry& e)
	: XFoam_DLListBase::link()
	, keyword_(e.keyword_)
	, startLineNumber_(e.startLineNumber_)
{
}

XFoam_AutoPtr<XFoam_Entry> XFoam_Entry::clone() const
{
	return clone(XFoam_Dictionary::null);
}

bool XFoam_Entry::getKeyword(
	XFoam_KeyType& keyword,
	XFoam_Token& keywordToken,
	XFoam_Label& keywordLineNo,
	XFoam_IStream& is)
{
	XFoam_Label lineNo = keywordLineNo;
	do
	{
		if (!XFoam_readTokenFromStream(is, lineNo, keywordToken) || !keywordToken.good())
		{
			return false;
		}
	} while (keywordToken == XFoam_Token::END_STATEMENT);
	keyword = keywordToken;
	keywordLineNo = lineNo;
	return !keyword.isUndefined();
}

bool XFoam_Entry::getKeyword(XFoam_KeyType& keyword, XFoam_Label& keywordLineNo, XFoam_IStream& is)
{
	XFoam_Token keywordToken;
	const bool ok = getKeyword(keyword, keywordToken, keywordLineNo, is);
	if (ok)
	{
		return true;
	}
	if (keywordToken == XFoam_Token::END_BLOCK || is.peek() == std::char_traits<char>::eof())
	{
		return false;
	}
	std::cerr << "--> XFoam Warning : " << std::endl
			  << "    From function XFoam_Entry::getKeyword(keyType&, Label&, IStream&)" << std::endl
			  << "    found invalid token while reading stream" << std::endl;
	return false;
}

bool XFoam_Entry::New(XFoam_Dictionary& parentDict, XFoam_IStream& is)
{
	(void)parentDict;
	(void)is;
	return false;
}

XFoam_AutoPtr<XFoam_Entry> XFoam_Entry::New(XFoam_IStream& is)
{
	(void)is;
	return XFoam_AutoPtr<XFoam_Entry>();
}

bool XFoam_Entry::read(const XFoam_Dictionary& dict, XFoam_IStream& is)
{
	(void)dict;
	(void)is;
	// ????Foam::entry::read?? false??
	return false;
}

void XFoam_Entry::operator=(const XFoam_Entry& e)
{
	if (this == &e)
	{
		throw XFoam_Error(XFoam_String("XFoam_Entry::operator=: attempted assignment to self"));
	}
	keyword_ = e.keyword_;
}

bool XFoam_Entry::operator==(const XFoam_Entry& e) const
{
	if (keyword_ != e.keyword_)
	{
		return false;
	}
	XFoam_OStringStream oss1;
	oss1 << *this;
	XFoam_OStringStream oss2;
	oss2 << e;
	return static_cast<std::string>(oss1.str()) == static_cast<std::string>(oss2.str());
}

bool XFoam_Entry::operator!=(const XFoam_Entry& e) const
{
	return !operator==(e);
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Entry& e)
{
	e.write(os);
	return os;
}

// * * * * * * * * * * XFoam_PrimitiveEntry * * * * * * * * * * * * * * * * //

void XFoam_PrimitiveEntry::append(const XFoam_UList<XFoam_Token>& varTokens)
{
	for (XFoam_Label i = 0; i < varTokens.size(); ++i)
	{
		newElmt(tokenIndex()++) = varTokens[i];
	}
}

void XFoam_PrimitiveEntry::append(const XFoam_Token& currToken, const XFoam_Dictionary& dict, XFoam_IStream& is)
{
	if (XFoam_Entry::disableFunctionEntries)
	{
		newElmt(tokenIndex()++) = currToken;
	}
	else if (currToken.isFunctionName())
	{
		if (!expandFunction(currToken.functionNameToken(), dict, is))
		{
			newElmt(tokenIndex()++) = currToken;
		}
	}
	else if (currToken.isVariable())
	{
		if (!expandVariable(currToken.variableToken(), dict))
		{
			newElmt(tokenIndex()++) = currToken;
		}
	}
	else
	{
		newElmt(tokenIndex()++) = currToken;
	}
}

bool XFoam_PrimitiveEntry::expandFunction(
	const XFoam_FunctionName& hashFn,
	const XFoam_Dictionary& parentDict,
	XFoam_IStream& is)
{
	(void)hashFn;
	(void)parentDict;
	(void)is;
	// XFoam ????Foam::functionEntry::execute?? false??
	return false;
}

bool XFoam_PrimitiveEntry::expandVariable(const XFoam_Variable& w, const XFoam_Dictionary& dict)
{
	if (w.size() > 2 && w[0] == '$' && w[1] == '{')
	{
		// XFoam ????Foam::stringOps::inplaceExpandEntry ????????false??
		(void)dict;
		return false;
	}
	const XFoam_String varName = w.size() > 1 ? XFoam_String(w.substr(1)) : XFoam_String();
	const XFoam_Entry* ePtr = dict.lookupScopedEntryPtr(XFoam_Word(varName), true, false);
	if (ePtr)
	{
		if (ePtr->isDict())
		{
			const XFoam_TokenList tl = ePtr->dict().tokens();
			append(static_cast<const XFoam_UList<XFoam_Token>&>(tl));
		}
		else
		{
			const XFoam_ITstream& s = ePtr->stream();
			append(static_cast<const XFoam_UList<XFoam_Token>&>(static_cast<const XFoam_TokenList&>(s)));
		}
		return true;
	}
	const XFoam_String envStr = xf_getEnvStub(varName);
	if (envStr.empty())
	{
		// XFoam ????Foam::FatalIOError ??????????false ????????token??
		return false;
	}
	// XFoam ????Foam::IStringStream ??env ????token ??????? word token??
	XFoam_TokenList one(1);
	one[0] = XFoam_Token(XFoam_Word(envStr), 1);
	append(static_cast<const XFoam_UList<XFoam_Token>&>(one));
	return true;
}

void XFoam_PrimitiveEntry::readEntry(const XFoam_Dictionary& dict, XFoam_IStream& is)
{
	tokenIndex() = 0;
	if (read(dict, is))
	{
		setSize(tokenIndex());
		tokenIndex() = 0;
	}
	else
	{
		std::ostringstream os;
		os << "ill defined primitiveEntry starting at keyword '" << static_cast<const std::string&>(keyword()) << '\''
		   << " on line " << startLineNumber();
		throw XFoam_Error(os.str());
	}
}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, XFoam_IStream& is)
	: XFoam_Entry(key, -1)
	, XFoam_ITstream(
		  XFoam_FileName(std::string("input") + "/" + static_cast<std::string>(key)),
		  XFoam_TokenList(10),
		  XFoam_IOstream::ASCII,
		  XFoam_IOstream::currentVersion,
		  false)
{
	readEntry(XFoam_Dictionary::null, is);
}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(
	const XFoam_KeyType& key,
	const XFoam_Dictionary& parentDict,
	XFoam_IStream& is)
	: XFoam_Entry(key, -1)
	, XFoam_ITstream(
		  XFoam_FileName(static_cast<std::string>(parentDict.name().name()) + "/" + static_cast<std::string>(key)),
		  XFoam_TokenList(10),
		  XFoam_IOstream::ASCII,
		  XFoam_IOstream::currentVersion,
		  false)
{
	readEntry(parentDict, is);
}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_ITstream& is)
	: XFoam_Entry(key)
	, XFoam_ITstream(is)
{
	name() = XFoam_FileName(static_cast<std::string>(static_cast<const XFoam_ITstream&>(is).name()) + "/" + static_cast<std::string>(keyword()));
}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_Token& t)
	: XFoam_Entry(key)
	, XFoam_ITstream(key, XFoam_TokenList(XFoam_Label(1), t))
{}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_UList<XFoam_Token>& tokens)
	: XFoam_Entry(key)
	, XFoam_ITstream(key, tokens)
{}

XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, XFoam_TokenList&& tokens)
	: XFoam_Entry(key)
	, XFoam_ITstream(key, std::move(tokens))
{}

XFoam_Label XFoam_PrimitiveEntry::endLineNumber() const
{
	const XFoam_UList<XFoam_Token>& tokens = static_cast<const XFoam_UList<XFoam_Token>&>(
		static_cast<const XFoam_TokenList&>(*this));
	if (tokens.empty())
	{
		return startLineNumber();
	}
	return tokens[tokens.size() - 1].lineNumber();
}

XFoam_ITstream& XFoam_PrimitiveEntry::stream() const
{
	XFoam_PrimitiveEntry& p = const_cast<XFoam_PrimitiveEntry&>(*this);
	p.rewind();
	return p;
}

const XFoam_Dictionary& XFoam_PrimitiveEntry::dict() const
{
	throw XFoam_Error("Attempt to return primitive entry as a sub-dictionary");
}

XFoam_Dictionary& XFoam_PrimitiveEntry::dict()
{
	throw XFoam_Error("Attempt to return primitive entry as a sub-dictionary");
}

bool XFoam_PrimitiveEntry::read(const XFoam_Dictionary& dict, XFoam_IStream& is)
{
	XFoam_Label lineNo = 1;
	startLineNumber() = lineNo;
	XFoam_Label blockCount = 0;
	XFoam_Token currToken;
	if (XFoam_readTokenFromStream(is, lineNo, currToken) && currToken.good() && currToken != XFoam_Token::END_STATEMENT)
	{
		append(currToken, dict, is);
		if (currToken == XFoam_Token::BEGIN_BLOCK || currToken == XFoam_Token::BEGIN_LIST)
		{
			++blockCount;
		}
		// OpenFOAM???? `keyword { ... }` ???????? `(...)` ?????????????
		// ????? END_STATEMENT ? depth==0 ???????????????? primitive?
		while (XFoam_readTokenFromStream(is, lineNo, currToken) && currToken.good())
		{
			if (currToken == XFoam_Token::END_STATEMENT && blockCount == 0)
			{
				// 已读入的分号属于下一条目/外层，须放回供 Dictionary::read 跳过（否则 ITstream 尾 eof 再读 Fatal）。
				is.putBack(currToken);
				break;
			}
			if (currToken == XFoam_Token::BEGIN_BLOCK || currToken == XFoam_Token::BEGIN_LIST)
			{
				++blockCount;
			}
			else if (currToken == XFoam_Token::END_BLOCK || currToken == XFoam_Token::END_LIST)
			{
				--blockCount;
			}
			append(currToken, dict, is);
			if (blockCount == 0
				&& (currToken == XFoam_Token::END_BLOCK || currToken == XFoam_Token::END_LIST))
			{
				break;
			}
		}
	}
	if (currToken.good())
	{
		return true;
	}
	return false;
}

void XFoam_PrimitiveEntry::write(XFoam_OStream& os, const bool contentsOnly) const
{
	if (!contentsOnly && keyword().size())
	{
		XFoam_writeKeyword(os, keyword());
	}
	for (XFoam_Label i = 0; i < size(); ++i)
	{
		os << operator[](i);
		if (i < size() - 1)
		{
			os << ' ';
		}
	}
	if (!contentsOnly)
	{
		os << static_cast<char>(XFoam_Token::END_STATEMENT) << '\n';
	}
}

void XFoam_PrimitiveEntry::write(XFoam_OStream& os) const
{
	write(os, false);
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_PrimitiveEntry>& ip)
{
	const XFoam_PrimitiveEntry& e = ip();
	// XFoam ????Foam::primitiveEntry::print??????write ????
	e.write(os, true);
	const XFoam_Label nPrintTokens = 10;
	os << "    primitiveEntry '" << static_cast<const std::string&>(e.keyword()) << "' comprises ";
	for (XFoam_Label i = 0; i < e.size() && i < nPrintTokens; ++i)
	{
		os << '\n'
		   << "        " << e[i];
	}
	if (e.size() > nPrintTokens)
	{
		os << " ...";
	}
	os << '\n';
	return os;
}

int XFoam_Dictionary::writeOptionalEntries = 0;

XFoam_API const XFoam_Dictionary XFoam_Dictionary::null;

XFoam_Dictionary::XFoam_Dictionary()
	: XFoam_DictionaryName()
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(XFoam_Dictionary::null)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
}

XFoam_Dictionary::XFoam_Dictionary(const XFoam_FileName& name)
	: XFoam_DictionaryName(name)
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(XFoam_Dictionary::null)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
}

XFoam_Dictionary::XFoam_Dictionary(const XFoam_FileName& name, const XFoam_Dictionary& parentDict)
	: XFoam_DictionaryName(parentDict.name().size() ? parentDict.name() / name : name)
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(parentDict)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
}

XFoam_Dictionary::XFoam_Dictionary(const XFoam_Dictionary& parentDict, const XFoam_Dictionary& dict)
	: XFoam_DictionaryName(dict.name())
	, XFoam_IDLList<XFoam_Entry>(dict, *this)
	, hashedEntries_()
	, parent_(parentDict)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
	for (const_iterator iter = begin(); iter != end(); ++iter)
	{
		XFoam_Entry* e = const_cast<XFoam_Entry*>(&iter());
		hashedEntries_.insert(iter().keyword(), e);
		if (iter().keyword().isPattern())
		{
			patternEntries_.append(e);
			patternRegexps_.append(XFoam_AutoPtr<XFoam_RegExp>(new XFoam_RegExp(xf_patternRegexText(iter().keyword()))));
		}
	}
}

XFoam_Dictionary::XFoam_Dictionary(const XFoam_Dictionary& dict)
	: XFoam_DictionaryName(dict.name())
	, XFoam_IDLList<XFoam_Entry>(dict, *this)
	, hashedEntries_()
	, parent_(XFoam_Dictionary::null)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
	for (const_iterator iter = begin(); iter != end(); ++iter)
	{
		XFoam_Entry* e = const_cast<XFoam_Entry*>(&iter());
		hashedEntries_.insert(iter().keyword(), e);
		if (iter().keyword().isPattern())
		{
			patternEntries_.append(e);
			patternRegexps_.append(XFoam_AutoPtr<XFoam_RegExp>(new XFoam_RegExp(xf_patternRegexText(iter().keyword()))));
		}
	}
}

XFoam_Dictionary::XFoam_Dictionary(const XFoam_Dictionary* dictPtr)
	: XFoam_DictionaryName()
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(XFoam_Dictionary::null)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
	if (dictPtr)
	{
		operator=(*dictPtr);
	}
}

XFoam_Dictionary::~XFoam_Dictionary() = default;

XFoam_AutoPtr<XFoam_Dictionary> XFoam_Dictionary::clone() const
{
	return XFoam_AutoPtr<XFoam_Dictionary>(new XFoam_Dictionary(*this));
}

const XFoam_Dictionary& XFoam_Dictionary::topDict() const
{
	const XFoam_Dictionary& p = parent();
	if (&p != this && !p.name().empty())
	{
		return p.topDict();
	}
	return *this;
}

XFoam_Word XFoam_Dictionary::topDictKeyword() const
{
	const XFoam_Dictionary& p = parent();
	if (&p != this && !p.name().empty())
	{
		const XFoam_Word pKeyword = p.topDictKeyword();
		const char pSeparator = '/';
		return pKeyword == XFoam_Word::null ? dictName() : XFoam_Word(pKeyword + pSeparator + static_cast<std::string>(dictName()));
	}
	return XFoam_Word::null;
}

XFoam_Label XFoam_Dictionary::startLineNumber() const
{
	if (size())
	{
		return first()->startLineNumber();
	}
	return -1;
}

XFoam_Label XFoam_Dictionary::endLineNumber() const
{
	if (size())
	{
		return last()->endLineNumber();
	}
	return -1;
}

XFoam_SHA1Digest XFoam_Dictionary::digest() const
{
	// ????Foam::OSHA1stream?????????
	return XFoam_SHA1Digest();
}

XFoam_TokenList XFoam_Dictionary::tokens() const
{
	// ????Foam::OStringStream/IStringStream/token ?????
	return XFoam_TokenList();
}

bool XFoam_Dictionary::findInPatterns(
	const bool patternMatch,
	const XFoam_Word& Keyword,
	XFoam_DLList<XFoam_Entry*>::const_iterator& wcLink,
	XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::const_iterator& reLink) const
{
	const XFoam_Label nPat = patternEntries_.size();
	if (nPat)
	{
		wcLink = patternEntries_.begin();
		reLink = patternRegexps_.begin();
		for (XFoam_Label i = 0; i < nPat; ++i)
		{
			if (patternMatch ? (*reLink)->match(Keyword) : (*wcLink)->keyword() == Keyword)
			{
				return true;
			}
			++reLink;
			++wcLink;
		}
	}
	return false;
}

bool XFoam_Dictionary::findInPatterns(
	const bool patternMatch,
	const XFoam_Word& Keyword,
	XFoam_DLList<XFoam_Entry*>::iterator& wcLink,
	XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::iterator& reLink)
{
	const XFoam_Label nPat = patternEntries_.size();
	if (nPat)
	{
		wcLink = patternEntries_.begin();
		reLink = patternRegexps_.begin();
		for (XFoam_Label i = 0; i < nPat; ++i)
		{
			if (patternMatch ? (*reLink)->match(Keyword) : (*wcLink)->keyword() == Keyword)
			{
				return true;
			}
			++reLink;
			++wcLink;
		}
	}
	return false;
}

bool XFoam_Dictionary::found(const XFoam_Word& keyword, bool recursive, bool patternMatch) const
{
	if (hashedEntries_.found(keyword))
	{
		return true;
	}
	if (patternMatch && patternEntries_.size())
	{
		XFoam_DLList<XFoam_Entry*>::const_iterator wcLink = patternEntries_.begin();
		XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::const_iterator reLink = patternRegexps_.begin();
		if (findInPatterns(patternMatch, keyword, wcLink, reLink))
		{
			return true;
		}
	}
	if (recursive && &parent_ != &XFoam_Dictionary::null)
	{
		return parent_.found(keyword, recursive, patternMatch);
	}
	return false;
}

const XFoam_Entry* XFoam_Dictionary::lookupEntryPtr(const XFoam_Word& keyword, bool recursive, bool patternMatch) const
{
	XFoam_HashTable<XFoam_Entry*>::const_iterator iter = hashedEntries_.find(keyword);
	if (iter == hashedEntries_.cend())
	{
		if (patternMatch && patternEntries_.size())
		{
			XFoam_DLList<XFoam_Entry*>::const_iterator wcLink = patternEntries_.begin();
			XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::const_iterator reLink = patternRegexps_.begin();
			if (findInPatterns(patternMatch, keyword, wcLink, reLink))
			{
				return *wcLink;
			}
		}
		if (recursive && &parent_ != &XFoam_Dictionary::null)
		{
			return parent_.lookupEntryPtr(keyword, recursive, patternMatch);
		}
		return nullptr;
	}
	return iter();
}

XFoam_Entry* XFoam_Dictionary::lookupEntryPtr(const XFoam_Word& keyword, bool recursive, bool patternMatch)
{
	XFoam_HashTable<XFoam_Entry*>::iterator iter = hashedEntries_.find(keyword);
	if (iter == hashedEntries_.end())
	{
		if (patternMatch && patternEntries_.size())
		{
			XFoam_DLList<XFoam_Entry*>::iterator wcLink = patternEntries_.begin();
			XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::iterator reLink = patternRegexps_.begin();
			if (findInPatterns(patternMatch, keyword, wcLink, reLink))
			{
				return *wcLink;
			}
		}
		if (recursive && &parent_ != &XFoam_Dictionary::null)
		{
			return const_cast<XFoam_Dictionary&>(parent_).lookupEntryPtr(keyword, recursive, patternMatch);
		}
		return nullptr;
	}
	return iter();
}

const XFoam_Entry* XFoam_Dictionary::lookupEntryPtrBackwardsCompatible(
	const XFoam_WordList& keywords,
	bool recursive,
	bool patternMatch) const
{
	const XFoam_Entry* result = nullptr;
	for (XFoam_Label keywordi = 0; keywordi < keywords.size(); ++keywordi)
	{
		const XFoam_Entry* entryPtr = lookupEntryPtr(keywords[keywordi], recursive, patternMatch);
		if (entryPtr)
		{
			if (result)
			{
				(void)result;
				(void)entryPtr;
			}
			else
			{
				result = entryPtr;
			}
		}
	}
	return result;
}

const XFoam_Entry& XFoam_Dictionary::lookupEntry(const XFoam_Word& keyword, bool recursive, bool patternMatch) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, recursive, patternMatch);
	if (entryPtr == nullptr)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keyword " << keyword << " is undefined in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return *entryPtr;
}

const XFoam_Entry& XFoam_Dictionary::lookupEntryBackwardsCompatible(
	const XFoam_WordList& keywords,
	bool recursive,
	bool patternMatch) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtrBackwardsCompatible(keywords, recursive, patternMatch);
	if (entryPtr == nullptr)
	{
		return lookupEntry(keywords[0], recursive, patternMatch);
	}
	return *entryPtr;
}

XFoam_ITstream& XFoam_Dictionary::lookup(const XFoam_Word& keyword, bool recursive, bool patternMatch) const
{
	const XFoam_Entry* e = lookupEntryPtr(keyword, recursive, patternMatch);
	if (!e)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keyword " << keyword << " undefined in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return e->stream();
}

XFoam_ITstream& XFoam_Dictionary::lookupBackwardsCompatible(
	const XFoam_WordList& keywords,
	bool recursive,
	bool patternMatch) const
{
	const XFoam_Entry* e = lookupEntryPtrBackwardsCompatible(keywords, recursive, patternMatch);
	if (!e)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keywords lookup failed in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return e->stream();
}

void XFoam_Dictionary::clear()
{
	XFoam_IDLList<XFoam_Entry>::clear();
	hashedEntries_.clear();
	patternEntries_.clear();
	patternRegexps_.clear();
}

void XFoam_Dictionary::transfer(XFoam_Dictionary& dict)
{
	name() = dict.name();
	XFoam_IDLList<XFoam_Entry>::transfer(dict);
	hashedEntries_.transfer(dict.hashedEntries_);
	patternEntries_.transfer(dict.patternEntries_);
	patternRegexps_.transfer(dict.patternRegexps_);
}

bool XFoam_Dictionary::add(XFoam_Entry* entryPtr, bool mergeEntry)
{
	if (!entryPtr)
	{
		return false;
	}
	(void)mergeEntry;
	if (!hashedEntries_.insert(entryPtr->keyword(), entryPtr))
	{
		if (XFoam_Entry::disableFunctionEntries)
		{
			entryPtr->name() = name() + '/' + entryPtr->keyword();
			append(entryPtr);
			return true;
		}
		delete entryPtr;
		return false;
	}
	entryPtr->name() = name() + '/' + entryPtr->keyword();
	append(entryPtr);
	if (entryPtr->keyword().isPattern())
	{
		patternEntries_.append(entryPtr);
		patternRegexps_.append(XFoam_AutoPtr<XFoam_RegExp>(new XFoam_RegExp(xf_patternRegexText(entryPtr->keyword()))));
	}
	return true;
}

void XFoam_Dictionary::add(const XFoam_Entry& e, bool mergeEntry)
{
	add(e.clone(*this).ptr(), mergeEntry);
}

bool XFoam_Dictionary::remove(const XFoam_Word& Keyword)
{
	XFoam_HashTable<XFoam_Entry*>::iterator iter = hashedEntries_.find(Keyword);
	if (iter != hashedEntries_.end())
	{
		XFoam_DLList<XFoam_Entry*>::iterator wcLink = patternEntries_.begin();
		XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::iterator reLink = patternRegexps_.begin();
		if (findInPatterns(false, Keyword, wcLink, reLink))
		{
			(void)patternEntries_.remove(wcLink);
			(void)patternRegexps_.remove(reLink);
		}
		XFoam_Entry* e = iter();
		XFoam_IDLList<XFoam_Entry>::remove(e);
		delete e;
		hashedEntries_.erase(iter);
		return true;
	}
	return false;
}

bool XFoam_Dictionary::merge(const XFoam_Dictionary& dict)
{
	if (this == &dict)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "attempted merge to self for dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	bool changed = false;
	for (const_iterator iter = dict.begin(); iter != dict.end(); ++iter)
	{
		XFoam_HashTable<XFoam_Entry*>::iterator fnd = hashedEntries_.find(iter().keyword());
		if (fnd != hashedEntries_.end())
		{
			if (fnd()->isDict() && iter().isDict())
			{
				if (fnd()->dict().merge(iter().dict()))
				{
					changed = true;
				}
			}
			else
			{
				add(iter().clone(*this).ptr(), true);
				changed = true;
			}
		}
		else
		{
			add(iter().clone(*this).ptr());
			changed = true;
		}
	}
	return changed;
}

void XFoam_Dictionary::operator=(const XFoam_Dictionary& rhs)
{
	if (this == &rhs)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "attempted assignment to self for dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	name() = rhs.name();
	clear();
	for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter)
	{
		add(iter().clone(*this).ptr(), false);
	}
}

void XFoam_Dictionary::operator+=(const XFoam_Dictionary& rhs)
{
	if (this == &rhs)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "attempted addition assignment to self for dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter)
	{
		add(iter().clone(*this).ptr(), false);
	}
}

void XFoam_Dictionary::operator|=(const XFoam_Dictionary& rhs)
{
	if (this == &rhs)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "attempted assignment to self for dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter)
	{
		if (!found(iter().keyword()))
		{
			add(iter().clone(*this).ptr(), false);
		}
	}
}

void XFoam_Dictionary::operator<<=(const XFoam_Dictionary& rhs)
{
	if (this == &rhs)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "attempted assignment to self for dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter)
	{
		set(iter().clone(*this).ptr());
	}
}

XFoam_WordList XFoam_Dictionary::toc() const
{
	XFoam_WordList keys(static_cast<XFoam_Label>(size()));
	XFoam_Label nKeys = 0;
	for (const_iterator iter = begin(); iter != end(); ++iter)
	{
		keys[nKeys++] = iter().keyword();
	}
	keys.setSize(nKeys);
	return keys;
}

XFoam_WordList XFoam_Dictionary::sortedToc() const
{
	const std::vector<XFoam_String> v = hashedEntries_.sortedToc();
	XFoam_WordList out(static_cast<XFoam_Label>(v.size()));
	for (XFoam_Label i = 0; i < static_cast<XFoam_Label>(v.size()); ++i)
	{
		out[i] = XFoam_Word(v[static_cast<XFoam_Size>(i)]);
	}
	return out;
}

XFoam_List<XFoam_KeyType> XFoam_Dictionary::keys(bool patterns) const
{
	XFoam_List<XFoam_KeyType> k(static_cast<XFoam_Label>(size()));
	XFoam_Label nKeys = 0;
	for (const_iterator iter = begin(); iter != end(); ++iter)
	{
		if (iter().keyword().isPattern() ? patterns : !patterns)
		{
			k[nKeys++] = iter().keyword();
		}
	}
	k.setSize(nKeys);
	return k;
}

bool XFoam_Dictionary::global() const
{
	return false;
}

bool XFoam_Dictionary::read(XFoam_IStream& is, const bool keepHeader)
{
	(void)keepHeader;
	XFoam_Label lineNo = 1;
	for (;;)
	{
		XFoam_Token keywordToken;
		do
		{
			if (!XFoam_readTokenFromStream(is, lineNo, keywordToken) || !keywordToken.good())
			{
				return true;
			}
		} while (keywordToken == XFoam_Token::END_STATEMENT);

		if (keywordToken == XFoam_Token::END_BLOCK && is.peek() == std::char_traits<char>::eof())
		{
			return true;
		}

		const XFoam_KeyType keyword(keywordToken);
		if (keyword.isUndefined())
		{
			return true;
		}
		if (keyword.isFunctionName())
		{
			while (is.peek() != std::char_traits<char>::eof() && is.peek() != '\n')
			{
				is.get();
			}
			if (is.peek() == '\n')
			{
				is.get();
				++lineNo;
			}
			continue;
		}
		XFoam_PrimitiveEntry* pe = nullptr;
		try
		{
			pe = new XFoam_PrimitiveEntry(keyword, *this, is);
		}
		catch (const XFoam_Error&)
		{
			return false;
		}
		catch (const std::exception&)
		{
			return false;
		}
		catch (...)
		{
			return false;
		}
		if (!add(pe))
		{
			if (keyword.isPattern())
			{
				delete pe;
				return false;
			}
			const XFoam_Word wKey(static_cast<const std::string&>(keyword));
			(void)remove(wKey);
			if (!add(pe))
			{
				delete pe;
				return false;
			}
		}
	}
}

void XFoam_Dictionary::write(XFoam_OStream& os, bool subDict) const
{
	(void)os;
	(void)subDict;
	// ????Foam::token::BEGIN_BLOCK / entry::write
}

XFoam_AutoPtr<XFoam_Dictionary> XFoam_Dictionary::New(XFoam_IStream& is)
{
	return XFoam_AutoPtr<XFoam_Dictionary>(new XFoam_Dictionary(is));
}

XFoam_Dictionary::XFoam_Dictionary(XFoam_IStream& is, const bool keepHeader)
	: XFoam_DictionaryName()
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(XFoam_Dictionary::null)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
	(void)is;
	(void)keepHeader;
}

XFoam_Dictionary::XFoam_Dictionary(
	const XFoam_FileName& name,
	const XFoam_Dictionary& parentDict,
	XFoam_IStream& is)
	: XFoam_DictionaryName(parentDict.name().size() ? parentDict.name() / name : name)
	, XFoam_IDLList<XFoam_Entry>()
	, hashedEntries_()
	, parent_(parentDict)
	, filePtr_(nullptr)
	, patternEntries_()
	, patternRegexps_()
{
	(void)is;
}

XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Dictionary& dict)
{
	dict.clear();
	dict.read(is);
	return is;
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Dictionary& dict)
{
	dict.write(os, true);
	return os;
}

XFoam_API XFoam_Dictionary operator+(const XFoam_Dictionary& dict1, const XFoam_Dictionary& dict2)
{
	XFoam_Dictionary sum(dict1);
	sum += dict2;
	return sum;
}

XFoam_API XFoam_Dictionary operator|(const XFoam_Dictionary& dict1, const XFoam_Dictionary& dict2)
{
	XFoam_Dictionary sum(dict1);
	sum |= dict2;
	return sum;
}

XFoam_API void XFoam_dictArgList(
	const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
	XFoam_Word& configName,
	XFoam_List<XFoam_Tuple2<XFoam_WordRe, XFoam_Label>>& args,
	XFoam_List<XFoam_Tuple3<XFoam_Word, XFoam_String, XFoam_Label>>& namedArgs)
{
	(void)args;
	(void)namedArgs;
	configName = XFoam_Word(argString.first());
}

XFoam_API void XFoam_dictArgList(
	const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
	XFoam_List<XFoam_Tuple2<XFoam_WordRe, XFoam_Label>>& args,
	XFoam_List<XFoam_Tuple3<XFoam_Word, XFoam_String, XFoam_Label>>& namedArgs)
{
	(void)argString;
	(void)args;
	(void)namedArgs;
}

XFoam_API XFoam_Pair<XFoam_Word> XFoam_dictAndKeyword(const XFoam_Word& scopedName)
{
	const XFoam_String::size_type i = scopedName.rfind('/');
	if (i != XFoam_String::npos)
	{
		return XFoam_Pair<XFoam_Word>(XFoam_Word(scopedName.substr(0, i)), XFoam_Word(scopedName.substr(i + 1)));
	}
	return XFoam_Pair<XFoam_Word>(XFoam_Word(), scopedName);
}

XFoam_API XFoam_WordList XFoam_listAllConfigFiles(const XFoam_FileName& configFilesPath)
{
	(void)configFilesPath;
	return XFoam_WordList();
}

XFoam_API XFoam_FileName XFoam_findConfigFile(
	const XFoam_Word& configName,
	const XFoam_FileName& configFilesPath,
	const XFoam_Word& configFilesDir,
	const XFoam_Word& region)
{
	(void)configName;
	(void)configFilesPath;
	(void)configFilesDir;
	(void)region;
	return XFoam_FileName::null;
}

XFoam_API XFoam_String XFoam_expandArg(const XFoam_String& arg, XFoam_Dictionary& dict, const XFoam_Label lineNumber)
{
	(void)dict;
	(void)lineNumber;
	return arg;
}

XFoam_API void XFoam_addArgEntry(XFoam_Dictionary& dict, const XFoam_Word& keyword, const XFoam_String& value, const XFoam_Label lineNumber)
{
	(void)dict;
	(void)keyword;
	(void)value;
	(void)lineNumber;
}

XFoam_API bool XFoam_readConfigFile(
	const XFoam_Word& configType,
	const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
	XFoam_Dictionary& parentDict,
	const XFoam_FileName& configFilesPath,
	const XFoam_Word& configFilesDir,
	const XFoam_Word& region)
{
	(void)configType;
	(void)argString;
	(void)parentDict;
	(void)configFilesPath;
	(void)configFilesDir;
	(void)region;
	return false;
}

XFoam_API void XFoam_writeEntry(XFoam_OStream& os, const XFoam_Dictionary& dict)
{
	os << dict;
}

const XFoam_Entry* XFoam_Dictionary::lookupScopedSubEntryPtr(
	const XFoam_Word& keyword,
	bool recursive,
	bool patternMatch) const
{
	// ????OF lookupDotScoped / lookupSlashScoped / includedDictionary '!' ???????????
	return lookupEntryPtr(keyword, recursive, patternMatch);
}

const XFoam_Entry* XFoam_Dictionary::lookupScopedEntryPtr(
	const XFoam_Word& keyword,
	bool recursive,
	bool patternMatch) const
{
	if (!keyword.empty() && keyword[0] == '!')
	{
		const XFoam_Dictionary* dictPtr = this;
		while (&dictPtr->parent_ != &XFoam_Dictionary::null)
		{
			dictPtr = &dictPtr->parent_;
		}
		return dictPtr->lookupScopedSubEntryPtr(keyword.substr(1), false, patternMatch);
	}
	return lookupScopedSubEntryPtr(keyword, recursive, patternMatch);
}

bool XFoam_Dictionary::substituteKeyword(const XFoam_Word& keyword)
{
	(void)keyword;
	// ????OF substituteKeyword?? ????primitiveEntry/dictionaryEntry???
	return false;
}

bool XFoam_Dictionary::isDict(const XFoam_Word& keyword) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	return entryPtr && entryPtr->isDict();
}

const XFoam_Dictionary* XFoam_Dictionary::subDictPtr(const XFoam_Word& keyword) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	return entryPtr ? &entryPtr->dict() : nullptr;
}

XFoam_Dictionary* XFoam_Dictionary::subDictPtr(const XFoam_Word& keyword)
{
	XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	return entryPtr ? &entryPtr->dict() : nullptr;
}

const XFoam_Dictionary& XFoam_Dictionary::subDict(const XFoam_Word& keyword) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	if (!entryPtr)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keyword " << keyword << " is undefined in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return entryPtr->dict();
}

XFoam_Dictionary& XFoam_Dictionary::subDict(const XFoam_Word& keyword)
{
	XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	if (!entryPtr)
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keyword " << keyword << " is undefined in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return entryPtr->dict();
}

const XFoam_Dictionary& XFoam_Dictionary::subDictBackwardsCompatible(const XFoam_WordList& keywords) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtrBackwardsCompatible(keywords, false, true);
	if (!entryPtr)
	{
		return subDict(keywords[0]);
	}
	return entryPtr->dict();
}

const XFoam_Dictionary& XFoam_Dictionary::subOrEmptyDict(const XFoam_Word& keyword, const bool mustRead) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	if (!entryPtr)
	{
		if (mustRead)
		{
			XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
				<< "keyword " << keyword << " is undefined in dictionary " << name() << XFoam_exit(XFoam_FatalIOError, 1);
		}
		// ????OF ????????????? null ????
		return XFoam_Dictionary::null;
	}
	return entryPtr->dict();
}

const XFoam_Dictionary& XFoam_Dictionary::optionalSubDict(const XFoam_Word& keyword) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, true);
	return entryPtr ? entryPtr->dict() : *this;
}

const XFoam_Dictionary& XFoam_Dictionary::scopedDict(const XFoam_Word& keyword) const
{
	if (keyword.empty())
	{
		return *this;
	}
	const XFoam_Entry* entPtr = lookupScopedEntryPtr(keyword, false, false);
	if (!entPtr || !entPtr->isDict())
	{
		XFoam_FatalIOErrorInFunction(xf_ioLocDict(*this))
			<< "keyword " << keyword << " is undefined in dictionary " << name() << " or is not a dictionary"
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
	return entPtr->dict();
}

XFoam_Dictionary& XFoam_Dictionary::scopedDict(const XFoam_Word& keyword)
{
	return const_cast<XFoam_Dictionary&>(const_cast<const XFoam_Dictionary*>(this)->scopedDict(keyword));
}

void XFoam_Dictionary::add(const XFoam_KeyType&, const XFoam_Word&, bool)
{
	// ????primitiveEntry + token
}

void XFoam_Dictionary::add(const XFoam_KeyType&, const XFoam_String&, bool)
{
	// ????primitiveEntry + token
}

void XFoam_Dictionary::add(const XFoam_KeyType&, const XFoam_Label, bool)
{
	// ????primitiveEntry + token
}

void XFoam_Dictionary::add(const XFoam_KeyType&, const XFoam_Scalar, bool)
{
	// ????primitiveEntry + token
}

void XFoam_Dictionary::add(const XFoam_KeyType&, const XFoam_Dictionary&, bool)
{
	// ????dictionaryEntry
}

void XFoam_Dictionary::set(XFoam_Entry* entryPtr)
{
	if (!entryPtr)
	{
		return;
	}
	XFoam_Entry* existingPtr = lookupEntryPtr(entryPtr->keyword(), false, true);
	if (existingPtr && existingPtr->isDict())
	{
		existingPtr->dict().clear();
	}
	add(entryPtr, true);
}

void XFoam_Dictionary::set(const XFoam_Entry& e)
{
	set(e.clone(*this).ptr());
}

void XFoam_Dictionary::set(const XFoam_KeyType& k, const XFoam_Dictionary& d)
{
	(void)k;
	(void)d;
	// ????dictionaryEntry
}

void XFoam_Dictionary::remove(const XFoam_WordList& kw)
{
	for (XFoam_Label i = 0; i < kw.size(); ++i)
	{
		remove(kw[i]);
	}
}

bool XFoam_Dictionary::changeKeyword(
	const XFoam_KeyType& oldKeyword,
	const XFoam_KeyType& newKeyword,
	bool forceOverwrite)
{
	(void)oldKeyword;
	(void)newKeyword;
	(void)forceOverwrite;
	// ??????OF ????? pattern / HashTable ????????Entry ?????
	return false;
}

XFoam_ITstream& XFoam_Dictionary::operator[](const XFoam_Word& keyword) const
{
	return lookup(keyword);
}

void XFoam_Dictionary::assertNoConvertUnits(
	const char* typeName,
	const XFoam_Word& keyword,
	const XFoam_UnitConversion& defaultUnits,
	XFoam_ITstream& is) const
{
	(void)typeName;
	(void)keyword;
	(void)defaultUnits;
	(void)is;
}

XFoam_Dictionary::includedDictionary::includedDictionary(const XFoam_FileName& fName, const XFoam_Dictionary& parentDict)
	: XFoam_Dictionary(fName)
	, global_(parentDict.topDict().global())
{
	(void)parentDict;
	// ????Foam::IFstream / fileHandler ??included ????
}

bool XFoam_IODictionary::writeDictionaries = false;

XFoam_IODictionary::XFoam_IODictionary(const XFoam_IOobject& io, const XFoam_Word& wantedType)
	: XFoam_RegIOobject(io)
	, XFoam_Dictionary()
{
	(void)wantedType;
	XFoam_Dictionary::name() = XFoam_FileName(static_cast<const std::string&>(objectPath()));
}

XFoam_IODictionary::XFoam_IODictionary(const XFoam_IOobject& io)
	: XFoam_RegIOobject(io)
	, XFoam_Dictionary()
{
	XFoam_Dictionary::name() = XFoam_FileName(static_cast<const std::string&>(objectPath()));
	(void)readHeaderOk(XFoam_IOstream::ASCII, XFoam_Word(typeName));
	addWatch();
}

XFoam_IODictionary::XFoam_IODictionary(const XFoam_IOobject& io, const XFoam_Dictionary& dict)
	: XFoam_RegIOobject(io)
	, XFoam_Dictionary()
{
	XFoam_Dictionary::name() = XFoam_FileName(static_cast<const std::string&>(objectPath()));
	if (!readHeaderOk(XFoam_IOstream::ASCII, XFoam_Word(typeName)))
	{
		XFoam_Dictionary::operator=(dict);
	}
	addWatch();
}

XFoam_IODictionary::XFoam_IODictionary(const XFoam_IOobject& io, XFoam_IStream& is)
	: XFoam_RegIOobject(io)
	, XFoam_Dictionary()
{
	XFoam_Dictionary::name() = XFoam_FileName(static_cast<const std::string&>(objectPath()));
	is >> *static_cast<XFoam_Dictionary*>(this);
	addWatch();
}

XFoam_IODictionary::XFoam_IODictionary(const XFoam_IODictionary& dict)
	: XFoam_RegIOobject(static_cast<const XFoam_RegIOobject&>(dict))
	, XFoam_Dictionary(static_cast<const XFoam_Dictionary&>(dict))
{
}

XFoam_IODictionary::XFoam_IODictionary(XFoam_IODictionary&& dict)
	: XFoam_RegIOobject(static_cast<XFoam_RegIOobject&&>(dict))
	, XFoam_Dictionary(static_cast<XFoam_Dictionary&&>(dict))
{
}

XFoam_IODictionary::~XFoam_IODictionary() = default;

bool XFoam_IODictionary::global() const
{
	return true;
}

bool XFoam_IODictionary::readData(XFoam_IStream& is)
{
	is >> *static_cast<XFoam_Dictionary*>(this);
	// 读到文件尾时 eofbit 置位，good() 为 false，但字典已完整读入；以 !bad() 判定成功。
	return !is.bad();
}

bool XFoam_IODictionary::writeData(XFoam_OStream& os) const
{
	XFoam_Dictionary::write(os, false);
	return os.good();
}

void XFoam_IODictionary::operator=(const XFoam_IODictionary& rhs)
{
	XFoam_Dictionary::operator=(rhs);
}

void XFoam_IODictionary::operator=(XFoam_IODictionary&& rhs)
{
	XFoam_Dictionary::operator=(XFoam_move(static_cast<XFoam_Dictionary&&>(rhs)));
}

XFoam_IOobject XFoam_systemDictIO(const XFoam_FileName& path)
{
	static XFoam_Time standaloneRunTime;
	if (path.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_systemDict(path): empty path"));
	}
	// IOobject::objectPath() == path() / name()：instance 须为字典所在目录，name 为文件名，不能把「整文件路径」再当 instance。
	const XFoam_Word dictName(path.name());
	const XFoam_FileName parent = path.path();
	const XFoam_FileName instanceForIo = parent;
	return XFoam_systemDictIO(
		dictName,
		standaloneRunTime,
		XFoam_polyMeshDefaultRegion(),
		instanceForIo);
}

XFoam_IOobject XFoam_systemDictIO(
	const XFoam_Word& dictName,
	const XFoam_ObjectRegistry& ob,
	const XFoam_Word& regionName,
	const XFoam_FileName& path)
{
	const XFoam_Time& tm = ob.time();
	// 绝对路径（含 Windows 盘符）不得再拼在 system/ 之后，否则得到 "system/D:/..." 且 IOobject::path 非法。
	const XFoam_FileName instance = path.isAbsolute() ? path : (tm.system() / path);
	return XFoam_IOobject(
		dictName,
		instance,
		regionName == XFoam_polyMeshDefaultRegion() ? XFoam_Word::null : regionName,
		ob,
		XFoam_IOobject::MUST_READ_IF_MODIFIED,
		XFoam_IOobject::NO_WRITE,
		false);
}

XFoam_IODictionary XFoam_systemDict(
	const XFoam_Word& dictName,
	const XFoam_ObjectRegistry& ob,
	const XFoam_Word& regionName,
	const XFoam_FileName& path)
{
	return XFoam_IODictionary(XFoam_systemDictIO(dictName, ob, regionName, path));
}
