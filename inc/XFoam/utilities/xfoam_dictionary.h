#ifndef XFoam_Dictionary_H_
#define XFoam_Dictionary_H_
#ifdef _WIN32
#ifdef toc
#undef toc
#endif
#endif
#ifndef FOR_ALL_FIELD_TYPES
#define FOR_ALL_FIELD_TYPES(func)
#endif
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_unit.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_hash.h"
#include "XFoam/utilities/xfoam_list.h"

#include "XFoam/utilities/xfoam_tuple.h"
#include "XFoam/utilities/xfoam_regexp.h"
#include "XFoam/utilities/xfoam_ioobject.h"
#include <tuple>
#include <sstream>
#include <cctype>
#include <algorithm>

#include "XFoam/utilities/xfoam_error.h"

/*---------------------------------------------------------------------------*\
                           Class XFoam_KeyType Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_KeyType
	: public XFoam_Variable
{
	enum type
	{
		UNDEFINED,
		WORD,
		FUNCTIONNAME,
		VARIABLE,
		PATTERN
	};

	type type_;

public:
	static const XFoam_KeyType null;

	XFoam_KeyType();
	XFoam_KeyType(const XFoam_KeyType&);
	XFoam_KeyType(const XFoam_Word&);
	explicit XFoam_KeyType(const XFoam_FunctionName&);
	explicit XFoam_KeyType(const XFoam_Variable&);
	explicit XFoam_KeyType(const std::string&);
	XFoam_KeyType(const char*);
	explicit XFoam_KeyType(const XFoam_Token&);
	explicit XFoam_KeyType(XFoam_IStream&);

	bool isUndefined() const { return type_ == UNDEFINED; }
	bool isFunctionName() const { return type_ == FUNCTIONNAME; }
	bool isVariable() const { return type_ == VARIABLE; }
	bool isPattern() const { return type_ == PATTERN; }

	bool match(const std::string& str, bool literalMatch = false) const;

	void operator=(const XFoam_KeyType&);
	void operator=(const XFoam_FunctionName&);
	void operator=(const XFoam_Variable&);
	void operator=(const XFoam_Word&);
	void operator=(const char*);
	void operator=(const std::string&);
	void operator=(const XFoam_Token&);

	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream&, XFoam_KeyType&);
	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_KeyType&);
};

XFoam_API XFoam_OStream& XFoam_writeKeyword(XFoam_OStream& os, const XFoam_KeyType& kw);

class XFoam_Dictionary;

struct XFoam_API XFoam_SHA1Digest
{
	XFoam_SHA1Digest() = default;
};
// XFoam_WordList：typedef 见 xfoam_list.h
// XFoam_DLList：与 OpenFOAM 一致，为 XFoam_LList<XFoam_DLListBase, T>（定义见 xfoam_list.h）。
#define XFoam_ClassName(TypeNameString) \
    static const char* typeName_() { return TypeNameString; }

// 对标 OpenFOAM db/IOstreams/Tstreams/ITstream.H：公开继承 Istream + tokenList。
class XFoam_API XFoam_ITstream
	: public XFoam_IStream,
	  public XFoam_TokenList
{
	XFoam_FileName name_;
	XFoam_Label tokenIndex_{0};

public:
	XFoam_ITstream();

	XFoam_ITstream(const XFoam_ITstream& its);

	void operator=(const XFoam_ITstream& its);

	XFoam_ITstream(
		const XFoam_FileName& streamName,
		const XFoam_UList<XFoam_Token>& tokens,
		XFoam_IOstream::streamFormat format = XFoam_IOstream::ASCII,
		const XFoam_IOstream::versionNumber& version = XFoam_IOstream::currentVersion,
		bool global = false);

	XFoam_ITstream(
		const XFoam_FileName& streamName,
		XFoam_TokenList&& tokens,
		XFoam_IOstream::streamFormat format = XFoam_IOstream::ASCII,
		const XFoam_IOstream::versionNumber& version = XFoam_IOstream::currentVersion,
		bool global = false);

	XFoam_ITstream(const XFoam_KeyType& keywordForName, const XFoam_UList<XFoam_Token>& tokens);
	XFoam_ITstream(const XFoam_KeyType& keywordForName, XFoam_TokenList&& tokens);
	// 与 OpenFOAM ITstream(key, tokenList(10)) 对齐：预分配 token 槽位。
	XFoam_ITstream(const XFoam_KeyType& keywordForName, XFoam_Label reservedTokens);

	~XFoam_ITstream() override;

	const XFoam_FileName& name() const override { return name_; }

	XFoam_FileName& name() override { return name_; }

	XFoam_Label tokenIndex() const { return tokenIndex_; }

	XFoam_Label& tokenIndex() { return tokenIndex_; }

	XFoam_Label nRemainingTokens() const { return size() - tokenIndex_; }

	std::ios_base::fmtflags flags() const override;

	std::ios_base::fmtflags flags(const std::ios_base::fmtflags) override;

	XFoam_IStream& read(XFoam_Token& t) override;

	XFoam_IStream& read(char& c) override;

	XFoam_IStream& read(XFoam_Word& w) override;

	XFoam_IStream& read(XFoam_String& s) override;

	XFoam_IStream& read(int32_t& v) override;

	XFoam_IStream& read(int64_t& v) override;

	XFoam_IStream& read(uint32_t& v) override;

	XFoam_IStream& read(uint64_t& v) override;

	XFoam_IStream& read(float& v) override;

	XFoam_IStream& read(double& v) override;

	XFoam_IStream& read(long double& v) override;

	XFoam_IStream& read(char* data, XFoam_StreamSize count) override;

	XFoam_IStream& rewind() override;

	int peek() override;

	XFoam_IStream& get(char& c) override;

	XFoam_IStream& get() override;

	XFoam_IStream& putback(const char c) override;

	void print(XFoam_OStream& os) const override;
};

// ---- XFoam_Entry (OpenFOAM entry.H) ----

class XFoam_API XFoam_Entry
	: public XFoam_DLListBase::link
{
private:
	XFoam_KeyType keyword_;
	XFoam_Label startLineNumber_;

	static bool getKeyword(XFoam_KeyType& keyword, XFoam_Token& keywordToken, XFoam_Label& keywordLineNo, XFoam_IStream& is);
	static bool getKeyword(XFoam_KeyType& keyword, XFoam_Label& keywordLineNo, XFoam_IStream& is);

public:
	static int disableFunctionEntries;

	explicit XFoam_Entry(const XFoam_KeyType& keyword, const XFoam_Label lineNumber = -1);

	XFoam_Entry(const XFoam_Entry& e);

	virtual XFoam_AutoPtr<XFoam_Entry> clone(const XFoam_Dictionary& parentDict) const = 0;

	virtual XFoam_AutoPtr<XFoam_Entry> clone() const;

	// 未移植：OF entry::New 与 dictionaryEntry/primitiveEntry/token 管线；恒 false / 空指针。
	static bool New(XFoam_Dictionary& parentDict, XFoam_IStream& is);

	static XFoam_AutoPtr<XFoam_Entry> New(XFoam_IStream& is);

	virtual ~XFoam_Entry() {}

	const XFoam_KeyType& keyword() const { return keyword_; }

	XFoam_KeyType& keyword() { return keyword_; }

	virtual const XFoam_FileName& name() const = 0;

	virtual XFoam_FileName& name() = 0;

	virtual XFoam_Label startLineNumber() const { return startLineNumber_; }

	virtual XFoam_Label& startLineNumber() { return startLineNumber_; }

	virtual XFoam_Label endLineNumber() const = 0;

	virtual bool isStream() const { return false; }

	virtual XFoam_ITstream& stream() const = 0;

	virtual bool isDict() const { return false; }

	virtual const XFoam_Dictionary& dict() const = 0;

	virtual XFoam_Dictionary& dict() = 0;

	virtual void write(XFoam_OStream& os) const = 0;

	virtual bool read(const XFoam_Dictionary& dict, XFoam_IStream& is);

	void operator=(const XFoam_Entry& e);

	bool operator==(const XFoam_Entry& e) const;

	bool operator!=(const XFoam_Entry& e) const;

	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Entry& e);
};

typedef XFoam_PtrList<XFoam_Entry> XFoam_EntryList;

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Entry& e);

// ---- end XFoam_Entry ----

/*---------------------------------------------------------------------------*\
                       Class XFoam_PrimitiveEntry Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_PrimitiveEntry
	: public XFoam_Entry,
	  public XFoam_ITstream
{
	void append(const XFoam_UList<XFoam_Token>&);
	void append(const XFoam_Token& currToken, const XFoam_Dictionary& dict, XFoam_IStream& is);
	bool expandVariable(const XFoam_Variable& w, const XFoam_Dictionary& dict);
	bool expandFunction(const XFoam_FunctionName& hashFn, const XFoam_Dictionary& dict, XFoam_IStream& is);
	void readEntry(const XFoam_Dictionary& dict, XFoam_IStream& is);

public:
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, XFoam_IStream& is);
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_Dictionary& parentDict, XFoam_IStream& is);
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_ITstream& is);
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_Token& t);
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, const XFoam_UList<XFoam_Token>& tokens);
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, XFoam_TokenList&& tokens);

	template<class T>
	XFoam_PrimitiveEntry(const XFoam_KeyType& key, const T& t);

	template<class T>
	XFoam_PrimitiveEntry
	(
		const XFoam_KeyType& key,
		const T& t,
		const XFoam_Label startLineNumber,
		const XFoam_Label endLineNumber = -1
	);

	XFoam_AutoPtr<XFoam_Entry> clone(const XFoam_Dictionary& parentDict) const
	{
		(void)parentDict;
		return XFoam_AutoPtr<XFoam_Entry>(new XFoam_PrimitiveEntry(*this));
	}

	const XFoam_FileName& name() const override { return XFoam_ITstream::name(); }

	XFoam_FileName& name() override { return XFoam_ITstream::name(); }

	XFoam_Label endLineNumber() const override;

	bool isStream() const override { return true; }

	XFoam_ITstream& stream() const override;

	const XFoam_Dictionary& dict() const override;

	XFoam_Dictionary& dict() override;

	virtual bool read(const XFoam_Dictionary& dict, XFoam_IStream& is);

	void write(XFoam_OStream& os) const override;

	void write(XFoam_OStream& os, const bool contentsOnly) const;

	XFoam_InfoProxy<XFoam_PrimitiveEntry> info() const { return XFoam_InfoProxy<XFoam_PrimitiveEntry>(*this); }
};

template<class T>
XFoam_PrimitiveEntry::XFoam_PrimitiveEntry(const XFoam_KeyType& key, const T& t)
	: XFoam_Entry(key)
	, XFoam_ITstream(key, XFoam_Label(10))
{
	std::ostringstream os;
	os << t << static_cast<char>(XFoam_Token::END_STATEMENT);
	std::istringstream iss(os.str());
	XFoam_ISstream is(iss, XFoam_String("PrimitiveEntry.template"));
	readEntry(XFoam_Dictionary::null, is);
}

template<class T>
XFoam_PrimitiveEntry::XFoam_PrimitiveEntry
(
	const XFoam_KeyType& key,
	const T& t,
	const XFoam_Label startLineNumber,
	const XFoam_Label endLineNumber
)
	: XFoam_Entry(key, startLineNumber)
	, XFoam_ITstream(key, XFoam_Label(10))
{
	std::ostringstream os;
	os << t << static_cast<char>(XFoam_Token::END_STATEMENT);
	std::istringstream iss(os.str());
	// XFoam 未实现：Foam::IStringStream::lineNumber() 与 OF 完全一致赋值；此处忽略 endLineNumber。
	(void)endLineNumber;
	XFoam_ISstream is(iss, XFoam_String("PrimitiveEntry.template"));
	readEntry(XFoam_Dictionary::null, is);
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_InfoProxy<XFoam_PrimitiveEntry>& ip);

XFoam_API XFoam_IStream& operator>>(XFoam_IStream&, XFoam_Dictionary&);
XFoam_API XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_Dictionary&);

// FatalError 等通过 std::ostringstream 拼接；Dictionary 仅有 XFoam_OStream 的 operator<<。
inline std::ostream& operator<<(std::ostream& os, const XFoam_Dictionary& d)
{
	XFoam_OStringStream oss;
	oss << d;
	return os << oss.str();
}

/*---------------------------------------------------------------------------*\
                       Class XFoam_DictionaryName Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_DictionaryName
{
    // Private Data

        XFoam_FileName name_;


public:

    // Constructors

        //- Construct XFoam_DictionaryName null
        XFoam_DictionaryName()
        {}

        //- Construct XFoam_DictionaryName as copy of the given XFoam_FileName
        XFoam_DictionaryName(const XFoam_FileName& name)
        :
            name_(name)
        {}

        //- Move constructor
        XFoam_DictionaryName(XFoam_DictionaryName&& name)
        :
            name_(XFoam_move(name.name_))
        {}


    // Member Functions

        //- Return the XFoam_Dictionary name
        const XFoam_FileName& name() const
        {
            return name_;
        }

        //- Return the XFoam_Dictionary name
        XFoam_FileName& name()
        {
            return name_;
        }

        //- Return the local XFoam_Dictionary name (final part of scoped name)
        const XFoam_Word dictName() const
        {
            const XFoam_Word scopedName = name_.name();

            XFoam_String::size_type i = scopedName.rfind('/');

            if (i == scopedName.npos)
            {
                return scopedName;
            }
            else
            {
                return XFoam_Word(scopedName.substr(i + 1, scopedName.npos));
            }
        }


    // Member Operators

        void operator=(const XFoam_DictionaryName& name)
        {
            name_ = name.name_;
        }

        void operator=(XFoam_DictionaryName&& name)
        {
            name_ = XFoam_move(name.name_);
        }
};


/*---------------------------------------------------------------------------*\
                         Class XFoam_Dictionary Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Dictionary
:
    public XFoam_DictionaryName,
    public XFoam_IDLList<XFoam_Entry>
{
    // Private Data

        //- XFoam_HashTable of the entries held on the DL-list for quick lookup
        XFoam_HashTable<XFoam_Entry*> hashedEntries_;

        //- Parent XFoam_Dictionary
        const XFoam_Dictionary& parent_;

        //- Current stream/file pointer
        mutable const XFoam_IStream* filePtr_;

        //- Entries of matching patterns
        XFoam_DLList<XFoam_Entry*> patternEntries_;

        //- Patterns as precompiled regular expressions
        XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>> patternRegexps_;


    // Private Member Functions

        //- Find and return an XFoam_Entry data stream pointer if present
        //  otherwise return nullptr.
        //  Allows scoping using '/' with special handling for '!' and '..'.
        const XFoam_Entry* lookupScopedSubEntryPtr
        (
            const XFoam_Word&,
            bool recursive,
            bool patternMatch
        ) const;

        //- Search patterns table for exact match or regular expression match
        bool findInPatterns
        (
            const bool patternMatch,
            const XFoam_Word& Keyword,
            XFoam_DLList<XFoam_Entry*>::const_iterator& wcLink,
            XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::const_iterator& reLink
        ) const;

        //- Search patterns table for exact match or regular expression match
        bool findInPatterns
        (
            const bool patternMatch,
            const XFoam_Word& Keyword,
            XFoam_DLList<XFoam_Entry*>::iterator& wcLink,
            XFoam_DLList<XFoam_AutoPtr<XFoam_RegExp>>::iterator& reLink
        );

        //- Check that no unit conversions are being performed
        void assertNoConvertUnits
        (
            const char* typeName,
            const XFoam_Word& keyword,
            const XFoam_UnitConversion& defaultUnits,
            XFoam_ITstream& is
        ) const;

        //- Read a value, check its dimensions and convert its units
        template<class T>
        T readTypeAndConvertUnits
        (
            const XFoam_Word& keyword,
            const XFoam_UnitConversion& defaultUnits,
            XFoam_ITstream& is
        ) const;

        //- Read a value from the token stream
        template<class T>
        T readType
        (
            const XFoam_Word& keyword,
            const XFoam_UnitConversion& defaultUnits,
            XFoam_ITstream& is
        ) const;

        //- Read a value from the token stream
        template<class T>
        T readType(const XFoam_Word& keyword, XFoam_ITstream& is) const;

        //- Assign multiple entries, overwriting any existing entries
        template<class ... Entries, size_t ... Indices>
        void set
        (
            const std::tuple<const Entries& ...>&,
            const std::integer_sequence<size_t, Indices ...>&
        );

        //- Assign multiple entries, overwriting any existing entries
        template<class ... Entries>
        void set(const std::tuple<const Entries& ...>&);


    // Private Classes

        class includedDictionary;


public:

    //- Declare friendship with the XFoam_Entry class for IO
    friend class XFoam_Entry;


    // Declare name of the class and its debug switch
    XFoam_ClassName("XFoam_Dictionary");


    // Public static data

        //- Null XFoam_Dictionary
        static const XFoam_Dictionary null;

        //- If true write optional keywords and values
        //  if not present in XFoam_Dictionary
        static int writeOptionalEntries;


    // Static Member Functions

        //- Construct an entries tuple from which to make a XFoam_Dictionary
        template<class ... Entries>
        static std::tuple<const Entries& ...> entries(const Entries& ...);


    // Constructors

        //- Construct top-level XFoam_Dictionary null
        XFoam_Dictionary();

        //- Construct top-level empty XFoam_Dictionary with given name
        XFoam_Dictionary(const XFoam_FileName& name);

        //- Construct an empty sub-XFoam_Dictionary with given name and parent
        XFoam_Dictionary(const XFoam_FileName& name, const XFoam_Dictionary& parentDict);

        //- Construct given the name, parent XFoam_Dictionary and XFoam_IStream,
        //  reading entries until lastEntry or EOF
        XFoam_Dictionary
        (
            const XFoam_FileName& name,
            const XFoam_Dictionary& parentDict,
            XFoam_IStream&
        );

        //- Construct top-level XFoam_Dictionary from XFoam_IStream,
        //  reading entries until EOF, optionally keeping the header
        XFoam_Dictionary(XFoam_IStream&, const bool keepHeader=false);

        //- Construct as copy given the parent XFoam_Dictionary
        XFoam_Dictionary(const XFoam_Dictionary& parentDict, const XFoam_Dictionary&);

        //- Construct top-level XFoam_Dictionary as copy
        XFoam_Dictionary(const XFoam_Dictionary&);

		XFoam_Dictionary(XFoam_Dictionary&&) = default;

		XFoam_Dictionary& operator=(XFoam_Dictionary&&) = default;

        //- Construct top-level XFoam_Dictionary as copy from pointer to XFoam_Dictionary.
        //  A null pointer is treated like an empty XFoam_Dictionary.
        XFoam_Dictionary(const XFoam_Dictionary*);

        //- Construct top-level XFoam_Dictionary with given entries
        template<class ... Entries>
        XFoam_Dictionary(const std::tuple<const Entries& ...>&);

        //- Construct top-level XFoam_Dictionary with given name and entries
        template<class ... Entries>
        XFoam_Dictionary
        (
            const XFoam_FileName& name,
            const std::tuple<const Entries& ...>&
        );

        //- Construct XFoam_Dictionary with given name, parent and entries
        template<class ... Entries>
        XFoam_Dictionary
        (
            const XFoam_FileName& name,
            const XFoam_Dictionary& parentDict,
            const std::tuple<const Entries& ...>&
        );

        //- Construct XFoam_Dictionary as copy and add a list of entries
        template<class ... Entries>
        XFoam_Dictionary
        (
            const XFoam_Dictionary& dict,
            const std::tuple<const Entries& ...>&
        );

        //- Construct and return clone
        XFoam_AutoPtr<XFoam_Dictionary> clone() const;

        //- Construct top-level XFoam_Dictionary on freestore from XFoam_IStream
        static XFoam_AutoPtr<XFoam_Dictionary> New(XFoam_IStream&);


    //- Destructor
    virtual ~XFoam_Dictionary();


    // Member Functions

        //- Return the parent XFoam_Dictionary
        const XFoam_Dictionary& parent() const
        {
            return parent_;
        }

        //- Return whether this XFoam_Dictionary is null
        bool isNull() const
        {
            return this == &null;
        }

        //- Return the top of the tree
        const XFoam_Dictionary& topDict() const;

        //- Return the scoped keyword with which this XFoam_Dictionary can be
        //  accessed from the top XFoam_Dictionary in the tree
        XFoam_Word topDictKeyword() const;

        //- Return line number of first token in XFoam_Dictionary
        virtual XFoam_Label startLineNumber() const;

        //- Return line number of last token in XFoam_Dictionary
        virtual XFoam_Label endLineNumber() const;

        //- Return the SHA1 digest of the XFoam_Dictionary contents
        XFoam_SHA1Digest digest() const;

        //- Return the XFoam_Dictionary as a list of tokens
        XFoam_TokenList tokens() const;


        // Search and lookup

            //- Search XFoam_Dictionary for given keyword
            //  If recursive, search parent dictionaries
            //  If patternMatch, use regular expressions
            bool found
            (
                const XFoam_Word&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return an XFoam_Entry data stream pointer if present
            //  otherwise return nullptr.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions
            const XFoam_Entry* lookupEntryPtr
            (
                const XFoam_Word&,
                bool recursive,
                bool patternMatch
            ) const;

            //- Find and return an XFoam_Entry data stream pointer for manipulation
            //  if present otherwise return nullptr.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            XFoam_Entry* lookupEntryPtr
            (
                const XFoam_Word&,
                bool recursive,
                bool patternMatch
            );

            //- Find and return an XFoam_Entry data stream if present, trying a list
            //  of keywords in sequence, otherwise return nullptr.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions
            const XFoam_Entry* lookupEntryPtrBackwardsCompatible
            (
                const XFoam_WordList&,
                bool recursive,
                bool patternMatch
            ) const;

            //- Find and return an XFoam_Entry data stream if present otherwise error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            const XFoam_Entry& lookupEntry
            (
                const XFoam_Word&,
                bool recursive,
                bool patternMatch
            ) const;

            //- Find and return an XFoam_Entry data stream if present, trying a list
            //  of keywords in sequence, otherwise error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions
            const XFoam_Entry& lookupEntryBackwardsCompatible
            (
                const XFoam_WordList&,
                bool recursive,
                bool patternMatch
            ) const;

            //- Find and return an XFoam_Entry data stream
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            XFoam_ITstream& lookup
            (
                const XFoam_Word&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return an XFoam_Entry data stream, trying a list of keywords
            //  in sequence
            //  if not found throw a fatal error relating to the first keyword
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            XFoam_ITstream& lookupBackwardsCompatible
            (
                const XFoam_WordList&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return a T, if not found throw a fatal error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookup
            (
                const XFoam_Word&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return a T, with dimension checking and unit
            //  conversions, and if not found throw a fatal error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookup
            (
                const XFoam_Word&,
                const XFoam_UnitConversion&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return a T, trying a list of keywords in sequence,
            //  and if not found throw a fatal error relating to the first
            //  (preferred) keyword.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupBackwardsCompatible
            (
                const XFoam_WordList&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return a T, with dimension checking and unit
            //  conversions, trying a list of keywords in sequence, and if not
            //  found throw a fatal error relating to the first (preferred)
            //  keyword.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupBackwardsCompatible
            (
                const XFoam_WordList&,
                const XFoam_UnitConversion&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return a T, if not found return the given default
            //  value.
            template<class T>
            T lookupOrDefault
            (
                const XFoam_Word&,
                const T&,
                const bool writeDefault = writeOptionalEntries > 0
            ) const;

            //- Find and return a T with dimension checking and unit
            //  conversions, and if not found return the given default value.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupOrDefault
            (
                const XFoam_Word&,
                const XFoam_UnitConversion&,
                const T&,
                const bool writeDefault = writeOptionalEntries > 0
            ) const;

            //- Find and return a T, trying a list of keywords in sequence,
            //  and if not found throw a fatal error relating to the first
            //  (preferred) keyword
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupOrDefaultBackwardsCompatible
            (
                const XFoam_WordList&,
                const T&
            ) const;

            //- Find and return a T, with dimension checking and unit
            //  conversions, trying a list of keywords in sequence, and if not
            //  found throw a fatal error relating to the first (preferred)
            //  keyword
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupOrDefaultBackwardsCompatible
            (
                const XFoam_WordList&,
                const XFoam_UnitConversion&,
                const T&
            ) const;

            //- Find and return a T, if not found return the given
            //  default value, and add to XFoam_Dictionary.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            T lookupOrAddDefault
            (
                const XFoam_Word&,
                const T&
            );

            //- Find an XFoam_Entry if present, and assign to T.
            //  Returns true if the XFoam_Entry was found.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            bool readIfPresent
            (
                const XFoam_Word&,
                T&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find an XFoam_Entry if present, and assign to T, with dimension
            //  checking and unit conversions.
            //  Returns true if the XFoam_Entry was found.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            template<class T>
            bool readIfPresent
            (
                const XFoam_Word&,
                const XFoam_UnitConversion&,
                T&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find and return an XFoam_Entry data stream pointer if present,
            //  otherwise return nullptr.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            //  Allows scoping using '/' with special handling for '!' and '..'.
            const XFoam_Entry* lookupScopedEntryPtr
            (
                const XFoam_Word&,
                bool recursive,
                bool patternMatch
            ) const;

            //- Find and return a T,
            //  if not found throw a fatal error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            //  Allows scoping using '/' with special handling for '!' and '..'.
            template<class T>
            T lookupScoped
            (
                const XFoam_Word&,
                bool recursive=false,
                bool patternMatch=true
            ) const;

            //- Find return the reference to the compound T,
            //  if not found or not a compound throw a fatal error.
            //  If recursive, search parent dictionaries.
            //  If patternMatch, use regular expressions.
            //  Allows scoping using '/' with special handling for '!' and '..'.
            template<class T>
            const T& lookupCompoundScoped
            (
                const XFoam_Word& keyword,
                bool recursive,
                bool patternMatch
            ) const;

            //- Check if XFoam_Entry is a sub-XFoam_Dictionary
            bool isDict(const XFoam_Word&) const;

            //- Find and return a sub-XFoam_Dictionary pointer if present
            //  otherwise return nullptr.
            const XFoam_Dictionary* subDictPtr(const XFoam_Word&) const;

            //- Find and return a sub-XFoam_Dictionary pointer if present
            //  otherwise return nullptr.
            XFoam_Dictionary* subDictPtr(const XFoam_Word&);

            //- Find and return a sub-XFoam_Dictionary
            const XFoam_Dictionary& subDict(const XFoam_Word&) const;

            //- Find and return a sub-XFoam_Dictionary for manipulation
            XFoam_Dictionary& subDict(const XFoam_Word&);

            //- Find and return a sub-XFoam_Dictionary, trying a list of keywords in
            //  sequence, otherwise error.
            const XFoam_Dictionary& subDictBackwardsCompatible(const XFoam_WordList&) const;

            //- Find and return a sub-XFoam_Dictionary
            //  or empty XFoam_Dictionary if the sub-XFoam_Dictionary does not exist
            const XFoam_Dictionary& subOrEmptyDict
            (
                const XFoam_Word&,
                const bool mustRead = false
            ) const;

            //- Find and return a sub-XFoam_Dictionary if found
            //  otherwise return this XFoam_Dictionary
            const XFoam_Dictionary& optionalSubDict(const XFoam_Word&) const;

            //- Find and return a sub-XFoam_Dictionary by scoped lookup
            //  i.e. the keyword may contain scope characters.
            //  If the keyword is null this XFoam_Dictionary is returned
            const XFoam_Dictionary& scopedDict(const XFoam_Word&) const;

            //- Find and return a sub-XFoam_Dictionary by scoped lookup
            //  i.e. the keyword may contain scope characters.
            //  If the keyword is null this XFoam_Dictionary is returned
            XFoam_Dictionary& scopedDict(const XFoam_Word&);

            //- Return the table of contents
            XFoam_WordList toc() const;

            //- Return the sorted table of contents
            XFoam_WordList sortedToc() const;

            //- Return the list of available keys or patterns
            XFoam_List<XFoam_KeyType> keys(bool patterns=false) const;


        // Editing

            //- Substitute the given keyword prepended by '$' with the
            //  corresponding sub-XFoam_Dictionary entries
            bool substituteKeyword(const XFoam_Word& keyword);

            //- Add a new XFoam_Entry
            //  With the merge option, dictionaries are interwoven and
            //  primitive entries are overwritten
            bool add(XFoam_Entry*, bool mergeEntry=false);

            //- Add an XFoam_Entry
            //  With the merge option, dictionaries are interwoven and
            //  primitive entries are overwritten
            void add(const XFoam_Entry&, bool mergeEntry=false);

            //- Add a XFoam_Word XFoam_Entry
            //  optionally overwrite an existing XFoam_Entry
            void add(const XFoam_KeyType&, const XFoam_Word&, bool overwrite=false);

            //- Add a XFoam_String XFoam_Entry
            //  optionally overwrite an existing XFoam_Entry
            void add(const XFoam_KeyType&, const XFoam_String&, bool overwrite=false);

            //- Add a XFoam_Label XFoam_Entry
            //  optionally overwrite an existing XFoam_Entry
            void add(const XFoam_KeyType&, const XFoam_Label, bool overwrite=false);

            //- Add a XFoam_Scalar XFoam_Entry
            //  optionally overwrite an existing XFoam_Entry
            void add(const XFoam_KeyType&, const XFoam_Scalar, bool overwrite=false);

            //- Add a XFoam_Dictionary XFoam_Entry
            //  optionally merge with an existing sub-XFoam_Dictionary
            void add
            (
                const XFoam_KeyType&,
                const XFoam_Dictionary&,
                bool mergeEntry=false
            );

            //- Add a T XFoam_Entry
            //  optionally overwrite an existing XFoam_Entry
            template<class T>
            void add(const XFoam_KeyType&, const T&, bool overwrite=false);

            //- Assign a new XFoam_Entry, overwrite any existing XFoam_Entry
            void set(XFoam_Entry*);

            //- Assign a new XFoam_Entry, overwrite any existing XFoam_Entry
            void set(const XFoam_Entry&);

            //- Assign a XFoam_Dictionary XFoam_Entry, overwrite any existing XFoam_Entry
            void set(const XFoam_KeyType&, const XFoam_Dictionary&);

            //- Assign a T XFoam_Entry, overwrite any existing XFoam_Entry
            template<class T>
            void set(const XFoam_KeyType&, const T&);

            //- Assign multiple entries, overwriting any existing entries
            template<class ... Entries>
            void set(const XFoam_Entry& e, const Entries& ...);

            //- Assign multiple T entries, overwriting any existing entries
            template<class T, class ... Entries>
            void set(const XFoam_KeyType&, const T&, const Entries& ...);

            //- Remove an XFoam_Entry specified by keyword
            bool remove(const XFoam_Word&);

            //- Remove entries specified by keywords
            void remove(const XFoam_WordList&);

            //- Change the keyword for an XFoam_Entry,
            //  optionally forcing overwrite of an existing XFoam_Entry
            bool changeKeyword
            (
                const XFoam_KeyType& oldKeyword,
                const XFoam_KeyType& newKeyword,
                bool forceOverwrite=false
            );

            //- Merge entries from the given XFoam_Dictionary.
            //  Also merge sub-dictionaries as required.
            bool merge(const XFoam_Dictionary&);

            //- Clear the XFoam_Dictionary
            void clear();

            //- Transfer the contents of the argument and annul the argument.
            void transfer(XFoam_Dictionary&);


        // Read

            //- Read XFoam_Dictionary from XFoam_IStream, optionally keeping the header
            bool read(XFoam_IStream&, const bool keepHeader=false);

            //- Return true if the XFoam_Dictionary global,
            //  i.e. the same on all processors.
            //  Defaults to false, must be overridden by global IO dictionaries
            virtual bool global() const;


        // Write

            //- Write XFoam_Dictionary, normally with sub-XFoam_Dictionary formatting
            void write(XFoam_OStream&, const bool subDict=true) const;


    // Member Operators

        //- Find and return XFoam_Entry
        XFoam_ITstream& operator[](const XFoam_Word&) const;

        void operator=(const XFoam_Dictionary&);

        //- Include entries from the given XFoam_Dictionary.
        //  Warn, but do not overwrite existing entries.
        void operator+=(const XFoam_Dictionary&);

        //- Conditionally include entries from the given XFoam_Dictionary.
        //  Do not overwrite existing entries.
        void operator|=(const XFoam_Dictionary&);

        //- Unconditionally include entries from the given XFoam_Dictionary.
        //  Overwrite existing entries.
        void operator<<=(const XFoam_Dictionary&);


    // IOstream Operators

        //- Read XFoam_Dictionary from XFoam_IStream
        friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream&, XFoam_Dictionary&);

        //- Write XFoam_Dictionary to XFoam_OStream
        friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_Dictionary&);
};

// 模板成员定义（对标 OpenFOAM db/dictionary/dictionaryTemplates.C）：readType / lookupOrDefault / readIfPresent
// 按「移植 OpenFOAM 规范」不另建碎片头文件，集中在本头文件。

template<class T>
inline T XFoam_Dictionary::readType(const XFoam_Word& keyword, XFoam_ITstream& is) const
{
	(void)keyword;
	T val{};
	is >> val;
	return val;
}

template<>
inline XFoam_Label XFoam_Dictionary::readType(const XFoam_Word& keyword, XFoam_ITstream& is) const
{
	(void)keyword;
	int32_t v = 0;
	is >> v;
	return static_cast<XFoam_Label>(v);
}

template<>
inline bool XFoam_Dictionary::readType(const XFoam_Word& keyword, XFoam_ITstream& is) const
{
	(void)keyword;
	XFoam_Switch sw;
	is >> sw;
	return static_cast<bool>(sw);
}

template<class T>
inline T XFoam_Dictionary::readType(
	const XFoam_Word& keyword,
	const XFoam_UnitConversion& defaultUnits,
	XFoam_ITstream& is) const
{
	assertNoConvertUnits("T", keyword, defaultUnits, is);
	return readType<T>(keyword, is);
}

// * * * * * * * * * * * * * * * * lookupOrDefault * * * * * * * * * * * * * //

template<class T>
inline T XFoam_Dictionary::lookupOrDefault(
	const XFoam_Word& keyword,
	const T& defaultValue,
	const bool writeDefault) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, false);
	if (entryPtr)
	{
		return readType<T>(keyword, entryPtr->stream());
	}
	(void)writeDefault;
	return defaultValue;
}

template<class T>
inline T XFoam_Dictionary::lookupOrDefault(
	const XFoam_Word& keyword,
	const XFoam_UnitConversion& defaultUnits,
	const T& defaultValue,
	const bool writeDefault) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, false, false);
	if (entryPtr)
	{
		return readType<T>(keyword, defaultUnits, entryPtr->stream());
	}
	(void)writeDefault;
	return defaultValue;
}

// * * * * * * * * * * * * * * * * readIfPresent * * * * * * * * * * * * * * //

template<class T>
inline bool XFoam_Dictionary::readIfPresent(
	const XFoam_Word& keyword,
	T& val,
	const bool recursive,
	const bool patternMatch) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, recursive, patternMatch);
	if (entryPtr)
	{
		val = readType<T>(keyword, entryPtr->stream());
		return true;
	}
	return false;
}

template<class T>
inline bool XFoam_Dictionary::readIfPresent(
	const XFoam_Word& keyword,
	const XFoam_UnitConversion& defaultUnits,
	T& val,
	const bool recursive,
	const bool patternMatch) const
{
	const XFoam_Entry* entryPtr = lookupEntryPtr(keyword, recursive, patternMatch);
	if (entryPtr)
	{
		val = readType<T>(keyword, defaultUnits, entryPtr->stream());
		return true;
	}
	(void)defaultUnits;
	return false;
}

template<class... Entries>
inline std::tuple<const Entries&...> XFoam_Dictionary::entries(const Entries&... es)
{
	return std::tie(es...);
}


// Private Classes

class XFoam_Dictionary::includedDictionary
:
    public XFoam_Dictionary
{
    // Private Data

        //- Global IO status inherited from the parent XFoam_Dictionary
        bool global_;


public:

    // Constructors

        //- Construct an included XFoam_Dictionary for the given parent
        //  setting the "global" status XFoam_Dictionary without setting the parent
        includedDictionary
        (
            const XFoam_FileName& fName,
            const XFoam_Dictionary& parentDict
        );


    //- Destructor
    virtual ~includedDictionary()
    {}


    // Member Functions

        //- Return true if the XFoam_Dictionary global,
        //  i.e. the same on all processors.
        //  Inherited from the parent XFoam_Dictionary into which this is included
        virtual bool global() const
        {
            return global_;
        }
};


// Template Specialisations

//- Specialise readType for types for which unit conversions can be performed
#define DECLARE_SPECIALISED_READ_TYPE(T, nullArg)                              \
                                                                               \
    template<>                                                                 \
    T XFoam_Dictionary::readType                                                      \
    (                                                                          \
        const XFoam_Word& keyword,                                                   \
        const XFoam_UnitConversion& defaultUnits,                                    \
        XFoam_ITstream& is                                                           \
    ) const;                                                                   \
                                                                               \
    template<>                                                                 \
    T XFoam_Dictionary::readType                                                      \
    (                                                                          \
        const XFoam_Word& keyword,                                                   \
        XFoam_ITstream& is                                                           \
    ) const;

#define DECLARE_SPECIALISED_READ_PAIR_TYPE(T, nullArg)                         \
    DECLARE_SPECIALISED_READ_TYPE(XFoam_Pair<T>, nullArg)

#define DECLARE_SPECIALISED_READ_LIST_TYPE(T, nullArg)                         \
    DECLARE_SPECIALISED_READ_TYPE(XFoam_List<T>, nullArg)

FOR_ALL_FIELD_TYPES(DECLARE_SPECIALISED_READ_TYPE)
FOR_ALL_FIELD_TYPES(DECLARE_SPECIALISED_READ_PAIR_TYPE)
FOR_ALL_FIELD_TYPES(DECLARE_SPECIALISED_READ_LIST_TYPE)

#undef DECLARE_SPECIALISED_READ_TYPE
#undef DECLARE_SPECIALISED_READ_PAIR_TYPE
#undef DECLARE_SPECIALISED_READ_LIST_TYPE


// Global Operators

//- Combine dictionaries.
//  Starting from the entries in dict1 and then including those from dict2.
//  Warn, but do not overwrite the entries from dict1.
XFoam_API XFoam_Dictionary operator+(const XFoam_Dictionary& dict1, const XFoam_Dictionary& dict2);

//- Combine dictionaries.
//  Starting from the entries in dict1 and then including those from dict2.
//  Do not overwrite the entries from dict1.
XFoam_API XFoam_Dictionary operator|(const XFoam_Dictionary& dict1, const XFoam_Dictionary& dict2);


// Global Functions

//- Parse XFoam_Dictionary substitution argument list
XFoam_API void XFoam_dictArgList
(
    const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
    XFoam_Word& configName,
    XFoam_List<XFoam_Tuple2<XFoam_WordRe, XFoam_Label>>& args,
    XFoam_List<XFoam_Tuple3<XFoam_Word, XFoam_String, XFoam_Label>>& namedArgs
);

//- Parse XFoam_Dictionary substitution argument list
XFoam_API void XFoam_dictArgList
(
    const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
    XFoam_List<XFoam_Tuple2<XFoam_WordRe, XFoam_Label>>& args,
    XFoam_List<XFoam_Tuple3<XFoam_Word, XFoam_String, XFoam_Label>>& namedArgs
);

//- Extracts dict name and keyword
XFoam_API XFoam_Pair<XFoam_Word> XFoam_dictAndKeyword(const XFoam_Word& scopedName);

//- Return the list of configuration files in
//  user/group/shipped directories.
//  The search scheme allows for version-specific and
//  version-independent files using the following hierarchy:
//  - \b user settings:
//    - ~/.OpenFOAM/\<VERSION\>/caseDicts/functions
//    - ~/.OpenFOAM/caseDicts/functions
//  - \b group (site) settings (when $WM_PROJECT_SITE is set):
//    - $WM_PROJECT_SITE/\<VERSION\>/etc/caseDicts/functions
//    - $WM_PROJECT_SITE/etc/caseDicts/functions
//  - \b group (site) settings (when $WM_PROJECT_SITE is not set):
//    - $WM_PROJECT_INST_DIR/site/\<VERSION\>/etc/
//          caseDicts/functions
//    - $WM_PROJECT_INST_DIR/site/etc/caseDicts/functions
//  - \b other (shipped) settings:
//    - $WM_PROJECT_DIR/etc/caseDicts/functions
XFoam_API XFoam_WordList XFoam_listAllConfigFiles
(
    const XFoam_FileName& configFilesPath
);

//- Search for configuration file for given region
//  and if not present also search the case directory as well as the
//  user/group/shipped directories.
//  The search scheme allows for version-specific and
//  version-independent files using the following hierarchy:
//  - \b user settings:
//    - ~/.OpenFOAM/\<VERSION\>/caseDicts/functions
//    - ~/.OpenFOAM/caseDicts/functions
//  - \b group (site) settings (when $WM_PROJECT_SITE is set):
//    - $WM_PROJECT_SITE/\<VERSION\>/etc/caseDicts/functions
//    - $WM_PROJECT_SITE/etc/caseDicts/functions
//  - \b group (site) settings (when $WM_PROJECT_SITE is not set):
//    - $WM_PROJECT_INST_DIR/site/\<VERSION\>/etc/
//          caseDicts/functions
//    - $WM_PROJECT_INST_DIR/site/etc/caseDicts/functions
//  - \b other (shipped) settings:
//    - $WM_PROJECT_DIR/etc/caseDicts/functions
//
//  \return The path of the configuration file if found
//  otherwise null
XFoam_API XFoam_FileName XFoam_findConfigFile
(
    const XFoam_Word& configName,
    const XFoam_FileName& configFilesPath,
    const XFoam_Word& configFilesDir,
    const XFoam_Word& region = XFoam_Word::null
);

//- Expand arg within the dict context and return
XFoam_API XFoam_String XFoam_expandArg
(
    const XFoam_String& arg,
    XFoam_Dictionary& dict,
    const XFoam_Label lineNumber
);

//- Add the keyword value pair to dict
//  setting the given lineNumber for the XFoam_Entry
XFoam_API void XFoam_addArgEntry
(
    XFoam_Dictionary& dict,
    const XFoam_Word& keyword,
    const XFoam_String& value,
    const XFoam_Label lineNumber
);

//- Read the specified configuration file
//  parsing the optional arguments included in the XFoam_String
//  'argString', inserting 'field' or 'fields' entries as required
//  and merging the resulting configuration XFoam_Dictionary into
//  'parentDict'.
//
//  Parses the optional arguments:
//      'Q(U)' -> configFileName = Q; args = (U)
//             -> field U;
//
//  Supports named arguments:
//      'patchAverage(patch=inlet, p,U)'
//  or
//      'patchAverage(patch=inlet, field=(p U))'
//       -> configFileName = patchAverage;
//          args = (patch=inlet, p,U)
//       -> patch inlet;
//          fields (p U);
XFoam_API bool XFoam_readConfigFile
(
    const XFoam_Word& configType,
    const XFoam_Tuple2<XFoam_String, XFoam_Label>& argString,
    XFoam_Dictionary& parentDict,
    const XFoam_FileName& configFilesPath,
    const XFoam_Word& configFilesDir,
    const XFoam_Word& region = XFoam_Word::null
);

//- Write a XFoam_Dictionary XFoam_Entry
XFoam_API void XFoam_writeEntry(XFoam_OStream& os, const XFoam_Dictionary& dict);

//- Helper function to write the keyword and XFoam_Entry
template<class EntryType>
void XFoam_writeEntry(XFoam_OStream& os, const XFoam_Word& entryName, const EntryType& value);

//- Helper function to write the keyword and XFoam_Entry
template<class EntryType>
void XFoam_writeEntry
(
    XFoam_OStream& os,
    const XFoam_Word& entryName,
    const XFoam_UnitConversion& defaultUnits,
    const EntryType& value
);

//- Helper function to write the keyword and XFoam_Entry only if the
//  values are not equal. The value is then output as value2
template<class EntryType>
void XFoam_writeEntryIfDifferent
(
    XFoam_OStream& os,
    const XFoam_Word& entryName,
    const EntryType& value1,
    const EntryType& value2
);

//- Helper function to write the keyword and XFoam_Entry only if the
//  values are not equal. The value is then output as value2
template<class EntryType>
void XFoam_writeEntryIfDifferent
(
    XFoam_OStream& os,
    const XFoam_Word& entryName,
    const XFoam_UnitConversion& defaultUnits,
    const EntryType& value1,
    const EntryType& value2
);


/*---------------------------------------------------------------------------*\
                        Class XFoam_IODictionary Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_IODictionary
	: public XFoam_RegIOobject,
	  public XFoam_Dictionary
{
protected:
	XFoam_IODictionary(const XFoam_IOobject& io, const XFoam_Word& wantedType);

public:
	static constexpr const char* typeName = "dictionary";

	static bool writeDictionaries;

	XFoam_IODictionary(const XFoam_IOobject& io);

	XFoam_IODictionary(const XFoam_IOobject& io, const XFoam_Dictionary& dict);

	XFoam_IODictionary(const XFoam_IOobject& io, XFoam_IStream& is);

	XFoam_IODictionary(const XFoam_IODictionary&);

	XFoam_IODictionary(XFoam_IODictionary&&);

	virtual ~XFoam_IODictionary();

	using XFoam_RegIOobject::name;

	virtual bool global() const;

	virtual bool readData(XFoam_IStream&);

	virtual bool writeData(XFoam_OStream&) const;

	void operator=(const XFoam_IODictionary&);

	void operator=(XFoam_IODictionary&&);
};

template<>
struct XFoam_TypeGlobal<XFoam_IODictionary>
{
	static const bool global = true;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// 对标 OpenFOAM systemDict.H / systemDict.C（无 argList：-dict 选项逻辑未移植）。

inline const XFoam_Word& XFoam_polyMeshDefaultRegion()
{
	static const XFoam_Word def("region0");
	return def;
}

/// \brief 由字典文件路径构造读 system 字典用的 IOobject（dictName 取路径末段，region 为 region0，Time 为模块内静态默认对象）。
/// \param path 字典文件路径（如 .../system/blockMeshDict 或绝对路径下的 blockMeshDict）
/// \return 供 XFoam_IODictionary(io) 等使用的 XFoam_IOobject
/// \note 内部调用 XFoam_systemDictIO；与四参数 XFoam_IODictionary XFoam_systemDict(...) 为重载。
XFoam_API XFoam_IOobject XFoam_systemDictIO(const XFoam_FileName& path);

XFoam_API XFoam_IODictionary XFoam_systemDict
(
	const XFoam_Word& dictName,
	const XFoam_ObjectRegistry& ob,
	const XFoam_Word& regionName = XFoam_polyMeshDefaultRegion(),
	const XFoam_FileName& path = XFoam_FileName::null
);

XFoam_API XFoam_IOobject XFoam_systemDictIO
(
	const XFoam_Word& dictName,
	const XFoam_ObjectRegistry& ob,
	const XFoam_Word& regionName,
	const XFoam_FileName& path
);

/*---------------------------------------------------------------------------*\
        关键字提取（DictionaryBase / PtrListDictionary）
\*---------------------------------------------------------------------------*/
inline XFoam_Word xf_dict_base_key_from_kw(const XFoam_KeyType& kw)
{
	return XFoam_Word(static_cast<const std::string&>(kw));
}

template<class T>
inline XFoam_Word xf_dict_base_key(const T& t)
{
	return xf_dict_base_key_from_kw(t.keyword());
}

inline XFoam_Word xf_dict_base_key(const XFoam_Dictionary& d)
{
	return d.dictName();
}

template<class IDLListType, class T>
class XFoam_DictionaryBase;

template<class IDLListType, class T>
XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_DictionaryBase<IDLListType, T>& dict);

/*---------------------------------------------------------------------------*\
                      Class XFoam_DictionaryBase Declaration
        对标 OpenFOAM .../DictionaryBase/DictionaryBase.H + DictionaryBase.C
\*---------------------------------------------------------------------------*/
// IDLListType 须与 OF 一致：提供 size、operator()(label) 取 T*、insert(T*)、append(T*)、
// clear、transfer、Istream 构造及（用于 remove）removeAt(label)->AutoPtr<T>。
template<class IDLListType, class T>
class XFoam_DictionaryBase
	: public IDLListType
{
protected:
	XFoam_HashTable<T*, XFoam_Word> hashedTs_;

	void addEntries()
	{
		for (XFoam_Label i = 0; i < static_cast<IDLListType*>(this)->size(); ++i)
		{
			T* p = static_cast<IDLListType*>(this)->operator()(i);
			if (p)
			{
				(void)hashedTs_.insert(xf_dict_base_key(*p), p);
			}
		}
	}

public:
	explicit XFoam_DictionaryBase(const XFoam_Label hashSize = 128)
		: IDLListType()
		, hashedTs_(hashSize > 0 ? hashSize : 128)
	{}

	XFoam_DictionaryBase(const XFoam_DictionaryBase& dict)
		: IDLListType(static_cast<const IDLListType&>(dict))
		, hashedTs_()
	{
		addEntries();
	}

	XFoam_DictionaryBase(XFoam_DictionaryBase&& dict) noexcept
		: IDLListType(std::move(static_cast<IDLListType&&>(dict)))
		, hashedTs_(std::move(dict.hashedTs_))
	{}

	template<class INew>
	XFoam_DictionaryBase(XFoam_IStream& is, const INew& inewt)
		: IDLListType(is, inewt)
		, hashedTs_()
	{
		addEntries();
	}

	explicit XFoam_DictionaryBase(XFoam_IStream& is)
		: IDLListType(is)
		, hashedTs_()
	{
		addEntries();
	}

	bool found(const XFoam_Word& keyword) const { return hashedTs_.found(keyword); }

	const T* lookupPtr(const XFoam_Word& keyword) const
	{
		const typename XFoam_HashTable<T*, XFoam_Word>::const_iterator it = hashedTs_.find(keyword);
		if (it != hashedTs_.end())
		{
			return *it;
		}
		return nullptr;
	}

	T* lookupPtr(const XFoam_Word& keyword)
	{
		typename XFoam_HashTable<T*, XFoam_Word>::iterator it = hashedTs_.find(keyword);
		if (it != hashedTs_.end())
		{
			return *it;
		}
		return nullptr;
	}

	const T* lookup(const XFoam_Word& keyword) const
	{
		const T* p = lookupPtr(keyword);
		if (!p)
		{
			XFoam_FatalErrorInFunction << keyword << " is undefined" << XFoam_abort(XFoam_FatalError);
		}
		return p;
	}

	T* lookup(const XFoam_Word& keyword)
	{
		T* p = lookupPtr(keyword);
		if (!p)
		{
			XFoam_FatalErrorInFunction << keyword << " is undefined" << XFoam_abort(XFoam_FatalError);
		}
		return p;
	}

	XFoam_WordList toc() const
	{
		const XFoam_Label n = static_cast<const IDLListType*>(this)->size();
		XFoam_WordList keywords(n);
		for (XFoam_Label i = 0; i < n; ++i)
		{
			T* p = const_cast<IDLListType*>(static_cast<const IDLListType*>(this))->operator()(i);
			if (p)
			{
				keywords[i] = xf_dict_base_key(*p);
			}
			else
			{
				keywords[i] = XFoam_Word::null;
			}
		}
		return keywords;
	}

	XFoam_WordList sortedToc() const
	{
		const std::vector<XFoam_Word> v = hashedTs_.sortedToc();
		XFoam_WordList out(static_cast<XFoam_Label>(v.size()));
		for (XFoam_Size j = 0; j < v.size(); ++j)
		{
			out[static_cast<XFoam_Label>(j)] = v[j];
		}
		return out;
	}

	void insert(const XFoam_Word& keyword, T* tPtr)
	{
		(void)hashedTs_.insert(keyword, tPtr);
		static_cast<IDLListType*>(this)->insert(tPtr);
	}

	void append(const XFoam_Word& keyword, T* tPtr)
	{
		(void)hashedTs_.insert(keyword, tPtr);
		static_cast<IDLListType*>(this)->append(tPtr);
	}

	T* remove(const XFoam_Word& keyword)
	{
		typename XFoam_HashTable<T*, XFoam_Word>::iterator it = hashedTs_.find(keyword);
		if (it == hashedTs_.end())
		{
			return nullptr;
		}
		T* const p = *it;
		hashedTs_.erase(keyword);
		for (XFoam_Label i = 0; i < static_cast<IDLListType*>(this)->size(); ++i)
		{
			if (static_cast<IDLListType*>(this)->operator()(i) == p)
			{
				return static_cast<IDLListType*>(this)->removeAt(i).ptr();
			}
		}
		return p;
	}

	void clear()
	{
		static_cast<IDLListType*>(this)->clear();
		hashedTs_.clear();
	}

	void transfer(XFoam_DictionaryBase& dict)
	{
		static_cast<IDLListType&>(*this).transfer(static_cast<IDLListType&>(dict));
		hashedTs_.transfer(dict.hashedTs_);
	}

	void operator=(const XFoam_DictionaryBase& dict)
	{
		if (this == &dict)
		{
			XFoam_FatalErrorInFunction << "attempted assignment to self" << XFoam_abort(XFoam_FatalError);
		}
		static_cast<IDLListType&>(*this) = static_cast<const IDLListType&>(dict);
		hashedTs_.clear();
		addEntries();
	}

	const T* operator[](const XFoam_Word& key) const { return lookup(key); }

	T* operator[](const XFoam_Word& key) { return lookup(key); }
};

template<class IDLListType, class T>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_DictionaryBase<IDLListType, T>& dict)
{
	const IDLListType& lst = static_cast<const IDLListType&>(dict);
	for (XFoam_Label i = 0; i < lst.size(); ++i)
	{
		const T* p = lst.operator()(i);
		if (p)
		{
			os << *p << '\n';
		}
		if (!os.good())
		{
			break;
		}
	}
	return os;
}

/*---------------------------------------------------------------------------*\
                      Class XFoam_PtrListDictionary Declaration
        对标 OpenFOAM .../PtrListDictionary/PtrListDictionary.H + .C（子集）
\*---------------------------------------------------------------------------*/
// convert / lookupType 模板、UPtrListDictionary 友元及 tmp 相关重载未移植。
template<class T>
class XFoam_PtrListDictionary
	: public XFoam_DictionaryBase<XFoam_PtrList<T>, T>
{
public:
	using Base = XFoam_DictionaryBase<XFoam_PtrList<T>, T>;

	using XFoam_PtrList<T>::operator[];

	XFoam_PtrListDictionary()
		: Base(128)
	{}

	explicit XFoam_PtrListDictionary(const XFoam_Label n)
		: Base(2 * n)
	{
		XFoam_PtrList<T>::setSize(n);
	}

	XFoam_PtrListDictionary(const XFoam_PtrListDictionary& d)
		: Base(d)
	{}

	XFoam_PtrListDictionary(XFoam_PtrListDictionary&& d) noexcept
		: Base(std::move(d))
	{}

	template<class INew>
	XFoam_PtrListDictionary(XFoam_IStream& is, const INew& inewt)
		: Base(is, inewt)
	{}

	explicit XFoam_PtrListDictionary(XFoam_IStream& is)
		: Base(is)
	{}

	XFoam_Label findIndex(const XFoam_Word& key) const
	{
		for (XFoam_Label i = 0; i < XFoam_PtrList<T>::size(); ++i)
		{
			if (xf_dict_base_key((*this)[i]) == key)
			{
				return i;
			}
		}
		return -1;
	}

	XFoam_LabelList findIndices(const XFoam_WordRe& key) const
	{
		XFoam_LabelList indices;
		if (static_cast<const XFoam_String&>(key).empty())
		{
			return indices;
		}
		if (!key.isPattern())
		{
			const XFoam_Word kw(static_cast<const XFoam_String&>(key));
			for (XFoam_Label i = 0; i < XFoam_PtrList<T>::size(); ++i)
			{
				if (xf_dict_base_key((*this)[i]) == kw)
				{
					indices.append(i);
				}
			}
			return indices;
		}
		// 与 OF PtrListDictionary::findIndices 一致：对已编译正则的 XFoam_WordRe 逐条 match 关键字。
		for (XFoam_Label i = 0; i < XFoam_PtrList<T>::size(); ++i)
		{
			const XFoam_Word entKw(xf_dict_base_key((*this)[i]));
			if (key.match(static_cast<const XFoam_String&>(entKw), false))
			{
				indices.append(i);
			}
		}
		return indices;
	}

	inline void append(const XFoam_Word& key, T* ptr)
	{
		if (!this->hashedTs_.insert(key, ptr))
		{
			XFoam_FatalErrorInFunction << "Cannot insert with key '" << key.c_str() << "' into hash-table"
						   << XFoam_abort(XFoam_FatalError);
		}
		XFoam_PtrList<T>::append(ptr);
	}

	inline void append(const XFoam_Word& key, const XFoam_AutoPtr<T>& aptr)
	{
		append(key, const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	// XFoam_Tmp 仅适用于继承 XFoam_RefCount 的类型；否则 XFoam_Tmp<T> 类体 static_assert 会失败（如 T=Dictionary）。
	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value>::type append(
		const XFoam_Word& key,
		const XFoam_Tmp<T>& t)
	{
		append(key, const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	inline void append(T* ptr)
	{
		append(xf_dict_base_key(*ptr), ptr);
	}

	inline void append(const XFoam_AutoPtr<T>& aptr)
	{
		append(const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value>::type append(const XFoam_Tmp<T>& t)
	{
		append(const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	inline XFoam_AutoPtr<T> set(const XFoam_Label i, const XFoam_Word& key, T* ptr)
	{
		if (ptr == nullptr)
		{
			this->hashedTs_.erase(key);
		}
		else
		{
			(void)this->hashedTs_.set(key, ptr);
		}
		return XFoam_PtrList<T>::set(i, ptr);
	}

	inline XFoam_AutoPtr<T> set(const XFoam_Label i, const XFoam_Word& key, const XFoam_AutoPtr<T>& aptr)
	{
		return set(i, key, const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value, XFoam_AutoPtr<T>>::type set(
		const XFoam_Label i,
		const XFoam_Word& key,
		const XFoam_Tmp<T>& t)
	{
		return set(i, key, const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	inline XFoam_AutoPtr<T> set(const XFoam_Label i, T* ptr)
	{
		if (ptr == nullptr)
		{
			const XFoam_Word key = xf_dict_base_key((*this)[i]);
			return set(i, key, nullptr);
		}
		return set(i, xf_dict_base_key(*ptr), ptr);
	}

	inline XFoam_AutoPtr<T> set(const XFoam_Label i, const XFoam_AutoPtr<T>& aptr)
	{
		T* raw = const_cast<XFoam_AutoPtr<T>&>(aptr).ptr();
		if (!raw)
		{
			const XFoam_Word key = xf_dict_base_key((*this)[i]);
			return set(i, key, nullptr);
		}
		return set(i, xf_dict_base_key(*raw), raw);
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value, XFoam_AutoPtr<T>>::type set(
		const XFoam_Label i,
		const XFoam_Tmp<T>& t)
	{
		T* raw = t.ptr();
		if (!raw)
		{
			const XFoam_Word key = xf_dict_base_key((*this)[i]);
			return set(i, key, nullptr);
		}
		return set(i, xf_dict_base_key(*raw), raw);
	}

	// 对标 OF PtrListDictionary::remove（autoPtr+shrink）；与基类 DictionaryBase::remove(T*) 并存。
	inline XFoam_AutoPtr<T> removeEntry(const XFoam_Word& key)
	{
		const XFoam_Label i = findIndex(key);
		if (i < 0)
		{
			return XFoam_AutoPtr<T>();
		}
		XFoam_AutoPtr<T> ptr(set(i, key, nullptr));
		XFoam_PtrList<T>::shrink();
		return ptr;
	}

	const T& operator[](const XFoam_Word& key) const { return *Base::operator[](key); }

	T& operator[](const XFoam_Word& key) { return *Base::operator[](key); }
};

#endif
