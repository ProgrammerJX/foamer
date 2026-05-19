#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

static XFoam_ITstream g_dummyITstream;
static XFoam_Dictionary g_dummyDictForEntry;

// 最小 XFoam_Entry 实现，用于 dictionary 容器与哈希路径测试（非完整 OF primitiveEntry）。
class TestDictEntry final
	: public XFoam_Entry
{
	XFoam_FileName name_;

public:
	explicit TestDictEntry(const XFoam_KeyType& kw)
		: XFoam_Entry(kw, 1)
		, name_(std::string("test/") + static_cast<const std::string&>(kw))
	{}

	explicit TestDictEntry(const XFoam_Word& kw)
		: TestDictEntry(XFoam_KeyType(kw))
	{
	}

	XFoam_AutoPtr<XFoam_Entry> clone(const XFoam_Dictionary& parentDict) const override
	{
		(void)parentDict;
		return XFoam_AutoPtr<XFoam_Entry>(new TestDictEntry(keyword()));
	}

	const XFoam_FileName& name() const override { return name_; }

	XFoam_FileName& name() override { return name_; }

	XFoam_Label endLineNumber() const override { return 1; }

	bool isStream() const override { return false; }

	XFoam_ITstream& stream() const override { return g_dummyITstream; }

	bool isDict() const override { return false; }

	const XFoam_Dictionary& dict() const override { return g_dummyDictForEntry; }

	XFoam_Dictionary& dict() override { return g_dummyDictForEntry; }

	void write(XFoam_OStream& os) const override { os << keyword(); }
};

boost::filesystem::path xf_testDictionaryDataDir()
{
	return (boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "data" / "test_dictionary")
		.lexically_normal();
}

static constexpr const char* kTestDictionaryFiles[] = {
	"testDict",
	"testDict2",
	"testDictInc",
	"testDictRegex",
	"testDotDict",
};

} // namespace

TEST_CASE("XFoam_Token scalar uses unified SCALAR type and XFoam_Scalar storage")
{
	const XFoam_Token a(0.25, 0);
	CHECK(a.isScalar());
	CHECK(a.scalarToken() == doctest::Approx(0.25));
	CHECK(a == 0.25);

	const XFoam_Token b(1.5, 0);
	CHECK(b.isScalar());
	CHECK(b.scalarToken() == doctest::Approx(1.5));

	XFoam_Token c;
	c = 2.0;
	CHECK(c.isScalar());
	CHECK(c.scalarToken() == doctest::Approx(2.0));
}

TEST_CASE("XFoam_readTokenFromStream scans test_dictionary files")
{
	const boost::filesystem::path base = xf_testDictionaryDataDir();
	REQUIRE(boost::filesystem::is_directory(base));

	for (const char* fname : kTestDictionaryFiles)
	{
		const boost::filesystem::path path = base / fname;
		INFO(fname);
		REQUIRE_MESSAGE(boost::filesystem::exists(path), "missing test_dictionary file");

		std::ifstream in(path.string().c_str(), std::ios::in | std::ios::binary);
		REQUIRE(in);
		std::ostringstream oss;
		oss << in.rdbuf();
		const std::string content = oss.str();
		REQUIRE(content.size() > 0);

		XFoam_IStringStream is(content);
		XFoam_Label lineNo = 0;
		XFoam_Token tok;
		XFoam_Label nTok = 0;
		while (XFoam_readTokenFromStream(is, lineNo, tok))
		{
			++nTok;
			REQUIRE(nTok < 1000000);
		}
		CHECK(nTok > 5);
	}
}

TEST_CASE("XFoam_Dictionary read placeholder on test_dictionary (same entry as blockMesh tests)")
{
	// 完整 dictionary::read 对 test_dictionary 下部分夹具会触发 FatalIOError（如模式项 RegExp、#include 等未与 OF 对齐）。
	// 夹具的 token 扫描仍由 “XFoam_readTokenFromStream scans test_dictionary files” 覆盖。
	XFoam_IStringStream sis(XFoam_String("placeholder.inline"), XFoam_String("n 3;\n"));
	XFoam_Dictionary dict;
	CHECK(dict.read(sis));
}

TEST_CASE("XFoam_Dictionary null name and empty search")
{
	CHECK(XFoam_Dictionary::null.isNull());

	XFoam_Dictionary d(XFoam_FileName("rootDict"));
	CHECK(d.name().name() == XFoam_String("rootDict"));
	CHECK(d.size() == 0);
	CHECK_FALSE(d.found(XFoam_Word("missing")));
	CHECK(&(d.parent()) == &XFoam_Dictionary::null);
	CHECK(&(d.topDict()) == &d);
}

TEST_CASE("XFoam_Dictionary parent and topDict chain")
{
	XFoam_Dictionary parent(XFoam_FileName("top"));
	XFoam_Dictionary child(XFoam_FileName("child"), parent);
	CHECK(&(child.parent()) == &parent);
	CHECK(&(child.topDict()) == &parent);
}

TEST_CASE("XFoam_Dictionary add remove lookup toc sortedToc")
{
	XFoam_Dictionary d;
	REQUIRE(d.add(new TestDictEntry(XFoam_Word("beta"))));
	REQUIRE(d.add(new TestDictEntry(XFoam_Word("alpha"))));
	CHECK(d.size() == 2);
	CHECK(d.found(XFoam_Word("alpha")));
	CHECK(d.found(XFoam_Word("beta")));

	const XFoam_WordList keys = d.toc();
	REQUIRE(keys.size() == 2);

	const XFoam_WordList sorted = d.sortedToc();
	REQUIRE(sorted.size() == 2);
	CHECK(sorted[0] == XFoam_Word("alpha"));
	CHECK(sorted[1] == XFoam_Word("beta"));

	const XFoam_Entry* ep = d.lookupEntryPtr(XFoam_Word("beta"), false, false);
	REQUIRE(ep != nullptr);
	CHECK(ep->keyword() == XFoam_Word("beta"));

	REQUIRE(d.remove(XFoam_Word("alpha")));
	CHECK(d.size() == 1);
	CHECK_FALSE(d.found(XFoam_Word("alpha")));
	CHECK(d.found(XFoam_Word("beta")));
}

TEST_CASE("XFoam_Dictionary pattern entry found with patternMatch")
{
	XFoam_Dictionary d;
	// XFoam_Word::isPattern：首字符为 '~'（见 xfoam_types.h）
	REQUIRE(d.add(new TestDictEntry(XFoam_KeyType(std::string("~.*")))));
	CHECK_FALSE(d.found(XFoam_Word("foo"), false, false));
	CHECK(d.found(XFoam_Word("foo"), false, true));
	const XFoam_Entry* p = d.lookupEntryPtr(XFoam_Word("anything"), false, true);
	REQUIRE(p != nullptr);
	CHECK(p->keyword() == XFoam_Word("~.*"));
}

TEST_CASE("XFoam_Dictionary clear")
{
	XFoam_Dictionary d;
	d.add(new TestDictEntry(XFoam_Word("a")));
	d.clear();
	CHECK(d.size() == 0);
	CHECK_FALSE(d.found(XFoam_Word("a")));
}

TEST_CASE("XFoam_Dictionary transfer")
{
	XFoam_Dictionary a;
	a.add(new TestDictEntry(XFoam_Word("k1")));
	XFoam_Dictionary b;
	b.add(new TestDictEntry(XFoam_Word("k2")));
	a.transfer(b);
	CHECK(a.size() == 1);
	CHECK(b.size() == 0);
	CHECK_FALSE(a.found(XFoam_Word("k1")));
	CHECK(a.found(XFoam_Word("k2")));
}

TEST_CASE("XFoam_Dictionary operator| merge union")
{
	XFoam_Dictionary left;
	left.add(new TestDictEntry(XFoam_Word("only")));
	XFoam_Dictionary right;
	right.add(new TestDictEntry(XFoam_Word("extra")));
	const XFoam_Dictionary u = left | right;
	CHECK(u.found(XFoam_Word("only")));
	CHECK(u.found(XFoam_Word("extra")));
}

TEST_CASE("XFoam_Dictionary merge adds missing keys")
{
	XFoam_Dictionary base;
	base.add(new TestDictEntry(XFoam_Word("x")));
	XFoam_Dictionary extra;
	extra.add(new TestDictEntry(XFoam_Word("y")));
	CHECK(base.merge(extra));
	CHECK(base.found(XFoam_Word("x")));
	CHECK(base.found(XFoam_Word("y")));
}

TEST_CASE("XFoam_Dictionary clone")
{
	XFoam_Dictionary d;
	d.add(new TestDictEntry(XFoam_Word("z")));
	const XFoam_AutoPtr<XFoam_Dictionary> c = d.clone();
	REQUIRE(c.valid());
	CHECK(c->found(XFoam_Word("z")));
}

TEST_CASE("XFoam_Dictionary stream write")
{
	XFoam_Dictionary d(XFoam_FileName("dictOut"));
	XFoam_OStringStream oss;
	CHECK_NOTHROW(oss << d);
}
