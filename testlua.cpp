#include <cstdlib>

#include <iostream>

extern "C" {

#include "lua/lualib.h"

}

#include "lualite/lualite.hpp"

struct point
{
  int x;
  int y;
};

inline int set_result(lua_State* const L, point p)
{
  using namespace ::lualite::detail;

  lua_createtable(L, 2, 0);

  lua_pushliteral(L, "x");
  set_result(L, const_cast<decltype(p.x) const&>(p.x));
  assert(lua_istable(L, -3));
  lua_rawset(L, -3);

  lua_pushliteral(L, "y");
  set_result(L, const_cast<decltype(p.y) const&>(p.y));
  assert(lua_istable(L, -3));
  lua_rawset(L, -3);

  return 1;
}

template <std::size_t I, typename T>
inline typename std::enable_if<
  std::is_same<T, point>::value, point>::type
get_arg(lua_State* const L)
{
  using namespace ::lualite::detail;

  assert(lua_istable(L, I));

  struct point p;

  lua_pushliteral(L, "x");
  lua_rawget(L, -2);
  p.x = lua_tointeger(L, -1);

  lua_pushliteral(L, "y");
  lua_rawget(L, -2);
  p.y = lua_tointeger(L, -1);

  return p;
}

point testfunc(int i, int j, int k)
{
  std::cout << "testfunc(): " << i << " " << j << " " << k << std::endl;

  return {-1, -222};
}

void testpair(std::pair<char const*, char const*> const& p)
{
  std::cout << "first: " << p.first
    << " second: " << p.second << std::endl;
}

void testtuple(std::tuple<char const*, char const*, int> const& p)
{
  std::cout << "first: " << std::get<0>(p)
    << " second: " << std::get<1>(p)
    << " third: " << std::get<2>(p) << std::endl;
}

struct testbase
{
  std::string dummy(std::string msg)
  {
    return { "dummy() called: " + msg };
  }
};

struct testclass : testbase
{
  testclass()
    : a_(777)
  {
    std::cout << "testclass::testclass()" << std::endl;
  }

  testclass(int i)
    : a_(777)
  {
    std::cout << "testclass::testclass(int):" << i << std::endl;
  }

  std::vector<std::string> print(std::string msg) const
  {
    std::cout << "hello world!: " << msg << std::endl;

    return { 10, "bla!!!" };
  }

  std::tuple<int, std::string, char const*> print(int i)
  {
    std::cout << i << std::endl;

    return std::make_tuple(9, "huh?", "tralala");
  }

  int const& a()
  {
    std::cout << "getter called: " << a_ << std::endl;

    return a_;
  }

  void set_a(int i)
  {
    std::cout << "setter called: " << i << std::endl;

    a_ = i;
  }

  std::string const& test_array(std::array<int, 10> const& a)
  {
    std::cout << a[a.size() - 1] << std::endl;

    s_ = "blablabla";

    return s_;
  }

  testclass* pointer()
  {
    return this;
  }

  testclass& reference()
  {
    return *this;
  }

private:
  int a_;
  std::string s_;
};

int main(int argc, char* argv[])
{
  lua_State* L(luaL_newstate());

  luaL_openlibs(L);

  lualite::module{L,
    lualite::class_<testbase>("testbase")
      .constant("__classname", "testbase")
      .constant("__b", true)
      .constant("__pi", 3.1459)
      .def<decltype(&testbase::dummy), &testbase::dummy>("dummy"),
    lualite::class_<testclass>("testclass")
      .constructor("defaultNew")
      .constructor<int>()
      .inherits<testbase>() // you can add more classes to inherit from
      .enum_("smell", 9)
      .def<std::tuple<int, std::string, char const*> (testclass::*)(int), &testclass::print>("print")
      .def<std::vector<std::string> (testclass::*)(std::string) const, &testclass::print>("print_")
      .def<LLFUNC(testclass::pointer)>("pointer")
      .def<LLFUNC(testclass::reference)>("reference")
      .property<LLFUNC(testclass::a), LLFUNC(testclass::set_a)>("a")
      .def<LLFUNC(testclass::test_array)>("test_array"),
    lualite::scope("subscope",
      lualite::class_<testclass>("testclass")
        .constructor<>("defaultNew")
        .constructor<int>()
        .enum_("smell", 10)
        .def<LLFUNC(testfunc)>("testfunc")
        .def<std::tuple<int, std::string, char const*> (testclass::*)(int), &testclass::print>("print")
        .def<std::vector<std::string> (testclass::*)(std::string) const, &testclass::print>("print_")
    )
  }
  .enum_("apple", 1)
  .def<LLFUNC(testfunc)>("testfunc")
  .def<LLFUNC(testpair)>("testpair")
  .def<LLFUNC(testtuple)>("testtuple");

  luaL_dostring(
    L,
    "local a = testfunc(3, 2, 1)\n"
    "r = { \"my\", \"pair\" }\n"
    "testpair(r)\n"
    "r[3] = 3\n"
    "testtuple(r)\n"
    "print(a.y)\n"
    "print(apple)\n"
    "print(testclass.smell)\n"
    "print(testbase.__classname)\n"
    "print(testbase.__b)\n"
    "print(testbase.__pi)\n"
    "local b = testclass.defaultNew()\n"
    "print(\"---\")\n"
    "print(b.a)\n"
    "b:reference().a = 888\n"
    "print(b.dummy)\n"
    "print(b.a .. \" \" .. b:dummy(\"test\"))\n"
    "local tmp1, tmp2, tmp3 = b:pointer():print(100)\n"
    "print(tmp1 .. \" \" .. tmp2 .. \" \" .. tmp3)\n"
    "b:reference():print_(\"msg1\")\n"
    "local a = subscope.testclass.new(1111)\n"
    "print(subscope.testclass.smell)\n"
    "subscope.testclass.testfunc(200, 0, 1)\n"
    "local c = a:reference():print_(\"msg2\")\n"
    "print(c[10])\n"
    "r = {}"
    "for i = 1, 10 do\n"
    "  r[i] = 7\n"
    "end\n"
    "print(a:test_array(r))\n"
  );

  ::std::cout << ::lualite::class_<testclass>::inherits("testbase") <<
    ::std::endl;

  lua_getglobal(L, "print");
  ::lualite::call(L, 0, "--test--");

  ::std::cout << ::lualite::detail::hash("test") <<
    " " <<
    ::lualite::detail::hash("test") <<
    ::std::endl;

  lua_close(L);

  return EXIT_SUCCESS;
}
