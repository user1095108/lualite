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

inline void set_result(lua_State* const L, point p)
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
}

template <std::size_t I, typename T>
inline typename std::enable_if<
  std::is_same<point, T>::value, point>::type
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

point testfunc(int i)
{
  std::cout << "testfunc(): " << i << std::endl;

  return point{-1, -222};
}

struct testbase
{
  std::string dummy(std::string msg)
  {
    return std::string("dummy() called: " + msg);
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

    return std::vector<std::string>(10, "bla!!!");
  }

  void print(int i)
  {
    std::cout << i << std::endl;
  }

  int const& a()
  {
    std::cout << "getter called" << std::endl;
    return a_;
  }

  void set_a(int i)
  {
    std::cout << "setter called" << std::endl;
    a_ = i;
  }

  std::string const& test_array(std::array<int, 10> const& a)
  {
    std::cout << a[0] << std::endl;
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

  lualite::module(L,
    lualite::class_<testbase>("testbase")
      .def("dummy", &testbase::dummy),
    lualite::class_<testclass>("testclass")
      .constructor<int>()
      .inherits<testbase>()
      .enum_("smell", 9)
      .def("print", (void (testclass::*)(int))&testclass::print)
      .def("print_", (std::vector<std::string> (testclass::*)(std::string) const)&testclass::print)
      .def("pointer", &testclass::pointer)
      .def("reference", &testclass::reference)
      .property("a", &testclass::a, &testclass::set_a)
      .def("test_array", &testclass::test_array),
    lualite::scope("subscope",
      lualite::class_<testclass>("testclass")
        .constructor<int>()
        .enum_("smell", 10)
        .def("testfunc", &testfunc)
        .def("print", (void (testclass::*)(int))&testclass::print)
        .def("print_", (std::vector<std::string> (testclass::*)(std::string) const)&testclass::print)
    )
  )
  .enum_("apple", 1)
  .def("testfunc", &testfunc);

  luaL_dostring(
    L,
    "local a = testfunc(3)\n"
    "print(a.y)\n"
    "print(apple)\n"
    "print(testclass.__classname)\n"
    "print(testclass.smell)\n"
    "local b = testclass.new(1000)\n"
    "print(\"---\")\n"
    "print(b.a)\n"
    "b:reference().a = 888\n"
    "print(b.a .. \" \" .. b:dummy(\"test\"))\n"
    "b:pointer():print(100)\n"
    "b:reference():print_(\"msg1\")\n"
    "local a = subscope.testclass.new(1111)\n"
    "print(subscope.testclass.smell)\n"
    "subscope.testclass.testfunc(200)\n"
    "local c = a:reference():print_(\"msg2\")\n"
    "print(c[10])\n"
    "r = {}"
    "for i = 1, 10 do\n"
    "  r[i] = 7\n"
    "end\n"
    "print(a:test_array(r))\n"
  );

  lua_close(L);

  return EXIT_SUCCESS;
}
