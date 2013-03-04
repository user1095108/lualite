#include <cstdlib>

#include <iostream>

extern "C" {

#include "lua/lualib.h"

}

#include "lualite/lualite_stdcontainers.hpp"

#include "lualite/lualite.hpp"

struct point
{
  int x;
  int y;
};

inline void set_result(lua_State* const L,
  point && p)
{
  lua_createtable(L, 2, 0);

  lua_pushliteral(L, "x");
  ::lualite::detail::set_result(L, p.x);
  assert(lua_istable(L, -3));
  lua_rawset(L, -3);

  lua_pushliteral(L, "y");
  ::lualite::detail::set_result(L, p.y);
  assert(lua_istable(L, -3));
  lua_rawset(L, -3);
}

template <std::size_t I>
inline point get_arg(lua_State* const L,
  point &&)
{
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

struct testclass
{
  testclass()
  {
    std::cout << "testclass::testclass()" << std::endl;
  }

  testclass(int i)
  {
    std::cout << "testclass::testclass(int):" << i << std::endl;
  }

  std::vector<std::string> print() const
  {
    std::cout << "hello world!" << std::endl;

    return std::vector<std::string>(10, "bla!!!");
  }

  void print(int i)
  {
    std::cout << i << std::endl;
  }

  testclass const* pointer()
  {
    return this;
  }

  testclass const& reference()
  {
    return *this;
  }
};

int main(int argc, char* argv[])
{
  lua_State* L(luaL_newstate());

  luaL_openlibs(L);

  lualite::module(L,
    lualite::class_<testclass>("testclass")
      .constructor<int>()
      .enum_("smell", 9)
      .def("print", (void (testclass::*)(int))&testclass::print)
      .def("print_", (std::vector<std::string> (testclass::*)() const)&testclass::print)
      .def("pointer", &testclass::pointer)
      .def("reference", &testclass::reference),
    lualite::scope("subscope",
      lualite::class_<testclass>("testclass")
        .constructor<int>()
        .enum_("smell", 10)
        .def("testfunc", &testfunc)
        .def("print", (void (testclass::*)(int))&testclass::print)
        .def("print_", (std::vector<std::string> (testclass::*)() const)&testclass::print)
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
    "b:pointer():print(100)\n"
    "b:reference():print_()\n"
    "local a = subscope.testclass.new(1111)\n"
    "print(subscope.testclass.smell)\n"
    "subscope.testclass.testfunc(200)\n"
    "local c = a:print_()\n"
    "print(c[10])\n"
  );

  lua_close(L);

  return EXIT_SUCCESS;
}
