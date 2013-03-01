/*
** Copyright (c) 2013, Janez Žemva
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the copyright holder nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#ifndef LUALITE_HPP
# define LUALITE_HPP

#if __cplusplus < 201103L
# error "You need a C++11 compiler to use lualite"
#endif // __cplusplus

#include <cassert>

#include <cmath>

#include <stdexcept>

#include <type_traits>

#include <utility>

#include <array>

#include <deque>

#include <forward_list>

#include <list>

#include <map>

#include <unordered_map>

#include <string>

#include <vector>

extern "C" {

# include "lua/lauxlib.h"

}

namespace lualite
{

class scope;

namespace detail
{

template <std::size_t ...Ns>
struct indices
{
  typedef indices<Ns..., sizeof...(Ns)> next;
};

template <std::size_t N>
struct make_indices
{
  typedef typename make_indices<N - 1>::type::next type;
};

template<>
struct make_indices<0>
{
  typedef indices<> type;
};

typedef std::vector<std::pair<char const*, int> > enum_info_type;

typedef std::vector<std::pair<char const*, lua_CFunction> > member_info_type;

typedef member_info_type func_info_type;

void dummy();

struct dummy_
{
  void dummy();
};

struct class_meta_info
{
  char const* class_name;

  member_info_type* members;
};

struct func_meta_info
{
  std::aligned_storage<sizeof(&dummy)>::type ptr_to_func;
};

struct member_meta_info
{
  std::aligned_storage<sizeof(&dummy_::dummy)>::type ptr_to_member;
};

inline void set_result(lua_State* const L,
  long double const value)
{
  lua_pushnumber(L, value);
}

inline void set_result(lua_State* const L,
  double const value)
{
  lua_pushnumber(L, value);
}

inline void set_result(lua_State* const L,
  float const value)
{
  lua_pushnumber(L, value);
}

inline void set_result(lua_State* const L,
  long long const value)
{
  lua_pushinteger(L, value);
}

inline void set_result(lua_State* const L,
  unsigned long long const value)
{
  lua_pushunsigned(L, value);
}

inline void set_result(lua_State* const L,
  long const value)
{
  lua_pushinteger(L, value);
}

inline void set_result(lua_State* const L,
  unsigned long const value)
{
  lua_pushunsigned(L, value);
}

inline void set_result(lua_State* const L,
  int const value)
{
  lua_pushinteger(L, value);
}

inline void set_result(lua_State* const L,
  unsigned int const value)
{
  lua_pushunsigned(L, value);
}

inline void set_result(lua_State* const L,
  signed char const value)
{
  lua_pushunsigned(L, value);
}

inline void set_result(lua_State* const L,
  unsigned char const value)
{
  lua_pushunsigned(L, value);
}

inline void set_result(lua_State* const L,
  char const value)
{
  lua_pushinteger(L, value);
}

inline void set_result(lua_State* const L,
  bool const value)
{
  lua_pushboolean(L, value);
}

inline void set_result(lua_State* const L,
  void* const value)
{
  lua_pushlightuserdata(L, value);
}

inline void set_result(lua_State* const  L,
  std::string const& value)
{
  lua_pushlstring(L, value.c_str(), value.size());
}

inline void set_result(lua_State* const L,
  char const* const value)
{
  lua_pushstring(L, value);
}

template <typename T, std::size_t N>
inline void set_result(lua_State* const L,
  std::array<T, N> && a)
{
  lua_createtable(L, N, 0);

  auto const end(a.cend());

  for (auto i(a.cbegin()); i != end; ++i)
  {
    lua_pushunsigned(L, i - a.cbegin() + 1);
    set_result(L, *i);

    lua_rawset(L, -3);
  }
}

template <typename T, class Alloc>
inline void set_result(lua_State* const L,
  std::deque<T, Alloc> && d)
{
  lua_createtable(L, d.size(), 0);

  auto const end(d.cend());

  for (auto i(d.cbegin()); i != end; ++i)
  {
    lua_pushunsigned(L, i - d.cbegin() + 1);
    set_result(L, *i);

    lua_rawset(L, -3);
  }
}

template <typename T, class Alloc>
inline void set_result(lua_State* const L,
  std::forward_list<T, Alloc> && l)
{
  lua_createtable(L, l.size(), 0);

  std::size_t j(1);

  auto const end(l.cend());

  for (auto i(l.cbegin()); i != end; ++i, ++j)
  {
    lua_pushunsigned(L, j);
    set_result(L, *i);

    lua_rawset(L, -3);
  }
}

template <typename T, class Alloc>
inline void set_result(lua_State* const L,
  std::list<T, Alloc> && l)
{
  lua_createtable(L, l.size(), 0);

  std::size_t j(1);

  auto const end(l.cend());

  for (auto i(l.cbegin()); i != end; ++i, ++j)
  {
    lua_pushunsigned(L, j);
    set_result(L, *i);

    lua_rawset(L, -3);
  }
}


template <class Key, class T, class Compare, class Alloc>
inline void set_result(lua_State* const L,
  std::map<Key, T, Compare, Alloc> && m)
{
  lua_createtable(L, 0, m.size());

  auto const end(m.cend());

  for (auto i(m.cbegin()); i != end; ++i)
  {
    set_result(L, i->first);
    set_result(L, i->second);

    lua_rawset(L, -3);
  }
}

template <class Key, class T, class Hash, class Pred, class Alloc>
inline void set_result(lua_State* const L,
  std::unordered_map<Key, T, Hash, Pred, Alloc> && m)
{
  lua_createtable(L, 0, m.size());

  auto const end(m.cend());

  for (auto i(m.cbegin()); i != end; ++i)
  {
    set_result(L, i->first);
    set_result(L, i->second);

    lua_rawset(L, -3);
  }
}


template <typename T, class Alloc>
inline void set_result(lua_State* const L,
  std::vector<T, Alloc> && v)
{
  lua_createtable(L, v.size(), 0);

  auto const end(v.cend());

  for (auto i(v.cbegin()); i != end; ++i)
  {
    lua_pushunsigned(L, i - v.cbegin() + 1);
    set_result(L, *i);

    lua_rawset(L, -3);
  }
}

template <std::size_t I>
inline std::string get_arg(lua_State* const L,
  std::string const)
{
  assert(lua_isstring(L, I));

  std::size_t len;
  char const* const val(lua_tolstring(L, I, &len));

  return std::string(val, len);
}

template <std::size_t I>
inline char const* get_arg(lua_State* const L,
  char const* const)
{
  assert(lua_isstring(L, I));
  return lua_tostring(L, I);
}

template <std::size_t I>
inline void* get_arg(lua_State* const L,
  void* const)
{
  assert(lua_islightuserdata(L, I));
  return lua_touserdata(L, I);
}

template <std::size_t I>
inline long double get_arg(lua_State* const L,
  long double const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <std::size_t I>
inline double get_arg(lua_State* const L,
  double const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <std::size_t I>
inline float get_arg(lua_State* const L,
  float const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <std::size_t I>
inline long long get_arg(lua_State* const L,
  long long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline unsigned long long get_arg(lua_State* const L,
  unsigned long long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline long get_arg(lua_State* const L,
  long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline unsigned long get_arg(lua_State* const L,
  unsigned long const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <std::size_t I>
inline int get_arg(lua_State* const L,
  int const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline unsigned int get_arg(lua_State* const L,
  unsigned int const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <std::size_t I>
inline signed char get_arg(lua_State* const L,
  signed char const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline unsigned char get_arg(lua_State* const L,
  unsigned char const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <std::size_t I>
inline char get_arg(lua_State* const L,
  char const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <std::size_t I>
inline bool get_arg(lua_State* const L,
  bool const)
{
  assert(lua_isboolean(L, I));
  return lua_toboolean(L, I);
}

template <std::size_t O, typename C, typename ...A, std::size_t ...I>
inline C* forward(lua_State* const L, indices<I...>)
{
  return new C(get_arg<I + O>(L, A())...);
}

template <class_meta_info const* cmi, class C, class ...A>
inline int constructor_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  C* const instance(forward<1, C, A...>(L, indices_type()));

  lua_createtable(L, 0, 1);

  for (auto const& i: *cmi->members)
  {
    assert(lua_istable(L, -1));

    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, i.second, 1);

    lua_setfield(L, -2, i.first);
  }

  assert(lua_istable(L, -1));

  lua_pushstring(L, cmi->class_name);
  lua_setfield(L, -2, "__instanceof");

  assert(lua_istable(L, -1));
  return 1;
}

template <std::size_t O, typename R, typename ...A, std::size_t ...I>
inline R forward(lua_State* const L, R (*f)(A...), indices<I...>)
{
  return (*f)(get_arg<I + O>(L, A())...);
}

template <func_meta_info const* fmi, class R, class ...A>
inline typename std::enable_if<std::is_same<void, R>::value, int>::type
func_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef R (*ptr_to_func_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  forward<1, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      static_cast<void const*>(&fmi->ptr_to_func)),
    indices_type());
  return 0;
}

template <func_meta_info const* fmi, class R, class ...A>
inline typename std::enable_if<!std::is_same<void, R>::value, int>::type
func_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef R (*ptr_to_func_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  set_result(L, forward<1, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      static_cast<void const*>(&fmi->ptr_to_func)),
    indices_type()));
  return 1;
}

template <std::size_t O, typename C, typename R,
  typename ...A, std::size_t ...I>
inline R forward(lua_State* const L, C* c,
  R (C::*ptr_to_member)(A...), indices<I...>)
{
  return (c->*ptr_to_member)(get_arg<I + O>(L, A())...);
}

template <member_meta_info const* mmi, class C, class R, class ...A>
inline typename std::enable_if<std::is_same<void, R>::value, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + 1 == lua_gettop(L));

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  forward<2, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
      *static_cast<ptr_to_member_type const*>(
        static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type());
  return 0;
}

template <member_meta_info const* mmi, class C, class R, class ...A>
inline typename std::enable_if<!std::is_same<void, R>::value, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + 1 == lua_gettop(L));

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  set_result(L, forward<2, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
    *static_cast<ptr_to_member_type const*>(
    static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type()));
  return 1;
}

}

class scope
{
public:
  scope(char const* const name)
  : name_(name),
    parent_scope_(0),
    scope_create_(true)
  {
  }

  template <typename ...A>
  scope(char const* const name, A&&... args)
  : name_(name),
    scope_create_(true)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);
  }

  scope(scope const&) = delete;

  virtual void apply(lua_State* const L)
  {
    if (parent_scope_)
    {
      scope::get_scope(L);

      assert(lua_istable(L, -1));

      for (auto const& i: enums_)
      {
        assert(lua_istable(L, -1));

        lua_pushinteger(L, i.second);
        lua_setfield(L, -2, i.first);
      }

      for (auto const& i: functions_)
      {
        assert(lua_istable(L, -1));

        lua_pushcfunction(L, i.second);
        lua_setfield(L, -2, i.first);
      }

      lua_remove(L, -1);
    }
    else
    {
      for (auto const& i: enums_)
      {
        lua_pushinteger(L, i.second);
        lua_setglobal(L, i.first);
      }

      for (auto const& i: functions_)
      {
        lua_pushcfunction(L, i.second);
        lua_setglobal(L, i.first);
      }
    }

    for (auto a: apply_instances_)
    {
      a->apply(L);
    }

    assert(!lua_gettop(L));
  }

  void set_apply_instance(scope* instance)
  {
    apply_instances_.emplace_back(std::move(instance));
  }

  void set_parent_scope(scope* const parent_scope)
  {
    parent_scope_ = parent_scope;

    parent_scope->set_apply_instance(this);
  }

  template <class R, class ...A>
  scope& def(char const* const name, R (*ptr_to_func)(A...))
  {
    static detail::func_meta_info fmi;

    static_assert(sizeof(ptr_to_func) <= sizeof(fmi.ptr_to_func),
      "pointer size mismatch");

    *static_cast<decltype(ptr_to_func)*>(
      static_cast<void*>(&fmi.ptr_to_func))
      = ptr_to_func;

    functions_.emplace_back(
      std::make_pair(name, detail::func_stub<&fmi, R, A...>));
    return *this;
  }

  scope& enum_(char const* const name, int value)
  {
    enums_.emplace_back(std::make_pair(name, value));
    return *this;
  }

protected:
  void get_scope(lua_State* const L)
  {
    if (parent_scope_)
    {
      assert(name_);

      parent_scope_->get_scope(L);

      if (scope_create_)
      {
        scope_create_ = false;

        if (lua_gettop(L) && lua_istable(L, -1))
        {
          lua_createtable(L, 0, 1);
          lua_setfield(L, -2, name_);
        }
        else
        {
          lua_createtable(L, 0, 1);
          lua_setglobal(L, name_);
        }
      }
      // else do nothing

      if (lua_gettop(L) && lua_istable(L, -1))
      {
        luaL_getsubtable(L, -1, name_);
        lua_remove(L, -2);
      }
      else
      {
        lua_getglobal(L, name_);
      }
    }
    else
    {
      if (scope_create_ && name_)
      {
        scope_create_ = false;

        lua_createtable(L, 0, 1);
        lua_setglobal(L, name_);
      }
      // else do nothing

      if (name_)
      {
        lua_getglobal(L, name_);
      }
      // else do nothing
    }
  }

protected:
  char const* const name_;

  scope* parent_scope_;

  std::vector<scope*> apply_instances_;

private:
  bool scope_create_;

  detail::enum_info_type enums_;

  detail::func_info_type functions_;
};

class module : public scope
{
public:
  template <typename ...A>
  module(lua_State* const L, A&&... args)
    : scope(0),
      L_(L)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);

    scope::apply(L);
  }

  template <typename ...A>
  module(lua_State* const L, char const* const name, A&&... args)
    : scope(name),
      L_(L)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);

    scope::apply(L);
  }

  template <class R, class ...A>
  module& def(char const* const name, R (*ptr_to_func)(A...))
  {
    static detail::func_meta_info fmi;

    static_assert(sizeof(ptr_to_func) <= sizeof(fmi.ptr_to_func),
      "pointer size mismatch");

    *static_cast<decltype(ptr_to_func)*>(
      static_cast<void*>(&fmi.ptr_to_func))
      = ptr_to_func;

    if (name_)
    {
      scope::get_scope(L_);
      assert(lua_istable(L_, -1));

      lua_pushcfunction(L_, (detail::func_stub<&fmi, R, A...>));
      lua_setfield(L_, -2, name);

      lua_remove(L_, -1);
    }
    else
    {
      lua_pushcfunction(L_, (detail::func_stub<&fmi, R, A...>));
      lua_setglobal(L_, name);
    }
    return *this;
  }

  module& enum_(char const* const name, int value)
  {
    if (name_)
    {
      scope::get_scope(L_);
      assert(lua_istable(L_, -1));

      lua_pushinteger(L_, value);
      lua_setfield(L_, -2, name);

      lua_remove(L_, -1);
    }
    else
    {
      lua_pushinteger(L_, value);
      lua_setglobal(L_, name);
    }
    return *this;
  }

private:
  lua_State* const L_;
};

template <class C>
class class_ : public scope
{
public:
  class_(char const* const name)
    : scope(name)
  {
  }

  void apply(lua_State* const L)
  {
    assert(parent_scope_);

    scope::apply(L);

    scope::get_scope(L);
    assert(lua_istable(L, -1));

    for (auto const& i: constructors_)
    {
      assert(lua_istable(L, -1));

      lua_pushcfunction(L, i.second);
      lua_setfield(L, -2, i.first);
    }

    lua_pushstring(L, name_);
    lua_setfield(L, -2, "__classname");

    lua_remove(L, -1);

    assert(!lua_gettop(L));
  }

  template <class ...A>
  class_& constructor()
  {
    static detail::class_meta_info const cmi {
      name_,
      &members_
    };

    constructors_.emplace_back(std::make_pair("new",
      detail::constructor_stub<&cmi, C, A...>));
    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (*ptr_to_func)(A...))
  {
    scope::def(name, ptr_to_func);
    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::*ptr_to_member)(A...))
  {
    static detail::member_meta_info mmi;

    static_assert(sizeof(ptr_to_member) <= sizeof(mmi.ptr_to_member),
      "pointer size mismatch");

    *static_cast<decltype(ptr_to_member)*>(
      static_cast<void*>(&mmi.ptr_to_member))
      = ptr_to_member;

    members_.emplace_back(std::make_pair(name,
      detail::member_stub<&mmi, C, R, A...>));
    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::*ptr_to_member)(A...) const)
  {
    static detail::member_meta_info mmi;

    static_assert(sizeof(ptr_to_member) <= sizeof(mmi.ptr_to_member),
      "pointer size mismatch");

    *static_cast<decltype(ptr_to_member)*>(
      static_cast<void*>(&mmi.ptr_to_member))
      = ptr_to_member;

    members_.emplace_back(std::make_pair(name,
      detail::member_stub<&mmi, C, R, A...>));
    return *this;
  }

  class_& enum_(char const* const name, int value)
  {
    scope::enum_(name, value);
    return *this;
  }

private:
  detail::member_info_type constructors_;

  static detail::member_info_type members_;
};

template <class C>
detail::member_info_type class_<C>::members_;

}

#endif // LUALITE_HPP
