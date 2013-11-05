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

#include <cstring>

#include <stdexcept>

#include <type_traits>

#include <forward_list>

#include <unordered_map>

#include <vector>

#ifndef LUALITE_NO_STD_CONTAINERS

#include <array>

#include <deque>

#include <list>

#include <map>

#include <string>

#include <tuple>

#include <utility>

#endif // LUALITE_NO_STD_CONTAINERS

extern "C" {

# include "lua/lauxlib.h"

}

namespace lualite
{

class scope;

template <class C> class class_;

namespace detail
{

template<typename T> constexpr inline T const& as_const(T& t) { return t; }

template <typename T>
using is_nc_lvalue_reference
  = ::std::integral_constant<bool,
      ::std::is_lvalue_reference<T>{}
      && !::std::is_const<typename ::std::remove_reference<T>::type>{}
    >;

// key is at the top of the stack
inline void rawgetfield(lua_State* const L, int const index,
  char const* const key)
{
  lua_pushstring(L, key);
  lua_rawget(L, index >= 0 ? index : index - 1);
}

// value is at the top of the stack, key is shifted below the top
inline void rawsetfield(lua_State* const L, int const index,
  char const* const key)
{
  lua_pushstring(L, key);
  lua_insert(L, -2);
  lua_rawset(L, index >= 0 ? index : index - 1);
}

struct unordered_eq
{
  inline bool operator()(char const* const s1,
    char const* const s2) const noexcept
  {
    return !::std::strcmp(s1, s2);
  }
};

struct unordered_hash
{
  inline ::std::size_t operator()(char const* s) const noexcept
  {
    ::std::size_t h{};

    while (*s)
    {
      h ^= (h << 5) + (h >> 2) + static_cast<unsigned char>(*s++);
    }

    return h;
//  return ::std::hash<::std::string>()(s);
  }
};

template <::std::size_t...> struct indices
{
};

template <::std::size_t M, ::std::size_t... Is>
struct make_indices : make_indices<M - 1, M - 1, Is...>
{
};

template <::std::size_t... Is>
struct make_indices<0, Is...> : indices<Is...>
{
};

void dummy();

struct dummy_
{
  virtual void dummy();
};

struct alignas (decltype(&dummy)) func_type
{
  char p[sizeof(&dummy)];
};

struct alignas (decltype(&dummy_::dummy)) member_func_type
{
  char p[sizeof(&dummy_::dummy)];
};

using enum_info_type = ::std::vector<::std::pair<char const* const, int const> >;

struct func_info_type
{
  char const* name;

  lua_CFunction callback;

  void* func;
};

struct map_member_info_type
{
  lua_CFunction callback;

  member_func_type func;
};

struct member_info_type
{
  char const* name;

  lua_CFunction callback;

  member_func_type func;
};

template <class C>
int default_getter(lua_State* const L)
{
  assert(2 == lua_gettop(L));
  auto const i(lualite::class_<C>::getters_.find(lua_tostring(L, 2)));

  return lualite::class_<C>::getters_.end() == i
    ? 0
    : (lua_pushlightuserdata(L, &i->second.func),
      lua_replace(L, lua_upvalueindex(2)),
      (i->second.callback)(L));
}

template <class C>
int default_setter(lua_State* const L)
{
  assert(3 == lua_gettop(L));
  auto const i(lualite::class_<C>::setters_.find(lua_tostring(L, 2)));

  if (lualite::class_<C>::setters_.end() != i)
  {
    lua_pushlightuserdata(L, &i->second.func);
    lua_replace(L, lua_upvalueindex(2));

    (i->second.callback)(L);
  }
  // else do nothing

  return {};
}

template <class C>
inline void create_wrapper_table(lua_State* const L, C* const instance)
{
  lua_pushlightuserdata(L, instance);
  lua_rawget(L, LUA_REGISTRYINDEX);

  if (lua_isnil(L, -1))
  {
    lua_createtable(L, 0, 10);

    for (auto const i: detail::as_const(
      lualite::class_<C>::inherited_.inherited_defs))
    {
      for (auto& mi: *i)
      {
        assert(lua_istable(L, -1));

        lua_pushlightuserdata(L, instance);
        lua_pushlightuserdata(L, &mi.func);

        lua_pushcclosure(L, mi.callback, 2);

        rawsetfield(L, -2, mi.name);
      }
    }

    for (auto& mi: lualite::class_<C>::defs_)
    {
      assert(lua_istable(L, -1));

      lua_pushlightuserdata(L, instance);
      lua_pushlightuserdata(L, &mi.func);

      lua_pushcclosure(L, mi.callback, 2);

      rawsetfield(L, -2, mi.name);
    }

    // metatable
    assert(lua_istable(L, -1));
    lua_createtable(L, 0, 1);

    for (auto const i: detail::as_const(
      lualite::class_<C>::inherited_.inherited_metadefs))
    {
      for (auto& mi: *i)
      {
        assert(lua_istable(L, -1));

        lua_pushlightuserdata(L, instance);
        lua_pushlightuserdata(L, &mi.func);

        lua_pushcclosure(L, mi.callback, 2);

        rawsetfield(L, -2, mi.name);
      }
    }

    for (auto& mi: lualite::class_<C>::metadefs_)
    {
      assert(lua_istable(L, -1));

      if (::std::strcmp("__gc", mi.name))
      {
        lua_pushlightuserdata(L, instance);
        lua_pushlightuserdata(L, &mi.func);

        lua_pushcclosure(L, mi.callback, 2);

        rawsetfield(L, -2, mi.name);
      }
      // else do nothing
    }

    if (!lualite::class_<C>::has_index)
    {
      assert(lua_istable(L, -1));
      lua_pushlightuserdata(L, instance);
      lua_pushlightuserdata(L, 0);

      lua_pushcclosure(L, default_getter<C>, 2);

      rawsetfield(L, -2, "__index");
    }
    // else do nothing

    if (!lualite::class_<C>::has_newindex)
    {
      assert(lua_istable(L, -1));
      lua_pushlightuserdata(L, instance);
      lua_pushlightuserdata(L, nullptr);

      lua_pushcclosure(L, default_setter<C>, 2);

      rawsetfield(L, -2, "__newindex");
    }
    // else do nothing

    lua_setmetatable(L, -2);

    lua_pushlightuserdata(L, instance);
    lua_pushvalue(L, -2);
    lua_rawset(L, LUA_REGISTRYINDEX);
  }

  assert(lua_istable(L, -1));
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_floating_point<typename ::std::remove_reference<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T && v)
{
  lua_pushnumber(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_integral<typename ::std::remove_reference<T>::type>{}
  && ::std::is_signed<typename ::std::remove_reference<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushinteger(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_integral<typename ::std::remove_reference<T>::type>{}
  && !::std::is_signed<typename ::std::remove_reference<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushunsigned(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_same<typename ::std::decay<T>::type, bool>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushboolean(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_same<typename ::std::decay<T>::type, char const*>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushstring(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_same<typename ::std::decay<T>::type, void const*>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushlightuserdata(L, const_cast<void*>(v));

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_pointer<T>{}
  && !::std::is_const<typename ::std::remove_pointer<T>::type>{}
  && !::std::is_class<typename ::std::remove_pointer<T>::type>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushlightuserdata(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  is_nc_lvalue_reference<T>{}
  && !::std::is_class<typename ::std::remove_reference<T>::type>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  lua_pushlightuserdata(L, &v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  ::std::is_pointer<T>{}
  && !::std::is_const<typename ::std::remove_pointer<T>::type>{}
  && ::std::is_class<typename ::std::remove_pointer<T>::type>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  create_wrapper_table(L, v);

  return 1;
}

template <typename T>
inline typename ::std::enable_if<
  is_nc_lvalue_reference<T>{}
  && ::std::is_class<typename ::std::remove_reference<T>::type>{},
  int>::type
set_result(lua_State* const L, T&& v)
{
  create_wrapper_table(L, &v);

  return 1;
}

template <int I, typename T>
inline typename ::std::enable_if<
  ::std::is_floating_point<typename ::std::decay<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <int I, typename T>
inline typename ::std::enable_if<
  ::std::is_integral<typename ::std::decay<T>::type>{}
  && ::std::is_signed<typename ::std::decay<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I, typename T>
inline typename ::std::enable_if<
  ::std::is_integral<typename ::std::decay<T>::type>{}
  && ::std::is_unsigned<typename ::std::decay<T>::type>{}
  && !is_nc_lvalue_reference<T>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <int I, typename T>
inline typename ::std::enable_if<::std::is_same<
  typename ::std::decay<T>::type, bool>{}
  && !is_nc_lvalue_reference<T>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isboolean(L, I));
  return lua_toboolean(L, I);
}

template <int I, typename T>
inline typename ::std::enable_if<::std::is_same<
  typename ::std::decay<T>::type, char const*>{}
  && !is_nc_lvalue_reference<T>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isstring(L, I));
  return lua_tostring(L, I);
}

template <int I, typename T>
inline typename ::std::enable_if<
  ::std::is_pointer<T>{}
  && !::std::is_same<typename ::std::decay<T>::type, char const*>{},
  typename ::std::decay<T>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_islightuserdata(L, I));
  return static_cast<T>(lua_touserdata(L, I));
}

template <int I, typename T>
inline typename ::std::enable_if<is_nc_lvalue_reference<T>{}, T>::type
get_arg(lua_State* const L)
{
  assert(lua_islightuserdata(L, I));
  return *static_cast<typename ::std::remove_reference<T>::type*>(
    lua_touserdata(L, I));
}

#ifndef LUALITE_NO_STD_CONTAINERS

template <typename>
struct is_std_pair : ::std::false_type { };

template <class T1, class T2>
struct is_std_pair<::std::pair<T1, T2> > : ::std::true_type { };

template <typename>
struct is_std_array : ::std::false_type { };

template <typename T, ::std::size_t N>
struct is_std_array<::std::array<T, N> > : ::std::true_type { };

template <typename>
struct is_std_deque : ::std::false_type { };

template <typename T, class Alloc>
struct is_std_deque<::std::deque<T, Alloc> > : ::std::true_type { };

template <typename>
struct is_std_forward_list : ::std::false_type { };

template <typename T, class Alloc>
struct is_std_forward_list<::std::forward_list<T, Alloc> > : ::std::true_type { };

template <typename>
struct is_std_list : ::std::false_type { };

template <typename T, class Alloc>
struct is_std_list<::std::list<T, Alloc> > : ::std::true_type { };

template <typename>
struct is_std_map : ::std::false_type { };

template <class Key, class T, class Compare, class Alloc>
struct is_std_map<::std::map<Key, T, Compare, Alloc> > : ::std::true_type { };

template <typename>
struct is_std_unordered_map : ::std::false_type { };

template <class Key, class T, class Hash, class Pred, class Alloc>
struct is_std_unordered_map<::std::unordered_map<Key, T, Hash, Pred, Alloc> >
  : ::std::true_type { };

template <typename>
struct is_std_tuple : ::std::false_type { };

template <class ...Types>
struct is_std_tuple<::std::tuple<Types...> > : ::std::true_type { };

template <typename>
struct is_std_vector : ::std::false_type { };

template <typename T, class Alloc>
struct is_std_vector<::std::vector<T, Alloc> > : ::std::true_type { };

template <typename T>
inline typename ::std::enable_if<
  ::std::is_same<typename ::std::decay<T>::type, ::std::string>{}
  && !is_nc_lvalue_reference<T>{},
  int>::type
set_result(lua_State* const L, T&& s)
{
  lua_pushlstring(L, s.c_str(), s.size());

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_pair<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& p)
{
  set_result(L, p.first);
  set_result(L, p.second);

  return 2;
}

template <typename ...Types, ::std::size_t ...I>
inline void set_tuple_result(lua_State* const L,
  ::std::tuple<Types...> const& t, indices<I...> const)
{
  ::std::initializer_list<int>{ (set_result(L, ::std::get<I>(t)), 0)... };
}

template <typename C>
inline typename ::std::enable_if<
  is_std_tuple<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& t)
{
  using result_type = typename ::std::decay<C>::type;

  set_tuple_result(L, t, make_indices<::std::tuple_size<C>{}>());

  return ::std::tuple_size<result_type>{};
}

template <typename C>
inline typename ::std::enable_if<
  is_std_array<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& a)
{
  lua_createtable(L, a.size(), 0);

  auto const cend(a.cend());

  for (auto i(a.cbegin()); i != cend; ++i)
  {
    set_result(L, *i);

    lua_rawseti(L, -2, i - a.cbegin() + 1);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_deque<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& d)
{
  lua_createtable(L, d.size(), 0);

  auto const cend(d.cend());

  for (auto i(d.cbegin()); i != cend; ++i)
  {
    set_result(L, *i);

    lua_rawset(L, -2, i - d.cbegin() + 1);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_forward_list<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& l)
{
  lua_createtable(L, l.size(), 0);

  auto j(typename ::std::decay<C>::type::size_type(1));

  auto const cend(l.cend());

  for (auto i(l.cbegin()); i != cend; ++i, ++j)
  {
    set_result(L, *i);

    lua_rawseti(L, -2, j);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_list<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& l)
{
  lua_createtable(L, l.size(), 0);

  auto j(typename ::std::decay<C>::type::size_type(1));

  auto const cend(l.cend());

  for (auto i(l.cbegin()); i != cend; ++i, ++j)
  {
    set_result(L, *i);

    lua_rawseti(L, -2, j);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_map<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& m)
{
  lua_createtable(L, 0, m.size());

  auto const cend(m.cend());

  for (auto i(m.cbegin()); i != cend; ++i)
  {
    set_result(L, i->first);
    set_result(L, i->second);

    lua_rawset(L, -3);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_unordered_map<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& m)
{
  lua_createtable(L, 0, m.size());

  auto const cend(m.cend());

  for (auto i(m.cbegin()); i != cend; ++i)
  {
    set_result(L, i->first);
    set_result(L, i->second);

    lua_rawset(L, -3);
  }

  return 1;
}

template <typename C>
inline typename ::std::enable_if<
  is_std_vector<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  int>::type
set_result(lua_State* const L, C&& v)
{
  lua_createtable(L, v.size(), 0);

  auto const cend(v.cend());

  for (auto i(v.cbegin()); i != cend; ++i)
  {
    set_result(L, *i);

    lua_rawseti(L, -2, i - v.cbegin() + 1);
  }

  return 1;
}

template <int I, class C>
inline typename ::std::enable_if<
  ::std::is_same<typename ::std::decay<C>::type, ::std::string>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_isstring(L, I));

  ::std::size_t len;

  char const* const val(lua_tolstring(L, I, &len));

  return { val, len };
}

template<int I, class C>
inline typename ::std::enable_if<
  is_std_pair<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));
 
  using result_type = typename ::std::decay<C>::type;

  lua_rawgeti(L, -1, 1);
  lua_rawgeti(L, -2, 2);

  result_type const result(
    get_arg<-2, typename result_type::first_type>(L),
    get_arg<-1, typename result_type::second_type>(L));

  lua_pop(L, 2);

  return result;
}

template <::std::size_t O, class C, ::std::size_t ...I>
inline C get_tuple_arg(lua_State* const L, indices<I...> const)
{
  ::std::initializer_list<int>{ (lua_rawgeti(L, O, I + 1), 0)... };

  C result(::std::make_tuple(get_arg<int(I - sizeof...(I)),
    typename ::std::tuple_element<I, C>::type>(L)...));

  lua_pop(L, int(sizeof...(I)));

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_tuple<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;

  return get_tuple_arg<I, result_type>(L,
    make_indices<::std::tuple_size<result_type>{}>());
}

template<int I, class C>
inline typename ::std::enable_if<
  is_std_array<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  auto const end(::std::min(lua_rawlen(L, I), result.size()) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result[i - 1] = get_arg<-1, typename result_type::value_type>(L);
  }

  lua_pop(L, end - 1);

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_deque<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.push_back(get_arg<-1, typename result_type::value_type>(L));
  }

  lua_pop(L, end - 1);

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_forward_list<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  auto const len(lua_rawlen(L, I));

  for (auto i(len); i; --i)
  {
    lua_rawgeti(L, I, i);

    result.emplace_front(get_arg<-1, typename result_type::value_type>(L));
  }

  lua_pop(L, len);

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_list<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.push_back(get_arg<-1, typename result_type::value_type>(L));
  }

  lua_pop(L, end - 1);

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_vector<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.push_back(get_arg<-1, typename result_type::value_type>(L));
  }

  lua_pop(L, end - 1);

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_map<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  lua_pushnil(L);

  while (lua_next(L, I))
  {
    result.emplace(get_arg<-2, typename result_type::key_type>(L),
      get_arg<-1, typename result_type::mapped_type>(L));

    lua_pop(L, 1);
  }

  return result;
}

template <int I, class C>
inline typename ::std::enable_if<
  is_std_unordered_map<typename ::std::decay<C>::type>{}
  && !is_nc_lvalue_reference<C>{},
  typename ::std::decay<C>::type>::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  using result_type = typename ::std::decay<C>::type;
  result_type result;

  lua_pushnil(L);

  while (lua_next(L, I))
  {
    result.emplace(get_arg<-2, typename result_type::key_type>(L),
      get_arg<-1, typename result_type::mapped_type>(L));

    lua_pop(L, 1);
  }

  return result;
}

#endif // LUALITE_NO_STD_CONTAINERS

template <class C>
int default_finalizer(lua_State* const L)
{
  auto const instance(lua_touserdata(L, lua_upvalueindex(1)));

  lua_pushlightuserdata(L, instance);
  lua_pushnil(L);
  lua_rawset(L, LUA_REGISTRYINDEX);

  delete static_cast<C*>(instance);

  return {};
}

template <::std::size_t O, typename C, typename ...A, ::std::size_t ...I>
constexpr inline C* forward(lua_State* const L, indices<I...> const)
{
  return new C(get_arg<I + O, A>(L)...);
}

template <::std::size_t O, class C, class ...A>
int constructor_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  auto const instance(forward<O, C, A...>(L, make_indices<sizeof...(A)>()));

  // table
  lua_createtable(L, 0, 10);

  for (auto const i: detail::as_const(
    lualite::class_<C>::inherited_.inherited_defs))
  {
    for (auto& mi: *i)
    {
      assert(lua_istable(L, -1));
      lua_pushlightuserdata(L, instance);
      lua_pushlightuserdata(L, &mi.func);

      lua_pushcclosure(L, mi.callback, 2);

      rawsetfield(L, -2, mi.name);
    }
  }

  for (auto& mi: lualite::class_<C>::defs_)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushlightuserdata(L, &mi.func);

    lua_pushcclosure(L, mi.callback, 2);

    rawsetfield(L, -2, mi.name);
  }

  // metatable
  assert(lua_istable(L, -1));
  lua_createtable(L, 0, 1);

  for (auto const i: detail::as_const(
    lualite::class_<C>::inherited_.inherited_metadefs))
  {
    for (auto& mi: *i)
    {
      assert(lua_istable(L, -1));
      lua_pushlightuserdata(L, instance);
      lua_pushlightuserdata(L, &mi.func);

      lua_pushcclosure(L, mi.callback, 2);
    
      rawsetfield(L, -2, mi.name);
    }
  }

  for (auto& mi: lualite::class_<C>::metadefs_)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushlightuserdata(L, &mi.func);

    lua_pushcclosure(L, mi.callback, 2);
  
    rawsetfield(L, -2, mi.name);
  }

  if (!lualite::class_<C>::has_gc)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);

    lua_pushcclosure(L, default_finalizer<C>, 1);

    rawsetfield(L, -2, "__gc");
  }
  // else do nothing

  if (!lualite::class_<C>::has_index)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushnil(L);

    lua_pushcclosure(L, default_getter<C>, 2);

    rawsetfield(L, -2, "__index");
  }
  // else do nothing

  if (!lualite::class_<C>::has_newindex)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushlightuserdata(L, nullptr);

    lua_pushcclosure(L, default_setter<C>, 2);

    rawsetfield(L, -2, "__newindex");
  }
  // else do nothing

  assert(lua_istable(L, -2));
  lua_setmetatable(L, -2);
  assert(lua_istable(L, -1));

  return 1;
}

template <::std::size_t O, typename R, typename ...A, ::std::size_t ...I>
inline R forward(lua_State* const L, R (* const f)(A...), indices<I...> const)
{
  return (*f)(get_arg<I + O, A>(L)...);
}

template <::std::size_t O, class R, class ...A>
typename ::std::enable_if<::std::is_void<R>{}, int>::type
func_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));
  using ptr_to_func_type = R (* const)(A...);

  forward<O, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      lua_touserdata(L, lua_upvalueindex(1))),
    make_indices<sizeof...(A)>());

  return {};
}

template <::std::size_t O, class R, class ...A>
typename ::std::enable_if<!::std::is_void<R>{}, int>::type
func_stub(lua_State* const L)
{
//::std::cout << lua_gettop(L) << " " << sizeof...(A) << ::std::endl;
  assert(sizeof...(A) == lua_gettop(L));
  using ptr_to_func_type = R (* const)(A...);

  return set_result(L, forward<O, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      lua_touserdata(L, lua_upvalueindex(1))),
    make_indices<sizeof...(A)>()));
}

template <::std::size_t O, typename C, typename R,
  typename ...A, ::std::size_t ...I>
inline R forward(lua_State* const L, C* const c,
  R (C::* const ptr_to_member)(A...), indices<I...> const)
{
  return (c->*ptr_to_member)(get_arg<I + O, A>(L)...);
}

template <::std::size_t O, class C, class R, class ...A>
typename ::std::enable_if<::std::is_void<R>{}, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + O - 1 == lua_gettop(L));
  using ptr_to_member_type = R (C::* const )(A...);

  forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
    *static_cast<ptr_to_member_type const*>(
      lua_touserdata(L, lua_upvalueindex(2))),
    make_indices<sizeof...(A)>());

  return {};
}

template <::std::size_t O, class C, class R, class ...A>
typename ::std::enable_if<!::std::is_void<R>{}, int>::type
member_stub(lua_State* const L)
{
//::std::cout << lua_gettop(L) << " " << sizeof...(A) + O - 1 << ::std::endl;
  assert(sizeof...(A) + O - 1 == lua_gettop(L));
  using ptr_to_member_type = R (C::* const)(A...);

  return set_result(L, static_cast<R>(forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
    *static_cast<ptr_to_member_type const*>(
      lua_touserdata(L, lua_upvalueindex(2))),
    make_indices<sizeof...(A)>())));
}

} // detail

#ifdef __GNUC__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif // __GNUC__

template <class R, class ...A>
inline constexpr detail::func_type convert(
  R (*func_ptr)(A...))
{
  return *static_cast<detail::func_type*>(
    static_cast<void*>(&func_ptr));
}

template <class C, class R, class ...A>
inline constexpr detail::member_func_type convert(
  R (C::*func_ptr)(A...))
{
  return *static_cast<detail::member_func_type*>(
    static_cast<void*>(&func_ptr));
}

template <class C, class R, class ...A>
inline constexpr detail::member_func_type convert(
  R (C::*func_ptr)(A...) const)
{
  return *static_cast<detail::member_func_type*>(
    static_cast<void*>(&func_ptr));
}

#ifdef __GNUC__
# pragma GCC diagnostic pop
#endif // __GNUC__

class scope
{
public:
  scope(char const* const name) :
    name_(name)
  {
  }

  template <typename ...A>
  scope(char const* const name, A&&... args) :
    name_(name)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);
  }

  scope(scope const&) = delete;

  scope& operator=(scope const&) = delete;

  template <class R, class ...A>
  scope& def(char const* const name, R (* const ptr_to_func)(A...))
  {
    address_pool().push_front(convert(ptr_to_func));

    functions_.push_back(detail::func_info_type{
      name, detail::func_stub<1, R, A...>, &address_pool().front()});

    return *this;
  }

  scope& enum_(char const* const name, int value)
  {
    enums_.push_back(::std::make_pair(name, value));

    return *this;
  }

protected:
  virtual void apply(lua_State* const L)
  {
    if (parent_scope_)
    {
      scope::get_scope(L);
      assert(lua_istable(L, -1));

      for (auto& i: detail::as_const(enums_))
      {
        assert(lua_istable(L, -1));
        lua_pushinteger(L, i.second);

        detail::rawsetfield(L, -2, i.first);
      }

      for (auto& i: detail::as_const(functions_))
      {
        assert(lua_istable(L, -1));
        lua_pushlightuserdata(L, i.func);
        lua_pushcclosure(L, i.callback, 1);

        detail::rawsetfield(L, -2, i.name);
      }

      lua_pop(L, 1);
    }
    else
    {
      for (auto& i: detail::as_const(enums_))
      {
        lua_pushinteger(L, i.second);

        lua_setglobal(L, i.first);
      }

      for (auto& i: detail::as_const(functions_))
      {
        lua_pushlightuserdata(L, i.func);
        lua_pushcclosure(L, i.callback, 1);

        lua_setglobal(L, i.name);
      }

      auto next(next_);

      while (next)
      {
        next->apply(L);

        next = next->next_;
      }
    }

    assert(!lua_gettop(L));
  }

  void set_apply_instance(scope* const instance)
  {
    auto next(next_);

    if (next)
    {
      while (next->next_)
      {
        next = next->next_;
      }

      next->next_ = instance;
    }
    else
    {
      next_ = instance;
    }
  }

  void set_parent_scope(scope* const parent_scope)
  {
    parent_scope_ = parent_scope;

    parent_scope->set_apply_instance(this);
  }

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
          detail::rawsetfield(L, -2, name_);
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

  using address_pool_type = ::std::forward_list<detail::func_type>;

  static address_pool_type& address_pool()
  {
    static address_pool_type address_pool;

    return address_pool;
  }

protected:
  char const* const name_;

  scope* parent_scope_{};

  ::std::vector<detail::func_info_type> functions_;

private:
  friend class module;

  bool scope_create_{true};

  detail::enum_info_type enums_;

  scope* next_{};
};

class module : public scope
{
public:
  template <typename ...A>
  module(lua_State* const L, A&&... args) :
    scope(0),
    L_(L)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);

    scope::apply(L);
  }

  template <typename ...A>
  module(lua_State* const L, char const* const name, A&&... args) :
    scope(name),
    L_(L)
  {
    [](...){ }((args.set_parent_scope(this), 0)...);

    scope::apply(L);
  }

  template <class R, class ...A>
  module& def(char const* const name, R (* const ptr_to_func)(A...))
  {
    address_pool().push_front(convert(ptr_to_func));

    if (name_)
    {
      scope::get_scope(L_);
      assert(lua_istable(L_, -1));

      lua_pushlightuserdata(L_, &address_pool().front());
      lua_pushcclosure(L_, (detail::func_stub<1, R, A...>), 1);

      detail::rawsetfield(L_, -2, name);

      lua_pop(L_, 1);
    }
    else
    {
      lua_pushlightuserdata(L_, &address_pool().front());
      lua_pushcclosure(L_, (detail::func_stub<1, R, A...>), 1);

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
      detail::rawsetfield(L_, -2, name);

      lua_pop(L_, 1);
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

  class_& constructor(char const* const name = "new")
  {
    return constructor<>(name);
  }

  template <class ...A>
  class_& constructor(char const* const name = "new")
  {
    constructors_.push_back(detail::func_info_type{
      name, detail::constructor_stub<1, C, A...> });

    return *this;
  }

  template <class ...A>
  class_& inherits()
  {
    ::std::initializer_list<int>{(
      inherited_.inherited_defs.push_back(&class_<A>::defs_),
      inherited_.inherited_metadefs.push_back(&class_<A>::metadefs_),
      0)...};

    ::std::initializer_list<int>{(
      getters_.insert(class_<A>::getters_.cbegin(),
        class_<A>::getters_.cend()),
      0)...};

    ::std::initializer_list<int>{(
      setters_.insert(class_<A>::setters_.cbegin(),
        class_<A>::setters_.cend()),
      0)...};

    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (* const ptr_to_func)(A...))
  {
    scope::def(name, ptr_to_func);

    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::* const ptr_to_member)(A...))
  {
    member_function(name, ptr_to_member);

    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::* const ptr_to_member)(A...) const)
  {
    const_member_function(name, ptr_to_member);

    return *this;
  }

  class_& enum_(char const* const name, int const value)
  {
    scope::enum_(name, value);

    return *this;
  }

  template <class R, class ...A>
  class_& metadef(char const* const name, R (C::* const ptr_to_member)(A...))
  {
    has_gc = has_gc || !::std::strcmp("__gc", name);

    has_index = has_index || !::std::strcmp("__index", name);

    has_newindex = has_newindex || !::std::strcmp("__newindex", name);

    member_function(name, ptr_to_member);

    return *this;
  }

  template <class R, class ...A>
  class_& metadef(char const* const name,
    R (C::* const ptr_to_member)(A...) const)
  {
    has_gc = has_gc || !::std::strcmp("__gc", name);

    has_index = has_index || !::std::strcmp("__index", name);

    has_newindex = has_newindex || !::std::strcmp("__newindex", name);

    const_member_function(name, ptr_to_member);

    return *this;
  }

  template <class R, class ...A>
  class_& property(char const* const name,
    R (C::* const ptr_to_const_member)(A...) const)
  {
    getters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, R, A...>, convert(ptr_to_const_member)});

    return *this;
  }

  template <class R, class ...A>
  class_& property(char const* const name,
    R (C::* const ptr_to_member)(A...))
  {
    getters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, R, A...>, convert(ptr_to_member)});

    return *this;
  }

  template <class RA, class ...A, class RB, class ...B>
  class_& property(char const* const name,
    RA (C::* const ptr_to_membera)(A...) const,
    RB (C::* const ptr_to_memberb)(B...))
  {
    getters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, RA, A...>, convert(ptr_to_membera)});
    setters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, RB, B...>, convert(ptr_to_memberb)});

    return *this;
  }

  template <class RA, class ...A, class RB, class ...B>
  class_& property(char const* const name,
    RA (C::* const ptr_to_membera)(A...),
    RB (C::* const ptr_to_memberb)(B...))
  {
    getters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, RA, A...>, convert(ptr_to_membera)});
    setters_.emplace(name, detail::map_member_info_type{
      detail::member_stub<3, C, RB, B...>, convert(ptr_to_memberb)});

    return *this;
  }

private:
  void apply(lua_State* const L)
  {
    assert(parent_scope_);
    scope::apply(L);

    scope::get_scope(L);
    assert(lua_istable(L, -1));

    for (auto& i: constructors_)
    {
      assert(lua_istable(L, -1));

      lua_pushcfunction(L, i.callback);

      detail::rawsetfield(L, -2, i.name);
    }

    lua_pushstring(L, name_);
    detail::rawsetfield(L, -2, "__classname");

    lua_pop(L, 1);

    assert(!lua_gettop(L));
  }

  template <::std::size_t O = 2, class R, class ...A>
  void member_function(char const* const name,
    R (C::* const ptr_to_member)(A...))
  {
    static_assert(sizeof(ptr_to_member) <= sizeof(detail::member_func_type),
      "pointer size mismatch");

    defs_.push_back(detail::member_info_type{name,
      detail::member_stub<O, C, R, A...>, convert(ptr_to_member)});
  }

  template <::std::size_t O = 2, class R, class ...A>
  void const_member_function(char const* const name,
    R (C::* const ptr_to_member)(A...) const)
  {
    static_assert(sizeof(ptr_to_member) <= sizeof(detail::member_func_type),
      "pointer size mismatch");

    defs_.push_back(detail::member_info_type{name,
      detail::member_stub<O, C, R, A...>, convert(ptr_to_member)});
  }

public:
  struct inherited_info
  {
    ::std::vector<::std::vector<detail::member_info_type>*> inherited_defs;
    ::std::vector<::std::vector<detail::member_info_type>*> inherited_metadefs;
  };

  static struct inherited_info inherited_;

  static bool has_gc;
  static bool has_index;
  static bool has_newindex;

  static ::std::vector<detail::func_info_type> constructors_;

  static ::std::vector<detail::member_info_type> defs_;

  static ::std::vector<detail::member_info_type> metadefs_;

  static ::std::unordered_map<char const*, detail::map_member_info_type,
    detail::unordered_hash, detail::unordered_eq> getters_;
  static ::std::unordered_map<char const*, detail::map_member_info_type,
    detail::unordered_hash, detail::unordered_eq> setters_;
};

template <class C>
struct class_<C>::inherited_info class_<C>::inherited_;

template <class C>
bool class_<C>::has_gc;

template <class C>
bool class_<C>::has_index;

template <class C>
bool class_<C>::has_newindex;

template <class C>
::std::vector<detail::func_info_type> class_<C>::constructors_;

template <class C>
::std::vector<detail::member_info_type> class_<C>::defs_;

template <class C>
::std::vector<detail::member_info_type> class_<C>::metadefs_;

template <class C>
::std::unordered_map<char const*, detail::map_member_info_type,
  detail::unordered_hash, detail::unordered_eq> class_<C>::getters_;

template <class C>
::std::unordered_map<char const*, detail::map_member_info_type,
  detail::unordered_hash, detail::unordered_eq> class_<C>::setters_;

} // lualite

#endif // LUALITE_HPP
