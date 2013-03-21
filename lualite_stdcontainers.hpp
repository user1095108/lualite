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
#ifndef LUALITE_STDCONTAINERS_HPP
# define LUALITE_STDCONTAINERS_HPP

#if __cplusplus < 201103L
# error "You need a C++11 compiler to use lualite"
#endif // __cplusplus

#include <cassert>

#include <array>

#include <deque>

#include <forward_list>

#include <list>

#include <map>

#include <unordered_map>

#include <string>

#include <vector>

namespace lualite
{

namespace detail
{

inline void set_result(lua_State* const  L,
  std::string const& value)
{
  lua_pushlstring(L, value.c_str(), value.size());
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

template <int I, class C>
inline typename std::enable_if<
  std::is_constructible<C, std::string>::value,
    std::string>::type
get_arg(lua_State* const L)
{
  assert(lua_isstring(L, I));

  std::size_t len;
  char const* const val(lua_tolstring(L, I, &len));

  return std::string(val, len);
}

template<int I, class C, class T, std::size_t N>
inline typename std::enable_if<
  std::is_constructible<C, std::array<T, N> >::value,
    std::array<T, N> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::array<T, N> result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result[i - 1] = get_arg<I + 1, T>(L);

    lua_pop(L, 1);
  }

  return result;
}

template <int I, class C, typename T, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::deque<T, Alloc> >::value,
    std::deque<T, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::deque<T, Alloc> result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.emplace_back(get_arg<I + 1, T>(L));

    lua_pop(L, 1);
  }

  return result;
}

template <int I, class C, typename T, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::forward_list<T, Alloc> >::value,
    std::forward_list<T, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::list<T, Alloc> result;

  static decltype(lua_rawlen(L, I)) const end(0);

  for (decltype(lua_rawlen(L, I)) i(lua_rawlen(L, I)); i != end; --i)
  {
    lua_rawgeti(L, I, i);

    result.emplace_front(get_arg<I + 1, T>(L));

    lua_pop(L, 1);
  }

  return result;
}

template <int I, class C, typename T, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::list<T, Alloc> >::value,
    std::list<T, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::list<T, Alloc> result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.emplace_back(get_arg<I + 1, T>(L));

    lua_pop(L, 1);
  }

  return result;
}

template <int I, class C, typename T, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::vector<T, Alloc> >::value,
    std::vector<T, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::vector<T, Alloc> result;

  auto const end(lua_rawlen(L, I) + 1);

  for (decltype(lua_rawlen(L, I)) i(1); i != end; ++i)
  {
    lua_rawgeti(L, I, i);

    result.emplace_back(get_arg<I + 1, T>(L));

    lua_pop(L, 1);
  }
  return result;
}

template <int I, class C, class Key, class T, class Compare, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::map<Key, T, Compare, Alloc> >::value,
    std::map<Key, T, Compare, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::map<Key, T, Compare, Alloc> result;

  lua_pushnil(L);

  while (lua_next(L, I))
  {
    result.emplace(get_arg<I + 1, Key>(L), get_arg<I + 2, T>(L));

    lua_pop(L, 1);
  }
  return result;
}

template <int I, class C, class Key, class T, class Hash, class Pred, class Alloc>
inline typename std::enable_if<
  std::is_constructible<C, std::unordered_map<Key, T, Hash, Pred, Alloc> >::value,
    std::unordered_map<Key, T, Hash, Pred, Alloc> >::type
get_arg(lua_State* const L)
{
  assert(lua_istable(L, I));

  std::unordered_map<Key, T, Hash, Pred, Alloc> result;

  lua_pushnil(L);

  while (lua_next(L, I))
  {
    result.emplace(get_arg<I + 1, Key>(L), get_arg<I + 2, T>(L));

    lua_pop(L, 1);
  }
  return result;
}

} // detail

} // lualite

#endif // LUALITE_STDCONTAINERS_HPP
