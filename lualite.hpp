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

#include <cstring>

#include <stdexcept>

#include <type_traits>

#include <utility>

#include <vector>

extern "C" {

# include "lua/lauxlib.h"

}

namespace lualite
{

class scope;

template <class C>
class class_;

namespace detail
{

inline void rawsetfield(lua_State* const L, int const index,
  char const* const key)
{
  lua_pushstring(L, key);
  lua_insert(L, -2);
  lua_rawset(L, index >= 0 ? index : index - 1);
}

struct unordered_eq
{
  bool operator()(const char* const s1, const char* const s2) const
  {
    return !std::strcmp(s1, s2);
  }
};

struct unordered_hash
{
  std::size_t operator()(char const* s) const
  {
    std::size_t h(0);

    while (*s)
    {
      h ^= (h << 5) + (h >> 2) + static_cast<unsigned char>(*s++);
    }

    return h;
//  return std::hash<std::string>()(s);
  }
};

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

struct member_info_type
{
  char const* name;

  lua_CFunction func;

  member_info_type* next;
};

typedef std::vector<std::pair<char const*, lua_CFunction> > func_info_type;

void dummy();

struct dummy_
{
  void dummy();
};

struct class_meta_info
{
  char const* class_name;

  member_info_type** firstdef;

  member_info_type** firstmetadef;
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
  lua_pushinteger(L, value);
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
  char const* const value)
{
  lua_pushstring(L, value);
}

inline void set_result(lua_State* const L,
  void* const value)
{
  lua_pushlightuserdata(L, value);
}

template <int I>
inline long double get_arg(lua_State* const L,
  long double const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <int I>
inline double get_arg(lua_State* const L,
  double const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <int I>
inline float get_arg(lua_State* const L,
  float const)
{
  assert(lua_isnumber(L, I));
  return lua_tonumber(L, I);
}

template <int I>
inline long long get_arg(lua_State* const L,
  long long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline unsigned long long get_arg(lua_State* const L,
  unsigned long long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline long get_arg(lua_State* const L,
  long const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline unsigned long get_arg(lua_State* const L,
  unsigned long const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <int I>
inline int get_arg(lua_State* const L,
  int const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline unsigned int get_arg(lua_State* const L,
  unsigned int const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <int I>
inline signed char get_arg(lua_State* const L,
  signed char const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline unsigned char get_arg(lua_State* const L,
  unsigned char const)
{
  assert(lua_isnumber(L, I));
  return lua_tounsigned(L, I);
}

template <int I>
inline char get_arg(lua_State* const L,
  char const)
{
  assert(lua_isnumber(L, I));
  return lua_tointeger(L, I);
}

template <int I>
inline bool get_arg(lua_State* const L,
  bool const)
{
  assert(lua_isboolean(L, I));
  return lua_toboolean(L, I);
}

template <int I>
inline char const* get_arg(lua_State* const L,
  char const* const)
{
  assert(lua_isstring(L, I));
  return lua_tostring(L, I);
}

template <int I>
inline void* get_arg(lua_State* const L,
  void* const)
{
  assert(lua_islightuserdata(L, I));
  return lua_touserdata(L, I);
}

template <class C>
int default_getter(lua_State* const L)
{
  assert(2 == lua_gettop(L));

  auto i(lualite::class_<C>::getters_.find(lua_tostring(L, 2)));

  if (i == lualite::class_<C>::getters_.end())
  {
    return 0;
  }
  else
  {
    return (i->second)(L);
  }
}

template <class C>
int default_setter(lua_State* const L)
{
  assert(3 == lua_gettop(L));

  auto i(lualite::class_<C>::setters_.find(lua_tostring(L, 2)));

  if (i != lualite::class_<C>::setters_.end())
  {
    (i->second)(L);
  }
  // else do nothing

  return 0;
}


template <class C>
int default_finalizer(lua_State* const L)
{
  delete static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1)));
  return 0;
}

template <std::size_t O, typename C, typename ...A, std::size_t ...I>
inline C* forward(lua_State* const L, indices<I...>)
{
  return new C(get_arg<I + O>(L, A())...);
}

template <class_meta_info const* cmi_ptr, std::size_t O, class C, class ...A>
inline int constructor_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  C* const instance(forward<O, C, A...>(L, indices_type()));

  // table
  lua_createtable(L, 0, 1);

  detail::member_info_type* mi_ptr(*cmi_ptr->firstdef);

  while (mi_ptr)
  {
    assert(lua_istable(L, -1));

    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, mi_ptr->func, 1);
  
    rawsetfield(L, -2, mi_ptr->name);

    mi_ptr = mi_ptr->next;
  }

  lua_pushlightuserdata(L, const_cast<void*>(
    static_cast<void const*>(cmi_ptr)));
  rawsetfield(L, -2, "__cmi_ptr");

  lua_pushstring(L, cmi_ptr->class_name);
  rawsetfield(L, -2, "__instanceof");

  // metatable
  lua_createtable(L, 0, 1);

  mi_ptr = *cmi_ptr->firstmetadef;

  while (mi_ptr)
  {
    assert(lua_istable(L, -1));

    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, mi_ptr->func, 1);
  
    rawsetfield(L, -2, mi_ptr->name);

    mi_ptr = mi_ptr->next;
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
    lua_pushcclosure(L, default_getter<C>, 1);

    rawsetfield(L, -2, "__index");
  }
  // else do nothing

  if (!lualite::class_<C>::has_newindex)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, default_setter<C>, 1);

    rawsetfield(L, -2, "__newindex");
  }
  // else do nothing

  assert(lua_istable(L, -2));
  lua_setmetatable(L, -2);
  return 1;
}

template <std::size_t O, typename R, typename ...A, std::size_t ...I>
inline R forward(lua_State* const L, R (*f)(A...), indices<I...>)
{
  return (*f)(get_arg<I + O>(L, A())...);
}

template <func_meta_info const* fmi_ptr, std::size_t O, class R, class ...A>
inline typename std::enable_if<std::is_same<void, R>::value, int>::type
func_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef R (*ptr_to_func_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  forward<O, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      static_cast<void const*>(&fmi_ptr->ptr_to_func)),
    indices_type());
  return 0;
}

template <func_meta_info const* fmi_ptr, std::size_t O, class R, class ...A>
inline typename std::enable_if<!std::is_same<void, R>::value, int>::type
func_stub(lua_State* const L)
{
  assert(sizeof...(A) == lua_gettop(L));

  typedef R (*ptr_to_func_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  set_result(L, forward<O, R, A...>(L,
    *static_cast<ptr_to_func_type const*>(
      static_cast<void const*>(&fmi_ptr->ptr_to_func)),
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

template <member_meta_info const* mmi, std::size_t O, class C, class R, class ...A>
inline typename std::enable_if<std::is_same<void, R>::value
  && !std::is_pointer<R>::value && !std::is_reference<R>::value, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + O - 1 == lua_gettop(L));

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
      *static_cast<ptr_to_member_type const*>(
        static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type());
  return 0;
}

template <member_meta_info const* mmi, std::size_t O, class C, class R, class ...A>
inline typename std::enable_if<!std::is_same<void, R>::value
  && !std::is_pointer<R>::value && !std::is_reference<R>::value, int>::type
member_stub(lua_State* const L)
{
//std::cout << lua_gettop(L) << " " << sizeof...(A) + O - 1 << std::endl;
  assert(sizeof...(A) + O - 1 == lua_gettop(L));

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  set_result(L, forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
    *static_cast<ptr_to_member_type const*>(
      static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type()));
  return 1;
}

template <member_meta_info const* mmi, std::size_t O, class C, class R, class ...A>
inline typename std::enable_if<
  std::is_pointer<R>::value
  && std::is_class<
    typename std::remove_const<
      typename std::remove_pointer<R>::type
    >::type
  >::value, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + O - 1 == lua_gettop(L));
  assert(lua_istable(L, -1));

  typedef typename std::remove_const<
    typename std::remove_pointer<R>::type
  >::type T;

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  T* const instance(const_cast<T*>(forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
      *static_cast<ptr_to_member_type const*>(
        static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type())));

  lua_pushliteral(L, "__cmi_ptr");
  lua_rawget(L, -2);

  auto const cmi_ptr(static_cast<class_meta_info*>(lua_touserdata(L, -1)));
  lua_pop(L, 1);

  detail::member_info_type* mi_ptr(*cmi_ptr->firstdef);

  while (mi_ptr)
  {
    assert(lua_istable(L, -1));

    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, mi_ptr->func, 1);

    rawsetfield(L, -2, mi_ptr->name);

    mi_ptr = mi_ptr->next;
  }

  assert(lua_istable(L, -1));
  lua_pushstring(L, cmi_ptr->class_name);
  rawsetfield(L, -2, "__instanceof");

  // metatable
  lua_createtable(L, 0, 1);

  mi_ptr = *cmi_ptr->firstmetadef;

  while (mi_ptr)
  {
    assert(lua_istable(L, -1));

    if (std::strcmp("__gc", mi_ptr->name))
    {
      lua_pushlightuserdata(L, instance);
      lua_pushcclosure(L, mi_ptr->func, 1);

      rawsetfield(L, -2, mi_ptr->name);
    }
    // else do nothing

    mi_ptr = mi_ptr->next;
  }

  if (!lualite::class_<C>::has_index)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, default_getter<C>, 1);

    rawsetfield(L, -2, "__index");
  }
  // else do nothing

  if (!lualite::class_<C>::has_newindex)
  {
    assert(lua_istable(L, -1));
    lua_pushlightuserdata(L, instance);
    lua_pushcclosure(L, default_setter<C>, 1);

    rawsetfield(L, -2, "__newindex");
  }
  // else do nothing

  lua_setmetatable(L, -2);

  assert(lua_istable(L, -1));
  return 1;
}

template <member_meta_info const* mmi, std::size_t O, class C, class R, class ...A>
inline typename std::enable_if<
  std::is_reference<R>::value
  && std::is_class<
    typename std::remove_const<
      typename std::remove_reference<R>::type
    >::type
  >::value, int>::type
member_stub(lua_State* const L)
{
  assert(sizeof...(A) + O - 1 == lua_gettop(L));
  assert(lua_istable(L, -1));

  typedef typename std::remove_const<
    typename std::remove_reference<R>::type >::type T;

  typedef R (C::*ptr_to_member_type)(A...);

  typedef typename make_indices<sizeof...(A)>::type indices_type;

  T* const instance(&const_cast<T&>(forward<O, C, R, A...>(L,
    static_cast<C*>(lua_touserdata(L, lua_upvalueindex(1))),
      *static_cast<ptr_to_member_type const*>(
        static_cast<void const*>(&mmi->ptr_to_member)),
    indices_type())));

  lua_pushliteral(L, "__cmi_ptr");
  lua_rawget(L, -2);

  auto const cmi_ptr(static_cast<class_meta_info*>(lua_touserdata(L, -1)));
  lua_pop(L, 1);

  assert(lua_istable(L, -1));
  lua_pushstring(L, cmi_ptr->class_name);
  rawsetfield(L, -2, "__instanceof");

  detail::member_info_type* mi_ptr(*cmi_ptr->firstdef);

  while (mi_ptr)
  {
    assert(lua_istable(L, -1));

    if (std::strcmp("__gc", mi_ptr->name))
    {
      lua_pushlightuserdata(L, instance);
      lua_pushcclosure(L, mi_ptr->func, 1);

      rawsetfield(L, -2, mi_ptr->name);
    }
    // else do nothing

    mi_ptr = mi_ptr->next;
  }

  // metatable
  if ((mi_ptr = *cmi_ptr->firstmetadef))
  {
    lua_createtable(L, 0, 1);

    while (mi_ptr)
    {
      assert(lua_istable(L, -1));

      lua_pushlightuserdata(L, instance);
      lua_pushcclosure(L, mi_ptr->func, 1);

      rawsetfield(L, -2, mi_ptr->name);

      mi_ptr = mi_ptr->next;
    }

    lua_setmetatable(L, -2);
  }
  // else do nothing

  assert(lua_istable(L, -1));
  lua_pushlightuserdata(L, instance);
  lua_pushcclosure(L, default_getter<C>, 1);

  rawsetfield(L, -2, "__index");

  assert(lua_istable(L, -1));
  lua_pushlightuserdata(L, instance);
  lua_pushcclosure(L, default_setter<C>, 1);

  rawsetfield(L, -2, "__newindex");

  assert(lua_istable(L, -1));
  return 1;
}

} // detail

class scope
{
public:
  scope(char const* const name)
  : name_(name),
    parent_scope_(0),
    next_(0),
    scope_create_(true)
  {
  }

  template <typename ...A>
  scope(char const* const name, A&&... args)
  : name_(name),
    parent_scope_(0),
    next_(0),
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
        detail::rawsetfield(L, -2, i.first);
      }

      for (auto const& i: functions_)
      {
        assert(lua_istable(L, -1));

        lua_pushcfunction(L, i.second);
        detail::rawsetfield(L, -2, i.first);
      }

      lua_pop(L, 1);
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
      std::make_pair(name, detail::func_stub<&fmi, 1, R, A...>));
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

protected:
  char const* const name_;

  scope* parent_scope_;

  scope* next_;

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

      lua_pushcfunction(L_, (detail::func_stub<&fmi, 1, R, A...>));
      detail::rawsetfield(L_, -2, name);

      lua_pop(L_, 1);
    }
    else
    {
      lua_pushcfunction(L_, (detail::func_stub<&fmi, 1, R, A...>));
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
    : scope(name),
      lastdef_(0),
      lastmetadef_(0)
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
      detail::rawsetfield(L, -2, i.first);
    }

    lua_pushstring(L, name_);
    detail::rawsetfield(L, -2, "__classname");

    lua_pop(L, 1);

    assert(!lua_gettop(L));
  }

  template <class ...A>
  class_& constructor()
  {
    static detail::class_meta_info const cmi {
      name_,
      &firstdef_,
      &firstmetadef_
    };

    constructors_.emplace_back(std::make_pair("new",
      detail::constructor_stub<&cmi, 1, C, A...>));
    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (*ptr_to_func)(A...))
  {
    scope::def(name, ptr_to_func);
    return *this;
  }

  template <std::size_t O = 2, class R, class ...A>
  void member_function(detail::member_info_type*& first,
    detail::member_info_type*& last, char const* const name,
    R (C::*ptr_to_member)(A...))
  {
    static detail::member_meta_info mmi;

    static_assert(sizeof(ptr_to_member) <= sizeof(mmi.ptr_to_member),
      "pointer size mismatch");

    static detail::member_info_type mi{name,
      detail::member_stub<&mmi, O, C, R, A...>,
      0};

    if (!mi.next)
    {
      *static_cast<decltype(ptr_to_member)*>(
        static_cast<void*>(&mmi.ptr_to_member))
        = ptr_to_member;

      if (last)
      {
        last->next = &mi;
      }
      else
      {
        first = &mi;
      }
    }
    // else do nothing

    last = &mi;
  }

  template <std::size_t O = 2, class R, class ...A>
  void const_member_function(detail::member_info_type*& first,
    detail::member_info_type*& last, char const* const name,
    R (C::*ptr_to_member)(A...) const)
  {
    static detail::member_meta_info mmi;

    static_assert(sizeof(ptr_to_member) <= sizeof(mmi.ptr_to_member),
      "pointer size mismatch");

    static detail::member_info_type mi{name,
      detail::member_stub<&mmi, O, C, R, A...>,
      0};

    if (!mi.next)
    {
      *static_cast<decltype(ptr_to_member)*>(
        static_cast<void*>(&mmi.ptr_to_member))
        = ptr_to_member;

      if (last)
      {
        last->next = &mi;
      }
      else
      {
        first = &mi;
      }
    }
    // else do nothing

    last = &mi;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::*ptr_to_member)(A...))
  {
    member_function(firstdef_, lastdef_, name, ptr_to_member);
    return *this;
  }

  template <class R, class ...A>
  class_& def(char const* const name, R (C::*ptr_to_member)(A...) const)
  {
    const_member_function(firstdef_, lastdef_, name, ptr_to_member);
    return *this;
  }

  class_& enum_(char const* const name, int value)
  {
    scope::enum_(name, value);
    return *this;
  }

  template <class R, class ...A>
  class_& metadef(char const* const name, R (C::*ptr_to_member)(A...))
  {
    member_function(firstmetadef_, lastmetadef_, name, ptr_to_member);
    return *this;
  }

  template <class R, class ...A>
  class_& metadef(char const* const name, R (C::*ptr_to_member)(A...) const)
  {
    has_gc = has_gc || !std::strcmp("__gc", name);

    has_index = has_index || !std::strcmp("__index", name);

    has_newindex = has_newindex || !std::strcmp("__newindex", name);

    const_member_function(firstmetadef_, lastmetadef_, name, ptr_to_member);
    return *this;
  }

  template <class R, class ...A>
  class_& property(char const* const name,
    R (C::*ptr_to_const_member)(A...) const)
  {
    static detail::member_meta_info mmi;

    *static_cast<decltype(ptr_to_const_member)*>(
      static_cast<void*>(&mmi.ptr_to_member))
      = ptr_to_const_member;

    getters_.emplace(name, detail::member_stub<&mmi, 3, C, R, A...>);
    return *this;
  }

  template <class R, class ...A>
  class_& property(char const* const name,
    R (C::*ptr_to_member)(A...))
  {
    static detail::member_meta_info mmi;

    *static_cast<decltype(ptr_to_member)*>(
      static_cast<void*>(&mmi.ptr_to_member))
      = ptr_to_member;

    getters_.emplace(name, detail::member_stub<&mmi, 3, C, R, A...>);
    return *this;
  }

  template <class RA, class ...A, class RB, class ...B>
  class_& property(char const* const name,
    RA (C::*ptr_to_const_member)(A...) const,
    RB (C::*ptr_to_member)(B...))
  {
    static detail::member_meta_info mmia;

    *static_cast<decltype(ptr_to_const_member)*>(
      static_cast<void*>(&mmia.ptr_to_member))
      = ptr_to_const_member;

    getters_.emplace(name, detail::member_stub<&mmia, 3, C, RA, A...>);

    static detail::member_meta_info mmib;

    *static_cast<decltype(ptr_to_member)*>(
      static_cast<void*>(&mmib.ptr_to_member))
      = ptr_to_member;

    setters_.emplace(name, detail::member_stub<&mmib, 3, C, RB, B...>);
    return *this;
  }

  template <class RA, class ...A, class RB, class ...B>
  class_& property(char const* const name, RA (C::*ptr_to_membera)(A...),
    RB (C::*ptr_to_memberb)(B...))
  {
    static detail::member_meta_info mmia;

    *static_cast<decltype(ptr_to_membera)*>(
      static_cast<void*>(&mmia.ptr_to_member))
      = ptr_to_membera;

    getters_.emplace(name, detail::member_stub<&mmia, 3, C, RA, A...>);

    static detail::member_meta_info mmib;

    *static_cast<decltype(ptr_to_memberb)*>(
      static_cast<void*>(&mmib.ptr_to_member))
      = ptr_to_memberb;

    setters_.emplace(name, detail::member_stub<&mmib, 3, C, RB, B...>);
    return *this;
  }

private:
  template <detail::class_meta_info const* cmi_ptr, std::size_t O, class C_, class ...A>
  friend int detail::constructor_stub(lua_State* const);

  template <detail::member_meta_info const* mmi, std::size_t O, class C_, class R, class ...A>
  typename std::enable_if<
    std::is_pointer<R>::value
    && std::is_class<
      typename std::remove_const<
        typename std::remove_pointer<R>::type
      >::type
    >::value, int>::type
  friend detail::member_stub(lua_State* const);

  static bool has_gc;
  static bool has_index;
  static bool has_newindex;

  static detail::func_info_type constructors_;

  static detail::member_info_type* firstdef_;
  detail::member_info_type* lastdef_;

  static detail::member_info_type* firstmetadef_;
  detail::member_info_type* lastmetadef_;

public:
  static std::unordered_map<char const*, lua_CFunction,
    detail::unordered_hash, detail::unordered_eq> getters_;
  static std::unordered_map<char const*, lua_CFunction,
    detail::unordered_hash, detail::unordered_eq> setters_;
};

template <class C>
bool class_<C>::has_gc;

template <class C>
bool class_<C>::has_index;

template <class C>
bool class_<C>::has_newindex;

template <class C>
detail::func_info_type class_<C>::constructors_;

template <class C>
detail::member_info_type* class_<C>::firstdef_;

template <class C>
detail::member_info_type* class_<C>::firstmetadef_;

template <class C>
std::unordered_map<char const*, lua_CFunction,
  detail::unordered_hash, detail::unordered_eq> class_<C>::getters_;

template <class C>
std::unordered_map<char const*, lua_CFunction,
  detail::unordered_hash, detail::unordered_eq> class_<C>::setters_;

} // lualite

#endif // LUALITE_HPP
