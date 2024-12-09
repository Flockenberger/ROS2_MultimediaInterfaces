#pragma once
///
/// This file defines and includes common functionality
/// used throughout the program.
/// 
/// Author: Florian Wagner
/// 

#define MMI_NAMESPACE_BEGIN(ns) namespace ns {
#define MMI_NAMESPACE_END(ns) };

#if _WIN32 || _WIN64

#define MMI_PLATFORM_WINDOWS
#include <string>
#include <memory>
#include <ostream>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <vector>
#include <sstream>
#include <algorithm>
MMI_NAMESPACE_BEGIN(mmi)
using String = std::string;
MMI_NAMESPACE_END(mmi)
#if _WIN64
#define MMI_ENV_64
#else
#define MMI_ENV_32
#endif
#elif defined(__APPLE__) || defined(__MACH__)
#include <TargetConditionals.h>
/* TARGET_OS_MAC exists on all the platforms
 * so we must check all of them (in this order)
 * to ensure that we're running on MAC
 * and not some other Apple platform */
#if TARGET_IPHONE_SIMULATOR == 1
#elif TARGET_OS_IPHONE == 1
#define MMI_PLATFORM_IOS
#elif TARGET_OS_MAC == 1
#define MMI_PLATFORM_MACOS
#else
#error "Unknown Apple platform!"
#endif
#elif defined(__ANDROID__)
#define MMI_PLATFORM_ANDROID
#elif defined(__linux__)
#define MMI_PLATFORM_LINUX
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__) || defined(__AVR__) || defined(ARDUINO)
#define MMI_PLATFORM_ARDUINO
#include "WString.h" //Arduino wiring string
MMI_NAMESPACE_BEGIN(mmi)
using String = ::String;
MMI_NAMESPACE_END(mmi)
#else
/* Unknown compiler/platform */
#error "Unknown platform!"
#endif

//other pre-defines
#define MMI_EXPAND_MACRO(x) x
#define MMI_STRINGIFY_MACRO(x) #x

#define MMI_BIT(x) (1 << x)

#define MMI_STATIC static
#define MMI_CONST const

#define SIZE(...) sizeof(__VA_ARGS__)

#define SIZE_INT8			sizeof(mmi::Int8)
#define SIZE_INT16			sizeof(mmi::Int16)
#define SIZE_INT32			sizeof(mmi::Int32)
#define SIZE_INT64			sizeof(mmi::Int64)

#define SIZE_UINT8			sizeof(mmi::UInt8)
#define SIZE_UINT16			sizeof(mmi::UInt16)
#define SIZE_UINT32			sizeof(mmi::UInt32)
#define SIZE_UINT64			sizeof(mmi::UInt64)

#define SIZE_FLOAT			sizeof(mmi::Float)
#define SIZE_DOUBLE			sizeof(mmi::Double)

#define SIZE_INT			sizeof(mmi::Int)
#define SIZE_UINT			sizeof(mmi::UInt)

MMI_NAMESPACE_BEGIN(mmi)

//intergers
typedef signed char			Byte;	/**< Represents a signed char. <br>Its size is one byte.*/

typedef signed char			Int8;	/**< Represents a signed char. <br>Its size is one byte.*/
typedef Int8				s1;		/**< Represents a signed char. <br>Its size is one byte.*/

typedef short				Int16;	/**< Represents a short. <br>Its size is two byte.*/
typedef Int16				s2;		/**< Represents a short. <br>Its size is two byte.*/

typedef int					Int32;	/**< Represents an int. <br>Its size is four byte.*/
typedef Int32				s4;		/**< Represents an int. <br>Its size is four byte.*/

typedef long long			Int64;	/**< Represents a long. <br>Its size is eight byte.*/
typedef Int64				s8;		/**< Represents a long. <br>Its size is eight byte.*/

typedef unsigned char	   UChar;	/**< Represents an unsigned signed char. <br>Its size is one byte.*/

typedef unsigned char      UInt8;	/**< Represents an unsigned signed char. <br>Its size is one byte.*/
typedef UInt8			   u1;		/**< Represents an unsigned signed char. <br>Its size is one byte.*/

typedef unsigned short     UInt16;	/**< Represents an unsigned short. <br>Its size is two byte.*/
typedef UInt16			   u2;		/**< Represents an unsigned short. <br>Its size is two byte.*/

typedef unsigned int       UInt32;	/**< Represents an unsigned int. <br>Its size is four byte.*/
typedef UInt32			   u4;		/**< Represents an unsigned int. <br>Its size is four byte.*/

typedef unsigned long long UInt64;	/**< Represents an unsigned long. <br>Its size is eight byte.*/
typedef UInt64			   u8;		/**< Represents an unsigned long. <br>Its size is eight byte.*/

typedef bool			   Bool;	/**< Represents a boolean.*/

typedef char			   Char;	/**< Represents a normal character.*/

//floating point numbers
typedef float			   Float32;	/**< Represents a 32-bit floating point value.*/
typedef Float32			   Float;	/**< Represents a 32-bit floating point value.*/

typedef double			   Float64;	/**< Represents a 64-bit floating point value.*/
typedef Float64			   Double;	/**< Represents a 64-bit floating point value.*/
template<typename T>
using Scope = std::unique_ptr<T>;	/**< A scope is an alias for a unique_ptr of type <T>*/

template<typename T>
using Ref = std::shared_ptr<T>;		/**< A Ref is an alias for a shared_ptr of type <T>*/


/// <summary>
/// Creates a new scope. A scope is a unique_ptr<br>
/// </summary>
/// <typeparam name="T"> the type that this scope holds</typeparam>
/// <typeparam name="...Args">The arguments to call the constructor of the given type T with</typeparam>
/// <param name="...args">The arguments to call the constructor of the given type T with</param>
/// <returns>a new Scope of type T</returns>
template<typename T, typename ... Args>
constexpr Scope<T> CreateScope(Args&& ... args)
{
	return std::make_unique<T>(std::forward<Args>(args)...);
}

/// <summary>
/// Creates a new Ref. A ref is a shared_ptr<br>
/// </summary>
/// <typeparam name="T"> the type that this reference holds</typeparam>
/// <typeparam name="...Args">The arguments to call the constructor of the given type T with</typeparam>
/// <param name="...args">The arguments to call the constructor of the given type T with</param>
/// <returns>a new Ref of type T</returns>
template<typename T, typename ... Args>
constexpr Ref<T> CreateRef(Args&& ... args)
{
	return std::make_shared<T>(std::forward<Args>(args)...);
}

/// <summary>
/// Wraps a Pointer of type T into a Ref
/// </summary>
/// <typeparam name="T">The type of the pointer and class</typeparam>
/// <param name="ptr">The pointer to wrap</param>
/// <returns>A new Ref</returns>
template<typename T>
constexpr Ref<T> ToRef(T* ptr)
{
	return std::shared_ptr<T>(ptr);
}



MMI_NAMESPACE_END(mmi)