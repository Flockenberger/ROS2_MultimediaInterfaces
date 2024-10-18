#pragma once
///
/// This file defines and includes common functionality
/// used throughout the program.
///
/// Author: Florian Wagner
///

#if _WIN32 || _WIN64
#define MMI_PLATFORM_WINDOWS
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
#else
/* Unknown compiler/platform */

#endif
#include "WString.h"

//other pre-defines
#define MMI_NAMESPACE_BEGIN(ns) namespace ns {
#define MMI_NAMESPACE_END(ns) \
  } \
  ;

#define MMI_EXPAND_MACRO(x) x
#define MMI_STRINGIFY_MACRO(x) #x

#define MMI_BIT(x) (1 << x)


#define SIZE(...) sizeof(__VA_ARGS__)

#define SIZE_INT8 sizeof(mmi::Int8)
#define SIZE_INT16 sizeof(mmi::Int16)
#define SIZE_INT32 sizeof(mmi::Int32)
#define SIZE_INT64 sizeof(mmi::Int64)

#define SIZE_UINT8 sizeof(mmi::UInt8)
#define SIZE_UINT16 sizeof(mmi::UInt16)
#define SIZE_UINT32 sizeof(mmi::UInt32)
#define SIZE_UINT64 sizeof(mmi::UInt64)

#define SIZE_FLOAT sizeof(mmi::Float)
#define SIZE_DOUBLE sizeof(mmi::Double)

#define SIZE_INT sizeof(mmi::Int)
#define SIZE_UINT sizeof(mmi::UInt)


MMI_NAMESPACE_BEGIN(mmi)

//intergers
typedef signed char Byte; /**< Represents a signed char. <br>Its size is one byte.*/

typedef signed char Int8; /**< Represents a signed char. <br>Its size is one byte.*/
typedef Int8 s1;          /**< Represents a signed char. <br>Its size is one byte.*/

typedef short Int16; /**< Represents a short. <br>Its size is two byte.*/
typedef Int16 s2;    /**< Represents a short. <br>Its size is two byte.*/

typedef int Int32; /**< Represents an int. <br>Its size is four byte.*/
typedef Int32 s4;  /**< Represents an int. <br>Its size is four byte.*/

typedef long long Int64; /**< Represents a long. <br>Its size is eight byte.*/
typedef Int64 s8;        /**< Represents a long. <br>Its size is eight byte.*/

typedef unsigned char UChar; /**< Represents an unsigned signed char. <br>Its size is one byte.*/

typedef unsigned char UInt8; /**< Represents an unsigned signed char. <br>Its size is one byte.*/
typedef UInt8 u1;            /**< Represents an unsigned signed char. <br>Its size is one byte.*/

typedef unsigned short UInt16; /**< Represents an unsigned short. <br>Its size is two byte.*/
typedef UInt16 u2;             /**< Represents an unsigned short. <br>Its size is two byte.*/

typedef unsigned int UInt32; /**< Represents an unsigned int. <br>Its size is four byte.*/
typedef UInt32 u4;           /**< Represents an unsigned int. <br>Its size is four byte.*/

typedef unsigned long long UInt64; /**< Represents an unsigned long. <br>Its size is eight byte.*/
typedef UInt64 u8;                 /**< Represents an unsigned long. <br>Its size is eight byte.*/

typedef bool Bool; /**< Represents a boolean.*/

typedef char Char; /**< Represents a normal character.*/

//floating point numbers
typedef float Float32; /**< Represents a 32-bit floating point value.*/
typedef Float32 Float; /**< Represents a 32-bit floating point value.*/

typedef double Float64; /**< Represents a 64-bit floating point value.*/
typedef Float64 Double; /**< Represents a 64-bit floating point value.*/

MMI_NAMESPACE_END(mmi)