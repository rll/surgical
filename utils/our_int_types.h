#ifndef OUR_INT_TYPES_H
#define OUT_INT_TYPES_H
/**
 * @file OurIntTypes.h
 *
 * @brief Contains definitions of basic primitive types for use
 * throughout the code.
 */
// integral types
#if HAVE_INTTYPES_H
#include <inttypes.h>
#elif HAVE_STDINT_H
#include <stdint.h>
#elif( defined( _MSC_VER ) )
//	typedef signed char			int8_t;		// C99 stdint.h not supported in VC++/VS.NET yet.
//	typedef unsigned char		uint8_t;	// C99 stdint.h not supported in VC++/VS.NET yet.
//	typedef signed short		int16_t;	// C99 stdint.h not supported in VC++/VS.NET yet.
//	typedef unsigned short		uint16_t;	// C99 stdint.h not supported in VC++/VS.NET yet.
//	typedef signed long			int32_t;	// C99 stdint.h not supported in VC++/VS.NET yet.
//	typedef unsigned long		uint32_t;	// C99 stdint.h not supported in VC++/VS.NET yet.
//  typedef long long           int64_t;
//  typedef unsigned long long  uint64_t;


typedef signed __int8		int8_t;
typedef unsigned __int8		uint8_t;
typedef signed __int16		int16_t;
typedef unsigned __int16	uint16_t;
typedef signed __int32		int32_t;
typedef unsigned __int32	uint32_t;
typedef signed __int64		int64_t;
typedef unsigned __int64	uint64_t;

#else
#include <inttypes.h> // Client builds should assume this in the absence of macros
#endif


#endif /* OUR_INT_TYPES_H */

