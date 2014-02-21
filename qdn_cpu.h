/**************************************************************************
 *
 * Copyright (c) 2013, Qromodyn Corporation
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 **************************************************************************/

#ifndef _QDN_CPU_H_
#define _QDN_CPU_H_

#include <stdint.h>
// #include "qdn_project.h"


#ifdef __C51__
    #define __istate_t                 bit
    #define __get_interrupt_state()    EA
    #define __disable_interrupt()      do { EA = 0;} while (0)
    #define __set_interrupt_state(s)   do { EA = (s); } while (0)
    
    #define SystemFastTicks() 0
    
    #define _QDN_BIG_ENDIAN 1
    #define XDATA xdata
    #define CODE  code
    #include <intrins.h>						// for _testbit_(), _nop_()
    #define CALL_CONV 

    typedef uint16_t MemoryAddress_t;

#endif

#if defined(__ICCARM__) || defined(__arm__)

    #define _QDN_LITTLE_ENDIAN 1
    #define XDATA 
    #define CODE
	#ifdef __ICCARM__
    	#include <intrinsics.h>
	#endif
    
    #define SystemFastTicks() 0 // to do
    #define CALL_CONV 
    typedef uint32_t MemoryAddress_t;

	#ifdef __GNUC__
		#define __no_init    __attribute__ ((section (".noinit")))
	#endif
#endif

#ifdef _MSC_VER
    #define _QDN_LITTLE_ENDIAN 1
    #define XDATA 
    #define CODE
    
    #define __istate_t                 int
    #define __get_interrupt_state()    0
    #define __disable_interrupt()      do { } while (0)
    #define __set_interrupt_state(s)   do {  } while (0)
    #define __enable_interrupt()       do {  } while (0)
    
    #define CALL_CONV __clrcall
    typedef uint32_t MemoryAddress_t;
#endif

#ifdef __TI_COMPILER_VERSION__
# ifdef __MSP430__
#  define _QDN_LITTLE_ENDIAN 1
#  ifndef XDATA
 #  define XDATA
#  endif
#  ifndef CODE
 #  define CODE
#  endif

#  define __istate_t                uint16_t
#  define CALL_CONV ??fixme??

#  ifdef __MSP430F5438A__
#   define QDN_ADDRESS_IS_32BIT
    typedef uint32_t MemoryAddress_t;
#  elif defined(__CC430F5137__)
#   define QDN_ADDRESS_IS_16BIT
    typedef uint16_t MemoryAddress_t;
#  else
#   error processor not supported yet
#  endif
# endif
#endif

#ifdef __cplusplus
#define QDN_EXTERN_C     extern "C" {
#define QDN_END_EXTERN_C }
#else
#define QDN_EXTERN_C
#define QDN_END_EXTERN_C
#endif


#endif // _QDN_CPU_H_
