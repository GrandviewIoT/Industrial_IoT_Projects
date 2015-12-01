/*******************************************************************************
*                                     errno_MCU.h
*
* Provides a list of Unix-cvompatible errno.h codes that are normally
* not present in low-end MCUs.
*
* Designed to allow common Error codes across both Linux and MCU versions
* of the code.
* Only variance is that we set the error code to negative value, since most
* MCU vendors standard APIs use negative numbers to denote errors.
*
* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
*
* The MIT License (MIT)
*
* Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*******************************************************************************/


#ifndef _ERRNO_MCU_H_
#define _ERRNO_MCU_H_


#define EINTR          -4     /* Interrupted system call */
#define EBADF          -9     /* Bad file number */

#define EINVAL        -22     /* Invalid argument */
#define EPIPE         -32     /* Broken pipe */

#define ETIMEDOUT    -110     /* Connection timed out */

#define ENOTSUP      -199     /* Function is not supported  */


#endif /* _ERRNO_MCU_H_ */
