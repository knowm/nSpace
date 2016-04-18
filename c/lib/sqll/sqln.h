////////////////////////////////////////////////////////////////////////
//
//									SQLN.H
//
//			Include file for the ActiveX Data Object node library
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) 2003, nSpace, LLC
   All right reserved

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions
   are met:

      - Redistributions of source code must retain the above copyright 
        notice, this list of conditions and the following disclaimer.
      - nSpace, LLC as the copyright holder reserves the right to 
        maintain, update, and provide future releases of the source code.
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in 
        the documentation and/or other materials provided with the 
        distribution.
      - Neither the name of nSpace, nor the names of its contributors may 
        be used to endorse or promote products derived from this software
        without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT 
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSBILITY OF SUCH DAMAGE.
*/

#ifndef	SQLN_H
#define	SQLN_H

// Includes
#include "../../lib/nspcl/nspcl.h"

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_SQLConnection, 0x2534d0c0, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLCreateDatabase, 0x2534d0c1, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLDelete, 0x2534d1c2, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLQuery, 0x2534d0c3, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLQueryKey, 0x2534d0c4, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLQueryRange, 0x2534d0c5, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLRecordEnum, 0x2534d0c6, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLTableCreate, 0x2534d0c7, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLTableWrite, 0x2534d0c8, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQLUpdate, 0x2534d1c9, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

// Vesion 2 nodes

DEFINE_GUID	(	CLSID_SQL2Table, 0x2534d0ca, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

DEFINE_GUID	(	CLSID_SQL2Index, 0x2534d0cb, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xeb );

#endif
