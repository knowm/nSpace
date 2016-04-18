////////////////////////////////////////////////////////////////////////
//
//									HANDLE.CPP
//
//				Implementation of the SQL handle object
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
      - The copyrights to all improvements and modifications to this 
        distribution shall reside in nSpace, LLC.
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in 
        the documentation and/or other materials provided with the 
        distribution.
      - Neither the name of nSpace, LLC nor the names of its contributors 
        may be used to endorse or promote products derived from this 
        software without specific prior written permission.

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

#include "sqln_.h"
#include <stdio.h>

#ifdef	USE_ODBC

// Globals
static U32 nEnv	= 0;
static U32 nConn	= 0;
static U32 nStmt	= 0;

SQLHandle :: SQLHandle ( SQLSMALLINT _HandleType, SQLHANDLE _InputHandle )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_HandleType is the type of handle to be allocated
	//		-	_InputHandle is context of the new handle
	//
	////////////////////////////////////////////////////////////////////////
	HandleType	= _HandleType;
	InputHandle	= _InputHandle;
	Handle		= NULL;
	}	// SQLHandle

HRESULT SQLHandle :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	SQLRETURN	sqlr;

	// Attempt to create the requested handle
	CCLTRYE ( (sqlr = SQLAllocHandle ( HandleType, InputHandle, &Handle ))
					== SQL_SUCCESS, sqlr );

	// AddRef ourselves, internal object
	CCLOK ( AddRef(); )

	switch (HandleType)
		{
		// Environment
		case SQL_HANDLE_ENV :
			++nEnv;
			break;

		// Statement
		case SQL_HANDLE_STMT :
			++nStmt;
			break;

		// Connection
		case SQL_HANDLE_DBC :
			++nConn;
			break;
			}	// switch

	return hr;
	}	// construct

void SQLHandle :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Destroy handle based on type
	if (Handle != NULL)
		{
		switch (HandleType)
			{
			// Environment
			case SQL_HANDLE_ENV :
				if (nEnv) --nEnv;
				break;

			// Statement
			case SQL_HANDLE_STMT :
				if (nStmt) --nStmt;
				break;

			// Connection
			case SQL_HANDLE_DBC :
				SQLDisconnect ( Handle );
				if (nConn) --nConn;
				break;
			}	// switch

		// Free handle independent of type
		SQLFreeHandle ( HandleType, Handle );
		}	// if

	// Debug
//	static WCHAR	wBfr[100];
//	swprintf ( wBfr, L"SQLHandle::destruct:nEnv %d nStmt %d nConn %d\n",
//					nEnv, nStmt, nConn );
//	OutputDebugString ( wBfr );
	}	// destruct

HRESULT SQLHandle :: getValue ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IHaveValue
	//
	//	PURPOSE
	//		-	Returns the value for the object.
	//
	//	PARAMETERS
	//		-	ppV will receive the value.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return adtValue::copy ( adtLong((U64)Handle), v );
	}	// getValue

HRESULT SQLHandle :: setValue ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IHaveValue
	//
	//	PURPOSE
	//		-	Sets the value for the object.
	//
	//	PARAMETERS
	//		-	pV contains the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	Handle = (SQLHANDLE)(U64)adtLong(v);
	return S_OK;
	}	// setValue

#endif
