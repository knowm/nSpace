////////////////////////////////////////////////////////////////////////
//
//									Handle.CPP
//
//				Implementation of the SQL handle object
//
////////////////////////////////////////////////////////////////////////

#include "sqll_.h"
#include <stdio.h>

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
	AddRef();
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

