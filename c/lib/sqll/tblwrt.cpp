////////////////////////////////////////////////////////////////////////
//
//									TBLWRT.CPP
//
//				Implementation of the SQL write table node
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
static WCHAR	wInsertTbl[]=	L"INSERT INTO \"%s\" (";
static WCHAR	wValues[]=		L" VALUES (";

SQLTableWrite :: SQLTableWrite ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pBfr		= NULL;
	pwBfr		= NULL;
	pStmBfr	= NULL;
	pvStmBfr	= NULL;
	pFlds		= NULL;
	pConn		= NULL;
	hConn		= SQL_NULL_HANDLE;
	}	// SQLTableWrite

HRESULT SQLTableWrite::bindVariant ( SQLHANDLE hStmt, U32 uNum,
													SQLCol *pValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Bind a variant data type to a parameter number.
	//
	//	PARAMETERS
	//		-	hStmt is the current statement handle
	//		-	uNum is the parameter number to sue
	//		-	vValue is the value to bind
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr						= S_OK;
	SQLUSMALLINT	ParameterNumber	= (SQLUSMALLINT) uNum;
	SQLSMALLINT		ValueType;
	SQLSMALLINT		ParameterType;
	SQLUINTEGER		ColumnSize;
	SQLSMALLINT		DecimalDigits;
	SQLPOINTER		ParameterValuePtr;
	SQLINTEGER		BufferLength;

	// Based on the data type we have to generate :
	// 1 - The 'C' data type
	// 2 - The 'SQL' data type
	// 3 - The pointer to the data
	// For objects a NULL pointer will be used and the object will be
	//	streamed to the location during 'SQLPutData' later on.

	// Data type information
	if (hr == S_OK)
		{
		BufferLength	= 0;								// Default
		ColumnSize		= 0;								// Default
		DecimalDigits	= 0;								// Default
		switch (pValue->sData.vtype)
			{
			case VALT_I4 :
				ValueType			= SQL_C_ULONG;
				ParameterType		= SQL_INTEGER;
				ParameterValuePtr	= &(pValue->sData.vint);
				break;

			case VALT_DATE :
				{
				// Since we can only STORE dates as doubles and not LOAD dates
				// as doubles convert to the SQL structure for compatibility.  This
				// is also the only way to store fractions of a second.
				SYSTEMTIME	st;

				// Convert our variant date to system time
				CCLTRY( adtDate::toSystemTime ( pValue->sData.vdate, &st ) );

				// Copy fields
				if (hr == S_OK)
					{
					pValue->TimeStamp.year		= st.wYear;
					pValue->TimeStamp.month		= st.wMonth;
					pValue->TimeStamp.day		= st.wDay;
					pValue->TimeStamp.hour		= st.wHour;
					pValue->TimeStamp.minute	= st.wMinute;
					pValue->TimeStamp.second	= st.wSecond;
					// 'fraction' is number of nanoseconds
					pValue->TimeStamp.fraction	= ((U32)(st.wMilliseconds)*1000000);
					}	// if

				// SQL setup
				if (hr == S_OK)
					{
					ValueType			= SQL_C_TYPE_TIMESTAMP;
					ParameterType		= SQL_TYPE_TIMESTAMP;
					ParameterValuePtr	= &(pValue->TimeStamp);
					DecimalDigits		= 3;				// Store fractions of a second
					ColumnSize			= 23;
					BufferLength		= 23;
					}	// if

				}	// VALT_DATE
				break;

			case VALT_STR :
				ValueType			= SQL_C_WCHAR;
				ParameterType		= SQL_WCHAR;
				ParameterValuePtr	= (SQLPOINTER)(LPCWSTR)adtString(pValue->sData);
				ColumnSize			= adtString(pValue->sData).length();
				BufferLength		= ColumnSize*sizeof(WCHAR);
				pValue->uSz			= SQL_NTS;
				break;

			case VALT_R4 :
				ValueType			= SQL_C_FLOAT;
				ParameterType		= SQL_FLOAT;
				ParameterValuePtr	= &(pValue->sData.vflt);
				break;

			case VALT_R8 :
				ValueType			= SQL_C_DOUBLE;
				ParameterType		= SQL_DOUBLE;
				ParameterValuePtr	= &(pValue->sData.vdbl);
				break;

			// Objects are 'pre-streamed'.
			// Obtain size of region.
			case VALT_UNKNOWN :
				{
				// Default parameters
				ValueType			= SQL_C_BINARY;
				ParameterType		= SQL_LONGVARBINARY;
				ParameterValuePtr	= NULL;

				// Get size of object based on type
				IByteStream		*pStm	= NULL;
				IMemoryMapped	*pMem	= NULL;

				// Memory mapped ?
				if (	pValue->sData.punk != NULL &&
						_QI(pValue->sData.punk,IID_IMemoryMapped,&pMem) == S_OK )
					{
					PVOID	pvMem	= NULL;
					U32	sz;

					// Determine size by locking full region
					CCLTRY ( pMem->lock ( 0, 0, &pvMem, &sz ) );
					CCLOK  ( ColumnSize = sz; )

					// Clean up
					_UNLOCK(pMem,pvMem);
					_RELEASE(pMem);
					}	// if

				// Byte stream ?
				else if (	pValue->sData.punk != NULL &&
								_QI(pValue->sData.punk,IID_IByteStream,&pStm) == S_OK)
					{
					U32	then,now;

					// Determine size by seeking to end of stream
					// from current position
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_CUR, &then ) );
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_END, &now ) );
					CCLTRY ( pStm->seek ( then, STREAM_SEEK_SET, NULL ) );
					CCLOK	 ( ColumnSize = now; )

					// Clean up
					_RELEASE(pStm);
					}	// else if

				// Match buffer length
				CCLOK ( BufferLength = ColumnSize; )
				CCLOK ( pValue->uSz = SQL_LEN_DATA_AT_EXEC(((SQLINTEGER)ColumnSize)); )
				CCLOK ( ParameterValuePtr = pValue->sData.punk; )
				}	// VALT_UNKNOWN
				break;

			// Not handled...
			default :
				hr = E_UNEXPECTED;
			}	// switch
		}	// if

	// Perform the bind
	CCLTRY ( SQLSTMT(hStmt,(
		SQLBindParameter (	hStmt,
									ParameterNumber,
									SQL_PARAM_INPUT,
									ValueType,
									ParameterType,
									ColumnSize,
									DecimalDigits,
									ParameterValuePtr,
									BufferLength,
									&(pValue->uSz) ) )));

	return hr;
	}	// bindVariant

HRESULT SQLTableWrite :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// Create buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ));
	CCLTRY ( FactCache.AddRef(); );

	// Stream buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pStmBfr ));
	CCLTRY ( pStmBfr->setSize ( SIZE_STM_BUFFER ) );
	CCLTRY ( pStmBfr->lock ( 0, 0, &pvStmBfr, NULL ) );

	return hr;
	}	// construct

void SQLTableWrite :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pFlds);
	_UNLOCK(pStmBfr,pvStmBfr);
	_RELEASE(pStmBfr);
	_RELEASE(pBfr);
	_RELEASE(pConn);
	FactCache.Release();
	}	// destruct

HRESULT SQLTableWrite :: putData ( SQLHANDLE hStmt, IUnknown *unkP )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Performs SQLPutData on the specified stream.
	//
	//	PARAMETERS
	//		-	hStmt is the current statement handle
	//		-	unkP is either a byte stream or a memory block.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	IByteStream		*pStm	= NULL;
	IMemoryMapped	*pMem	= NULL;
	U32				szBfr;

	// We bind the IUnknown ptr. for binary data... Write data to SQL engine
	CCLTRYE	( (IUnknown *)(NULL) != unkP, E_UNEXPECTED );

	// Memory mapped ?
	if (hr == S_OK && _QI(unkP,IID_IMemoryMapped,&pMem) == S_OK)
		{
		PVOID	pvMem	= NULL;

		// Access buffer
		CCLTRY ( pMem->lock ( 0, 0, &pvMem, &szBfr ) );
		CCLTRY ( SQLSTMT(hStmt,SQLPutData ( hStmt, pvMem, szBfr )) );

		// Clean up
		_UNLOCK(pMem,pvMem);
		_RELEASE(pMem);
		}	// if

	// Byte Stream ?
	else if (hr == S_OK && _QI(unkP,IID_IByteStream,&pStm) == S_OK)
		{
		// Copy data into SQL engine
		while (hr == S_OK)
			{
			// Read from stream
			CCLTRY ( pStm->read ( pvStmBfr, SIZE_STM_BUFFER, &szBfr ) );

			// Out of data ?
			if (hr == S_FALSE && szBfr == 0)
				{
				hr = S_OK;
				break;
				}	// if

			// SQL
			CCLTRY ( SQLSTMT(hStmt,SQLPutData ( hStmt, pvStmBfr, szBfr )) );
			}	// while

		// Clean up
		_RELEASE(pStm);
		}	// else if

	// Unsupported
	else hr = E_UNEXPECTED;

	return hr;
	}	// putData

HRESULT SQLTableWrite :: receive ( IReceptor *pR, const adtValue &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pR is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Fire
	if (prFire == pR)
		{
		IADTInIt			*pKeys		= NULL;
		SQLHANDLE		hStmt			= NULL;
		WCHAR				*pwBfr		= NULL;
		SQLCol			*pvValues	= NULL;
		SQLPOINTER		ValuePtrPtr;
		adtString		sKey;
		U32				szBfr,nkeys,kidx;
		SQLRETURN		sqlr;

		// State check
		CCLTRYE	( (hConn != SQL_NULL_HANDLE), ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnAttr->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
		CCLTRYE	( (pFlds != NULL),				ERROR_INVALID_STATE );

		// Our statement handle
		CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );

		// Field information
		CCLTRY ( pFlds->keys ( &pKeys ) );
		CCLTRY ( pFlds->size ( &nkeys ) );

		//
		// Allocate an array of 'values' for direct binding during the insertion.
		// The only parameters that are delayed are objects (IUnknown).  This will
		// be streamed to the DB when the need arises.
		//

		// Allocate memory for our copied value table
		CCLTRYE ( (pvValues = new SQLCol[nkeys]) != NULL, E_OUTOFMEMORY );

		// FIRST PASS : Gets lengths of all the key names.  We need this to allocate
		// enough space in our 'INSERT INTO' buffer.  Add closing parens
		CCLOK	 ( szBfr =	(U32)wcslen(wInsertTbl)+(U32)wcslen(wValues)+sTableName.length()+2; )
		CCLOK  ( kidx = 0; )
		CCLTRY ( pKeys->begin() );
		while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
			{
			// Add to total length plus a two quotes and comma
			CCLOK ( szBfr += (U32)(wcslen(sKey)+3); )

			// Additional space for the end of the query string (?,)
			CCLOK ( szBfr += 2; )

			// Count keys
			CCLOK ( ++kidx; )

			// Clean up
			pKeys->next();
			}	// while

		// Allocate and initialize SQL command string
		CCLTRY ( pBfr->setSize ( szBfr*sizeof(WCHAR) ) );
		CCLTRY ( pBfr->lock ( 0, 0, (PVOID *) &pwBfr, NULL ) );
		CCLOK  ( swprintf ( pwBfr, wInsertTbl, (LPCWSTR) sTableName ); )
		// static WCHAR	wInsertTbl[]=	L"INSERT INTO %s (";

		// SECOND PASS : Bind the parameters and continue generating the
		// SQL query string
		CCLTRY ( pKeys->begin() );
		CCLOK  ( kidx = 0; )
		while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
			{
			// Value for key
			CCLTRY ( pFlds->load ( sKey, pvValues[kidx].sData ) );

			// Next key name
			CCLOK ( wcscat ( pwBfr, L"\"" ); )
			CCLOK ( wcscat ( pwBfr, sKey ); )
			CCLOK ( wcscat ( pwBfr, L"\"" ); )

			// Need comma ?
			if (hr == S_OK && kidx+1 < nkeys)
				wcscat ( pwBfr, L"," );

			// If data is an object and it is a 'persistable' object, stream it to a memory block
			// and use that as the storable item.  That way we only have
			// 'MemoryMapped' and 'ByteStream' objects to deal with.
			// It would have been nice to stream the object directly to the database
			// but SQL wants the size of the blob during parameter binding.
			if ( hr == S_OK && pvValues[kidx].sData.vtype == VALT_UNKNOWN )
				{
				IUnknown	*pObj = pvValues[kidx].sData.punk;
				hr = streamObj ( pObj, &(pvValues[kidx].sData.punk) );
				_RELEASE(pObj);
				}	// if

			// Bind parameter
			CCLTRY ( bindVariant ( hStmt, kidx+1, &(pvValues[kidx]) ) );

			// Clean up
			CCLOK ( kidx++; )
			pKeys->next();
			}	// while

		// Terminate key list
		CCLOK ( wcscat ( pwBfr, L") VALUES (" ); )

		// Add '?' for all values.  They will be bound/copied at execution time.
		for (kidx = 0;hr == S_OK && kidx < nkeys;++kidx)
			{
			CCLOK ( wcscat ( pwBfr, L"?" ); )
			if (hr == S_OK && kidx+1 < nkeys)
				wcscat ( pwBfr, L"," );
			}	// for

		// Terminate value list
		CCLOK ( wcscat ( pwBfr, L")" ); )

		// Execute query.  If successful the query will need our bound parameters
	//	CCLOK ( OutputDebugString ( pwBfr ); )
	//	CCLOK ( OutputDebugString ( L"\n" ); )

		// Perform any 'late' binding as needed
		if (hr == S_OK)
			{
			// Execute statement
	//		WCHAR wBfr[100];
	//		CCLOK (swprintf ( wBfr, L"(%d/%d):", (U32)adtString::length(pwBfr), szBfr*sizeof(WCHAR) ); )
	//		CCLOK	(OutputDebugString ( wBfr ); )
	//		CCLOK	( OutputDebugString ( pwBfr ); )
	//		CCLOK	( OutputDebugString ( L"\n" ); )
			sqlr = SQLExecDirect ( hStmt, pwBfr, (SQLINTEGER)wcslen(pwBfr) );

			// More data ?
			if (sqlr == SQL_NEED_DATA)
				{
				// Next parameter
				while ( hr == S_OK && (sqlr = SQLParamData ( hStmt, &ValuePtrPtr )) == SQL_NEED_DATA )
					{
					// Write data
					CCLTRY ( putData ( hStmt, (IUnknown *) ValuePtrPtr ) );
					}	// while

				// Error ?
				if (sqlr == SQL_ERROR) hr = SQLSTMT(hStmt,sqlr);
				}	// if

			// Other result
			else if (sqlr == SQL_SUCCESS)
				hr = S_OK;

			// Error
			else
				hr = SQLSTMT(hStmt,sqlr);
			}	// if

		// Clean up
		if (pvValues != NULL) delete[] pvValues;
		_UNLOCK(pBfr,pwBfr);
		SQLFREESTMT(hStmt);
		_RELEASE(pKeys);

		// Result
		peFire->emit ( v );
		}	// if

	// Connection
	else if (prConn == pR)
		{
		HRESULT		hr = S_OK;
		adtIUnknown unkV(v);
		adtLong		lTmp;
		_RELEASE(pConn);
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pConn));
		CCLTRY(pConn->getValue ( lTmp ));
		CCLOK (hConn = (SQLHANDLE)(U64)lTmp;)
		}	// else if

	// Fields
	else if (prFlds == pR)
		{
		adtIUnknown unkV(v);
		_RELEASE(pFlds);
		hr = _QISAFE(unkV,IID_IADTDictionary,&pFlds);
		}	// else if

	// State
	else if (prTbl == pR)
		hr = adtValueImpl::copy ( sTableName, adtString(v) );

	return hr;
	}	// receive

HRESULT SQLTableWrite :: streamObj ( IUnknown *pObj, IUnknown **ppUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Streams a persistable object to a memory block.
	//
	//	PARAMETERS
	//		-	pObj is the object to save
	//		-	ppUnk will receive the memory block
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr				= S_OK;
	IParseStm			*pParse		= NULL;
	IByteStream			*pStm			= NULL;

	// Setup
	CCLOK		( (*ppUnk) = NULL; )
	CCLTRYE	( (pObj != NULL), E_INVALIDARG );

	// Create memory stream and binary parser
	CCLTRY	( COCREATEINSTANCEC ( FactCache, CLSID_MemoryStream,
												IID_IByteStream, &pStm ) );
	CCLTRY	( COCREATEINSTANCEC ( FactCache, CLSID_SysParserBin,
												IID_IParseStm, &pParse ) );

	// Save object
	CCLTRY	( pParse->valueSave ( pStm, adtIUnknown(pObj) ) );
	CCLTRY	( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );

	// Result
	CCLOK		( (*ppUnk) = pStm; )
	CCLOK		( (*ppUnk)->AddRef(); )

	// Clean up
	_RELEASE ( pParse );
	_RELEASE ( pStm );

	return hr;
	}	// streamObj

#endif

#ifdef	USE_OLEDB

#define	SIZE_SQLBFR		1024

// Globals
static WCHAR	wInsertTbl[]=	L"INSERT INTO \"%s\" (";
static WCHAR	wValues[]=		L" VALUES (";

SQLTableWrite :: SQLTableWrite ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pQryBfr	= NULL;
	pwQryBfr	= NULL;
	pBfr		= NULL;
	uBfrSz	= 0;
	pFlds		= NULL;
	pConn		= NULL;

	// Interfaces
	addInterface ( IID_INodeBehaviour, (INodeBehaviour *) this );
	}	// SQLTableWrite

HRESULT SQLTableWrite :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// Create buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pQryBfr ) );
	CCLTRY ( pQryBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pQryBfr->lock ( 0, 0, (PVOID *) &pwQryBfr, NULL ) );

	// Data buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ) );
	CCLTRY ( pBfr->setSize ( 10 ) );

	return hr;
	}	// construct

void SQLTableWrite :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pFlds);
	_RELEASE(pBfr);
	_UNLOCK(pQryBfr,pwQryBfr);
	_RELEASE(pQryBfr);
	_RELEASE(pConn);
	}	// destruct

HRESULT SQLTableWrite :: receiveFire ( const adtValue &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Callback function for a receptor
	//
	//	PARAMETERS
	//		-	v is the value that was emitted
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT					hr				= S_OK;
/*
	IDBCreateSession		*pCreate		= NULL;
	IDBCreateCommand		*pCmdCreate	= NULL;
	ICommandText			*pCmdTxt		= NULL;
	ICommandPrepare		*pCmdPrep	= NULL;
	ICommandProperties	*pCmdProp	= NULL;
	IAccessor				*pAccessor	= NULL;
	IADTDictionary			*pDict		= NULL;
	IColumnsInfo			*pColInfo	= NULL;
	IRowset					*pRowset		= NULL;
	U8							*pcBfr		= NULL;
	HACCESSOR				hAccessor	= NULL;
	IADTInIt					*pKeys		= NULL;
	GUID						guidDialect	= DBGUID_DEFAULT;
	DBPROPSET				set[1];
	DBPROP					prop[1];
	DBPARAMS					dbparams;
	DBROWCOUNT				dbrc;
	adtValueImpl			vVal;
	U32						uBfrPos;
	adtString				sTable,sKey;
	U32						szBfr,nkeys,kidx;

	// State check
	CCLTRYE	( (pConn != NULL),	ERROR_INVALID_STATE );
	CCLTRYE	( (pFlds != NULL),	ERROR_INVALID_STATE );

	// Node
	CCLTRY	( pnAttr->load ( strRefTableName, sTable ) );

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_IDBCreateCommand,
													(IUnknown **) &pCmdCreate ) );
	CCLTRY	( pCmdCreate->CreateCommand ( NULL, IID_ICommandText, 
														(IUnknown **) &pCmdTxt ));
	CCLTRY	( _QI(pCmdTxt,IID_IAccessor,&pAccessor) );
	CCLTRY	( _QI(pCmdTxt,IID_ICommandPrepare,&pCmdPrep) );
	CCLTRY	( _QI(pCmdTxt,IID_IColumnsInfo,&pColInfo) );

	// Field information
	CCLTRY ( pFlds->keys ( &pKeys ) );
	CCLTRY ( pFlds->size ( &nkeys ) );

	// Initialize SQL command string
	CCLOK  ( swprintf ( pwQryBfr, wInsertTbl, (WCHAR *) sTable ); )
	// static WCHAR	wInsertTbl[]=	L"INSERT INTO %s (";

	// Append keys name to query string
	CCLTRY ( OLEDBAppendValues ( pFlds, pwQry ) );

	// SECOND PASS : Bind the parameters and continue generating the
	// SQL query string
	CCLTRY ( pKeys->begin() );
	CCLOK  ( kidx = 0; )
	while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
		{
		// Value for key
		CCLTRY ( pFlds->load ( sKey, pvValues[kidx].sData ) );

		// Next key name
		CCLOK ( wcscat ( pwQryBfr, L"\"" ); )
		CCLOK ( wcscat ( pwQryBfr, sKey ); )
		CCLOK ( wcscat ( pwQryBfr, L"\"" ); )

		// Need comma ?
		if (hr == S_OK && kidx+1 < nkeys)
			wcscat ( pwQryBfr, L"," );

		// If data is an object and it is a 'persistable' object, stream it to a memory block
		// and use that as the storable item.  That way we only have
		// 'MemoryMapped' and 'ByteStream' objects to deal with.
		// It would have been nice to stream the object directly to the database
		// but SQL wants the size of the blob during parameter binding.
		if ( hr == S_OK && pvValues[kidx].sData.vtype == VALT_UNKNOWN )
			{
			IUnknown	*pObj = pvValues[kidx].sData.punk;
			hr = streamObj ( pObj, &(pvValues[kidx].sData.punk) );
			_RELEASE(pObj);
			}	// if

		// Bind parameter
		CCLTRY ( bindVariant ( hStmt, kidx+1, &(pvValues[kidx]) ) );

		// Clean up
		CCLOK ( kidx++; )
		pKeys->next();
		}	// while

	// Terminate key list
	CCLOK ( wcscat ( pwQryBfr, L") VALUES (" ); )

	// Add '?' for all values.  They will be bound/copied at execution time.
	for (kidx = 0;hr == S_OK && kidx < nkeys;++kidx)
		{
		CCLOK ( wcscat ( pwQryBfr, L"?" ); )
		if (hr == S_OK && kidx+1 < nkeys)
			wcscat ( pwQryBfr, L"," );
		}	// for

	// Terminate value list
	CCLOK ( wcscat ( pwQryBfr, L")" ); )


	// 'Prepare' the command text.  This is required so that the 'ordinal' columns #'s
	// can be obtain so the data can be bound
	CCLOK ( OutputDebugString ( pwQryBfr ); )
	CCLOK ( OutputDebugString ( L"\n" ); )
	CCLTRY ( pCmdTxt->SetCommandText ( guidDialect, pwQryBfr ) );
	CCLTRY ( pCmdPrep->Prepare(0) );

	// Copy values into buffer
	CCLOK	( uBfrPos = 0; )
	CCLTRY( OLEDBApplyValues ( pKeys, pBfr, pbCons, &uBfrPos ) );

		// Create an accessor for the constraints
		CCLTRY ( pAccessor->CreateAccessor ( DBACCESSOR_PARAMETERDATA, szCons, pbCons,
															uBfrPos, &hAccessor, NULL ) );
		}	// if

	// Fill database parameters
	CCLTRY	( pBfr->lock ( 0, 0, (void **) &pcBfr, NULL ) );
	if (hr == S_OK)
		{
		dbparams.cParamSets	= (szCons) ? 1 : 0;
		dbparams.pData			= pcBfr;
		dbparams.hAccessor	= hAccessor;
		}	// if

	// Execute query with bound parameters
	CCLTRY ( pCmdTxt->Execute ( NULL, IID_IRowset, &dbparams, &dbrc,
											(IUnknown **) &pRowset ) );

	// Perform any 'late' binding as needed
	if (hr == S_OK)
		{
		// Execute statement
//		WCHAR wBfr[100];
//		CCLOK (swprintf ( wBfr, L"(%d/%d):", (U32)adtString::length(pwQryBfr), szBfr*sizeof(WCHAR) ); )
//		CCLOK	(OutputDebugString ( wBfr ); )
//		CCLOK	( OutputDebugString ( pwQryBfr ); )
//		CCLOK	( OutputDebugString ( L"\n" ); )
		sqlr = SQLExecDirect ( hStmt, pwQryBfr, adtString::length(pwQryBfr) );

		// More data ?
		if (sqlr == SQL_NEED_DATA)
			{
			// Next parameter
			while ( hr == S_OK && (sqlr = SQLParamData ( hStmt, &ValuePtrPtr )) == SQL_NEED_DATA )
				{
				// Write data
				CCLTRY ( putData ( hStmt, (IUnknown *) ValuePtrPtr ) );
				}	// while

			// Error ?
			if (sqlr == SQL_ERROR) hr = SQLSTMT(hStmt,sqlr);
			}	// if

		// Other result
		else if (sqlr == SQL_SUCCESS)
			hr = S_OK;

		// Error
		else
			hr = SQLSTMT(hStmt,sqlr);
		}	// if

	// Clean up
	if (pvValues != NULL) delete[] pvValues;
	_UNLOCK(pQryBfr,pwQryBfr);
	SQLFREESTMT(hStmt);
	_RELEASE(pKeys);

	// Result
	peFire->emit ( v );
*/
	return hr;
	}	// receiveFire

//
// Context
//

HRESULT SQLTableWrite :: receiveConnection ( const adtValue &v )
	{
	adtIUnknown	unkV(v);
	_RELEASE(pConn);
	_QISAFE(unkV,IID_IUnknown,&pConn);
	return S_OK;
	}	// receiveConnection

HRESULT SQLTableWrite :: receiveFields ( const adtValue &v )
	{
	adtIUnknown unkV(v);
	_RELEASE(pFlds);
	_QISAFE(unkV,IID_IADTDictionary,&pFlds);
	return S_OK;
	}	// receiveFields

#endif

