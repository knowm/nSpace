////////////////////////////////////////////////////////////////////////
//
//									RECENUM.CPP
//
//				Implementation of the SQL record enumerator
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
#include <stddef.h>

#ifdef	USE_ODBC

// Globals

SQLRecordEnum :: SQLRecordEnum ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pStmt			= NULL;
	hStmt			= SQL_NULL_HANDLE;
	pCols			= NULL;
	bEnd			= false;
	ColumnCount	= 0;
	pBfr			= NULL;
	uBfrSz		= 0;
	}	// SQLRecordEnum

HRESULT SQLRecordEnum :: construct ( void )
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

	// Factory
	CCLTRY ( FactCache.AddRef() );

	// Buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ) );

	// Parser
	CCLTRY ( COCREATEINSTANCE ( CLSID_SysParserBin, IID_IParseStm, &pParse ) );

	return hr;
	}	// construct

void SQLRecordEnum :: destruct ( void )
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
	FactCache.Release();
	if (pCols != NULL) delete[] pCols;
	_RELEASE(pStmt);
	_RELEASE(pParse);
	_RELEASE(pBfr);
	}	// destruct

HRESULT SQLRecordEnum :: getObject ( SQLHANDLE hStmt, SQLUSMALLINT col,
													adtValue &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Loads an object from the specified column.
	//
	//	PARAMETERS
	//		-	hStmt is the statement handle
	//		-	col is the column number
	//		-	vValue will reecive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IByteStream	*pStm	= NULL;
	PVOID			pvBfr	= NULL;
	SQLRETURN	sqlr;
	SQLINTEGER	len;

	// Call SQLGetData a single time with a zero buffer so we can
	// get the total amount of data waiting for us
	CCLTRYE ( (sqlr = SQLGetData(hStmt,col,SQL_C_BINARY,&hr,0,&len)) == 
					SQL_SUCCESS_WITH_INFO, E_UNEXPECTED );
	CCLTRYE ( len > 0, E_UNEXPECTED );

	// Internal object buffer
	if (hr == S_OK && len > (SQLINTEGER)uBfrSz)
		{
		CCLTRY(pBfr->setSize ( len ));
		CCLOK (uBfrSz = len;)
		}	// if
	CCLTRY ( pBfr->lock ( 0, 0, &pvBfr, NULL ) );

	// Read blob
	CCLTRY(SQLSTMT(hStmt,SQLGetData(hStmt,col,SQL_C_BINARY,pvBfr,len,&len)));

	// Create a stream based on our memory block and parse it into an object
	CCLTRY(pBfr->stream ( &pStm ));
	CCLTRY(pParse->valueLoad ( pStm, vValue ));

	// Clean up
	_RELEASE(pStm);
	_UNLOCK(pBfr,pvBfr);

	return hr;
	}	// getObject

HRESULT SQLRecordEnum :: prepare ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Prepares the object's state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr					= S_OK;
	SQLSMALLINT	ColumnNumber	= 0;
	SQLSMALLINT	NameLength;
	SQLSMALLINT	DataType;
	SQLUINTEGER	ColumnSize;
	SQLSMALLINT	DecimalDigits;
	SQLSMALLINT	Nullable;
	SQLSMALLINT	TargetType;
	SQLPOINTER	TargetValuePtr;
	WCHAR			wColName[200];
	SQLRETURN	sqlr;
	SQLCol		*pCol;

	// State check
	CCLTRYE ( (hStmt != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

	// Allocate space for each column's data
	CCLTRY ( SQLSTMT(hStmt,SQLNumResultCols ( hStmt, &ColumnCount)) );
	CCLTRYE( (pCols = new SQLCol[ColumnCount]) != NULL, E_OUTOFMEMORY );

	// Prepare result arrays by creating and binding the data types.  Blobs
	// are read later.
	while (hr == S_OK &&
				((sqlr = SQLDescribeCol ( hStmt, ColumnNumber+1, wColName,
					sizeof(wColName), &NameLength, &DataType, &ColumnSize,
					&DecimalDigits, &Nullable )) == SQL_SUCCESS) )
		{
		// Name
		pCol						= &(pCols[ColumnNumber]);
		pCol->sName				= wColName;
//		OutputDebugString ( L"SQLRecordEnum::prepare:" );
//		OutputDebugString ( pCol->sName );
//		OutputDebugString ( L"\n" );

		// Data type
		pCol->DataType = DataType;
		switch (DataType)
			{
			case SQL_INTEGER :
				pCol->sData		= adtInt();
				pCol->uSz		= sizeof(pCol->sData.vint);
				TargetType		= SQL_C_ULONG;
				TargetValuePtr	= &(pCol->sData.vint);
				break;
			case SQL_BIGINT :
				pCol->sData		= adtLong();
				pCol->uSz		= sizeof(pCol->sData.vlong);
				TargetType		= SQL_C_UBIGINT;
				TargetValuePtr	= &(pCol->sData.vlong);
				break;
			case SQL_REAL :
				pCol->sData		= adtFloat();
				pCol->uSz		= sizeof(pCol->sData.vflt);
				TargetType		= SQL_C_FLOAT;
				TargetValuePtr	= &(pCol->sData.vflt);
				break;
			case SQL_DOUBLE :
				pCol->sData.vtype	= VALT_R8;;
				pCol->uSz			= sizeof(pCol->sData.vdbl);
				TargetType			= SQL_C_DOUBLE;
				TargetValuePtr		= &(pCol->sData.vdbl);
				break;
			case SQL_TYPE_TIMESTAMP :
				// Apparently dates can be STORED as doubles but not RETRIEVED as doubles
				pCol->uSz		= sizeof(pCol->TimeStamp);
				TargetType		= SQL_C_TIMESTAMP;
				TargetValuePtr	= &(pCol->TimeStamp);
				break;
			case SQL_WCHAR :
			case SQL_WVARCHAR :
				{
				adtString	sData;

				// Allocate max. size.
				sData.allocate ( ColumnSize );
				sData.at(0)				= WCHAR('\0');
				TargetValuePtr			= (SQLPOINTER) &sData.at();
				adtValueImpl::copy ( pCol->sData, sData );
				pCol->uSz				= ColumnSize*sizeof(WCHAR);
				TargetType				= SQL_C_WCHAR;
				}
				break;
			case SQL_BINARY :
			case SQL_VARBINARY :
			case SQL_LONGVARBINARY :
				pCol->sData.vtype	= VALT_UNKNOWN;
				TargetType		= SQL_C_BINARY;
				TargetValuePtr	= NULL;
				break;
			default :
				hr = E_UNEXPECTED;
			}	// switch

		// Bind column if not binary
		if (hr == S_OK && TargetType != SQL_C_BINARY)
			{
			CCLTRY ( SQLSTMT(hStmt,SQLBindCol ( hStmt, ColumnNumber+1, TargetType,
							TargetValuePtr, pCol->uSz, &(pCol->StrLen_or_Ind) )) );
			}	// if

		// Next column
		++ColumnNumber;
		}	// while

	return hr;
	}	// prepare

HRESULT SQLRecordEnum :: receive ( IReceptor *pr, const WCHAR *pl, 
												const ADTVALUE &v )
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

	// Next
	if (prNext == pR)
		{
		IADTDictionary	*pDict	= NULL;
		SQLRETURN		sqlr;
		S32				c;
		adtValueImpl	vValue;

		// State check
		CCLTRYE ( (hStmt != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

		// Fetch data for next
		if (hr == S_OK)
			{
			sqlr = SQLFetch ( hStmt );
			hr = (sqlr == SQL_NO_DATA) ? ERROR_NOT_FOUND : SQLSTMT(hStmt,sqlr);
	//if (hr == ERROR_NOT_FOUND)
	//	OutputDebugString ( L"SQLRecordEnum::receiveNext:No data!\n" );
			}	// if

		// If successful, output result
		if (hr == S_OK)
			{
			// Output dictionary
			CCLTRY ( COCREATEINSTANCEC ( FactCache, CLSID_ADTDictionary, 
													IID_IADTDictionary, &pDict ) );

			// Store all values
			for (c = 0;hr == S_OK && c < ColumnCount;++c)
				{
				// See date comments in 'prepare'
				if (	hr == S_OK &&
						pCols[c].DataType == SQL_TYPE_TIMESTAMP )
					{
					SYSTEMTIME	st;
					// Convert fields
					st.wYear				= pCols[c].TimeStamp.year;
					st.wMonth			= pCols[c].TimeStamp.month;
					st.wDay				= pCols[c].TimeStamp.day;
					st.wHour				= pCols[c].TimeStamp.hour;
					st.wMinute			= pCols[c].TimeStamp.minute;
					st.wSecond			= pCols[c].TimeStamp.second;
					// 'fraction' is number of nanosecond but only the millisecond
					// portion is valid in 'datetime' data types
					st.wMilliseconds	= (WORD) pCols[c].TimeStamp.fraction/1000000;

					// Convert to date
					CCLTRY ( adtDate::fromSystemTime ( &st, &(pCols[c].sData.vdate) ) );
					CCLOK  ( pCols[c].sData.vtype = VALT_DATE; )
					}	// if

				// If item is an object, it has been streamed.  Load whatever
				// value is stored there
				else if (hr == S_OK && pCols[c].sData.vtype == VALT_UNKNOWN )
					hr = getObject ( hStmt, c+1, pCols[c].sData );

				// Store, do not overwrite duplicates, this means duplicate field names
				// that show up will not get stored (usually happens in joins).
				if (hr == S_OK && pDict->load ( pCols[c].sName, vValue ) != S_OK)
					{
					// Send new refrence of string data
					if (pCols[c].sData.vtype == VALT_STR && pCols[c].sData.pstr != NULL)
						hr = pDict->store ( pCols[c].sName, adtString ( (wchar_t *) &(pCols[c].sData.pstr[1]) ) );
					else
						hr = pDict->store ( pCols[c].sName, pCols[c].sData );
					}	// if
				}	// for
			}	// if

		// Result
		if (hr == S_OK)	peFire->emit(adtIUnknown(pDict));
		else					peEnd->emit(adtInt());

		// Clean up
		_RELEASE(pDict);
		}	// if

	// Position
	else if (prPos == pR)
		{
		adtInt			iPos(v);

		// State check
		CCLTRYE ( (hStmt != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

		// For scrollable cursors this will set the position of the record
		// that is read the next time 'Next' is received.
		// We are 1 based instead of zero based.
		CCLTRY ( SQLSTMT(hStmt,SQLFetchScroll ( hStmt, SQL_FETCH_ABSOLUTE, iPos-1 )) );
		}	// else if

	// Context
	else if (prCtx == pR)
		{
		adtIUnknown unkV(v);
		adtLong		lTmp;

		// Reset previous state
		_RELEASE(pStmt);
		if (pCols != NULL)
			{
			delete[] pCols;
			pCols = NULL;
			}	// if
		bEnd = false;

		// New state
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pStmt));
		CCLTRY(pStmt->getValue ( lTmp ));
		CCLOK (hStmt = (SQLHANDLE)(U64)lTmp;)

		// Prepare with new statement handle
		CCLTRY(prepare());
		}	// else if

	// Count
//	else if (prCnt == pR)
		//	iMax = adtInt(v);

	return hr;
	}	// receive

#endif

#ifdef	USE_OLEDB

SQLRecordEnum :: SQLRecordEnum ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pRowset		= NULL;
	psCols		= NULL;
	pbCols		= NULL;
	hAccessor	= NULL;
	bEnd			= false;
	pBfr			= NULL;
	uBfrSz		= 0;
	nColumns		= 0;
	iCount		= 0;
	iMax			= 0;

	// Interfaces
	addInterface ( IID_INodeBehaviour, (INodeBehaviour *) this );
	}	// SQLRecordEnum

HRESULT SQLRecordEnum :: construct ( void )
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

	// Factory
	CCLTRY ( FactCache.AddRef() );

	// Buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ) );

	// Parser
	CCLTRY ( COCREATEINSTANCE ( CLSID_SysParserBin, IID_IParseStm, &pParse ) );

	return hr;
	}	// construct

void SQLRecordEnum :: destruct ( void )
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
	if (psCols != NULL) delete[] psCols;
	if (pbCols != NULL) delete[] pbCols;
	_RELEASE(pRowset);
	_RELEASE(pParse);
	_RELEASE(pBfr);
	FactCache.Release();
	}	// destruct

HRESULT SQLRecordEnum :: prepare ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Prepares the object's state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr					= S_OK;
	IColumnsInfo	*pColsInfo		= NULL;
	IAccessor		*pAccessor		= NULL;
	DBCOLUMNINFO	*pdbci			= NULL;
	WCHAR				*pwStrs			= NULL;
	DBORDINAL		idx;
	U32				uSz,uPos;

	// State check
	CCLTRYE ( pRowset != NULL, ERROR_INVALID_STATE );

	// Needed interfaces
	CCLTRY	( _QI(pRowset,IID_IColumnsInfo,&pColsInfo) );
	CCLTRY	( _QI(pRowset,IID_IAccessor,&pAccessor) );

	// Get information about columns in rowset
	CCLTRY	( pColsInfo->GetColumnInfo ( &nColumns, &pdbci, &pwStrs ) );

	// Allocate memory for binding descriptors and column names
	CCLTRYE	( (psCols = new adtString[nColumns]) != NULL,		E_OUTOFMEMORY );
	CCLTRYE	( (pbCols = new DBBINDING[nColumns]) != NULL,		E_OUTOFMEMORY );
	CCLOK		( memset ( pbCols, 0, nColumns*sizeof(DBBINDING) ); )

	// Prebind buffers for results
	for (idx = 0,uSz = 0,uPos = 0;hr == S_OK && idx < nColumns;++idx)
		{
		// Leave room for status
		pbCols[idx].obStatus		= uSz;
		uSz							+= sizeof(DBBINDSTATUS);

		// Binding information for current column
		pbCols[idx].iOrdinal		= pdbci[idx].iOrdinal;
		pbCols[idx].obValue		= uSz;
		pbCols[idx].dwPart		= DBPART_VALUE|DBPART_STATUS;
		pbCols[idx].dwMemOwner	= DBMEMOWNER_CLIENTOWNED;
		pbCols[idx].eParamIO		= DBPARAMIO_NOTPARAM;
		pbCols[idx].cbMaxLen		= pdbci[idx].ulColumnSize;
		pbCols[idx].wType			= pdbci[idx].wType;
		pbCols[idx].bPrecision	= pdbci[idx].bPrecision;
		pbCols[idx].bScale		= pdbci[idx].bScale;
		uSz							+= pbCols[idx].cbMaxLen;

		// Make sure data stays word aligned
		if (uSz % 4) uSz += (4 - (uSz % 4));

		// Column name
		psCols[idx] = &(pwStrs[uPos]);
		uPos			+= (psCols[idx].length()+1);

		}	// for

	// Allocate an acessor for the data
	CCLTRY ( pAccessor->CreateAccessor ( DBACCESSOR_ROWDATA, nColumns, pbCols, 0,
														&hAccessor, NULL ) );

	// Make sure our internal buffer is big enough
	if (hr == S_OK && uSz > uBfrSz)
		{
		CCLTRY ( pBfr->setSize ( uSz ) );
		CCLOK  ( uBfrSz = uSz; )
		}	// if

	// Clean up
	_FREETASKMEM(pwStrs);
	_FREETASKMEM(pdbci);
	_RELEASE(pAccessor);
	_RELEASE(pColsInfo);

	return hr;
	}	// prepare

HRESULT SQLRecordEnum :: receiveContext ( const adtValue &v )
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
	HRESULT		hr				= S_OK;
	IAccessor	*pAccessor	= NULL;
	adtIUnknown unkV(v);

	// Reset previous state
	if (	hAccessor != NULL && pRowset != NULL &&
			_QI(pRowset,IID_IAccessor,&pAccessor) == S_OK )
		{
		pAccessor->ReleaseAccessor(hAccessor,NULL);
		hAccessor = NULL;
		pAccessor->Release();
		}	// if
	_RELEASE(pRowset);
	if (psCols != NULL) { delete[] psCols; psCols = 0; }
	if (pbCols != NULL) { delete[] pbCols; pbCols = 0; }
	iCount	= 0;
	nColumns	= 0;
	bEnd		= false;

	// New state
	CCLTRY(_QISAFE(unkV,IID_IRowset,&pRowset));

	// Prepare with context
	CCLTRY(prepare());

	return hr;
	}	// receiveContext

HRESULT SQLRecordEnum :: receiveNext ( const adtValue &v )
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
	HRESULT			hr			= S_OK;
	IADTDictionary	*pDict	= NULL;
	U8					*pcBfr	= NULL;
	U8					*pcBfrNow;
	HROW				hRows[1];
	HROW				*phRow	= &(hRows[0]);
	DBCOUNTITEM		dbci;
	U32				uPos,c;
	DBBINDSTATUS	status;
	adtValueImpl	vValue;
	bool				bDup;

	// State check
	CCLTRYE ( pRowset != NULL, ERROR_INVALID_STATE );

	// Support a maximum iteration count.  This is for database row sets
	// that may not support limiting the count at query time
	CCLTRYE ( !iMax || (iCount < iMax), S_FALSE );

	// Get data for next row
	CCLTRY ( pBfr->lock				( 0, 0, (void **) &pcBfr, NULL ) );
	CCLTRY ( pRowset->GetNextRows ( NULL, 0, 1, &dbci, &phRow ) );
	CCLTRY ( pRowset->GetData		( *phRow, hAccessor, pcBfr ) );

	// If successful, output result
	if (hr == S_OK)
		{
		// Output dictionary
		CCLTRY ( COCREATEINSTANCEC ( FactCache, CLSID_ADTDictionary, 
												IID_IADTDictionary, &pDict ) );

		// Store all values
		for (c = 0,uPos = 0;hr == S_OK && c < nColumns;++c)
			{
			// Do not overwrite duplicates, this means duplicate field names
			// that show up will not get stored (usually happens in joins).
			bDup = (pDict->load ( psCols[c], vValue ) == S_OK);

			// Status
			pcBfrNow = &(pcBfr[uPos]);
			memcpy ( &status, pcBfrNow, sizeof(DBBINDSTATUS) );
			uPos		+= sizeof(DBBINDSTATUS);

			// All data stored to buffer in a row, parse it out and
			// store in dictionary based on type
			pcBfrNow = &(pcBfr[uPos]);
			switch (pbCols[c].wType)
				{
				case DBTYPE_WSTR :
					{
					// Store in database
					if (hr == S_OK && !bDup)
						hr = pDict->store ( psCols[c], adtString((WCHAR *)pcBfrNow) );

					// Skip to next block
					CCLOK ( uPos += pbCols[c].cbMaxLen; )
					}	// DBTYPE_WSTR
					break;
				case DBTYPE_I4 :
					{
					// Store in database
					if (hr == S_OK && !bDup)
						hr = pDict->store ( psCols[c], adtInt(*((U32 *)(pcBfrNow))) );

					// Skip to next block
					CCLOK ( uPos += sizeof(U32); )
					}	// DBTYPE_I4
					break;
				default :
					hr = E_NOTIMPL;
				}	// switch

			// Keep position word aligned as during bind
			if (uPos % 4) uPos += (4 - (uPos % 4));
			}	// for
		}	// if

	// Result
	if (hr == S_OK)
		{
		++iCount;
		peFire->emit(adtIUnknown(pDict));
		}	// if
	else					peEnd->emit(adtInt());

	// Clean up
	if (*phRow != NULL) pRowset->ReleaseRows ( 1, phRow, NULL, NULL, NULL );
	_RELEASE(pDict);
	_UNLOCK(pBfr,pcBfr);

	return hr;
	}	// receiveNext

HRESULT SQLRecordEnum :: receivePosition ( const adtValue &v )
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
	HRESULT			hr			= S_OK;
/*
	adtInt			iPos(v);

	// State check
	CCLTRYE ( (hStmt != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

	// For scrollable cursors this will set the position of the record
	// that is read the next time 'Next' is received.
	// We are 1 based instead of zero based.
	CCLTRY ( SQLSTMT(hStmt,SQLFetchScroll ( hStmt, SQL_FETCH_ABSOLUTE, iPos-1 )) );
*/
	return hr;
	}	// receivePosition

//
// Context
//

HRESULT SQLRecordEnum :: receiveCount ( const adtValue &v )
	{
	iMax = adtInt(v);
	return S_OK;
	}	// receiveCount

#endif
