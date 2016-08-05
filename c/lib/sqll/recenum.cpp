////////////////////////////////////////////////////////////////////////
//
//									RECENUM.CPP
//
//				Implementation of the SQL record enumerator
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) nSpace, LLC
   All right reserved
*/

#include "sqll_.h"
#include <stdio.h>
#include <stddef.h>

// Globals
extern SQLiteDll	sqliteDll;

RecordEnum :: RecordEnum ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pStmt		= NULL;
	pDct		= NULL;
	}	// RecordEnum

HRESULT RecordEnum :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
//		adtValue		vL;

		// Defaults
//		if (pnDesc->load ( adtString(L"Table"), vL ) == S_OK)
//			adtValue::toString ( vL, sTableName );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDct);
		_RELEASE(pStmt);
		}	// else

	return hr;
	}	// onAttach

HRESULT RecordEnum :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Next record
	if (_RCP(Next))
		{
		int ret;

		// State check
		CCLTRYE ( pStmt != NULL && pStmt->plite_stmt != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Move to the first/next result.  SQLITE_ROW means a new row of data is ready
		CCLTRYE( (ret = sqliteDll.sqlite3_step ( pStmt->plite_stmt ))
						== SQLITE_ROW, ret );

		// Read out data
		if (hr == S_OK)
			{
			U32	iCols = 0;

			// Number of columns in result set
			CCLTRYE ( (iCols = sqliteDll.sqlite3_column_count ( pStmt->plite_stmt )) > 0,
							E_UNEXPECTED );

			// Retrieve each column
			for (U32 c = 0;hr == S_OK && c < iCols;++c)
				{
				adtString	strCol;
				adtValue		vCol;

				// Column name
				strCol = (LPCWSTR) sqliteDll.sqlite3_column_name16 ( pStmt->plite_stmt, c );
				if (strCol.length() == 0)
					continue;

				// Retrieve the value for the column
				switch ((ret = sqliteDll.sqlite3_column_type ( pStmt->plite_stmt, c )))
					{
					case SQLITE_INTEGER :
						CCLTRY ( adtValue::copy ( 
							adtInt (
								sqliteDll.sqlite3_column_int ( pStmt->plite_stmt, c ) ), vCol ) );
						break;
					case SQLITE_FLOAT :
						CCLTRY ( adtValue::copy ( 
							adtDouble (
								sqliteDll.sqlite3_column_double ( pStmt->plite_stmt, c ) ), vCol ) );
						break;
					case SQLITE_TEXT :
						{
						adtString	strV;
						strV = (LPCWSTR) sqliteDll.sqlite3_column_text16 ( pStmt->plite_stmt, c );
						CCLTRY ( adtValue::copy ( strV, vCol ) );
						}	
						break;
					case SQLITE_NULL :
						adtValue::clear(vCol);
						break;
					case SQLITE_BLOB :
					default :
						lprintf ( LOG_WARN, L"Unhandled data type : %d\r\n", ret );
					}	// switch

				// Store in results
				CCLTRY ( pDct->store ( strCol, vCol ) );
				
				// Debug
//				lprintf ( LOG_INFO, L"Column %d) %s\r\n", c, (LPCWSTR)strCol );
				}	// for

			}	// if
	
		// Result
		if (hr == S_OK)
			_EMT(Next,adtIUnknown(pDct));
		else
			_EMT(End,adtInt(hr));

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_WARN, L"Next failed : %d\r\n", hr );
		}	// if

	// State
	else if (_RCP(Statement))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pStmt);
		pStmt = (SQLRef *)(IUnknown *)unkV;
		_ADDREF(pStmt);
		}	// else if
	else if (_RCP(Dictionary))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

#ifdef	USE_ODBC

// Globals

RecordEnum :: RecordEnum ( void )
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
	}	// RecordEnum

HRESULT RecordEnum :: construct ( void )
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
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBfr ) );

	// Parser
	CCLTRY ( COCREATE ( L"Io.StmPrsBin",  IID_IStreamPersist, &pParse ) );

	return hr;
	}	// construct

void RecordEnum :: destruct ( void )
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

HRESULT RecordEnum :: getObject ( SQLHANDLE hStmt, SQLUSMALLINT col,
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
	SQLLEN		len;

	// Call SQLGetData a single time with a zero buffer so we can
	// get the total amount of data waiting for us
	CCLTRYE ( (sqlr = SQLGetData(hStmt,col,SQL_C_BINARY,&hr,0,&len)) == 
					SQL_SUCCESS_WITH_INFO, E_UNEXPECTED );
	CCLTRYE ( len > 0, E_UNEXPECTED );

	// Internal object buffer
	if (hr == S_OK && len > (SQLINTEGER)uBfrSz)
		{
		CCLTRY(pBfr->setSize ( (U32)len ));
		CCLOK (uBfrSz = (U32)len;)
		}	// if
	CCLTRY ( pBfr->lock ( 0, 0, &pvBfr, NULL ) );

	// Read blob
	CCLTRY(SQLSTMT(hStmt,SQLGetData(hStmt,col,SQL_C_BINARY,pvBfr,len,&len)));

	// Create a stream based on our memory block and parse it into an object
	CCLTRY(pBfr->stream ( &pStm ));
	CCLTRY(pParse->load ( pStm, vValue ));

	// Clean up
	_RELEASE(pStm);
	_UNLOCK(pBfr,pvBfr);

	return hr;
	}	// getObject

HRESULT RecordEnum :: prepare ( void )
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
	SQLULEN		ColumnSize;
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
//		OutputDebugString ( L"RecordEnum::prepare:" );
//		OutputDebugString ( pCol->sName );
//		OutputDebugString ( L"\n" );

		// Data type
		pCol->DataType = DataType;
		switch (DataType)
			{
			case SQL_INTEGER :
				adtValue::copy(adtInt(),pCol->sData);
				pCol->uSz		= sizeof(pCol->sData.vint);
				TargetType		= SQL_C_ULONG;
				TargetValuePtr	= &(pCol->sData.vint);
				break;
			case SQL_BIGINT :
				adtValue::copy(adtLong(),pCol->sData);
				pCol->uSz		= sizeof(pCol->sData.vlong);
				TargetType		= SQL_C_UBIGINT;
				TargetValuePtr	= &(pCol->sData.vlong);
				break;
			case SQL_REAL :
				adtValue::copy(adtFloat(),pCol->sData);
				pCol->uSz		= sizeof(pCol->sData.vflt);
				TargetType		= SQL_C_FLOAT;
				TargetValuePtr	= &(pCol->sData.vflt);
				break;
			case SQL_DOUBLE :
				adtValue::copy(adtDouble(),pCol->sData);
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
				sData.allocate ( (U32)ColumnSize );
				sData.at(0)				= WCHAR('\0');
				TargetValuePtr			= (SQLPOINTER) &sData.at();
				adtValue::copy ( sData,  pCol->sData );
				pCol->uSz				= (U32)ColumnSize*sizeof(WCHAR);
				TargetType				= SQL_C_WCHAR;
				}
				break;
			case SQL_BINARY :
			case SQL_VARBINARY :
			case SQL_LONGVARBINARY :
				pCol->sData.vtype	= VTYPE_UNK;
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

HRESULT RecordEnum :: receive ( IReceptor *pr, const WCHAR *pl, 
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
	if (_RCP(Next))
		{
		IDictionary	*pDict	= NULL;
		SQLRETURN	sqlr;
		S32			c;
		adtValue		vValue;

		// State check
		CCLTRYE ( (hStmt != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

		// Fetch data for next
		if (hr == S_OK)
			{
			sqlr = SQLFetch ( hStmt );
			hr = (sqlr == SQL_NO_DATA) ? ERROR_NOT_FOUND : SQLSTMT(hStmt,sqlr);
	//if (hr == ERROR_NOT_FOUND)
	//	OutputDebugString ( L"RecordEnum::receiveNext:No data!\n" );
			}	// if

		// If successful, output result
		if (hr == S_OK)
			{
			// Output dictionary
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDict ) );

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
					CCLOK  ( pCols[c].sData.vtype = VTYPE_DATE; )
					}	// if

				// If item is an object, it has been streamed.  Load whatever
				// value is stored there
				else if (hr == S_OK && adtValue::type(pCols[c].sData) == VTYPE_UNK )
					hr = getObject ( hStmt, c+1, pCols[c].sData );

				// Store, do not overwrite duplicates, this means duplicate field names
				// that show up will not get stored (usually happens in joins).
				if (hr == S_OK && pDict->load ( pCols[c].sName, vValue ) != S_OK)
					{
					// Send new refrence of string data
					if (adtValue::type(pCols[c].sData) == VTYPE_STR && pCols[c].sData.pstr != NULL)
						hr = pDict->store ( pCols[c].sName, adtString ( (wchar_t *) &(pCols[c].sData.pstr[1]) ) );
					else
						hr = pDict->store ( pCols[c].sName, pCols[c].sData );
					}	// if
				}	// for
			}	// if

		// Result
		if (hr == S_OK)	_EMT(Fire,adtIUnknown(pDict));
		else					_EMT(End,adtInt());

		// Clean up
		_RELEASE(pDict);
		}	// if

	// Position
	else if (_RCP(Position))
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
	else if (_RCP(Context))
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

RecordEnum :: RecordEnum ( void )
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
	}	// RecordEnum

HRESULT RecordEnum :: construct ( void )
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

void RecordEnum :: destruct ( void )
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

HRESULT RecordEnum :: prepare ( void )
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

HRESULT RecordEnum :: receiveContext ( const adtValue &v )
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

HRESULT RecordEnum :: receiveNext ( const adtValue &v )
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
	IDictionary	*pDict	= NULL;
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
												IID_IDictionary, &pDict ) );

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

HRESULT RecordEnum :: receivePosition ( const adtValue &v )
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

HRESULT RecordEnum :: receiveCount ( const adtValue &v )
	{
	iMax = adtInt(v);
	return S_OK;
	}	// receiveCount

#endif
