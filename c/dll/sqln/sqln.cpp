////////////////////////////////////////////////////////////////////////
//
//									SQLN.CPP
//
//			Main file for the SQL database node library
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) nSpace, LLC
   All right reserved
*/

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Sql"

// Library implementations
#include "../../lib/sqll/sqll_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(Connection)
	CCL_OBJLIST_ENTRY	(Query)
	CCL_OBJLIST_ENTRY	(RecordEnum)

CCL_OBJLIST_END()


#ifdef	USE_OLEDB

HRESULT OLEDBAppendConstraints ( IADTContainer *pCons, WCHAR *pwQry )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Appends the SQL to the pwQry string appropriate to the given
	//			constraints.
	//
	//	PARAMETERS
	//		-	pCons are the container of constraints
	//		-	pwQry will have appended the 'WHERE ...' clause
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IADTDictionary	*pDict	= NULL;
	IADTIt			*pIt		= NULL;
	IADTInIt			*pIn		= NULL;
	adtIUnknown		unkV;
	adtString		strV;

	// Container setup
	CCLTRYE(pCons->isEmpty() == S_FALSE,E_UNEXPECTED);
	CCLTRY (pCons->iterate ( &pIt ));
	CCLTRY (_QI(pIt,IID_IADTInIt,&pIn));
	CCLTRY (pIn->begin());

	// WHERE
	CCLOK	( wcscat ( pwQry, L" WHERE " ); )

	// Append string
	while (hr == S_OK && pIn->read ( unkV ) == S_OK)
		{
		// Properties
		CCLTRY( _QISAFE(unkV,IID_IADTDictionary,&pDict) );

		// Name
		CCLTRY( pDict->load ( strRefName, strV ) );
		CCLOK ( wcscat ( pwQry, L"[" ); )
		CCLOK	( wcscat ( pwQry, strV ); )
		CCLOK ( wcscat ( pwQry, L"] " ); )

		// Constraint operator ( <, >, LIKE, etc. )
		CCLTRY( pDict->load ( strRefOp, strV ) );
		CCLOK	( wcscat ( pwQry, strV ); )

		// Value
		CCLOK ( wcscat ( pwQry, L" ? " ); )

		// Next constraint
		wcscat ( pwQry, L"AND " );
		_RELEASE(pDict);
		pIn->next();
		}	// while

	// Terminate query buffer before the last " AND "
	CCLOK ( pwQry[adtString::length(pwQry)-5] = WCHAR('\0'); )

	// Clean up
	_RELEASE(pIn);
	_RELEASE(pIt);

	return hr;
	}	// OLEDBAppendConstraints

HRESULT OLEDBApplyConstraints ( IADTContainer *pCons, IMemoryMapped *pBfr,
											DBBINDING *pbCons, U32 *puBfrPos )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Applies the specified constraints by binding the columns to
	//			the given 'DBBINDINGS's and by copy the constraint values into
	//			the provided memory buffer.
	//
	//	PARAMETERS
	//		-	pCons are the container of constraints
	//		-	pBfr will receive the constraint values
	//		-	pbCons will receieve the binding information.  The 'pdb' array
	//			should have enough room to contain all constraints
	//		-	puBfrPos is/will be the current buffer position
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IADTDictionary	*pDict	= NULL;
	IADTIt			*pIt		= NULL;
	IADTInIt			*pIn		= NULL;
	U8					*pcBfr	= NULL;
	adtIUnknown		unkV;
	adtString		strV;
	U32				szCons,uBfrSz,idx;
	adtValueImpl	vVal;
	bool				bLike;

	// Container setup
	CCLTRY (pCons->size ( &szCons ));
	CCLTRYE(szCons > 0, E_UNEXPECTED);
	CCLTRY (pCons->iterate ( &pIt ));
	CCLTRY (_QI(pIt,IID_IADTInIt,&pIn));
	CCLTRY (pIn->begin());

	// Reset binding information
	CCLOK		( memset ( pbCons, 0, szCons*sizeof(DBBINDING) ); )
	CCLTRY	( pBfr->lock ( 0, 0, (void **) &pcBfr, &uBfrSz ) );

	// Bind constraint values
	CCLOK		( idx = 0; )
	CCLTRY	( pIn->begin() );
	while (hr == S_OK && pIn->read ( unkV ) == S_OK)
		{
		// Properties
		CCLTRY( _QISAFE(unkV,IID_IADTDictionary,&pDict) );

		// Constraint value
		CCLTRY( pDict->load ( strRefValue,	vVal ) );

		// Constraint operator ( <, >, LIKE, etc. )
		CCLTRY( pDict->load ( strRefOp, strV ) );
		CCLOK ( bLike = (WCASECMP(strV,L"LIKE") == 0); )

		// If the operator is 'like' and the value is a string, add the wildcards
		if (hr == S_OK && bLike && vVal.vt == VALT_BSTR)
			{
			CCLTRY ( strV.allocate ( adtString::length(vVal.bstrVal) + 2 ) );
			CCLOK  ( wcscpy ( strV, L"%" ); )
			CCLOK  ( wcscat ( strV, vVal.bstrVal ); )
			CCLOK  ( wcscat ( strV, L"%" ); )
			CCLTRY ( adtValueImpl::copy ( vVal, strV ) );
			}	// if

		// Initialize the binding information for value
		CCLTRY ( OLEDBBindVariant ( idx+1, &vVal, &(pbCons[idx]), (*puBfrPos) ) );

		// Internal buffer big enough for next block ?
		if ((*puBfrPos)+pbCons[idx].cbMaxLen > uBfrSz)
			{
			_UNLOCK(pBfr,pcBfr);
			CCLTRY	( pBfr->setSize ( (*puBfrPos)+pbCons[idx].cbMaxLen+1024 ) );
			CCLTRY	( pBfr->lock ( 0, 0, (void **) &pcBfr, &uBfrSz ) );
			}	// if

		// Copy constraint value
		CCLTRY ( OLEDBCopyVariant ( &(pcBfr[(*puBfrPos)]), &vVal ) );

		// Keep the buffer position, word aligned
		if (hr == S_OK)
			{
			(*puBfrPos) += pbCons[idx].cbMaxLen;
			if ((*puBfrPos) % 4) (*puBfrPos) += (4 - ((*puBfrPos) % 4));
			}	// if

		// Next constraint
		CCLOK ( ++idx; )
		_RELEASE(pDict);
		pIn->next();
		}	// while

	// Clean up
	_RELEASE(pIn);
	_RELEASE(pIt);

	return hr;
	}	// OLEDBApplyConstraints

HRESULT OLEDBBindVariant	( DBORDINAL dbord, adtValue *pV, DBBINDING *pDB,
										U32 uPos )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Fills in an OLE DB bind structure with the information
	//			necessary to bind the specified value.
	//
	//	PARAMETERS
	//		-	dbord is the ordinal to use
	//		-	pV is the value
	//		-	pDB will receive the binding information
	//		-	uPos is the position of the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Common values
	if (hr == S_OK)
		{
		pDB->iOrdinal		= dbord;
		pDB->obValue		= uPos;
		pDB->dwPart			= DBPART_VALUE;
		pDB->dwMemOwner	= DBMEMOWNER_CLIENTOWNED;
		pDB->eParamIO		= DBPARAMIO_INPUT;
		}	// if

	// Fill based on type
	switch (pV->vtype)
		{
		case VALT_BSTR :
			pDB->wType		= DBTYPE_WSTR;
			pDB->cbMaxLen	= (adtString::length(pV->bstrVal)+1)*sizeof(WCHAR);
			break;
		default :
			hr = E_NOTIMPL;
		}	// switch

	return hr;
	}	// OLEDBBindVariant

HRESULT OLEDBCopyVariant	( PVOID pBfr, adtValue *pV )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Copies the specified variant into a buffer prepared for binding.
	//
	//	PARAMETERS
	//		-	pBfr will receive the value
	//		-	pV is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Copy based on type
	switch (pV->vtype)
		{
		case VALT_BSTR :
			memcpy ( pBfr, pV->bstrVal,
						(adtString::length(pV->bstrVal)+1)*sizeof(WCHAR) );
			break;
		default :
			hr = E_NOTIMPL;
		}	// switch

	return hr;
	}	// OLEDBCopyVariant

#endif

#ifdef	USE_ODBC

HRESULT SQLBindVariantParam ( SQLHANDLE hStmt, U32 uNum,
										adtValue *pValue, SQLINTEGER *puSz )
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
	//		-	puSz will receive the length parameter
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
		*puSz				= 0;								// Default
		switch (pValue->vtype)
			{
			case VALT_I4 :
				ValueType			= SQL_C_ULONG;
				ParameterType		= SQL_INTEGER;
				ParameterValuePtr	= &(pValue->vint);
				break;

			case VALT_DATE :
				ValueType			= SQL_C_DOUBLE;
				ParameterType		= SQL_DOUBLE;
				ParameterValuePtr	= &(pValue->vdbl);
				break;

			case VALT_STR :
				ValueType			= SQL_C_WCHAR;
				ParameterType		= SQL_WCHAR;
				ParameterValuePtr	= &(pValue->pstr[1]);
				ColumnSize			= (U32)wcslen(adtString(*pValue));
				BufferLength		= ColumnSize*sizeof(WCHAR);
				*puSz					= SQL_NTS;
				break;

			case VALT_R4 :
				ValueType			= SQL_C_FLOAT;
				ParameterType		= SQL_REAL;
				ParameterValuePtr	= &(pValue->vflt);
				break;

			case VALT_R8 :
				ValueType			= SQL_C_DOUBLE;
				ParameterType		= SQL_DOUBLE;
				ParameterValuePtr	= &(pValue->vdbl);
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
				if (	pValue->punk != NULL &&
						_QI(pValue->punk,IID_IMemoryMapped,&pMem) == S_OK )
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
				else if (	pValue->punk != NULL &&
								_QI(pValue->punk,IID_IByteStream,&pStm) == S_OK)
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
				CCLOK ( (*puSz) = SQL_LEN_DATA_AT_EXEC(((SQLINTEGER)ColumnSize)); )
				CCLOK ( ParameterValuePtr = pValue->punk; )
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
									0,
									ParameterValuePtr,
									BufferLength,
									puSz ) )));

	return hr;
	}	// SQLBindVariantParam

HRESULT SQLHandleError ( SQLSMALLINT hType, SQLHANDLE hSQL, SQLRETURN rSQL )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Handle interpretation of the SQL error.
	//
	//	PARAMETERS
	//		-	hType is the handle type
	//		-	hSQL is the handle
	//		-	rSQL is the SQL return code
	//
	//	RETURN VALUE
	//		S_OK if no error
	//
	////////////////////////////////////////////////////////////////////////
	SQLSMALLINT	recnum = 1;
	SQLRETURN	r;
	SQLTCHAR		state[6];
	SQLINTEGER	inative;
	SQLTCHAR		sqlMsg[200];
	SQLSMALLINT	mLen;

	// Ok ?
	if (rSQL == SQL_SUCCESS) return S_OK;

	// Get errors
	r = SQL_SUCCESS;
	while (r == SQL_SUCCESS || r == SQL_SUCCESS_WITH_INFO)
		{
		// Next error record
		r = SQLGetDiagRec ( hType, hSQL, recnum++, state, &inative, sqlMsg,
									sizeof(sqlMsg)/sizeof(SQLTCHAR), &mLen );
		if (r != SQL_SUCCESS && r != SQL_SUCCESS_WITH_INFO) continue;

		// Debug
		OutputDebugString ( sqlMsg );
		OutputDebugString ( L" State  :" );
		OutputDebugString ( state );
		OutputDebugString ( L"\n" );
		}	// while
										
	return (rSQL == SQL_SUCCESS || rSQL == SQL_SUCCESS_WITH_INFO) ? S_OK : rSQL;
	}	// SQLHandleError

HRESULT SQLStringItLen ( IADTInIt *pIn, U32 *ns, U32 *slen )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to sum the lengths of all the BSTRs in the specified
	//			iterator.
	//
	//	PARAMETERS
	//		-	pIn is the iterator
	//		-	ns will receive the # of strings
	//		-	slen will receive the total length of all the strings
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	vStr;

	// Setup
	(*ns)		= 0;
	(*slen)	= 0;

	// Count
	CCLTRY(pIn->begin());
	while (hr == S_OK && pIn->read ( vStr ) == S_OK)
		{
		// Length
		(*slen) += (U32)wcslen ( vStr );

		// Next field
		++(*ns);
		pIn->next();
		}	// while

	return hr;
	}	// SQLStringItLen

HRESULT SQLVtToSQLC ( VARTYPE vt, SQLSMALLINT *pType )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts a 'VARTYPE' to an SQL 'C' data type.
	//
	//	PARAMETERS
	//		-	vt is the VARTYPE
	//		-	pType will receive the SQL C type
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Convert
	switch (vt)
		{
		case VALT_I4 :
			(*pType) = SQL_C_ULONG;
			break;
		case VALT_I8 :
			(*pType) = SQL_C_UBIGINT;
			break;
		case VALT_R4 :
			(*pType) = SQL_C_FLOAT;
			break;
		case VALT_R8 :
			(*pType) = SQL_C_DOUBLE;
			break;
		case VALT_DATE :
			(*pType) = SQL_C_DOUBLE;
			break;
		case VALT_STR :
			(*pType) = SQL_C_WCHAR;
			break;
		case VALT_UNKNOWN :
			(*pType) = SQL_C_BINARY;
			break;
		default :
			hr = E_INVALIDARG;
		}	// switch

	return hr;
	}	// SQLVtToSQLC

HRESULT SQLVtToSQLType ( VARTYPE vt, SQLSMALLINT *pType )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts a 'VARTYPE' to an SQL defined data type.
	//
	//	PARAMETERS
	//		-	vt is the VARTYPE
	//		-	pType will receive the SQL type
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Convert
	switch (vt)
		{
		case VALT_I4 :
			(*pType) = SQL_INTEGER;
			break;
		case VALT_I8 :
			(*pType) = SQL_BIGINT;
			break;
		case VALT_R4 :
			(*pType) = SQL_REAL;
			break;
		case VALT_R8 :
			(*pType) = SQL_DOUBLE;
			break;
		case VALT_DATE :
			(*pType) = SQL_DOUBLE;
			break;
		case VALT_STR :
			(*pType) = SQL_WCHAR;
			break;
		case VALT_UNKNOWN :
			(*pType) = SQL_BINARY;
			break;
		default :
			hr = E_INVALIDARG;
		}	// switch

	return hr;
	}	// SQLVtToSQLType

#endif
