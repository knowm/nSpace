////////////////////////////////////////////////////////////////////////
//
//									SQLLCPP
//
//					Utility functions for SQL library
//
////////////////////////////////////////////////////////////////////////

#include "sqll_.h"

HRESULT SQLBindVariantParam ( SQLHANDLE hStmt, U32 uNum,
										ADTVALUE *pValue, 
										const WCHAR *pwAltType, VOID *pvAltPtr,
										SQLLEN *puSz )
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
	//		-	wAltType can optionally specify an 'alternative' type if
	//			type is to be different that the default.
	//		-	pvAltPtr can optionally specify alternative ptr. to the data.
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
			case VTYPE_I4 :
				ValueType			= SQL_C_ULONG;
				ParameterType		= SQL_INTEGER;
				ParameterValuePtr	= &(pValue->vint);
				break;

			case VTYPE_DATE :
				ValueType			= SQL_C_DOUBLE;
				ParameterType		= SQL_DOUBLE;
				ParameterValuePtr	= &(pValue->vdbl);
				break;

			case VTYPE_STR :
				// No alternate type specified
				if (pwAltType == NULL || !wcslen(pwAltType))
					{
					ValueType			= SQL_C_WCHAR;
					ParameterType		= SQL_WCHAR;
					ParameterValuePtr	= &(pValue->pstr[1]);
					ColumnSize			= (U32)wcslen(adtString(*pValue));
					BufferLength		= ColumnSize*sizeof(WCHAR);
					*puSz					= SQL_NTS;
					}	// if
				#if (ODBCVER >= 0x0350)
				else if (!WCASECMP(pwAltType,L"GUID") && pvAltPtr != NULL)
					{
					ValueType			= SQL_C_GUID;
					ParameterType		= SQL_GUID;
					ParameterValuePtr	= pvAltPtr;
					}	// else if
				#endif
				else
					hr = E_INVALIDARG;
				break;

			case VTYPE_R4 :
				ValueType			= SQL_C_FLOAT;
				ParameterType		= SQL_REAL;
				ParameterValuePtr	= &(pValue->vflt);
				break;

			case VTYPE_R8 :
				ValueType			= SQL_C_DOUBLE;
				ParameterType		= SQL_DOUBLE;
				ParameterValuePtr	= &(pValue->vdbl);
				break;

			// Objects are 'pre-streamed'.
			// Obtain size of region.
			case VTYPE_UNK :
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
					U64	then,now;

					// Determine size by seeking to end of stream
					// from current position
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_CUR, &then ) );
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_END, &now ) );
					CCLTRY ( pStm->seek ( then, STREAM_SEEK_SET, NULL ) );
					CCLOK	 ( ColumnSize = (SQLUINTEGER)now; )

					// Clean up
					_RELEASE(pStm);
					}	// else if

				// Match buffer length
				CCLOK ( BufferLength = ColumnSize; )
				CCLOK ( (*puSz) = SQL_LEN_DATA_AT_EXEC(((SQLINTEGER)ColumnSize)); )
				CCLOK ( ParameterValuePtr = pValue->punk; )
				}	// VTYPE_UNK
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

HRESULT SQLStringItLen ( IIt *pIn, U32 *ns, U32 *slen )
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
		case VTYPE_I4 :
			(*pType) = SQL_C_ULONG;
			break;
		case VTYPE_I8 :
			(*pType) = SQL_C_UBIGINT;
			break;
		case VTYPE_R4 :
			(*pType) = SQL_C_FLOAT;
			break;
		case VTYPE_R8 :
			(*pType) = SQL_C_DOUBLE;
			break;
		case VTYPE_DATE :
			(*pType) = SQL_C_DOUBLE;
			break;
		case VTYPE_STR :
			(*pType) = SQL_C_WCHAR;
			break;
		case VTYPE_UNK :
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
		case VTYPE_I4 :
			(*pType) = SQL_INTEGER;
			break;
		case VTYPE_I8 :
			(*pType) = SQL_BIGINT;
			break;
		case VTYPE_R4 :
			(*pType) = SQL_REAL;
			break;
		case VTYPE_R8 :
			(*pType) = SQL_DOUBLE;
			break;
		case VTYPE_DATE :
			(*pType) = SQL_DOUBLE;
			break;
		case VTYPE_STR :
			(*pType) = SQL_WCHAR;
			break;
		case VTYPE_UNK :
			(*pType) = SQL_BINARY;
			break;
		default :
			hr = E_INVALIDARG;
		}	// switch

	return hr;
	}	// SQLVtToSQLType
