////////////////////////////////////////////////////////////////////////
//
//									QUERY.CPP
//
//					Implementation of the SQL query node
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

#define	SIZE_SQLBFR		1024

// Globals

SQLQuery :: SQLQuery ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn			= NULL;
	hConn			= NULL;
	bDistinct	= false;
	pConsIn		= NULL;
	pFldsIn		= NULL;
	bCount		= false;
	pvCons		= NULL;
	pCons			= NULL;
	szCons		= 0;
	pQryBfr		= NULL;
	pwQryBfr		= NULL;
	pJoin			= NULL;
	iCount		= 0;
	}	// SQLQuery

HRESULT SQLQuery :: construct ( void )
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

	// SQL query string buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pQryBfr ) );
	CCLTRY ( pQryBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pQryBfr->lock ( 0, 0, (PVOID *) &pwQryBfr, NULL ) );

	return hr;
	}	// construct

void SQLQuery :: destruct ( void )
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
	if (pvCons != NULL) delete[] pvCons;
	_RELEASE(pCons);
	_RELEASE(pConsIn);
	_RELEASE(pFldsIn);
	_RELEASE(pJoin);
	_RELEASE(pConn);
	_UNLOCK(pQryBfr,pwQryBfr);
	_RELEASE(pQryBfr);
	}	// destruct

HRESULT SQLQuery :: receive ( IReceptor *pr, const WCHAR *pl, 
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

	// Fire
	if (prFire == pR)
		{
		IADTDictionary	*pDict	= NULL;
		SQLHandle		*pStmt	= NULL;
		IHaveValue		*phv;
		adtValueImpl	vVal;
		adtIUnknown		unkV;
		adtString		strV;
		U32				idx;
		bool				bLike;

		// State check
		CCLTRYE ( (hConn != NULL),					ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnAttr->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
		if (hr == S_OK && sSort.length() == 0)
			pnAttr->load ( strRefSort, sSort );

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Static query (allows SQLFetchScroll)
		CCLTRY ( SQLSTMT(pStmt->Handle,SQLSetStmtAttr ( pStmt->Handle,
			SQL_ATTR_CURSOR_TYPE, (SQLPOINTER) SQL_CURSOR_STATIC, SQL_IS_INTEGER )));

		// Count limit ?
		if (hr == S_OK && iCount > (U32)0)
			{
			CCLTRY ( SQLSTMT(pStmt->Handle,SQLSetStmtAttr ( pStmt->Handle,
				SQL_ATTR_MAX_ROWS, (SQLPOINTER) (U64) iCount.vint, SQL_IS_INTEGER )));
			}	// if

		// Time to generate query string.
		// Form is : SELECT (Fields) FROM (Table) WHERE (Constraints)

		// SELECT
		CCLOK		( wcscpy ( pwQryBfr, L"SELECT " ); )

		// Distinct ?
		if (hr == S_OK && bDistinct == true)
			wcscat ( pwQryBfr, L"DISTINCT " );

		// Fields (F1,F2,...,Fn) or All (*)
		if (hr == S_OK && pFldsIn != NULL)
			{
			CCLTRY	( pFldsIn->begin() );
			while (hr == S_OK && pFldsIn->read ( vVal ) == S_OK)
				{
				// Field name
//				CCLOK ( wcscat ( pwQryBfr, L"[" ); )
				CCLOK ( wcscat ( pwQryBfr, adtString(vVal) ); )
//				CCLOK ( wcscat ( pwQryBfr, L"]," ); )
				CCLOK ( wcscat ( pwQryBfr, L"," ); )

				// Next
				pFldsIn->next();
				}	// while
			CCLOK ( pwQryBfr[wcslen(pwQryBfr)-1] = WCHAR('\0'); )
			}	// if
		else if (hr == S_OK && pFldsIn == NULL)
			CCLOK ( wcscat ( pwQryBfr, L"*" ); )

		// FROM
		CCLOK		( wcscat ( pwQryBfr, L" FROM " ); )

		// Oddity: Access seems to need parathesis, used as embedded joins
		// Count number of joins and prepend the correct number of left parens.
		if (hr == S_OK && pJoin != NULL)
			{
			U32				i,cnt;
			CCLTRY ( pJoin->size ( &cnt ) );
			if (hr == S_OK)
				for (i = 0;i < cnt;++i)
					wcscat ( pwQryBfr, L"(" );
			}	// if

		// Table name
		CCLOK		( wcscat ( pwQryBfr, sTableName ); )

		// Optional joins
		if (hr == S_OK && pJoin != NULL)
			{
			IADTDictionary	*pCtxJ	= NULL;
			IADTIt			*pIt		= NULL;
			IADTInIt			*pIn		= NULL;
			adtValueImpl	vJ;
			adtString		sTableJ,sFrom,sTo;
		
			// Iterate join context list
			CCLTRY ( pJoin->iterate ( &pIt ) );
			CCLTRY ( _QI(pIt,IID_IADTInIt,&pIn) );

			// Iterate joins
			while (hr == S_OK && pIn->read ( vJ ) == S_OK)
				{
				// Context
				CCLTRYE ( vJ.vtype == VALT_UNKNOWN && vJ.punk != NULL, E_UNEXPECTED );
				CCLTRY  ( _QI(vJ.punk,IID_IADTDictionary,&pCtxJ) );

				// Check parameters
				CCLTRY(pCtxJ->load ( strRefTableName, sTableJ ));
				CCLTRY(pCtxJ->load ( strRefFrom, sFrom ));
				CCLTRY(pCtxJ->load ( strRefTo, sTo ));

				// Generate the join statement.
				// Format is : JOIN <Join table> ON (<Table>.<From> = <Join table>.<To>)

				// JOIN
				CCLOK ( wcscat ( pwQryBfr, L" INNER JOIN " ); )

				// Join table
				CCLOK ( wcscat ( pwQryBfr, sTableJ ); )

				// ON
				CCLOK ( wcscat ( pwQryBfr, L" ON " ); )

				// <Table>.<From>
				CCLOK ( wcscat ( pwQryBfr, L"[" ); )
				CCLOK ( wcscat ( pwQryBfr, sTableName ); )
				CCLOK ( wcscat ( pwQryBfr, L"].[" ); )
				CCLOK ( wcscat ( pwQryBfr, sFrom ); )
				CCLOK ( wcscat ( pwQryBfr, L"]=[" ); )

				// <Join table>.<To>
				CCLOK ( wcscat ( pwQryBfr, sTableJ ); )
				CCLOK ( wcscat ( pwQryBfr, L"].[" ); )
				CCLOK ( wcscat ( pwQryBfr, sTo ); )
				CCLOK ( wcscat ( pwQryBfr, L"])" ); )

				// Next entry
				pIn->next();

				// Clean up
				_RELEASE(pCtxJ);
				}	// while

			// Clean up
			_RELEASE(pIn);
			_RELEASE(pIt);
			}	// if

		// Optional constraints
		if (hr == S_OK && szCons == 0 && pCons != NULL)
			{
			// Remove previous constraints
			if (pvCons != NULL) { delete[] pvCons; pvCons = NULL; }

			// Constraint size
			CCLTRY(pCons->size ( &szCons ));

			// Allocate new value list for binding
			if (hr == S_OK && szCons > 0)
				hr = ((pvCons = new SQLCol[szCons]) != NULL) ? S_OK : E_OUTOFMEMORY;
			}	// if
		if (hr == S_OK && szCons > 0)
			{
			// WHERE
			CCLOK		( wcscat ( pwQryBfr, L" WHERE " ); )

			// Names, values
			CCLOK		( idx = 0; )
			CCLTRYE	( pConsIn != NULL, E_UNEXPECTED );
			CCLTRY	( pConsIn->begin() );
			while (hr == S_OK && pConsIn->read ( unkV ) == S_OK && idx < szCons)
				{
				// Properties
				CCLTRY( _QISAFE(unkV,IID_IADTDictionary,&pDict) );

				// Name
				CCLTRY( pDict->load ( strRefKey, strV ) );
				CCLOK ( wcscat ( pwQryBfr, L"[" ); )
				CCLOK	( wcscat ( pwQryBfr, strV ); )
				CCLOK ( wcscat ( pwQryBfr, L"] " ); )

				// Constraint operator ( <, >, LIKE, etc. )
				CCLTRY( pDict->load ( strRefOp, strV ) );
				CCLOK	( wcscat ( pwQryBfr, strV ); )
				CCLOK ( bLike = (WCASECMP(strV,L"LIKE") == 0); )

				// Value, bind
				CCLOK ( wcscat ( pwQryBfr, L" ? " ); )
				CCLTRY( pDict->load ( strRefValue, pvCons[idx].sData ) );

				// If the operator is 'like' and the value is a string, add the wildcards
				if (hr == S_OK && bLike && pvCons[idx].sData.vtype == VALT_STR)
					{
					CCLTRY ( strV.allocate ( adtString(pvCons[idx].sData).length() + 2 ) );
					CCLOK  ( wcscpy ( &strV.at(), L"%" ); )
					CCLOK  ( wcscat ( &strV.at(), adtString(pvCons[idx].sData) ); )
					CCLOK  ( wcscat ( &strV.at(), L"%" ); )
					CCLTRY ( adtValueImpl::copy ( pvCons[idx].sData, strV ) );
					}	// if
				CCLTRY( SQLBindVariantParam ( pStmt->Handle, idx+1,
							&(pvCons[idx].sData), &(pvCons[idx].uSz) ) );
				CCLOK ( ++idx; )

				// Next constraint
				if (hr == S_OK && idx < szCons)
					wcscat ( pwQryBfr, L"AND " );
				_RELEASE(pDict);
				pConsIn->next();
				}	// while
			CCLOK ( pwQryBfr[wcslen(pwQryBfr)-1] = WCHAR('\0'); )
			}	// if

		// Sort ?  Prefix with table name in case of join
		if (hr == S_OK && sSort.length() > 0)
			{
			wcscat ( pwQryBfr, L" ORDER BY \"" );
			wcscat ( pwQryBfr, sSort );
			wcscat ( pwQryBfr, L"\" ASC" );
			}	// if

		// Execute.  We should not get SQL_NEED_DATA here because keys are not blobs
		// and other data types are known size
		// Do not error just if the query fails, missing fields, etc.
//		CCLOK ( OutputDebugString ( pwQryBfr ); )
//		CCLOK ( OutputDebugString ( L"\n" ); )
		CCLOK ( SQLSTMT(pStmt->Handle,SQLExecDirect ( pStmt->Handle, pwQryBfr, 
																		(SQLINTEGER)wcslen(pwQryBfr) ));)

		// Result
		if (hr == S_OK)	peFire->emit ( adtIUnknown((phv = pStmt)) );
		else					peFail->emit ( adtInt(hr) );

		// Clean up
		_RELEASE(pStmt);
		}	// if

	// Connection
	else if (prConn == pR)
		{
		adtIUnknown unkV(v);
		adtLong		lTmp;

		// Previous connection
		_RELEASE(pConn);
		hConn = NULL;

		// New connection
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pConn));
		CCLTRY(pConn->getValue ( lTmp ));
		CCLOK (hConn = (SQLHANDLE)(U64)lTmp;)
		}	// else if

	// Constraints
	else if (prCons == pR)
		{
		IADTIt			*pIt		= NULL;
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pCons);
		_RELEASE(pConsIn);
		szCons = 0;

		// New object
		CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCons));
		CCLTRY(pCons->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IADTInIt,&pConsIn));
		_RELEASE(pIt);
		}	// else if

	// Fields
	else if (prFlds == pR)
		{
		IADTContainer	*pCont	= NULL;
		IADTIt			*pIt		= NULL;
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pFldsIn);

		// New object
		CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCont));
		CCLTRY(pCont->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IADTInIt,&pFldsIn));

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pCont);
		}	// else if

	// State
	else if (prCnt == pR)
		iCount = adtInt(v);
	else if (prDis == pR)
		bDistinct = adtBool(v);
	else if (prJoin == pR)
		{
		adtIUnknown	unkV(v);

		// Previous info.
		_RELEASE(pJoin);

		// New object
		CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pJoin));
		}	// else if
	else if (prSort == pR)
		hr = adtValueImpl::copy ( sSort, adtString(v) );
	else if (prTbl == pR)
		hr = adtValueImpl::copy ( sTableName, adtString(v) );

	return hr;
	}	// receive

#endif

#ifdef	USE_OLEDB

#define	SIZE_SQLBFR		1024

// Globals

SQLQuery :: SQLQuery ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn			= NULL;
	bDistinct	= false;
	pConsIn		= NULL;
	pFldsIn		= NULL;
	bCount		= false;
	pCons			= NULL;
	pbCons		= NULL;
	szCons		= 0;
	pQryBfr		= NULL;
	pwQryBfr		= NULL;
	pJoin			= NULL;
	iCount		= 0;

	// Interfaces
	addInterface ( IID_INodeBehaviour, (INodeBehaviour *) this );
	}	// SQLQuery

HRESULT SQLQuery :: construct ( void )
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

	// SQL query string buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pQryBfr ) );
	CCLTRY ( pQryBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pQryBfr->lock ( 0, 0, (PVOID *) &pwQryBfr, NULL ) );

	// Data buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ) );
	CCLTRY ( pBfr->setSize ( 10 ) );

	return hr;
	}	// construct

void SQLQuery :: destruct ( void )
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
	if (pbCons != NULL) delete[] pbCons;
	_RELEASE(pCons);
	_RELEASE(pConsIn);
	_RELEASE(pFldsIn);
	_RELEASE(pJoin);
	_RELEASE(pConn);
	_RELEASE(pBfr);
	_UNLOCK(pQryBfr,pwQryBfr);
	_RELEASE(pQryBfr);
	}	// destruct

HRESULT SQLQuery :: receiveFire ( const adtValue &v )
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
	GUID						guidDialect	= DBGUID_DEFAULT;
	DBPROPSET				set[1];
	DBPROP					prop[1];
	DBPARAMS					dbparams;
	DBROWCOUNT				dbrc;
	adtValueImpl			vVal;
	U32						uBfrPos;

	// State check
	CCLTRYE	( (pConn != NULL),		ERROR_INVALID_STATE );
	if (hr == S_OK && sTableName.length() == 0)
		hr = pnAttr->load ( strRefTableName, sTableName );
	CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
	if (hr == S_OK && sSort.length() == 0)
		 pnAttr->load ( strRefSort, sSort );

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_IDBCreateCommand,
													(IUnknown **) &pCmdCreate ) );
	CCLTRY	( pCmdCreate->CreateCommand ( NULL, IID_ICommandText, 
														(IUnknown **) &pCmdTxt ));
	CCLTRY	( _QI(pCmdTxt,IID_IAccessor,&pAccessor) );
	CCLTRY	( _QI(pCmdTxt,IID_ICommandPrepare,&pCmdPrep) );
	CCLTRY	( _QI(pCmdTxt,IID_IColumnsInfo,&pColInfo) );

	// Count limit ?  Not all providers support this...
	if (hr == S_OK && iCount > (U32)0)
		{
		// Limit query results
		memset ( &(prop[0]), 0, sizeof(DBPROP) );
		prop[0].dwPropertyID		= DBPROP_MAXROWS;
		prop[0].dwOptions			= DBPROPOPTIONS_OPTIONAL;
		prop[0].vValue				= iCount;

		// Initialize property set
		set[0].guidPropertySet	= DBPROPSET_ROWSET;
		set[0].cProperties		= 1;
		set[0].rgProperties		= prop;

		// Set properties for command
		CCLTRY(_QI(pCmdTxt,IID_ICommandProperties,&pCmdProp));
		CCLOK(pCmdProp->SetProperties ( 1, set );)
		_RELEASE(pCmdProp);
		}	// if

	// Time to generate query string.
	// Form is : SELECT (Fields) FROM (Table) WHERE (Constraints)

	// SELECT
	CCLOK		( wcscpy ( pwQryBfr, L"SELECT " ); )

	// Distinct ?
	if (hr == S_OK && bDistinct == true)
		wcscat ( pwQryBfr, L"DISTINCT " );

	// Fields (F1,F2,...,Fn) or All (*)
	if (hr == S_OK && pFldsIn != NULL)
		{
		CCLTRY	( pFldsIn->begin() );
		while (hr == S_OK && pFldsIn->read ( vVal ) == S_OK)
			{
			// Field name
			CCLOK ( wcscat ( pwQryBfr, L"[" ); )
			CCLOK ( wcscat ( pwQryBfr, adtString(vVal) ); )
			CCLOK ( wcscat ( pwQryBfr, L"]," ); )

			// Next
			pFldsIn->next();
			}	// while
		CCLOK ( pwQryBfr[adtString::length(pwQryBfr)-1] = WCHAR('\0'); )
		}	// if
	else if (hr == S_OK && pFldsIn == NULL)
		CCLOK ( wcscat ( pwQryBfr, L"*" ); )

	// FROM
	CCLOK		( wcscat ( pwQryBfr, L" FROM " ); )

	// Table name
	CCLOK		( wcscat ( pwQryBfr, sTableName ); )

	// Optional join
	if (hr == S_OK && pJoin != NULL)
		{
		adtString	sTableJ,sFrom,sTo;

		// Check parameters
		CCLTRY(pJoin->load ( strRefTableName, sTableJ ));
		CCLTRY(pJoin->load ( strRefFrom, sFrom ));
		CCLTRY(pJoin->load ( strRefTo, sTo ));

		// Generate the join statement.
		// Format is : JOIN <Join table> ON (<Table>.<From> = <Join table>.<To>)

		// JOIN
		CCLOK ( wcscat ( pwQryBfr, L" INNER JOIN " ); )

		// Join table
		CCLOK ( wcscat ( pwQryBfr, sTableJ ); )

		// ON
		CCLOK ( wcscat ( pwQryBfr, L" ON " ); )

		// <Table>.<From>
		CCLOK ( wcscat ( pwQryBfr, L"[" ); )
		CCLOK ( wcscat ( pwQryBfr, sTableName ); )
		CCLOK ( wcscat ( pwQryBfr, L"].[" ); )
		CCLOK ( wcscat ( pwQryBfr, sFrom ); )
		CCLOK ( wcscat ( pwQryBfr, L"]=[" ); )

		// <Join table>.<To>
		CCLOK ( wcscat ( pwQryBfr, sTableJ ); )
		CCLOK ( wcscat ( pwQryBfr, L"].[" ); )
		CCLOK ( wcscat ( pwQryBfr, sTo ); )
		CCLOK ( wcscat ( pwQryBfr, L"]" ); )
		}	// if

	// Optional constraints
	CCLOK ( memset ( &dbparams, 0, sizeof(dbparams) ); )
	if (hr == S_OK && szCons == 0 && pCons != NULL)
		{
		// Remove previous constraints
		if (pbCons != NULL) { delete[] pbCons; pbCons = NULL; }

		// Constraint size
		CCLTRY(pCons->size ( &szCons ));

		// Allocate new value list for binding
		CCLTRYE (((pbCons = new DBBINDING[szCons]) != NULL), E_OUTOFMEMORY);
		}	// if
	if (hr == S_OK && szCons > 0)
		hr = OLEDBAppendConstraints ( pCons, pwQryBfr );

	// Sort ?  Prefix with table name in case of join
	if (hr == S_OK && sSort.length() > 0)
		{
		wcscat ( pwQryBfr, L" ORDER BY [" );
		wcscat ( pwQryBfr, sSort );
		wcscat ( pwQryBfr, L"] ASC" );
		}	// if

	// 'Prepare' the command text.  This is required so that the 'ordinal' columns #'s
	// can be obtain so the constraint data can be bound
	CCLOK ( OutputDebugString ( pwQryBfr ); )
	CCLOK ( OutputDebugString ( L"\n" ); )
	CCLTRY ( pCmdTxt->SetCommandText ( guidDialect, pwQryBfr ) );
	CCLTRY ( pCmdPrep->Prepare(0) );

	// Time to bind constraint fields
	if (hr == S_OK && szCons > 0)
		{
		// Copy constraints into buffer
		CCLOK	( uBfrPos = 0; )
		CCLTRY( OLEDBApplyConstraints ( pCons, pBfr, pbCons, &uBfrPos ) );

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

	// Result
	if (hr == S_OK)	peFire->emit ( adtIUnknown(pRowset) );
	else					peFail->emit ( adtInt(hr) );

	// Clean up
	_RELEASE(pRowset);
	if (hAccessor != NULL)
		pAccessor->ReleaseAccessor(hAccessor,NULL);
	_UNLOCK(pBfr,pcBfr);
	_RELEASE(pColInfo);
	_RELEASE(pCmdPrep);
	_RELEASE(pAccessor);
	_RELEASE(pCmdTxt);
	_RELEASE(pCmdCreate);
	_RELEASE(pCreate);

	return hr;
	}	// receiveFire

//
// Context
//

HRESULT SQLQuery :: receiveConnection ( const adtValue &v )
	{
	adtIUnknown	unkV(v);
	_RELEASE(pConn);
	_QISAFE(unkV,IID_IUnknown,&pConn);
	return S_OK;
	}	// receiveConnection

HRESULT SQLQuery :: receiveConstraints ( const adtValue &v )
	{
	HRESULT			hr			= S_OK;
	IADTIt			*pIt		= NULL;
	adtIUnknown		unkV(v);

	// Previous object
	_RELEASE(pCons);
	_RELEASE(pConsIn);
	szCons = 0;

	// New object
	CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCons));
	CCLTRY(pCons->iterate ( &pIt ));
	CCLTRY(_QI(pIt,IID_IADTInIt,&pConsIn));
	_RELEASE(pIt);

	return hr;
	}	// receiveConstraints

HRESULT SQLQuery :: receiveCount ( const adtValue &v )
	{
	iCount = adtInt(v);
	return S_OK;
	}	// receiveCount

HRESULT SQLQuery :: receiveDistinct ( const adtValue &v )
	{
	bDistinct = adtBool(v);
	return S_OK;
	}	// receiveDistinct

HRESULT SQLQuery :: receiveFields ( const adtValue &v )
	{
	HRESULT			hr			= S_OK;
	IADTContainer	*pCont	= NULL;
	IADTIt			*pIt		= NULL;
	adtIUnknown		unkV(v);

	// Previous object
	_RELEASE(pFldsIn);

	// New object
	CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));
	CCLTRY(_QI(pIt,IID_IADTInIt,&pFldsIn));

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pCont);

	return hr;
	}	// receiveFields

HRESULT SQLQuery :: receiveJoin ( const adtValue &v )
	{
	HRESULT		hr = S_OK;
	adtIUnknown	unkV(v);

	// Previous info.
	_RELEASE(pJoin);

	// New object
	CCLTRY(_QISAFE(unkV,IID_IADTDictionary,&pJoin));

	return hr;
	}	// receiveJoin

HRESULT SQLQuery :: receiveSort ( const adtValue &v )
	{
	adtValueImpl::copy ( sSort, adtString(v) );
	return S_OK;
	}	// receiveSort

HRESULT SQLQuery :: receiveTable ( const adtValue &v )
	{
	adtValueImpl::copy ( sTableName, adtString(v) );
	return S_OK;
	}	// receiveTable

#endif

