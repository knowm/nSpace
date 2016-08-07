////////////////////////////////////////////////////////////////////////
//
//									DELETE.CPP
//
//					Implementation of the SQL delete node
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

#define	SIZE_SQLBFR		1024

#ifdef	USE_ODBC

// Globals

SQLDelete :: SQLDelete ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn			= NULL;
	hConn			= NULL;
	pConsIn		= NULL;
	pvCons		= NULL;
	pCons			= NULL;
	szCons		= 0;
	pSQLBfr		= NULL;
	pwSQLBfr		= NULL;
	}	// SQLDelete

HRESULT SQLDelete :: construct ( void )
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

	// SQL string buffer
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pSQLBfr ) );
	CCLTRY ( pSQLBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pSQLBfr->lock ( 0, 0, (PVOID *) &pwSQLBfr, NULL ) );

	return hr;
	}	// construct

void SQLDelete :: destruct ( void )
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
	_RELEASE(pConn);
	_UNLOCK(pSQLBfr,pwSQLBfr);
	_RELEASE(pSQLBfr);
	}	// destruct

HRESULT SQLDelete :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
	if (_RCP(Fire))
		{
		IDictionary	*pDict	= NULL;
		SQLHandle	*pStmt	= NULL;
		IHaveValue	*phv;
		adtValue		vVal;
		adtIUnknown	unkV;
		adtString	strV;
		U32			idx;
		bool			bLike;

		// State check
		CCLTRYE ( (hConn != NULL),		ERROR_INVALID_STATE );
		CCLTRYE ( (pCons != NULL),		ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnDesc->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Time to generate SQL string.
		// Form is : DELETE FROM (Table) WHERE (Constraints)

		// DELETE
		CCLOK		( WCSCPY ( pwSQLBfr, SIZE_SQLBFR, L"DELETE FROM " ); )

		// Table name
		CCLOK		( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, sTableName ); )

		// Required constraints
		if (hr == S_OK && szCons == 0)
			{
			// Constraint size
			CCLTRY(pCons->size ( &szCons ));
			CCLTRYE(szCons > 0,E_UNEXPECTED);

			// Allocate new value list for binding
			if (pvCons != NULL) { delete[] pvCons; pvCons = NULL; }
			CCLTRYE ( (pvCons = new SQLCol[szCons]) != NULL, E_OUTOFMEMORY );
			}	// if
		CCLTRYE ( (szCons > 0), ERROR_INVALID_STATE );

		// WHERE
		CCLOK		( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, L" WHERE " ); )

		// Names, values
		CCLOK		( idx = 0; )
		CCLTRYE	( pConsIn != NULL, E_UNEXPECTED );
		CCLTRY	( pConsIn->begin() );
		while (hr == S_OK && pConsIn->read ( unkV ) == S_OK)
			{
			// Properties
			CCLTRY( _QISAFE(unkV,IID_IDictionary,&pDict) );

			// Name
			CCLTRY( pDict->load ( strRefKey, strV ) );
			CCLOK ( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, L"\"" ); )
			CCLOK	( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, strV ); )
			CCLOK ( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, L"\" " ); )

			// Constraint operator ( <, >, LIKE, etc. )
			CCLTRY( pDict->load ( strRefOp, strV ) );
			CCLOK	( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, strV ); )
			CCLOK ( bLike = (WCASECMP(strV,L"LIKE") == 0); )

			// Value, bind
			CCLOK ( WCSCAT ( pwSQLBfr, SIZE_SQLBFR, L" ? " ); )
			CCLTRY( pDict->load ( strRefValue, pvCons[idx].sData ) );

			// If the operator is 'like' and the value is a string, add the wildcards
			if (hr == S_OK && bLike && adtValue::type(pvCons[idx].sData) == VTYPE_STR)
				{
				adtString	sData(pvCons[idx].sData);
				CCLTRY ( strV.allocate ( sData.length() + 2 ) );
				CCLOK  ( strV = L"%"; )
				CCLTRY ( strV.append ( sData ) );
				CCLTRY ( strV.append ( L"%" ) );
				CCLTRY ( adtValue::copy ( strV, pvCons[idx].sData ) );
				}	// if
			CCLTRY( SQLBindVariantParam ( pStmt->Handle, idx+1,
						&(pvCons[idx].sData), &(pvCons[idx].uSz) ) );
			CCLOK ( ++idx; )

			// Next constraint
			if (hr == S_OK && idx < szCons)
				WCSCAT ( pwSQLBfr, SIZE_SQLBFR, L"AND " );
			_RELEASE(pDict);
			pConsIn->next();
			}	// while
		CCLOK ( pwSQLBfr[wcslen(pwSQLBfr)-1] = WCHAR('\0'); )

		// Execute.  
		CCLOK ( OutputDebugString ( pwSQLBfr ); )
		CCLOK ( OutputDebugString ( L"\n" ); )
		CCLTRY( SQLSTMT(pStmt->Handle,SQLExecDirect ( pStmt->Handle, pwSQLBfr, 
																		(SQLINTEGER)wcslen(pwSQLBfr) )));

		// Result
		CCLOK ( _EMT(Fire,adtIUnknown((phv = pStmt)) ); )

		// Clean up
		_RELEASE(pStmt);
		}	// if

	// Connection
	else if (_RCP(Connection))
		{
		HRESULT		hr = S_OK;
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
	else if (_RCP(Constraints))
		{
		HRESULT		hr			= S_OK;
		IIt			*pIt		= NULL;
		adtIUnknown	unkV(v);

		// Previous object
		_RELEASE(pCons);
		_RELEASE(pConsIn);
		szCons = 0;

		// New object
		CCLTRY(_QISAFE(unkV,IID_IContainer,&pCons));
		CCLTRY(pCons->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IIt,&pConsIn));
		_RELEASE(pIt);
		}	// else if

	// State
	else if (_RCP(Table))
		hr = adtValue::copy ( adtString(v), sTableName );

	return hr;
	}	// receive

#endif

#ifdef	USE_OLEDB

// Globals

SQLDelete :: SQLDelete ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn			= NULL;
	pCons			= NULL;
	pConsIn		= NULL;
	pbCons		= NULL;
	szCons		= 0;
	pSQLBfr		= NULL;
	pwSQLBfr		= NULL;

	// Interfaces
	addInterface ( IID_INodeBehaviour, (INodeBehaviour *) this );
	}	// SQLDelete

HRESULT SQLDelete :: construct ( void )
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

	// SQL string buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pSQLBfr ) );
	CCLTRY ( pSQLBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pSQLBfr->lock ( 0, 0, (PVOID *) &pwSQLBfr, NULL ) );

	// Data buffer
	CCLTRY ( COCREATEINSTANCE ( CLSID_MemoryBlock, IID_IMemoryMapped, &pBfr ) );
	CCLTRY ( pBfr->setSize ( 10 ) );

	return hr;
	}	// construct

void SQLDelete :: destruct ( void )
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
	_RELEASE(pConn);
	_RELEASE(pBfr);
	_UNLOCK(pSQLBfr,pwSQLBfr);
	_RELEASE(pSQLBfr);
	}	// destruct

HRESULT SQLDelete :: receiveFire ( const adtValue &v )
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
	IAccessor				*pAccessor	= NULL;
	IDictionary			*pDict		= NULL;
	IColumnsInfo			*pColInfo	= NULL;
	IRowset					*pRowset		= NULL;
	U8							*pcBfr		= NULL;
	HACCESSOR				hAccessor	= NULL;
	GUID						guidDialect	= DBGUID_DEFAULT;
	DBPARAMS					dbparams;
	DBROWCOUNT				dbrc;
	adtValueImpl			vVal;
	U32						uBfrPos;

	// State check
	CCLTRYE ( (pConn != NULL),		ERROR_INVALID_STATE );
	if (hr == S_OK && sTableName.length() == 0)
		hr = pnAttr->load ( strRefTableName, sTableName );
	CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_IDBCreateCommand,
													(IUnknown **) &pCmdCreate ) );
	CCLTRY	( pCmdCreate->CreateCommand ( NULL, IID_ICommandText, 
														(IUnknown **) &pCmdTxt ));
	CCLTRY	( _QI(pCmdTxt,IID_IAccessor,&pAccessor) );
	CCLTRY	( _QI(pCmdTxt,IID_ICommandPrepare,&pCmdPrep) );
	CCLTRY	( _QI(pCmdTxt,IID_IColumnsInfo,&pColInfo) );

	// Time to generate SQL string.
	// Form is : DELETE FROM (Table) WHERE (Constraints)

	// DELETE
	CCLOK		( wcscpy ( pwSQLBfr, L"DELETE FROM " ); )

	// Table name
	CCLOK		( wcscat ( pwSQLBfr, sTableName ); )

	// Required constraints
	if (hr == S_OK && szCons == 0)
		{
		// Constraint size
		CCLTRY(pCons->size ( &szCons ));
		CCLTRYE(szCons > 0,E_UNEXPECTED);

		// Allocate new value list for binding
		if (pbCons != NULL) { delete[] pbCons; pbCons = NULL; }
		CCLTRYE (((pbCons = new DBBINDING[szCons]) != NULL), E_OUTOFMEMORY);
		}	// if
	CCLTRYE ( (szCons > 0), ERROR_INVALID_STATE );

	// Generate 'where' clause from constraints
	if (hr == S_OK && szCons > 0)
		hr = OLEDBAppendConstraints ( pCons, pwSQLBfr );

	// 'Prepare' the command text.  This is required so that the 'ordinal' columns #'s
	// can be obtain so the constraint data can be bound
	CCLOK ( OutputDebugString ( pwSQLBfr ); )
	CCLOK ( OutputDebugString ( L"\n" ); )
	CCLTRY ( pCmdTxt->SetCommandText ( guidDialect, pwSQLBfr ) );
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

HRESULT SQLDelete :: receiveConnection ( const adtValue &v )
	{
	adtIUnknown	unkV(v);
	_RELEASE(pConn);
	_QISAFE(unkV,IID_IUnknown,&pConn);
	return S_OK;
	}	// receiveConnection

#endif

