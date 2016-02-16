////////////////////////////////////////////////////////////////////////
//
//									CONN.CPP
//
//				Implementation of the SQL connection node
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
#include <odbcinst.h>
#endif

// Globals
static WCHAR wCreate[]	= L"CREATE_DB=";
static WCHAR wDriver[]	= L"uid=admin;Driver={";
static WCHAR wDBQ[]		= L"};DBQ=";

SQLCreateDatabase :: SQLCreateDatabase ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// SQLCreateDatabase

#ifdef	USE_ODBC

HRESULT SQLCreateDatabase :: receive ( IReceptor *pR, const adtValue &v )
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
		adtString		sConn;
		BOOL				bRet;	

		// State check
		if (hr == S_OK && sLoc.length() == 0)
			hr = pnAttr->load ( adtString ( L"Location" ), sLoc );
		CCLTRYE ( (sLoc.length() > 0), ERROR_INVALID_STATE );
		if (hr == S_OK && sDriver.length() == 0)
			hr = pnAttr->load ( adtString ( L"Driver" ), sDriver );
		CCLTRYE ( (sDriver.length() > 0), ERROR_INVALID_STATE );

		// If given location is 'relative', obtain environment root from storage object
		if (	hr == S_OK && sLoc.length() &&
				sLoc[0] != WCHAR('\\') &&
				sLoc[1] != WCHAR(':') )			// Drive letter
			{
			IADTDictionary	*pDict	= NULL;
			adtString		sLocRel,sRoot;
			adtIUnknown		unkV;

			// Local copy
			CCLTRY ( adtValueImpl::copy ( sLocRel, sLoc ) );

			// Root location
			CCLTRY ( pnAttr->load ( adtString(L"_Storage"), unkV ) );
			CCLTRY ( _QISAFE(unkV,IID_IADTDictionary,&pDict) );
			CCLTRY ( pDict->load ( adtString(L"Root"), sRoot ) );

			// Generate full path
			CCLTRY ( sLoc.allocate ( sLocRel.length() + sRoot.length() ) );
			CCLOK  ( wcscpy ( &sLoc.at(), sRoot ); )
			CCLOK	 ( wcscat ( &sLoc.at(), sLocRel ); )

			// Clean up
			_RELEASE(pDict);

			// Do not error out of creation on root error
			hr = S_OK;
			}	// if

		// Only create if it does not exist
		if (hr == S_OK && GetFileAttributes ( sLoc ) == MAXDWORD)
			{
			// Allocate/generate configuration string
			CCLTRY ( sConn.allocate (	(U32)wcslen(wCreate) + sLoc.length() + 3));
			CCLOK	( wcscpy ( &sConn.at(), wCreate ); )
			CCLOK ( wcscat ( &sConn.at(), L"\"" ); )
			CCLOK ( wcscat ( &sConn.at(), sLoc ); )
			CCLOK ( wcscat ( &sConn.at(), L"\"" ); )

			// API wants double termination
			CCLOK ( sConn.at(sConn.length()+1) = WCHAR('\0'); )

			// Create source, watch for existing file error
			CCLOK ( bRet = SQLConfigDataSource ( NULL, ODBC_ADD_DSN, sDriver, sConn ); )

			// Error ?
			if (hr == S_OK && bRet == false)
				SQLInstallerError ( 1, (DWORD *) &hr, NULL, 0, NULL );
			}	// if

		// Generate connection string for created source
		CCLTRY ( sConn.allocate ( sDriver.length() +	// Driver name
					wcslen(wDriver) +							// Driver specification
					wcslen(wDBQ) +								// DBQ
					sLoc.length() +							// Length
					2 ) );										// +2 for quotes around location
		CCLOK ( wcscpy ( &sConn.at(), wDriver ); )
		CCLOK ( wcscat ( &sConn.at(), sDriver ); )
		CCLOK ( wcscat ( &sConn.at(), wDBQ ); )
		CCLOK ( wcscat ( &sConn.at(), sLoc ); )

		// Result
		if (hr == S_OK)	peFire->emit ( sConn );
		else					peErr->emit ( adtInt(hr) );
		}	// if

	// State
	else if (prLoc == pR)
		sLoc = (LPCWSTR)adtString(v);

	return hr;
	}	// receive

#endif

#ifdef	USE_OLEDB
HRESULT SQLCreateDatabase :: receiveFire ( const adtValue &v )
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
	HRESULT					hr			= S_OK;
	IDBDataSourceAdmin	*pAdmin	= NULL;
	DBPROPSET				set[1];
	DBPROP					prop[1];
	CLSID						clsid;

	// State check
	if (hr == S_OK && sLoc.length() == 0)
		hr = pnAttr->load ( adtString ( L"Location" ), sLoc );
	CCLTRYE ( (sLoc.length() > 0), ERROR_INVALID_STATE );

	// For now default to Microsoft Access/Jet engine for default
	CCLTRY(CLSIDFromProgID ( L"Microsoft.Jet.OLEDB.4.0", &clsid ) );
	CCLTRY(COCREATEINSTANCE(clsid, IID_IDBDataSourceAdmin, &pAdmin ));

	// Only create if it does not exist
	if (hr == S_OK && GetFileAttributes ( sLoc ) == MAXDWORD)
		{
		// Location of data source
		VariantInit ( &(prop[0].vValue) );
		prop[0].dwPropertyID	= DBPROP_INIT_DATASOURCE;
		prop[0].dwOptions		= DBPROPOPTIONS_REQUIRED;
		prop[0].vValue			= sLoc;

		// Property set
		set[0].guidPropertySet	= DBPROPSET_DBINIT;
		set[0].cProperties		= 1;
		set[0].rgProperties		= prop;

		// Create data source
		hr = pAdmin->CreateDataSource ( 1, set, NULL, IID_NULL, NULL );
		}	// if

	// Result
	if (hr == S_OK)	peFire->emit ( sLoc );
	else					peErr->emit ( adtInt(hr) );

	// Clean up
	_RELEASE(pAdmin);

	return hr;
	}	// receiveFire
#endif
