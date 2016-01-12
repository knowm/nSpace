////////////////////////////////////////////////////////////////////////
//
//									CCL_IMPL.CPP
//
//				An implementation of the COM Compatibility Layer.  Application
//				links to libcclimpl.a.
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
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in 
        the documentation and/or other materials provided with the 
        distribution.
      - Neither the name of nSpace, nor the names of its contributors may 
        be used to endorse or promote products derived from this software
        without specific prior written permission.

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


#include "ccl_impl.h"
#if defined(__unix__) || defined(__APPLE__)
#include <libgen.h>
#include <dlfcn.h>
#endif
#if defined(__APPLE__)
#include <libproc.h>
#endif

// Globals
static char				cDirProc[1024]		= "";
static IDictionary	*pDctLib				= NULL;
static IDictionary	*pDctFct				= NULL;

extern "C"
HRESULT cclCreateObject ( const wchar_t *pId, IUnknown *pOuter, 
									REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Creates the specified object.
	//
	//	PARAMETERS
	//		-	pId is the class Id
	//		-	pOuter is the outer unknown
	//		-	iid is the interface to query for
	//		-	ppv will receive the object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IClassFactory	*pFact	= NULL;

	// Class factory for object
	CCLTRY ( cclGetFactory ( pId, IID_IClassFactory, (void **) &pFact ) );
   
	// Create object from factory
	CCLTRY(pFact->CreateInstance ( pOuter, iid, ppv ) );

	// Clean up
	_RELEASE(pFact);

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"cclCreateObject : Fail : %s : 0x%x\r\n", pId, hr );

	return hr;
	}	// cclCreateObject

extern "C"
HRESULT cclGetFactory ( const wchar_t *pId, REFIID iid, void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Obtain class factory for specified object.
	//
	//	PARAMETERS
	//		-	pId is the class Id
	//		-	iid is the factory interface to query for
	//		-	ppv will receive the factory object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IClassFactory	*pFact	= NULL;
	U32				sz			= 0;
	adtValue			vL;

	// State check
//	dbgprintf ( L"CoGetClassObject : pId %s pDctFct 0x%x\r\n", pId, pDctFct );
	*ppv = NULL;
	CCLTRYE ( pDctFct != NULL, ERROR_INVALID_STATE );
	CCLTRY  ( pDctFct->size ( &sz ) );
//	dbgprintf ( L"cclCreateObject:%s:%d factories cached:0x%x\r\n", pId, sz, hr );
//	dbgprintf ( L"cclCreateObject:%S\r\n", cDirProc );

	// Check cache for pre-existing factory
	if (hr == S_OK && pDctFct->load ( adtString(pId), vL ) == S_OK)
		{
		adtIUnknown	unkV(vL);

		// Factory object
		CCLTRY ( _QISAFE(unkV,IID_IClassFactory,&pFact) );
//		dbgprintf ( L"cclCreateObject:Factory cached:pFact:%p:0x%x\r\n", pFact, hr );
		}	// if

	// New factory required
	else if (hr == S_OK)
		{
		void				*pvLib	= NULL;
		adtString		strLib;
		wchar_t			*wDot;

		// Object Ids are : nSpace.<Library name>.<Object name>
		CCLTRYE ( !WCASENCMP ( pId, L"nSpace.", 7 ), E_UNEXPECTED );

		// Generate full path to library
		CCLOK ( strLib = cDirProc; )
		CCLTRY( strLib.append ( L"lib" ) );
		CCLTRY( strLib.append ( pId+7 ) );

		// Find dot separating module and object name and finish path
		CCLTRYE ( (wDot = wcsrchr ( &strLib.at(), '.' )) != NULL, E_UNEXPECTED );
		CCLOK   ( strLib.at(wDot-(LPCWSTR)strLib) = '\0'; )
		CCLTRY  ( strLib.append ( L"n.so" ) );
		CCLOK   ( strLib.toLower(); )

		// Obtain factory
		CCLTRY ( cclLoadFactoryInt ( strLib, pId, &pvLib, &pFact ) );

		// Cache factory under Id
		CCLTRY ( pDctFct->store ( adtString(pId), adtIUnknown(pFact) ) );
//		dbgprintf ( L"cclCreateObject:New factory cached:pFact:%p:0x%x\r\n", pFact, hr );
		}	// else if

	// Result
	if (hr == S_OK)
		{
		(*ppv) = pFact;
		pFact->AddRef();
		}	// if
	
	// Clean up
	_RELEASE(pFact);

	// Debug
//	dbgprintf ( L"cclGetFactory:%s:0x%x:0x%x\r\n", pId, (int)(pDctFct), *((int *)ppv) );
	if (hr != S_OK)
		dbgprintf ( L"cclGetFactory : Fail : %s : 0x%x\r\n", pId, hr );

	return hr;
	}	// cclGetFactory

HRESULT cclLoadFactoryInt	( const wchar_t *pwLib, const wchar_t *pwId,
										void **ppvLib, IClassFactory **ppFact )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Loads/creates a factory for the specified object from the
	//			specified shared library.
	//
	//	PARAMETERS
	//		-	pwLib is the path to the library
	//		-	pwId is the object Id
	//		-	ppvLib will receive the handle to the loaded library
	//		-	ppFact will receive the factory
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr 															= S_OK;
	HRESULT 		(*cclgco) ( const wchar_t *, REFIID, void ** ) 	= NULL;
	char			*pcLib														= NULL;
	adtLong		iHandle;
	adtString	strErr,sLib(pwLib);

	// Setup
	(*ppvLib) 	= NULL;
	(*ppFact)	= NULL;

	// Load library
	if (hr == S_OK)
		{
		// Already loaded ?
		if (pDctLib == NULL || pDctLib->load ( sLib, iHandle ) != S_OK)
			{
			// Ascii version for API
			CCLTRY ( sLib.toAscii ( &pcLib ) );

			// Load
			CCLOK 	( dbgprintf ( L"cclLoadFactoryInt : Loading '%s'\r\n", (LPCWSTR)sLib ); )
			CCLTRYE 	( ((*ppvLib) = dlopen ( pcLib, RTLD_NOW|RTLD_LOCAL )) != NULL, E_UNEXPECTED );
			if (hr != S_OK)
				{
				strErr = dlerror();
				dbgprintf ( L"cclLoadFactoryInt : Load failed : %s\n", (LPCWSTR) strErr );
				}	// if

			// Cache
			CCLOK  ( iHandle = (U64) (*ppvLib); )
			if (hr == S_OK && pDctLib != NULL)
				hr = pDctLib->store ( sLib, iHandle );

			// Clean up
			_FREEMEM(pcLib);
			}	// if
		else
			(*ppvLib) = (void *)(U64)iHandle;
		}	// if

	// Entry point
	if (hr == S_OK)
		{
		cclgco = (HRESULT (*) (const wchar_t *, REFIID, void **)) dlsym ( (*ppvLib), "cclGetClassObject" );
		hr = (cclgco != NULL) ? S_OK : E_UNEXPECTED;
		if (hr != S_OK)
			{
			strErr = dlerror();
			dbgprintf ( L"cclLoadFactoryInt: Symbol failed : %s\n", (LPCWSTR)strErr );
			}	// if
		}	// if

	// Create the factory
	CCLTRY ( (cclgco) ( pwId, IID_IClassFactory, (void **) ppFact ) );

	// Clean up
	if (hr != S_OK)
		{
		_RELEASE((*ppFact));
		if ((*ppvLib) != NULL) dlclose((*ppvLib));
		}	// if

	return hr;
	}	// cclLoadFactoryInt

extern "C"
HRESULT	CoInitialize ( void *pv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initializes the CCL implementation.
	//
	//	PARAMETERS
	//		-	pv is not used.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr 			= S_OK;

	// Currently it is assume internal shared libraries are in the same
	// directory and the main process.
   if (cDirProc[0] == '\0')
      {
      // Attempt to get the path to the process EXE
		#if defined(__unix__)
      CCLTRYE ( readlink ( "/proc/self/exe", cDirProc, sizeof(cDirProc) ) != -1, errno );
		#elif defined(__APPLE__)
      CCLTRYE ( proc_pidpath ( getpid(), cDirProc, sizeof(cDirProc) )
                  > 0, errno );
		#endif
         
      // Convert to directory
      CCLOK ( strcpy ( cDirProc, dirname ( cDirProc ) ); )
      CCLOK ( strcat ( cDirProc, "/" ); )
      }  // if

	// A dictionary is used to keep track of loaded modules, ensure dictionary
	// (and required library) exists
	if (hr == S_OK && pDctLib == NULL)
		{
		void				*pvLib	= NULL;
		IClassFactory	*pFact	= NULL;
		adtString		strLib;
		adtString		strAdt(L"libadtn.so");
		adtString		strDct(L"nSpace.Adt.Dictionary");

		// Location of required library
		CCLOK ( strLib = cDirProc; )
		CCLTRY( strLib.append ( strAdt ) );

		// Attempt to get class factory for dictionary
		CCLTRY ( cclLoadFactoryInt ( strLib, strDct, &pvLib, &pFact ) );

		// Create a dictionary for the libraries and factories
		CCLTRY ( pFact->CreateInstance ( NULL, IID_IDictionary, (void **) &pDctLib ) );
		CCLTRY ( pFact->CreateInstance ( NULL, IID_IDictionary, (void **) &pDctFct ) );

		// Store own library in dictionary
		CCLTRY ( pDctLib->store ( strAdt, adtLong((U64)pvLib) ) );

		// Store factory in cache
		CCLTRY ( pDctFct->store ( strDct, adtIUnknown(pFact) ) );

		// Clean up
		_RELEASE(pFact);
		}	// if

	// Debug
	dbgprintf ( L"CoInitialize : %S:0x%x\r\n", cDirProc, hr );
		
	return hr;
	}	// CoInitialize

extern "C"
HRESULT	CoInitializeEx ( void *pv, DWORD dwCoInit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initializes the CCL implementation.
	//
	//	PARAMETERS
	//		-	pv is not used.
	//		-	dwCoInit specify concurrency model
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return CoInitialize(pv);
	}	// CoInitializeEx

extern "C"
void CoUninitialize ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Uninitializes the CCL implementation.
	//
	////////////////////////////////////////////////////////////////////////

	// Free loaded libraries
//	_RELEASE(pDctLib);	

	}	// CoUninitialize

extern "C"
HRESULT StringFromCLSID ( REFCLSID clsid, WCHAR **ppwStr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts a CLSID to a string.
	//
	//	PARAMETERS
	//		-	clsid is the class Id to use
	//		-	vStr will receive the string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Allocate memory for string (caller must free)
	CCLTRYE( ((*ppwStr) = (WCHAR *) _ALLOCMEM ( 40*sizeof(wchar_t) ))
					!= NULL, E_OUTOFMEMORY );


	// Format string
	if (hr == S_OK)
		swprintf ( SWPF((*ppwStr),39),
						L"{%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
						clsid.Data1, clsid.Data2, clsid.Data3,
						clsid.Data4[0], clsid.Data4[1], clsid.Data4[2],
						clsid.Data4[3], clsid.Data4[4], clsid.Data4[5],
						clsid.Data4[6], clsid.Data4[7] );

	return S_OK;
	}	// StringFromCLSID
