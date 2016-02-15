////////////////////////////////////////////////////////////////////////
//
//									VARIANT.CPP
//
//			Implementation of the WIN32 VARIANT helper class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"

// For persistence...
#include "../iol/iol.h"

adtVariant :: adtVariant ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	VariantInit ( this );
	punkPrsrL	= NULL;
	punkPrsrS	= NULL;
	punkStmL		= NULL;
	punkStmS		= NULL;
	}	// adtVariant

adtVariant :: ~adtVariant ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(punkPrsrL);
	_RELEASE(punkPrsrS);
	_RELEASE(punkStmL);
	_RELEASE(punkStmS);
	VariantClear(this);
	}	// ~adtVariant

HRESULT adtVariant :: clear ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Clear the contents of the variant.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return VariantClear(this);
	}	// clear

HRESULT adtVariant :: toValue ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert the contained variant into a local value.
	//
	//	PARAMETERS
	//		-	v will receive the converted value
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Target value
	adtValue::clear(v);

	// Map internal types to variant types
	switch(vt)
		{
		case VT_BOOL:
			v.vtype = VTYPE_BOOL;
			v.vbool = (boolVal == VARIANT_TRUE) ? true : false;
			break;
		case VT_R4:
			v.vtype	= VTYPE_R4;
			v.vflt	= fltVal;
			break;
		case VT_R8:
			v.vtype = VTYPE_R8;
			v.vdbl = dblVal;
			break;
		case VT_I4:
			v.vtype = VTYPE_I4;
			v.vint = intVal;
			break;
		case VT_I8:
			v.vtype = VTYPE_I8;
			v.vlong = llVal;
			break;
		case VT_DATE:
			v.vtype = VTYPE_DATE;
			v.vdate = date;
			break;
		case VT_BSTR :
			hr = adtValue::copy(adtString(bstrVal),v);
			break;

		// Objects are persisted to a safe array
		case (VT_ARRAY|VT_I1) :
			{
			HRESULT			hr			= S_OK;
			IStreamPersist	*pPrsr	= NULL;
			IByteStream		*pStm		= NULL;
			VOID				*pvData	= NULL;

			// Create cached versions of parser and stream
			if (hr == S_OK && punkPrsrL == NULL)
				hr = COCREATE(L"Io.StmPrsBin",IID_IUnknown,&punkPrsrL);
			if (hr == S_OK && punkStmL == NULL)
				hr = COCREATE(L"Io.StmMemory",IID_IUnknown,&punkStmL);

			// Access interfaces
			CCLTRY(_QISAFE(punkPrsrL,IID_IStreamPersist,&pPrsr));
			CCLTRY(_QISAFE(punkStmL,IID_IByteStream,&pStm));

			// Access buffer for direct copy of data
			CCLTRY ( SafeArrayAccessData ( parray, &pvData ) );
			CCLTRY ( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );
			CCLTRY ( pStm->write ( pvData, parray->rgsabound[0].cElements, NULL ) );

			// Persist object from stream
			CCLTRY(pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );
			CCLTRY(pPrsr->load(pStm,v));

			// Clean up
			if (pvData != NULL)
				SafeArrayUnaccessData ( parray );
			_RELEASE(pStm);
			_RELEASE(pPrsr);
			}	// VTYPE_ARRAY|VTI1
			break;
		case VTYPE_EMPTY :
			// Empty values are fine
			break;
		default :
			dbgprintf(L"adtVariant::toValue:Unhandled type:0x%x\r\n", vt );
			hr = E_NOTIMPL;
		}	// switch

	return hr;
	}	// toValue

//
// Constructors
//

adtVariant::adtVariant( const VARIANT *pv )
	{
	adtVariant::adtVariant();
	*this = pv;
	}	// adtVariant

adtVariant :: adtVariant ( IUnknown *punk )
	{
	adtVariant::adtVariant();
	vt			= VT_UNKNOWN;
	punkVal	= punk;
	if (punkVal) punkVal->AddRef();
	}	// adtVariant

adtVariant :: adtVariant ( int val )
	{
	adtVariant::adtVariant();
	vt		= VT_I4;
	lVal	= val;
	}	// adtVariant

adtVariant :: adtVariant ( long val )
	{
	adtVariant::adtVariant();
	vt		= VT_I8;
	llVal	= val;
	}	// adtVariant

adtVariant :: adtVariant ( WCHAR *val )
	{
	adtVariant::adtVariant();
	vt			= VT_BSTR;
	bstrVal	= SysAllocString ( val );
	}	// adtVariant

adtVariant :: adtVariant ( const WCHAR *val )
	{
	adtVariant::adtVariant();
	vt			= VT_BSTR;
	bstrVal	= SysAllocString ( val );
	}	// adtVariant

adtVariant :: adtVariant ( bool val )
	{
	adtVariant::adtVariant();
	vt			= VT_BOOL;
	boolVal	= (val == true) ? VARIANT_TRUE : VARIANT_FALSE;
	}	// adtVariant

adtVariant :: adtVariant ( const ADTVALUE &val )
	{
	adtVariant::adtVariant();

	// Use operator
	*this = val;
	}	// adtVariant

//
// Operators
//

adtVariant& adtVariant::operator= ( const WCHAR *val )
	{
	VariantClear(this);
	vt			= VT_BSTR;
	bstrVal	= SysAllocString(val);
	return *this;
	}	// operator=

adtVariant& adtVariant::operator= ( const VARIANT *pv )
	{
	clear();
	VariantCopy(this,pv);
	return *this;
	}	// operator=

adtVariant& adtVariant::operator= ( const ADTVALUE &v )
	{
	VariantClear(this);

	// Map internal types to variant types
	switch (v.vtype)
		{
		case VTYPE_BOOL :
			vt			= VT_BOOL;
			boolVal	= (v.vbool == TRUE) ? VARIANT_TRUE : VARIANT_FALSE;
			break;
		case VTYPE_DATE :
			vt			= VT_DATE;
			date		= v.vdate;
			break;
		case VTYPE_R4 :
			vt			= VT_R4;
			fltVal	= v.vflt;
			break;
		case VTYPE_R8 :
			vt			= VT_R8;
			dblVal	= v.vdbl;
			break;
		case VTYPE_I4 :
			vt			= VT_I4;
			lVal		= v.vint;
			break;
		case VTYPE_I8 :
			vt			= VT_I8;
			dblVal	= v.vdbl;
			break;
		case VTYPE_STR						:
		case (VTYPE_STR|VTYPE_BYREF)	:
		case (VTYPE_STR|VTYPE_CONST)	:
			vt			= VT_BSTR;
			bstrVal	= SysAllocString ( v.pstr );
			break;
		case VTYPE_UNK :
			{
			HRESULT			hr			= S_OK;
			IStreamPersist	*pPrsr	= NULL;
			IByteStream		*pStm		= NULL;
			VOID				*pvData	= NULL;
			SAFEARRAYBOUND	sab[1];
			U64				iAv;

			// Objects are persisted to byte stream and stored in
			// a SAFEARRAY.
			vt = VT_ARRAY | VT_I1;

			// Create cached versions of parser and stream
			if (hr == S_OK && punkPrsrS == NULL)
				hr = COCREATE(L"Io.StmPrsBin",IID_IUnknown,&punkPrsrS);
			if (hr == S_OK && punkStmS == NULL)
				hr = COCREATE(L"Io.StmMemory",IID_IUnknown,&punkStmS);

			// Access interfaces
			CCLTRY(_QISAFE(punkPrsrS,IID_IStreamPersist,&pPrsr));
			CCLTRY(_QISAFE(punkStmS,IID_IByteStream,&pStm));

			// Persist object to stream
			CCLTRY(pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );
			CCLTRY(pPrsr->save(pStm,v));

			// Transfer memory stream contents into a safe array
			CCLTRY(pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );
			CCLTRY(pStm->available(&iAv));

			// Allocate safe array
			if (hr == S_OK)
				{
				sab[0].lLbound		= 0;
				sab[0].cElements	= (ULONG)iAv;
				CCLTRYE ( (parray = SafeArrayCreate ( VT_I1, 1, sab )) != NULL,
								E_OUTOFMEMORY );
				}	// if

			// Access buffer for direct copy of data
			CCLTRY ( SafeArrayAccessData ( parray, &pvData ) );
			CCLTRY ( pStm->read ( pvData, iAv, NULL ) );

			// Clean up
			if (pvData != NULL)
				SafeArrayUnaccessData ( parray );
			_RELEASE(pStm);
			_RELEASE(pPrsr);
			if (hr != S_OK)
				{
				// Free array
				if (parray != NULL)
					SafeArrayDestroy(parray);
				parray	= NULL;
				vt			= 0;
				}	// if

			}	// VTYPE_UNK
			break;
		case VTYPE_EMPTY :
			// Empty values are fine
			vt = VT_EMPTY;
			break;
		default :
			dbgprintf ( L"adtVariant::operator =:Unhandled value type %d\r\n", v.vtype );
		}	// switch

	return *this;
	}	// operator=

/*
			// Assume array of values of the same type to support safe arrays
			CCLTRY ( _QISAFE(v.punk,IID_IContainer,&pCnt) );
			CCLTRYE( _QI(v.punk,IID_IDictionary,&pDct) != S_OK, S_FALSE );

			// Obtain count and iterate values
			CCLTRY ( pCnt->size ( &sz ) );
			CCLTRY ( pCnt->iterate ( &pIt ) );

			// Get type of first value to safe array can be allocted
			CCLTRY ( pIt->begin() );
			if (hr == S_OK && pIt->read ( vL ) == S_OK)
				{
				// Convert to variant to get type
				varL = vL;

				// Default type
				vta = varL.vt;
				}	// if

			// Allocate array to receive all values
			CCLOK ( sab[0].lLbound		= 0; )
			CCLOK ( sab[0].cElements	= sz; )
			CCLTRYE( (psa = SafeArrayCreate ( vta, 1, sab )) != NULL,
							E_OUTOFMEMORY );

			// Populate list
			CCLTRY ( pIt->begin() );
			for (i = 0;hr == S_OK && i < sz;++i)
				{
				long		idx[1] = { i };
				VARIANT	*pv;

				// Next value
				CCLTRY ( pIt->read(vL) );
				CCLTRY ( pIt->next() );
				CCLOK  ( varL = vL; )

				// Value type match ?
				CCLTRYE ( varL.vt == vta, E_UNEXPECTED );

				// Add to array
				CCLTRY ( SafeArrayPutElement ( psa, idx, 
								(varL.vt == VT_BSTR) ? (void *)varL.bstrVal : (pv = &varL) ) );
				}	// for

			// Use as final value
			if (hr == S_OK)
				{
				vt			= vta | VT_ARRAY;
				parray	= psa;
				}	// if

			// On failure default to just the object
			else
				{
				vt			= VT_UNKNOWN;
				punkVal	= v.punk;
				if (punkVal) punkVal->AddRef();
				}	// if

			// Clean up
			if (hr != S_OK && psa != NULL)
				{
				SafeArrayDestroyData(psa);
				SafeArrayDestroy(psa);
				}	// if
			_RELEASE(pIt);
			_RELEASE(pDct);
			_RELEASE(pCnt);
*/