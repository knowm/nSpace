////////////////////////////////////////////////////////////////////////
//
//									STMPRBIN.CPP
//
//							Binary value stream parser
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

// Markers
#define	MARKER_BEGIN		0x3141
#define	MARKER_END			0x5926

// Endian macros

#define	SWAPS(a)										\
				(											\
				(((a) << 8)	& 0xff00) |				\
				(((a) >> 8) & 0x00ff)				\
				)
#define	SWAPI(a)													\
				(														\
				(SWAPS(((a) >>  0) & 0x0000ffff) << 16) |	\
				(SWAPS(((a) >> 16) & 0x0000ffff) <<  0)	\
				)
#define	SWAPL(a)													\
				(														\
				(SWAPI(((a) >>  0) & 0xffffffff) << 32) |	\
				(SWAPI(((a) >> 32) & 0xffffffff) <<  0)	\
				)

StmPrsBin :: StmPrsBin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// StmPrsBin

HRESULT StmPrsBin :: load ( IByteStream *pStm, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IStreamPersist
	//
	//	PURPOSE
	//		-	Load a value from the stream.
	//
	//	PARAMETERS
	//		-	pStm is the source stream
	//		-	v will receive the loaded value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U16		vt;

	// Setup
	adtValue::clear(v);

	// Initialize marker
	CCLTRY	( read ( pStm, &vt ) );

	// Marker should be a 'begin'
	CCLTRYE ( (vt == MARKER_BEGIN), E_UNEXPECTED );

	// Value type
	CCLTRY	( read ( pStm, &vt ) );

	// Process value
	switch (vt)
		{
		// Strings
		case VTYPE_STR :
			{
			adtString	vStr;

			// Read and store string
			CCLTRY ( readStr ( pStm, vStr ) );
			CCLTRY ( adtValue::copy ( vStr, v ) );
			}
			break;

		// Object
		case VTYPE_UNK :
			{
			IDictionary		*pDct		= NULL;
			IList				*pLst		= NULL;
			IUnknown			*pUnk		= NULL;
			IByteStream		*pStmV	= NULL;
			IMemoryMapped	*pMap		= NULL;
			adtIUnknown		vUnk;
			adtValue			vK,vV;
			adtString		strId;
			U32				i,sz     = 0;

			// Object ID
			CCLTRY ( readStr ( pStm, strId ) );

			// Create object.
			if (hr == S_OK && strId.length() > 0)
				{
				CCLTRY ( strId.prepend ( L"nSpace." ) );
				CCLTRY ( cclCreateObject ( strId, NULL, IID_IUnknown, (void **) &pUnk ) );
				}	// if

			// Null object ok
			if (hr == S_OK && pUnk == NULL)
				{
				// NULL object
				vUnk	= (IUnknown *)NULL;
				}	// if

			// List - Check 'List' first since it also support the dictionary interface
			else if (hr == S_OK && _QI(pUnk,IID_IList,&pLst) == S_OK)
				{
				// Read count of key/value pairs
				CCLTRY ( read ( pStm, &sz ) );

				// Read values until complete
				for (i = 0;hr == S_OK && i < sz;++i)
					{
					// Value
					CCLTRY ( load ( pStm, vV ) );

					// Store value
					CCLTRY ( pLst->write ( vV ) );
					}	// for
				}	// if

			// Dictionary
			else if (hr == S_OK && _QI(pUnk,IID_IDictionary,&pDct) == S_OK)
				{
				// Read count of key/value pairs
				CCLTRY ( read ( pStm, &sz ) );

				// Read key/value pairs until complete
				for (i = 0;hr == S_OK && i < sz;++i)
					{
					// Key/value
					CCLTRY ( load ( pStm, vK ) );
					CCLTRY ( load ( pStm, vV ) );

					// Store pair
					CCLTRY ( pDct->store ( vK, vV ) );
					}	// for

				}	// if

			// Byte stream ?
			else if (hr == S_OK && _QI(pUnk,IID_IByteStream,&pStmV) == S_OK)
				{
				// Size of byte stream
				CCLTRY ( read ( pStm, &sz ) );
				CCLTRY ( pStmV->setSize ( sz ) );

				// Stream in bytes
				if (hr == S_OK && sz > 0)
					hr = pStm->copyTo ( pStmV, sz, NULL );

				// Reset position of stream
				CCLTRY ( pStmV->seek ( 0, STREAM_SEEK_SET, NULL ) );

				// Clean up
				_RELEASE(pStmV);
				}	// else if

			// Memory block
			else if (hr == S_OK && _QI(pUnk,IID_IMemoryMapped,&pMap) == S_OK)
				{
				void	*pv	= NULL;

				// Size of region
				CCLTRY ( read ( pStm, &sz ) );
				CCLTRY ( pMap->setSize ( sz ) );

				// Access the region
				CCLTRY ( pMap->lock ( 0, 0, &pv, NULL ) );

				// Read bytes
//				dbgprintf ( L"-> StmPrsBin::load:Memory Block:0x%x:%d bytes\r\n", hr, sz );
				CCLTRY ( read ( pStm, pv, sz ) );
//				dbgprintf ( L"<- StmPrsBin::load:Memory Block:0x%x:%d bytes\r\n", hr, sz );

				// Clean up
				if (pv != NULL)
					pMap->unlock(pv);
				}	// else if

			// Unhandled object
			else if (hr == S_OK)
				{
				hr = E_NOTIMPL;
				dbgprintf ( L"StmPrsBin::load:Object not implemented\r\n" );
				}	// else

			// Result
			CCLTRY ( adtValue::copy ( adtIUnknown(pUnk), v ) );

			// Clean up
			_RELEASE(pDct);
			_RELEASE(pLst);
			_RELEASE(pUnk);
			}	// TYPE_UNKNOWN
			break;

		// Simple types

		// Integer
		case VTYPE_I4 :
			CCLTRY ( read ( pStm, (U32 *) &(v.vint) ) );
			break;
		// Long
		case VTYPE_I8 :
			CCLTRY ( read ( pStm, (U64 *) &(v.vlong) ) );
			break;
		// Float
		case VTYPE_R4 :
			CCLTRY ( read ( pStm, (U32 *) &(v.vflt) ) );
			break;
		// Double
		case VTYPE_R8 :
			CCLTRY ( read ( pStm, (U64 *) &(v.vdbl) ) );
			break;
		// Date
		case VTYPE_DATE :
			CCLTRY ( read ( pStm, (U64 *) &(v.vdate) ) );
			break;
		// Boolean
		case VTYPE_BOOL :
			CCLTRY ( read ( pStm, &(v.vbool) ) );
			break;
		// Empty
		case VTYPE_EMPTY :
			// Empty values ok
			break;
		default :
			dbgprintf ( L"StmPrsBin::load:Unhandled value type 0x%x\r\n", v.vtype );
			break;
		}	// switch

	// Set type if successful
	CCLOK ( v.vtype = vt; )

	// Termination marker for value
	CCLTRY ( read ( pStm, &vt ) );
	CCLTRYE ( (vt == MARKER_END), E_UNEXPECTED );

	return hr;
	}	// load

HRESULT StmPrsBin :: read ( IByteStream *pStm, U16 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Read a 2 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr will receive the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U16		v;

	// Read
	hr = pStm->read ( &v, sizeof(v), NULL );

	// To big endian
	*pBfr = SWAPS(v);

	return hr;
	}	// read

HRESULT StmPrsBin :: read ( IByteStream *pStm, U32 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Read a 4 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr will receive the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U32		v;

	// Read
	hr = pStm->read ( &v, sizeof(v), NULL );

	// To big endian
	*pBfr = SWAPI(v);

	return hr;
	}	// read

HRESULT StmPrsBin :: read ( IByteStream *pStm, U64 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Read a 8 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr will receive the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U64		v;

	// Read
	hr = pStm->read ( &v, sizeof(v), NULL );

	// To big endian
	*pBfr = SWAPL(v);

	return hr;
	}	// read

HRESULT StmPrsBin :: read ( IByteStream *pStm, void *pvBfr, U32 sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Reads from stream until error or complete.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pvBfr will receive the data.
	//		-	sz is the size of the transfer.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U8			*pcBfr	= (U8 *) pvBfr;
	U64		nleft		= sz;
	U64		nx;

	// Continue until error or done
	while (hr == S_OK && nleft)
		{
		CCLTRY( pStm->read ( pcBfr, nleft, &nx ) );
		CCLOK	( nleft -= nx; )
		CCLOK	( pcBfr += nx; )
		}	// while

	return hr;
	}	// read

HRESULT StmPrsBin :: readStr ( IByteStream *pStm, adtString &str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Read string from stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	str will receive the string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U32		len;
	U16		wsz;
			
	// Length of string
	CCLTRY ( read ( pStm, &len ) );

	// Allocate for string
	CCLTRY ( str.allocate ( len ) );

	// Size of local 'wchar_t' when written
	CCLTRY ( read ( pStm, &wsz ) );

	// Different environments have different size unicode characters.
	// Example: Windows sizeof(wchar_t) = 2, OSX sizeof(wchar_t) = 4
	if (hr == S_OK)
		{
		// Same size, read directly
		if (wsz == sizeof(wchar_t))
			hr = read ( pStm, &str.at(), len*wsz );

		// 1 byte per character
		else if (wsz == 1)
			{
			char	*pc	= NULL;

			// Allocate byte array for string
			CCLTRYE ( (pc = (char *) _ALLOCMEM((len+1)*sizeof(char))) != NULL,
							E_OUTOFMEMORY );
			CCLOK   ( pc[len] = '\0'; )

			// Read string
			CCLTRY ( read ( pStm, pc, len*sizeof(char) ) );

			// Convert to local format
			CCLOK ( str = pc; )

			// Clean up
			_FREEMEM(pc);
			}	// else if

		// Remote 32 bits, Local 16 bits
		else if (wsz == 4 && sizeof(WCHAR) == 2)
			{
			U32	uc;

			// Read all characters
			for (U32 i = 0;hr == S_OK && i < len;++i)
				{
				// Read 4 byte char.
				CCLTRY ( read ( pStm, &uc, sizeof(uc) ) );

				// Store two bytes
				CCLOK  ( str.at(i) = (U16) (uc & 0xffff); )
				}	// for

			}	// else if

		// Remote 16 bits, Local 32 bits
		else if (wsz == 2 && sizeof(WCHAR) == 4)
			{
			U16	uc;

			// Read all characters
			for (U32 i = 0;hr == S_OK && i < len;++i)
				{
				// Read 2 byte char.
				CCLTRY ( read ( pStm, &uc, sizeof(uc) ) );

				// Store four bytes
				CCLOK  ( str.at(i) = (U32) (uc); )
				}	// for

			}	// else if
					
		// TODO:
		else
			{
			dbgprintf ( L"StmPrsBin::load:Different size WCHARs!\r\n" );
//			DebugBreak();
//			MessageBeep(0);
			hr = E_NOTIMPL;
			}	// else
					
		}	// if
				
	// Terminate string
	CCLOK  ( str.at(len) = '\0'; )

	return hr;
	}	// readStr

HRESULT StmPrsBin :: save ( IByteStream *pStm, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IStreamPersist
	//
	//	PURPOSE
	//		-	Save a value to the stream.
	//
	//	PARAMETERS
	//		-	pStm will receive the data
	//		-	v contains the value to save
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	U16		v2;
	U32		v4;

	// De-reference if necessary
	if (hr == S_OK && v.vtype == (VTYPE_VALUE|VTYPE_BYREF) && v.pval != NULL)
		return save ( pStm, *(v.pval) );

	// Value begin
	CCLTRY ( write ( pStm, &(v2 = MARKER_BEGIN) ) );
	CCLTRY ( write ( pStm, &(v2 = v.vtype & VTYPE_TYPEMASK) ) );

	// Process value based on type
	switch ( adtValue::type(v) )
		{
		// Strings
		case VTYPE_STR :
			{
			// Ptr. to string
			const wchar_t *str = (adtValue::type(v) == VTYPE_STR) ? v.pstr : NULL;

			// Valid string ?
			CCLTRYE	( str != NULL, E_UNEXPECTED );

			// Write to stream
			CCLTRY ( writeStr ( pStm, str ) );
			}
			break;

		// Object
		case VTYPE_UNK :
			{
			IDictionary		*pDct		= NULL;
			IList				*pLst		= NULL;
			IByteStream		*pStmV	= NULL;
			IIt				*pIt		= NULL;
			IUnknown			*pUnk		= NULL;
			IMemoryMapped	*pMap		= NULL;
			adtValue			vK,vV;

			// IUnknown ptr.
			pUnk	=	v.punk;

			// Null object ok
			if (hr == S_OK && pUnk == NULL)
				{
				// Store NULL object ID
				CCLTRY ( writeStr ( pStm, L"" ) );
				}	// if

			// List
			else if (hr == S_OK && _QI(pUnk,IID_IList,&pLst) == S_OK)
				{
				U32	sz;

				// Write Id of object
				CCLTRY ( writeStr ( pStm, L"Adt.List" ) );

				// Write count of values
				CCLTRY ( pLst->size ( &sz ) );
				CCLTRY( write ( pStm, &sz ) );

				// Iterate and save values in dictionary
				CCLTRY ( pLst->iterate ( &pIt ) );
				while (hr == S_OK && pIt->read ( vV ) == S_OK)
					{
					// Save value 
					CCLTRY ( save ( pStm, vV ) );

					// Next item in list
					pIt->next();
					}	// while
				}	// if

			// Dictionary
			else if (hr == S_OK && _QI(pUnk,IID_IDictionary,&pDct) == S_OK)
				{
				U32	sz;

				// Write Id of object
				CCLTRY ( writeStr ( pStm, L"Adt.Dictionary" ) );

				// Write count of value pairs
				CCLTRY( pDct->size ( &sz ) );
				CCLTRY( write ( pStm, &sz ) );

				// Iterate and save values in dictionary
				CCLTRY ( pDct->keys ( &pIt ) );
				while (hr == S_OK && pIt->read ( vK ) == S_OK)
					{
					// Load value for key and save both
					if (pDct->load ( vK, vV ) == S_OK)
						{
						CCLTRY ( save ( pStm, vK ) );
						CCLTRY ( save ( pStm, vV ) );
						}	// if

					// Next pair
					pIt->next();
					}	// while

				}	// if

			// Stream
			else if (hr == S_OK && _QI(pUnk,IID_IByteStream,&pStmV) == S_OK)
				{
				U64 opos,sz;

				// Size the stream from current position.
				CCLTRY ( pStmV->available ( &sz ) );

				// See if object has class ID.  If not default to a memory-based
				// stream.  Some objects can stream but need to be created on the other side
/*				if (hr == S_OK)
					{
					IPersist	*pPersist	= NULL;
					if (_QI(pStm,IID_IPersist,&pPersist) == S_OK)
						hr = pPersist->GetClassID ( &clsid );
					else
						clsid = CLSID_StmMemory;
					_RELEASE(pPersist);
					}	// if
*/
				// Write class id
				CCLTRY ( writeStr ( pStm, L"Io.StmMemory" ) );

				// Write size
				CCLTRY( write ( pStm, &(v4 = (U32)sz) ) );

				// Current position
				CCLTRY ( pStmV->seek ( 0, STREAM_SEEK_CUR, &opos ) );

				// Write stream
				if (hr == S_OK && sz > 0)
					hr = pStmV->copyTo ( pStm, sz, NULL );

				// Restore position
				CCLTRY ( pStmV->seek ( opos, STREAM_SEEK_SET, NULL ) );
				}	// else if

			// Memory block
			else if (hr == S_OK && _QI(pUnk,IID_IMemoryMapped,&pMap) == S_OK)
				{
				void	*pv	= NULL;
				U32	sz		= 0;

				// Lock/size the region
				CCLTRY ( pMap->lock ( 0, 0, &pv, &sz ) );

				// Write class id
				CCLTRY ( writeStr ( pStm, L"Io.MemoryBlock" ) );

				// Write size of region
				CCLTRY( write ( pStm, &(v4 = sz) ) );

				// Write region
//				dbgprintf ( L"-> StmPrsBin::save:Memory Block:0x%x:%d bytes\r\n", hr, sz );
				CCLTRY ( write ( pStm, pv, sz ) );
//				dbgprintf ( L"<- StmPrsBin::save:Memory Block:0x%x:%d bytes\r\n", hr, sz );

				// Clean up
				if (pv != NULL)
					pMap->unlock(pv);
				}	// else if

			// Unhandled object
			else if (hr == S_OK)
				{
				hr = E_NOTIMPL;
				dbgprintf ( L"StmPrsBin::save:Object not implemented\r\n" );
				}	// else

			// Clean up
			_RELEASE(pMap);
			_RELEASE(pStmV);
			_RELEASE(pIt);
			_RELEASE(pDct);
			_RELEASE(pLst);
			}	// VTYPE_UNK
			break;

		// Simple types

		// Integer
		case VTYPE_I4 :
			CCLTRY( write ( pStm, (U32 *) &v.vint ) );
			break;
		// Long
		case VTYPE_I8 :
			CCLTRY( write ( pStm, (U64 *) &v.vlong ) );
			break;
		// Float
		case VTYPE_R4 :
			CCLTRY ( write ( pStm, (U32 *) &v.vflt ) );
			break;
		// Double
		case VTYPE_R8 :
			CCLTRY( write ( pStm, (U64 *) &v.vdbl ) );
			break;
		// Date
		case VTYPE_DATE :
			CCLTRY( write ( pStm, (U64 *) &v.vdate ) );
			break;
		// Boolean
		case VTYPE_BOOL :
			CCLTRY( write ( pStm, &v.vbool ) );
			break;
		// Empty
		case VTYPE_EMPTY :
			// Empty values ok
			break;
		default :
			dbgprintf ( L"StmPrsBin::save:Unhandled value type 0x%x\r\n", v.vtype );
			break;
		}	// switch

	// Value end
	CCLTRY ( write ( pStm, &(v2 = MARKER_END) ) );

	return hr;
	}	// save

HRESULT StmPrsBin :: write ( IByteStream *pStm, const U16 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write a 2 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr contains the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U16		v;

	// To big endian
	v = SWAPS(*pBfr);

	// Write
	CCLTRY ( pStm->write ( &v, sizeof(v), NULL ) );

	return hr;
	}	// write

HRESULT StmPrsBin :: write ( IByteStream *pStm, const U32 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write a 4 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr contains the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U32		v;

	// To big endian
	v = SWAPI(*pBfr);

	// Write
	CCLTRY ( pStm->write ( &v, sizeof(v), NULL ) );

	return hr;
	}	// write

HRESULT StmPrsBin :: write ( IByteStream *pStm, const U64 *pBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write a 8 byte value to the stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pBfr contains the data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U64		v;

	// To big endian
	v = SWAPL(*pBfr);

	// Write
	CCLTRY ( pStm->write ( &v, sizeof(v), NULL ) );

	return hr;
	}	// write

HRESULT StmPrsBin :: write ( IByteStream *pStm, const void *pcvBfr, U32 sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Writes entire buffer to stream until error or complete.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	pcvBfr contains the data.
	//		-	sz is the size of the transfer.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	const U8	*pccBfr	= (U8 *) pcvBfr;
	U64		nleft		= sz;
	U64		nx;

	// Continue until error or done
	while (hr == S_OK && nleft)
		{
		CCLTRY( pStm->write ( pccBfr, nleft, &nx ) );
		CCLOK	( nleft	-= nx; )
		CCLOK	( pccBfr += nx; )
		}	// while

	return hr;
	}	// write

HRESULT StmPrsBin :: writeStr ( IByteStream *pStm, const WCHAR *pstr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write string to stream.
	//
	//	PARAMETERS
	//		-	pStm is the stream
	//		-	str is the string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U32		len;
	U16		wsz;
			
	// Valid string ?
	CCLTRYE	( pstr != NULL, E_UNEXPECTED );

	// Write length of string
	CCLOK		( len = (U32)wcslen(pstr); )
	CCLTRY	( write ( pStm, &len ) );

	// Size of local 'wchar'
	CCLOK		( wsz = sizeof(wchar_t); )
	CCLTRY	( write ( pStm, &wsz ) );
			
	// Write string
	CCLTRY	( write ( pStm, pstr, len*sizeof(wchar_t) ) );

	return hr;
	}	// writeStr
