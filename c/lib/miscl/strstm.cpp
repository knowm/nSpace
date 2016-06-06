////////////////////////////////////////////////////////////////////////
//
//									STRSTM.CPP
//
//				Implementation of the string stream node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

#define	SIZE_STRALLOC		100						// Alloc increments

StringStream :: StringStream ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pStm			= NULL;
	pbStr			= NULL;
	nstr			= 0;
	nalloc		= 0;
	bTerm			= false;
	bHex			= false;
	sCodePage	= L"ANSI";
	}	// StringStream

HRESULT StringStream :: decodeStr ( adtString &str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to decode the cached string bytes according to
	//			the selected code page.
	//
	//	PARAMETERS
	//		-	str will receive the string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	#ifdef	_WIN32
	UINT		cp,len = 0;

	// Code page
	cp =	(!WCASECMP(sCodePage,L"UTF8")) ? CP_UTF8 :
			(!WCASECMP(sCodePage,L"UTF7")) ? CP_UTF7 : CP_ACP;

	// Sanity check
	CCLTRYE ( pbStr != NULL, E_UNEXPECTED );

	// Compute how	much memory is needed
	CCLTRYE ( (len = MultiByteToWideChar ( cp, 0, (LPCSTR) pbStr, -1, NULL,
														0 )) > 0, GetLastError() );

	// Allocate memory to hold string
	CCLTRY ( str.allocate ( len ) );

	// Convert string
	CCLTRYE ( MultiByteToWideChar ( cp, 0, (LPCSTR) pbStr, -1, &str.at(),
												len ) > 0, GetLastError() );
	#elif		__unix__ || __APPLE__
	// TODO: For unix

	// Sanity check
	CCLTRYE ( pbStr != NULL, E_UNEXPECTED );

	// Succeed for now since most strings will untouched
	CCLOK ( str = (char *) pbStr; )
	#endif

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"StringStream::decodeStr:Failed, hr 0x%x\r\n", hr );

	return hr;
	}	// decodeStr

void StringStream :: destruct ( void )
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
	_FREEMEM(pbStr);
	_RELEASE(pStm);
	}	// destruct

HRESULT StringStream :: onAttach ( bool bAttach )
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
	adtValue vL;

	// Attach ?
	if (!bAttach) return S_OK;

	// Attributes
	pnDesc->load ( adtString ( L"Terminator" ),	sTerm );
	pnDesc->load ( adtString ( L"Terminate" ),	bTerm );
	pnDesc->load ( adtString ( L"CodePage" ),		sCodePage );

	// Support ASCII-encoded binary. i.e. "0AF3" = ( 10, 243 );
	if (pnDesc->load ( adtString(L"Hex"), vL ) == S_OK)
		bHex = adtBool(vL);

	return S_OK;
	}	// onAttach

HRESULT StringStream :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// From stream
	if (_RCP(From))
		{
		U32	tlen	= sTerm.length();
		U32	plen	= sPre.length();
		U32	i;
		U8		c;

		// State check
		CCLTRYE ( (pStm != NULL), ERROR_INVALID_STATE );

		// Continue reading bytes from stream until a string has been extracted.
		while (hr == S_OK && pStm->read ( &c, 1, NULL ) == S_OK)
			{
			// Need bigger string ? (Leave room for possible double character for byte (hex)
			if (hr == S_OK && nstr+1 >= nalloc)
				{
				// Place upper limit on how big the string can grow.  Make this
				// a node property if someone needs a larger limit.
				if (nalloc < 0x10000)				// 64k
					{
					CCLTRYE (	(pbStr = (BYTE *) _REALLOCMEM ( pbStr,
									(nalloc+SIZE_STRALLOC+1) )) != NULL, E_OUTOFMEMORY );
					CCLOK ( nalloc += SIZE_STRALLOC; )
					}	// if
				else
					nstr			= 0;
				}	// if

			// Add to string
			if (hr == S_OK && bHex == false)
				{
				pbStr[nstr++] = c;
				pbStr[nstr] = '\0';
				}	// if
			else if (hr == S_OK)
				{
				pbStr[nstr++] = HEX2WCHAR( ((c>>4) & 0xf) );
				pbStr[nstr++] = HEX2WCHAR( ((c>>0) & 0xf) );
				pbStr[nstr] = '\0';
				}	// else

			// Not really efficient to keep decoding the string after every character
			// but it seems to have to be done before proper termination and such can be checked for.
			// This does not handle 'noise' in the form of nulls... TODO ?
//			CCLTRY ( decodeStr ( sStr ) );
//			CCLOK  ( slen = sStr.length(); )

			// If a prefix is terminated, make sure it matches
			if (hr == S_OK && plen && nstr >= plen)
				{
				// Prefix check
				for (i = 0;i < plen;++i)
					if (pbStr[i] != sPre[i])
						break;

				// If loop did not make it to end, prefix does not match.
				if (i < plen)
					nstr	= 0;
				}	// if

			// Termination check.  
			// Defaulting to null termination does not make sense when converting
			// from binary streams, do not assume null termination.
			if (hr == S_OK && (tlen > 0 && nstr >= tlen))// || (tlen == 0 && c == '\0')))
				{
				// Search for matching termination
				for (i = 0;hr == S_OK && i < tlen;++i)
					if (pbStr[nstr-tlen+i] != sTerm[i])
						break;

				// Termination reached ?
				if (hr == S_OK && i >= tlen)
					{
					adtString	sStr;

					// Convert string
					if (sCodePage.length() > 0)
						decodeStr ( sStr );
					else
						sStr = (char *) pbStr;

					// Result
					nstr			= 0;
					pbStr[nstr] = '\0';
					_EMT(Fire,sStr);

					// String found, end of loop
					break;
					}	// if
				}	// if

			}	// while

		// If the end of the stream has been reached and string data was
		// accumulated, transmit.
		if (hr == S_OK && nstr > 0)
			{
			adtString	sStr;

			// Convert string
			if (sCodePage.length() > 0)
				decodeStr ( sStr );
			else
				sStr = (char *) pbStr;

			// Result
			nstr			= 0;
			pbStr[nstr] = '\0';
			_EMT(Fire,sStr);
			}	// if

		// End of stream has been reached
//		CCLOK ( _EMT(End,adtIUnknown(pStm)); )
		}	// if

	// To stream
	else if (_RCP(To))
		{
		U32		len	= sFrom.length();
		U32		tlen	= sTerm.length();
		U32		i,j;
		U64		opos;
		U8			c;

		// State check
		CCLTRYE ( (pStm != NULL), ERROR_INVALID_STATE );

		// Support writing in the middle of a stream for 'append' mode but
		// still need to avoid sending out the stream with unwanted 'extra' bytes.
		// So set the final size of the stream.
		CCLTRY	( pStm->seek ( 0, STREAM_SEEK_CUR, &opos ) );
		CCLOK    ( j = (bTerm == false)	? 0 :
							(!tlen)				? 1 : tlen; )
		if (hr == S_OK && bHex == true) len >>= 1;

		// Invalid ?  This sets the ptr at the end and then additional
		// writes below insert unnecessary bytes, let stream length grow by iself.
//		CCLTRY	( pStm->setSize ( opos + len + j ) );

		// Write string.
		for (i = 0,j = 0;i < len && hr == S_OK;++i)
			{
			// Convert from string to byte
			if (bHex == false)
				c = (U8) (sFrom[i]);
			else
				{
				c =	(U8)	((WCHAR2HEX(sFrom[2*i+0])) << 4);
				c |=	(U8)	WCHAR2HEX(sFrom[2*i+1]);
				}	// else

			// Write
			hr = pStm->write ( &c, 1, NULL );
			}	// for

		// Write terminator
		for (i = 0;i < tlen && hr == S_OK;++i)
			{
			// Single byte
			c = (char) (sTerm[i]);

			// Write
			hr = pStm->write ( &c, 1, NULL );
			}	// for

		// If no terminator write default
		if (hr == S_OK && !tlen && bTerm == true)
			{
			c  = '\0';
			hr = pStm->write ( &c, 1, NULL );
			}	// if

		// Result
		CCLOK ( _EMT(Fire,adtIUnknown(pStm) ); )
		}	// if

	// New stream
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		hr = _QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if

	// End
	else if (_RCP(End))
		{
		adtString	sStr;

		// Signal to indicate end of incoming binary data, emit current string
		if (nstr > 0)
			{
			decodeStr ( sStr );
			nstr = 0;
			}	// if
		_EMT(Fire,sStr);
		}	// else if

	// Reset
	else if (_RCP(Reset))
		{
		// Reset internal state
		nstr			= 0;
		if (pbStr != NULL)
			pbStr[nstr] = '\0';
		}	// else if

	// Context
	else if (_RCP(Prefix))
		sPre = v;
	else if (_RCP(String))
		sFrom = v;
	else if (_RCP(Terminate))
		sTerm = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


