////////////////////////////////////////////////////////////////////////
//
//									STMPRXML.CPP
//
//							XML value stream parser
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"

StmPrsXML :: StmPrsXML ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// StmPrsXML

HRESULT StmPrsXML :: emit ( IByteStream *pStm, const WCHAR *pwStr, 
										bool bSpecial )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Emit a string into the document.
	//
	//	PARAMETERS
	//		-	pStm is the output stream
	//		-	pwStr is the string to emit
	//		-	bSpecial is true to watch for special characters
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U32		len	= (U32)wcslen(pwStr);

	// State check
	CCLTRYE ( pStm != NULL, E_UNEXPECTED );
//	dbgprintf ( L"StmPrsXML::emit:%s\r\n", pwStr );

	// Output characters to document, watch for special characters
	for (U32 i = 0;hr == S_OK && i < len;++i)
		{
		// Special char ?
		if (	bSpecial &&
				(	pwStr[i] == WCHAR('&')	||
					pwStr[i] == WCHAR('<')	||
					pwStr[i] == WCHAR('>')	||
					pwStr[i] == WCHAR('\"') ||
					pwStr[i] == WCHAR('\'') ) )
			{
			U8	c = (U8)('&');

			// Output an ampersand then output the rest of the string
			hr = writeAll ( pStm, &c, 1 );

			// Special string
			CCLTRY ( emit(	pStm, 
								(pwStr[i] == WCHAR('&'))	? L"amp;" :
								(pwStr[i] == WCHAR('<'))	? L"lt;" :
								(pwStr[i] == WCHAR('>'))	? L"gt;" :
								(pwStr[i] == WCHAR('\"'))	? L"quot;" :
								(pwStr[i] == WCHAR('\''))	? L"apos;" : L"" ) );
			}	// if
		else
			{
			U8	c = (U8)(pwStr[i]);

			// Output byte version of char
			hr = writeAll ( pStm, &c, 1 );
			}	// else

		}	// for

	return hr;
	}	// emit

HRESULT StmPrsXML :: load ( IByteStream *pStm, ADTVALUE &v )
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
	HRESULT	hr	= E_NOTIMPL;

	return hr;
	}	// load

HRESULT StmPrsXML :: save ( IByteStream *pStm, const ADTVALUE &v )
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

	// Object ?
	if (hr == S_OK && v.vtype == VTYPE_UNK)
		{
		IDictionary	*pDct	= NULL;
		IList			*pLst	= NULL;
		IIt			*pIt	= NULL;
		adtValue		vK,vV;

		// List
		if (hr == S_OK && v.punk != NULL && _QI(v.punk,IID_IList,&pLst) == S_OK)
			{
			// Begin container
			CCLTRY ( emit ( pStm, L"<List>" ) );

			// Iterate and write values
			CCLTRY ( pLst->iterate ( &pIt ) );
			while (hr == S_OK && pIt->read ( vV ) == S_OK)
				{
				// Write value
				CCLTRY ( save ( pStm, vV ) );

				// Next key
				pIt->next();
				}	// while

			// End container
			CCLTRY ( emit ( pStm, L"</List>" ) );

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pLst);
			}	// if

		// Dictionary ?
		else if (hr == S_OK && v.punk != NULL && _QI(v.punk,IID_IDictionary,&pDct) == S_OK)
			{
			// Begin dictionary
			CCLTRY ( emit ( pStm, L"<Dictionary>" ) );

			// Iterate and write values
			CCLTRY ( pDct->keys ( &pIt ) );
			while (hr == S_OK && pIt->read ( vK ) == S_OK)
				{
				// Write value
				if (pDct->load ( vK, vV ) == S_OK)
					{
					CCLTRY ( save ( pStm, vK ) );
					CCLTRY ( save ( pStm, vV ) );
					}	// if

				// Next key
				pIt->next();
				}	// while

			// End dictionary
			CCLTRY ( emit ( pStm, L"</Dictionary>" ) );

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pDct);
			}	// if

		// Unsupported object
		else if (hr == S_OK)
			{
			// Good default ?
			CCLTRY ( emit ( pStm, L"<Object></Object>" ) );
			}	// else if

		}	// if

	// Value
	else if (hr == S_OK)
		{
		adtString	strVal;

		// If data type is not a string, must place attribute in node.
		if (adtValue::type(v) != VTYPE_STR)
			{
			// Begin value
			CCLTRY ( emit ( pStm, L"<Value Type=\"" ) );

			// Name of data type
			CCLTRY ( emit (	pStm, 
									(v.vtype == VTYPE_I4)		? L"Integer\">" :
									(v.vtype == VTYPE_I8)		? L"Long\">" :
									(v.vtype == VTYPE_R4)		? L"Float\">" :
									(v.vtype == VTYPE_R8)		? L"Double\">" :
									(v.vtype == VTYPE_DATE)		? L"Date\">" :
									(v.vtype == VTYPE_BOOL)		? L"Boolean\">" :
									(v.vtype == VTYPE_EMPTY)	? L"Empty\">" : L"String\">" ) );
			}	// if
		else
			hr = emit ( pStm, L"<Value>" );

		// Convert value to a string
		if (hr == S_OK && adtValue::toString ( v, strVal ) == S_OK)
			hr = emit ( pStm, &strVal[0], true );

		// End value
		CCLTRY ( emit ( pStm, L"</Value>" ) );
		}	// else if

	return hr;
	}	// save

HRESULT StmPrsXML :: writeAll ( IByteStream *pStm, const void *pcvBfr,
											U32 sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Writes to stream until error or complete.
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
		CCLOK	( nleft -= nx; )
		CCLOK	( pccBfr += nx; )
		}	// while

	return hr;
	}	// writeAll
