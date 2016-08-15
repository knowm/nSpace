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
	#ifdef	__USESAX__
	punkRdr		= NULL;
	pSAXStk		= NULL;
	pSAXStkIt	= NULL;
	tSAX			= VTYPE_EMPTY;
	strResv		= L"___Reserved___";
	#endif
	#ifdef		_WIN32
	pStmStm		= NULL;
	#endif
	}	// StmPrsXML

HRESULT StmPrsXML :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Stream on byte stream for Win32
	#ifdef	_WIN32
	CCLTRY( COCREATE ( L"Io.StmOnByteStm", IID_IHaveValue, &pStmStm ));
	#endif

	#ifdef	__USESAX__
	// Create Simple API XML reader
	CCLTRY(CoCreateInstance ( __uuidof(SAXXMLReader), NULL, CLSCTX_ALL,
										__uuidof(ISAXXMLReader), (void **) &punkRdr ));

	// Object stack
	CCLTRY(COCREATE ( L"Adt.Stack", IID_IList, &pSAXStk ));
	CCLTRY(pSAXStk->iterate ( &pSAXStkIt ));
	#endif

	return hr;
	}	// construct

void StmPrsXML :: destruct ( void )
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
	#ifdef	__USESAX__
	_RELEASE(pSAXStkIt);
	_RELEASE(pSAXStk);
	_RELEASE(punkRdr);
	#endif
	}	// destruct

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
	HRESULT		hr	= S_OK;

	// 'IStream' interface in front of the 'IByteStream' for the XML object
	CCLTRY ( pStmStm->setValue ( adtIUnknown(pStm) ) );

	#ifdef	__USESAX__
	ISAXXMLReader	*pRdr	= (ISAXXMLReader *) punkRdr;

	// Parse document
	CCLTRY(pRdr->putContentHandler(this));
	CCLTRY(pRdr->parse ( adtVariant(pStmStm) ));
	CCLTRY(pRdr->putContentHandler(NULL));

	// Result
	CCLTRY ( adtValue::copy ( vSAXFrom, v ) );

	// Clean up
	adtValue::clear(vSAXFrom);
	#endif

	// Clean up
	hr = E_NOTIMPL;

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

//
// SAX callbacks
//

#ifdef	__USESAX__

// Routines for Simple API for XML.  Speeds up parsing as this API allows bypassing
// of the whole document object model (DOM) which is overkill for this node.

HRESULT StmPrsXML :: putDocumentLocator ( ISAXLocator *pLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Interface of origin of SAX document events.
	//
	//	PARAMETERS
	//		-	pLoc is a valid instance of the ISAXLocator interface
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// putDocumentLocator

HRESULT StmPrsXML :: startDocument ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of beginning of document.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Prepare stack
	CCLTRY(pSAXStk->clear());

	return (hr == S_OK) ? S_OK : E_FAIL;
	}	// startDocument

HRESULT StmPrsXML :: endDocument ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of end of document.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// endDocument

HRESULT StmPrsXML ::
	startPrefixMapping (	const wchar_t *pwchPrefix, int cchPrefix,
								const wchar_t *pwchUri, int cchUri )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Begins the scope of the prefix-URI namespace mapping
	//
	//	PARAMETERS
	//		-	pwchPrefix,cchPrefix is the prefix being mapped
	//		-	pwchUri,cchUri is the namespace to which the prefix is mapped
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// startPrefixMapping

HRESULT StmPrsXML ::
	endPrefixMapping ( const wchar_t *pwchPrefix, int cchPrefix )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Ends the scope of the prefix-URI namespace mapping
	//
	//	PARAMETERS
	//		-	pwchPrefix,cchPrefix is the prefix being mapped
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// endPrefixMapping

HRESULT StmPrsXML ::
	startElement (	const wchar_t *pwchNamespaceUri, int cchNamespaceUri,
						const wchar_t *pwchLocalName, int cchLocalName,
						const wchar_t *pwchQName, int cchQName,
						ISAXAttributes *pAttr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of the beginning of an element.
	//
	//	PARAMETERS
	//		-	pwchNamespaceUri,cchNamespaceUri is the namespace URI.
	//		-	pwchLocalName,cchLocalName is the local name string.
	//		-	pwchQName,cchQName is the QName string.
	//		-	pAttr are the attributes attached to the element
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Push previous object
	if (vSAXFrom.vtype != VTYPE_EMPTY)
		hr = pSAXStk->write ( vSAXFrom );

	// Valid elements are : Dictionary, List, or Value

	// Dictionary
	if (!WCASENCMP(L"Dictionary",pwchLocalName,cchLocalName))
		{
		IDictionary	*pDict	= NULL;

		// Create a dictionary object for the value
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDict));

		// New value
		CCLTRY(adtValue::copy ( adtIUnknown(pDict), vSAXFrom ));

		// Clean up
		_RELEASE(pDict);
		}	// if

	// List
	else if (!WCASENCMP(L"List",pwchLocalName,cchLocalName))
		{
		IContainer	*pCont = NULL;

		// Create a list object for the value
		CCLTRY ( COCREATE ( L"Adt.List", IID_IContainer, &pCont ) );

		// New value
		CCLTRY(adtValue::copy ( adtIUnknown(pCont), vSAXFrom ));

		// Clean up
		_RELEASE(pCont);
		}	// if

	// Value
	else if (!WCASENCMP(L"Value",pwchLocalName,cchLocalName))
		{
		const wchar_t	*pwType;
		int				tLen;

		// Data type specified ?
		if (pAttr != NULL && pAttr->getValueFromName ( L"", 0, 
				L"Type", 4, &pwType, &tLen ) == S_OK && tLen > 1)
			{
			// Determine type from first letter
			if			(pwType[0] == 'I' || pwType[0] == 'i')		tSAX = VTYPE_I4;
			else if	(pwType[0] == 'L' || pwType[0] == 'l')		tSAX = VTYPE_I8;
			else if	(pwType[0] == 'B' || pwType[0] == 'b')		tSAX = VTYPE_BOOL;
			else if	(pwType[0] == 'F' || pwType[0] == 'f')		tSAX = VTYPE_R4;
			else if	(pwType[0] == 'D' || pwType[0] == 'd')
				{
				if			(pwType[1] == 'A' || pwType[1] == 'a')	tSAX = VTYPE_DATE;
				else if	(pwType[1] == 'O' || pwType[1] == 'o')	tSAX = VTYPE_R8;
				else															tSAX = VTYPE_STR;
				}	// else if
			else if	(pwType[0] == 'D' || pwType[0] == 'd')		tSAX = VTYPE_DATE;
			else																tSAX = VTYPE_STR;
			}	// if
		else
			tSAX = VTYPE_STR;

		// Clear current value
		adtValue::clear(vSAXFrom);
		}	// if

	// Invalid
	else
		hr = E_UNEXPECTED;

	return (hr == S_OK) ? S_OK : E_FAIL;
	}	// startElement

HRESULT StmPrsXML ::
	endElement (	const wchar_t *pwchNamespaceUri, int cchNamespaceUri,
						const wchar_t *pwchLocalName, int cchLocalName,
						const wchar_t *pwchQName, int cchQName )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of the end of an element.
	//
	//	PARAMETERS
	//		-	pwchNamespaceUri,cchNamespaceUri is the namespace URI.
	//		-	pwchLocalName,cchLocalName is the local name string.
	//		-	pwchQName,cchQName is the QName string.
	//		-	pAttr are the attributes attached to the element
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDict	= NULL;
	IList			*pCont	= NULL;
	adtValue		vParent;

	// Value
	if (!WCASENCMP(L"Value",pwchLocalName,cchLocalName))
		{
		// Convert value from a string
		CCLTRY ( adtValue::fromString ( sSAXFrom, tSAX, vSAXFrom ) );
		sSAXFrom = L"";
		tSAX		= VTYPE_EMPTY;
		}	// if

	// Look at next value on stack
	if (pSAXStkIt->read ( vParent ) == S_OK)
		{
		// Parent type
		if ( vParent.vtype == VTYPE_UNK && vParent.punk != NULL )
			{
			// Dictionary
			if (_QI(vParent.punk,IID_IDictionary,&pDict) == S_OK)
				{
				adtValue	vKey;

				// If key not available, current value becomes key
				if ( pDict->load ( strResv, vKey ) != S_OK )
					{
					// Store key temporarily in dictionary for value later on.
					CCLTRY(pDict->store ( strResv, vSAXFrom ));
					}	// if
				else
					{
					// Its possible to send a 'null' value, in this case the 'characters' callback
					// will never get called and 'vSAXFrom' will be empty
					if (vSAXFrom.vtype == VTYPE_EMPTY)
						{
						CCLTRY ( adtValue::fromString ( adtString(L""), tSAX, vSAXFrom ) );
						tSAX	= VTYPE_EMPTY;
						}	// if

					// Store value under active key
					CCLTRY(pDict->store ( vKey, vSAXFrom ));
					CCLOK (pDict->remove ( strResv );)
					}	// if

				// Clean up
				_RELEASE(pDict);
				}	// if

			// List
			else if (_QI(vParent.punk,IID_IList,&pCont) == S_OK)
				{
				// Write active value to list
				CCLTRY(pCont->write ( vSAXFrom ));

				// Clean up
				_RELEASE(pCont);
				}	// else if

			}	// if

		// New active value
		CCLTRY ( adtValue::copy ( vParent, vSAXFrom ) );

		// Pop value off stack
		pSAXStkIt->next();
		}	// if

	return (hr == S_OK) ? S_OK : E_FAIL;
	}	// endElement

HRESULT StmPrsXML ::
	characters ( const wchar_t *pwchChars, int cchChars )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Receives notification of character data.
	//
	//	PARAMETERS
	//		-	pwchrChars,cchChars is the character data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	sVal;

	// Inside a value ?  The reader passes whitespace as well as valid
	// 'in node' text, so ignore non-value characters
	if (tSAX == VTYPE_EMPTY)
		return S_OK;

	// It's possible this function will be called repeatadly for the
	// same text line (to handle special characters).  Must simply append
	// until the whole string is read before conversion to a value can
	// take place.

	// Generate new string
	CCLTRY( sVal.allocate ( sSAXFrom.length() + cchChars ) );
	CCLOK ( WCSCPY ( &sVal.at(), sSAXFrom.length() + cchChars, sSAXFrom ); )
	CCLOK ( WCSCAT ( &sVal.at(), cchChars, pwchChars ); )
	CCLTRY( adtValue::copy ( sVal, sSAXFrom ) );

	return (hr == S_OK) ? S_OK : E_FAIL;
	}	// characters

HRESULT StmPrsXML ::
	ignorableWhitespace ( const wchar_t *pwchChars, int cchChars )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Not currently used in SAX.
	//
	//	PARAMETERS
	//		-	pwchrChars,cchChars is the character data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// ignorableWhitespace

HRESULT StmPrsXML ::
	processingInstruction (	const wchar_t *pwchTarget, int cchTarget,
									const wchar_t *pwchData, int cchData )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of instruction processing
	//
	//	PARAMETERS
	//		-	pwchTarget,cchTarget is the target for the instruction
	//		-	pwchData,cchData is the instruction data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// processingInstruction

HRESULT StmPrsXML ::
	skippedEntity ( const wchar_t *pwchName, int cchName )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ISAXContentHandler
	//
	//	PURPOSE
	//		-	Notification of a skipped entity.
	//
	//	PARAMETERS
	//		-	pwchrName,cchName is the skipped entity.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// skippedEntity

#endif
