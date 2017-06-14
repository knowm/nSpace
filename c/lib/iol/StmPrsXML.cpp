////////////////////////////////////////////////////////////////////////
//
//									STMPRXML.CPP
//
//							XML value stream parser
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#if		defined(_WIN32)
#include <msxml.h>
#elif		defined(__unix__)
#include <libxml2/libxml/parser.h>
#endif
#include <stdio.h>

#define	HDR_XML	"<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n"

StmPrsXML :: StmPrsXML ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStmDoc		= NULL;
	#ifdef		_WIN32
	pStmStm		= NULL;
	pXMLDocLoad	= NULL;
	pXMLDocNode	= NULL;
	lChild		= 0;
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
	HRESULT	hr		= S_OK;

	#ifdef	_WIN32

	// Stream on byte stream for Win32
	CCLTRY(COCREATE ( L"Io.StmOnByteStm", IID_IHaveValue, &pStmStm ));

	// Create XML documents using during processing
	CCLTRY(CoCreateInstance ( CLSID_DOMDocument, NULL, CLSCTX_ALL, 
										IID_IXMLDOMDocument, (void **) &pXMLDocLoad ) );
	#endif

	return hr;
	}	// construct

void StmPrsXML :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pStmDoc);
	#ifdef		_WIN32
	_RELEASE(pStmStm);
	_RELEASE(pXMLDocLoad);
	#endif
	}	// destruct

HRESULT StmPrsXML :: emit ( const WCHAR *pwStr, bool bSpecial )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Emit a string into the document.
	//
	//	PARAMETERS
	//		-	pwStr is the string to emit
	//		-	bSpecial is true to watch for special characters
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U32		len	= (U32)wcslen(pwStr);
	U8			cs;

	// State check
	CCLTRYE ( pStmDoc != NULL, E_UNEXPECTED );

	// Output characters to document, watch for special characters
	for (U32 i = 0;hr == S_OK && i < len;++i)
		{
		// Filtered character (TODO: Handle real unicode ?)
		cs = (pwStr[i] & 0xff);

		// Unprintable character ?
		if (cs < 32 || cs > 126)
			{
			WCHAR	wEnc[11];

			// Some characters are invalid even if encoded properly, only
			// handle know ones
			if (cs == '\t' || cs == '\r' || cs == '\n')
				{
				// Encode unprintable character
				swprintf ( SWPF(wEnc,11), L"&#x%02x;", (U8)(cs & 0xff) );
				emit ( wEnc );
				}	// if
			else
				emit (L" ");
			}	// else if

		// Special char ?
		else if (	bSpecial &&
					(	cs == WCHAR('&')	||
						cs == WCHAR('<')	||
						cs == WCHAR('>')	||
						cs == WCHAR('\"') ||
						cs == WCHAR('\'') ||
						cs == WCHAR('%') ) )
			{
			U8	c = (U8)('&');

			// Output an ampersand then output the rest of the string
			hr = pStmDoc->write ( &c, 1, NULL );

			// Special string
			CCLTRY ( emit(	(cs == WCHAR('&'))	? L"amp;" :
								(cs == WCHAR('<'))	? L"lt;" :
								(cs == WCHAR('>'))	? L"gt;" :
								(cs == WCHAR('\"'))	? L"quot;" :
								(cs == WCHAR('\''))	? L"apos;" :
								(cs == WCHAR('%'))	? L"#37;" : L"" ) );
			}	// if

		else
			{
			U8	c = (U8)(cs);

			// Output byte version of char
			hr = pStmDoc->write ( &c, 1, NULL );
			}	// else

		}	// for

	return hr;
	}	// emit

HRESULT StmPrsXML :: load ( IByteStream *pUnkStm, ADTVALUE &oVal )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IParseStream
	//
	//	PURPOSE
	//		-	Called to load an object from a stream.
	//
	//	PARAMETERS
	//		-	pUnkStm is the stream to load from
	//		-	oVal will receive the loaded object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	/////////////////
	// Win32 - MSXML
	/////////////////
	#ifdef		_WIN32
	IXMLDOMDocument	*pXMLDoc		= (IXMLDOMDocument *) pXMLDocLoad;
	IXMLDOMNode			*pXMLNode	= NULL;
	VARIANT_BOOL		b;
	adtValue				vFrom;

	// Place 'IStream' interface in front of the 'IByteStream' for the 
	// XML object to API can be used.
	CCLTRY ( pStmStm->setValue ( adtIUnknown(pUnkStm) ) );

	// Create and load XML document
	CCLTRY (pXMLDoc->load ( adtVariant(pStmStm), &b ) );
	CCLTRYE(b == VARIANT_TRUE,ERROR_INVALID_STATE);

	// Start current node at document level
	CCLOK ( pXMLDocNode = pXMLDoc; )
	_ADDREF(pXMLDocNode);
	CCLOK ( lChild = 0; )

	// Load
	CCLTRY ( valueLoad ( oVal ) );

	// Clean up
	_RELEASE(pXMLDocNode);

	////////////////////////
	// Unix (using libxml2)
	////////////////////////
	#elif		__unix__
	xmlDocPtr		pDoc		= NULL;
	xmlNodePtr		pNode		= NULL;
	IByteStream		*pStmSrc	= NULL;
	IByteStream		*pStmDst	= NULL;
	IMemoryMapped	*pMemDst	= NULL;
	char				*pcXML	= NULL;
	U32				sz;

	// Destination stream
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pMemDst ) );
	CCLTRY ( pMemDst->stream ( &pStmDst ) );

	// Load provided stream into memory
	CCLTRY ( pUnkStm->copyTo ( pStmDst, 0, NULL ) );

	// Parse file
	if (hr == S_OK)
		{
		CCLTRY 	( pMemDst->lock ( 0, 0, (void **) &pcXML, &sz ) );
		CCLTRYE 	( (pDoc = xmlParseMemory ( pcXML, sz )) != NULL, E_INVALIDARG );
		if (hr != S_OK) lprintf ( LOG_ERR, L"StmPrsXML::valueLoad:Parse error\n" );
		}	// if

	// Top element
	CCLTRYE ( (pNode = xmlDocGetRootElement ( pDoc )) != NULL, E_UNEXPECTED );

	// Initialize first child (root)
	CCLOK ( pXMLDocChild	= pNode; )

	// Load
	CCLTRY ( valueLoad ( oVal ) );

	// Clean up
	if (pDoc != NULL) xmlFreeDoc ( pDoc );
	_UNLOCK(pMemDst,pcXML);
	_RELEASE(pStmDst);
	_RELEASE(pMemDst);
	_RELEASE(pStmSrc);
	#endif

	return hr;
	}	// load

HRESULT StmPrsXML :: save ( IByteStream *pUnkStm, const ADTVALUE &oVal )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IParseStream
	//
	//	PURPOSE
	//		-	Called to save an object to a stream.
	//
	//	PARAMETERS
	//		-	pUnkStm is the stream to save to.
	//		-	oVal contains the object to save
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	// State check
	CCLTRYE ( (adtValue::empty(oVal) == false), ERROR_INVALID_STATE );

	// Byte stream interface
	CCLTRY ( _QISAFE(pUnkStm,IID_IByteStream,&pStmDoc) );

	// Write value to document
	CCLTRY( valueSave ( oVal ) );

	// Clean up
	_RELEASE(pStmDoc);

	return hr;
	}	// save

HRESULT StmPrsXML :: valueLoad ( ADTVALUE &oVal )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Loads next child from current node.
	//
	//	PARAMETERS
	//		-	oVal will receive the loaded object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr				= S_OK;
	#ifdef				_WIN32
	BSTR					name			= NULL;
	IXMLDOMNode			*pXMLNode	= NULL;
	IXMLDOMNode			*pChild		= NULL;
	IXMLDOMNodeList	*pChildren	= NULL;
	IUnknown				*pTmp;
	LONG					lTmp,l;
	DOMNodeType			nt;
	#elif					__unix__
	xmlNodePtr			pChild;
	void					*pChildTmp;
	adtString			name;
	#endif

	// Active node
	#ifdef	_WIN32
	CCLTRY ( _QISAFE(pXMLDocNode,IID_IXMLDOMNode,&pXMLNode) );
	#endif

	// Obtain current child node, skip non-element nodes
	#ifdef	_WIN32
	CCLTRY	( pXMLNode->get_childNodes ( &pChildren ) );
	CCLTRY	( pChildren->get_length ( &l ) );
	for (;hr == S_OK && lChild < l;++lChild)
		{
		// Child
		CCLTRY	( pChildren->get_item ( lChild, &pChild ) );

		// Process only if valid node type
		CCLTRY	( pChild->get_nodeType ( &nt ) );
		if (hr == S_OK && nt == NODE_ELEMENT)
			break;

		// Clean up
		_RELEASE(pChild);
		}	// for

	// Valid child ?
	CCLTRYE	( pChild != NULL, S_FALSE );
	CCLOK		( ++lChild; )

	#elif		__unix__
	CCLOK		( pChild 		= (xmlNodePtr) pXMLDocChild; )
	CCLOK		( pXMLDocChild = NULL; )
	while (hr == S_OK && pChild != NULL)
		{
		// Element node ?
		if (pChild->type == XML_ELEMENT_NODE)
			break;

		// Next child
		pChild = pChild->next;
		}	// while

	// Valid child ?
	CCLTRYE	( pChild != NULL, S_FALSE );
	CCLOK		( pXMLDocChild = pChild->next; )
	#endif

	// Obtain node string to obtain value type
	#ifdef	_WIN32
	CCLTRY ( pChild->get_nodeName ( &name ) );
	#elif		__unix__
	CCLOK ( name = (const char *) pChild->name; )
	#endif

	// Dictionary ?
	if (hr == S_OK && (!WCASECMP(name,L"DICTIONARY") || !WCASECMP(name,L"LIST")))
		{
		IDictionary	*pDct	= NULL;
		IList			*pLst	= NULL;
		adtValue		vKey,vValue;

		// Create a dictionary object for the value
		if (!WCASECMP(name,L"DICTIONARY"))
			hr = COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDct);
		else
			hr = COCREATE(L"Adt.List",IID_IList,&pLst);

		// Backup state for current node and set new state
		#ifdef		_WIN32
		pTmp				= pXMLDocNode;
		pXMLDocNode		= pChild;
		lTmp				= lChild;
		lChild			= 0;
		#elif			__unix__
		pChildTmp		= pXMLDocChild;
		pXMLDocChild	= pChild->children;
		#endif

		// Load each key/value pair and add to container
		if (pDct != NULL)
			{
			while (	hr							== S_OK &&
						valueLoad ( vKey )	== S_OK &&
						valueLoad ( vValue )	== S_OK )
				hr = pDct->store ( vKey, vValue );
			}	// if
		else
			{
			while (	hr							== S_OK &&
						valueLoad ( vValue )	== S_OK )
				hr = pLst->write ( vValue );
			}	// else

		// Restore state
		#ifdef		_WIN32
		pXMLDocNode		= pTmp;
		lChild			= lTmp;
		#elif			__unix__
		pXMLDocChild	= pChildTmp;
		#endif

		// Value load successful
		if (hr == S_OK && pDct != NULL)
			hr = adtValue::copy ( adtIUnknown(pDct), oVal );
		else if (hr == S_OK)
			hr = adtValue::copy ( adtIUnknown(pLst), oVal );

		// Clean up
		_RELEASE(pLst);
		_RELEASE(pDct);
		}	// if

	// Primitive value ?
	else if (hr == S_OK && !WCASECMP(name,L"VALUE"))
		{
		VALUETYPE				vtype			= VTYPE_EMPTY;
		#ifdef					_WIN32
		IXMLDOMNamedNodeMap	*pAttr		= NULL;
		IXMLDOMNode				*pNodeType	= NULL;
		VARIANT					vT,vS;

		// Setup
		adtValue::clear	( oVal );
		VariantInit			( &vT );
		VariantInit			( &vS );

		// See if a data type is specified in the attribute list
		if (	hr == S_OK &&
				pChild->get_attributes ( &pAttr )								== S_OK &&
				pAttr->getNamedItem ( adtVariant(L"Type"), &pNodeType )	== S_OK &&
				pNodeType->get_nodeValue ( &vT )									== S_OK)
			{
			// Determine type from first letter
			if			(vT.bstrVal[0] == 'I' || vT.bstrVal[0] == 'i')		vtype = VTYPE_I4;
			else if	(vT.bstrVal[0] == 'L' || vT.bstrVal[0] == 'l')		vtype = VTYPE_I8;
			else if	(vT.bstrVal[0] == 'B' || vT.bstrVal[0] == 'b')		vtype = VTYPE_BOOL;
			else if	(vT.bstrVal[0] == 'F' || vT.bstrVal[0] == 'f')		vtype = VTYPE_R4;
			else if	(vT.bstrVal[0] == 'D' || vT.bstrVal[0] == 'd')
				{
				if			(vT.bstrVal[1] == 'A' || vT.bstrVal[1] == 'a')	vtype = VTYPE_DATE;
				else if	(vT.bstrVal[1] == 'O' || vT.bstrVal[1] == 'o')	vtype = VTYPE_R8;
				else																		vtype = VTYPE_STR;
				}	// else if
			else if	(vT.bstrVal[0] == 'D' || vT.bstrVal[0] == 'd')		vtype = VTYPE_DATE;
			else																			vtype = VTYPE_STR;
			}	// if
		else if (hr == S_OK) vtype = VTYPE_STR;

		// Convert value from a string
		CCLOK  ( vS.vt = VT_BSTR; )
		CCLTRY ( pChild->get_text ( &(vS.bstrVal) ) );
		CCLTRY ( adtValue::fromString ( vS.bstrVal, vtype, oVal ) );

		// Clean up
		VariantClear ( &vS );
		VariantClear ( &vT );
		_RELEASE(pNodeType);
		_RELEASE(pAttr);

		#elif		__unix__
		xmlChar		*type;
		adtString	strContent;

		// Is a valid type specified ?
		if ( (type = xmlGetProp ( pChild, (const xmlChar *) "Type" )) != NULL)
			{
			// Determine type from first letter
			if			(type[0] == 'I' || type[0] == 'i')		vtype = VTYPE_I4;
			else if	(type[0] == 'L' || type[0] == 'l')		vtype = VTYPE_I8;
			else if	(type[0] == 'B' || type[0] == 'b')		vtype = VTYPE_BOOL;
			else if	(type[0] == 'F' || type[0] == 'f')		vtype = VTYPE_R4;
			else if	(type[0] == 'D' || type[0] == 'd')
				{
				if			(type[1] == 'A' || type[1] == 'a')	vtype = VTYPE_DATE;
				else if	(type[1] == 'O' || type[1] == 'o')	vtype = VTYPE_R8;
				else														vtype = VTYPE_STR;
				}	// else if
			else if	(type[0] == 'D' || type[0] == 'd')		vtype = VTYPE_DATE;
			else															vtype = VTYPE_STR;
			}	// if
		else vtype = VTYPE_STR;

		// Value strings show up as a text node under the child value
		if (pChild->children != NULL)
			strContent = (const char *) pChild->children->content;
		
		// Apparently a null ("") string results in no child data
		else
			{
			strContent 	= "";
			vtype 		= VTYPE_STR;
			}	// else
		
		// Convert value from a string
//		CCLOK ( dbgprintf ( L"StmPrsXML::valueLoad:Value:%S\r\n", (const char *) pChild->children->content ); )
		CCLTRY ( adtValue::fromString ( strContent, vtype, oVal ) );
		#endif
		}	// else if

	// ?
	else if (hr == S_OK)
		hr = E_UNEXPECTED;

	// Clean up
	#ifdef	_WIN32
	_FREEBSTR(name);
	_RELEASE(pChild);
	_RELEASE(pChildren);
	_RELEASE(pXMLNode);
	#endif

	return hr;
	}	// valueLoad

HRESULT StmPrsXML :: valueSave ( const ADTVALUE &oVal )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Saves next child to a new XML node.
	//
	//	PARAMETERS
	//		-	oVal contains the object to save.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Object ?
	if (hr == S_OK && adtValue::type(oVal) == VTYPE_UNK)
		{
		IDictionary			*pDict	= NULL;
		IContainer			*pCont	= NULL;

		// See if it is a supported object
		CCLTRYE ( oVal.punk != NULL, E_UNEXPECTED );

		// Dictionary ?
		if (hr == S_OK && _QI(oVal.punk,IID_IDictionary,&pDict) == S_OK)
			{
			IIt		*pKeys	= NULL;
			adtValue	vKey,vValue;
			
			// Begin dictionary
			CCLTRY ( emit ( L"<Dictionary>" ) );

			// Save each key/value pair
			CCLTRY ( pDict->keys ( &pKeys ) );
			CCLTRY ( pKeys->begin() );
			while (hr == S_OK && pKeys->read ( vKey ) == S_OK)
				{
				// Save next pair
				if (pDict->load ( vKey, vValue ) == S_OK)
					{
					CCLTRY ( valueSave ( vKey ) );
					CCLTRY ( valueSave ( vValue ) );
					}	// if
				pKeys->next();
				}	// while

			// End dictionary
			CCLTRY ( emit ( L"</Dictionary>" ) );

			// Clean up
			_RELEASE(pKeys);
			_RELEASE(pDict);
			}	// if

		// List ?
		else if (hr == S_OK && _QI(oVal.punk,IID_IContainer,&pCont) == S_OK)
			{
			IIt		*pIt	= NULL;
			adtValue	v;

			// Begin container
			CCLTRY ( emit ( L"<List>" ) );

			// Save each value
			CCLTRY(pCont->iterate ( &pIt ));
			while (hr == S_OK && pIt->read ( v ) == S_OK)
				{
				// Save it
				CCLTRY(valueSave(v));

				// Next
				pIt->next();
				}	// while

			// End container
			CCLTRY ( emit ( L"</List>" ) );

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pCont);
			}	// if

		// Unsupported object
		else if (hr == S_OK)
			{
			// TODO: Should this be an error condition ?  Currently not since 'unsaveable' objects
			// do not stop processing. 
			CCLTRY ( emit ( L"<Value></Value>" ) );
//			hr = E_NOTIMPL;
			}	// else if

		}	// if

	// Value
	else if (hr == S_OK)
		{
		adtString	sVal;

		// If data type is not a string, must place attribute in node.
		if ( oVal.vtype != VTYPE_STR )
			{
			// Begin value
			CCLTRY ( emit ( L"<Value Type=\"" ) );

			// Name of data type
			CCLTRY ( emit (	(oVal.vtype == VTYPE_I4)		? L"Integer\">" :
									(oVal.vtype == VTYPE_I8)		? L"Long\">" :
									(oVal.vtype == VTYPE_R4)		? L"Float\">" :
									(oVal.vtype == VTYPE_R8)		? L"Double\">" :
									(oVal.vtype == VTYPE_DATE)		? L"Date\">" :
									(oVal.vtype == VTYPE_BOOL)		? L"Boolean\">" :
									(oVal.vtype == VTYPE_EMPTY)	? L"Empty\">" : L"String\">" ) );
			}	// if
		else
			CCLTRY ( emit ( L"<Value>" ) );

		// Convert value to a string
		if (hr == S_OK && adtValue::toString ( oVal, sVal ) == S_OK)
			hr =  emit ( &sVal.at(), true );

		// End value
		CCLTRY ( emit ( L"</Value>" ) );
		}	// else if

	return hr;
	}	// valueSave
