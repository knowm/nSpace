////////////////////////////////////////////////////////////////////////
//
//									PARSER.CPP
//
//						Implementation of the nSpace text parser.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

// Definitions
#define	SECINDAY		(60.0*60.0*24.0)
#define	CHAR2HEX(a)													\
	((a) >= ('0') && (a) <= ('9')) ? (a)-'0' 			:		\
	((a) >= ('a') && (a) <= ('f')) ? (a)-('a')+10 	:		\
	((a) >= ('A') && (a) <= ('F')) ? (a)-('A')+10	: 0
#define	HEX2CHAR(a)													\
	(((a) < 10) ? ((a) + '0') : ((a) - 10 + 'a'))
#define	DOC2W(begin,end,aS,wS)									\
		{	char c = aS[end]; aS[end] = '\0';					\
			wS = &(aS[begin]); aS[end] = c; }
#define	ISSPACE(a)													\
			( ((a) >= 0x09 && (a) <= 0x0D) || ((a) == 0x20) )

// Globals
extern GlobalNspc	nspcglb;

PersistTxt :: PersistTxt ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pStmOut		= NULL;
	pDocIn		= NULL;
	paDocIn		= NULL;
	pGraph		= NULL;
	pNodes		= NULL;
	pConns		= NULL;
	pSubs			= NULL;
	pNames		= NULL;
	}	// PersistTxt

HRESULT PersistTxt :: construct ( void )
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

	// Input stream buffer
	CCLTRY	( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pDocIn ) );

	return hr;
	}	// construct

void PersistTxt :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Clean up
	_RELEASE(pConns);
	_RELEASE(pSubs);
	_RELEASE(pNodes);
	_RELEASE(pGraph);
	_RELEASE(pNames);
	_RELEASE(pStmOut);
	_RELEASE(pDocIn);

	}	// destruct

HRESULT PersistTxt :: load ( IByteStream *pStm, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IStreamPersist
	//
	//	PURPOSE
	//		-	Load a value from the specified stream.
	//
	//	PARAMETERS
	//		-	pStm is the source stream
	//		-	v will receive the loaded value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Stream interface
	CCLTRYE	( (pStm != NULL), E_INVALIDARG );

	// Size the stream from current position.
	CCLTRY ( pStm->available ( &uDocIn ) );

	// Read in file into memory to avoid system calls during parsing
	CCLTRY	( pDocIn->setSize ( (U32)(uDocIn+1) ) );
	CCLTRY	( pDocIn->lock ( 0, 0, (void **) &paDocIn, NULL ) );
	CCLOK		( paDocIn[uDocIn] = '\0'; )
	CCLTRY	( pStm->read ( paDocIn, uDocIn, &uDocIn ) );

	// Although this object is a graph loader, general objects can be loaded as well.
	CCLOK		( uLineBegin = uLineEnd = uLineEOL = 0; )
	CCLTRY	( valueLoad ( v ) );

	// Clean up
	_UNLOCK(pDocIn,paDocIn);
	_RELEASE(pConns);
	_RELEASE(pSubs);
	_RELEASE(pNodes);
	_RELEASE(pGraph);

	return hr;
	}	// load

HRESULT PersistTxt :: nextChar ( U32 *pIdx, bool bSkipW )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Retrieves the next non-whitespace character from the document
	//			stream.
	//
	//	PARAMETERS
	//		-	pIdx is the position of the new character
	//		-	bSkipW is true to skip whitespace, false to not skip it
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	bool			bNewLine	= false;
	U32			i,uWhite = 0;

	// Search for next character
	*pIdx	= MAXDWORD;
	while (hr == S_OK && *pIdx == MAXDWORD)
		{
		// If the current line has been exhausted, time to read in next line.
		if (hr == S_OK && uLineBegin >= uLineEnd)
			{
			// Read until next line.  Lines are read one at a time to filter the last
			// parts of lines that are comments (i.e. '%')
			bNewLine		= true;
			uWhite		= uLineEOL;
			uLineBegin	= uLineEOL;
			while (hr == S_OK)
				{
				// Next character, watch EOF
				if (uLineEOL < uDocIn)
					++uLineEOL;
				else
					hr	= S_FALSE;

				// End of line ?
				if (hr == S_OK && (paDocIn[uLineEOL] == '\r' || paDocIn[uLineEOL] == '\n' || paDocIn[uLineEOL] == '\0'))
					{
					// Valid line ? Search until the first non-whitespace
					// character is encountered.
					while (	hr == S_OK &&
								uLineBegin < uLineEOL &&
								ISSPACE(paDocIn[uLineBegin]) &&
								paDocIn[uLineBegin] != '%' )
						++uLineBegin;

					// If loop terminated on a white space or first character is comment, empty line
					// otherwise perform backwards scan... Check for 'end of line' comments (i.e. '%')
					if (uLineBegin != uLineEOL && paDocIn[uLineBegin] != '%')
						{
						int	iq	= 0;
						// Above test catches first char. being a comment
						uLineEnd = uLineEOL;
						for (i = uLineEnd-1;i > uLineBegin;--i)
							{
							// Quote / End quote ?
							if (paDocIn[i] == '"' && paDocIn[i-1] != '\\')
								iq = (iq == 0) ? 1 : 0;

							// If current character is '%' and previous character is not '\' and
							// not inside a quoted string truncate line at comment
							if (paDocIn[i] == '%' && paDocIn[i-1] != '\\' && !iq)
								uLineEnd = i;
							}	// for

						}	// if
					else
						uLineBegin = uLineEOL;

					// If any characters were stored for line, time to pause scan
					if (uLineBegin != uLineEOL) break;
					}	// if
				}	// while
			}	// if

		// Valid char ?
		if (hr == S_OK)
			{
			// Next valid char
			if (!bSkipW || !ISSPACE(paDocIn[uLineBegin]))
				*pIdx = (!bSkipW && bNewLine) ? uWhite : uLineBegin++;
			else
				++uLineBegin;
			}	// if
		}	// while

	return hr;
	}	// nextChar

HRESULT PersistTxt :: nextString ( U32 *pIdxB, U32 *pIdxE, U32 uIdxB )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Retrieves the next string from the document stream.  A string
	//			is defined in quotes or the next group of continuous,
	//			non-whitespace characters.
	//
	//	PARAMETERS
	//		-	pIdxB,pIdxE will receive the begin and end index of string
	//			(pIdxE will point to first character NOT in string (i.e. NULL))
	//		-	uIdxB specifies an optional alternative 'begin index'
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	U32			uEnd;
	bool			bQuoted;
	char			p,pp;

	// Next character if not obtained
	if (uIdxB == MAXDWORD)
		hr = nextChar ( pIdxB, true );
	else
		*pIdxB = uIdxB;

	// First character of string a quote ?  This assumes entire string is
	// enclosed in quotes.
	CCLOK ( bQuoted = (paDocIn[*pIdxB] == '"'); )

	// Copy string into provided buffer.  Supports 'C-like' backslash sequences
	// so make sure an 'end' quote is not proceeded by a backslash.
	if (hr == S_OK)
		{
		CCLOK ( *pIdxE = *pIdxB; )
		CCLOK ( p=pp= ' '; )
		while (hr == S_OK)
			{
			// End of string ?
			if (	(!bQuoted && ISSPACE(paDocIn[*pIdxE])) )
				break;

			// Otherwise move to next character
			else
				{
				// Previous characters update
				pp = p;
				p	= paDocIn[*pIdxE];

				// Next character
				if ((hr = nextChar ( &uEnd, false )) == S_OK)
					{
					*pIdxE = uEnd;

					// End of 'quote' ?
					if (bQuoted && paDocIn[*pIdxE] == '"' && (p != '\\' || pp == '\\'))
						bQuoted = false;
					}	// if
				else
					(*pIdxE)++;
				}	// else

			}	// while

		}	// if

	return (hr == S_OK || (*pIdxB != *pIdxE)) ? S_OK : hr;
	}	// nextString

HRESULT PersistTxt :: save ( IByteStream *pStm, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERsave
	//	FROM		IStreamPersist
	//
	//	PURPOSE
	//		-	Save a value to the specified stream.
	//
	//	PARAMETERS
	//		-	pStm is the destination stream
	//		-	v contains the value to save
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Stream interface
	CCLTRYE	( (pStm != NULL), E_INVALIDARG );
	CCLTRY	( _QI(pStm,IID_IByteStream,&pStmOut) );

	// Save object
	CCLOK		( uDepth = 0; )
	CCLTRY	( valueSave ( v ) );

	// Clean up
	_RELEASE(pStmOut);

	return hr;
	}	// save

HRESULT PersistTxt :: toValue ( U32 uIdxB, U32 uIdxE, ADTVALUE &Value )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the given string into a value.
	//
	//	PARAMETERS
	//		-	uIdxB,uIdxE are the begin and end indicies of the string
	//			(See comments for 'nextString' in relation to end index)
	//		-	Value will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	bool			bStm	= false;
	bool			bQuote;
	char			*pColon,*pQuote,*paStart,ct,clast;
	VALUETYPE	vtype;

	// Temporarily terminate current string, restored later
	clast				= paDocIn[uIdxE];
	paDocIn[uIdxE]	= '\0';

	// If string is quoted, skip over first quote
	paStart	= &(paDocIn[uIdxB]);
	if ( (bQuote = (paDocIn[uIdxB] == '"')) )
		++paStart;

	// If quoted, locate closing quote
	if (bQuote)	pQuote = strrchr ( &(paDocIn[uIdxB]), '"' );
	else			pQuote = NULL;

	// Remove quote and see if data type is specified by ':'
	pColon	= NULL;
	if (pQuote)
		{
		pQuote[0]	= '\0';								// Terminate string
		if (pQuote[1] == ':')							// Next char colon ?
			pColon = &(pQuote[1]);						// Data type specified
		}	// if
	else
		pColon = strrchr ( &(paDocIn[uIdxB]), ':' );	// Data type specified ?

	// Determine value type.  Only count colon if its right after end quote
	if ( pColon != NULL )
		{
		// Determine type from first letter
		ct				= pColon[1];
		if			(ct == 'I' || ct == 'i')	vtype = VTYPE_I4;
		else if	(ct == 'L' || ct == 'l')	vtype = VTYPE_I8;
		else if	(ct == 'B' || ct == 'b')	vtype = VTYPE_BOOL;
		else if	(ct == 'F' || ct == 'f')	vtype = VTYPE_R4;
		else if	(ct == 'D' || ct == 'd')
			{
			if			(pColon[2] == 'A' || pColon[2] == 'a')	vtype = VTYPE_DATE;
			else if	(pColon[2] == 'O' || pColon[2] == 'o')	vtype = VTYPE_R8;
			else															vtype = VTYPE_STR;
			}	// else if
		else if	(ct == 'D' || ct == 'd')	vtype = VTYPE_DATE;
		else if  (	(ct == 'S' || ct == 's') &&
						(pColon[4] == 'E' || pColon[4] == 'e') )
			{
			vtype = VTYPE_UNK;
			bStm	= TRUE;
			}	// else if
		else											vtype = VTYPE_STR;
		}	// if
	else
		vtype = VTYPE_STR;

	if (hr == S_OK)
		{
		switch (vtype)
			{
			case VTYPE_STR :
				{
				U32	i,len = 0;

				// Support 'C-like' escape sequences.  Scan for these.
				for (i = 0;paStart[i] != ('\0');++i)
					{
					// Escape sequence ?
					if (paStart[i] != ('\\'))
						paStart[len++] = paStart[i];
					else
						{
						// Process sequence
						++i;
						switch (paStart[i])
							{
							case ('n') : paStart[len] = ('\n'); break;
							case ('t') : paStart[len] = ('\t'); break;
							case ('v') : paStart[len] = ('\v'); break;
							case ('b') : paStart[len] = ('\b'); break;
							case ('r') : paStart[len] = ('\r'); break;
							case ('a') : paStart[len] = ('\a'); break;
							case ('\\'): paStart[len] = ('\\'); break;
							case ('?') : paStart[len] = ('\?'); break;
							case ('\''): paStart[len] = ('\''); break;
							case ('\"'): paStart[len] = ('\"'); break;
							case ('0') : paStart[len] = ('\0'); break;
							case ('x') :
								// Convert 2-digit hex number to value
								paStart[len] =	((CHAR2HEX(paStart[i+1])) << 4) |
														(CHAR2HEX(paStart[i+2]));

								// Two extra chars. used up
								i += 2;
								break;
							// Not reserved, use straight char.	
							default :
								paStart[len] = paStart[i];
							}	// switch
						// Next char.
						++len;
						}	// else
					}	// for
				paStart[len] = ('\0');

				// Clear incoming value
				adtValue::clear(Value);

				// ASCII to string
				adtString	sStr;
				sStr	= paStart;
				hr		= adtValue::copy ( sStr, Value );
				}	// case VTYPE_STR
				break;
			case VTYPE_I4 :
				{
				adtInt	oInt;
				// Convert based on hex or decimal...
				if (	paStart[0] == WCHAR('0') &&
						(	paStart[1] == WCHAR('x') ||
							paStart[1] == WCHAR('X') ) )
					oInt = (U32)(strtoul ( paStart+2, NULL, 16 ));
				else
					oInt = (U32)(strtoul ( paStart, NULL, 10 ));
				adtValue::copy ( oInt, Value );
				}	// case VTYPE_I4
				break;
			case VTYPE_I8 :
				{
				adtLong	oLong;
				// Convert based on hex or decimal...
				if (	paStart[0] == WCHAR('0') &&
						(	paStart[1] == WCHAR('x') ||
							paStart[1] == WCHAR('X') ) )
					oLong = (U32)(strtoul ( paStart+2, NULL, 16 ));
				else
					oLong = (U32)(strtoul ( paStart, NULL, 10 ));
				adtValue::copy ( oLong, Value );
				}	// case VTYPE_I8
				break;
			case VTYPE_BOOL :
				{
				adtBool	oBool;
				// TRUE/FALSE
				oBool = (paStart[0] == 'T' || paStart[0] == 't') ? true : false;
				adtValue::copy ( oBool, Value );
				}	// case VTYPE_BOOL
				break;
			case VTYPE_R4 :
				{
				// Convert
				adtFloat	oFlt;
				SSCANF ( paStart, "%f", &(oFlt.vflt) );
				adtValue::copy ( oFlt, Value );
				}	// case VTYPE_R4
				break;
			case VTYPE_R8 :
				{
				// Convert
				adtDouble	oDbl;
				SSCANF ( paStart, "%lf", &(oDbl.vdbl) );
				adtValue::copy ( oDbl, Value );
				}	// case VTYPE_R8
				break;
			case VTYPE_DATE :
				{
				// Remove colon 
				CCLOK ( *pColon = '\0'; )

				// Convert
				adtString	strDate(paStart);
				CCLTRY ( adtDate::fromString ( strDate, Value ) );

				// Restore colon
				CCLOK ( *pColon = ':'; )
				}	// VTYPE_DATE
				break;
			case VTYPE_UNK :
				// Streams will be decoded and placed into memory.
				if (bStm)
					{
					IByteStream	*pStm	= NULL;
					BYTE			b,h,l;

					// Create a stream object to receive the data
					CCLTRY ( COCREATE(L"Io.StmMemory",IID_IByteStream,&pStm) );

					// Decode the stream
					for (int i = 0;hr == S_OK && paStart[i] != '\0' && paStart[i] != ':';)
						{
						// Encoding is '\xYY' where YY is the hex value
						CCLTRYE ( paStart[i++] == '\\', E_UNEXPECTED );
						CCLTRYE ( paStart[i++] == 'x', E_UNEXPECTED );

						// Decode
						CCLOK ( h = CHAR2HEX(paStart[i]);)
						CCLOK ( l = CHAR2HEX(paStart[i+1]);)
						CCLOK ( b = ((h << 4) | l); )
						CCLOK ( i += 2; )

						// Send to stream
						CCLTRY ( pStm->write ( &b, 1, NULL ) );
						}	// for

					// Reset position of stream
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );

					// Finalize value
					CCLTRY ( adtValue::copy ( adtIUnknown(pStm), Value ) );

					// Clean up
					_RELEASE(pStm);
					}	// if
				break;
			default :
				dbgprintf ( L"SysParserXML::childLoad:Unknown value type\r\n" );
				hr = E_NOTIMPL;
			}	// switch
		}	// if

	// Restore terminated character
	paDocIn[uIdxE]	= clast;

	return hr;
	}	// toValue

HRESULT PersistTxt :: valueLoad ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to load the next child from the stream.
	//
	//	PARAMETERS
	//		-	oVal is the child object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr				= S_OK;
	adtString			sStr,sTo;
	U32					idxC,idxB,idxE;

	// Setup
	adtValue::clear ( v );

	// Obtain next non-whitespace character
	CCLTRY ( nextChar ( &idxC ) );

	// Process new 'object'
	if (hr == S_OK)
		{
		switch (paDocIn[idxC])
			{
			// Graph.  Must be first item in stream.
			case '@' :
				{
				// Active graph
				CCLTRYE ( (pGraph == NULL), E_UNEXPECTED );

				// Create a context to hold graph information.
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pGraph ) );

				// Node dictionary
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pNodes ) );
				CCLTRY ( pGraph->store ( adtStringSt(L"Node"), adtIUnknown(pNodes) ) );

				// Subgraph dictionary
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pSubs ) );
				CCLTRY ( pGraph->store ( adtStringSt(L"Subgraph"), adtIUnknown(pSubs) ) );

				// Connnection dictionary
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pConns ) );
				CCLTRY ( pGraph->store ( adtStringSt(L"Connection"), adtIUnknown(pConns) ) );

				// Name list
				CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pNames ) );
				CCLTRY ( pGraph->store ( adtStringSt(L"Names"), adtIUnknown(pNames) ) );

				// Load children of graph
				adtValue	vC;
				while (hr == S_OK && valueLoad ( vC ) == S_OK)
					{}

				// The result is the graph
				if (hr == S_OK)
					{
					v.vtype	= VTYPE_UNK;
					v.punk	= pGraph;
					_ADDREF(v.punk);
					}	// if

				// Sanity check for graph
//				CCLTRY ( pGraph->iterate ( &pIt ) );
				// Check for duplicates
//				if (hr == S_OK && pNodes->load ( sName, vCheck ) == S_OK)
//					dbgprintf ( L"PersistTxt::childLoad:WARNING! Duplicate node name : %s\r\n", (PCWSTR)sName );

				// Clean up
				_RELEASE(pNames);
				_RELEASE(pConns);
				_RELEASE(pSubs);
				_RELEASE(pNodes);
				_RELEASE(pGraph);
				}	// '@'
				break;

			// Node.  Syntax $ <Node name> <Behaviour class> { <additional properties> }
			case '$' :
				{
				IDictionary	*pCtxNode = NULL;
				adtString	sName,sBehave;
				adtValue		vCheck,vC;

				// State check
				CCLTRYE ( (pNodes != NULL), E_UNEXPECTED );

				// Next string is the node name
				CCLTRY ( nextString ( &idxB, &idxE ) );
				CCLOK  ( DOC2W(idxB,idxE,paDocIn,sName); )

				// Next string is the behaviour class
				CCLTRY ( nextString ( &idxB, &idxE ) );
				CCLOK  ( DOC2W(idxB,idxE,paDocIn,sBehave); )

				// Child of node is a context with possible node properties
				CCLTRY ( valueLoad ( vC ) );

				// Properties
				CCLTRYE( vC.vtype == VTYPE_UNK, E_UNEXPECTED );
				CCLTRY ( _QISAFE(vC.punk,IID_IDictionary,&pCtxNode) );

				// Check for duplicate node names
				if (hr == S_OK && (pNodes->load ( sName, vCheck ) == S_OK || pSubs->load ( sName, vCheck ) == S_OK))
					dbgprintf ( L"PersistTxt::childLoad:WARNING! Duplicate node name : %s\r\n", (PCWSTR)sName );

				// Store the node name and behaviour under the node properties
				CCLTRY ( pCtxNode->store ( adtString(STR_NSPC_NAME),		sName ) );
				CCLTRY ( pCtxNode->store ( adtString(STR_NSPC_BEHAVE),	sBehave ) );

				// Store under the next available index
				CCLTRY ( pNodes->store ( sName, adtIUnknown(pCtxNode) ) );
//				CCLTRY ( pNodes->size ( &sz ) );
//				CCLTRY ( pNodes->store ( adtInt(sz+1), adtIUnknown(pCtxNode) ) );

				// Name encountered
				CCLTRY ( pNames->write ( sName ) );

				// Clean up
				_RELEASE(pCtxNode);
				}	// '$'
				break;

			// Edge.  Syntax ! <From> <To>
			case '!' :
				{
				IDictionary	*pCtxConn = NULL;
				U32			sz;

				// State check
				CCLTRYE ( pConns != NULL, E_UNEXPECTED );

				// Create dictionary for connection
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pCtxConn ) );

				// From/to.  Watch for quoted connection names.
				CCLTRY ( nextString ( &idxB, &idxE ) );
				if (hr == S_OK)
					{
					if (paDocIn[idxB] == '"')
						{
						DOC2W(idxB+1,idxE-1,paDocIn,sStr);
						}	// if
					else
						{
						DOC2W(idxB,idxE,paDocIn,sStr);
						}	// else if
					}	// if
				CCLTRY ( nextString ( &idxB, &idxE ) );
				if (hr == S_OK)
					{
					if (paDocIn[idxB] == '"')
						{
						DOC2W(idxB+1,idxE-1,paDocIn,sTo);
						}	// if
					else
						{
						DOC2W(idxB,idxE,paDocIn,sTo);
						}	// else if
					}	// if

				// Store connection pair
				CCLTRY ( pCtxConn->store ( adtStringSt(L"From"), sStr ) );
				CCLTRY ( pCtxConn->store ( adtStringSt(L"To"), sTo ) );

				// Store under the next available index
				CCLTRY ( pConns->size ( &sz ) );
				CCLTRY ( pConns->store ( adtInt(sz+1), adtIUnknown(pCtxConn) ) );

				// Clean up
				_RELEASE(pCtxConn);
				}	// '!'
				break;

			// Subgraph.  Syntax # <Subgraph name> <Location> { <additional properties> }
			case '#' :
				{
				IDictionary	*pCtxSub = NULL;
				adtString	sName;
				adtValue		vC;

				// State check
				CCLTRYE ( (pSubs != NULL), E_UNEXPECTED );

				// Next string is the subgraph name.  Watch for quoted subgraphs.
				CCLTRY ( nextString ( &idxB, &idxE ) );
				if (hr == S_OK)
					{
					if (paDocIn[idxB] == '"')
						{
						DOC2W(idxB+1,idxE-1,paDocIn,sName);
						}	// if
					else
						{
						DOC2W(idxB,idxE,paDocIn,sName);
						}	// else if
					}	// if

				// Next string is the location of the subgraph definition
				CCLTRY ( nextString ( &idxB, &idxE ) );
				CCLOK  ( DOC2W(idxB,idxE,paDocIn,sStr); )

				// Child is a context with possible properties
				CCLTRY ( valueLoad ( vC ) );

				// Store subgraph information in context
				CCLTRYE( vC.vtype == VTYPE_UNK, E_UNEXPECTED );
				CCLTRY ( _QISAFE(vC.punk,IID_IDictionary,&pCtxSub) );
				CCLTRY ( pCtxSub->store ( adtString(STR_NSPC_NAME),	sName ) );
				CCLTRY ( pCtxSub->store ( adtString(STR_NSPC_LOC),		sStr ) );

				// Store under the next available index
				CCLTRY ( pSubs->store ( sName, adtIUnknown(pCtxSub) ) );
//				CCLTRY ( pSubs->size ( &sz ) );
//				CCLTRY ( pSubs->store ( adtInt(sz+1), adtIUnknown(pCtxSub) ) );

				// Name encountered
				CCLTRY ( pNames->write ( sName ) );

				// Clean up
				_RELEASE(pCtxSub);
				}	// '#'
				break;

			// Dictionary
			case '{' :
				{
				IDictionary	*pDct	= NULL;
				adtValue		vK,vV;

				// Create dictionary to receive objects
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );

				// Load key/value pairs
				while (	hr == S_OK && 
							valueLoad ( vK ) == S_OK &&
							valueLoad ( vV ) == S_OK )
					hr = pDct->store ( vK, vV );

				// Result is the dictionary
				if (hr == S_OK)
					{
					v.vtype	= VTYPE_UNK;
					v.punk	= pDct;
					_ADDREF(v.punk);
					}	// if

				// Clean up
				_RELEASE(pDct);
				}	// '{'
				break;
			// End of dictionary
			case '}' :
				hr = S_FALSE;
				break;

			// List
			case '(' :
				{
				IList		*pLst	= NULL;
				adtValue	vV;

				// Create list to receive objects
				CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLst ) );

				// Load key/value pairs
				while (hr == S_OK && valueLoad ( vV ) == S_OK)
					hr = pLst->write ( vV );

				// Result is the dictionary
				if (hr == S_OK)
					{
					v.vtype	= VTYPE_UNK;
					v.punk	= pLst;
					_ADDREF(v.punk);
					}	// if

				// Clean up
				_RELEASE(pLst);
				}	// '('
				break;

			// End of list
			case ')' :
				hr = S_FALSE;
				break;

			// Default case is no recognized characters so next string
			// must be a value (string) or property (string=string)
			default :
				// Result
				CCLTRY ( nextString ( &idxB, &idxE, idxC ) );
				CCLTRY ( toValue ( idxB, idxE, v ) );
			}	// switch
		}	// if

	return hr;
	}	// valueLoad

HRESULT PersistTxt :: valueSave ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to load the next child from the stream.
	//
	//	PARAMETERS
	//		-	oVal is the child object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IByteStream	*pStm			= NULL;
	U32			i;
	U8				b;

	// NOTE: Cannot save graphs yet.  Used for dictionary,lists and values.

	// Object ?
	if (v.vtype == VTYPE_UNK && v.punk != NULL)
		{
		IDictionary	*pDct	= NULL;
		IList			*pLst	= NULL;

		// Dictionary ?
		if (	_QI(v.punk,IID_IDictionary,&pDct)	== S_OK ||
				_QI(v.punk,IID_IList,&pLst)			== S_OK)
			{
			IIt		*pIt	= NULL;
			adtValue	vK,vV;

			// For readability...
			for (i = 0,b = '\t';hr == S_OK && i <= uDepth;++i)
				hr = pStmOut->write ( &b, 1, NULL );

			// Start marker
			CCLOK	 ( b = (pDct != NULL) ? '{' : '('; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );
			CCLOK	 ( b = '\n'; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );

			// Persist object
			CCLOK  ( ++uDepth; )
			if (hr == S_OK)
				hr = (pLst != NULL) ? pLst->iterate ( &pIt ) : pDct->keys ( &pIt );
			if (hr == S_OK && pLst != NULL)
				{
				// Save values
				while (hr == S_OK && pIt->read ( vV ) == S_OK)
					{
					// Save next value
					CCLTRY ( valueSave ( vV ) );
					pIt->next();
					}	// while
				}	// if
			else if (hr == S_OK && pDct != NULL)
				{
				// Save key/value pairs
				while (hr == S_OK && pIt->read ( vK ) == S_OK)
					{
					// Save next pair
					if (pDct->load ( vK, vV ) == S_OK)
						{
						CCLTRY ( valueSave ( vK ) );
						CCLTRY ( valueSave ( vV ) );
						}	// if
					pIt->next();
					}	// while
				}	// if
			CCLOK  ( --uDepth; )

			// For readability...
			for (i = 0,b = '\t';hr == S_OK && i <= uDepth;++i)
				hr = pStmOut->write ( &b, 1, NULL );

			// End marker
			CCLOK	 ( b = (pDct != NULL) ? '}' : ')'; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );
			CCLOK	 ( b = '\n'; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pLst);
			_RELEASE(pDct);
			}	// if

		// Allow byte streams.  They will be encoded with hex characters.
		else if (_QI(v.punk,IID_IByteStream,&pStm) == S_OK)
			{
			U64	pos	= 0;
			U64	avail	= 0;
			BYTE	b;
			char	cbfr[21];

			// Compute available bytes
			CCLTRY ( pStm->available ( &avail ) );

			// For readability...
			for (i = 0,b = '\t';hr == S_OK && i <= uDepth;++i)
				hr = pStmOut->write ( &b, 1, NULL );

			// Encode stream into destination
			for (U32 i = 0;hr == S_OK && i < avail;++i)
				{
				// Next byte
				CCLTRY ( pStm->read ( &b, 1, NULL ) );

				// Encode
				if (hr == S_OK)
					{
					cbfr[0] = WCHAR('\\');
					cbfr[1] = WCHAR('x');
					cbfr[2] = HEX2CHAR( ((b>>4) & 0xf) );
					cbfr[3] = HEX2CHAR( ((b>>0) & 0xf) );

					// Write 'character'
					CCLTRY ( pStmOut->write ( cbfr, 4, NULL ) );
					}	// if

				}	// for

			// Restore stream to original state
			CCLTRY ( pStm->seek ( pos, STREAM_SEEK_SET, NULL ) );

			// Add identifier for stream
			CCLTRY ( pStmOut->write ( ":Stream", 7, NULL ) );

			// For readability...
			CCLOK	 ( b = '\n'; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );

			// Space after value
			CCLOK  ( b = ' '; )
			CCLTRY ( pStmOut->write ( &b, 1, NULL ) );

			// Clean up
			_RELEASE(pStm);
			}	// if

		else
			hr = E_NOTIMPL;
		}	// if

	// Value
	else if (v.vtype != VTYPE_UNK && v.vtype != VTYPE_EMPTY)
		{
		// For readability...
		for (i = 0,b = '\t';hr == S_OK && i < uDepth;++i)
			hr = pStmOut->write ( &b, 1, NULL );

		// Save
		CCLTRY ( writeValue ( v ) );

		// For readability...
		CCLOK	 ( b = '\n'; )
		CCLTRY ( pStmOut->write ( &b, 1, NULL ) );
		}	// else if

	return hr;
	}	// valueSave

HRESULT PersistTxt :: writeValue ( const ADTVALUE &Value )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Saves a value to the document stream.
	//
	//	PARAMETERS
	//		-	Value is the value to save
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	const char	*type		= NULL;
	char			cbfr[21];
	bool			nq;
	adtString	vStr;
	U8				uc;

	// Convert value to its string representation
	CCLTRY ( adtValue::toString ( Value, vStr ) );

	// Handle escape sequences/binary data
	if (hr == S_OK)
		{
		// Does string need quotes ?
		nq = (wcscspn ( vStr, L" {}()$#!%:" ) < wcslen(vStr) || !vStr.length());
		uc = '\"';
		if (hr == S_OK && nq) hr = pStmOut->write ( &uc, 1, NULL );

		// Write string to buffer
		U32	srcidx = 0;
		while (hr == S_OK && vStr.at(srcidx) != WCHAR('\0'))
			{
			U32 dstidx = 0;

			// ASCII character
			CCLOK ( uc = (vStr.at(srcidx) & 0xff); )

			// Ok ?
			if (hr != S_OK) continue;

			// Assume an escape character
			cbfr[dstidx] = WCHAR('\\');
			switch (uc)
				{
				case WCHAR('\n') : cbfr[++dstidx] = WCHAR('n'); ++dstidx; break;
				case WCHAR('\t') : cbfr[++dstidx] = WCHAR('t'); ++dstidx; break;
				case WCHAR('\v') : cbfr[++dstidx] = WCHAR('v'); ++dstidx; break;
				case WCHAR('\b') : cbfr[++dstidx] = WCHAR('b'); ++dstidx; break;
				case WCHAR('\r') : cbfr[++dstidx] = WCHAR('r'); ++dstidx; break;
				case WCHAR('\a') : cbfr[++dstidx] = WCHAR('a'); ++dstidx; break;
				case WCHAR('\\') : cbfr[++dstidx] = WCHAR('\\'); ++dstidx; break;
				case WCHAR('\?') : cbfr[++dstidx] = WCHAR('\?'); ++dstidx; break;
				case WCHAR('\'') : cbfr[++dstidx] = WCHAR('\''); ++dstidx; break;
				case WCHAR('\"') : cbfr[++dstidx] = WCHAR('\"'); ++dstidx; break;
				case WCHAR('%')  : cbfr[++dstidx] = WCHAR('%'); ++dstidx; break;
				default :
					// 'Normal' character ?
					if (	uc >= 32 && uc <= 126 )
						cbfr[dstidx++] = (U8)(vStr.at(srcidx) & 0xff);
					// Hex
					else
						{
						// Escape code for hex value
						cbfr[++dstidx] = WCHAR('x');
						++dstidx;

						// Hex value
						cbfr[dstidx++] = HEX2CHAR( ((uc>>4) & 0xf) );
						cbfr[dstidx++] = HEX2CHAR( ((uc>>0) & 0xf) );
						}	// else
				}	// switch

			// Write 'character'
			CCLTRY ( pStmOut->write ( cbfr, dstidx, NULL ) );

			// Next source
			++srcidx;
			}	// while

		// Does string need quotes ?
		CCLOK ( uc = '\"'; )
		if (hr == S_OK && nq) hr = pStmOut->write ( &uc, 1, NULL );
		}	// if

	// Type
	if (hr == S_OK)
		type =	(adtValue::type(Value) == VTYPE_I4)		? "Integer" :
					(adtValue::type(Value) == VTYPE_I8)		? "Long"		:
					(adtValue::type(Value) == VTYPE_R4)		? "Float"	:
					(adtValue::type(Value) == VTYPE_R8)		? "Double"	:
					(adtValue::type(Value) == VTYPE_DATE)	? "Date"		:
					(adtValue::type(Value) == VTYPE_BOOL)	? "Boolean" :
					(adtValue::type(Value) == VTYPE_STR)	? "String"	: NULL;
	CCLTRYE	( (type != NULL), E_NOTIMPL );

	// Default type is string, output other
	if (hr == S_OK && adtValue::type(Value) != VTYPE_STR)
		{
		CCLOK  ( uc = ':'; )
		CCLTRY ( pStmOut->write ( &uc, 1, NULL ) );
		CCLTRY ( pStmOut->write ( type, (U32)strlen(type), NULL ) );
		}	// if

	// Space after value
	CCLOK  ( uc = ' '; )
	CCLTRY ( pStmOut->write ( &uc, 1, NULL ) );

	return hr;
	}	// writeValue

/*					// Valid line ? Search until the first non-whitespace or an unquoted comment 
					// character is encountered.
					int	ps	= 0;
					int	iq	= 0;
					while (	hr == S_OK &&
								uLineBegin < uLineEOL &&
								ISSPACE(paDocIn[uLineBegin]) )
						{
						// Quote ?
						if (!ps && paDocIn[uLineBegin] == '"')
							iq = (iq == 0) ? 1 : 0;

						// Unquoted comment ?
						if (!iq && paDocIn[uLineBegin] != '%')
							break;

						// Previous character a slash ?
						ps = (paDocIn[uLineBegin] == '\\');

						++uLineBegin;
						}	// while
*/
