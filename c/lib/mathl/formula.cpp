////////////////////////////////////////////////////////////////////////
//
//									FORMULA.CPP
//
//					Implementation of the formula node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>
#include <math.h>

// Whitespace
#define	ISSPACE(a)													\
			( ((a) >= 0x09 && (a) <= 0x0D) || ((a) == 0x20) )

// Table of operator descriptors
static struct stOp
	{
	const WCHAR	*wOp;										// Operator
	int			iPri;										// Priority
	int			iParam;									// Parameter count
	int			iId;										// Operator Id
	} ops[] = 
	{
	// Special characters, position in list expected to not change in code below
	{ L"(", 10, 0, 0 },
	{ L")", 10, 0, 0 },
	{ L",", 0, 0, 0 },

	// Single char operators
	{ L"*", 9, 2, MATHOP_MUL },
	{ L"/", 9, 2, MATHOP_DIV },
	{ L"%", 9, 2, MATHOP_MOD },
	{ L"+", 8, 2, MATHOP_ADD },
	{ L"-", 8, 2, MATHOP_SUB },

	// Functions.  Highest priority since they must be evaluated first
	{ L"abs", 10, 1, MATHOP_ABS },
	{ L"cos", 10, 1, MATHOP_COS },
	{ L"sin", 10, 1, MATHOP_SIN },
	{ L"tan", 10, 1, MATHOP_TAN },
	{ L"acos", 10, 1, MATHOP_ACOS },
	{ L"asin", 10, 1, MATHOP_ASIN },
	{ L"atan", 10, 1, MATHOP_ATAN },
	{ L"dot", 10, 2, MATHOP_DOT },
	{ L"cross", 10, 2, MATHOP_CROSS },
	{ L"norm", 10, 1, MATHOP_NORM },
	{ L"ceil", 10, 1, MATHOP_CEIL },
	{ L"floor", 10, 1, MATHOP_FLOOR },
	{ L"sqrt", 10, 1, MATHOP_SQRT },
	{ L"min", 10, 2, MATHOP_MIN },
	{ L"max", 10, 2, MATHOP_MAX },

	// Casting
	{ L"int", 10, 1, MATHOP_INT },
	{ L"long", 10, 1, MATHOP_LONG },
	{ L"float", 10, 1, MATHOP_FLOAT },
	{ L"double", 10, 1, MATHOP_DOUBLE },
	{ L"date", 10, 1, MATHOP_DATE },
	{ L"string", 10, 1, MATHOP_STRING },

	// End of list
	{ L"", 0, 0 }
	};

Formula :: Formula ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pLstPost		= NULL;
	pVals			= NULL;
	pRcps			= NULL;
	pStkEval		= NULL;
	pItStkEval	= NULL;
	}	// Formula

HRESULT Formula :: eval ( IList *pLst, IDictionary *pV, ADTVALUE &vRes )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to evalulate a formula.
	//
	//	PARAMETERS
	//		-	pLst is the postfix list of operands and operators
	//		-	pV contains the values for the operands
	//		-	vRes will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	IIt		*pItSym	= NULL;
	adtValue	vL;

	// Initialize result
	CCLOK ( adtValue::clear ( vRes ); )

	// Iterate symbols
	CCLTRY ( pItStkEval->begin() );
	CCLTRY ( pLst->iterate ( &pItSym ) );
	while (hr == S_OK && pItSym->read ( vL ) == S_OK)
		{
		// Operator
		if (vL.vtype == VTYPE_I4)
			{
			adtInt	iOp(vL);
			adtValue	vP0,vP1;

			// NOTE: For functions that are order dependent the operands
			// on the stack will be popped in the reverse order.
			if (hr == S_OK && ops[iOp].iParam > 1)
				{
				// Next symbol
				CCLTRY ( pItStkEval->read ( vP1 ) );
				CCLOK  ( pItStkEval->next(); )

				// Resolve into value (ok if it fails in case of string operations)
				if (hr == S_OK && adtValue::type(vP1) == VTYPE_STR)
					pV->load ( vP1, vP1 );
				}	// if
			if (hr == S_OK && ops[iOp].iParam > 0)
				{
				// Next symbol
				CCLTRY ( pItStkEval->read ( vP0 ) );
				CCLOK  ( pItStkEval->next(); )

				// Resolve into value (ok if it fails in case of string operations)
				if (hr == S_OK && adtValue::type(vP0) == VTYPE_STR)
					pV->load ( vP0, vP0 );
				}	// if

			// Process
			if (hr == S_OK)
				{
				switch ( ops[iOp].iId )
					{
					// Binary
					case MATHOP_ADD :
					case MATHOP_SUB :
					case MATHOP_MUL :
					case MATHOP_DIV :
					case MATHOP_DOT :
					case MATHOP_MOD :
					case MATHOP_CROSS :
					case MATHOP_MIN :
					case MATHOP_MAX :
						CCLTRY ( mathBinary ( ops[iOp].iId, vP0, vP1, vRes ) );
						break;

					// Unary
					case MATHOP_ABS :
					case MATHOP_COS :
					case MATHOP_SIN :
					case MATHOP_TAN :
					case MATHOP_ACOS :
					case MATHOP_ASIN :
					case MATHOP_ATAN :
					case MATHOP_NORM :
					case MATHOP_CEIL :
					case MATHOP_FLOOR :
					case MATHOP_SQRT :
					case MATHOP_INT :
					case MATHOP_LONG :
					case MATHOP_FLOAT :
					case MATHOP_DOUBLE :
					case MATHOP_DATE :
					case MATHOP_STRING :
						CCLTRY ( mathUnary ( ops[iOp].iId, vP0, vRes ) );
						break;

					default :
						hr = E_NOTIMPL;
					}	// switch
				}	// if

			// Place result on stack
			CCLTRY ( pStkEval->write ( vRes ) );
			}	// if

		// Operand
		else
			{
			// Place operands on evaluation stack
			CCLTRY ( pStkEval->write ( vL ) );
			}	// else

		// Next symbol
		pItSym->next();
		}	// while

	// Last value on stack is the result
	CCLTRY ( pItStkEval->read ( vRes ) );

	return hr;
	}	// eval

HRESULT Formula :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;
	adtValue	vL;

	// Attach
	if (bAttach)
		{
		// Create dictionary for values
		CCLTRY ( COCREATE(L"Adt.Dictionary",IID_IDictionary,&pVals) );

		// Create a dictionary to keep track of receptors
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pRcps));

		// Create evaluation stack
		CCLTRY ( COCREATE(L"Adt.Stack",IID_IList,&pStkEval) );
		CCLTRY ( pStkEval->iterate ( &pItStkEval ) );

		// Provided formula
		if (	pnDesc->load ( adtStringSt(L"Formula"), vL ) == S_OK &&
				(strForm = vL).length() )
			{
			IList	*pSym		= NULL;
			IIt	*pIt		= NULL;

			// Break into list of symbols
			CCLTRY ( symbols ( strForm, &pSym ) );

			// Convert into a postfix list
			CCLTRY ( post ( pSym, &pLstPost ) );

			// Create a receptor for each operand
			CCLTRY ( pLstPost->iterate ( &pIt ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK)
				{
				IReceptor	*pR	= NULL;
				adtValue		vTst;

				// A string is an operand and unique
				if (adtValue::type(vL) == VTYPE_STR && pnLoc->load ( vL, vTst ) != S_OK)
					{
					adtString	strRecep(vL);

					// Add a receptor of the specified name
//					dbgprintf (L"Formula::onAttach:Connection:%p:%s:%s\r\n", this, (LPCWSTR)strnName, (LPCWSTR)strRecep );
					CCLTRY ( pnSpc->connection ( pnLoc, strRecep, L"Receptor", this, &pR ) );

					// Associate name with key
					CCLTRY ( pRcps->store ( adtLong((U64)pR), strRecep ) );

					// Attributes can contain default values for operands
					if (hr == S_OK && pnDesc->load ( strRecep, vL ) == S_OK)
						hr = pVals->store ( strRecep, vL );
					}	// if

				// Next symbol
				pIt->next();
				}	// while

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pSym);
			}	// if

		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pLstPost);
		_RELEASE(pVals);
		_RELEASE(pRcps);
		_RELEASE(pItStkEval);
		_RELEASE(pStkEval);
		}	// else

	return hr;
	}	// onAttach

HRESULT Formula :: post ( IList *pSym, IList **ppP )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert an infix ordered symbols list into post fix.
	//
	//	PARAMETERS
	//		-	pSym is the symbol list
	//		-	ppP will receive the postfix list
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	IList		*pStk		= NULL;
	IIt		*pItSym	= NULL;
	IIt		*pItStk	= NULL;
	adtValue	vIt;
	int		idx;

	// Create result list
	CCLTRY ( COCREATE(L"Adt.List",IID_IList,ppP) );

	// Create operator stack
	CCLTRY ( COCREATE(L"Adt.Stack",IID_IList,&pStk) );
	CCLTRY ( pStk->iterate ( &pItStk ) );

	// Iterate symbols
	CCLTRY ( pSym->iterate ( &pItSym ) );
	while (hr == S_OK && pItSym->read ( vIt ) == S_OK)
		{
		adtString	strSym(vIt);

		// Operator ?
		for (idx = 0;ops[idx].wOp[0] != '\0';++idx)
			if (!WCASECMP(strSym,ops[idx].wOp))
				break;
		if (ops[idx].wOp[0] != '\0')
			{
			// Current operator index
			adtInt	iOp(idx);

			// Right paren ?
			if (idx == 1)
				{
				// Move operators from stack to post list until the matching
				// left paren is encountered.
				while (hr == S_OK && pItStk->read ( vIt ) == S_OK)
					{
					adtInt	iOpStk(vIt);

					// Operator will be removed from stack no matter what
					pItStk->next();

					// Do not write left parens or commas to output
					if (iOpStk > 2)
						hr = (*ppP)->write ( iOpStk );

					// For left paren, stop search
					if (iOpStk == 0)
						break;
					}	// while
				}	// if

			// Other
			else
				{
				// Pop symbols from stack and place into post list 
				// until a lower priority operator is encountered
				while (hr == S_OK && pItStk->read ( vIt ) == S_OK)
					{
					adtInt	iOpStk(vIt);

					// Left parens on not removed from the stack
					// except by a right paren
					if (iOpStk == 0)
						break;

					// If operator priority on the stack less than
					// the current operator, then end of search
					if (ops[iOpStk].iPri <= ops[iOp].iPri)
						break;

					// Remove from stack and place on output list
					CCLOK  ( pItStk->next(); )
					CCLTRY ( (*ppP)->write ( iOpStk ) );
					}	// while

				// Place current operator on stack (use index as operator identifier from now on)
				CCLTRY ( pStk->write ( iOp ) );
				}	// else
			}	// if

		// Operands
		else
			{
			// Write operands to postfix list
			CCLTRY ( (*ppP)->write ( strSym ) );
			}	// else

		// Next symbol
		pItSym->next();
		}	// while

	// Remaining operators on stack go to output list
	while (hr == S_OK && pItStk->read ( vIt ) == S_OK)
		{
		adtInt	iOpStk(vIt);

		// Operator will be removed from stack no matter what
		pItStk->next();

		// Do not write left parens or commas to output
		if (iOpStk > 2)
			hr = (*ppP)->write ( iOpStk );

		// For left parent, stop search
		if (iOpStk == 0)
			break;
		}	// while

	// Clean up
	_RELEASE(pItSym);
	_RELEASE(pItStk);
	_RELEASE(pStk);

	// DEBUG
/*
	IIt		*pIt	= NULL;
	CCLTRY ( (*ppP)->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( vIt ) == S_OK)
		{
		adtString	str;
		if (vIt.vtype == VTYPE_I4)
			str = ops[vIt.vint].wOp;
		else if (vIt.vtype == VTYPE_STR)
			str = vIt.pstr;
		dbgprintf ( L"Formula::toPost:%s\r\n", str.pstr );
		pIt->next(); 
		}	// while
	_RELEASE(pIt);
*/
	return hr;
	}	// post

HRESULT Formula :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Evalulate
	if (_RCP(Eval))
		{
		adtValue	vRes;

		// State check
		CCLTRYE  ( pLstPost != NULL, ERROR_INVALID_STATE );

		// Evaluate
		CCLTRY ( eval ( pLstPost, pVals, vRes ) );

		// Result
		if (hr == S_OK)
			_EMT(Eval,vRes );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Operand
	else
		{
		adtValue	vRecep;

		// A value can be fed directly into a receptor name of the desired key.
		// This will store the value in that key.

		// State check
		CCLTRYE ( pVals != NULL, ERROR_INVALID_STATE );

		// Access key name
		CCLTRY ( pRcps->load ( adtLong((U64)pr), vRecep ) );

		// Cache value at key
		CCLTRY ( pVals->store ( vRecep, v ) );
		}	// else

	return hr;
	}	// receive

HRESULT Formula :: symbols ( const WCHAR *pw, IList **ppLst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert incoming infix string into a list of distinct symbols.
	//
	//	PARAMETERS
	//		-	pw is the formula string
	//		-	ppLst will receive the list of symbols
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	WCHAR			wChar[2]	= { '\0', '\0' };
	int			idx;
	const WCHAR	*pws;
	adtString	str;

	// Create list for symbols
	CCLTRY ( COCREATE(L"Adt.List",IID_IList,ppLst) );

	// Continue until end of string
	while (hr == S_OK && *pw != '\0')
		{
		// Skip whitespace
		while (ISSPACE((*pw)))
			++pw;

		// Single character operator ?
		for (idx = 0;ops[idx].wOp[0] != '\0';++idx)
			if (wcslen(ops[idx].wOp) == 1 && *pw == ops[idx].wOp[0])
				break;
		if (ops[idx].wOp[0] != '\0')
			{
			// Add operator as own symbol to list
			wChar[0] = *pw++;
			CCLTRY ( (*ppLst)->write ( (str = wChar) ) );
			}	// if

		// Not single char operator
		else
			{
			// Continue with search for remaining symbol characters
			pws = pw++;
			while (*pw != '\0')
				{
				// For whitespace, search complete
				if (ISSPACE(*pw))
					break;

				// If a single char operator is encountered, end of search for this symbol
				for (idx = 0;ops[idx].wOp[0] != '\0';++idx)
					if (wcslen(ops[idx].wOp) == 1 && *pw == ops[idx].wOp[0])
						break;
				if (ops[idx].wOp[0] != '\0')
					break;

				// Next char
				++pw;
				}	// while

			// Add symbol to list
			CCLTRY ( str.allocate ( (U32)(pw-pws) ) );
			CCLOK  ( str.at(pw-pws) = '\0' ; )
			for (idx = 0;hr == S_OK && pws < pw;++pws,++idx)
				str.at(idx) = *pws;
			CCLTRY ( (*ppLst)->write ( str ) );
			}	// else

		}	// while

	// DEBUG
/*
	IIt		*pIt	= NULL;
	adtValue	vIt;
	CCLTRY ( (*ppLst)->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( vIt ) == S_OK)
		{
		dbgprintf ( L"Formula::symbols:%s\r\n", vIt.pstr );
		pIt->next();
		}	// while
	_RELEASE(pIt);
*/
	return hr;
	}	// symbols

