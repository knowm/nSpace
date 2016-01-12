////////////////////////////////////////////////////////////////////////
//
//									ITERATE.CPP
//
//					Implementation of the iterator node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Iterate :: Iterate ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pCnt	= NULL;
	pDct	= NULL;
	pIt	= NULL;
	}	// Iterate

void Iterate :: destruct ( void )
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
	onAttach(false);
	}	// destruct

HRESULT Iterate :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the behaviour is attached to a node.
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Detach
	if (!bAttach)
		{
		_RELEASE(pIt);
		_RELEASE(pCnt);
		_RELEASE(pDct);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Iterate :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

		// Debug
//if (!WCASECMP(this->strnName,L"ConnIt"))
//if (!WCASECMP(this->strnName,L"StkValueIt"))
//	dbgprintf ( L"Hi\r\n" );

	// First/last value
	if (_RCP(First) || _RCP(Last))
		{
		// State check
		CCLTRYE ( pCnt != NULL,	ERROR_INVALID_STATE );

		// Debug
//		if (!WCASECMP(this->strnName,L"NodeUnIt"))
//			dbgprintf ( L"Hi\r\n" );

		// Previous state
		_RELEASE(pIt);

		// Appropriate iterator based on container type
		if (hr == S_OK)
			hr = (pDct != NULL) ? pDct->keys ( &pIt ) : pCnt->iterate ( &pIt );

		// First/Next
		if (hr == S_OK && _RCP(First))
			{
			// Beginning of list
			CCLTRY ( pIt->begin() );

			// Next value
			CCLOK ( receive ( prNext, pl, v ); )
			}	// if

		// Last/Previous
		else if (hr == S_OK && _RCP(Last))
			{
			// End of list
			CCLTRY ( pIt->end() );

			// Previous value
			CCLOK ( receive ( prPrevious, pl, v ); )
			}	// else if

		}	// if

	// Next/previous item
	else if (_RCP(Next) || _RCP(Previous))
		{
		// State check
		CCLTRYE ( pIt != NULL, ERROR_INVALID_STATE );

		// Debug
//		if (!WCASECMP(this->strnName,L"StkValueIt"))
//			dbgprintf ( L"Hi\r\n" );

		// Previous item
		if (hr == S_OK && _RCP(Previous))
			pIt->prev();

		// Read the current item/key
		CCLTRY ( pIt->read ( vKey ) );
		if (hr == S_OK && pDct != NULL)
			hr = pDct->load ( vKey, vValue );

		// Next item
		if (hr == S_OK && _RCP(Next))
			pIt->next();

		// Results
		if (hr == S_OK)
			{
			// Key/value pair
			if (pDct != NULL)	_EMT(Key,vKey);
			if (_RCP(Next))
				_EMT(Next,(pDct != NULL) ? vValue : vKey );
			else
 				_EMT(Previous,(pDct != NULL) ? vValue : vKey );
			}	// if

		// End of list
		else
			{
			if (_RCP(Next))
				_EMT(Last,iZ);
			else
				_EMT(First,iZ);
			}	// else

		}	// else if

	// Container
	else if (_RCP(Container))
		{
		_RELEASE(pIt);
		_RELEASE(pCnt);
		_RELEASE(pDct);
		_QISAFE((unkV=v),IID_IContainer,&pCnt);
		_QISAFE(unkV,IID_IDictionary,&pDct);

		// Appropriate iterator based on container type
		if (hr == S_OK)
			hr =	(pDct != NULL) ? pDct->keys ( &pIt ) : 
					(pCnt != NULL) ? pCnt->iterate ( &pIt ) : S_FALSE;
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

