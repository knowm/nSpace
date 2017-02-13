////////////////////////////////////////////////////////////////////////
//
//									STAT.CPP
//
//				Implementation of the container statistics node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Stat :: Stat ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pCnt	= NULL;
	}	// Stat

void Stat :: destruct ( void )
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
	_RELEASE(pCnt);
	onAttach(false);
	}	// destruct

HRESULT Stat :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		{
		}	// if

	// Detach
	else
		{
		_RELEASE(pCnt);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Stat :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Statistics
	if (_RCP(Fire))
		{
		U32	sz;

		// State check
		CCLTRYE ( pCnt != NULL, ERROR_INVALID_STATE );

		// Count elements
		CCLTRY ( pCnt->size ( &sz ) );

		// Results
		if (hr == S_OK)
			{
			// Count
			_EMT(Count,(iV=sz));

			// Empty
			if (sz == 0)
				_EMT(Empty,(unkV=pCnt));
			else
				_EMT(NotEmpty,(unkV=pCnt));
			}	// if

		}	// if

	// State
	else if (_RCP(Container))
		{
		_RELEASE(pCnt);
		hr = _QISAFE((unkV=v),IID_IContainer,&pCnt);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
