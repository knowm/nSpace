////////////////////////////////////////////////////////////////////////
//
//									CREATE.CPP
//
//				Implementation of the object creation node.
//
////////////////////////////////////////////////////////////////////////

#include "pcll_.h"
#include <stdio.h>

// Globals

Create :: Create ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	}	// Create

HRESULT Create :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		{
		adtValue		vL;

		// Defaults
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			iW = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			iH = vL;
		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
			iSz = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT Create :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Execute
	if (_RCP(Fire))
		{
		bool			bOrg		= false;
		pclObjRef	*pRef		= NULL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( (iW > 0 && iH > 0) || (iSz > 0), ERROR_INVALID_STATE );

		// Reference object for cloud
		CCLTRYE ( (pRef = new pclObjRef()) != NULL, E_OUTOFMEMORY );

		// Cloud object
//		CCLOK		( pRef->cloud.reset ( new pcl::PointCloud<pcl::PointXYZ> ); )
//		CCLTRYE	( pRef->cloud.get() != NULL, E_OUTOFMEMORY );

		// Create cloud of specified size.
		// 'Size' means a count of the specified # of points
		// 'Width' and 'Height' means organized grid.
		CCLOK ( bOrg = (iW > 0 && iH > 0); )
		if (hr == S_OK)
			{
			// Assign sizes
//			pRef->cloud->width	= (bOrg) ? iW : iSz;
//			pRef->cloud->height	= (bOrg) ? iH : 1;
//			pRef->cloud->points.resize ( pRef->cloud->width*pRef->cloud->height );
			pRef->cloud.width	= (bOrg) ? iW : iSz;
			pRef->cloud.height	= (bOrg) ? iH : 1;
			pRef->cloud.points.resize ( pRef->cloud.width*pRef->cloud.height );
			}	// if

		// Wrap in dictionary ?  Other information to pass with cloud object needed
		// by graph ? Size ?
		CCLTRY ( pDct->store ( adtString(L"pcl"), adtIUnknown(pRef) ) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRef);
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Width))
		iW = v;
	else if (_RCP(Height))
		iH = v;
	else if (_RCP(Size))
		iSz = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

