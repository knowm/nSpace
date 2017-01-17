////////////////////////////////////////////////////////////////////////
//
//									NORMAL.CPP
//
//			Implementation of the point cloud normals node.
//
////////////////////////////////////////////////////////////////////////

#include "pcll_.h"
#include <stdio.h>

// Globals

Normal :: Normal ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	}	// Normal

HRESULT Normal :: onAttach ( bool bAttach )
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
//		adtValue		vL;

		// Defaults
//		if (pnDesc->load ( adtString(L"Xaxis"), vL ) == S_OK)
//			strX = vL;

		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT Normal :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		pclObjRef	*pObj		= NULL;
		adtValue		vL;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pDct->load(adtString(L"pcl"),vL) );
		CCLTRYE	( (pObj = (pclObjRef *)(IUnknown *)adtIUnknown(vL)) != NULL,
									ERROR_INVALID_STATE );
		CCLOK		( pObj->AddRef(); )

		// Normal estimation
		if (hr == S_OK)
			{
			pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal>	n;
			pcl::PointCloud<pcl::PointXYZ>::Ptr						cloud ( &(pObj->cloud) );
			pcl::PointCloud<pcl::Normal>::Ptr						normals ( new pcl::PointCloud<pcl::Normal> );
			pcl::PointCloud<pcl::PointNormal>::Ptr					cloud_with_normals ( new pcl::PointCloud<pcl::PointNormal> );
			pcl::search::KdTree<pcl::PointXYZ>::Ptr				tree ( new pcl::search::KdTree<pcl::PointXYZ>);

			// Execute
			n.setInputCloud ( cloud );
			n.setSearchMethod ( tree );
			n.setKSearch ( 20 );
			n.compute ( *normals );

			// Concatenate the original cloud and normals
			pcl::concatenateFields ( *cloud, *normals, *cloud_with_normals );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pObj));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pObj);
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

