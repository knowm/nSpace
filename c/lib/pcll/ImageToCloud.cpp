////////////////////////////////////////////////////////////////////////
//
//									IMAGETOCLOUD.CPP
//
//			Implementation of the image to point cloud conversion node.
//
////////////////////////////////////////////////////////////////////////

#include "pcll_.h"
#include <stdio.h>

// Globals

// Axis source information
#define	SRC_INV	-1
#define	SRC_X		0
#define	SRC_Y		1
#define	SRC_LST	2

ImageToCloud :: ImageToCloud ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pImg	= NULL;
	pItX	= NULL;
	pItY	= NULL;
	pItZ	= NULL;
	}	// ImageToCloud

float ImageToCloud :: getAxisPt ( U8 src, U32 x, U32 y, IIt *pIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Return the appropriate axis value for the specified position
	//
	//	PARAMETERS
	//		-	src is the axis source (SRC_XXX)
	//		-	x,y is the current image position
	//		-	pIt is the iterator for the axis if needed
	//
	//	RETURN VALUE
	//		Floating point value for axis
	//
	////////////////////////////////////////////////////////////////////////
	float	fRet = 0;

	// Source
	switch (src)
		{
		case SRC_X :
			fRet = x;
			break;
		case SRC_Y :
			fRet = y;
			break;
		case SRC_LST :
			{
			adtValue vL;
			if (pIt->read(vL) == S_OK)
				{
				fRet = adtFloat(vL);
				pIt->next();
				}	// if
			}	// SRC_LST
			break;
		}	// switch

	return fRet;
	}	// onAttach

HRESULT ImageToCloud :: onAttach ( bool bAttach )
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

		// How to assign each axis
		if (pnDesc->load ( adtString(L"Xaxis"), vL ) == S_OK)
			strX = vL;
		if (pnDesc->load ( adtString(L"Yaxis"), vL ) == S_OK)
			strY = vL;
		if (pnDesc->load ( adtString(L"Zaxis"), vL ) == S_OK)
			strZ = vL;

		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pItX);
		_RELEASE(pItY);
		_RELEASE(pItZ);
		_RELEASE(pImg);
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT ImageToCloud :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		pclObjRef	*pObj		= NULL;
		ImageDct		imgDct;
		U8				srcX,srcY,srcZ;
		adtValue		vL;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pDct->load(adtString(L"pcl"),vL) );
		CCLTRYE	( (pObj = (pclObjRef *)(IUnknown *)adtIUnknown(vL)) != NULL,
									ERROR_INVALID_STATE );
		CCLOK		( pObj->AddRef(); )

		// Organized cloud or not
		CCLOK ( bOrg = (	pObj->cloud.isOrganized() == true ); )

		// Source for axis information
		if (hr == S_OK)
			{
			srcX =	(!WCASECMP(strX,L"X"))		? SRC_X :
						(!WCASECMP(strX,L"Y"))		? SRC_Y :
						(!WCASECMP(strX,L"List"))	? SRC_LST : SRC_INV;
			srcY =	(!WCASECMP(strY,L"X"))		? SRC_X :
						(!WCASECMP(strY,L"Y"))		? SRC_Y :
						(!WCASECMP(strY,L"List"))	? SRC_LST : SRC_INV;
			srcZ =	(!WCASECMP(strZ,L"X"))		? SRC_X :
						(!WCASECMP(strZ,L"Y"))		? SRC_Y :
						(!WCASECMP(strZ,L"List"))	? SRC_LST : SRC_INV;

			// Axis assign valid sources ?
			CCLTRYE ( (srcX != SRC_INV && srcY != SRC_INV && srcZ != SRC_INV),
							ERROR_INVALID_STATE );

			// Valid lists ?
			CCLTRYE (	(srcX != SRC_LST || pItX != NULL) &&
							(srcY != SRC_LST || pItY != NULL) &&
							(srcZ != SRC_LST || pItZ != NULL), 
							ERROR_INVALID_STATE );
			}	// if

		// DEBUG

		// Cube test
/*		pObj->cloud.points[0].x = -1;
		pObj->cloud.points[0].y = -1;
		pObj->cloud.points[0].z = -1;

		pObj->cloud.points[1].x = +1;
		pObj->cloud.points[1].y = -1;
		pObj->cloud.points[1].z = -1;

		pObj->cloud.points[2].x = -1;
		pObj->cloud.points[2].y = +1;
		pObj->cloud.points[2].z = -1;

		pObj->cloud.points[3].x = +1;
		pObj->cloud.points[3].y = +1;
		pObj->cloud.points[3].z = -1;

		pObj->cloud.points[4].x = -1;
		pObj->cloud.points[4].y = -1;
		pObj->cloud.points[4].z = -1;

		pObj->cloud.points[5].x = +1;
		pObj->cloud.points[5].y = -1;
		pObj->cloud.points[5].z = -1;

		pObj->cloud.points[6].x = -1;
		pObj->cloud.points[6].y = +1;
		pObj->cloud.points[6].z = -1;

		pObj->cloud.points[7].x = +1;
		pObj->cloud.points[7].y = +1;
		pObj->cloud.points[7].z = -1;
*/

		// Plane test
		pObj->cloud.points[0].x = -1;
		pObj->cloud.points[0].y = -1;
		pObj->cloud.points[0].z = -1;

		pObj->cloud.points[1].x = +1;
		pObj->cloud.points[1].y = -1;
		pObj->cloud.points[1].z = -1;

		pObj->cloud.points[2].x = -1;
		pObj->cloud.points[2].y = +1;
		pObj->cloud.points[2].z = -1;

		pObj->cloud.points[3].x = +1;
		pObj->cloud.points[3].y = +1;
		pObj->cloud.points[3].z = -1;


		/*
		// Access image information
		CCLTRY ( imgDct.lock ( pImg ) );
		if (hr == S_OK)
			{
			adtValue		vL;
			U32			row,col,cld;
			float			fX,fY,fZ;

			// Add cloud point for non-zero image pixel
			for (row = 0,cld = 0;row < imgDct.height();++row)
				{
				for (col = 0;col < imgDct.width() && cld < pObj->cloud.size();++col,++cld)
					{
					// Get pixel intensity
					if (	fabs(imgDct.getFloat(col,row)) < 1.e-20 )
						continue;

					// Add point to cloud using axis sources
					fX = getAxisPt ( srcX, col, row, pItX );
					fY = getAxisPt ( srcY, col, row, pItY );
					fZ = getAxisPt ( srcZ, col, row, pItZ );

					// Add based on organization
					if (bOrg)
						hr = E_NOTIMPL;
					else
						{
						pObj->cloud.points[cld].x = fX;
						pObj->cloud.points[cld].y = fY;
						pObj->cloud.points[cld].z = fZ;
						}	// else 
					}	// for

				// Reset iterators after each run
				if (pItX != NULL)
					pItX->begin();
				if (pItY != NULL)
					pItY->begin();
				if (pItZ != NULL)
					pItZ->begin();
				}	// for
			}	// if
		*/

		// TESTING
//		pcl::StatisticalOutlierRemoval<pcl::PointXYZ>	sor;
//		pcl::PointCloud<pcl::PointXYZ>::Ptr					cloud_ptr(&(pObj->cloud));
//		pcl::PointCloud<pcl::PointXYZ>::Ptr					cloud_f_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		
//		sor.setInputCloud(cloud_ptr);
//		sor.setMeanK(50);
//		sor.setStddevMulThresh(1.0);
//		sor.filter(*cloud_f_ptr);

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
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Xaxis) || _RCP(Yaxis) || _RCP(Zaxis))
		{
		IContainer	*pCnt = NULL;
		IIt			*pIt	= NULL;
		adtIUnknown	unkV(v);

		// Obtain iterator for axis
		CCLTRY(_QISAFE(unkV,IID_IContainer,&pCnt));
		CCLTRY(pCnt->iterate(&pIt));

		// Assign
		_ADDREF(pIt);										// For assignment
		if (_RCP(Xaxis))
			{
			_RELEASE(pItX);
			pItX = pIt;
			}	// if
		else if (_RCP(Yaxis))
			{
			_RELEASE(pItY);
			pItY = pIt;
			}	// if
		else 
			{
			_RELEASE(pItZ);
			pItZ = pIt;
			}	// if

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pCnt);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

