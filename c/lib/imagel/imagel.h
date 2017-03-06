////////////////////////////////////////////////////////////////////////
//
//										IMAGEL.H
//
//							Image processing library
//
////////////////////////////////////////////////////////////////////////

#ifndef	IMAGEL_H
#define	IMAGEL_H

// System includes
#include "../../lib/nspcl/nspcl.h"

///////////
// Objects
///////////

// Integer version of image formats
#define	IMGFMT_INV		-1
#define	IMGFMT_U8X2		0
#define	IMGFMT_S8X2		1
#define	IMGFMT_U16X2	2
#define	IMGFMT_S16X2	3
#define	IMGFMT_F32X2	4
#define	IMGFMT_R8G8B8	5

//
// Class - ImageDct.  Convience class for dealing with images stored
//		in a dictionary.
//
// NOTE: Implemented in header to avoid having to link to library
//

class ImageDct
	{
	public :
	ImageDct ( void )
		{
		pDct		= NULL;
		pBits		= NULL;
		pvBits	= NULL;
		iW			= 0;
		iH			= 0;
		iFmt		= 0;	
		iBpp		= 0;
		iCh		= 0;
		}	// ImageDct
	virtual ~ImageDct ( void )
		{
		_UNLOCK(pBits,pvBits);
		_RELEASE(pBits);
		_RELEASE(pDct);
		}	// ~ImageDct

	// Utilities
	HRESULT	lock ( IUnknown *punkDct )
		{
		HRESULT			hr		= S_OK;
		adtIUnknown		unkV(punkDct);
		adtValue			vL;
		adtString		strFmt;

		// Container
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDct) );

		// Parameters
		CCLTRY ( pDct->load ( adtString(L"Width"), vL ) );
		CCLOK  ( iW = adtInt(vL); )
		CCLTRY ( pDct->load ( adtString(L"Height"), vL ) );
		CCLOK  ( iH = adtInt(vL); )
		CCLTRY ( pDct->load ( adtString(L"Format"), vL ) );
		CCLTRY ( adtValue::toString ( vL, strFmt ) );

		// Map string format to integer for conveinance
		if (hr == S_OK)
			{
			if (!WCASECMP(strFmt,L"U8X2"))
				{
				iFmt	= IMGFMT_U8X2;
				iBpp	= 8;
				iCh	= 1;
				}	// if
			else if (!WCASECMP(strFmt,L"S8X2"))
				{
				iFmt	= IMGFMT_S8X2;
				iBpp	= 8;
				iCh	= 1;
				}	// if
			else if (!WCASECMP(strFmt,L"U16X2"))
				{
				iFmt	= IMGFMT_U16X2;
				iBpp	= 16;
				iCh	= 1;
				}	// if
			else if (!WCASECMP(strFmt,L"S16X2"))
				{
				iFmt	= IMGFMT_S16X2;
				iBpp	= 16;
				iCh	= 1;
				}	// if
			else if (!WCASECMP(strFmt,L"F32x2"))
				{
				iFmt	= IMGFMT_F32X2;
				iBpp	= 32;
				iCh	= 1;
				}	// if
			else if (!WCASECMP(strFmt,L"R8G8B8"))
				{
				iFmt	= IMGFMT_R8G8B8;
				iBpp	= 24;
				iCh	= 3;
				}	// if
			else
				iFmt = IMGFMT_INV;
			}	// if

		// Get bits and lock
		CCLTRY ( pDct->load ( adtString(L"Bits"), vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBits) );
		CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

		return hr;
		}	// lock

	float		getFloat	( S32 x, S32 y )
		{
		float		fRet	= 0;

		// State check
		if (pDct == NULL || pvBits == NULL)
			return 0;

		// Based on format
		switch (iFmt)
			{
			case	IMGFMT_U8X2		:
				fRet = *(((U8 *)pvBits) + y*iW + x);
				break;
			case	IMGFMT_S8X2		:
				fRet = *(((S8 *)pvBits) + y*iW + x);
				break;
			case	IMGFMT_U16X2	:
				fRet = *(((U16 *)pvBits) + (y*iW + x)*sizeof(U16));
				break;
			case	IMGFMT_S16X2	:
				fRet = *(((S16 *)pvBits) + (y*iW + x)*sizeof(S16));
				break;
			case	IMGFMT_F32X2	:
				fRet = *(((float *)pvBits) + (y*iW + x)*sizeof(float));
				break;
			}	// switch

		return fRet;
		}
	S32		getInt	( S32 x, S32 y ) { return (S32)getFloat(x,y); }

	// Run-time data
	IDictionary		*pDct;								// Image dictionary
	S32				iW,iH,iFmt,iCh,iBpp;				// Size information
	IMemoryMapped	*pBits;								// Memory block
	void				*pvBits;								// Ptr. to memory
	};

//
// Class - libJpeg.  Object for JPEG images.
//

class libJpeg 
	{
	public :
	libJpeg ( void );										// Constructor
	virtual ~libJpeg ( void );							// Destructor

	// Run-time data
	adtInt	iQ;											// Quality setting
	U8			*pccinfo[2],*pcjerr[2];					// JPEG library support
	U8			*pcdstmgr[2],*pcdinfo[2];				// JPEG library support
	U8			*pcsrcmgr[2];								// JPEG library support

	// Utilities
	HRESULT	construct	( void );					// Construct object
	HRESULT	compress		( IDictionary * );
	HRESULT	compress		( U32, U32, U32, U32, IMemoryMapped *,
									IMemoryMapped * );
	HRESULT	decompress	( IDictionary * );
	HRESULT	decompress	( IMemoryMapped *, IMemoryMapped *,
									U32 *, U32 *, U32 *, U32 * );

	private :

	};

//
// Class - libPng.  Object for PNG images.
//

class libPng 
	{
	public :
	libPng ( void );										// Constructor
	virtual ~libPng ( void );							// Destructor

	// Run-time data
	HRESULT			hr_int;								// Parameters
	U8					*pcBitsSrc;							// Decompression bits
	U32				idxSrc,lenSrc;						// Decompression bits

	// Utilities
	HRESULT	construct	( void );					// Construct object
//	HRESULT	compress		( IDictionary * );
//	HRESULT	compress		( U32, U32, U32, U32, IMemoryMapped *,
//									IMemoryMapped * );
	HRESULT	decompress	( IDictionary * );
	HRESULT	decompress	( IMemoryMapped *, IMemoryMapped *,
									U32 *, U32 *, U32 *, U32 * );

	private :

	};

//////////////
// Interfaces
//////////////

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_At, 0x2534d0ce, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Binary, 0x2534d017, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_CascadeClassifier, 0x2534d0e4, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Codec, 0x2534d0db, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Contours, 0x2534d0d6, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Convert, 0x2534d018, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Create, 0x2534d0bd, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Distance, 0x2534d0d4, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Draw, 0x2534d0bc, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_FaceRecognizer, 0x2534d0e3, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Features, 0x2534d0da, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Flip, 0x2534d0cf, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_FFT, 0x2534d011, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Gradient, 0x2534d0d8, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Match, 0x2534d0d2, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Morph, 0x2534d0d5, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Normalize, 0x2534d019, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_PersistImage, 0x2534d012, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Prepare, 0x2534d013, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Resize, 0x2534d0d1, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Roi, 0x2534d0cc, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Smooth, 0x2534d0d3, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Stats, 0x2534d0cd, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Threshold, 0x2534d014, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_VideoCapture, 0x2534d0e5, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_VideoWriter, 0x2534d0e6, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif

