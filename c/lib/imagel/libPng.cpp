////////////////////////////////////////////////////////////////////////
//
//										PNG.CPP
//
//				Implementation of the PNG (de)compression utilities
//
//				Copyright (c) nSpace, LLC
//				All right reserved
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// External SDK
#include "../ext/libpng-1.2.7/png.h"
#include <setjmp.h>

// Prototypes
static void png_read_data	( png_structp, png_bytep, png_size_t );

libPng :: libPng ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	hr_int = S_OK;
	}	// libPng

libPng :: ~libPng ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	}	// destruct

HRESULT libPng :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	return hr;
	}	// construct

HRESULT libPng :: decompress ( IDictionary *pDct )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Decompresses an compressed image 'in-place'.
	//
	//	PARAMETERS
	//		-	pDct contains and will receive decompressed image data.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr					= S_OK;
	IMemoryMapped	*pBitsSrc		= NULL;
	IMemoryMapped	*pBitsDst		= NULL;
	U8					*pcBitsDst		= NULL;
	png_structp		png_ptr			= NULL;
	png_infop		info_ptr			= NULL;
	png_infop		end_info			= NULL;
	png_bytepp		row_pointers	= NULL;
	U32				w,h,bpp,stride,sz,row,ch;
	adtIUnknown		unkV;
	adtValue			vL;

	// Access source bits
	CCLTRYE( pDct != NULL, E_INVALIDARG );
	CCLTRY ( pDct->load ( adtString(L"Bits"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IMemoryMapped,&pBitsSrc) );
	CCLTRY ( pBitsSrc->lock ( 0, 0, (void **) &pcBitsSrc, &lenSrc ) );
	CCLOK  ( idxSrc = 0; )

	// Initialize/allocate structures
	CCLTRYE ( (png_ptr = png_create_read_struct ( PNG_LIBPNG_VER_STRING,
					png_voidp_NULL, png_error_ptr_NULL, png_error_ptr_NULL ))
					!= NULL, E_OUTOFMEMORY );
	CCLTRYE ( (info_ptr = png_create_info_struct ( png_ptr )) != NULL,
					E_OUTOFMEMORY );
	CCLTRYE ( (end_info = png_create_info_struct ( png_ptr )) != NULL,
					E_OUTOFMEMORY );

	// PNG uses archaic 'setjmp'
	if (hr == S_OK)
		{
		if (setjmp(png_jmpbuf(png_ptr)))
			{
			hr = S_FALSE;
			goto cleanup;
			}	// if
		}	// if

	// Use custom I/O routines to access stream.
	CCLOK ( png_set_read_fn ( png_ptr, this, png_read_data ); )

	// Read image from stream
	CCLOK		( hr_int = S_OK; )
	CCLOK		( png_read_png ( png_ptr, info_ptr, PNG_TRANSFORM_BGR, NULL ); )
	CCLTRYE	( hr_int == S_OK, hr_int );

	// Obtain row pointers for image
	CCLTRYE ( (row_pointers = png_get_rows ( png_ptr, info_ptr )) != NULL,
					E_UNEXPECTED );

	// Allocate memory for bits
	if (hr == S_OK)
		{
		// Image parameters
		w			= png_get_image_width(png_ptr,info_ptr);
		h			= png_get_image_height(png_ptr,info_ptr);
		ch			= png_get_channels(png_ptr,info_ptr);
		bpp		= ch*png_get_bit_depth(png_ptr,info_ptr);
		stride	= w*(bpp/8);

		// Allocate memory block for bits
		sz			= (h*stride);
		CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBitsDst ) );
		CCLTRY(pBitsDst->setSize ( sz ));
		CCLTRY(pBitsDst->lock ( 0, 0, (void **) &pcBitsDst, NULL ));
		}	// if

	// Copy bits from rows into own buffer
	for (row = 0;hr == S_OK && row < h;++row)
		{
		// Copy row and move to next one
		memcpy ( pcBitsDst, row_pointers[row], stride );
		pcBitsDst += stride;
		}	// for

	// Store image parameters
	CCLTRY ( pDct->store ( adtString(L"Width"),	adtInt(w) ) );
	CCLTRY ( pDct->store ( adtString(L"Height"),	adtInt(h) ) );
	CCLTRY ( pDct->store ( adtString(L"Bits"),	adtIUnknown(pBitsDst) ) );

	// Format
	if (hr == S_OK)
		{
		if (bpp == 8 && ch == 1)
			hr = pDct->store ( adtString(L"Format"), adtString(L"U8x2") );
		else if (bpp == 16 && ch == 1)
			hr = pDct->store ( adtString(L"Format"), adtString(L"U16x2") );
		else if (bpp == 24 && ch == 3)
			hr = pDct->store ( adtString(L"Format"), adtString(L"R8G8B8") );
		else if (bpp == 32 && ch == 3)
			hr = pDct->store ( adtString(L"Format"), adtString(L"R8G8B8A8") );
		else
			hr = E_UNEXPECTED;
		}	// if

	// Clean up
	cleanup:
	png_destroy_read_struct ( &png_ptr, &info_ptr, &end_info );
	_UNLOCK(pBitsSrc,pcBitsSrc);
	_RELEASE(pBitsSrc);
	_UNLOCK(pBitsDst,pcBitsDst);
	_RELEASE(pBitsDst);

	return hr;
	}	// decompress

//
// PNG callback routines
//

static void png_read_data	( png_structp read_ptr, png_bytep data, png_size_t length )
	{
	// Obtain ptr. to byte stream that was specified as registration time.
	libPng		*pThis	= (libPng *) png_get_io_ptr(read_ptr);
	HRESULT		hr			= pThis->hr_int;

	// Valid copy ?
	CCLTRYE ( pThis->idxSrc + length <= pThis->lenSrc, E_UNEXPECTED );

	// Copy block
	CCLOK ( memcpy ( data, &pThis->pcBitsSrc[pThis->idxSrc], length ); )
	CCLOK ( pThis->idxSrc += (U32)length; )

	// Error ?
	pThis->hr_int = hr;
	if (hr != S_OK) png_error ( read_ptr, "Error copying data\r\n" );
	}	// png_read_data
