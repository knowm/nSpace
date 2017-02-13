////////////////////////////////////////////////////////////////////////
//
//									STMMEM.CPP
//
//					Implementation of the memory stream object
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

StmMemory :: StmMemory ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pBlock	= NULL;
	pos		= 0;
	}	// StmMemory

HRESULT StmMemory :: available ( U64 *puAv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Retrieve the number of bytes available for reading.
	//
	//	PARAMETERS
	//		-	puAv will receive the available bytes
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	U64		pos;

	// Use seek to get remaining size
	CCLTRY ( seek ( 0, STREAM_SEEK_CUR, &pos ) );
	CCLTRY ( seek ( 0, STREAM_SEEK_END, puAv ) );
	CCLTRY ( seek ( pos, STREAM_SEEK_SET, NULL ) );

	// Compute remaining size from current position
	CCLOK ( (*puAv) -= pos; )

	return hr;
	}	// available

HRESULT StmMemory :: clone ( IUnknown **ppUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICloneable
	//
	//	PURPOSE
	//		-	Clones the object.
	//
	//	PARAMETERS
	//		-	ppUnk will receive the cloned object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	StmMemory	*pCopy	= NULL;

	// Cloning a stream just clones the state (position, etc) not the
	// contents of the stream.

	// Create another instance of this object
	CCLTRYE	( (pCopy = new StmMemory()) != NULL, E_OUTOFMEMORY );
	CCLOK		( pCopy->AddRef(); )
	CCLTRY	( pCopy->construct(); )

	// Mirror state
	if (hr == S_OK)
		{
		_RELEASE((pCopy->pBlock));
		pCopy->pBlock	= pBlock;
		_ADDREF((pCopy->pBlock));
		pCopy->pos		= pos;
		}	// if

	// Result
	if (hr == S_OK)
		{
		(*ppUnk) = (IByteStream *) pCopy;
		_ADDREF((*ppUnk));
		}	// if

	// Clean up
	_RELEASE(pCopy);

	return hr;
	}	// clone

HRESULT StmMemory :: construct ( void )
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
	HRESULT			hr			= S_OK;

	// Create a memory block for ourselves.  Possible for 'MemoryBlock'
	// to create this class directly
	if (hr == S_OK && pBlock == NULL)
		hr = ((pBlock = new MemoryBlock()) != NULL) ? S_OK : E_OUTOFMEMORY;
	CCLOK ( pBlock->AddRef(); )

	return hr;
	}	// construct

HRESULT StmMemory :: copyTo ( IByteStream *pStmDst, U64 uSz, U64 *puSz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Copies the specified # of bytes to another stream.
	//
	//	PARAMETERS
	//		-	pStmDst is the target stream
	//		-	uSz is the amount to copy (zero for rest of stream)
	//		-	puSz is the amount copied
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	U8			*pcBfr	= NULL;
	U64		uNum		= 0;
	U64		uNumW;

	// Setup
	if (puSz) *puSz = 0;

	// Copy from memory directly to destination stream
	CCLOK ( uNum	= (uSz) ? uSz : (pBlock->szBlk-pos); )
	CCLOK ( pcBfr	= &(pBlock->pcBlk[pos]); )
	while (hr == S_OK && pos < pBlock->szBlk && uNum)
		{
		// Write
		CCLOK  ( uNumW = (pos+uNum <= pBlock->szBlk) ? uNum : (pBlock->szBlk-pos); )
		CCLTRY ( pStmDst->write ( pcBfr, uNumW, &uNumW ) );

		// Next block
		CCLOK ( pcBfr	+= uNumW; )
		CCLOK ( pos		+= uNumW; )
		CCLOK ( uNum	-= uNumW; )
		if (hr == S_OK && puSz) (*puSz) += uNumW;
		}	// while

	return hr;
	}	// copyTo

void StmMemory :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pBlock);
	}	// destruct

HRESULT StmMemory :: flush ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Flush the stream state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Nothing to do for memory based stream

	return hr;
	}	// flush

HRESULT StmMemory :: read ( void *pvBfr, U64 nio, U64 *pnio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr will receive the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	U64		uLeft,uC;

	// Calculate # of bytes to copy
	uLeft	= (pos < pBlock->szBlk) ? (pBlock->szBlk-pos) : 0;
	uC		= (uLeft < nio) ? uLeft : nio;

	// Copy data
	if (uC)
		{
		CCLTRYE	( (pBlock->pcBlk != NULL), E_UNEXPECTED );
		CCLOK		( memcpy ( pvBfr, &(pBlock->pcBlk[pos]), (size_t)uC ); )
		}	// if
	else
		hr = S_FALSE;

	// Count
	if (pnio != NULL)	*pnio = uC;

	// Move pointer
	pos += uC;

	return hr;
	}	// read

HRESULT StmMemory :: seek ( S64 sPos, U32 uFrom, U64 *puPos )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Places the current byte position at the specified location.
	//
	//	PARAMETERS
	//		-	sPos is the new position
	//		-	uFrom specified where to start seek from
	//		-	puPos will receive the new position
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	S64		sNewPos 	= 0;

	// Seek
	switch (uFrom)
		{
		case STREAM_SEEK_SET :
			sNewPos = sPos;
			break;

		case STREAM_SEEK_CUR :
			sNewPos = pos + sPos;
			break;

		case STREAM_SEEK_END :
			sNewPos = pBlock->szBlk - sPos;
			break;

		default :
			hr = E_INVALIDARG;
		}	// switch

	// Rail position
	if			(sNewPos < 0)							pos = 0;
	else if	((U32)sNewPos > pBlock->szBlk)	pos = pBlock->szBlk;
	else													pos = sNewPos;

	// New position
	if (puPos != NULL) *puPos = sNewPos;

	return hr;
	}	// seek

HRESULT StmMemory :: setSize ( U64 uSz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Sets the size of the stream.
	//
	//	PARAMETERS
	//		-	uSz is the new size
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Set size of backing block
	CCLTRY ( pBlock->setSize ( (U32)uSz ) );

	// Adjust stream position if necessary
	if (hr == S_OK && pos > uSz)
		pos = uSz;

	return hr;
	}	// setSize

HRESULT StmMemory :: write ( void const *pcvBfr, U64 nio, U64 *pnio )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IByteStream
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the stream.
	//
	//	PARAMETERS
	//		-	pvBfr will receive the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// If count will go over current size, resize
	if ((pos + nio) > pBlock->szBlk)
		{
		CCLTRY ( setSize ( pos+nio ) );
		}	// if

	// Copy data
	if (nio)
		{
		CCLTRYE	( (pBlock->pcBlk != NULL), E_UNEXPECTED );
		CCLOK		( memcpy ( &(pBlock->pcBlk[pos]), pcvBfr, (size_t)nio ); )
		}	// if
	else
		hr = S_FALSE;

	// Count
	if (pnio != NULL)	*pnio = nio;

	// Move pointer
	pos += nio;

	return hr;
	}	// write

