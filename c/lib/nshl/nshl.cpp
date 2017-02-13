////////////////////////////////////////////////////////////////////////
//
//									NSHL.CPP
//
//							nSpace shell utilities
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "nshl_.h"

HRESULT strToVal ( const adtString &str, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert an nSpace formatted string to a value.
	//
	//	PARAMETERS
	//		-	str is the string to use
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	IStreamPersist	*pPer	= NULL;
	IByteStream		*pStm	= NULL;
	int				len	= str.length();
	U8					c;

	// Setup
	adtValue::clear(v);

	// Create parser for string
	CCLTRY ( COCREATE ( L"Nspc.PersistTxt",IID_IStreamPersist,&pPer ) );

	// Create memory byte stream for string
	CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStm ) );

	// Write string to stream
	for (int i = 0;hr == S_OK && i < (int)len;++i)
		hr = pStm->write ( &(c = (U8)str[i]), 1, NULL );
	CCLTRY ( pStm->write ( &(c = '\r'), 1, NULL ) );
	CCLTRY ( pStm->write ( &(c = '\n'), 1, NULL ) );

	// Reset position
	CCLTRY ( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );

	// Parse value
	CCLTRY ( pPer->load ( pStm, v ) );

	// Clean up
	_RELEASE(pStm);
	_RELEASE(pPer);

	return hr;
	}	// strToVal

HRESULT valToStr ( const ADTVALUE &v, adtString &str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert a value into an nSpace formatted string.
	//
	//	PARAMETERS
	//		-	v contains the value
	//		-	str will receive the string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	IStreamPersist	*pPer	= NULL;
	IByteStream		*pStm	= NULL;
	U8					c;
	U64				pos;

	// Create parser for string
	CCLTRY ( COCREATE ( L"Nspc.PersistTxt",IID_IStreamPersist,&pPer ) );

	// Create memory byte stream to receive string
	CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStm ) );

	// Format value
	CCLTRY ( pPer->save ( pStm, v ) );

	// Current position
	CCLTRY ( pStm->seek ( 0, STREAM_SEEK_CUR, &pos ) );

	// Reset position
	CCLTRY ( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );

	// Allocate memory for string
	CCLTRY ( str.allocate ( (int)pos ) );

	// Write stream to string
	for (int i = 0;hr == S_OK && i < (int)pos;++i)
		{
		// Next byte
		CCLTRY ( pStm->read ( &c, 1, NULL ) );

		// String
		CCLOK ( str.at(i) = c; )
		}	// for

	// Terminate string
	CCLOK  ( str.at((int)pos) = WCHAR('\0'); )

	// Clean up
	_RELEASE(pStm);
	_RELEASE(pPer);

	return hr;
	}	// valToStr

