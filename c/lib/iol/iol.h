////////////////////////////////////////////////////////////////////////
//
//										IOL.H
//
//									I/O library
//
////////////////////////////////////////////////////////////////////////

#ifndef	IOL_H
#define	IOL_H

// System includes
#include "../adtl/adtl.h"

// Macros

#define	_UNLOCK(a,b)		if ((a) != NULL && (b) != NULL)					\
										{ (a)->unlock ( (b) ); (b) = NULL; }

#ifndef	STREAM_SEEK_SET
#define	STREAM_SEEK_SET		0
#define	STREAM_SEEK_CUR		1
#define	STREAM_SEEK_END		2
#endif

//////////////
// Interfaces
//////////////

//
// Interface - IByteStream.  Base interface for a byte stream.
//

DEFINE_GUID	(	IID_IByteStream, 0x2534d008, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IByteStream,IUnknown)
	{
	public :
	STDMETHOD(available)	( U64 * )								PURE;
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * )		PURE;
	STDMETHOD(flush)		( void )									PURE;
	STDMETHOD(read)		( void *, U64, U64 * )				PURE;
	STDMETHOD(seek)		( S64, U32, U64 * )					PURE;
	STDMETHOD(setSize)	( U64 )									PURE;
	STDMETHOD(write)		( void const *, U64, U64 * )		PURE;
	};

//
// Interface - ILocations.  Base interface for a source of streams.
//

DEFINE_GUID	(	IID_ILocations, 0x2534d007, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(ILocations,IUnknown)
	{
	public :
	STDMETHOD(open)		( IDictionary *,	IUnknown ** )		PURE;
	STDMETHOD(locations)	( const WCHAR *,	IIt ** )				PURE;
	STDMETHOD(status)		( const WCHAR *,	IDictionary * )	PURE;
	};

//
// Interface - IResource.  Interface to a generic I/O resource object that can
//										be opened and closed.
//

DEFINE_GUID	(	IID_IResource, 0x2534d00b, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IResource,IUnknown)
	{
	public :
	STDMETHOD(close)		( void )				PURE;
	STDMETHOD(getResId)	( ADTVALUE & )		PURE;
	STDMETHOD(open)		( IDictionary * )	PURE;
	};

//
// Interface - IStreamPersist.  Interface to an object that can persist
//		values from and to a stream.
//

DEFINE_GUID	(	IID_IStreamPersist, 0x2534d00d, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IStreamPersist,IUnknown)
	{
	public :
//	STDMETHOD(iterate)	( IByteStream *, IIt ** )				PURE;
	STDMETHOD(load)		( IByteStream *, ADTVALUE & )			PURE;
	STDMETHOD(save)		( IByteStream *, const ADTVALUE & )	PURE;
	};

//
// Interface - IMemoryMapped.  Interface to an object that is
//		memory mapped.
//

DEFINE_GUID	(	IID_IMemoryMapped, 0x2534d01f, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IMemoryMapped,IUnknown)
	{
	public :
	STDMETHOD(getSize)	( U32 * )							PURE;
	STDMETHOD(lock)		( U32, U32, void **, U32 * )	PURE;
	STDMETHOD(setSize)	( U32 )								PURE;
	STDMETHOD(stream)		( IByteStream ** )				PURE;
	STDMETHOD(unlock)		( void * )							PURE;
	};

///////////
// Classes
///////////

// Objects

DEFINE_GUID	(	CLSID_ByteCache, 0x2534d0b0, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_File, 0x2534d0bf, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Lock, 0x2534d03b, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_MemoryBlock, 0x2534d020, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StmFile, 0x2534d00c, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StmMemory, 0x2534d021, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StmPrsBin, 0x2534d00e, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StmPrsXML, 0x2534d07a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StmSrcFile, 0x2534d00a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

// Nodes

DEFINE_GUID	(	CLSID_EnumDevices, 0x2534d08e, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Persist, 0x2534d049, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Resource, 0x2534d036, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Serial, 0x2534d0be, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StreamCopy, 0x2534d07b, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StreamOp, 0x2534d04a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_StreamSource, 0x2534d031, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

#endif
