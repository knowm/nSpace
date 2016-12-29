////////////////////////////////////////////////////////////////////////
//
//										ADTL.H
//
////////////////////////////////////////////////////////////////////////

#ifndef	ADTL_H
#define	ADTL_H

#include "../sysl/sysl.h"
#include "../ccl/ccl.h"

//
// Enum - Value types
//

enum VALUETYPE
	{
	VTYPE_EMPTY		= 0,
	VTYPE_I4			= 3,
	VTYPE_I8			= 20,
	VTYPE_R4			= 4,
	VTYPE_R8			= 5,
	VTYPE_DATE		= 7,
	VTYPE_STR		= 8,
	VTYPE_BOOL		= 11,
	VTYPE_VALUE		= 12,
	VTYPE_UNK		= 13,
	VTYPE_CONST		= 0x2000,
	VTYPE_BYREF		= 0x4000,
	VTYPE_TYPEMASK	= 0x0fff
	};

//typedef U16 VALUETYPE;

//
// Structure - ADTVALUE.  Structure representing a system value.  
//		32-bit aligned, 16 bytes total.
//
typedef struct tagADTVALUE ADTVALUE;
struct tagADTVALUE
	{
	U16			vtype;									// Value type
	U16			reserved1;								// Padded for alignment
	U16			reserved2;								// Padded for alignment
	U16			reserved3;								// Padded for alignment
	union
		{
		U16			vbool;								// Boolean
		DATE			vdate;								// Date
		float			vflt;									// Float
		double		vdbl;									// Double
		S32			vint;									// 4 byte integer
		S64			vlong;								// 8 byte integer
		WCHAR			*pstr;								// String
		ADTVALUE		*pval;								// Value reference
		IUnknown		*punk;								// IUnknown / Object
		};
	};

//
// Macros
//

// Initialize a value structure
#define	_ADTVINIT(a)									\
	{															\
	(a).vtype		= VTYPE_EMPTY;						\
	(a).vdbl			= 0.0;								\
	}

// Endian

// U16 swap
#define	SWAPS(a)										\
				(											\
				(((a) << 8)	& 0xff00) |				\
				(((a) >> 8) & 0x00ff)				\
				)

// U32 swap
#define	SWAPI(a)													\
				(														\
				(SWAPS(((a) >>  0) & 0x0000ffff) << 16) |	\
				(SWAPS(((a) >> 16) & 0x0000ffff) <<  0)	\
				)

// U64 swap
#define	SWAPL(a)													\
				(														\
				(SWAPI(((a) >>  0) & 0xffffffff) << 32) |	\
				(SWAPI(((a) >> 32) & 0xffffffff) <<  0)	\
				)

// Forward dec.
struct IList;

//
// Helper value classes
//

// Forward dec.
class adtString;

//
// Class - adtValue.  Class to encapsulate a value.
//

class adtValue : public ADTVALUE
	{
	public :
	adtValue ( void );									// Constructor
	adtValue	( ADTVALUE * );							// Constructor
	virtual ~adtValue ( void );						// Destructor

	// Static functionality
	static	void			clear			( ADTVALUE & );
	static	HRESULT		clone			( const ADTVALUE &, ADTVALUE & );
	static	int			compare		( const ADTVALUE &, const ADTVALUE & );
	static	HRESULT		copy			( const ADTVALUE &, ADTVALUE &, bool = false );
	static	bool			empty			( const ADTVALUE &v )
													{ return (v.vtype == VTYPE_EMPTY) ? true : false; }
	static	HRESULT		fromString	( const WCHAR *, VALUETYPE, ADTVALUE & );
	static	HRESULT		toString		( const ADTVALUE &, adtString & );
	static	HRESULT		toType		( const ADTVALUE &, VALUETYPE, ADTVALUE & );
	inline
	static	VALUETYPE	type			( const ADTVALUE &v )
												{ return (VALUETYPE)(v.vtype & VTYPE_TYPEMASK); }

	// Extractors
	inline operator IUnknown *()		const { return (vtype == VTYPE_UNK) ? punk : NULL; }

	// Operators
	adtValue & operator= ( const adtValue & );// Assignment
	#ifdef	_WIN32
	adtValue & operator= ( const VARIANT & );	// Assignment
	#endif
	};

//
// Class - adtString.  Class to encapsulate common string functionality.
//

class adtString : public ADTVALUE
	{
	public :
	adtString ( void );									// Constructor
	adtString ( const WCHAR * );						// Constructor
	adtString ( const char * );						// Constructor
	adtString ( const ADTVALUE & );					// Constructor
	adtString ( const adtString & );					// Constructor
	virtual ~adtString ( void );						// Destructor

	// Utilities

	/** Allocate space for the specified number of characters */
	HRESULT		allocate		( U32 len );

	/** Generate a string composes of this string plus the specified string and place
			into the destination string */
	HRESULT		append		( const WCHAR *, adtString &v );

	/**	Append a constant string to this object.  Additional space is
		*	allocated for the string if necessary and ownership of the string is taken. */
	HRESULT		append		( const WCHAR * );
	WCHAR &		at				( size_t = 0 );
	S32			indexOf		( WCHAR, S32 = 0 );
	S32			indexOf		( const WCHAR *, S32 = 0 );
	S32			lastIndexOf	( WCHAR, S32 = -1 );
	U32			length		( void ) const { return (	adtValue::type(*this) == VTYPE_STR && pstr != NULL) ? (U32)wcslen(pstr) : 0; }
	HRESULT		prepend		( const WCHAR * );
	void			replace		( WCHAR, WCHAR );
	HRESULT		substring	( S32 begin, S32 end, adtString &v );
	HRESULT		toAscii		( char ** ) const;
	#ifdef	_WIN32
	HRESULT		toBSTR		( BSTR * ) const;
	#endif
	void			toLower		( void );
	HRESULT		toUnquoted	( IList * );
	void			toUpper		( void );

	// Operators

	// Extractors
	inline
	operator const WCHAR *() const { return	(adtValue::type(*this) == VTYPE_STR) ? pstr : NULL; }

	// Assignment
	adtString &operator = ( const ADTVALUE & );
	adtString &operator = ( const adtString & );
	adtString &operator = ( const WCHAR * );
	adtString &operator = ( const char * );

	private :

	// Run-time data
//	bool		bOwn;											// Own string ?
	WCHAR		wErr;											// Error/null character

	// Internal utilities
	void addref		( void );
	void own			( U32 = 0 );
	void release	( void );

	};

//
// Class - adtStringSt.  Static strings are  for strings that never 
//		change i.e. literals,statics.
//

class adtStringSt : public adtString
	{
	public :
	adtStringSt ( const WCHAR *pw )					// Constructor
//		{ *this = pw; }
		{ pstr = (WCHAR *) pw; vtype = (VTYPE_STR|VTYPE_CONST); }
//	adtStringConst ( const ADTVALUE & );				// Constructor
	};

//
// Class - adtDate.  Class containing routines to maipulated dates.
//

class adtDate : public ADTVALUE
	{
	public :
	adtDate	( DATE = 0.0 );							// Constructor
	adtDate	( const ADTVALUE & );					// Constructor

	// Utilities
	static	HRESULT fromEpochSeconds( S32, S32, DATE * );
	static	HRESULT fromString		( const WCHAR *, ADTVALUE & );
	#ifdef	_WIN32
	static	HRESULT fromSystemTime	( SYSTEMTIME *, DATE * );
	static	HRESULT toSystemTime		( double, SYSTEMTIME * );
	#endif
	static	HRESULT toString			( const ADTVALUE &, adtString & );
				adtDate &now				( bool = false );

	// Extractors
	inline operator DATE() const { return vdate; }

	// Operators
	adtDate& operator= ( const ADTVALUE & );		// Assignment

	// Internal utilities
	static DATE varToRef ( DATE );
	static DATE refToVar ( DATE );
	};

//
// Class - adtInt.  Class for a 32-bit value.
//

class adtInt : public ADTVALUE
	{
	public :
	adtInt	( U32 = 0 );									// Constructor
	adtInt	( const ADTVALUE & );						// Constructor

	// Extractors
	inline operator U32 ()	{ return (vtype == VTYPE_I4) ? (U32)vint :
												(vtype == VTYPE_I8) ? (U32)vlong : 0; } // Be careful!

	// Operators
	adtInt& operator= ( const ADTVALUE & );			// Assignment
	adtInt &operator= ( const U32 );
	};

//
// Class - adtLong.  Class for a 64-bit value.
//

class adtLong : public ADTVALUE
	{
	public :
	adtLong	( U64 = 0 );									// Constructor
	adtLong	( const ADTVALUE & );						// Constructor

	// Extractors
	inline operator U64 ()	{ return (vtype == VTYPE_I4) ? vint :
												(vtype == VTYPE_I8) ? vlong : 0; }

	// Operators
	adtLong& operator= ( const ADTVALUE & );			// Assignment
	adtLong &operator= ( const U64 );
	};

//
// Class - adtDouble.  Class for a double precision floating point value.
//

class adtDouble : public ADTVALUE
	{
	public :
	adtDouble	( double = 0.0f );						// Constructor
	adtDouble	( float );									// Constructor
	adtDouble	( const ADTVALUE & );					// Constructor

	// Operators
	adtDouble& operator= ( const ADTVALUE & );		// Assignment

	// Extractors
	inline operator double() const { return vdbl; }
	};

//
// Class - adtFloat.  Class for a floating point value.
//

class adtFloat : public ADTVALUE
	{
	public :
	adtFloat	( float = 0.0f );							// Constructor
	adtFloat	( double );									// Constructor
	adtFloat	( const ADTVALUE & );					// Constructor

	// Operators
	adtFloat& operator= ( const ADTVALUE & );		// Assignment

	// Extractors
	inline operator float()	const { return vflt; }
	};

//
// Class - adtBool.  Class for a boolean value.
//

class adtBool : public ADTVALUE
	{
	public :
	adtBool	( bool = FALSE );							// Constructor
	adtBool	( const ADTVALUE & );					// Constructor

	// Extractors
	inline operator bool() const { return (vbool) ? true : false; }

	// Operators
	adtBool& operator= ( const ADTVALUE & );		// Assignment
	};

//
// Class - adtIUnknown.  Class to contain an 'IUnknown *' value.
//

class adtIUnknown : public ADTVALUE
	{
	public :
	adtIUnknown	( IUnknown * = NULL );				// Constructor
	adtIUnknown	( const ADTVALUE & );				// Constructor
	virtual ~adtIUnknown ( void );					// Destructor

	// Operators
	adtIUnknown& operator= ( const ADTVALUE & );		// Assignment
	adtIUnknown& operator= ( const adtIUnknown & );	// Assignment
	adtIUnknown& operator= ( IUnknown *_pUnk );		// Assignment
	inline IUnknown *operator-> () { return punk; }

	// Extractors
	inline operator IUnknown *() const { return punk; }
	inline operator IUnknown **() { return &punk; }
	};

//
// Class - adtIUnknownRef.  Class to contain an 'IUnknown *' value
//		by reference.
//

class adtIUnknownRef : public ADTVALUE
	{
	public :
	adtIUnknownRef	( IUnknown * = NULL );			// Constructor
	virtual ~adtIUnknownRef ( void );				// Destructor

	// Operators
	adtIUnknownRef& operator= ( IUnknown *_pUnk );
	inline IUnknown *operator-> () { return punk; }

	// Extractors
	inline operator IUnknown *()	const { return punk; }

	private :
	};

#ifdef	_WIN32

//
// Class - adtVariant.  Helper class for the Win32 VARIANT type
//

class adtVariant : public ::tagVARIANT
	{
	public :
	adtVariant ( void );									// Constructor
	adtVariant ( IUnknown * );							// Constructor
	adtVariant ( int );									// Constructor
	adtVariant ( long );									// Constructor
	adtVariant ( WCHAR * );								// Constructor
	adtVariant ( const WCHAR * );						// Constructor
	adtVariant ( const ADTVALUE & );					// Constructor
	adtVariant ( bool );									// Constructor
	adtVariant ( const VARIANT * );					// Constructor
	virtual ~adtVariant(void);							// Destructor

	// Utilities
	HRESULT	toValue	( ADTVALUE & );				// Conversion
	HRESULT	clear		( void );						// Clear contents

	// Operators
	adtVariant&	operator= ( const WCHAR * );	// Assignment
	adtVariant& operator= ( const ADTVALUE & );	// Assignment
	adtVariant& operator= ( const VARIANT * );	// Assignment

	// Extractors
	inline operator WCHAR *()			const	{ return bstrVal; }
	inline operator const WCHAR *()	const	{ return (bstrVal != NULL) ? bstrVal : L""; }

	private :
	IUnknown			*punkPrsrL,*punkPrsrS;			// Cached parser
	IUnknown			*punkStmL,*punkStmS;				// Cached byte stream
	};

#endif

//////////////
// Interfaces
//////////////

//
// Interface - IIt.  Base interface for an iterator.
//

DEFINE_GUID	(	IID_IIt, 0x2534d000, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IIt,IUnknown)
	{
	public :
	//! Place the iterator at the beginning of the container.
	STDMETHOD(begin)	( void )			PURE;
	//! Place the iterator at the end of the container.
	STDMETHOD(end)		( void )			PURE;
	//! Move to the next value in the container.
	STDMETHOD(next)	( void )			PURE;
	//! Move to the previous value in the container.
	STDMETHOD(prev)	( void )			PURE;
	//! Read the value at the current iterator position.
	STDMETHOD(read)	( ADTVALUE & )	PURE;
	};

//
// Interface - IContainer.  Interface to a generic container.
//

DEFINE_GUID	(	IID_IContainer, 0x2534d003, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IContainer,IUnknown)
	{
	public :
	STDMETHOD(clear)		( void )					PURE;
	STDMETHOD(copyTo)		( IContainer * )		PURE;
	STDMETHOD(isEmpty)	( void )					PURE;
	STDMETHOD(iterate)	( IIt ** )				PURE;
	STDMETHOD(remove)		( const ADTVALUE & ) PURE;
	STDMETHOD(size)		( U32 * )				PURE;
	};

//
// Interface - IList.  Interface to a list container.
//

DEFINE_GUID	(	IID_IList, 0x2534d004, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IList,IContainer)
	{
	public :
	STDMETHOD(write)	( const ADTVALUE & )	PURE;
	};

//
// Interface - IDictionary.  Interface to a container that is also
//					a generic dictionary.
//

DEFINE_GUID	(	IID_IDictionary, 0x2534d005, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IDictionary,IContainer)
	{
	public :
	STDMETHOD(keys)	( IIt ** )										PURE;
	STDMETHOD(load)	( const ADTVALUE &, ADTVALUE & )			PURE;
	STDMETHOD(store)	( const ADTVALUE &, const ADTVALUE & )	PURE;
	};

//
// Interface - IDictionaryNotify.  Interface to objects requesting
//		notifications from a dictionary.
//

DEFINE_GUID	(	IID_IDictionaryNotify, 0x2534d0a9, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IDictionaryNotify,IUnknown)
	{
	public :
	STDMETHOD(remove)		( const ADTVALUE & )							PURE;
	STDMETHOD(removed)	( const ADTVALUE & )							PURE;
	STDMETHOD(store)		( const ADTVALUE &, const ADTVALUE & )	PURE;
	STDMETHOD(stored)		( const ADTVALUE &, const ADTVALUE & )	PURE;
	};

//
// Interface - IHaveValue.  Interface to a single value object.
//

DEFINE_GUID	(	IID_IHaveValue, 0x2534d0ab, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IHaveValue,IUnknown)
	{
	public :
	STDMETHOD(getValue)	( ADTVALUE & )			PURE;
	STDMETHOD(setValue)	( const ADTVALUE & )	PURE;
	};

///////////
// Classes
///////////

// Classes

DEFINE_GUID	(	CLSID_AdtValue, 0x2534d0ac, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Dictionary, 0x2534d009, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_DictionaryNotify, 0x2534d0a7, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_DictionaryStm, 0x2534d068, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_List, 0x2534d00f, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Queue, 0x2534d016, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Stack, 0x2534d03e, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

// Nodes

DEFINE_GUID	(	CLSID_Iterate, 0x2534d02f, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Keys, 0x2534d043, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Load, 0x2534d02e, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Remove, 0x2534d032, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Stat, 0x2534d046, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Store, 0x2534d028, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_Write, 0x2534d02d, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

// Protoypes
bool adtwcasecmp ( const WCHAR *, const WCHAR *, const WCHAR * );

#endif

