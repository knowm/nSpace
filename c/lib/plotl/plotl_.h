////////////////////////////////////////////////////////////////////////
//
//										PLOTL_.H
//
//				Implementation include file for plot library
//
////////////////////////////////////////////////////////////////////////

#ifndef	PLOTL__H
#define	PLOTL__H

// Includes
#include "plotl.h"

// GnuPlot usage outputs PNG images
#include "../../lib/ext/libpng-1.2.7/png.h"

// Defines
#define MAX_VECTS		100								// Max plotting vectors

///////////
// Objects
///////////

// Forward dec.
class GnuPlotSrvrt;

//
// Class - GnuPlotSrvr.  Server object to interface to the GnuPlot application.
//

class GnuPlotSrvr :
	public CCLObject										// Base class
	{
	public :
	GnuPlotSrvr ( void );								// Constructor

	// Run-time data
	HANDLE					hStdIn,hStdOut;			// Named pipes
	HANDLE					hWrIn,hRdOut;				// Read/write for pipes
	PROCESS_INFORMATION	gnuInfo;						// GnuPlotSrvr process
	GnuPlotSrvrt			*pTick;						// Worker tickable
	IThread					*pThrd;						// Worker thread
	png_structp				png_ptr;						// PNG Read structure
	png_infop				info_ptr;					// PNG Info structure
	IDictionary				*pDctImg;					// Reusable image output
	IMemoryMapped			*pBits;						// Image bits
	VOID						*pvBits;						// Locked image bits
	HRESULT					hr_img;						// Image result
	U16						uStride;						// Image stride
	sysCS						csPlot;						// Thread safety
	sysEvent					evPlot;						// Plot complete
	IDictionary				*pDctV[MAX_VECTS];		// Vector values
	ADTVALUE					*pValV[MAX_VECTS];		// Vector values
	U32						iColV[MAX_VECTS];			// Vector column

	// Utilities
	STDMETHOD(run)		( bool );						// Run/stop server
	STDMETHOD(plot)	( IDictionary * );			// Perform plot

	// PNGs
	HRESULT	png_data		( BYTE *, U32 );
	HRESULT	png_end		( void );
	HRESULT	png_init		( void );
	HRESULT	png_uninit	( void );

	// CCL
	CCL_OBJECT_BEGIN_INT(GnuPlotSrvr)
	CCL_OBJECT_END()
	virtual HRESULT	construct(void);				// Construct object
	virtual void		destruct(void);				// Destruct object

	private :

	// Internal utilities
	HRESULT appLoc			( adtString & );			// Retrieve GNU plot application location
	HRESULT start			( void );					// Start interface
	HRESULT stop			( void );					// Stop interface
	};

//
// Class - GnuPlotSrvrt.  GnuPlot I/O thread
//

class GnuPlotSrvrt :
	public CCLObject,										// Base class
	public ITickable										// Interface
	{
	public:
	GnuPlotSrvrt ( GnuPlotSrvr * );					// Constructor

	// Run-time data
	GnuPlotSrvr	*pParent;								// Parent object
	bool			bRun;										// Worker thread should run
	HANDLE		hevW[2];									// Wait events
	OVERLAPPED	ovRd;										// Overlapped structures
	HANDLE		hevWr;									// Write event
	BYTE			bBfrRd[8192];							// Read buffers
	U8				bBfrWr[8192];							// Write buffer
	U32			uBfrWr;									// Write buffer count

	// Utilities
	HRESULT flush		( void );
	HRESULT writeStr	( const WCHAR * );
	HRESULT write		( const VOID *, U32 );

	// ITickable members
	STDMETHOD(tick)		(void);
	STDMETHOD(tickAbort)	(void);
	STDMETHOD(tickBegin)	(void);
	STDMETHOD(tickEnd)	(void);

	// CCL
	CCL_OBJECT_BEGIN_INT(GnuPlotSrvrt)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual HRESULT	construct(void);				// Construct object
	virtual void		destruct(void);				// Destruct object

	private :
	};

/////////
// Nodes
/////////

//
// Class - GnuPlot.  Client node for generating plots via GnuPlot server.
//

class GnuPlot :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	GnuPlot ( void );										// Constructor

	// Run-time data
	IDictionary	*pReq;									// Active plot request
	IDictionary	*pVcts;									// Request vectors
	bool			bReq;										// Request is valid

	// CCL
	CCL_OBJECT_BEGIN(GnuPlot)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct(void);				// Construct object
	virtual void		destruct(void);				// Destruct object

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
