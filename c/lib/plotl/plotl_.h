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
#include <crtdbg.h>

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
	bool						bPngEnd;						// End of PNG detected

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
// Class - Image.  Client node for generating plot images.
//								(Currently uses Image).
//

class Image :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Image ( void );										// Constructor

	// Run-time data
	adtInt			iIdx;									// Active index
	adtString		strTitle;							// Plot title
	adtInt			iPlotW,iPlotH;						// Plot size (pixels)
	IUnknown			*pData;								// Data block

	// Plot data
	IDictionary		*pReq;								// Plot request
	adtString		strLblX0,strLblX1;				// X-axis labels
	adtString		strLblY0,strLblY1;				// Y-axis labels

	// CCL
	CCL_OBJECT_BEGIN(Image)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct(void);				// Construct object
	virtual void		destruct(void);				// Destruct object

	// Connections
	DECLARE_RCP(Data)
	DECLARE_EMT(Error)
	DECLARE_RCP(Height)
	DECLARE_CON(Fire)
	DECLARE_RCP(Range)
	DECLARE_RCP(Width)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Data)
		DEFINE_EMT(Error)
		DEFINE_RCP(Height)
		DEFINE_CON(Fire)
		DEFINE_RCP(Range)
		DEFINE_RCP(Width)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT update	( void );
	};

#endif
