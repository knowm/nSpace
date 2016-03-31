////////////////////////////////////////////////////////////////////////
//
//								GNUPLOTSRVR.CPP
//
//						Interface to the GnuPlot application
//
////////////////////////////////////////////////////////////////////////

#include "plotl_.h"
#include "../../lib/mathl/mathl.h"
#include <stdio.h>
#include <math.h>

// Globals
static U32	gnuPipeCnt	= 1;

// String references
adtString strRefWidth	( L"Width" );
adtString strRefHeight	( L"Height" );
adtString strRefFormat	( L"Format" );
adtString strRefBits		( L"Bits" );
adtString strRefBGRA		( L"B8G8R8A8" );
adtString strRefBGR		( L"B8G8R8" );
adtString strRefOnImg	( L"OnImage" );
adtString strRefLblPt	( L"LabelPt" );
adtString strRefTitle	( L"Title" );
adtString strRefLeft		( L"Left" );
adtString strRefRight	( L"Right" );
adtString strRefTop		( L"Top" );
adtString strRefBottom	( L"Bottom" );
adtString strRefFront	( L"Front" );
adtString strRefBack		( L"Back" );

// Currently using current default install directory, use in path instead ?
#define	PATH_GNUPLOT	L"c:\\program files\\gnuplot\\bin\\gnuplot.exe"

// Prototypes
static void PNGAPI png_progressive_info	( png_structp, png_infop );
static void PNGAPI png_progressive_row		( png_structp, png_bytep,
																png_uint_32, int );
static void PNGAPI png_progressive_end		( png_structp, png_infop );

GnuPlotSrvr :: GnuPlotSrvr ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	//	PARAMETERS
	//		-	hInst is the application instance
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	hStdIn		= INVALID_HANDLE_VALUE;
	hStdOut		= INVALID_HANDLE_VALUE;
	hWrIn			= INVALID_HANDLE_VALUE;
	hRdOut		= INVALID_HANDLE_VALUE;
	pThrd			= NULL;
	pTick			= NULL;
	pDctImg		= NULL;
	pBits			= NULL;
	pvBits		= NULL;
	memset ( pDctV, 0, sizeof(pDctV) );
	memset ( pValV, 0, sizeof(pValV) );
	}	// GnuPlotSrvr

HRESULT GnuPlotSrvr :: construct ( void )
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
	HRESULT	hr	= S_OK;

	// Create a re-usable image dictionary for PNG output, 
	// pre-allocate memory bits so it can just be resized as needed.
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctImg ) );
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBits ) );
	CCLTRY ( pBits->setSize ( 128*128*sizeof(U32) ) );
	CCLTRY ( pDctImg->store ( strRefBits, adtIUnknown(pBits) ) );

	// Events
	evPlot.init();

	return hr;
	}	// construct

void GnuPlotSrvr :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Ensure shutdown	
	stop();

	// PNG clean up
	png_uninit();

	// Clean up
	_RELEASE(pBits);
	_RELEASE(pDctImg);
	}	// destruct

HRESULT GnuPlotSrvr :: plot ( IDictionary *pReq )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generate a plot with the information in the request.
	//
	//	PARAMETERS
	//		-	pReq is the plot request dictionary
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary		*pVcts		= NULL;
	U32				iRows			= 0;
	U32				iCols			= 0;
	IMemoryMapped	*pBlk			= NULL;
	float				*pfBlk		= NULL;
	U32				cnt			= 0;
	bool				b3D			= false;
	adtIUnknown		unkV;
	adtValue			vL;
	adtFloat			vF;
	adtString		strV;
	float				fVal;

	// Thread safety in case of multiple plot clients
	csPlot.enter();

	// Value is expected to be a dictionary of vectors label V1, V2, etc...
	// for each column of data.  Each vector dictionary also contains
	// options like labels.

	// Access vectors 
	CCLTRY ( pReq->load ( adtString(L"Vector"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pVcts) );

	// 3D plot ?
	if (hr == S_OK && pReq->load ( adtString(L"3D"), vL ) == S_OK &&
			adtBool(vL) == true )
		b3D = true;

	// Reset labels from last time in case they are not specified
	CCLTRY ( pTick->writeStr ( L"set xlabel" ) );
	CCLTRY ( pTick->writeStr ( L"set ylabel" ) );
	if (hr == S_OK && b3D)
		hr = pTick->writeStr ( L"set zlabel" );

	// Search for ordered vector names until they run out
	for (cnt = 0;hr == S_OK && cnt < MAX_VECTS;++cnt)
		{
		WCHAR	wName[21];
		U32	nc,nr;

		// Does next vector key exist ?
		CCLOK ( swprintf ( SWPF(wName,21), L"V%d", cnt ); )
		if (hr == S_OK && pVcts->load ( adtString(wName), vL ) != S_OK)
			break;
		CCLTRY( _QISAFE((unkV=vL),IID_IDictionary,&(pDctV[cnt])) );

		// Access vector information
//		CCLTRY( mathVector ( pDctV[cnt], &nc, &nr, &(pValV[cnt]) ) );
		nc = nr = 0;
hr = S_FALSE;

		// A specific column can be specified for plotting
		CCLOK ( iColV[cnt] = 0; )
		if (hr == S_OK && pDctV[cnt]->load ( adtString(L"PlotColumn"), vL ) == S_OK)
			iColV[cnt] = adtInt(vL);

		// Rows must match for all vectors
		if (cnt == 0)
			{
			iRows = nr;
			iCols = nc;
			}	// if
		else
			{
			CCLTRYE ( iRows == nr && iCols == nc, E_INVALIDARG );
			if (hr != S_OK)
				dbgprintf ( L"GnuPlotSrvr::onStore:Fire:Vector %d does not match size\r\n", cnt );
			}	// else

		// Optional axis labelling (0 = X, 1 = Y, 2 = Z)
		if (hr == S_OK && cnt < 3 && pDctV[cnt]->load ( adtString(L"Label"), vL ) == S_OK)
			{
			adtString	strLbl(vL);
			adtString	strCmd = (cnt == 0) ? L"set xlabel '" : 
										(cnt == 1) ? L"set ylabel '" : L"set zlabel '";

			// Generate and send command
			CCLTRY ( strCmd.append ( strLbl ) );
			CCLTRY ( strCmd.append ( L"'" ) );
			if (hr == S_OK && (cnt < 2 || b3D))
				hr = pTick->writeStr ( strCmd );
			}	// if

		}	// for

	// Need at least one X and one Y
	CCLTRYE ( cnt > 1, E_INVALIDARG );

	// For 3D exactly 3D vectors are required
	CCLTRYE ( !b3D || cnt == 3, E_INVALIDARG );

	//
	// Range
	// In order to keep the point placement predictable (along with the margins)
	// set the range of the axis to be the same as the limits of the data.  By
	// default GnuPlot pads the extremes
	//
	float	fMinv[3]	= { 0, 0, 0 };
	float	fMaxv[3]	= { 0, 0, 0 };
	float	fMinr[3]	= { 0, 0, 0 };
	float	fMaxr[3]	= { 0, 0, 0 };
	float	fMinp[3]	= { 0, 0, 0 };
	float	fMaxp[3]	= { 1, 1, 1 };

	// Optional range percentages 
	if (hr == S_OK && pReq->load ( strRefLeft, vL ) == S_OK)
		fMinp[0] = adtFloat(vL);
	if (hr == S_OK && pReq->load ( strRefBottom, vL ) == S_OK)
		fMinp[1] = adtFloat(vL);
	if (hr == S_OK && b3D && pReq->load ( strRefBack, vL ) == S_OK)
		fMinp[2] = adtFloat(vL);
	if (hr == S_OK && pReq->load ( strRefRight, vL ) == S_OK)
		fMaxp[0] = adtFloat(vL);
	if (hr == S_OK && pReq->load ( strRefTop, vL ) == S_OK)
		fMaxp[1] = adtFloat(vL);
	if (hr == S_OK && b3D && pReq->load ( strRefFront, vL ) == S_OK)
		fMinp[2] = adtFloat(vL);
	
	// Initialize limits
	for (U32 v = 0;hr == S_OK && v < cnt && v < 3;++v)
		{
		fMaxv[v] = -1.e28f;
		fMinv[v] = +1.e28f;
		}	// for

	// Get limits on all of the specified vector information
	for (U32 v = 0;hr == S_OK && v < cnt && v < 3;++v)
		{
		// TODO: Current using Y0 as the 'scaling vector'.  Possibly
		// scan all provided Y-columns for limits for 2D multi-Y plots

		// Limits of column
		for (U32 r = 0;hr == S_OK && r < iRows;++r)
			{
			// Limits.  For Y axis only include values that are within
			// the specified X-range.
			if (	v == 0 || 
					(vF = pValV[0][r]) >= fMinr[0] && vF <= fMaxr[0] )
				{
				if ((vF = pValV[v][r]) < fMinv[v])
					fMinv[v] = (vF = pValV[v][r]);
				if ((vF = pValV[v][r]) > fMaxv[v])
					fMaxv[v] = (vF = pValV[v][r]);
				}	// if
			}	// for

		// Compute ranges based on requested percentages
		fMinr[v] = fMinv[v]+(fMaxv[v]-fMinv[v])*fMinp[v];
		fMaxr[v] = fMinv[v]+(fMaxv[v]-fMinv[v])*fMaxp[v];
		}	// for

	// Gnuplot cannot handle min and max the same
	for (U32 v = 0;hr == S_OK && v < cnt && v < 3;++v)
		{
		// Matching limits ?
		if (fMinr[v] == fMaxr[v])
			{
			// First value
			vF = pValV[v][0];

			// Span of zero ?
			if (vF == 0)
				vF = 1.0f;

			// Center the range around value
			fMinr[v] = vF - .05f*vF;
			fMaxr[v] = vF + .05f*vF;
			}	// if

		}	// for

	// Set exact ranges on plot area
	if (hr == S_OK)
		{
		WCHAR	wBfrRng[101];

		// X limits
		swprintf ( SWPF(wBfrRng,101), L"set xrange [%g:%g]", fMinr[0], fMaxr[0] );
		dbgprintf ( L"%s\r\n", wBfrRng );
		CCLTRY ( pTick->writeStr ( wBfrRng ) );

		// Y limits
		swprintf ( SWPF(wBfrRng,101), L"set yrange [%g:%g]", fMinr[1], fMaxr[1] );
		dbgprintf  ( L"%s\r\n", wBfrRng );
		CCLTRY ( pTick->writeStr ( wBfrRng ) );

		// Z limits
		if (b3D)
			{
			swprintf ( SWPF(wBfrRng,101), L"set zrange [%g:%g]", fMinr[2], fMaxr[2] );
			dbgprintf  ( L"%s\r\n", wBfrRng );
			CCLTRY ( pTick->writeStr ( wBfrRng ) );
			}	// if

		}	// if

	//
	// Label closest point
	// Caller can specify an X,Y percent that will result in the closest
	// point receiving a label for it's values
	//
	if (hr == S_OK && pReq->load ( adtString(L"LabelX"), vL ) == S_OK)
		{
		float			fXlbl		= 0;
		float			fYlbl		= 0;
		float			fZlbl		= 0;
		float			fXAt		= 0;
		float			fYAt		= 0;
		float			fZAt		= 0;
		float			fDisMin	= 0;
		adtString	strCmd;
		WCHAR			wNum[51];

		// Access the values
		fXlbl = adtFloat(vL);
		if (hr == S_OK && pReq->load ( adtString(L"LabelY"), vL ) == S_OK)
			fYlbl = adtFloat(vL);
		if (hr == S_OK && pReq->load ( adtString(L"LabelZ"), vL ) == S_OK)
			fZlbl = adtFloat(vL);

		// Search pts. for the closest match.  
		for (U32 v = 1;hr == S_OK && v < cnt && v < MAX_VECTS;++v)
			for (U32 r = 0;hr == S_OK && r < iRows;++r)
				{
				// Search using percentages to avoid magnitude differences between 
				// the axis throwing off the distance calculation.
				float fValX = ((vF = pValV[0][r])-fMinv[0])/(fMaxv[0]-fMinv[0]);
				float fValY = ((vF = pValV[v][r])-fMinv[1])/(fMaxv[1]-fMinv[1]);
				float fValZ = (b3D) ? (((vF = pValV[v][r])-fMinv[2])/(fMaxv[2]-fMinv[2])) : 0;

				// Distance between this point and target point
				float fDis =	(fXlbl-fValX)*(fXlbl-fValX) + 
									(fYlbl-fValY)*(fYlbl-fValY) +
									(fZlbl-fValZ)*(fZlbl-fValZ) ;

				// First point is always the closest or check for even closer ptr.
				if ( (v == 1 && r == 0) || fDis < fDisMin )
					{
					// Assign pt.
					fXAt = fValX;
					fYAt = fValY;
					fZAt = fValZ;

					// New minimum distance
					fDisMin = fDis;
					}	// if

				}	// for

		// Calculate actual plot point corodinates
		CCLOK ( fXAt = fXAt*(fMaxv[0]-fMinv[0]) + fMinv[0]; )
		CCLOK ( fYAt = fYAt*(fMaxv[1]-fMinv[1]) + fMinv[1]; )
		CCLOK ( fZAt = fZAt*(fMaxv[2]-fMinv[2]) + fMinv[2]; )

		// Lable the actual point.
		CCLOK ( strCmd = L"set label 100 '  "; )
		CCLOK ( swprintf ( SWPF(wNum,51), L"%g,%g", fXAt, fYAt ); )
		if (hr == S_OK && b3D)
			swprintf ( SWPF(wNum,51), L"%g,%g,%g", fXAt, fYAt, fZAt );
		CCLTRY( strCmd.append ( wNum ) );
		CCLTRY( strCmd.append ( L"' at first " ) );
		CCLTRY( strCmd.append ( wNum ) );
		CCLTRY ( pTick->writeStr ( strCmd ) );

		// Draw circle around point
		CCLOK ( strCmd = L"set object 100 circle at first "; )
		CCLOK ( swprintf ( SWPF(wNum,51), L"%g,%g", fXAt, fYAt ); )
		if (hr == S_OK && b3D)
			swprintf ( SWPF(wNum,51), L"%g,%g,%g", fXAt, fYAt, fZAt );
		CCLTRY( strCmd.append ( wNum ) );
		CCLTRY( strCmd.append ( L" size 2" ) );
		CCLTRY ( pTick->writeStr ( strCmd ) );
		}	// if

	// Other no labeling
	else if (hr == S_OK)
		{
		CCLTRY ( pTick->writeStr ( L"unset label 100" ) );
		CCLTRY ( pTick->writeStr ( L"unset object 100" ) );
		}	// else if

	//
	// Canvas size
	//
	if (hr == S_OK && pReq->load ( strRefWidth, vL ) == S_OK)
		{
		adtInt		iW(vL),iH(600);
		WCHAR			wBfrSz[41];
		adtString	strSz ( L"set terminal pngcairo truecolor size " );

		// Height
		if (pReq->load ( strRefHeight, vL ) == S_OK)
			iH = vL;

		// Plots do not look good below a certain size so set a minimum
		if (hr == S_OK && iW < 800)
			{
			iH = (U32)((iH/((float)(iW)))*800);
			iW = 800;
			}	// if
		if (hr == S_OK && iH < 600)
			{
			iW = (U32)((iW/((float)(iH)))*600);
			iH = 600;
			}	// if

		// Set PNG image size
		CCLOK ( swprintf ( SWPF(wBfrSz,41), L"%d,%d", (U32)iW, (U32)iH );)
		CCLTRY( strSz.append ( wBfrSz ) );
		CCLTRY ( pTick->writeStr ( strSz ) );
		} // if

	// Margins
	if (hr == S_OK && !b3D)
		{
		// This will give a predictable area for the plot so that the coordinates
		// of data points can be calculated.
		CCLTRY ( pTick->writeStr ( L"set lmargin at screen 0.15" ) );
		CCLTRY ( pTick->writeStr ( L"set rmargin at screen 0.85" ) );
		CCLTRY ( pTick->writeStr ( L"set bmargin at screen 0.10" ) );
		CCLTRY ( pTick->writeStr ( L"set tmargin at screen 0.90" ) );
		}	// if
	else
		{
		// Margins mess up the 3D plot right now
		CCLTRY ( pTick->writeStr ( L"unset lmargin" ) );
		CCLTRY ( pTick->writeStr ( L"unset rmargin" ) );
		CCLTRY ( pTick->writeStr ( L"unset bmargin" ) );
		CCLTRY ( pTick->writeStr ( L"unset tmargin" ) );
		}	// else

	//
	// Plot title
	//
	if (	hr == S_OK && 
			pReq->load ( strRefTitle, vL ) == S_OK &&
			(strV = vL).length() > 0 )
		{
		CCLTRY ( strV.prepend ( L"set title \"" ) );
		CCLTRY ( strV.append ( L"\"" ) );
		CCLTRY ( pTick->writeStr ( strV ) );
		}	// if
	else
		{
		CCLTRY ( pTick->writeStr ( L"unset title" ) );
		}	// else

	//
	// Plot
	//

	if (hr == S_OK)
		{
		adtString	strCmdBfr;
		WCHAR			wBfrLn[101];

		// Initialize plot command
		CCLOK ( strCmdBfr = (!b3D) ? L"plot " : L"splot"; )

		// XY Plot
		if (hr == S_OK && !b3D)
			{
			// Record command, each column
			for (U32 i = 1;hr == S_OK && i < cnt;++i)
				{
				// Next line
				CCLOK ( swprintf ( SWPF(wBfrLn,101), 
							L"'-' binary record=%d with lines ls %d title \"", (U32)adtInt(iRows), i ); )
				CCLTRY( strCmdBfr.append ( wBfrLn ) );

				// Label for plot, default or specified
				if (hr == S_OK && pDctV[i]->load ( adtString(L"Label"), vL ) == S_OK)
					hr = strCmdBfr.append ( adtString(vL) );
				else if (hr == S_OK)
					{
					// Default
					CCLOK ( swprintf ( SWPF(wBfrLn,101), L"Series %d", i ); )
					CCLOK ( strCmdBfr.append ( wBfrLn ); )
					}	// else
				CCLTRY ( strCmdBfr.append ( L"\"" ) );

				// Separator
				if (hr == S_OK && i+1 < cnt)
					hr = strCmdBfr.append ( L", " );
				}	// for
			}	// if
			
		// XYZ plot
		else if (hr == S_OK)
			{
			// XYZ data
			CCLOK ( swprintf ( SWPF(wBfrLn,101), 
						L"'-' binary record=%d", (U32)adtInt(iRows) ); )
			CCLTRY( strCmdBfr.append ( wBfrLn ) );
			}	// else if

		// Send the command
//			CCLOK ( strCmdBfr = (	L"plot '-' binary record=576 with lines ls 1,"
//											L"'-' binary record=576 with lines ls 2" ); )

//			CCLOK ( dbgprintf ( L"Plot command : %s\r\n", (LPCWSTR)strCmdBfr ); )
//			CCLTRY ( pTick->writeStr ( strCmdBfr ) );

//		CCLOK ( strCmdBfr = ( L"plot '-' binary record=576 with lines" ); )
//		CCLTRY ( pTick->writeStr ( L"plot '-' binary record=1024 using 0:1 with lines,"
//											L"'-' binary record=1024 using 0:2 with lines" ) );

		// Send the command and the data
		CCLOK  ( evPlot.reset(); )
		CCLOK  ( dbgprintf ( L"GnuPlotSrvr::Command:%s\r\n", (LPCWSTR)strCmdBfr ); )
		CCLTRY ( pTick->writeStr ( strCmdBfr ) );
		CCLTRY ( pTick->flush() );

		// GnuPlot wants column-ordered data so take one point
		// from each vector and write it.  
		// NOTE: This is very annonying but the GnuPlot does not 'cache' 
		// previously received data so the X coordinates need to
		// be sent for every plot/Y vector.
		if (!b3D)
			{
			for (U32 v = 1;hr == S_OK && v < cnt && v < MAX_VECTS;++v)
				{
				// Send vector X (Y)
				for (U32 r = 0;hr == S_OK && r < iRows;++r)
					{
					// X value
					fVal = (vF = pValV[0][r]);
					hr = pTick->write ( &fVal, sizeof(float) );

					// Y value
					fVal = (vF = pValV[v][r]);
					hr = pTick->write ( &fVal, sizeof(float) );
					}	// for
				}	// for
			}	// if

		// For 3D plots, feed all 3 columns at once
		else
			{
			// Send vector X,Y,Z
			for (U32 r = 0;hr == S_OK && r < iRows;++r)
				{
				// X value
				fVal = (vF = pValV[0][r]);
				hr = pTick->write ( &fVal, sizeof(float) );

				// Y value
				fVal = (vF = pValV[1][r]);
				hr = pTick->write ( &fVal, sizeof(float) );

				// Z value
				fVal = (vF = pValV[2][r]);
				hr = pTick->write ( &fVal, sizeof(float) );
				}	// for
			}	// else

		}	// if

	// DEBUG plot
//		if (hr == S_OK)
//			{
//			float data[2048];

		// Generate some data
//			for (int i = 0;i<1024;++i)
//				{
//				data[2*i+0] = (float)10*i;

//				data[2*i+1] = (float)cos(i/10.0);
//				}	// for
//			CCLTRY ( pTick->writeStr ( L"plot '-' binary record=1024 with lines" ) );
//			for (int i = 0;hr == S_OK && i < 16;++i)
//				hr = pTick->write ( &data[i*128], 128*sizeof(float) );
//			}	// if

	// Send any remaining data
	CCLTRY ( pTick->flush() );

	// Wait for plot to finish
//		CCLOK  ( evPlot.wait(1000); )
	CCLOK  ( evPlot.wait(10000); )

	// Store the resulting image in the original request
	// to more easily support multiple plot clients.
	if (hr == S_OK && hr_img == S_OK)
		hr = pReq->store ( strRefOnImg, adtIUnknown(pDctImg) );
	else if (pReq != NULL)
		pReq->store ( adtString(L"OnError"), adtInt(hr) );

	// Clean up
	_RELEASE(pVcts);
	_RELEASE(pReq);
	for (U32 v = 0;v < cnt && v < MAX_VECTS;++v)
		{
		_RELEASE(pDctV[v]);
		pValV[v] = NULL;
		}	// for

	// Thread safety
	csPlot.leave();

	return hr;
	}	// plot

HRESULT GnuPlotSrvr :: png_data ( BYTE *pbBfr, U32 len )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when PNG data becomes available.
	//
	//	PARAMETERS
	//		-	pbBfr contains the data
	//		-	len is the amount of available data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Debug
//	dbgprintf ( L"GnuPlotSrvr::png_data:%d bytes\r\n", len );

	// Error handling
	if (hr == S_OK && setjmp(png_jmpbuf(png_ptr)))
		{
		dbgprintf ( L"GnuPlotSrvr::png_data:Error!\r\n" );
		hr = E_UNEXPECTED;
		return hr;
		}	// if

	// Feed data to PNG library
	png_process_data ( png_ptr, info_ptr, pbBfr, len );

	return hr;
	}	// png_data

HRESULT GnuPlotSrvr :: png_end ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when a PNG has finished loading.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Debug
	dbgprintf ( L"GnuPlotSrvr::png_end:0x%x\r\n", hr_img );

	// Error ?
	CCLTRYE ( hr_img == S_OK, hr_img );

	// In case object is waiting for completion of plot
	evPlot.signal();

	// Turn around and prepare for the next image
	CCLTRY ( png_uninit() );
	CCLTRY ( png_init() );

	return hr;
	}	// png_end

HRESULT GnuPlotSrvr :: png_init ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize/prepare PNG state for decoding an image.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// State check
	if (png_ptr != NULL)
		return S_OK;

	// Prepare PNG structures for reading
	CCLTRYE ( (png_ptr = png_create_read_struct ( PNG_LIBPNG_VER_STRING,
					png_voidp_NULL, png_error_ptr_NULL, png_error_ptr_NULL ))
					!= NULL, E_OUTOFMEMORY );
	CCLTRYE ( (info_ptr = png_create_info_struct ( png_ptr )) != NULL,
					E_OUTOFMEMORY );

	// Error handling
	if (hr == S_OK && setjmp(png_jmpbuf(png_ptr)))
		hr = E_UNEXPECTED;

	// Set up the callbacks for progressive reading
	CCLOK ( png_set_progressive_read_fn ( png_ptr, this, png_progressive_info,
				png_progressive_row, png_progressive_end ); )
		
	return hr;
	}	// png_init

HRESULT GnuPlotSrvr :: png_uninit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Free PNG state.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// State check
	if (png_ptr == NULL)
		return S_OK;

	// Error handling
	if (hr == S_OK && setjmp(png_jmpbuf(png_ptr)))
		hr = E_UNEXPECTED;

	// PNG clean up
	if (png_ptr != NULL)
		{
		png_destroy_read_struct ( &png_ptr, &info_ptr, NULL );
		png_ptr = NULL;
		}	// if

	return hr;
	}	// png_uninit

HRESULT GnuPlotSrvr :: run ( bool bRun )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Start/stop GnuPlotSrvring
	//
	//	PARAMETERS
	//		-	bRun is true to run, false to stop
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;

	// Start
	if (bRun && pThrd != NULL)
		{
		// Attempt to start link with GnuPlotSrvr
		CCLTRY ( start() );

		// Defaults, can/will be changed at run-time
		CCLTRY ( pTick->writeStr ( L"set terminal pngcairo truecolor size 800,600 enhanced font 'Verdana,9'" ) );
//		CCLTRY ( pTick->writeStr ( L"set terminal png truecolor size 800,600 enhanced font 'Verdana,9'" ) );
		CCLTRY ( pTick->writeStr ( L"set key top right" ) );
		CCLTRY ( pTick->writeStr ( L"set grid xtics" ) );
		CCLTRY ( pTick->writeStr ( L"set grid ytics" ) );

		// Formatting
//		CCLTRY ( pTick->writeStr ( L"set format x '%.1s%c'" ) );
//		CCLTRY ( pTick->writeStr ( L"set format y '%.1s%c'" ) );
		CCLTRY ( pTick->writeStr ( L"set xlabel 'X-Axis'" ) );
		CCLTRY ( pTick->writeStr ( L"set ylabel 'Y-Axis'" ) );
//		CCLTRY ( pTick->writeStr ( L"set key off" ) );
		CCLTRY ( pTick->writeStr ( L"set autoscale xy" ) );

		// General prettiness
		CCLTRY ( pTick->writeStr ( L"set xlabel 'X-Axis'" ) );
		CCLTRY ( pTick->writeStr ( L"set ylabel 'Y-Axis'" ) );
//		CCLTRY ( pTick->writeStr ( L"set border  grid back ls 12" ) );

		// Axis
		CCLTRY ( pTick->writeStr ( L"set style line 11 lc rgb '#808080' lt 1" ) );
		CCLTRY ( pTick->writeStr ( L"set border 3 back ls 11" ) );
		CCLTRY ( pTick->writeStr ( L"set tics nomirror" ) );
		CCLTRY ( pTick->writeStr ( L"set style line 12 lc rgb '#808080' lt 0 lw 1" ) );
		CCLTRY ( pTick->writeStr ( L"set grid back ls 12" ) );
		CCLTRY ( pTick->writeStr ( L"set style data lines" ) );
//		CCLTRY ( pTick->writeStr ( L"set dgrid3d 30,30" ) );
		CCLTRY ( pTick->writeStr ( L"set hidden3d" ) );
		CCLTRY ( pTick->writeStr ( L"set style data impulses" ) );
//		CCLTRY ( pTick->writeStr ( L"set pm3d" ) );

		// Color definitions
//		CCLTRY ( pTick->writeStr ( L"set style line 1 lc rgb '#808080' pt 6 ps 1 lt 1 lw 2" ) );
//		CCLTRY ( pTick->writeStr ( L"set style line 2 lc rgb '#0e1a8b' pt 1 ps 1 lt 1 lw 1" ) );
		CCLTRY ( pTick->writeStr ( L"set style line 1 linecolor '#0060ad' linetype 1 linewidth 2" ) );
		CCLTRY ( pTick->writeStr ( L"set style line 2 linecolor '#dd181f' linetype 1 linewidth 2" ) );
		CCLTRY ( pTick->writeStr ( L"set style line 3 linecolor '#5e9c36' linetype 1 linewidth 2" ) );

		// 'Set output' with no parameter means output goes to STDOUT.
		// PNG images will be streamed directly to this object.
		CCLTRY ( pTick->writeStr ( L"set output" ) );

		// Ensure output is written
		CCLTRY ( pTick->flush() );
		}	// if

	// Stop
	else if (!bRun)
		{
		// Ensure link is shutdown with GnuPlotSrvr
		stop();
		}	// else if

	return hr;
	}	// run

HRESULT GnuPlotSrvr :: start ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Start GnuPlotSrvring
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;
	STARTUPINFO	si;
	WCHAR			wPipe[200];

	// Ensure object is ready to decode PNG images
	CCLTRY ( png_init() );

	// GnuPlotSrvr uses stdin and stdout for I/O so create 
	// the input and output pipes that the process will use

	// Use named pipes so that overlapped I/O can be used.

	// STDIN
	CCLOK   ( swprintf ( SWPF(wPipe,200), L"\\\\.\\Pipe\\GnuPlotSrvr.%08x.%08x",
								GetCurrentProcessId(), gnuPipeCnt++ ); )
	CCLTRYE ( (hStdIn = CreateNamedPipe ( wPipe, 
								PIPE_ACCESS_INBOUND|FILE_FLAG_OVERLAPPED, 
								PIPE_TYPE_BYTE | PIPE_WAIT, 1, 8192, 8192, 0, NULL ))
					!= INVALID_HANDLE_VALUE, GetLastError() );
	CCLTRYE ( SetHandleInformation ( hStdIn, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT )
					== TRUE, GetLastError() );

	// Open handle for writing to process STDIN
	CCLTRYE	( (hWrIn = CreateFile ( wPipe, GENERIC_WRITE, 0, 0, OPEN_EXISTING,
					FILE_FLAG_OVERLAPPED, NULL )) != INVALID_HANDLE_VALUE, GetLastError() );

	// STDOUT
	CCLOK   ( swprintf ( SWPF(wPipe,200), L"\\\\.\\Pipe\\GnuPlotSrvr.%08x.%08x",
								GetCurrentProcessId(), gnuPipeCnt++ ); )
	CCLTRYE ( (hStdOut = CreateNamedPipe ( wPipe, 
								PIPE_ACCESS_OUTBOUND|FILE_FLAG_OVERLAPPED, 
								PIPE_TYPE_BYTE | PIPE_WAIT, 1, 8192, 8192, 0, NULL ))
					!= INVALID_HANDLE_VALUE, GetLastError() );
	CCLTRYE ( SetHandleInformation ( hStdOut, HANDLE_FLAG_INHERIT, HANDLE_FLAG_INHERIT ) 
					== TRUE, GetLastError() );

	// Open handle for read from process STDOUT
	CCLTRYE	( (hRdOut = CreateFile ( wPipe, GENERIC_READ, 0, 0, OPEN_EXISTING,
					FILE_FLAG_OVERLAPPED, NULL )) != INVALID_HANDLE_VALUE, GetLastError() );

	// A GNU plot process for this object can now be started using the pipes above.
	if (hr == S_OK)
		{
		// Startup information for process
		memset ( &gnuInfo, 0, sizeof(gnuInfo) );
		memset ( &si, 0, sizeof(si) );
		si.cb				= sizeof(si);
		si.dwXSize		= 800;
		si.dwYSize		= 600;
		si.hStdInput	= hStdIn;
		si.hStdOutput	= hStdOut;
		si.hStdError	= hStdOut;
		si.wShowWindow	= SW_SHOWNORMAL;
//		si.wShowWindow	= SW_HIDE;
		si.dwFlags		=	STARTF_USESTDHANDLES | STARTF_USEPOSITION | STARTF_USESIZE |
								STARTF_USESHOWWINDOW;

		// Start process.  Currently using current default install directory, use in path instead ?
		CCLTRYE ( CreateProcess ( PATH_GNUPLOT, NULL, NULL, NULL, TRUE, 
						DETACHED_PROCESS, NULL, NULL,&si, &gnuInfo ) == TRUE,
						GetLastError() );
		}	// if

	// Start the I/O thread

	// Create worker thread object
	CCLTRYE ( (pTick = new GnuPlotSrvrt ( this )) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pTick->AddRef(); )
	CCLTRY  ( pTick->construct() );

	// Start the playback capture thread
	CCLTRY ( COCREATE ( L"Sys.Thread", IID_IThread, &pThrd ) );
	CCLTRY ( pThrd->threadStart ( pTick, 5000 ) );

	return hr;
	}	// start

HRESULT GnuPlotSrvr :: stop ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Stop GnuPlotSrvring
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Issue quit command to GnuPlotSrvr to shutdown
	if (pTick != NULL)
		pTick->writeStr ( L"quit" );

	// Shut down I/O thread
	if (pThrd != NULL)
		{
		pThrd->threadStop(10000);
		_RELEASE(pThrd);
		_RELEASE(pTick);
		}	// if

	// PNG
	png_uninit();

	// Close process
	if (gnuInfo.hProcess != NULL)
		{
		// Necessary just in case ?
		TerminateProcess ( gnuInfo.hProcess, 0 );

		// Clean up
		CloseHandle ( gnuInfo.hProcess );
		memset ( &gnuInfo, 0, sizeof(gnuInfo) );
		}	// if

	// Close handles
	if (hRdOut != INVALID_HANDLE_VALUE)
		{
		CloseHandle ( hRdOut );
		hRdOut = INVALID_HANDLE_VALUE;
		}	// if
	if (hWrIn != INVALID_HANDLE_VALUE)
		{
		CloseHandle ( hWrIn );
		hWrIn = INVALID_HANDLE_VALUE;
		}	// if
	if (hStdIn != INVALID_HANDLE_VALUE)
		{
		CloseHandle ( hStdIn );
		hStdIn = INVALID_HANDLE_VALUE;
		}	// if
	if (hStdOut != INVALID_HANDLE_VALUE)
		{
		CloseHandle ( hStdOut );
		hStdOut = INVALID_HANDLE_VALUE;
		}	// if

	return hr;
	}	// stop

////////////////
// GnuPlotSrvrt
////////////////

GnuPlotSrvrt :: GnuPlotSrvrt ( GnuPlotSrvr *_pParent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pParent is the parent object
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pParent	= _pParent;
	bRun		= true;
	memset ( &hevW, 0, sizeof(hevW) );
	hevWr		= NULL;
	uBfrWr	= 0;
	}	// GnuPlotSrvrt

HRESULT GnuPlotSrvrt :: construct ( void )
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

	// Create events for read
	CCLTRYE ( (hevW[0] = CreateEvent(NULL, FALSE, FALSE, NULL)) != NULL,
					GetLastError() );

	// Create event used to signal worker thread
	CCLTRYE ( (hevW[1] = CreateEvent(NULL, FALSE, FALSE, NULL)) != NULL,
					GetLastError() );

	// Create event to use for writes
	CCLTRYE ( (hevWr = CreateEvent(NULL, FALSE, FALSE, NULL)) != NULL,
					GetLastError() );

	return hr;
	}	// construct

void GnuPlotSrvrt :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	for (int i = 0;i < sizeof(hevW)/sizeof(hevW[0]);++i)
		if (hevW[i] != NULL)
			CloseHandle ( hevW[i] );
	if (hevWr != NULL)
		CloseHandle ( hevWr );
	}	// destruct

HRESULT GnuPlotSrvrt :: flush ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write the buffered output to the STDIN of the process.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	DWORD			dwWr		= 0;
	DWORD			dwLeft	= uBfrWr;
	U8				*pcBfr	= bBfrWr;
	OVERLAPPED	ov;
	BOOL			bRet;

	// Write data until all bytes are written or until error
	dbgprintf ( L"GnuPlotSrvrt::flush:%d bytes {\r\n", dwLeft );
	while (hr == S_OK && dwLeft > 0)
		{
		// Peform write
		memset ( &ov, 0, sizeof(ov) );
		ov.hEvent	= hevWr;
		bRet			= WriteFile ( pParent->hWrIn, pcBfr, dwLeft, NULL, &ov );
		hr				= (bRet || GetLastError() == ERROR_IO_PENDING) ? S_OK : GetLastError();

		// If write is pending, wait for finish
		if (hr == S_OK && !bRet)
			{
			// Wait for completion
			dwWr = WaitForSingleObject ( hevWr, 5000 );

			// Timeout ?
			hr = (dwWr == WAIT_OBJECT_0) ? S_OK : dwWr;
			}	// if

		// Check result
		if (hr == S_OK)
			{
			// Result of overlapped I/O
			bRet = GetOverlappedResult ( pParent->hWrIn, &ov, &dwWr, FALSE );
			hr = (bRet) ? S_OK : GetLastError();

			// Update state
			if (hr == S_OK)
				{
				dwLeft -= dwWr;
				pcBfr += dwWr;
				}	// if

			}	// if

		}	// while

	// Debug
	dbgprintf ( L"} GnuPlotSrvrt::flush:hr 0x%x:%d remaining of %d\r\n", hr, dwLeft, uBfrWr );
	if (hr != S_OK)
		dbgprintf ( L"GnuPlotSrvrt::flush:Failed 0x%x\r\n", hr );

	// Clean up
	uBfrWr = 0;

	return hr;
	}	// flush

HRESULT GnuPlotSrvrt :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Signal end
	bRun = false;
	SetEvent ( hevW[1] );

	return S_OK;
	}	// tickAbort

HRESULT GnuPlotSrvrt :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	DWORD		dwRet		= 0;
	bool		bStr		= false;

	// Wait for read or run events to be signalled
	dwRet = WaitForMultipleObjects ( sizeof(hevW)/sizeof(hevW[0]), 
												hevW, FALSE, INFINITE );

	// Process result
	switch ( dwRet )
		{
		// STDOUT read
		case WAIT_OBJECT_0 :
			// Result of reads
			while (hr == S_OK && GetOverlappedResult ( pParent->hRdOut, &ovRd, &dwRet, FALSE ))
				{
//				dbgprintf ( L"GnuPlotSrvrt::tick:Read %d bytes\r\n", dwRet );

				// PNG images are continously sent so feed data into PNG processor.
				// If there is a syntax error the first couple of bytes 
				// will be '\r\n', detect this (GnuPlotSrvr responding with ASCII).
				// TODO: How to handle this 'cleanly' and still recover for the next command 
				if (bStr || (bBfrRd[0] == '\r' && bBfrRd[1] == '\n') || (dwRet == 1 && bBfrRd[0] == ' '))
					{
					// TODO: Read response
					bBfrRd[dwRet] = '\0';
					dbgprintf ( L"GnuPlotSrvrt::%S\r\n", bBfrRd );
					bStr = true;
					}	// if

				// PNG processing
				else
					pParent->png_data ( bBfrRd, dwRet );

				// Issue read for next block
				memset ( &ovRd, 0, sizeof(ovRd) );
				ovRd.hEvent = hevW[0];
				if (!ReadFile ( pParent->hRdOut, bBfrRd, sizeof(bBfrRd), NULL, &ovRd ))
					hr = (GetLastError() == ERROR_IO_PENDING) ? S_OK : GetLastError();
				}	// if
			break;

		// Other
		default :
			hr = E_UNEXPECTED;
		}	// switch

	return (hr == S_OK && bRun) ? S_OK : S_FALSE;
	}	// tick

HRESULT GnuPlotSrvrt :: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr				= S_OK;
	BOOL		bRet;

	// Begin by hanging read off of STDOUT
	if (hr == S_OK)
		{
		memset ( &ovRd, 0, sizeof(ovRd) );
		ovRd.hEvent = hevW[0];
		bRet	= ReadFile ( pParent->hRdOut, bBfrRd, sizeof(bBfrRd), NULL, &ovRd );
		hr		= (bRet || GetLastError() == ERROR_IO_PENDING || 
					GetLastError() == ERROR_PIPE_LISTENING) ? S_OK : GetLastError();
		}	// if

	return hr;
	}	// tickBegin

HRESULT GnuPlotSrvrt :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;

	// Cancel pending I/O
	CancelIo ( pParent->hRdOut );

	return hr;
	}	// tickEnd

HRESULT GnuPlotSrvrt :: write ( const VOID *pvBfr, U32 len )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write a block of data the STDIN of the process.
	//
	//	PARAMETERS
	//		-	pvBfr contains the data
	//		-	len is the length to write
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	DWORD			dwWr		= 0;
	DWORD			dwLeft	= len;
	U8				*pcBfr	= (U8 *) pvBfr;

	// Write data until all bytes are written or until error
	while (hr == S_OK && dwLeft > 0)
		{
		// To avoid lots of small system calls, output is buffered until
		// buffer is full 
		dwWr = (dwLeft < sizeof(bBfrWr)-uBfrWr) ? dwLeft : (sizeof(bBfrWr)-uBfrWr);
		if (dwWr > 0)
			{
			// Copy segment of data that fits into buffer
			memcpy ( &(bBfrWr[uBfrWr]), pcBfr, dwWr );
			uBfrWr	+= dwWr;
			pcBfr		+= dwWr;
			dwLeft	-= dwWr;
			}	// if

		// Flush buffer if full
		else
			hr = flush();
		}	// while

	return hr;
	}	// write

HRESULT GnuPlotSrvrt :: writeStr ( const WCHAR *pwBfr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Write a string to the STDIN of the process.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	char			*pcAscii	= NULL;
	adtString	strWr(pwBfr);

	// Create an ASCII version of the string for writes
	CCLTRY ( strWr.append ( L"\n" ) );
	CCLTRY ( strWr.toAscii ( &pcAscii ) );

	// Write string data
	CCLTRY ( write ( pcAscii, (U32)strlen(pcAscii) ) );

	// Clean up
	_FREEMEM(pcAscii);

	return hr;
	}	// writeStr

//
// PNG callbacks
//

static void PNGAPI png_progressive_info ( png_structp png_ptr, png_infop info_ptr )
	{
	HRESULT	hr = S_OK;
	U32		w,h,bpp,type;

	// Supplied ptr. is to owner object
	GnuPlotSrvr *pThis = (GnuPlotSrvr *) png_get_io_ptr ( png_ptr );

	// Enough has been read to know the header, prepare the image parameters.
	w		= png_get_image_width(png_ptr,info_ptr);
	h		= png_get_image_height(png_ptr,info_ptr);
	bpp	= png_get_channels(png_ptr,info_ptr)*png_get_bit_depth(png_ptr,info_ptr);
	type	= png_get_color_type(png_ptr,info_ptr);
	dbgprintf ( L"png_progressive_info:pThis %p:%d X %d X %d (0x%x)\r\n", pThis, w, h, bpp, type );

	// Compute stride and format based on header info
	switch (type)
		{
		case PNG_COLOR_TYPE_RGB :
			pThis->uStride	= w*3;
			CCLTRY ( pThis->pDctImg->store ( strRefFormat, strRefBGR ) );
			break;
		case PNG_COLOR_TYPE_RGB_ALPHA :
			pThis->uStride	= w*4;
			CCLTRY ( pThis->pDctImg->store ( strRefFormat, strRefBGRA ) );
			break;
		default :
			hr = E_NOTIMPL;
		}	// switch

	// Update image dictionary with layout of new image
	CCLTRY ( pThis->pDctImg->store ( strRefWidth, adtInt(w) ) );
	CCLTRY ( pThis->pDctImg->store ( strRefHeight, adtInt(h) ) );

	// Size the image bits to hold all the data
	CCLTRY ( pThis->pBits->setSize ( pThis->uStride*h ) );
	CCLTRY ( pThis->pBits->lock ( 0, 0, &(pThis->pvBits), NULL ));

	// Update
	pThis->hr_img = hr;

	// Tell library to start reading the PNG image
	png_start_read_image(png_ptr);
	}	// png_progressive_info

static void PNGAPI png_progressive_row ( png_structp png_ptr, png_bytep data,
														png_uint_32 row, int pass )
	{
	U8	*pcBits	= NULL;

	// Supplied ptr. is to owner object
	GnuPlotSrvr *pThis = (GnuPlotSrvr *) png_get_io_ptr ( png_ptr );
//	dbgprintf ( L"png_progressive_row:pThis %p:data %p row %d pass %d\r\n", 
//					pThis, data, row, pass );

	// Copy data to image bits
	if (pThis->hr_img == S_OK)
		{
		// Compute row address
		pcBits = ((U8 *)(pThis->pvBits)) + pThis->uStride*row;

		// Copy row
		memcpy ( pcBits, data, pThis->uStride );
		}	// if

	}	// png_progressive_row

static void PNGAPI png_progressive_end ( png_structp png_ptr, png_infop info_ptr )
	{
	// Supplied ptr. is to owner object
	GnuPlotSrvr *pThis = (GnuPlotSrvr *) png_get_io_ptr ( png_ptr );
	dbgprintf ( L"png_progressive_end:pThis %p\r\n", pThis );

	// Clean up bits
	if (pThis->pvBits != NULL)
		{
		pThis->pBits->unlock(pThis->pvBits);
		pThis->pvBits = NULL;
		}	// if

	// Signal end of image
	pThis->png_end();
	}	// png_progressive_end


