////////////////////////////////////////////////////////////////////////
//
//									EVENT.CPP
//
//								Event object
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#if      __unix__ || __APPLE__
#include	<sys/time.h>
#endif

sysEvent :: sysEvent ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	// Wait until 'init' for creation to save resources
	hEv	= NULL;
	#elif		__unix__ || __APPLE__
	pthread_mutex_init ( &mtx, NULL );
	pthread_cond_init ( &cnd, NULL );
	sig = false;
	#endif
	}	// sysEvent

sysEvent :: ~sysEvent ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Destructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL) CloseHandle ( hEv );
	#elif		__unix__ || __APPLE__
	pthread_mutex_destroy ( &mtx );
	pthread_cond_destroy ( &cnd );
	#endif
	}	// ~sysEvent

bool sysEvent :: init ( BOOL bManRst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Initialize the event for use
	//! \param bManRst is true for a manually reset, false for automatic 
	//				reset (default)
	//! \return true if successful
	//
	////////////////////////////////////////////////////////////////////////
	bManual = bManRst;
	#ifdef	_WIN32
	if (hEv != NULL)
		return TRUE;
	else
		return ( (hEv = CreateEvent ( NULL, bManual, FALSE, NULL )) != NULL );
	#elif		__unix__ || __APPLE__
	return 1;
	#else
	return FALSE;
	#endif
	}	// init

bool sysEvent :: reset ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Resets the event signaled state.
	//! \return true if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL) ResetEvent ( hEv );
	#elif		__unix__ || __APPLE__
	pthread_mutex_lock ( &mtx );
	sig = false;
	pthread_mutex_unlock ( &mtx );
	#endif
	return 1;
	}	// reset

bool sysEvent :: signal ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Signal the event
	//! \return true if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL) SetEvent ( hEv );
	#elif		__unix__ || __APPLE__
	pthread_mutex_lock ( &mtx );
	sig = true;
	pthread_mutex_unlock ( &mtx );
	pthread_cond_signal ( &cnd );
	#endif
	return 1;
	}	// signal

bool sysEvent :: wait ( U32 to )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Wait until event signaled or times out.
	//! \param to is the timeout value (in ms, -1 = no timeout)
	//! \return true if signaled, false on timeout
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if ( (hEv != NULL) && (WaitForSingleObject ( hEv, to ) == WAIT_OBJECT_0) )
		return TRUE;
	else
		return FALSE;
	#elif		__unix__ || __APPLE__
	BOOL	bRet = 1;

	// Perform pthread equivalent for signal waiting
	if (pthread_mutex_lock ( &mtx ) == 0)
		{
		// Wait for signal
		while (bRet && !sig)
			{
			// Wait
			if (to == (U32)-1)
				bRet = (pthread_cond_wait ( &cnd, &mtx ) == 0);
			else if (to != (U32)-1)
				{
				int ret;

				// pthread waits are absolute time
				// Watch difference between 'timeval' and 'timespec'
				struct timespec 	ts;
				struct timeval		tv;

				// Now
				gettimeofday ( &tv, NULL );

				// Future
				ts.tv_sec	= tv.tv_sec + (to/1000);
				ts.tv_nsec	= (tv.tv_usec*1000) + ((to % 1000)*1000*1000);
				if (ts.tv_nsec >= 1000000000)
					{
					ts.tv_sec++;
					ts.tv_nsec -= 1000000000;
					}	// if

				// Wait
				bRet = ((ret = pthread_cond_timedwait ( &cnd, &mtx, &ts )) == 0);
				}	// else

			}	// while

		// If success and not manual reset, clear signal
		if (bRet && !bManual)
			sig = false;

		// Clean up
		pthread_mutex_unlock ( &mtx );
		}	// if
	return bRet;
	#else
	return FALSE;
	#endif
	}	// wait


