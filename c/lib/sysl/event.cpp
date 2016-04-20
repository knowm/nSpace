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

sysEvent :: sysEvent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	// Wait until 'init' for creation to save resources
	hEv	= NULL;
	#elif		__unix__ || __APPLE__
	pthread_mutex_init ( &mtx, NULL );
	pthread_cond_init ( &cnd, NULL );
	#endif
	}	// sysEvent

sysEvent :: ~sysEvent ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
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
	//	PURPOSE
	//		-	Initializes event
	//
	//	PARAMETERS
	//		-	bManRst is false if the event should auto-reset (default), 
	//			true for manual reset.
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL)
		return TRUE;
	else
		return ( (hEv = CreateEvent ( NULL, bManRst, FALSE, NULL )) != NULL );
	#elif		__unix__ || __APPLE__
	return 1;
	#else
	return FALSE;
	#endif
	}	// init

bool sysEvent :: reset ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Reset the events 'signal' state.
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL) ResetEvent ( hEv );
	#elif		__unix__ || __APPLE__
	// Simulate reset
	signal();
	wait(0);
	#endif
	return 1;
	}	// reset

bool sysEvent :: signal ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Signal event.
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hEv != NULL) SetEvent ( hEv );
	#elif		__unix__ || __APPLE__
	pthread_cond_signal ( &cnd );
	#endif
	return 1;
	}	// signal

bool sysEvent :: wait ( U32 to )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Wait until event signaled or timeout.
	//
	//	PARAMETERS
	//		-	to is the timeout value (ms, -1 = no timeout)
	//
	//	RETURN VALUE
	//		TRUE if signaled
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if ( (hEv != NULL) && (WaitForSingleObject ( hEv, to ) == WAIT_OBJECT_0) )
		return TRUE;
	else
		return FALSE;
	#elif		__unix__ || __APPLE__
	BOOL	bRet = 0;

	// Perform pthread equivalent for signal waiting
	if (pthread_mutex_lock ( &mtx ) == 0)
		{
		// Wait
		if (to == (U32)-1 && pthread_cond_wait ( &cnd, &mtx ) == 0)
			bRet = 1;
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

		// Clean up
		pthread_mutex_unlock ( &mtx );
		}	// if
	return bRet;
	#else
	return FALSE;
	#endif
	}	// wait


