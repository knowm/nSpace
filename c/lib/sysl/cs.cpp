////////////////////////////////////////////////////////////////////////
//
//									CS.CPP
//
//							Critical section object
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"

// Writing custom critical section code because certain operating systems
// (Windows CE) always create a critical section during initialization,
// unlike the 'lazy resource allocation' methodology (Windows XP) of just
// creating it when necessary.  Creating it all the time ends up creating
// way too many (unnecessary) resources.

sysCS :: sysCS ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	lOwnerThrd	= 0;
	uLockCount	= 0;
	uWaitCount	= 0;
	}	// sysCS

sysCS :: ~sysCS ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// ~sysCS

bool sysCS :: enter ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Enters critical section.
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	LONG			lThrd = GetCurrentThreadId();
	#elif		__APPLE__	|| __unix__
	pthread_t	lThrd = pthread_self();
	#endif

	// If thread already owner, increment count
	if (lOwnerThrd == lThrd)
		++uLockCount;

	// Attempt to set current thread as owner if there is no owner
	else if (InterlockedCompareExchange ( &lOwnerThrd, lThrd, 0 ) == 0)
		{
		// Current thread is now owner
		uLockCount = 1;
		}	// else if

	// Current owner is not current thread, must wait for access.
	else
		{
		// Using 'lazy' resource allocation to avoid creation for
		// critical sections that never have a contention.
		#ifdef	_WIN32
		if (hEv == NULL)
			{
			HANDLE	hEvNew;

			// Create the resource
			hEvNew = CreateEvent ( NULL, FALSE, FALSE, NULL );

			// Store in object
			#if	!defined(_WIN32_WCE) || (_WIN32_WCE >= 400)
			if (hEvNew != NULL && InterlockedCompareExchangePointer ( &hEv, hEvNew, 0 ) != 0)
			#else
			if (hEvNew != NULL && InterlockedCompareExchange ( &hEv, hEvNew, 0 ) != 0)
			#endif
				{
				// Previous value was not NULL so another thread beat us to creation
				CloseHandle ( hEvNew );
				}	// if
			}	// if
		#endif

		// Wait count
		InterlockedIncrement ( &uWaitCount );

		// Wait for succesful ownership acqusition of critical section.
		while (InterlockedCompareExchange ( &lOwnerThrd, lThrd, 0 ) != 0)
			{
			// Handle abnormal termination of thread with lock on critical section.
			#ifdef	_WIN32
			if (WaitForSingleObject ( hEv, 500 ) == WAIT_TIMEOUT)
				{
				// Owner still a valid, running thread ?
				LONG		lThrdChk = lOwnerThrd;
				HANDLE	hThrdChk;
				DWORD		ec;

				// Owner still valid after timeout ?
				if (lThrdChk != NULL)
					{
					#if		defined(UNDER_CE)
					// Under Windows CE, the thread ID is also the thread handle
					hThrdChk	= (HANDLE) lThrdChk;
					#else
					// Non-Windows CE, obtain thread handle from ID.
					hThrdChk = OpenThread ( THREAD_QUERY_INFORMATION, FALSE, lThrdChk );
					#endif

					// Get exit code of thread
					if (	hThrdChk == NULL								||
							!GetExitCodeThread ( hThrdChk, &ec )	||
							ec != STILL_ACTIVE )
						{
						// Reclaim critical section
						dbgprintf ( L"sysCS::enter:WARNING:Critical section abandoned by thread %d\r\n", lThrdChk );
						InterlockedCompareExchange ( &lOwnerThrd, 0, lThrdChk );
						}	// if

					// Clean up
					#if	!defined(UNDER_CE)
					if (hThrdChk != NULL)
						CloseHandle ( hThrdChk );
					#endif
					}	// if
				}	// if
			#elif		__unix__ || __APPLE__
			// TODO: Abnormal termination detection as with Win32
			if (wait ( 5000 ) != 1)
				dbgprintf ( L"sysCS::enter:WARNING:Mutex has been held for 5 seconds\r\n" );
			#endif
			}	// while

		// Current thread is now owner
		uLockCount = 1;

		// Wait count
		InterlockedDecrement ( &uWaitCount );
		}	// else

	return 1;
	}	// enter

bool sysCS :: leave ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Leaves critical section.
	//
	//	RETURN VALUE
	//		TRUE if successful
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	LONG			lThrd = GetCurrentThreadId();
	#elif		__APPLE__ || __unix__
	pthread_t	lThrd = pthread_self();
	#endif

	// Current thread owner ?
	if (lThrd != lOwnerThrd)
		{
		// Should not happen
		dbgprintf ( L"sysCS::leave:Non-owner thread releasing critical section (0x%x,0x%x)\r\n",
							lThrd, lOwnerThrd );
		#ifdef	_WIN32
		DebugBreak();
		#endif
		return 1;
		}	// if

	// More than one ?
	if (uLockCount > 1)
		--uLockCount;

	// Critical section free
	else
		{
		// This should not happen!
		if (uLockCount == 0)
			{
			dbgprintf ( L"sysCS::leave:WARNING:No lock count (lThrd 0x%x,lOwnerThrd 0x%x)\r\n",
							lThrd, lOwnerThrd );
			// Debug
			if (lThrd != lOwnerThrd)
				{
				#ifdef	_WIN32
				DebugBreak();
				#endif
				}	// if
			}	// if

		// No owner
		lOwnerThrd 	= 0;
		uLockCount	= 0;

		// Release potential contentions
		if (uWaitCount)
			signal();
		}	// else

	return 1;
	}	// leave

