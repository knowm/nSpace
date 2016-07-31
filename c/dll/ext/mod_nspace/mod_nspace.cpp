////////////////////////////////////////////////////////////////////////
//
//								MOD_NSPACE.CPP
//
//				Main file for the nSpace Apache module
//
////////////////////////////////////////////////////////////////////////

// Apache
#include <httpd.h>
#include <http_config.h>
#include <http_protocol.h>
#include <ap_config.h>

// Prototypes
void	nspace_register_hooks ( apr_pool_t *p );
int	nspace_handler ( request_rec *r );

//
// Dispatch list
//
extern "C"
module AP_MODULE_DECLARE_DATA nspace_module =
	{
	STANDARD20_MODULE_STUFF,
	NULL,														// create per-dir
	NULL,														// merge per-dir
	NULL,														// create per-server
	NULL,														// merge per-server
	NULL,														// table of config file commands
	nspace_register_hooks								// Register hooks
	};

int nspace_handler ( request_rec *r )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Handle incoming request.
	//
	//	PARAMETERS
	//		-	r is the request
	//
	//	RETURN VALUE
	//		Request result
	//
	////////////////////////////////////////////////////////////////////////

	// Request for this handler ?
	if (	r->handler == NULL || strcmp ( r->handler, "nspace" ) )
		return DECLINED;

	// Debug
	OutputDebugStringA ( r->method );
	OutputDebugStringA ( r->filename  );
	OutputDebugStringA ( r->uri );
	OutputDebugStringA ( r->args );
	OutputDebugStringA ( r->useragent_ip );

	return DECLINED;
	}	// nspace_handler

void nspace_register_hooks ( apr_pool_t *p )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Register the calbacks with the server.
	//
	//	PARAMETERS
	//		-	p is the allocation pool
	////////////////////////////////////////////////////////////////////////
	OutputDebugStringA("nspace_register_hooks");

	// Request handler
	ap_hook_handler ( nspace_handler, NULL, NULL, APR_HOOK_MIDDLE );

	}	// nspace_register_hooks

