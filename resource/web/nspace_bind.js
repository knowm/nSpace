////////////////////////////////////////////////////////////////////////
//
//									NSPACE_BIND.JS
//
//		nSpace script to bind existing HTML elements to nSpace web 
//		sockets server.
//
////////////////////////////////////////////////////////////////////////

// Create new instance of this object immediately
new function ()
	{
	var	ws = null;										// WebSocket object

	nSpace_bind =
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Object to bind HTML elements on page to nSpace server.
		//
		////////////////////////////////////////////////////////////////////////

		init : function() 
			{ 
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Initialize state of the binding object
			//
			////////////////////////////////////////////////////////////////////////

			// Establish connection to webserver
			ws = new WebSocket("ws://localhost:8080/nspace");
			ws.onopen		= this.onOpen;
			ws.onmessage	= this.onMessage;

			// Enumerate all of the elements on the page
			var
			elems = document.getElementsByTagName("*");

			// Find every element that specifies and nSpace path attribute.
			for (var i = 0, n = elems.length;i < n;++i)
				{
//				if (elems[i].getAttribute("data-npath") != null)
//					alert ( elems[i].getAttribute("data-npath") );
				}	// for
//			alert ( "YouCanByteMeNow!!!!!" );
//			this.open();
			},

		open : function()
			{
//			alert ( "YouCanByteMeNow!!" );
			},

		onMessage : function(event)
			{
			// Parse the incoming XML
			parser= new DOMParser();
			xmlDoc= parser.parseFromString(event.data,"text/xml");
			var 
			elems	= xmlDoc.getElementsByTagName("*");
//			alert ( elems.length );
			},

		onOpen : function ()
			{
			ws.send("<Dictionary><Value>You</Value><Value Type=\"Double\">3.14159265358979323</Value></Dictionary>");
//			alert ( "Open!!" );
			}

		}	// nSpace_bind

	}	// function

// Execute main on document loaded
document.onreadystatechange = function ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the documents ready state has changed.
	//
	////////////////////////////////////////////////////////////////////////

	// Document ready ?
	if (document.readyState == "complete")
		nSpace_bind.init();		
	}	// onreadystatechange
