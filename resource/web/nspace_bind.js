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
	var	ws			= null;								// WebSocket object
	var	nElems	= {};									// Bounded element states

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

			// Parser
			nSpaceXML.init();

			// URL for hosting document
			loc	= window.location;

			// Generate websocket string
			url	= "ws://"+loc.host+"/nspace/";

			// Establish connection to webserver
			ws					= new WebSocket(url);
//			ws					= new WebSocket("ws://localhost:8080/nspace");
			ws.binaryType	= "arraybuffer";
			ws.onopen		= this.onOpen;
			ws.onmessage	= this.onMessage;		
			},

		onMessage : function(event)
			{
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Callback when a WebSocket message is received.
			//
			//	PARAMETERS
			//		-	event contains the message
			//
			////////////////////////////////////////////////////////////////////////
			var msg		= null;
			var elem		= null;
			var loc		= null;
			var type		= null;
			var value	= null;
			var tag		= null;

			// Type of received message
			if (typeof event.data == "string")
				{
				// Extract nSpace value from incoming XML
				parser	= new DOMParser();
				xmlDoc	= parser.parseFromString(event.data,"text/xml");
				msg		= nSpaceXML.load ( xmlDoc.documentElement );

				// Debug
				console.log(msg);
				}	// if

			// Binary
			else if (event.data instanceof ArrayBuffer)
				{
				var bfr	= new DataView(event.data);
				var idx	= new Uint32Array(1);

				// Extract nSpace value from  ArrayBuffer
//				console.log("ArrayBuffer:"+event.data);
				msg		= nSpaceBin.load ( bfr, idx );
				}	// else if

			// Sanity check
			if (msg == null || !("Root" in msg) || !("Location" in msg))
				{
				console.log("onMessage:Missing fields:"+msg);
				return;
				}	// if

			// Root path of value mapped to element
			elem = nElems[msg["Root"]];
			if (elem == null)
				return;

			// To make it easier to process string
			loc	= msg["Location"].toLowerCase();
//			type	= elem.nodeName.toLowerCase();
			tag	= elem.tagName.toLowerCase();
			type	= elem["type"];
			value	= msg["Value"];

			//
			// Process value based on element type
			//
			console.log("Location:"+loc+":Tag:"+tag+":Type:"+type);

			// Remove common '/onfire/value' postfix
			loc = loc.substring(0,loc.length-13);

			// Non-type specific

			// Enable
			if (loc == "element/enable")
				elem.disabled = !value;

			// Button
			else if (tag == "button")
				{
				}	// if

			// Checkbox
			else if (tag == "input" && type == "checkbox")
				{
				// Activate
				if (loc == "activate")
					elem.checked = value;
				}	// else if

			// List box
			else if (tag == "select")
				{
				// Current list
				if (loc == "list")
					{
					var str;

					// Remove existing elements
					while (elem.length > 0)
						elem.remove(0);

					// Add strings to list
					for (idx in value)
						{
						var option = document.createElement("option");
						option.text = value[idx];
						elem.add(option);
						}	// for

					}	// if

				}	// else if

			// Text/edit box
			else if (tag == "input" && type == "text")
				{
				// Value
				if (loc == "element/default")
					elem.text = value;
				}	// else if

			// Image
			else if (tag == "canvas")
				{
				// A default value for images is simply a dictionary
				// contain all of the image information
				if (loc == "element/default")
					{
					// The data for an image is a dictionary with its parameters
					if (value.hasOwnProperty("Bits"))
						{
						// Load bits, create a data URL and assign to image
						var bits		= new Uint8Array(value["Bits"]);
						var ctx		= elem.getContext("2d");
						var width	= value["Width"];
						var height	= value["Height"];
elem.width	= width;
elem.height	= height;
						var imageD	= ctx.createImageData(width,height);
						var data		= imageD.data;
						var len		= bits.length;
						for (var y = 0,srcidx = 0,dstidx = 0;y < height;++y)
							for (var x = 0;x < width;++x)
								{
								data[dstidx++]	= bits[srcidx];
								data[dstidx++]	= bits[srcidx];
								data[dstidx++]	= bits[srcidx++];
								data[dstidx++] = 0xff;
								}	// for
						ctx.putImageData(imageD,0,0);
						}	// if
				

					}	// if
				}	// else if

			else
				{
				console.log ( "Unhandled type for message : " + tag + ":" + type );
				}	// else

			},

		onOpen : function ()
			{
			////////////////////////////////////////////////////////////////////////
			//
			//	PURPOSE
			//		-	Callback when a WebSocket connection is made.
			//
			////////////////////////////////////////////////////////////////////////
			var root		= "";
			var elems	= null;
			var path		= null;

			// Enumerate nSpace elements and bind them to their paths.
 
			// Enumerate all of the elements on the page
			var
			elems = document.getElementsByTagName("*");

			// Find every element that specifies and nSpace path attribute.
			for (var i = 0, n = elems.length;i < n;++i)
				{
				// A 'root' path can be specified so that subsequent elements
				// can specify relative paths
				if (elems[i].getAttribute("data-nroot") != null)
					root = elems[i].getAttribute("data-nroot");

				// nSpace path specified ?
				if (elems[i].getAttribute("data-nloc") != null)
					{
					// Generate full bind path
					path = root + elems[i].getAttribute("data-nloc");

					// Assign full path for quick reference
					elems[i].attributes["data-nabs"] = path;

					// Associate the path with the HTML element
					nElems[path] = elems[i];

					// Set-up events
					elems[i].onclick	= onClickn;
					elems[i].onchange	= onChangen;

					// Send listen request for path
//					console.log ( "tagName:"+elems[i].tagName );
					listen(path);
					}	// if

			}	// for

			},

		}	// nSpace_bind

	var listen = function(srcLoc)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Issue a 'listen' request for the specified path
		//
		//	PARAMETERS
		//		-	srcLoc is the location to listen to
		//
		////////////////////////////////////////////////////////////////////////
		var req = 
			{
			verb: "Listen",
			path: ""
			};

		// Location for listening
		req["location"] = srcLoc;

		// Send request
		if (ws != null)
			ws.send(nSpaceXML.save(req));
		}	// listen

	//
	// Events
	//

	var onChangen = function(event)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Called when the an option changes.
		//
		//	PARAMETERS
		//		-	event contains the event information
		//
		////////////////////////////////////////////////////////////////////////
		var type = event.srcElement["type"];
		var dct  = {};
		var xml	= null;
		var send = true;

		console.log("onChangem:"+event);

		// Store template
		dct["Verb"]			= "Store";
		dct["Location"]	= event.srcElement.attributes["data-nabs"] + 
									"Element/Default/Fire";

		// Select
		if (type == "select-one")
			{
			// Selected index
			dct["Value"]	= event.srcElement.selectedIndex;
			}	// if

		// Text/edit
		else if (type == "text")
			dct["Value"]	= event.srcElement.value;

		// Unhandled type
		else
			send = false;

		// Transmit store
		if (send == true)
			{
			// Convert to XML
			xml = nSpaceXML.save(dct);

			// Transmit
			ws.send ( xml );
			}	// if
		}	// onChangen

	var onClickn = function(event)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Called when an element is 'clicked'.
		//
		//	PARAMETERS
		//		-	event contains the event information
		//
		////////////////////////////////////////////////////////////////////////
		var type = event.srcElement["type"];
		var dct  = {};
		var xml	= null;
		var send = true;

		// Store template
		dct["Verb"]			= "Store";
		dct["Location"]	= event.srcElement.attributes["data-nabs"] + "Activate/Fire";

		// Button
		if (type == "button")
			{
			// For a button, value does not matter
			dct["Value"]		= 0;
			}	// if

		// Checkbox
		else if (type == "checkbox")
			{
			// Checked state
			dct["Value"]		= (event.srcElement.checked == true) ? true : false;
			}	// else

		// Unhandled type
		else
			{
			console.log ( "Unhandled type for click : " + type );
			send = false;
			}	// else

		// Transmit store
		if (send == true)
			{
			// Convert to XML
			xml = nSpaceXML.save(dct);

			// Transmit
			ws.send ( xml );
			}	// if
		}	// onClickn

	}	// function

// Execute main on document loaded
document.onreadystatechange = function ()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the documents ready state has changed.
	//
	/////////////	///////////////////////////////////////////////////////////

	// Document ready ?
	if (document.readyState == "complete")
		nSpace_bind.init();		
	}	// onreadystatechange
