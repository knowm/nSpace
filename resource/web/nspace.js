////////////////////////////////////////////////////////////////////////
//
//									NSPACE.JS
//
//		Unified nSpace script to bind existing HTML elements to nSpace web 
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

				// Default value is the selected index
				else if (loc == "element/default")
					{
					// nSpace indecies are 1-based
					elem.selectedIndex = parseInt(value)-1;
					}	// else if
				}	// else if

			// Text/edit box
			else if (tag == "input" && type == "text")
				{
				// Value
				if (loc == "element/default")
					elem.value= value;
				}	// else if

			// Image
			else if (tag == "canvas")
				{
				// A default value for images is simply a dictionary
				// contain all of the image information
				if (loc == "element/default" || loc == "")
					{
					// The data for an image is a dictionary with its parameters
					if (value.hasOwnProperty("Bits"))
						{
						// Access image information
						var bits		= new Uint8Array(value["Bits"]);

						// Context for rendering to the canvas
						var ctx	= elem.getContext("2d");

						// Create a new image object for rendering
						var img	= new Image();

						// In case blob gets created
						var blb	= null;

						// Handle required formats
						if (value["Format"] == "JPEG" || value["Format"] == "JPG")
							{
							// Create a blob for the image bits
							blb = new Blob([bits], { type : 'image/jpeg' } );
							}	// if
						else if (value["Format"] == "PNG")
							{
							// Create a blob for the image bits
							blb = new Blob([bits], { type : 'image/png' } );
							}	// if

						// NOTE: Special case of raw pixel data.  Does not URL/blob logic below.
						else if (value["Format"] == "R8G8B8" || value["Format"] == "B8G8R8" ||
									value["Format"].toLowerCase() == "u8x2")
							{
							// Size of image
							var width	= value["Width"];
							var height	= value["Height"];

							// Byte order
							var bRGB		= (value["Format"] == "R8G8B8");
							var bBGR		= (value["Format"] == "B8G8R8");
							var bGray8	= (value["Format"].toLowerCase() == "u8x2");

							// Must handle re-sizing to shape.  Create memory canvas and draw
							// entire image to it, then draw resized into target canvas
							var canvasMem	= document.createElement("canvas");
							var ctxMem		= canvasMem.getContext("2d");

							// Ensure memory canvas matches size of element for later drawing
							canvasMem.width	= width;
							canvasMem.height	= height;

							// Get image data directly from canvas and write raw pixel data to it.
							var imageD	= ctxMem.createImageData(width,height);
							var data		= imageD.data;
							var len		= bits.length;
							for (var y = 0,srcidx = 0,dstidx = 0;y < height;++y)
								for (var x = 0;x < width;++x)
									{
									if (bBGR)
										{
										data[dstidx+0]	= bits[srcidx+2];
										data[dstidx+1]	= bits[srcidx+1];
										data[dstidx+2]	= bits[srcidx+0];
										dstidx += 3;
										srcidx += 3;
										}	// if
									else if (bRGB)
										{
										data[dstidx++]	= bits[srcidx++];
										data[dstidx++]	= bits[srcidx++];
										data[dstidx++]	= bits[srcidx++];
										}	// else
									else if (bGray8)
										{
										data[dstidx++]	= bits[srcidx];
										data[dstidx++]	= bits[srcidx];
										data[dstidx++]	= bits[srcidx++];
										}	// else if
									data[dstidx++] = 0xff;
									}	// for

							// Write pixel data directly to memory canvas
							ctxMem.putImageData(imageD,0,0);

							// Draw to target canvas
							ctx.drawImage(canvasMem,0,0,elem.width,elem.height);
							}	// if

						// Raw RGB
//						else if (value["Format"] == "R8G8B8")
//							{
//							// Create a blob for the image bits
//							blb = new Blob([bits], { type : 'image/x-rgb' } );
//							}	// if

						// If valid blob, render it to canvas
						if (blb != null)
							{
							// URL for blob
							var url	= URL.createObjectURL(blb);

							// Image will be drawn when URL is loaded
							img.onload = function ()
								{
								// TODO: Resize/aspect ratio options ?
								ctx.drawImage(img,0,0,elem.width,elem.height);
								URL.revokeObjectURL(url);
								}	// onload

							// Assign URL to image for rendering
							img.src = url;
							}	// if
						}	// if
					}	// if

					// Raw pixel data
					/*
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
					*/

				}	// else if

			// Default handler for what is assumed to be generic text
			// (paragraph, div, headers, etc)
			else 
				{
				// Value
				if (loc == "element/default")
					elem.textContent = value;
				}	// else if

//			else
//				{
//				console.log ( "Unhandled type for message : " + tag + ":" + type );
//				}	// else

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
					// Generate full bind path if element path is relative
					path = elems[i].getAttribute("data-nloc");
					if (path[0] != '/')
						path = root + path;

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
			Verb: "Listen",
			};

		// Location for listening
		req["Location"] = srcLoc;

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
			// Selected index, indexes in nSpace are 1-based
			dct["Value"]	= event.srcElement.selectedIndex+1;
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

////////////////////////////////////////////////////////////////////////
//
//									NSPACE_BIN.JS
//
//							nSpace binary value parser.
//
////////////////////////////////////////////////////////////////////////

//
// Class - nSpaceBin. Object to parse nSpace values from/to array buffers.
//
 
var nSpaceBin =
	{
	init : function() 
		{ 
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Initialize state of the object
		//
		////////////////////////////////////////////////////////////////////////
		},

	load : function(bfr,idx)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a value from an array buffer
		//
		//	PARAMETERS
		//		-	bfr is the byte array
		//		-	idx is the index into buffer
		//
		// RETURN VALUE
		//		Converted value
		//
		////////////////////////////////////////////////////////////////////////
		var ret	= null;

		// Value begin
		if (nSpaceBin.load16(bfr,idx) != 0x3141)
			return ret;

		// Value type
		var type = nSpaceBin.load16(bfr,idx);
		switch (type)
			{
			// String
			case 0x8 :
				// Read string value
				ret = nSpaceBin.loadStr(bfr,idx);
				break;

			// Numbers
			case 3 :
				ret = nSpaceBin.load32(bfr,idx);
				break;
			case 11 :
				ret = (nSpaceBin.load16(bfr,idx) != 0) ? true : false;
				break;
			case 4 :
				ret	= bfr.getFloat32(idx[0]);
				idx[0] += 4;
				break;
			case 5 :
			case 7 :
				// Dates are doubles representing number of days since Jan 1, 2001
				// TODO: Convert to javascript data
				ret		= bfr.getFloat64(idx[0]);
				idx[0] += 8;
				break;

			// Object
			case 0xd :
				// Read object Id string
				var id = nSpaceBin.loadStr(bfr,idx).toLowerCase();

				// Dictionary
				if (id == "adt.dictionary")
					{
					// Empty dictionary
					ret = {};

					// Number of key/value pairs
					var cnt = nSpaceBin.load32(bfr,idx);

					// Read pairs
					for (var i = 0;i < cnt;++i)
						{
						var key	= nSpaceBin.load(bfr,idx);
						var val	= nSpaceBin.load(bfr,idx);
						ret[key]	= val;
						}	// for

					}	// if

				// List
				else if (id == "adt.list")
					{
					// Empty list
					ret = [];

					// Number of values
					var cnt = nSpaceBin.load32(bfr,idx);

					// Read values
					for (var i = 0;i < cnt;++i)
						{
						var val	= nSpaceBin.load(bfr,idx);
						ret.push	= val;
						}	// for
					}	// else if

				// Memory block
				else if (id == "io.memoryblock")
					{
					// Size of region
					var sz = nSpaceBin.load32(bfr,idx);

					// Read in block of memory
					ret		= bfr.buffer.slice(idx[0],idx[0]+sz);
					idx[0]	+= sz;
					}	// else if

				// Unknown
				else
					{
					console.log("Unhanlded object type : " + id );
					}	// else
				break;

			// Empty values ok
			case 0 :
				// Empty
				ret = null;
				break;

			default :
				console.log ( "nSpaceBin:Unhandled type:"+type );
			}	// switch

		// Value end
		if (nSpaceBin.load16(bfr,idx) != 0x5926)
			return null;

		return ret;
		},	// load

	load16 : function(bfr,idx)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a 16-bit value from the buffer
		//
		//	PARAMETERS
		//		-	bfr is the array buffer
		//		-	idx is the index from which to read
		//
		// RETURN VALUE
		//		Value
		//
		////////////////////////////////////////////////////////////////////////
		ret	= bfr.getUint16(idx[0]);
		idx[0] += 2;

		return ret;
		},	// load16

	load32 : function(bfr,idx)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a 32-bit value from the buffer
		//
		//	PARAMETERS
		//		-	bfr is the array buffer
		//		-	idx is the index from which to read
		//
		// RETURN VALUE
		//		Value
		//
		////////////////////////////////////////////////////////////////////////
		ret	= bfr.getUint32(idx[0]);
		idx[0] += 4;

		return ret;
		},	// load32

	loadStr : function(bfr,idx)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a string from the buffer
		//
		//	PARAMETERS
		//		-	bfr is the array buffer
		//		-	idx is the index from which to read
		//
		// RETURN VALUE
		//		String
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;

		// Length of string
		var len = nSpaceBin.load32(bfr,idx);

		// Size of WCHAR in remote system
		var szw = nSpaceBin.load16(bfr,idx);

		// Read string bytes
		switch (szw)
			{
			// 2 bytes per char (Windows)
			case 2 :
				ret = "";
				for (var i = 0;i < len;++i)
					{
					var c = nSpaceBin.load16(bfr,idx);
					c = ( ((c >> 8) & 0xff) | ((c << 8) & 0xff00) );
					ret += String.fromCharCode(c);
					}	// for
				break;
			// 4 bytes per char (Linux)
			case 4 :
				ret = "";
				for (var i = 0;i < len;++i)
					{
					var c = nSpaceBin.load32(bfr,idx);
					c = ( ((c >> 24) & 0xff) | ((c >> 8) & 0xff00) );
					ret += String.fromCharCode(c);
					}	// for
				break;

			default :
				console.log("Unhandled WCHAR size:"+szw);
				break;
			}	// switch

		return ret;
		},	// loadStr

	save : function(value)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Save value to array buffer
		//
		//	PARAMETERS
		//		-	value is the value to save
		//
		// RETURN VALUE
		//		Array buffer
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;

		return ret;
		}	// load

	}	// nSpaceBin
	

////////////////////////////////////////////////////////////////////////
//
//									NSPACE_XML.JS
//
//							nSpace XML value parser.
//
////////////////////////////////////////////////////////////////////////

//
// Class - nSpaceXML. Object to parse nSpace values from/to XML strings.
//
 
var nSpaceXML =
	{
	init : function() 
		{ 
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Initialize state of the object
		//
		////////////////////////////////////////////////////////////////////////
		},

	load : function(elem)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Load a value from an XML document
		//
		//	PARAMETERS
		//		-	elem is the root element of the XML document
		//
		// RETURN VALUE
		//		Converted value
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;

		// Type
		if (elem.nodeName == "Dictionary")
			{
			var key = null;
			var val = null;

			// Empty dictionary
			var dct = {};

			// Load key/value pairs into dictionary
			for (var i = 0;i < elem.childNodes.length;i+=2)
				if (	((key = nSpaceXML.load(elem.childNodes[i+0])) != null) &&
						((val = nSpaceXML.load(elem.childNodes[i+1])) != null) )
					dct[key] = val;

			// Use as return value
			ret = dct;
			}	// if
		else if (elem.nodeName == "List")
			{
			var val = null;

			// Empty list
			var lst = [];

			// Load child values into list
			for (var i = 0;i < elem.childNodes.length;++i)
				if ((val = nSpaceXML.load(elem.childNodes[i])) != null)
					lst.push ( val );

			// Use as return value
			ret = lst;
			}	// else if

		// Single value
		else if (elem.nodeName == "Value")
			{
			var val = null;

			// Type specified ?
			var type = (elem.attributes.length > 0) ?
							elem.attributes["Type"].nodeValue : null;

			// It is possible to receive an 'empty' value in which
			// case there is not 'firstChild'.
			if (	(elem.firstChild != null) &&
					(val = elem.firstChild.nodeValue) != null)
				{
				// Default
				if (type == null)
					type = "string";
				else 
					type = type.toLowerCase();

				// Convert type
				if (type == "float" || type == "double")
					val = parseFloat(val);
				else if (type == "int" || type == "long")
					val = parseInt(val);
				else if (type == "boolean")
					val = Boolean(val.toLowerCase() == "true");
				}	// if

			// Use as return value
			ret = val;
			}	// else if

		return ret;
		},	// load

	save : function(value)
		{
		////////////////////////////////////////////////////////////////////////
		//
		//	PURPOSE
		//		-	Save value to XML string
		//
		//	PARAMETERS
		//		-	value is the value to save
		//
		// RETURN VALUE
		//		XML string
		//
		////////////////////////////////////////////////////////////////////////
		var ret = null;
		var dct = {};
		var lst = [];

		// Dictionary
		if (value instanceof Object)
			{
			// Begin dictionary
			ret = "<Dictionary>"

			// Append values
			for (var key in value)
				{
				// Key then value
				ret += nSpaceXML.save(key);
				ret += nSpaceXML.save(value[key]);
				}	// for

			// End dictionary
			ret += "</Dictionary>";
			}	// if

		// List
		else if (value instanceof Array)
			{
			console.log("Array");
			}	// else if

		// Value
		else
			{
			// Begin value
			ret = "<Value";

			// String
//			console.log(typeof value);
			if (typeof value == "string")
				ret += ">"+value.toString();

			// Number
			else if (typeof value == "number")
				{
				// Integer
				if (Number.isInteger(value))
					{
					ret += " Type=\"Integer\">";
					ret += value.toString();
					}	// else if
				else
					{
					ret += " Type=\"Float\">";
					ret += value.toString();
					}	// else

				}	// else if

			// Boolean
			else if (typeof value == "boolean")
				{
				ret += " Type=\"Boolean\">";
				ret += value.toString();
				}	// else if

			// Default
			else
				ret += ">"+value.toString();

			// End value
			ret += "</Value>";
			}	// else

		return ret;
		}	// load

	}	// nSpaceXML
