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
			case 0 :
				// Empty
				ret = null;
				break;

			// Object
			case 0xd :
				// Read object Id string
				var id = nSpaceBin.loadStr(bfr,idx);

				// Dictionary
				if (id == "Adt.Dictionary")
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
		var ret = ((bfr[idx[0]++] << 8) | (bfr[idx[0]++] << 0));
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
		var ret = ( (nSpaceBin.load16(bfr,idx) << 16) |
						nSpaceBin.load16(bfr,idx));
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
			// 2 bytes per char
			case 2 :
				ret = "";
				for (var i = 0;i < len;++i)
					{
					var c = nSpaceBin.load16(bfr,idx);
					c = ( ((c >> 8) & 0xff) | ((c << 8) & 0xff00) );
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

	}	// nSpaceXML
