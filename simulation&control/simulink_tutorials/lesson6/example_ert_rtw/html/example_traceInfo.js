function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "example"};
	this.sidHashMap["example"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<Root>/In1"] = {sid: "example:81"};
	this.sidHashMap["example:81"] = {rtwname: "<Root>/In1"};
	this.rtwnameHashMap["<Root>/In2"] = {sid: "example:82"};
	this.sidHashMap["example:82"] = {rtwname: "<Root>/In2"};
	this.rtwnameHashMap["<Root>/In3"] = {sid: "example:83"};
	this.sidHashMap["example:83"] = {rtwname: "<Root>/In3"};
	this.rtwnameHashMap["<Root>/Gain"] = {sid: "example:84"};
	this.sidHashMap["example:84"] = {rtwname: "<Root>/Gain"};
	this.rtwnameHashMap["<Root>/Product"] = {sid: "example:85"};
	this.sidHashMap["example:85"] = {rtwname: "<Root>/Product"};
	this.rtwnameHashMap["<Root>/Sum"] = {sid: "example:86"};
	this.sidHashMap["example:86"] = {rtwname: "<Root>/Sum"};
	this.rtwnameHashMap["<Root>/Out1"] = {sid: "example:87"};
	this.sidHashMap["example:87"] = {rtwname: "<Root>/Out1"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
