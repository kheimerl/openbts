/*
* Copyright 2008 Free Software Foundation, Inc.
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribuion.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#define LOG_GROUP LogGroup::SIP		// Can set Log.Level.SIP for debugging.  If there were any LOG() calls in this file.

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <signal.h>
#include <stdlib.h>

#include <ortp/ortp.h>
#include <ortp/telephonyevents.h>

//#include "SIPInterface.h"
#include "SIPUtility.h"
#include <Globals.h>
#include <GSMConfig.h>
#include <GSML3CommonElements.h>



namespace SIP {
using namespace std;

// Unused, but here it is if you want it:
// Pair is goofed up, so just make our own.  It is trivial:
//
//template <class T1, class T2>
//struct Pair {
//	T1 first;
//	T2 second;
//	Pair(T1 a, T2 b) : first(a), second(b) {}
//};
//template <class T1, class T2>
//Pair<T1,T2> make_pair(T1 x, T2 y)
//{
//	return Pair<T1,T2>(x,y);
//}
//typedef Pair<string,string> SipParam;	// Used for both params and 'headers', which is stuff in a URI.

#if 0	// Dont bother.  Added isINVITE(), isACK(), etc.
// Note that all methods establish a new transaction, including ACK.
// This is still not handled correctly.
enum SipMethodType {
	MethodUndefined,
	MethodACK,
	MethodBYE,
	MethodCANCEL,
	MethodINVITE,
	MethodMESSAGE,
	MethodOPTIONS,
	MethodINFO,
	MethodREFER,
	MethodNOTIFY,
	MethodSUBSCRIBE,
	MethodPRACK,
	MethodUPDATE,
	MethodREGISTER,
	MethodPUBLISH
};
typedef std::pair<string,SIPMethodType> SipMethodPair;
typedef std::map<string,SIPMethodType> SipMethodMap;
static SipMethodMap gMethodMap;
SIPMethodType SipMessage::str2MethodType(string &methodname)
{
	if (gMethodMap.size() == 0) {
		// init the map
#define HELPER_ONE_SIP_METHOD(name) gMethodMap.insert(SipMethodPair(#name, Method##name));
		HELPER_ONE_SIP_METHOD(ACK)
		HELPER_ONE_SIP_METHOD(BYE)
		HELPER_ONE_SIP_METHOD(CANCEL)
		HELPER_ONE_SIP_METHOD(INVITE)
		HELPER_ONE_SIP_METHOD(MESSAGE)
		HELPER_ONE_SIP_METHOD(OPTIONS)
		HELPER_ONE_SIP_METHOD(INFO)
		HELPER_ONE_SIP_METHOD(REFER)
		HELPER_ONE_SIP_METHOD(NOTIFY)
		HELPER_ONE_SIP_METHOD(SUBSCRIBE)
		HELPER_ONE_SIP_METHOD(PRACK)
		HELPER_ONE_SIP_METHOD(UPDATE)
		HELPER_ONE_SIP_METHOD(REGISTER)
		HELPER_ONE_SIP_METHOD(PUBLISH)
		gMethodMap.insert(SipMethodPair("",MethodUndefined));		// empty string maps to Undefined method.
	}
	//for (SipMethodMap::iterator i = gMethodMap.begin(); i != gMethodMap.end(); ++i) {
	//	printf("SipMethodMap %s = %d\n",i->first,i->second);
	//}

	if (methodname == NULL) { return MethodUndefined; }
	SipMethodMap::iterator it = gMethodMap.find(methodname);
	if (it == gMethodMap.end()) { printf("MethodMap %s not found\n",methodname); }
	return it == gMethodMap.end() ? MethodUndefined : it->second;
}
SIPMethodType SipMessage::smGetMethod() const
{
	if (omsg()->sip_method == NULL) {
		return MethodUndefined;	// Hmm.
	}
	string methname(smGetMethodName());
	SIPMethodType result = str2MethodType(methname);
	LOG(DEBUG)<<LOGVAR(methname)<<LOGVAR(result);
	return result;
}
#endif

ResponseTable gResponseTable;

ResponseTable::ResponseTable()
{
		// class 1,"X: Provisional messages
		addResponse(100,"Trying");
		addResponse(181,"Call Is Being Forwarded");
		addResponse(182,"Queued");
		addResponse(183,"Session Progress");	//  FIXME we need to setup the sound channel (early media);
		addResponse(180,"Ringing");
		// class 2,"X: Success
		addResponse(200,"OK");

		// class 3,"x: Redirection
		addResponse(300,"Multiple Choices");
		addResponse(301,"Moved Permanently");
		addResponse(302,"Moved Temporarily");
		addResponse(305,"Use Proxy");
		addResponse(380,"Alternative Service");

		// class 4,"X: Request failures
		addResponse(400,"Bad Request");
		addResponse(401,"Unauthorized");	// Used only by registrars. Proxys should use proxy authorization 407");
		addResponse(402,"Payment Required");	// (Reserved for future use)");
		addResponse(403,"Forbidden");
		addResponse(404,"Not Found: User not found");
		addResponse(405,"Method Not Allowed");
		addResponse(406,"Not Acceptable");
		addResponse(407,"Proxy Authentication Required");
		addResponse(408,"Request Timeout");	//: Couldn't find the user in time");
		addResponse(409,"Conflict");
		addResponse(410,"Gone");		//: The user existed once, but is not available here any more.");
		addResponse(413,"Request Entity Too Large");
		addResponse(414,"Request-URI Too Long");
		addResponse(415,"Unsupported Media Type");
		addResponse(416,"Unsupported URI Scheme");
		addResponse(420,"Bad Extension");		//: Bad SIP Protocol Extension used, not understood by the server");
		addResponse(421,"Extension Required");
		addResponse(422,"Session Interval Too Small");
		addResponse(423,"Interval Too Brief");
		addResponse(480,"Temporarily Unavailable");
		addResponse(481,"Call/Transaction Does Not Exist");
		addResponse(482,"Loop Detected");
		addResponse(483,"Too Many Hops");
		addResponse(484,"Address Incomplete");
		addResponse(485,"Ambiguous");
		addResponse(486,"Busy Here");
		addResponse(487,"Request Terminated");
		addResponse(488,"Not Acceptable Here");
		addResponse(491,"Request Pending");
		addResponse(493,"Undecipherable");	//: Could not decrypt S/MIME body part");

		// class 5,"X: Server failures
		addResponse(500,"Server Internal Error");
		addResponse(501,"Not Implemented");		//: The SIP request method is not implemented here");
		addResponse(502,"Bad Gateway");
		addResponse(503,"Service Unavailable");
		addResponse(504,"Server Time-out");
		addResponse(505,"Version Not Supported");		//: The server does not support this version of the SIP protocol");
		addResponse(513,"Message Too Large");

		// class 6,"X: Global failures
		addResponse(600,"Busy Everywhere");
		addResponse(603,"Decline");
		addResponse(604,"Does Not Exist Anywhere");
		addResponse(606,"Not Acceptable");
}

void ResponseTable::addResponse(unsigned code, const char * name)
{
	mMap[code] = string(name);
}

string ResponseTable::get(unsigned code)
{
	return gResponseTable[code];
}

string ResponseTable::operator[](unsigned code) const
{
	ResponseMap::const_iterator it = mMap.find(code);
	return (it == mMap.end()) ? string("undefined") : it->second;
}


// On success, set the resolved address stuff, mipValid, and return true.
bool IPAddressSpec::ipSet(string addrSpec, const char * provenance)
{
	mipName = addrSpec;

	// (pat 7-230-2013) Someone else added this, I am preserving it:
	// Check for illegal hostname length. 253 bytes for domain name + 6 bytes for port and colon.
	// This isn't "pretty", but it should be fast, and gives us a ballpark. Their hostname will
	// fail elsewhere if it is longer than 253 bytes (since this assumes a 5 byte port string).
	if (addrSpec.size() == 0) {
		LOG(ALERT) << "hostname is empty from "<<provenance;
		return false;
	}
	if (addrSpec.size() > 259) {
		LOG(ALERT) << "hostname is greater than 253 bytes from "<<provenance;
		return false;
	}

	if (!resolveAddress(&mipSockAddr,addrSpec.c_str())) {
		//try to resolve with default port if it fails without
		if (!resolveAddress(&mipSockAddr,addrSpec.c_str(), 5060)) {
			LOG(CRIT) << "cannot resolve IP address for " << addrSpec <<" from "<<provenance;	// << sbText();
			return false;
		}
	}

	{	char host[256];
		const char* ret = inet_ntop(AF_INET,&(mipSockAddr.sin_addr),host,255);
		if (!ret) {
			LOG(CRIT) << "cannot translate proxy IP address:" << addrSpec <<" from "<<provenance;;
			return false;
		}
		mipIP = string(host);
		mipPort = ntohs(mipSockAddr.sin_port);
		mipValid = true;
	}
	return true;
}

string IPAddressSpec::ipToText() const
{
	ostringstream ss;
	ss <<LOGVARM(mipName)<<LOGVARM(mipIP)<<LOGVARM(mipPort);
	return ss.str();
}

static uint32_t timeUniquifier()
{
	struct timeval now;
	gettimeofday(&now,NULL);
	return ((now.tv_sec&0xffff)<<16) + (now.tv_usec/16);	// 32 bit number that changes every 15-16 usecs.
}

static unsigned cellid()
{
	// Cell id:
	unsigned c0 = gTRX.C0();		// 10 bits
	unsigned bcc = gBTS.BCC();		// 3 bits
	unsigned ncc = gBTS.NCC();		// 3 bits
	return (c0<<16) + (bcc<<3) + ncc;
}


// Make a globally unique identifier starting with string start.
// We could just use another big random number, but this is nicer.
// Use the GSM MCC+MNC+LAC+cellid of the cell, plus the time, plus 32 bits of random, which is both unique
// and has the added advantage that humans can read useful information out of it.
// The SIP spec shows examples making ids unique by appending the IP address, but that is just silly because
// we will probabably have a local IP address behind some firewall.
string globallyUniqueId(const char *start)
{
	// This is a a globally unique callid.
	GSM::L3LocationAreaIdentity lai = gBTS.LAI();	// 3 digit MCC, 3 digit MNC, 16-bit LAC.
	char buf[80];
	snprintf(buf,80,"%s%03u%02u%x%x-%x%x",start,lai.MCC(),lai.MNC(),lai.LAC(),cellid(), timeUniquifier(),(unsigned)(0xffffffff&random()));
	return string(buf);
}


static string make_tag1(const char *start)
{
	// The odds of one of these colliding is vanishingly small.
	// (pat) Actually, the odds are pretty good (like, near unity) unless you call srandom before calling random.
	uint64_t r1 = random();
	uint64_t r2 = random();
	uint64_t val = (r1<<32) + r2;
	char tag[60];
	
	// map [0->26] to [a-z] 
	strcpy(tag,start);
	char *cp = tag+strlen(start);
	for (int k=0; k<16; k++) {
		*cp++ = val%26+'a';
		val = val >> 4;
	}
	*cp = '\0';
	return string(tag);
}

string make_tag()
{
	return make_tag1("OBTS");
}

string make_branch(const char * /*hint*/)	// hint is unused.
{
	// RFC3261 17.2.3: The branch parameter should begin with the magic string "z9hG4bK" to
	// indicate compliance with this specification.
	return make_tag1("z9hG4bKOBTS");
}

string dequote(const string in)
{
	if (in[0] != '"') return in;
	if (in.size() < 2 || in[in.size() - 1] != '"') {
		LOG(ERR) << "Invalid quoted string:"<<in;
		return in;
	}
	return in.substr(1,in.size()-2);
}

std::ostream& operator<<(std::ostream& os, const SipTimer&t)
{
	os <<t.text();
	return os;
}

};	// namespace SIP

// vim: ts=4 sw=4
