
/* UbloxM8Q_GPS library 
 *   Copyright 2017 Lynd Wieman
 *   For use with the Particle Electron AssetTracker 2 and the AssetTracker2
 *   driver.
 *   See README.md and AssetTracker2 README.md
 */

#include "UbloxM8Q_GPS.h"


// ubx commands
// Defined according to the ublox spec (see README).
// These do not require the calculated check sum to be included in the 
// definitions, it is calculated at runtime, but they do require two
// byes at end for the check sum to be stored.
uint8_t enablePOSLLH[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x01, // class,id - message identifier, CFG_MSG_TYPE
0x03,0x00, // length
0x01,0x02, // NAV_POSLLH_TYPE - identifier of what this message is configuring
0x01,      // rate - generate specifed MSG once per navigation solution
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
// max time we wait for an update from the receiver. This must reflect
// measRate and navRate in the CFG_RATE_TYPE message, and the rate in 
// all CFG_MSG_TYPE messages.
// If we don't receive a message within this amount of time, stalled()
// returns true and hasFix() returns false. The application decides how to
// recover (TrailerMonitor power cycles the receiver).
// This implementation is fragile. If the measurement or nav rate in the 
// cfgRate message is changed this value probably should be changed along
// with the rate. It is handled much more robustly by the setRate routine.
uint32_t ubxWatchdog = 60000;  // ms
uint8_t cfgRate[] = // set rate of position report
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x08, // class,id - message identifier, CFG_RATE_TYPE
0x06,0x00, // length
0x30,0x75, // measRate - ms, 0x3E8 1 every sec, 0x7530 1 every 30sec
0x01,0x00, // navRate - little endian - 5 means generate navigation solution every 5 measurements
0x00,0x00, // timeRef  time system 0: UTC, 1: GPS, 3,4,5: GLOSNASS, BeiDou, Galileo
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
uint8_t enablePVT[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x01, // class,id - message identifier, CFG_MSG_TYPE
0x03,0x00, // length
0x01,0x07, // NAV_PVT_TYPE - identifier of what this message is configuring
0x01,      // rate - generate specifed MSG once per navigation solution
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
uint8_t cfgNav5[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x24, // class,id - message identifier, CFG_MSG_TYPE
0x24,0x00, // length - 36 decimal
0x01,0x00, // parameters bitmask, set dynamic model, turn off static hold
0x04,      // dynamic model - automotive (pedestrian, 3, is limited to 30 m/s, about 67 mph)
0x00,      // fix mode - recommended to not change
0x00,0x00, // fixed alt
0x00,0x00, // fixed alt
0x00,0x00, // fixed alt var
0x00,0x00, // fixed alt var
0x00,      // min elevation for satellite
0x00,      // drLimit reserved
0x00,0x00, // pDOP mask
0x00,0x00, // tDOP mask
0x00,0x00, // pAcc mask
0x00,0x00, // tAcc mask
0x64,      // Static hold thresh, cm/s (or 1.97 ft/min), 100 cm/s = ~200 ft/min = 02.2mph
0x00,      // DGNSS timeout
0x00,      // CNO thresh Num SVs
0x00,      // CNO thresh
0x00,0x00, // Reserved
0x28,0x00, // Static hold dist thresh before quitting static hold, m, 40 m =~150ft
0x00,      // UTC Standard
0x00,0x00, // Reserved
0x00,0x00, // Reserved
0x00,      // Reserved
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
// configuration for odometer
uint8_t cfgODOmsg[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x1E, // class,id - message identifier, CFG_ODO_TYPE
0x14,0x00, // length, 20
0x00,      // version, 0 for this version
0x00,0x00, // reserved 3 bytes
0x00,      // reserved
0x03,      // flags - enable odometer, COG filters
0x01,      // odocfg: profile: 0-running, 1-cycle, 2-swim, 3-car, 4-custom
0x00,0x00, // more reserved, 6 bytes
0x00,0x00, // more reserved, 
0x00,0x00, // more reserved,
0x02,      // cogMaxSpd - max speed for low-spd COG filter,m/s 1e-1,2=>40/ft/min
0x0A,      // cogMaxPosAcc - max accuracy for low-speed COG filter, m
0x00,0x00, // more reserved,
0x10,      // velLPGain - velocity Lo-pass filter level 0-255
0x10,      // cogLPGain - COG Lo-pass filter level (speed<8m/s) range: 0-255
0x00,0x00, // more reserved,
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
// Have to enable reporting the message also
uint8_t enableODO[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x06,0x01, // class,id - message identifier, CFG_MSG_TYPE
0x03,0x00, // length
0x01,0x09, // NAV_ODO_TYPE - identifier of what this message is configuring
0x01,      // rate - generate specifed MSG once per navigation solution
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
// reset trip odometer
uint8_t resetODOmsg[] =
{
0xb5,0x62, // header/sync bytes - always b5, 62
0x01,0x10, // class,id - NAV_RESETODO_TYPE
0x00,0x00, // length, no payload
0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};

// for simplifying ack detection
AckedMsg ackedMsg = AckedMsg();

// Scoped lock for the receive parser. Does nothing until the mutex exists.
class RxGuard
{
public:
    RxGuard(os_mutex_recursive_t m) : m(m) { if (m) os_mutex_recursive_lock(m); }
    ~RxGuard() { if (m) os_mutex_recursive_unlock(m); }
private:
    os_mutex_recursive_t m;
};

UbloxM8Q_GPS::UbloxM8Q_GPS()
{
    common_init();
}

void
UbloxM8Q_GPS::common_init()
{
  initComplete = false;
  gpsOn = false;
  rxLock = nullptr;
  rxType = rxLength = 0;
  lastMsgMs = lastPvtMs = 0;
  hour = minute = seconds = month = day = 0;
  year = 0;
  milliseconds = tow = 0;
  dateValid = false;
  timeValid = false;
  // Floating point latitude and longitude value in degrees.
  latitude = longitude = 0.0;
  latitudeDegrees = longitudeDegrees = 0.0;
  geoidheight = altitude = 0.0;
  speed = 0.0; // m/s - its just gSpeed/1000.0
  headingMot = headingVeh = 0.0; // degrees
  headVehValid = false;
  fix = false;
  fixquality = satellites = 0;
  horzAcc = vertAcc = 0; // mm
  velN = velE = velD = 0; // north, east, down velocity, mm/s
  gSpeed = 0; // ground speed, mm/s
  odoTrip = 0; // odometer since last time commanded reset, m
  odoTotal = 0; // odometer since last power up, m
  odoAcc = 0; // odometer accuracy, m (1-sigma)
}


// Must be called repeatedly until it returns ubxSUCCESS
// To add additional configuration, add a new case with the same code as
// case 1 (or any other case except 0), and change the message in the call
// to writeUBX(yourMsg) to the message you have defined. If you turn on a new
// report you also have to write a parsing routine for it and call the parser
// from UbloxM8Q_GPS::parse().
enum ubxReturn
UbloxM8Q_GPS::begin()
{
    static int state = 0;
    enum ubxReturn ret;

    switch (state){
	case 0:
	    // Read the version report from the receiver, then turn off
	    // NMEA ascii messages. We still get some ascii that gets
	    // thrown out by readUBXmsg. The first periodic ubx message gets 
	    // corrupted but, not to worry, it sends more soon.
	    if (!rxLock){
		// Created here rather than in the constructor, which runs
		// before the RTOS is ready.
		os_mutex_recursive_create(&rxLock);
	    }
	    readVer();
	    lastMsgMs = millis();
	    gpsOn = true;
	    state++;
	    // fall through to next case

	case 1:
	    ret = writeUBX(cfgRate) ;
	    if ( ret == ubxPENDING){
		return ret;
	    }
	    if ( ret == ubxSUCCESS){
		state++;
		return ubxPENDING;
	    } else { // some error
		reportError(state, ret);
		state = 0;
		return ret;
	    }
	    return ubxPENDING;
	break;
	case 2:
	    ret = writeUBX(enablePVT) ;
	    if ( ret == ubxPENDING){
		return ret;
	    }
	    if ( ret == ubxSUCCESS){
		state++;
		return ubxPENDING;
	    } else { // some error
		reportError(state, ret);
		state = 0;
		return ret;
	    }
	    return ubxPENDING;
	break;
	case 3:
	    ret = writeUBX(cfgNav5) ;
	    if ( ret == ubxPENDING){
		return ret;
	    }
	    if ( ret == ubxSUCCESS){
		state++;
		return ubxPENDING;
	    } else { // some error
		reportError(state, ret);
		state = 0;
		return ret;
	    }
	    return ubxPENDING;
	break;
	case 4:
	    ret = writeUBX(cfgODOmsg) ;
	    if ( ret == ubxPENDING){
		return ret;
	    }
	    if ( ret == ubxSUCCESS){
		state++;
		return ubxPENDING;
	    } else { // some error
		reportError(state, ret);
		state = 0;
		return ret;
	    }
	    return ubxPENDING;
	break;
	case 5:
	    ret = writeUBX(enableODO) ;
	    if ( ret == ubxPENDING){
		return ret;
	    }
	    if ( ret == ubxSUCCESS){
		state++;
		return ubxPENDING;
	    } else { // some error
		reportError(state, ret);
		state = 0;
		return ret;
	    }
	    return ubxPENDING;
	break;
	default:
	    // state eventually gets incremented beyond defined cases and
	    // we end up here. That means we are done!
	    initComplete = true;
	    state = 0;
	    return ubxSUCCESS;
	break;
    }
    return ubxSUCCESS;
}



// set the measurement rate and navigation solution rate. Also sets the
// ubxWatchdog timeout
// 
// measurement - ms. Time between position measurements
// navigationSol - cycles. Number of measuremnts for each navigation solution
// 
// ubxWatchdog timeout is set to measurement * navigationSol * 5
//  The 5 is an arbitrary number I picked out of the air.

// Don't use a local (stack) variable, other rtns modify it and use it over
// significant time
uint8_t rateMsg[] = {
    0xb5,0x62, // header/sync bytes - always b5, 62
    0x06,0x08, // class,id - message identifier, CFG_RATE_TYPE
    0x06,0x00, // length
    0x88,0x13, // measRate - little endian - ms, modified by setRate()
    0x01,0x00, // navRate - little endian - modified by setRate()
    0x00,0x00, // timeRef  time system 0: UTC, 1: GPS, 3,4,5: GLOSNASS, BeiDou, Galileo
    0x00,0x00  // CK_A,CK_B - checksum bytes, set by ubxCS(sentence)
};
void
UbloxM8Q_GPS::setRate(uint16_t measurement, int navigationSol)
{
    if (measurement < 10){ // I don't know what the real minimum is
	measurement = 10;
	Serial.println("setRate: changed measurement to minimum 10 ms");
    }
    // maximum has to fit in 16 bits and since that is the size of the input
    // parameter I don't have to check it.
    if ( navigationSol < 1 ){ 
	navigationSol = 1;
	Serial.println("setRate: changed navigationSol to minimum 1");
    }
    if ( navigationSol > 127 ){ 
	navigationSol = 127;
	Serial.println("setRate: changed navigationSol to maximum 127");
    }
    uint32_t newWatchdog = (uint32_t)measurement * navigationSol * 5;
    if (newWatchdog < 10000){
	newWatchdog = 10000; // don't declare the receiver dead over a few late messages
    }
    Serial.print("setRate: measurement: ");
    Serial.println(measurement);
    Serial.print("setRate: navigationSol: ");
    Serial.println(navigationSol);
    Serial.print("setRate: ubxWatchdog: ");
    Serial.println(newWatchdog);
    rateMsg[PAYLOAD_IDX] = (measurement & 0xFF); // little endian
    rateMsg[PAYLOAD_IDX + 1] = ((measurement/256) & 0xFF);
    rateMsg[PAYLOAD_IDX + 2] = (navigationSol & 0xFF);
    rateMsg[PAYLOAD_IDX + 3] = 0;

    // Only change the timeout if the receiver accepted the new rate
    if (sendCommand(rateMsg) == ubxSUCCESS){
	ubxWatchdog = newWatchdog;
    }
}
void
UbloxM8Q_GPS::resetODO(void)
{
    sendCommand(resetODOmsg);
    // That was easy!
}
void
UbloxM8Q_GPS::setThresh(int speed, uint16_t distance)
{
    if (speed < 1){
	speed = 1;
	Serial.println("setRate: changed speed to minimum 1 cm/s");
    }
    if (speed > 255){ // has to fit in 8 bits
	speed = 255;
	Serial.println("setRate: changed speed to maximum 255 cm/s");
    }
    if ( distance < 1 ){ 
	distance = 1;
	Serial.println("setRate: changed distance to minimum 1 m");
    }
    // no max on distance

    Serial.print("setThresh: speed: ");
    Serial.println(speed);
    Serial.print("setThresh: distance: ");
    Serial.println(distance);

    // Set the mask to only modify static hold parameters
    cfgNav5[PAYLOAD_IDX] = 0x40;
    cfgNav5[PAYLOAD_IDX + 1] = 0x00;
    // speed threshold
    cfgNav5[PAYLOAD_IDX + 22] = (speed & 0xFF);
    // distance
    cfgNav5[PAYLOAD_IDX + 28] = (distance & 0xFF);
    cfgNav5[PAYLOAD_IDX + 29] = ((distance/256) & 0xFF);

    sendCommand(cfgNav5);
}


// Won't return until command is ACKed or an error is detected.
enum ubxReturn
UbloxM8Q_GPS::sendCommand(uint8_t *msg) // send a ubx format command
{

    ubxReturn r = ubxPENDING;
    while( r == ubxPENDING ){
	r = writeUBX(msg);
    }
    if (r == ubxBAD_LENGTH){
	Serial.println("gps.sendCommand got ubxBAD_LENGTH");
    }
    if (r == ubxBAD_CHECK_SUM){
	Serial.println("gps.sendCommand got ubxBAD_CHECK_SUM");
    }
    if (r == ubxNO_ACK){
	Serial.println("gps.sendCommand got ubxNO_ACK");
    }
    return r;
}

// writeUBX(msg) - 
//   msg does not need the check sum calculated but it must have room 
//   for it, and it must be writable.
//   Calculates the check sum and writes the message, then checks for 
//   the ACK.
enum ubxReturn
UbloxM8Q_GPS::writeUBX(uint8_t *msg) // send a ubx format command
{
    int length, i;
    static unsigned long prev;
    unsigned long cur;
    static int msgType;
    static bool sent = false;

    // The ACK list is also updated by read() on the timer thread
    RxGuard guard(rxLock);

    if (!sent){
	msgType = ubxMsgType(msg);

	ackedMsg.clearAck(msgType);

	ubxAddCS(msg);
	Serial.print("writeUBX: type: "); Serial.println(msgType,HEX);
	showUbx(msg);
	length = ubx_length(msg);
	// length is just the payload, add 6 for header and 2 for CS
	for (i = 0; i < length + 8; i++ ){
	    Serial1.write(msg[i]);
	}
	prev = millis();
	sent = true;
    } else {
	read();
	if (!ackedMsg.isAcked(msgType)){
	    cur = millis();
	    if (cur - prev > 2000){
		sent = false;
		return ubxNO_ACK;
	    }
	} else {
	    sent = false;
	    return ubxSUCCESS;
	}
    }
    return ubxPENDING;
}


// Read data from the receiver, if we get a whole message parse it.
// Must be called repeatedly. It does not return info about what was
// read. Caller must check other results such as data access methods or
// ackedMsg().
// May be called from more than one thread (the software timer and the
// application thread while it waits for an ACK), so the parser and its
// buffer are protected by rxLock. The buffer is a member, not a local,
// because a message is normally assembled across several calls.
void
UbloxM8Q_GPS::read()
{
    enum ubxReturn r;

    if (!gpsOn){
	return;
    }
    RxGuard guard(rxLock);
    while(Serial1.available()){
	r = readUBXmsg(rxMsg, &rxType, &rxLength);
	if (r == ubxBAD_LENGTH){
	    Serial.println("gps.read got ubxBAD_LENGTH");
	}
	if (r == ubxBAD_CHECK_SUM){
	    Serial.println("gps.read got ubxBAD_CHECK_SUM");
	}
	if (r == ubxSUCCESS){
	    lastMsgMs = millis();
	    parse(rxMsg, rxType, rxLength);
	}
	// else r == ubxPENDING, do nothing
    }
}

// True when the receiver is supposed to be running but no valid message
// has arrived within the ubxWatchdog period. The application decides what
// to do about it (see AssetTracker2::gpsStalled()).
bool
UbloxM8Q_GPS::stalled()
{
    uint32_t last = lastMsgMs; // read before millis() so the difference can't go negative
    return gpsOn && ((millis() - last) > ubxWatchdog);
}

// True only while the receiver is reporting a valid fix and that report
// is recent.
bool
UbloxM8Q_GPS::hasFix()
{
    uint32_t last = lastPvtMs;
    return fix && ((millis() - last) <= ubxWatchdog);
}


void
UbloxM8Q_GPS::parse(uint8_t *msg, int type, int length)
{
    int typeAcked;

    // type is combination of class and id
    switch(type) {
    case ACK_TYPE:
	typeAcked = ubxMsgParm(&msg[PAYLOAD_IDX]);
	ackedMsg.setAck(typeAcked);
	Serial.print("parse: ACK_TYPE: ");
	Serial.print("message acked: ");
	Serial.println(typeAcked, HEX);
	break;
    case NAK_TYPE:
	typeAcked = ubxMsgParm(&msg[PAYLOAD_IDX]);
	ackedMsg.clearAck(typeAcked);
	Serial.print("parse: NAK_TYPE: ");
	Serial.print("message NAKed: ");
	Serial.println(typeAcked, HEX);
	break;
    case NAV_POSLLH_TYPE:
//	Serial.print("parse: type: NAV_POSLLH_TYPE ");
//	Serial.println(type, HEX);
	// showUbx(msg);
	parsePOSLLH(msg);
	break;
    case NAV_PVT_TYPE:
//	Serial.print("parse: type: NAV_PVT_TYPE ");
//	Serial.println(type, HEX);
//	showUbx(msg);
	parsePVT(msg);
	break;
    case NAV_ODO_TYPE:
//	Serial.print("parse: type: NAV_ODO_TYPE ");
//	Serial.println(type, HEX);
	// showUbx(msg);
	parseODO(msg);
	break;
    default:
	Serial.print("parse: Unknown type: ");
	Serial.println(type, HEX);
	showUbx(msg);
	break;
    }
}

void
UbloxM8Q_GPS::parsePOSLLH(uint8_t *msg)
{
    uint8_t *data = &msg[PAYLOAD_IDX]; 

    // The ublox specification for the data fields lists byte offests
    // into the message payload.
    // Setting this pointer and then using hardcoded indexes allows
    // me to directly compare the code with the ublox spec.

    // skipping Time Of Week, better to use other timestamps. See ublox spec.
    longitude =              // Longitude, deg
    longitudeDegrees =
       ((float) getLong(&data[4])) * 0.0000001;
    latitude =              // Latitude, deg
    latitudeDegrees =
       ((float) getLong(&data[8])) * 0.0000001;
    geoidheight =        // Height above ellipsoid, mm
       (float)getLong(&data[12]);  // WGS84 unless user altered. See ublox spec.
    altitude =          // Height above Mean Sea Level, mm (signed)
       (float)getLong(&data[16]);
    horzAcc =                // Horizontal Accuracy estimate, mm
       getULong(&data[20]);
    vertAcc =                // Vertical Accuracy extimate, mm
       getULong(&data[24]);

}

void
UbloxM8Q_GPS::parsePVT(uint8_t *msg)
{
    uint8_t *data = &msg[PAYLOAD_IDX];

    // The ublox specification for the data fields lists byte offests
    // into the message payload.
    // Setting this pointer and then using hardcoded indexes allows
    // me to directly compare the code with the ublox spec.

    lastPvtMs = millis();
    tow = getULong(&data[0]);
    // gnssFixOk bit in Bitfield Flags
    if ((data[21] & 0x01) == 0){
	// Fix not Ok. Say so, otherwise the last position keeps being
	// reported as current after the signal is lost.
	fix = false;
	return;
    }
    uint8_t flags = data[11];
    if ( flags & 0x01){
	dateValid = true;
	year = getUInt(&data[4]);  // UTC
	month = data[6];
	day = data[7];
    } else {
	// If date is not valid, stick with date we already have
	dateValid = false;
    }
    if ( flags & 0x02){
	timeValid = true;
	hour = data[8];
	minute = data[9];
	seconds = data[10];
    } else {
	// If time is not valid, stick with time we already have
	timeValid = false;
    }
    // I'm skipping time accuracy estimate,

    // Calculating milliseconds from nanoseconds. Nanoseconds is actually
    // a +/- adjustment on time. Rather than try to decrement everything 
    // when nanoseconds is negative on Jan 1 at midnight, I'm going to ignore
    // it when it is negative and just use it when it is positive.
    int32_t nanoseconds = getLong(&data[16]);
    if (nanoseconds > 0){
        milliseconds = nanoseconds/1000000;
    } else {
        milliseconds = 0;
    }

     // 0:none,1:dead reck,2:2d,3:3d,4:GNSS&reck,5:time only fix
    fixquality = data[20];
    // I tried it with fixquality of 3 for a long time, now try 2 or 3
    fix =  ((fixquality == 3) || (fixquality == 2));
    satellites = data[23];
    longitude =              // Longitude, deg
    longitudeDegrees =
          ((float) getLong(&data[24]))*0.0000001;
    latitude =              // Latitude, deg
    latitudeDegrees =
       ((float) getLong(&data[28]))*0.0000001;
    geoidheight = // Height above ellipsoid, mm
       (float)getLong(&data[32]); // WGS84 unless user altered. See ublox spec.
    altitude =          // Height above Mean Sea Level, mm (signed)
       (float)getLong(&data[36]);
    horzAcc =                // Horizontal Accuracy estimate, mm
       getULong(&data[40]);
    vertAcc =                // Vertical Accuracy extimate, mm
       getULong(&data[44]);
    velN =                   // North velocity, mm/s
        getLong(&data[48]);
    velE =                   // East velocity, mm/s
        getLong(&data[52]);
    velD =                   // Down velocity, mm/s
        getLong(&data[56]);
    gSpeed =                   // Ground speed, mm/s
        getLong(&data[60]);
    headingMot =                   // heading of motion, degrees 1e-5
        (float) (getLong(&data[64])/100000.0);
    headingVeh =                   // heading of vehicle, degrees 1e-5
        (float) (getLong(&data[84])/100000.0);
    if (data[21] & 0x20){
	headVehValid = true;
    } else {
	headVehValid = false;
    }
    speed = ((float)gSpeed/1000.0); // from integer mm/s to float m/s
    // Skipping heading and speed accuracy
    //   pDOP, magnetic declination and mag decl accuracy

}

// read the odometer data
void
UbloxM8Q_GPS::parseODO(uint8_t *msg)
{
    uint8_t *data = &msg[PAYLOAD_IDX]; 

    // The ublox specification for the data fields lists byte offests
    // into the message payload.
    // Setting this pointer and then using hardcoded indexes allows
    // me to directly compare the code with the ublox spec.

    // Ignoring version, reserved, and iTOW
    odoTrip = getULong(&data[8]); // since reset, m

//    Serial.print("odoTrip: ");
//    Serial.println(odoTrip);
    odoTotal = getULong(&data[12]); // since power up, m
//    Serial.print("odoTotal: ");
//    Serial.println(odoTotal);
    odoAcc = getULong(&data[16]); // m 1-sigma
//    Serial.print("odoAcc: ");
//    Serial.println(odoAcc);
}
// Consumes at most one byte from the receiver per call.
// msg must have room for 256 bytes.
enum ubxReturn
UbloxM8Q_GPS::readUBXmsg(uint8_t *msg, int *type, int *length)
{
    static enum readingUBX {
	waitB5,
	ubxLength,
	payload,
	checksum
    } state = waitB5;
    static int i;
    uint8_t c;

    if (!Serial1.available()){
	return ubxPENDING;
    }
    c = Serial1.read();

    switch (state){
    case waitB5:
	if (c == 0xb5){
	    msg[0] = c;
	    i = 1;
	    state = ubxLength;
	} else {
	    Serial.print(">");
	    Serial.print(c, HEX);
	}
	return ubxPENDING;
	break;
    case ubxLength:
	// Read header, class, id, length
	if (i == 1 && c != 0x62){
	    // Not the second sync byte, so this was not the start of a message
	    if (c != 0xb5){
		state = waitB5;
	    }
	    return ubxPENDING;
	}
	msg[i] = c;
	i++;
	if (i == 6){ // read all the header
	    *length = ubx_length(msg);
	    if ( *length > (256 - 8) ){
		Serial.print("readUBXmsg bad length: ");
		Serial.println(*length);
		state = waitB5;
		return ubxBAD_LENGTH;
	    } else {
		*type = ubxMsgType(msg);
		// A message with no payload goes straight to the check sum
		state = (*length == 0) ? checksum : payload;
	    }
	}
	return ubxPENDING;
	break;
    case payload:
	msg[i] = c;
	i++;
	if ( i >= (6 + *length)){
	    state = checksum;
	}
	return ubxPENDING;
	break;
    case checksum:
	msg[i] = c;
	i++;
	if ( i >= (8 + *length)){
	    uint8_t ck_a, ck_b;
	    state = waitB5; // going to start over error or not
	    ubxCS(msg, &ck_a, &ck_b);
	    if ( (msg[i-2] != ck_a) || (msg[i-1] != ck_b)){
		Serial.println("readUBXmsg bad check sum ");
		return ubxBAD_CHECK_SUM;
	    }
	    // c'est tout
	    return ubxSUCCESS;
	}
	return ubxPENDING;
	break;
    default:
	state = waitB5;
	break;
    }
    return ubxPENDING;
}

// Calculates and sets checksums for ublox proprietary messages.
// Checksum algorithm according to 
// Uses 8-bit Fletcher Algorithm, which is used in TCP standard RFC 1145.
// Note that this is a different CS algorithm than is used for NMEA sentences.
// Sentence must include two extra bytes on the end that will be overwritten 
// by the check sum bytes.
void
UbloxM8Q_GPS::ubxCS( uint8_t *msg, uint8_t *ck_a, uint8_t *ck_b) 
{
    int i;
    *ck_a = 0, *ck_b = 0;
    // The CS doesn't include the first two bytes. The length field just 
    // gives the length of the payload. So the CS is calculated over 4 bytes 
    // (class, id, and length fields), plus the payload. 
    int length = ubx_length(msg);
    for(i=0;i < length + 4;i++)
    {
	*ck_a = *ck_a + msg[i + 2]; // skip the 2 byte header
	*ck_b = *ck_b + *ck_a;
    }
}
void
UbloxM8Q_GPS::ubxAddCS(uint8_t *msg)
{
    int length = ubx_length(msg);
    ubxCS(msg, &msg[length+6], &msg[length+7]);
}
void
UbloxM8Q_GPS::showUbx(uint8_t *msg)
{
    int i;
    int length = ubx_length(msg);

    // length plus header (2) + class (1) + id (1) + length field (2) + cs (2)
    for (i = 0; i < length + 8; i++){
	Serial.print(msg[i], HEX); Serial.print(",");
    }
    Serial.println(" ");
}

// routines to access various sizes and types of data from ubx messages
int
UbloxM8Q_GPS::getInt(uint8_t *m)
{
    return((m[1]*256)+(m[0]));
}
uint16_t 
UbloxM8Q_GPS::getUInt(uint8_t *m)
{
    return((m[1]*256)+(m[0]));
}
int32_t
UbloxM8Q_GPS::getLong(uint8_t *m)
{
    return (int32_t)getULong(m);
}
uint32_t 
UbloxM8Q_GPS::getULong(uint8_t *m)
{
    // Unsigned shifts: multiplying m[3] as an int overflows for values >= 0x80
    return (((uint32_t)m[3] << 24) | ((uint32_t)m[2] << 16) | ((uint32_t)m[1] << 8) | (uint32_t)m[0]);
}
float 
UbloxM8Q_GPS::getFloat(uint8_t *m)
{
    union ctof_type {
	uint8_t c[4];
	float f;
    } *ctof;
    ctof = (union ctof_type *)m;
    return ctof->f;
}


void
UbloxM8Q_GPS::delayButProcess(uint32_t time)
{
    uint32_t start = millis();
    uint32_t cur = millis();
    while (cur - start < time){
	if(Particle.connected()){
	    Particle.process();
	}
	cur = millis();
    }

}

// Used only by UbloxM8Q_GPS.begin() to read the initial report of version
// information from the receiver, write it to Serial, and then turn off
// the NMEA ascii messages.
void
UbloxM8Q_GPS::readVer()
{
    int linecnt = 0;
    char c;
    uint32_t start = millis();
    // Give up after 5 seconds. A receiver that is missing, dead, or already
    // switched to UBX-only output must not hang setup() forever.
    while (linecnt < 10 && (millis() - start) < 5000){
	while (Serial1.available()){
	    c = Serial1.read();
	    if ( c == '\n'){
		linecnt++;
	    }
	    Serial.print(c);
	} 
    }
    Serial1.print("$PUBX,41,1,3,1,9600,0*16\r\n");  // Turn off nmea
    // Must wait for nmea data to rattle out or it gets confused
    // with ubx data.
    delay(2000);
}

// report error from begin()
void
UbloxM8Q_GPS::reportError(int state, enum ubxReturn ret)
{
    Serial.print("begin failed, state: ");
    Serial.print(state);
    Serial.print(" error: ");
    Serial.println(ret);
}


// AckedMsg methods
AckedMsg::AckedMsg()
{
    for(uint16_t i=0;i<sizeof(ackList)/sizeof(int);i++) {
	ackList[i] = 0;
    }
}
int 
AckedMsg::setAck(int msgType)
{
    // Already recorded? Don't use up another slot (ACKs for commands that
    // are never cleared, like the antenna selection, used to fill the list).
    if (isAcked(msgType)){
	return 0;
    }
    for(uint16_t i=0;i<sizeof(ackList)/sizeof(int);i++) {
	if (!ackList[i]){
	    ackList[i] = msgType;
	    return 0;
	}
    }
    return -1;
}
void
AckedMsg::clearAck(int msgType)
{
    for(uint16_t i=0;i<sizeof(ackList)/sizeof(int);i++) {
	if (ackList[i] == msgType){
	    ackList[i] = 0;
	}
    }
}
bool
AckedMsg::isAcked(int msgType)
{
    for(uint16_t i=0;i<sizeof(ackList)/sizeof(int);i++) {
	if (ackList[i] == msgType){
	    return true;
	}
    }
    return false;
}
