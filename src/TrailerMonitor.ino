// Electron Sample Application for fault tolerance and problem debugging techniques
// Requires system firmware 0.6.1 or later!
//
// Original code location:
// https://github.com/rickkas7/electronsample

#include "Particle.h"

#include "AssetTracker2.h"
#include "PowerCheck.h"
#include "PietteTech_DHT.h"
#include "Tinker.h"
#include "system_event.h"

#include "AppWatchdogWrapper.h"
#include "BatteryCheck.h"
#include "ConnectionCheck.h"
#include "ConnectionEvents.h"
#include "SessionCheck.h"
#include "Tester.h"

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11		// DHT 11
#define DHTTYPE DHT22		// DHT 22 (AM2302)
//#define DHTTYPE DHT21		// DHT 21 (AM2301)
//#define DHTVcc   D2                 // Digital pin to power the sensor
#define DHTPIN   D3                 // Digital pin for communications
//#define DHTGnd   D4                 // Digital pin for sensor GND
#define DHT_SAMPLE_INTERVAL   2000  // Sample every two seconds


#define APP_WATCHDOG_TIMEOUT	60000 // milliseconds

// Bit Masks to enable/disable transmission/serial printing particular events

#define TRANSMITTINGGPSDATA 0x00000001
#define TRANSMITTINGACCDATA 0x00000002
#define TRANSMITTINGPWRDATA 0x00000004
#define TRANSMITTINGDHTDATA 0x00000008
#define SERIALBTNDATA	      0x01000000
#define SERIALCONNDATA      0x02000000
#define SERIALSETUPDATA     0x04000000
#define SERIALLOOPDATA      0x08000000
#define SERIALGPSDATA       0x10000000
#define SERIALACCDATA       0x20000000
#define SERIALPWRDATA       0x40000000
#define SERIALDHTDATA       0x80000000

// If you are using a 3rd party SIM card, put your APN here. See also
// the call to Particle.keepAlive in setup()
// STARTUP(cellular_credentials_set("YOUR_APN_GOES_HERE", "", "", NULL));

SerialLogHandler logHandler;

// We use retained memory keep track of connection events; these are saved and later uploaded
// to the cloud even after rebooting
void startupMacro() {
	System.enableFeature(FEATURE_RETAINED_MEMORY);
	System.enableFeature(FEATURE_RESET_INFO);
}
STARTUP(startupMacro());

// System threaded mode is not required here, but it's a good idea with 0.6.0 and later.
// https://docs.particle.io/reference/firmware/electron/#system-thread
SYSTEM_THREAD(ENABLED);

// SEMI_AUTOMATIC mode or system thread enabled is required here, otherwise we can't
// detect a failure to connect
// https://docs.particle.io/reference/firmware/electron/#semi-automatic-mode
SYSTEM_MODE(SEMI_AUTOMATIC);


// Manage connection-related events with this object. Publish with the event name "connEventStats" and store up to 32 events
// in retained memory. This provides better visibility into what your Electron is using but doesn't use too much data.
ConnectionEvents connectionEvents("connEventStats");

// Check session by sending and receiving an event every hour. This can help troubleshoot problems where
// your Electron is online but not communicating
SessionCheck sessionCheck(3600);

// Monitors the state of the connection, and sends this data using the ConnectionEvents.
// Handy for visibility.
ConnectionCheck connectionCheck;

// Tester adds a function that makes it possible exercise some of the feature remotely using functions.
// testerFn is the function and and the second parameter that's a pin to test pin sleep modes.
Tester tester("testerFn", D2);

// BatteryCheck is used to put the device to sleep immediately when the battery is low.
// 15.0 is the minimum SoC, if it's lower than that and not externally powered, it will
// sleep for the number of seconds in the second parameter, in this case, 3600 seconds = 1 hour.
BatteryCheck batteryCheck(15.0, 3600);

// This is a wrapper around the ApplicationWatchdog. It just makes using it easier. It writes
// a ConnectionEvents event to retained memory then does System.reset().
AppWatchdogWrapper watchdog(APP_WATCHDOG_TIMEOUT);


//
//  Trailer Monitor Code Starts Here
//


// Creating an AssetTracker named 't' for us to reference
AssetTracker2 t = AssetTracker2();
// A FuelGauge named 'fuel' for checking on the battery state
FuelGauge fuel = FuelGauge();
// A PowerCheck object named 'pc' for watching the power state of the usb socket
PowerCheck pc = PowerCheck();
// A DHT object named dht to access the DHT22 temperature and humidity sensor
PietteTech_DHT dht(DHTPIN, DHTTYPE, NULL);

// DHT flag to indicate we started acquisition
bool bDHTstarted;
// keep track of what antenna we are using for the GPS receiver.
bool gpsAntennaExternal = true;
// Used to keep track of the last time we published gps data
long lastGPSPublish = 0;
// Used to keep track of the last time we published acceleration data
long lastACCPublish = 0;
// Used to keep track of the last time we published temperature / humidity data
long lastDHTPublish = 0;
// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayGPSMinutes = 60;
// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayACCMinutes = 1;
// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayDHTMinutes = 10;
// Threshold to trigger a publish
// 9000 is VERY sensitive, 12000 will still detect small bumps
int accelThreshold = 9000;
// string that will hold the accelleration values from the last over threshold event
String pubAccel = "";
// Did we have power the last time we checked -- only want to publish on the transition to no power
bool lastPower = true; // assume we do, then we will report power lost on boot if we really don't

// Set whether you want the device to publish data to the internet by default here.
// 1 will Particle.publish AND Serial.print, 0 will just Serial.print
// Extremely useful for saving data while developing close enough to have a cable plugged in.
// You can also change this remotely using the Particle.function "tmode" defined in setup()
//int transmittingData = ( TRANSMITTINGGPSDATA | TRANSMITTINGACCDATA | TRANSMITTINGPWRDATA | TRANSMITTINGDHTDATA | SERIALLOOPDATA | SERIALSETUPDATA | SERIALGPSDATA | SERIALACCDATA | SERIALPWRDATA | SERIALDHTDATA );
//int transmittingData = ( TRANSMITTINGGPSDATA | TRANSMITTINGACCDATA | TRANSMITTINGPWRDATA | TRANSMITTINGDHTDATA | SERIALSETUPDATA | SERIALGPSDATA | SERIALACCDATA | SERIALPWRDATA);
int transmittingData = ( TRANSMITTINGGPSDATA | TRANSMITTINGACCDATA | TRANSMITTINGPWRDATA | TRANSMITTINGDHTDATA );
// Run the GPS off a timer interrupt.
// read all bytes available, if an entire message was received,
// parse it store the data for access by the get routines.
void callGPS() {
     t.updateGPS();
}
// The period is based on the baud rate of the serial port
// connected to the gps.
Timer timer(50, callGPS);

//
//
//
void setup() {
	//
	Serial.begin(9600);

	// Wait to allow particle serial monitor to get connected
  // This lets you see the version info from the ublox receiver
  delay(10000);

  // Set up power monitoring routines.
  pc.setup();
  // Set up temperature and humidity sensor routines
  dht.begin();
  // Sets up all the necessary AssetTracker bits
  t.begin();
  // Enable the GPS module. Defaults to off to save power.
  // Takes 1.5s or so because of delays.
  t.gpsOn();

  SetGPSAntenna("external"); // set up for external antenna

	connectionCheck.setup();
	// If you're battery powered, it's a good idea to enable this. If a cellular or cloud connection cannot
	// be made, a full modem reset is first done. If that doesn't resolve the problem, on the second and
	// subsequent failures, the Electron will sleep for this many seconds. The intention is to set it to
	// maybe 10 - 20 minutes so if there is a problem like SIM paused or a network or cloud failure, the
	// Electron won't continuously try and fail to connect, depleting the battery.
	connectionCheck.withFailureSleepSec(15 * 60);

	// We store connection events in retained memory. Do this early because things like batteryCheck will generate events.
	connectionEvents.setup();

	// Check if there's sufficient battery power. If not, go to sleep immediately, before powering up the modem.
	batteryCheck.setup();

	// Set up the other modules
	sessionCheck.setup();

	tester.setup();

	// These functions are useful for remote diagnostics. Read more below.
  Particle.function("tMask", transmitMode);
  Particle.function("setgpsant", SetGPSAntenna);
  Particle.function("pubVal", pubValue);
  Particle.function("setACCThr", accelThresholder);
  Particle.function("setGPSDly", setDelayGPSMinutes);
  Particle.function("setACCDly", setDelayACCMinutes);
  Particle.function("setDHTDly", setDelayDHTMinutes);
  Particle.function("gpsRate", gpsRate);
  Particle.function("resetODO", resetODO);
  Particle.function("showAll", showAll);
  Particle.function("postValue", postValue);
  // Register all the Tinker functions
  Particle.function("digitalread", tinkerDigitalRead);
  Particle.function("digitalwrite", tinkerDigitalWrite);
  Particle.function("analogread", tinkerAnalogRead);
  Particle.function("analogwrite", tinkerAnalogWrite);
  // These variables are useful for remote diagnostics. Read more below.
  Particle.variable("lastPower", lastPower);
  Particle.variable("accelThresh", accelThreshold);
  Particle.variable("transmitMask", transmittingData);
  Particle.variable("lastGPSPub", lastGPSPublish);
  Particle.variable("lastACCPub", lastACCPublish);
  Particle.variable("lastDHTPub", lastDHTPublish);
  Particle.variable("delayGPSMin", delayGPSMinutes);
  Particle.variable("delayACCMin", delayACCMinutes);
  Particle.variable("delayDHTMin", delayDHTMinutes);
  Particle.variable("gpsExternal", gpsAntennaExternal);

	// If you are using a 3rd party SIM card, you will likely have to set this
	// https://docs.particle.io/reference/firmware/electron/#particle-keepalive-
	// Particle.keepAlive(180);

	// allow the setup button to enable all serial debug messages
	System.on(button_click, button_clicked);

	// We use semi-automatic mode so we can disconnect if we want to, but basically we
	// use it like automatic, as we always connect initially.
	Particle.connect();

	// Start reading from the gps
	timer.start();

	delay(DHT_SAMPLE_INTERVAL); // DHT 22 minumum sampling period

	if ((transmittingData & SERIALSETUPDATA) == SERIALSETUPDATA)
		Serial.println("End of setup() function");
}

void loop() {
	batteryCheck.loop();
	sessionCheck.loop();
	connectionCheck.loop();
	connectionEvents.loop();
	tester.loop();

  checkAccelStatus();
  checkGPSStatus();
  checkPowerStatus();
  checkDHTStatus();

  if ((transmittingData & SERIALLOOPDATA) == SERIALLOOPDATA)
    Serial.println("End Of loop() function");
}




void button_clicked(system_event_t event, int param)
{
    int times = system_button_clicks(param);
		if ((transmittingData & SERIALBTNDATA) == SERIALBTNDATA)
    	Serial.printlnf("button was clicked %d times", times);
		if ((transmittingData & SERIALLOOPDATA) == 0)
		 	transmittingData |= ( SERIALBTNDATA | SERIALCONNDATA | SERIALLOOPDATA | SERIALSETUPDATA | SERIALGPSDATA | SERIALACCDATA | SERIALPWRDATA | SERIALDHTDATA );
		else
		  transmittingData &= ~( SERIALBTNDATA | SERIALCONNDATA | SERIALLOOPDATA | SERIALSETUPDATA | SERIALGPSDATA | SERIALACCDATA | SERIALPWRDATA | SERIALDHTDATA );
}

// Remotely change the trigger threshold!
int accelThresholder(String command) {
    accelThreshold = atoi(command);
    return accelThreshold;
}
// Remotely change the publishing delay for GPS!
int setDelayGPSMinutes(String command) {
    delayGPSMinutes = atoi(command);
    return delayGPSMinutes;
}

// Remotely change the publishing delay for ACC!
int setDelayACCMinutes(String command) {
    delayACCMinutes = atoi(command);
    return delayACCMinutes;
}

// Remotely change the publishing delay for DHT!
int setDelayDHTMinutes(String command) {
    delayDHTMinutes = atoi(command);
    return delayDHTMinutes;
}

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command) {
    transmittingData = atoi(command);
    return transmittingData;
}

bool SetGPSAntenna(String command) {
  if (command == "internal") {
    gpsAntennaExternal = false;
    return t.antennaInternal();
  } else if (command == "external") {
    gpsAntennaExternal = true;
    return t.antennaExternal();
  } else {
    return false;
  }
}


int pubValue(String command) {
  if (command == "pwr") {
    return pwrPublish(command);
  } else if (command == "gps") {
    return gpsPublish(command);
  } else if (command == "acc") {
    return accPublish(command);
  } else if (command == "env") {
    return envPublish(command);
  } else
	  Particle.publish("LJCMDERR", String::format("{\"cmd\":\"%s\"}", command.c_str()), 60, PRIVATE);
    return 0;
}
// Actively ask for a GPS reading if you're impatient. Only publishes if there's
// a GPS fix, otherwise returns '0'
int gpsPublish(String command) {
    if (t.gpsFix()) {
			  time_t time = Time.now();
				// Short publish names save data!
			  Particle.publish("LJGPSFIX", String::format("{\"la\":%f,\"lo\":%f,\"ht\":%f,\"ac\":%f,\"tm\":\"%s\"}",t.readLatDeg(),t.readLonDeg(),(t.getAltitude() / 1000),(t.getGpsAccuracy() / 1000),Time.format(time, TIME_FORMAT_ISO8601_FULL).c_str()), 60, PRIVATE);
        return 1;
    } else {
        return 2;
    }
}

// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '2' if below
int pwrPublish(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("LJPWRSTAT", String::format("{\"s\": %d,\"n\": 0,\"v\":%.2f,\"c\":%.2f}",pc.getHasPower(),fuel.getVCell(),fuel.getSoC()), 60, PRIVATE );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC()>10){ return 1;}
    // if you're running out of battery, return 2
    else { return 2;}
}

// Lets you remotely check the accelleration status by calling the function "readXYZmagnitude"
// Triggers a publish with the info (so subscribe or watch the dashboard)
int accPublish(String command){
  Particle.publish("LJACELRT", String::format("{\"x\":%d,\"y\":%d,\"z\":%d,\"m\":%d}", t.readX(), t.readY(), t.readZ(), t.readXYZmagnitude()), 60, PRIVATE);
  return 1;
}

// Lets you remotely check the environmental status by calling the functions "getHumidity" and "getFahrenheit"
// Triggers a publish with the info (so subscribe or watch the dashboard)
int envPublish(String command){

    int result = dht.acquireAndWait(DHT_SAMPLE_INTERVAL);  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  	float h = dht.getHumidity();
  // Read temperature as Fahrenheit
  	float f = dht.getFahrenheit();

  // Check if any reads failed and exit early (to try again).
  	if (isnan(h) || isnan(f) || result != DHTLIB_OK) {
			if ((transmittingData & SERIALDHTDATA) == SERIALDHTDATA)
      	Serial.println("Failed to read from DHT sensor!");
  		return 2;
  	}

    Particle.publish("LJENVMT", String::format("{\"t\":%.2f,\"h\":%.2f}",f,h), 60, PRIVATE);
    return 1;
}


int checkPowerStatus() {
    bool currentPower = pc.getHasPower();
    if (lastPower != currentPower) {
      // Only publish if we're in transmittingData mode 1;
      if ((transmittingData & TRANSMITTINGPWRDATA) == TRANSMITTINGPWRDATA) {
        Particle.publish("LJPWRSTAT", String::format("{\"s\": %d,\"n\": 1,\"v\":%.2f,\"c\":%.2f}",currentPower,fuel.getVCell(),fuel.getSoC()), 60, PRIVATE );
        lastPower = currentPower; // note that we only update power status if we actually transmitted data so that when data is turned back on we will report
      }
    }
    return 1;
}

void checkGPSStatus() {
    // if the current time - the last time we published is greater than your set delay...
    if (millis()-lastGPSPublish > ((unsigned long) delayGPSMinutes*60*1000)) {

        // GPS requires a "fix" on the satellites to give good data,
        // so we should only publish data if there's a fix
        if (t.gpsFix()) {
            // Only publish if we're in transmittingData mode 1;
            if ((transmittingData & TRANSMITTINGGPSDATA) == TRANSMITTINGGPSDATA) {
							time_t time = Time.now();
							Time.format(time, TIME_FORMAT_ISO8601_FULL); // 2004-01-10T08:22:04-05:15
                // Short publish names save data!
              Particle.publish("LJGPSFIX", String::format("{\"la\":%f,\"lo\":%f,\"ht\":%f,\"ac\":%f,\"tm\":\"%s\"}",t.readLatDeg(),t.readLonDeg(),(t.getAltitude() / 1000),(t.getGpsAccuracy() / 1000),Time.format(time, TIME_FORMAT_ISO8601_FULL).c_str()), 60, PRIVATE);
                // Remember when we published
              lastGPSPublish = millis();
            }
            // but always report the data over serial for local development
						if ((transmittingData & SERIALGPSDATA) == SERIALGPSDATA)
            	Serial.println(t.readLatLon());
        }
    }
}

void checkAccelStatus() {
      // Check if there's been a big acceleration
      int readMagnitude = t.readXYZmagnitude();
      if (readMagnitude > accelThreshold) {
          pubAccel = String::format("{\"x\":%d,\"y\":%d,\"z\":%d,\"m\":%d}", t.readX(), t.readY(), t.readZ(), readMagnitude);
					if ((transmittingData & SERIALACCDATA) == SERIALACCDATA)
          	Serial.println(pubAccel);
      }
      // if the current time - the last time we published is greater than your set delay...
      if (millis()-lastACCPublish > ((unsigned long) delayACCMinutes*60*1000)) {
          if ((transmittingData & TRANSMITTINGACCDATA) == TRANSMITTINGACCDATA) {
            // only publish if we had an accelleration event occur since our last publication time
            if (pubAccel != "") {
              // Short publish names save data!
              Particle.publish("LJACELRT", pubAccel, 60, PRIVATE);
							gpsPublish(""); // publish position if we get an accelleration alert
              // Remember when we published
              lastACCPublish = millis();
              pubAccel = "";
            }
          }
      }
}

void checkDHTStatus() {
  static uint32_t msLastSample = 0;
  int result = 0;
  if (millis() - msLastSample <  DHT_SAMPLE_INTERVAL) return;

  if (!bDHTstarted) {               // start the sample
		if ((transmittingData & SERIALDHTDATA) == SERIALDHTDATA) {
      Serial.println("\r\nRetrieving information from DHT sensor. ");
    }
    dht.acquire();
    bDHTstarted = true;
  }

  if (!dht.acquiring()) {           // has sample completed?
    result = dht.getStatus();

		if ((transmittingData & SERIALDHTDATA) == SERIALDHTDATA) {
      Serial.print("Read sensor: ");
      switch (result) {
        case DHTLIB_OK:
          Serial.println("OK");
          break;
        case DHTLIB_ERROR_CHECKSUM:
          Serial.println("Error\n\r\tChecksum error");
          break;
        case DHTLIB_ERROR_ISR_TIMEOUT:
          Serial.println("Error\n\r\tISR time out error");
          break;
        case DHTLIB_ERROR_RESPONSE_TIMEOUT:
          Serial.println("Error\n\r\tResponse time out error");
          break;
        case DHTLIB_ERROR_DATA_TIMEOUT:
          Serial.println("Error\n\r\tData time out error");
          break;
        case DHTLIB_ERROR_ACQUIRING:
          Serial.println("Error\n\r\tAcquiring");
          break;
        case DHTLIB_ERROR_DELTA:
          Serial.println("Error\n\r\tDelta time to small");
          break;
        case DHTLIB_ERROR_NOTSTARTED:
          Serial.println("Error\n\r\tNot started");
          break;
        default:
          Serial.println("Unknown error");
          break;
      }
    }

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a
  // very slow sensor)
  	float h = dht.getHumidity();
  // Read temperature as Farenheit
  	float f = dht.getFahrenheit();
  // Check if any reads failed and exit early (to try again).
  	if (isnan(h) || isnan(f) || result != DHTLIB_OK) {
  		if ((transmittingData & SERIALDHTDATA) == SERIALDHTDATA)
  			Serial.println("Failed to read from DHT sensor!");
  		return;
  	}
    // if the current time - the last time we published is greater than your set delay...
    if (millis()-lastDHTPublish > ((unsigned long) delayDHTMinutes*60*1000)) {
      if ((transmittingData & TRANSMITTINGDHTDATA) == TRANSMITTINGDHTDATA) {
        // Short publish names save data!
        Particle.publish("LJENVMT", String::format("{\"t\":%.2f,\"h\":%.2f}",f,h), 60, PRIVATE);
        // Remember when we published
        lastDHTPublish = millis();
      }
    }
		if ((transmittingData & SERIALDHTDATA) == SERIALDHTDATA) {
      Serial.printlnf("Humidity (%%): %.2f", dht.getHumidity());
      Serial.printlnf("Temperature (oC): %.2f", dht.getCelsius());
      Serial.printlnf("Temperature (oF): %.2f", dht.getFahrenheit());
      Serial.printlnf("Temperature (K): %.2f", dht.getKelvin());
      Serial.printlnf("Dew Point (oC): %.2f", dht.getDewPoint());
      Serial.printlnf("Dew Point Slow (oC): %.2f", dht.getDewPointSlow());
    }

    bDHTstarted = false;  // reset the sample flag so we can take another
    msLastSample = millis();
  }
}


// Reset the trip odometer
int resetODO(String command) {
    t.gpsResetODO();
    return 1;
}
// Allows changing the measurement rate
int gpsRate(String command) {
    uint16_t rate = atoi(command);
    int nav = atoi(command.substring(command.indexOf(' ')));
		if ((transmittingData & SERIALGPSDATA) == SERIALGPSDATA) {
			Serial.print("rate: ");
	    Serial.print(rate);
	    Serial.print(" nav: ");
	    Serial.println(nav);
		}
    t.gpsRate(rate, nav);
    return 1;
}


// This shows how to access all the (current) values but isn't
// too useful otherwise since the Asset Tracker 2 hardware can't
// see satellites with the serial cable connected.
// It is useful to run this with the serial cable connetected to see
// the names of the values you can read, then disconnect the serial
// connection, put the gps out where it can see satellites. You can
// then call the postValue routine passing it the name of one of these
// vales.
// For details on all values see the ublox spec:
// https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf
int showAll(String cmd)
{

    bool initComplete = t.gpsInitComplete();
    uint8_t hour = t.getHour(); // Time is UTC
    uint8_t minute = t.getMinute();
    uint8_t seconds = t.getSeconds();
    uint8_t month = t.getMonth();
    uint8_t day = t.getDay();
    uint16_t year = t.getYear();
    uint16_t milliseconds = t.getMilliseconds(); // Since last measurement
    float latitude = t.readLat(); // degrees
    float longitude = t.readLon(); // degrees
    float latitudeDegrees = t.readLatDeg(); // degrees
    float longitudeDegrees = t.readLonDeg(); // degrees
    float geoidheight = t.getGeoIdHeight(); // Height above ellipsoid, mm
    float altitude = t.getAltitude(); // Height above mean sea level, mm
    float speed = t.getSpeed();  // m/s
    uint8_t fixquality = t.getFixQuality(); // 0:no fix,1:dead rec,2:2d,3:3d,4:gnss+dead rec,5:time only
    uint8_t satellites = t.getSatellites(); // how many
    uint32_t horzAcc = t.getHaccuracy(); // estimate, mm
    uint32_t vertAcc = t.getVaccuracy(); // estimate, mm
    int32_t velN = t.getVelN(); // North velocity, mm/s
    int32_t velE = t.getVelE(); // East velocity, mm/s
    int32_t velD = t.getVelD(); // Down velocity, mm/s
    int32_t gSpeed = t.getGspeed(); // Ground speed, mm/s
    int32_t odoTrip = t.getOdoTrip(); // m
    int32_t odoTotal = t.getOdoTotal(); // m



    Serial.print("initComplete: ");
    Serial.println(initComplete);
    Serial.print("hour: ");
    Serial.println(hour);
    Serial.print("minute: ");
    Serial.println(minute);
    Serial.print("seconds: ");
    Serial.println(seconds);
    Serial.print("month: ");
    Serial.println(month);
    Serial.print("day: ");
    Serial.println(day);
    Serial.print("year: ");
    Serial.println(year);
    Serial.print("milliseconds: ");
    Serial.println(milliseconds);
    Serial.print("latitude: ");
    Serial.println(latitude);
    Serial.print("longitude: ");
    Serial.println(longitude);
    Serial.print("latitudeDegrees: ");
    Serial.println(latitudeDegrees);
    Serial.print("longitudeDegrees: ");
    Serial.println(longitudeDegrees);
    Serial.print("geoidheight: ");
    Serial.println(geoidheight);
    Serial.print("altitude: ");
    Serial.println(altitude);
    Serial.print("speed: ");
    Serial.println(speed);
    Serial.print("fixquality: ");
    Serial.println(fixquality);
    Serial.print("satellites: ");
    Serial.println(satellites);
    Serial.print("horzAcc: ");
    Serial.println(horzAcc);
    Serial.print("vertAcc: ");
    Serial.println(vertAcc);
    Serial.print("velN: ");
    Serial.println(velN);
    Serial.print("velE: ");
    Serial.println(velE);
    Serial.print("velD: ");
    Serial.println(velD);
    Serial.print("gSpeed: ");
    Serial.println(gSpeed);
    Serial.print("odoTrip: ");
    Serial.println(odoTrip);
    Serial.print("odoTotal: ");
    Serial.println(odoTotal);

    return 1;
}

// Type the name of the value you want to see into the console function
// argument and it will be published.
// See showAll() above for names of values and units.
int postValue(String cmd)
{
    String buf; // for publishing values

    bool initComplete = t.gpsInitComplete();
    uint8_t hour = t.getHour();
    uint8_t minute = t.getMinute();
    uint8_t seconds = t.getSeconds();
    uint8_t month = t.getMonth();
    uint8_t day = t.getDay();
    uint16_t year = t.getYear();
    uint16_t milliseconds = t.getMilliseconds();
    float latitude = t.readLat();
    float longitude = t.readLon();
    float latitudeDegrees = t.readLatDeg();
    float longitudeDegrees = t.readLonDeg();
    float geoidheight = t.getGeoIdHeight();
    float altitude = t.getAltitude();
    float speed = t.getSpeed();
    uint8_t fixquality = t.getFixQuality();
    uint8_t satellites = t.getSatellites();
    uint32_t horzAcc = t.getHaccuracy();
    uint32_t vertAcc = t.getVaccuracy();
    int32_t velN = t.getVelN();
    uint32_t velE = t.getVelE();
    uint32_t velD = t.getVelD();
    int32_t gSpeed = t.getGspeed();
    int32_t odoTrip = t.getOdoTrip();
    int32_t odoTotal = t.getOdoTotal();




    // This implementation is a bit crude but it was easy to implement
    // and simple to read.
    if (cmd == "initComplete"){
        buf = String::format("%d", initComplete);
    } else if (cmd == "hour"){
        buf = String::format("%d", hour);
    } else if (cmd == "minute"){
        buf = String::format("%d", minute);
    } else if (cmd == "seconds"){
        buf = String::format("%d", seconds);
    } else if (cmd == "month"){
        buf = String::format("%d", month);
    } else if (cmd == "day"){
        buf = String::format("%d", day);
    } else if (cmd == "year"){
        buf = String::format("%d", year);
    } else if (cmd == "milliseconds"){
        buf = String::format("%d", milliseconds);
    } else if (cmd == "latitude"){
        buf = String::format("%f", latitude);
    } else if (cmd == "longitude"){
        buf = String::format("%f", longitude);
    } else if (cmd == "latitudeDegrees"){
        buf = String::format("%f", latitudeDegrees);
    } else if (cmd == "longitudeDegrees"){
        buf = String::format("%f", longitudeDegrees);
    } else if (cmd == "geoidheight"){
        buf = String::format("%f", geoidheight);
    } else if (cmd == "altitude"){
        buf = String::format("%f", altitude);
    } else if (cmd == "speed"){
        buf = String::format("%f", speed);
    } else if (cmd == "fixquality"){
        buf = String::format("%d", fixquality);
    } else if (cmd == "satellites"){
        buf = String::format("%d", satellites);
    } else if (cmd == "horzAcc"){
        buf = String::format("%lu", horzAcc);
    } else if (cmd == "vertAcc"){
        buf = String::format("%lu", vertAcc);
    } else if (cmd == "velN"){
        buf = String::format("%ld", velN);
    } else if (cmd == "velE"){
        buf = String::format("%ld", velE);
    } else if (cmd == "velD"){
        buf = String::format("%ld", velD);
    } else if (cmd == "gSpeed"){
        buf = String::format("%lu", gSpeed);
    } else if (cmd == "odoTrip"){
        buf = String::format("%lu", odoTrip);
    } else if (cmd == "odoTotal"){
        buf = String::format("%lu", odoTotal);
    } else {
        buf = "Don't know that one. Check spelling.";
    }
    Particle.publish("LJCMDVAL", cmd + ": " + buf, 60, PRIVATE);


    return 1;
}
