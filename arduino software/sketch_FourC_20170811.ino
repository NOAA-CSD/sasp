#include <math.h>

// RGB radiometer sensor code
//
// version fourA for photodiodes
//
// FourB with barometer
// C updgrades are
//   -- working barometer code
//   -- stow command
//   -- more information in housekeeping data: gate lengths, more
//   -- more digits in some GPS output
//   -- longer long gate

const int unitnumber = 24;   // for now also change elevation constants
//  unit number affects stow command, can be used for tilt correction
//
// constants for photodiode
//
const unsigned long gatelengthsh =  900;     // us
const unsigned long gatelength =  29100;     // us
//
// constants for program flow
//
const int n4readsPerElev =  75;       // calculate solar elevation every 4 nth.  e.g. 70 means every 300 reads
const int n4readsPerGPS =   400;      // read GPS every 4 nth read   ; 800
const int haveGPSflag = 1;
const unsigned long photooffset = 3800;        // subtracted from photodiode because zero volts is 4096 but don't want negatives
// cuts down number of digits (file size) in data file because a lot of short values are near zero volts
//
// constants for pin assignments
//
const int DataValidpin = 4;
const int Convertpin = 10;
const int stepdirpin = 8;
// const int stepsteppin = 3;   // not interchangeable due to timer fuction
const int stepnotenablepin = 5;
// const int elevationPWMpin  = 9;  //timer 1 servo library // not interchangeable due to timer fuction
const int Clockpin = 13;  // compatible with using SPI libary calls
const int Datapin = 12;   // ditto
// analog
const int homepin = 4;
const int GPSselect = 7;
const int VNselect = 11;
const int temppin = A7;   // changed from 2 on Ver 3 board
const int refVpin = 3;
const int baromclock = A1;  // analog used as digital hardcode until figure out how to do A5, ... in pinmode
const int baromdatain = A0;
const int baromdataout = A5;
const int baromselect = A2;
const int POPModepin = A3;
//
// other compiler constants
//
const float twopi = 6.2831853;
const float dtor = 0.017453293; //degrees to radians conversion constant
float msperradian = 5194.8;
//
// constants for elevation angle and tilt
//        default lat, lon, time defined in variable declarations
//
const int homethreshold = 700;
int homesenseflag = 0;
float elevintercept, elevfactor, homeangleoffset;
const float stowangle = -1.570;

//
// variable declarations
//
float lat = 40.0;
float lon = -105.0;
int   year = 15;
int   month = 1;
int   day = 8;
float UDThr =  15.0;
unsigned long millisAtGPS = 0;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float runninghome = 0.0;
unsigned int readcounter = 0, subreadcounter = 0;
unsigned long processmicros = 0, processmicrossh = 0, actualmicros = 0;
unsigned long baromraw = 0, baromteraw = 0;
unsigned long c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0;  // for barometer
long tediff, tediffb, baromte, tediffproduct;
int64_t proffset, prsens, pressure, tediff64;

int DataNotValid = HIGH;
int Gate = LOW;
int hadGPSfix = 0;
int gotGPS = 0;
int gotVN = 0;
int withintimeflag = 1;
int POPMode = 1, StowedFlag = 0, LastPOPMode = 0, ElevSlow = 1;
unsigned long PhotoA, PhotoB, PhotoC, PhotoD;
unsigned long PhotoAsh, PhotoBsh, PhotoCsh, PhotoDsh;
unsigned long gatestart, currentmicros,  rotationmicros;
int homevalue, tempvalue; //, prevhomevalue;
float hour, elevangle;
int  refV;
float corrsintiltx, corrsintilty, sintilt, tilt;  // use in correction
unsigned long tempmillis, lasthomemillis, readingmillis,  inboundreadingmillis, outboundreadingmillis;
float azimuth, phi, relangle, CorrElevApprox, CorrElev, NetApprox, ElevMicrosec;
//
//
// the setup routine runs once on power up or reset:
//
//
void setup() {
  //
  // constants for each unit
  //
  if (unitnumber > 20) {
    msperradian = 5194.8;
  }
  if (unitnumber == 1) {
    elevfactor = -690;  // us in pulse width signal per radian. About 10 us per 1 degree.
    // 573 nominal, depends a little on linkage
    elevintercept = 1795.; // microseconds at horizontal. Depends on linkage arm
    ElevSlow = 2;  // correct elevation half as often on ground unit
    homeangleoffset = 0.0;
  }
  if (unitnumber == 2) {
    elevfactor = -595; //588
    elevintercept = 1745.;//
    homeangleoffset = 0.0;
    ElevSlow = 2;
  }
  if (unitnumber == 3) {
    elevfactor = -612;
    elevintercept = 1744.;
    homeangleoffset = 3.14159;
    ElevSlow = 1;
  }
  if (unitnumber == 21) { //last upload 20180124, last elevation set 20180124
    elevfactor = 928; 
    elevintercept = 985;  
    homeangleoffset = 2.35619449; // 3/4 pi
    ElevSlow = 1;
  }
  if (unitnumber == 22) { //last upload 
    elevfactor = 618;
    elevintercept = 990;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 23) { //last upload 20161221, before deployment to Cyprus
    elevfactor = 742;
    elevintercept = 1004; //higher number -> higher angle
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 24) { //last upload 20170110, before deployment to China
    elevfactor = 705;
    elevintercept = 1024;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 25) { //last upload
    elevfactor = 654;
    elevintercept = 1082;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 26) { //last upload 20180124, last elevation set 20180124
    elevfactor = 635;
    elevintercept = 772;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 31) { //last upload 20171111, last elevation set 20171111
    elevfactor = 443; 
    elevintercept = 1072;  
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 32) { //last upload 20171111, last elevation set 20171111
    elevfactor = 407;
    elevintercept = 1030;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 33) { //last upload 20180515, last elevation analysis 20180515
    elevfactor = 454;
    elevintercept = 1020; 
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 34) { //last upload 20171715, last elevation analysis 20171215
    elevfactor = 413;
    elevintercept = 1066;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }
  if (unitnumber == 35) { //last upload 20180515, last elevation analysis 20180515
    elevfactor = 427;
    elevintercept = 1090;
    homeangleoffset = 2.35619449;
    ElevSlow = 1;
  }


  //analogReference(INTERNAL);  1.1 V
  pinMode(Convertpin, OUTPUT);
  pinMode(POPModepin, INPUT);
  digitalWrite(Convertpin, LOW);
  analogReference(DEFAULT);  // 5V ratio
  Serial.begin(9600);
  pinMode(GPSselect, OUTPUT);
  delay(1);
  digitalWrite(GPSselect, LOW);
  delay(1);
  Serial.println("$PMTK251,57600*2C");  // set GPS to 57600 baud
  Serial.flush();
  delay(100);
  Serial.begin(57600);
  delay(10);
  Serial.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");  // RMC data
  Serial.flush();
  Serial.println("$PMTK220,200*2C");  // 5 times a second to reduce delays in reading
  Serial.flush();
  Serial.setTimeout(3000);
  pinMode(Clockpin, OUTPUT);
  pinMode(Datapin, INPUT);
  digitalWrite(Clockpin, LOW);
  pinMode(A7, OUTPUT);
  digitalWrite(A7, LOW);
  pinMode(A6, OUTPUT);
  digitalWrite(A6, LOW);
  pinMode(A5, INPUT);

  pinMode(DataValidpin, INPUT);
  pinMode(stepnotenablepin, OUTPUT);
  digitalWrite(stepnotenablepin, HIGH);
  pinMode(stepdirpin, OUTPUT);
  digitalWrite(stepdirpin, LOW);
  pinMode(VNselect, OUTPUT);
  digitalWrite(VNselect, HIGH);  // active low on 74xx125
  digitalWrite(GPSselect, HIGH);

  //
  // stepper motor output from timer 2
  //
  pinMode(3, OUTPUT);
  TCCR2B = TCCR2B & 0b11111000 | 0x04;  // 3 for regular stepping, 04 supposed half-step
  // timer 2 is 16 Mhz divided by 510 * prescalar
  // 06 = 256 = 122.5 Hz
  // 04 = 64 - 490 Hz  (for half stepping, doesn t quite make sense
  // 03 = 32 - 980 Hz
  // 02 = 8  (no 16!)
  // playground.arduino.cc/Main/TimerPWMCheatsheet
  // be sure to change msperradian constant above if rotation rate is changed
  // below sets to more arbitrary pulse length at expense of 50% duty (Step controller doesn't care)
  //OCR2A = 170;        //  at x03 255 gives 980 Hz
  // 180 gives 813 Hz, doesnt work below 127, narrow pulses below about 160
  //bitSet(TCCR2A, 0);  // WGM20
  //bitSClear(TCCR2A, 1);    // WGM21
  //bitSey(TCCR2B, 3);  // WGM22
  // endarbitrary
  // below sets to factor of 2 faster than regular steppin
  //bitSet(TCCR2A, 0);  // WGM20  // set
  //bitSet(TCCR2A, 1);    // WGM21    // clear
  //bitClear(TCCR2B, 3);  // WGM22  // set
  // end factor 2
  analogWrite(3, 127);  // seems necessary to initialize output. Figure out why later
  //
  // elevation motor output from timer 1
  //
  pinMode(9, OUTPUT); // elevationPWMpin is pin 9
  TCCR1B = TCCR1B & 0b11111000 | 0x02;
  // timer 1 is 16 Mhz divided by 510 * prescalar
  // 05 = 8 = 1.0 MHz
  // playground.arduino.cc/Main/TimerPWMCheatsheet
  // set mode 14 = fast PWM length set by ICR1,
  TCCR1A =  TCCR1A & 0b11111100 | 0b00000010;
  TCCR1B =  TCCR1B & 0b11100111 | 0b00011000;
  ICR1 = 40000;          // 0x8FFF;   Frequency is 1E6 divided by this value
  analogWrite(9, 120);   // seems necessary to initialize PWM output figure out later
  OCR1A = 3000;          // initialize with 1500 us.
  //
  // Barometer setup
  //
  pinMode(baromclock, OUTPUT);
  digitalWrite(baromclock, LOW);
  pinMode(baromselect, OUTPUT);
  digitalWrite(baromselect, HIGH);
  pinMode(baromdatain, OUTPUT);   // data in from viewpoint of barometer is output from Arduino
  digitalWrite(baromdatain, LOW);
  pinMode(baromdataout, INPUT);
  delayMicroseconds(50);
  c1 = readbarometer(0);  // reset, doesn't matter which variable
  delay(2);
  //
  // read barometer PROM
  //
  c1 = readbarometer(5);   //  seem to need to read this first
  c1 = readbarometer(69);
  Serial.println(long(c1));
  c2 = readbarometer(37);
  Serial.println(long(c2));
  c3 = readbarometer(101);
  Serial.println(long(c3));
  c4 = readbarometer(21);
  Serial.println(long(c4));
  c5 = readbarometer(85);
  Serial.println(long(c5));
  c6 = readbarometer(53);
  Serial.println(long(c6));
  //
  // initialize variables
  //
  digitalWrite(stepnotenablepin, LOW);
  elevangle = 40.0 * dtor;
  digitalWrite(Convertpin, HIGH);
  delay(10);
  gatestart = micros();
  digitalWrite(Convertpin, 0);
}
//
//
// the loop routine runs over and over again forever:
//
//
void loop() {
  //
  // calculations such as GPS and solar elevation not done every time through
  // elevangle calculation takes about 2 ms, readGPS is up to 1 second
  //
  // --------------- short gate ---------------
  //
  // analog reads and stow calculations inside the short loop
  // have to wait for digitizer to have valid data anyway so doesn't cost time
  POPMode = digitalRead(POPModepin);
  if ((POPMode == LastPOPMode) && (unitnumber > 3)) {       // debounce - don't do anything if different
    // don't do anything with POPMode on ground units
    if ((POPMode == 1) || (LastPOPMode == 1)) {
      digitalWrite(stepnotenablepin, LOW);
      StowedFlag = 0;
    }
    else {
      if (abs(azimuth - stowangle) < 0.07) StowedFlag == 1; // 0.07 is about 4 degrees
      if (StowedFlag == 1) digitalWrite(stepnotenablepin, HIGH);
    }
  }
  LastPOPMode = POPMode;
  
  tempvalue = analogRead(temppin);
  
  //azimuth determination
  homevalue = analogRead(homepin);
  readingmillis = millis();
  
  //symmetric edge detection of magnet at HE sensor on each rotation
  if ((homevalue > homethreshold) && (homesenseflag == 0)) {
    inboundreadingmillis = readingmillis;
    homesenseflag = 1;
  }
  if ((homevalue < homethreshold) && (homesenseflag == 1)) {
    lasthomemillis = (readingmillis + inboundreadingmillis)/2;
    homesenseflag = 0;
  }
  azimuth = (readingmillis - lasthomemillis) / msperradian + homeangleoffset;
  azimuth = fmod(azimuth + twopi, twopi);
  
  
  readPhotodiode(PhotoA, PhotoB, PhotoC, PhotoD); // reading from last (long) gate
  processmicrossh = micros() - gatestart;  // uncomment here for lower bound on gate length
  currentmicros = micros();
  while ((currentmicros - gatestart) < gatelengthsh) {
    currentmicros = micros();
  }
  //
  // --------------- long gate ---------------
  //
  gatestart = micros();
  digitalWrite(Convertpin, 1);
  // tilt - alternate reading vectornav and doing calculations
  //   - no need to go faster than response time of sensors
  //   - writing to pulse width register more often than the pulse frequency is a little weird
  //
  if ((readcounter % 4) == 0) {
    if ((subreadcounter % 2) == 0) {
      baromteraw = readbarometer(42);  // 1024 converrsion
      tediff = baromteraw - 256L * c5;      // factor 2^7 above
      tediffproduct = (tediff * c6) / 8388608;
      tediffb = tediff;
      baromte = 2000L + tediffproduct;
      tediff64 = int64_t(tediff);
    }
    else {
      baromraw = readbarometer(34);  // 2 -> 256, 66 -> 512, 34 -> 1024
      proffset  = (int64_t(c2) << 17) + ((int64_t(c4) * tediff64) >> 6);
      prsens = (int64_t(c1) << 16) + ((int64_t(c3) * tediffb) >> 7);
      pressure = ((((baromraw * prsens) >> 21) - proffset) >> 15);
    }
  }
  if ((readcounter % 4) == 1) {  // to reduce dead time computing read VN, apply tilt correction next read
    if (unitnumber > 1) {  // temporary for bad VectorNav on unit 1.  Just keep flat!
      gotVN = readVN100(yaw, pitch, roll);
    }
    else {
      yaw = 0.0;
      pitch = 0.0;
      roll = 0.0;
    }
  }
  if ((readcounter % (4 * ElevSlow)) == 2) {
    //    ----- calculate corrected elevation angle -------
    // ElevSlower > 1 makes it correct every n cycles; better for ground
    corrsintiltx = -sin(roll * dtor); // or (plus) rrunningsintiltx, y
    corrsintilty = sin(pitch * dtor);
    sintilt = sqrt(corrsintiltx * corrsintiltx + corrsintilty * corrsintilty);
    tilt = asin(sintilt);
    phi = atan2(corrsintilty, corrsintiltx);
    relangle = phi - azimuth;
    //elevangle = 45.*dtor;  // for testing tilt correction
    CorrElevApprox = elevangle - tilt * sin(relangle);
    NetApprox = asin(sin(CorrElevApprox) * cos(tilt) + cos(CorrElevApprox) * sintilt * sin(relangle));
    CorrElev = CorrElevApprox - (NetApprox - elevangle);
    //digitalWrite(stepnotenablepin, HIGH);//for DEBUGGING elevation settings
    //CorrElev = 65.*dtor;
    //CorrElev = 0.*dtor;
    ElevMicrosec = elevintercept + elevfactor * constrain(CorrElev, -0.02, 1.39); //  -1 to 80 degrees.
    OCR1A = int(2.0 * ElevMicrosec);        //  ; sets directly in 2 us
  }
  //
  if ((readcounter % 4) == 3) {  // GPS or solar elevation
    if ((subreadcounter % n4readsPerGPS) == 0) {
      if (haveGPSflag == 1) {
        gotGPS = readGPS(lat, lon, year, month, day, UDThr, millisAtGPS);
        // routine will not update lat, lon, ... without valid GPS reading
        if (gotGPS == 1) hadGPSfix = 1;
      }
    }
    else {
      if ((subreadcounter % n4readsPerElev) == 0) {
        hour = fmod(UDThr + float((millis() - millisAtGPS)) / 3600000., 24.0);
        elevangle =  elevationangle(lat, lon, year, month, day, hour);
      }
    }
    subreadcounter += 1;
  }
   // if ((readcounter % 10) == 0) {  // for real data write every value, not every 80th...
  Serial.print((PhotoAsh - photooffset) / 16);
  Serial.print(",");
  Serial.print((PhotoBsh - photooffset) / 16);
  Serial.print(",");
  Serial.print((PhotoCsh - photooffset) / 16);
  Serial.print(",");
  Serial.print((PhotoDsh - photooffset) / 16);
  Serial.print(",");
  Serial.print(withintimeflag * (PhotoA - photooffset) / 16);
  Serial.print(",");
  Serial.print(withintimeflag * (PhotoB - photooffset) / 16);
  Serial.print(",");
  Serial.print(withintimeflag * (PhotoC - photooffset) / 16);
  Serial.print(",");
  Serial.print(withintimeflag * (PhotoD - photooffset) / 16);
  Serial.print(",");
  Serial.print(readingmillis / 10);
  Serial.print(",");
  //Serial.print(processmicrossh/10);
  switch ((readcounter % 50)) {
    case 10:
      Serial.print("1");
      Serial.print(",");
      Serial.print(lat, 1);
      Serial.print(",");
      Serial.print(lon, 1);
      Serial.print(",");
      //Serial.println(CorrElevApprox/dtor, 1);
      Serial.println(tempvalue);
      break;
    case 20:
      Serial.print("2");
      Serial.print(",");
      Serial.print(UDThr, 5);
      Serial.print(",");
      Serial.print(100 * month + day);
      Serial.print(",");
      Serial.println(millisAtGPS / 10);
      break;
    case 30:
      Serial.print("3");
      Serial.print(",");
      Serial.print(yaw, 1);
      Serial.print(",");
      Serial.print(pitch, 1);
      Serial.print(",");
      Serial.println(roll, 1);
      break;
    case 40:
      Serial.print("4");
      Serial.print(",");
      Serial.print(long(pressure));
      Serial.print(",");
      Serial.print(baromte);
      Serial.print(",");
      Serial.println(POPMode + 10 * StowedFlag, 1);
      break;
    case 0:
      Serial.print("5");
      Serial.print(",");
      Serial.print(gatelength);
      Serial.print(",");
      Serial.print(gatelengthsh);
      Serial.print(",");
      Serial.println(photooffset);
      break;
    default:
      Serial.print("0");
      Serial.print(",");
      Serial.print(azimuth / dtor, 1);
      Serial.print(",");
      Serial.print(homevalue);
      Serial.print(",");
      Serial.println((micros() - gatestart) / 100);
  } // switch 
 //  } // don't write every one (for debugging)
  readPhotodiode(PhotoAsh, PhotoBsh, PhotoCsh, PhotoDsh);
  processmicros = micros() - gatestart;  // uncomment here for lower bound on gate length
  currentmicros = micros();
  while ((currentmicros - gatestart) < gatelength) {
    currentmicros = micros();
  }
  actualmicros = currentmicros - gatestart;  // uncomment here for actual gate length including wait
  gatestart = micros();
  digitalWrite(Convertpin, 0);
  if ((float)actualmicros > 1.02 * gatelength) // will zero out long gate data if it ran over (mostly from reading GPS)
  {
    withintimeflag = 0;
  }
  else
  {
    withintimeflag = 1;
  }
  readcounter += 1;
} // end of loop routine


//
//
// ---------------  Elevation angle of sun -----------------------------------------
//
//
float elevationangle(float lat, float lon, int year, int month, int day, float UDThr) {
  // year is integer from 2000
  // may fail before 2013 because of modulo and negative times
  int cummonthdays[12] =     {
    -1, 30, 58, 89, 119, 150, 180, 211, 242, 272, 303, 333
  }; // starts at -1 because Jan 1 is 1, not 0
  int cummonthdaysleap[12] = {
    -1, 30, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
  };

  const float pi = 3.14159265;
  const float twopi = 6.2831853;
  const float dtor = 0.017453293;
  const float hrtor = 0.261799388; //hours to radians
  int dayyear, nleap;
  float deltayr;
  float time, mnlong, mnanom, eclong, num, den;
  float ha, ra, dec, gmst, lmst, el;

  month = constrain(month, 1, 12);
  if ((year % 4) == 0)
  {
    dayyear = day + cummonthdaysleap[month - 1];
  }
  else
  {
    dayyear = day + cummonthdays[month - 1];
  }

  deltayr = float(year - 13);
  nleap = floor(deltayr / 4.0 + 0.01);
  time = deltayr * 365.0 + float(nleap + dayyear) + UDThr / 24.0 - 0.5 ;
  mnlong = 281.29860 + 0.985647 * time;
  mnlong = fmod(mnlong, 360.0);
  mnanom = 358.14382 + 0.9856 * time;
  mnanom = fmod(mnanom, 360.0);
  mnanom = mnanom * dtor;
  eclong = mnlong + 1.915 * sin(mnanom) + 0.0205 * sin(2.0 * mnanom);
  eclong = fmod(eclong, 360.0);
  if (eclong < 0.0)
    eclong += 360.0;
  eclong = eclong * dtor;
  num =  0.91751185 * sin(eclong);
  den = cos(eclong);
  ra = atan(num / den);
  if (den < 0.0)
  {
    ra += pi;
  }
  else if (num < 0.0)
    ra += twopi;
  dec = asin(0.39770844 * sin(eclong));
  gmst = 6.75333 + 0.0657098242 * time + UDThr;
  gmst = fmod(gmst, 24.0);
  lmst = gmst + lon / 15.0;
  lmst += 24.0; //for MOD
  lmst = fmod(lmst, 24.0);
  // Hour angle
  ha = lmst * hrtor - ra;
  if (ha < -pi)
    ha += twopi;
  if (ha > pi)
    ha -= twopi;
  // elevation
  el = asin(sin(dec) * sin(lat * dtor) + cos(dec) * cos(lat * dtor) * cos(ha));
  //el = el/dtor;  // if you want degrees
  /*Serial.print(lat);
  Serial.print(",");
  Serial.print(lon);
  Serial.print(",");
  Serial.print(UDThr);
  Serial.print(",");
  Serial.print(year);
  Serial.print(",");
  Serial.print(month);
  Serial.print(",");
  Serial.print(el);
  Serial.print(",");
  Serial.print(ra);
  Serial.print(",");
  Serial.print(dec);
  Serial.print(",");
  Serial.println("elevation calculation");*/

  return el;
}


//
//
// ---------------  Read GPS -----------------------------------------
//
//
int readGPS(float &lat, float &lon, int &year, int &month, int &day, float &UDThr, unsigned long &millisAtGPS) {

  int serialbytes, readbytes, readuntilbytes;
  char GPSoutput[80];
  int oneByte, onenumber;
  int validGPS = 0;
  float readlat, readlon, readUDThr;
  int readyear, readmonth, readday;
  unsigned long timeoutmillis;
  int istring, commacount;

  digitalWrite(GPSselect, LOW);
  delayMicroseconds(5);

  //empty buffer because it may contain an old fragment of a GPS output string
  //  I think Arduino buffer is not a FIFO, rather fills and then discards

  timeoutmillis = millis() + 2000;
  while (Serial.read() >= 0) {
    ; // do nothing
  }

  readuntilbytes = 0;
  while ((readuntilbytes < 10) && (millis() < timeoutmillis)) {
    oneByte = Serial.read();
    //Serial.print(oneByte);
    if (oneByte == 36) { // $ is 36
      readuntilbytes = Serial.readBytesUntil('*', GPSoutput, 79);
      Serial.println(GPSoutput);
      if ((GPSoutput[0] == 'G') && (GPSoutput[1] == 'P') && (GPSoutput[2] == 'R') && (GPSoutput[3] == 'M') && (GPSoutput[4] == 'C')) {
        if (GPSoutput[17] == 'A') {
          validGPS = 1;
          // crude but effective parsing of GPS RMC string
          onenumber = GPSoutput[6] - 48;
          if ((onenumber >= 0) && (onenumber <= 2)) {
            readUDThr = float(10 * onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[7] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readUDThr += float(onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[8] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readUDThr += float(onenumber) / 6.;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[9] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readUDThr += float(onenumber) / 60.;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[10] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readUDThr += float(onenumber) / 360.;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[11] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readUDThr += float(onenumber) / 3600.;
          }
          else {
            validGPS = 0;
          }

          onenumber = GPSoutput[19] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlat = float(10 * onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[20] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlat += float(onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[21] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlat += float(onenumber) / 6;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[22] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlat += float(onenumber) / 60.;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[24] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlat += float(onenumber) / 600.;
          }
          else {
            validGPS = 0;
          }
          if (GPSoutput[31] == 'S') readlat = -readlat;

          onenumber = GPSoutput[31] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon = float(100 * onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[32] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon += float(10 * onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[33] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon += float(onenumber);
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[34] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon += float(onenumber) / 6;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[35] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon += float(onenumber) / 60.;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[37] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readlon += float(onenumber) / 600.;
          }
          else {
            validGPS = 0;
          }
          if (GPSoutput[42] == 'W') readlon = -readlon;

          commacount = 0;
          istring = 44;
          while (commacount < 2) {
            if (GPSoutput[istring] == ',') {
              commacount += 1;
            }
            istring += 1;
          }
          onenumber = GPSoutput[istring] - 48;
          if ((onenumber >= 0) && (onenumber <= 3)) {
            readday = 10 * onenumber;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[istring + 1] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readday += onenumber;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[istring + 2] - 48;
          if ((onenumber >= 0) && (onenumber <= 1)) {
            readmonth = 10 * onenumber;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[istring + 3] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readmonth += onenumber;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[istring + 4] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readyear = 10 * onenumber;
          }
          else {
            validGPS = 0;
          }
          onenumber = GPSoutput[istring + 5] - 48;
          if ((onenumber >= 0) && (onenumber <= 9)) {
            readyear += onenumber;
          }
          else {
            validGPS = 0;
          }
          /*
          Serial.print("[GPS");
           Serial.print(" ");
           Serial.print(validGPS);
           Serial.print(" ");
           Serial.print(readUDThr);
           Serial.print(" ");
           Serial.print(readlat);
           Serial.print(" ");
           Serial.print(readlon);
           Serial.print(" ");
           Serial.print(readyear);
           Serial.print(" ");
           Serial.print(readmonth);
           Serial.print(" ");
           Serial.println(readday);
           */
        } // equals A
      }
    }
  }
  digitalWrite(GPSselect, HIGH);
  if (validGPS == 1) {
    millisAtGPS = millis(); // could subtract a couple for serial reading but invite rollover bugs
    lat = readlat;
    lon = readlon;
    year = readyear;
    month = readmonth;
    day = readday;
    UDThr = readUDThr;
  }
  return validGPS;
}
//
//
// ---------------  Read VN100 -----------------------------------------
//
//
int readVN100(float &yaw, float &pitch, float &roll) {

  int serialbytes, readbytes, readuntilbytes;
  char VNoutput[46];
  int oneByte, onenumber;
  int validVN = 0;
  float readyaw, readpitch, readroll;
  int istring, commacount;
  unsigned long timeoutmillis;

  //empty buffer
  // don't seem to need this for VN
  /*while (Serial.read() >= 0){
   //; // do nothing
   }*/

  digitalWrite(VNselect, LOW);
  delayMicroseconds(5);
  timeoutmillis = millis() + 800;
  Serial.println("$VNRRG,8*4B");  // request register 8
  readuntilbytes = 0;
  while ((readuntilbytes < 10) && (millis() < timeoutmillis)) {
    oneByte = Serial.read();
    if (oneByte == 36) { // $ is 36
      readuntilbytes = Serial.readBytesUntil('*', VNoutput, 45);
      //Serial.println(VNoutput);
      if ((VNoutput[0] == 'V') && (VNoutput[1] == 'N') && (VNoutput[2] == 'R') && (VNoutput[3] == 'R') && (VNoutput[4] == 'G')) {
        validVN = 1;
        // $VNRRG,8,+006.271,+000.031,-002.000*66 example

        onenumber = VNoutput[10] - 48;
        if ((onenumber >= 0) && (onenumber <= 2)) {
          readyaw = float(100 * onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[11] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readyaw += 10.*float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[12] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readyaw += float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[14] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readyaw += float(onenumber) / 10.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[15] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readyaw += float(onenumber) / 100.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[16] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readyaw += float(onenumber) / 1000.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[9];
        if (onenumber == 45) {
          readyaw = -readyaw;
        }

        onenumber = VNoutput[19] - 48;
        if ((onenumber >= 0) && (onenumber <= 2)) {
          readpitch = float(100 * onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[20] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readpitch += 10.*float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[21] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readpitch += float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[23] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readpitch += float(onenumber) / 10.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[24] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readpitch += float(onenumber) / 100.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[25] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readpitch += float(onenumber) / 1000.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[18];
        if (onenumber == 45) {
          readpitch = -readpitch;
        }

        onenumber = VNoutput[28] - 48;
        if ((onenumber >= 0) && (onenumber <= 2)) {
          readroll = float(100 * onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[29] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readroll += 10.*float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[30] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readroll += float(onenumber);
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[32] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readroll += float(onenumber) / 10.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[33] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readroll += float(onenumber) / 100.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[34] - 48;
        if ((onenumber >= 0) && (onenumber <= 9)) {
          readroll += float(onenumber) / 1000.;
        }
        else {
          validVN = 0;
        }
        onenumber = VNoutput[27];
        if (onenumber == 45) {
          readroll = -readroll;
        }
        //Serial.println(millis() - (timeoutmillis-1000));
      }
    }
  }
  digitalWrite(VNselect, HIGH);
  if (validVN == 1) {
    yaw   = readyaw;
    pitch = readpitch;
    roll  = readroll;
  }
  return validVN;
}
//
//
// ---------------  Read barometer -----------------------------------------
//
//
unsigned long readbarometer(byte command) {
  //
  // read out barometer
  //
  const int baromclock = A1;  // analog used as digital
  const int baromdatain = A0;
  const int baromdataout = A5;
  const int baromselect = A2;

  unsigned long baromraw = 0;
  byte mask = 1;   // for parsing bits in command
  int onebit;
  int numbits = 24;
  int convertdelay = 4580;
  //
  // send command
  //
  digitalWrite(baromselect, LOW);
  delayMicroseconds(10);
  for (mask = 00000001; mask > 0; mask <<= 1) { //iterate through bit mask
    digitalWrite(baromclock, LOW);    // clock low
    if (command & mask) { // if bitwise AND resolves to true
      digitalWrite(baromdatain, HIGH); // send 1
    }
    else { //if bitwise and resolves to false
      digitalWrite(baromdatain, LOW); // send 0
    }
    delayMicroseconds(5);
    digitalWrite(baromclock, HIGH);
  }
  digitalWrite(baromclock, LOW);    // clock low
  if (command & 1) {  // read PROM
    numbits = 16;  // reading PROM is fewer bits
  }
  else {
    //
    // reading ADC means wait, send another command to read
    //
    if (command == 2)  convertdelay =  650;  //  add about 30 us to specified times
    if (command == 66) convertdelay = 1210;
    if (command == 34) convertdelay = 2320;
    if (command == 10) convertdelay =  650;
    if (command == 74) convertdelay = 1210;
    if (command == 42) convertdelay = 2320;
    delayMicroseconds(convertdelay);                 // for OSR=512 conversion time
    digitalWrite(baromselect, HIGH);
    // issue read command
    delayMicroseconds(10);
    digitalWrite(baromselect, LOW);
    digitalWrite(baromdatain, LOW); // read command is all zeros
    delayMicroseconds(5);
    for (int ibit = 0; ibit < 8; ibit++) {
      digitalWrite(baromclock, HIGH);
      delayMicroseconds(1);
      digitalWrite(baromclock, LOW);
      delayMicroseconds(1);
    }
  }
  //
  // get bits
  //
  for (int ibit = 0; ibit < numbits; ibit++) {
    digitalWrite(baromclock, HIGH);
    delayMicroseconds(1);
    onebit = digitalRead(baromdataout);
    baromraw = (baromraw << 1) + onebit;
    digitalWrite(baromclock, LOW);
  }
  digitalWrite(baromselect, HIGH);
  return baromraw;
}
//
//
// ---------------  Read photodiode -----------------------------------------
//
//
void readPhotodiode(unsigned long &photoA, unsigned long &photoB, unsigned long &photoC, unsigned long &photoD) {
  //
  // read out photodiode, right after gate pin goes low
  //
  const int Clockpin = 13;  // compatible with using SPI libary calls
  const int Datapin = 12;   // ditto
  const int DataValidpin = 4;
  //pinmode(Clockpin, OUTPUT); must be in setup
  //pinmode(Datapin, INPUT); must be in setup
  int onebit, validPD, dummyint;
  unsigned long PDtimeout;

  PDtimeout = millis() + 200;
  validPD = 0;
  //dummyint = 0;

  while ((millis() < PDtimeout) && (validPD == 0)) {
    if ((digitalRead(DataValidpin) == 0)) {
      photoD = 0;
      for (int ibit = 0; ibit < 20; ibit++) {
        onebit = bitRead(PINB, Datapin - 8);
        //      onebit = digitalRead(Datapin);
        bitSet(PORTB, Clockpin - 8);  // see page 630 Arduino Cookbook
        photoD = (photoD << 1) + onebit;
        //delayMicroseconds(1);  // may not need this
        //dummyint+=1;       // add a tiny delay
        bitClear(PORTB, Clockpin - 8);
      }
      photoC = 0;
      for (int ibit = 0; ibit < 20; ibit++) {
        onebit = bitRead(PINB, Datapin - 8);
        bitSet(PORTB, Clockpin - 8);  // see page 630 Arduino Cookbook
        photoC = (photoC << 1) + onebit;
        bitClear(PORTB, Clockpin - 8);
      }
      photoB = 0;
      for (int ibit = 0; ibit < 20; ibit++) {
        onebit = bitRead(PINB, Datapin - 8);
        bitSet(PORTB, Clockpin - 8);
        photoB = (photoB << 1) + onebit;
        bitClear(PORTB, Clockpin - 8);
      }
      photoA = 0;
      for (int ibit = 0; ibit < 20; ibit++) {
        onebit = bitRead(PINB, Datapin - 8);
        bitSet(PORTB, Clockpin - 8);
        photoA = (photoA << 1) + onebit;
        bitClear(PORTB, Clockpin - 8);
      }
      validPD = 1;
    }
  }
  delayMicroseconds(9);
}


