// Simple irrigation controller
// 25 June 2022, ESP32 version
// 4 Sept 2022, added programs 4,5

/* Simple display in four digits. 
 *   First digit for "program" ie. p0-p9
 *   2nd digit for "station" s1 to s6 (plus sA-sF for expansion) and 
 *   3rd,4th digits for hours (incremented in 30m blocks so 1.5 = 90min). 
 *   Program p9 is a loop for backwash and is only active when other programs are running. 
 *   Program p0 is for run mode. The program button stops all programs and goes to prog mode.
 *      When in p0, the station is actually the run prog, so 01xx is run only p1, but 00xx is run all.
 *      Generally we'll run one program at a time.
 *      Pump runs whenever in run mode.
 *
 * There is no time of day start. Only a manual start.
 * Initial design has only 8 relays, so allowing one for pump and two for backwash, leaves 5 useful stations. 
 * Stations 3-7 would be irrigation blocks, 
 *   1 and 2 for backwash filters and 
 *   0 for pump. 
 *   System could be expanded by using a 4-relay board and another 74hc595 register (or just 4 pins).
 *   
 *   Output pins are connected to opto-coupled relay board with 24vac supply to relays for actuators.
*/

/* Buttons
 *  1 - selects run mode = 0 or program mode 1,2,3,4,5,9
 *  2 - in run mode select 0 to run-all or 1,2,3,4,5 to run just one program, in program mode press to increment station/block
 *  3 - in run mode press to start, in program mode increment duration
 *  4 - in run mode press to stop , in program mode zero duration
 *  8 - fast mode button which speeds up clock x12 and displays "F" in the 5th digit
 */

/* Display
 *  In run mode 
 *    first digit 0, 2nd is 0 for all and 1,2,3,4,5 for only one program
 *    digits 3,4 show time elapsed since start of program in hours. So 1.3 is 1 hour and 18 minutes
 *    2nd bank of digits shows remaining time for each active station
 *  In program mode
 *    first digit shows program number 1,2,3,4,5,9 - where 9 is for backwash
 *    2nd digit shows station and duration in hours (except for backwash stations s1,s2 in minutes)
 */

#include <EEPROM.h> // size 512
#define CK1 96   // eeprom setup cksum low address
#define CK2 97   // eeprom setup cksum high address
#define LAST_PROG 102  // last run program
#define STATUS 103  // location of status eeprom register
#define LAST_TIME_INDEX 104 // refers to 105 to 121 in eeprom for wear leveling 
#define LAST_TIME_BASE 105
#define LAST_TIME_SIZE 16
                       // if in run mode this will be last run time in 6min chunks
                       // eg. if timer has been running a program for 1.4 hours it would have a value of 14
                       // values > 240 (ie. 24hrs) are invalid
                       
// eeprom data - 10 programs and 10 stations = 100 uint8_ts // a bit excessive
#define PROGS 10
#define MAX_PROG 5   // available progs are 1,2,3,4,5
#define STATIONS 16  // stations 0x0 to 0xF, but 0 is for pump and 0xD is for loop delay
#define EBASE 180    // BASE address for programs 

// backwash stations 1,2 and delay station 0xd
uint8_t bws[] = { 1,2,0xd };

// normal irrigation block stations
// expand here if adding a 4-port relay board
uint8_t bls[] = { 3,4,5,6,7 }; 

// TM1638 display/keyboard pins
const uint8_t strobe_pin =  4;
const uint8_t clock_pin  = 16;
const uint8_t data_pin   = 17;
/*
const uint8_t BUTTON_1_PIN = 14;  // touch 6
const uint8_t BUTTON_2_PIN = 27;  // touch 7
const uint8_t BUTTON_3_PIN = 13;  // touch 4
const uint8_t BUTTON_4_PIN = 12;  // touch 5
*/
// changes for the devkit-c versus devkit-v1 controller board gnd pin change 
const uint8_t BUTTON_1_PIN = 27;  // touch 7
const uint8_t BUTTON_2_PIN = 26;  // dac 2
const uint8_t BUTTON_3_PIN = 12;  // touch 5
const uint8_t BUTTON_4_PIN = 14;  // touch 6
const uint8_t BUTTON_GND = 13;  // touch 4 - fake ground pin - set to zero

#define CLOCK_NORMAL 0
#define CLOCK_INVERT 1
#define CLOCK_TYPE CLOCK_INVERT
#define CLOCK_DELAY_US 1
uint8_t buttons;  // buttons from 8-digit/key board

// 74HC595 pins for shift register to 8 relay board
#define SER_Pin 25
#define RCLK_Pin 33
#define SRCLK_Pin 32
uint8_t relays;  // 8-bit register of relay states

/* Segment uint8_t maps for numbers 0 to 9 and A to F */
                     /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/ /*A*/ /*B*/ /*C*/ /*D*/ /*E*/ /*F*/
uint8_t digits[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0 };
const uint8_t POINT_SEG = 0x80; 
const uint8_t BLANK_SEG = 16; 


long int j = 0, k = 0; // temp variables for display
boolean run_mode=false, dbg=false, fast_mode=false;
char buffer[128];  // output string buffer for debug
uint8_t b1,b2,b3,b4,b8;     // button states
uint8_t b1new,b2new,b3new,b4new,b8new;     // button states
uint8_t prog=0, station=0, duration=0;

unsigned long start_time; // in milli-seconds since boot
uint8_t last_time;    // time since program start (in 6 min blocks)
uint8_t backwash_delay; // delay backwash start after poweron 
uint8_t debounce; // key debounce delay hack
unsigned int ct,pt,lt,bt; // run time in minutes for run mode
unsigned int remaining[STATIONS]; // remaining run time in minutes
uint8_t sr; // station time remaining display counter
uint8_t p,s; // prog and station for run mode
uint8_t st[STATIONS], station_active;  // station states for run mode

void setup ()
{
  /* Set DIO pins to outputs */
  pinMode(strobe_pin, OUTPUT);
  pinMode(clock_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);

  // setup TM1638 led display
  sendCommand(0x8f);  // activate
  sendCommand(0x40);  // set auto increment mode
  digitalWrite(strobe_pin, LOW);
  shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, 0xc0);   // set starting address to 0
  for(uint8_t i = 0; i < 16; i++) // write 16 bytes of zero to clear display - each digit is 2 bytes - 10 bits used
    shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, 0x00);
  digitalWrite(strobe_pin, HIGH);

  // setup for external 4-key pad
  pinMode(BUTTON_GND, OUTPUT);
  digitalWrite(BUTTON_GND, LOW); // create a fake ground pin for easy button connection on dev-c and dev-v1 boards
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  b1=digitalRead(BUTTON_1_PIN);
  b2=digitalRead(BUTTON_2_PIN);
  b3=digitalRead(BUTTON_3_PIN);
  b4=digitalRead(BUTTON_4_PIN);
  
  b8=HIGH; // start with fast_mode button HIGH
  
  Serial.begin(115200); // for debug output

  EEPROM.begin(512); // create an eeprom object in esp32

  // check if EEPROM setup
if ( ((EEPROM.read(CK2)*256+EEPROM.read(CK1)) & 0x7fff) != checksum() ) {
  // init the program memory
  Serial.println("Init EEPROM...");
  for(p=0; p<PROGS; p++) for (s=0;s<STATIONS;s++) update_eeprom(p, s, 0);
  update_checksum();
  EEPROM.write(LAST_TIME_INDEX,0); // set the initial last time index to 0
  EEPROM.write(LAST_TIME_BASE,0); // set the initial last time value to 0
  EEPROM.write(STATUS,false); // disable run mode
  EEPROM.commit();
  }

// get last run mode and if needed get last_time
run_mode=EEPROM.read(STATUS);

last_time=0; // time since last start in 6 min blocks
if (run_mode) {
  last_time=EEPROM.read(LAST_TIME_BASE + EEPROM.read(LAST_TIME_INDEX) ); 
  start_time=millis();  // set start time base
  sr=1;  // time remaining display starts from station 1
  station = EEPROM.read(LAST_PROG);  // restore the last running program 0=all
  }
if (last_time>240) { last_time=0; run_mode=false; }  // ignore last_time > 24 hours

// note that prog will not resume until a minute after restart because initially ct==lt
lt=(unsigned int) last_time * 6;  // last loop time in minutes (set lt=0 to restart immediately)
bt=lt+1;                           // current backwash start time in minutes

backwash_delay=1; // delay backwash start after poweron

debounce=0; // external keypad debounce delay hack

  // 74hc595 output pins for relays
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  relays_off();

sprintf(buffer,"*** INIT run_mode,last_time(x6m),: %d %d\n",run_mode,last_time);
Serial.print(buffer);

}

/* Main program */
void loop()
{
/* Update the display with the current counter value */
  WriteNumberToSegment(0, prog, 0);
  delay(2);
  WriteNumberToSegment(1, station, 0);
  delay(2);
  if (k++ < 100) { // blink last two digits
    j= duration; // in 6 min units
    if (run_mode) j = ct / 6;  // ct is in min, so ct/6 means 18min shows as 0.3 on display
    WriteNumberToSegment(2 , (j/10 % 10), POINT_SEG );
    delay(2);
    WriteNumberToSegment(3 , j % 10, 0 );
    } else
    {
    WriteNumberToSegment(2 , BLANK_SEG, 0 );
    delay(2);
    WriteNumberToSegment(3 , BLANK_SEG, 0 );      
    }
  delay(2);
  if ( (k > 200) || (run_mode==false) ) k=0; // only blink in run_mode=true
  if ( (prog==0) && (run_mode==false) ) k=100; // blank in prog=0 and not running

  if (fast_mode)
      WriteNumberToSegment(4,0xF,0);  // write F to digit 4 to show fast_mode
    else
      WriteNumberToSegment(4,BLANK_SEG,0);     
  
  // blank last 3 digits if not in run mode
  if (run_mode==0) {
    WriteNumberToSegment(5 , BLANK_SEG, 0 );     
    WriteNumberToSegment(6 , BLANK_SEG, 0 );     
    WriteNumberToSegment(7 , BLANK_SEG, 0 );   
  } else {
    // display time remaining (in minutes so divide by 6 to get 6min scale so 90min goes to 1.5hrs)
    while ( (remaining[sr] == 0) && (sr < (STATIONS-1)) ) sr++;
    if ( remaining[sr] != 0 ) {
      // if backwash stations then display minutes instead of 6min units
      if ( (sr == bws[0]) || (sr == bws[1] ) ) j = remaining[sr]*10; else j = (remaining[sr]+3)/6;
      WriteNumberToSegment(5, sr, 0);
      WriteNumberToSegment(6, (int) (j/10) % 10, POINT_SEG);
      WriteNumberToSegment(7, (int) j % 10, 0);
    } else {
      sr=1;  // restart check for any running stations on next cycle
    }
    
    if (k==199) sr++; // check next station after a delay
    if (sr > (STATIONS-1)) sr=1;  // restart check on next cycle
  }

// Run Mode
if (run_mode) {
  station_active=false;
  
  if (fast_mode) {
    // for debug testing use seconds rather than minutes
    ct=((millis()-start_time)/5000) + last_time*6;  // number of 5s intervals since start - for debugging
  } else {
    ct=((millis()-start_time)/60000) + last_time*6;  // number of 60s intervals since start
  }
  /* in fast_mode last_time is measured in 30 second units and in normal mode it is in 6 minute units 
   * the display shows hours.minutes in normal time ie. 6 minutes shown as 0.1 and hence 1.0 means 1 hour 
   * and in fast mode the display time shows 30s as 0.1 and hence 1.0 means 5 minutes in fast mode
  */
  
  if (ct > 24*60) {  // stop if run time exceeds 24 hours
      stop_runmode();
      Serial.println("Run exceeded 24 hours. Stopping.");
      lt=ct;  // to skip rest of run mode section
    } 
 
  if (lt != ct) { // wait for ct to change before changing stations
    lt=ct;
    EEPROM.write(LAST_TIME_BASE + EEPROM.read(LAST_TIME_INDEX),(lt/6)); // save last time in case of power failure restart
    EEPROM.commit();
    for(s=0;s<STATIONS;s++) { st[s]=0; remaining[s]=0; } // all stations off
    
    for(p=1;p < (PROGS-1); p++) { // check each program
      s=3; pt=0;
      if ( (station == 0) || (station == p) ) {      // run all or run just one program
        while ( (pt <= ct) && (s < (STATIONS-3)) ) { // check stations s3 to 0xC excluding 0xD for delay and s0,s1,s2 which are pump and backwash
          pt += (read_eeprom(p, s) * 6); // add minutes
          s++;
        }  
      sprintf(buffer,"pt,ct,s-1: %d,%d,%d,%d\n",pt,ct,s-1,p);
      Serial.print(buffer);
        if ( pt > ct ) {
          st[s-1]=1;  // station on
          remaining[s-1]=max(pt-ct,remaining[s-1]);  // max remaining time on station s
          station_active=true; // flag at least on station active
        }
      }
    }
    
    // backwash program 9 (stop when no other stations on)
    if (station_active) {
      if (backwash_delay) {  // dont start in first minute
        bt=ct+1; // +1 so next cycle starts at effectively zero
        backwash_delay--; // decrement delay to zero
      } else {
      p=PROGS-1; // backwash program p9
      pt=bt; s=0; // filter one is s1 and filter two is s2
      while ( (pt <= ct) && (s < 3) ) {
        if (s<2) pt += (read_eeprom(p, bws[s]) / 10); // add minutes for s1,s2
          else pt += (read_eeprom(p, bws[s]) * 6); // add minutes for s0xd
        s++;   // stations s1,s2 are in 0.1 min units and station 0xd is in 6 min units
        }
      sprintf(buffer,"backwash bt,pt,ct,s-1,bws[s-1]: %d,%d,%d,%d,%d\n",bt,pt,ct,s-1,bws[s-1]);
      Serial.print(buffer);
      if ( pt > ct ) {
        st[bws[s-1]]=1;          // station on flag
        remaining[bws[s-1]]=pt-ct;  // remaining time on station s
      }
        else if (s == 3) bt=ct+1; // else reset bt to restart backwash cycle
      }
    
// for all stations set pin mode LOW if active (except last one which is a delay)
    Serial.print("Active stations: ");
    relays=1;  // all off except for pump
    setLed(0,1);
    Serial.print(1);
    for(s=1; s<8; s++) { // s1 to s7
      relays = (relays << 1) | (st[s] & 1);
      setLed(s,st[s]);
      Serial.print(st[s]);
      }
    Serial.println(st[0xd]);  // print backwash delay station

    // write to 74hc595 register for relays
    digitalWrite(RCLK_Pin, LOW);
    shiftOut(SER_Pin, SRCLK_Pin, LSBFIRST, relays ^ 0xff); // set 8 relay pins
    digitalWrite(RCLK_Pin, HIGH);
    } else { // if no main stations active so disable run_mode
      stop_runmode();
      Serial.println("Run complete.");
    }
  }
}

if (debounce) debounce--; else {

buttons = readButtons();  // b1=1; b2=2; b3=4; 

if ( (buttons & 1) || (digitalRead(BUTTON_1_PIN)==0) ) b1new=LOW; else b1new=HIGH;
if ( (buttons & 2) || (digitalRead(BUTTON_2_PIN)==0) ) b2new=LOW; else b2new=HIGH;
if ( (buttons & 4) || (digitalRead(BUTTON_3_PIN)==0) ) b3new=LOW; else b3new=HIGH;
if ( (buttons & 8) || (digitalRead(BUTTON_4_PIN)==0) ) b4new=LOW; else b4new=HIGH;
if ( (buttons & 128) ) b8new=LOW; else b8new=HIGH;
/*
if ( (buttons & 1)  ) b1new=LOW; else b1new=HIGH;
if ( (buttons & 2)  ) b2new=LOW; else b2new=HIGH;
if ( (buttons & 4)  ) b3new=LOW; else b3new=HIGH;
if ( (buttons & 8)  ) b4new=LOW; else b4new=HIGH;
*/

// fast mode speeds up clock
if ( (b8 == HIGH) && (b8new==LOW) ) {  // S8 low active
  Serial.println("button 8 LOW");
  debounce=32;
  if (fast_mode) fast_mode=false; else fast_mode=true; // toggle fast_mode
  if (run_mode) {  // rebase ct time as we're switching to/from 12s to 60s intervals
    last_time = ct / 6;
    start_time=millis();  // set start time base
  }
  dbg=true;
}
b8 = b8new;

if ( (b1 == HIGH) && (b1new==LOW) ) {  // S1 low active
  Serial.println("button 1 LOW");
  debounce=32;
  if (prog==0) 
    stop_runmode(); // exit run mode when going from prog=0 to prog=1
  else
    update_eeprom(prog, station, duration);  // record current duration
  station = 3;  // switch to station 3 when changing prog
  if (prog != 0) update_checksum();
  if (++prog>(PROGS-1)) prog=0;
  if (prog > MAX_PROG) { prog=PROGS-1; station=1; } // skip to prog 9 for backwash
  if (prog==0) 
    { station=0; duration=0; } 
  else
    duration = read_eeprom(prog, station);   // load next duration
  dbg=true;
}
b1 = b1new;

// in run mode (p0) then inc station which shows which prog to run or p0,s0 to run all
// in prog mode inc station number and save current and load next duration
if ( (b2 == HIGH) && (b2new==LOW) ) {  // S2 low active
  Serial.println("button 2 LOW");
  debounce=32;
    if (prog!=0) {
      update_eeprom(prog, station, duration);  // record current duration
      if (++station >= (STATIONS-3)) station=1;  // station in range 1 to 12
      if ( (prog==(PROGS-1)) && (station >= 3) ) station=0xd; // backwash delay
      if (prog!=(PROGS-1)) {
        if (station < 3) station=3; 
        if (station > 7) station=3; 
      }
      duration = read_eeprom(prog, station);   // load next duration
    } else {
      // in run mode station shows which program to run so 01xx means run p1
      if (run_mode==false) // if not running in p0 then can select which prog to run 
        if (++station>MAX_PROG) station=0; // no manual backwash alone
    }
  dbg=true;
}
b2 = b2new;

if ( (b3 == HIGH) && (b3new==LOW) ) {  // S3 low active
  Serial.println("button 3 LOW");
  debounce=32;
  if ( (prog==0) && (run_mode==false) ) start_runmode();
  else if (prog!=0) {
    duration=((duration / 5) + 1) * 5; // 5*6min = 30min increment
    if ((prog==(PROGS-1)) && (station<3)) duration+=5; // inc bachwash in half minutes
    if (duration>80) duration=0; // no more than 8 hours
  }
  
  dbg=true;
}
b3 = b3new;

if ( (b4 == HIGH) && (b4new==LOW) ) {  // S4 low active
  Serial.println("button 4 LOW");
  debounce=32;
  if ( (prog!=0) && (run_mode==false) ) duration=0;
  if ( run_mode==true ) stop_runmode();
}
b4 = b4new;

} // if debounce

// dubug
if (dbg) {
  sprintf(buffer,"Checksums %x %x\n",((EEPROM.read(CK2)*256 + EEPROM.read(CK1)) & 0x7fff),checksum());
  Serial.print(buffer);
  sprintf(buffer,"Display current p s d run: %d %d %d %d\n",prog,station,duration,run_mode);
  Serial.print(buffer);
  sprintf(buffer,"Display read eeprom: %d %d %d\n",prog,station,read_eeprom(prog,station));
  Serial.print(buffer);
  dbg=false;
  }
}

void start_runmode()
{
  uint8_t i;
  Serial.println("start runmode .....................");
    run_mode=true; // toggle run mode
    EEPROM.write(STATUS,run_mode);
    EEPROM.write(LAST_PROG,station); // station has the program number to run 0=all
    
    i = (EEPROM.read(LAST_TIME_INDEX) + 1) % LAST_TIME_SIZE; // inc index for wear leveling
    EEPROM.write(LAST_TIME_INDEX,i);
    
    EEPROM.write(LAST_TIME_BASE + EEPROM.read(LAST_TIME_INDEX),0);
    EEPROM.commit();

    sprintf(buffer,"last_time base,index: %d %d\n",LAST_TIME_BASE,EEPROM.read(LAST_TIME_INDEX));
    Serial.print(buffer);
    
    start_time=millis();  // set start time base
    ct=0; lt=9999; bt=0; // last current minute setup
    last_time=0;
    sr=1;  // time remaining display starts from station 1
    backwash_delay=1;

}

void stop_runmode()
{
    run_mode=false;
    EEPROM.write(STATUS,run_mode);
    EEPROM.commit();
    relays_off();
}

void relays_off()
{
  // set relays off
  relays=0;
  digitalWrite(RCLK_Pin, LOW);
  shiftOut(SER_Pin, SRCLK_Pin, LSBFIRST, relays ^ 0xff); // reset 8 relay pins to OFF
  digitalWrite(RCLK_Pin, HIGH);
  
  // set leds off
  for(s=0;s<8;s++) setLed(s,0);
}

// Calc checksum for programs
uint16_t checksum() // Fletcher16 
{
   uint16_t sum1 = 456; // start at non-zero
   uint16_t sum2 = 0;
   uint8_t a,b;

   for(a=0; a<PROGS; a++) for (b=0;b<STATIONS;b++) {
      sum1 = (sum1 + read_eeprom(a,b)) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   
   return ( (sum2 << 8) | sum1 ) & 0x7fff;
}

void update_checksum()
{
  uint16_t c;
  c = checksum();
  EEPROM.write(CK1, c & 0xff);
  EEPROM.write(CK2, c >> 8);
  EEPROM.commit();
}

// memory is address as prod=1 to 9 and station=1 to 9
uint8_t read_eeprom(uint8_t p, uint8_t s)
{
  return EEPROM.read(EBASE + p*STATIONS + s);
}

void update_eeprom(uint8_t p, uint8_t s, uint8_t d)
{
  EEPROM.write(EBASE + p*STATIONS + s, d);
}

/* Write a decimal number between 0 and 9 to one of the 4 digits of the display */
void WriteNumberToSegment(uint8_t Segment, uint8_t Value, uint8_t dp)
{
    sendCommand(0x44);
    digitalWrite(strobe_pin, LOW);
    shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, 0xC0 + (Segment << 1));
    shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, digits[Value] | dp );
    digitalWrite(strobe_pin, HIGH);

}

void sendCommand(uint8_t value)
{
  digitalWrite(strobe_pin, LOW);
  shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, value);
  digitalWrite(strobe_pin, HIGH);
}

void shiftOutMod(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t clock_type, uint16_t clock_delay_us, uint8_t val)
{
  uint8_t i;

  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST)
      digitalWrite(dataPin, !!(val & (1 << i)));
    else  
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));
      
    digitalWrite(clockPin, (clock_type ? LOW : HIGH));
    delayMicroseconds(clock_delay_us);
    digitalWrite(clockPin, (clock_type ? HIGH : LOW));    
  }
}

uint8_t shiftInMod(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t clock_type, uint16_t clock_delay_us) {
  uint8_t value = 0;
  uint8_t i;
  
  for (i = 0; i < 8; ++i) {
    digitalWrite(clockPin, (clock_type ? LOW : HIGH));
    delayMicroseconds(clock_delay_us);
    if (bitOrder == LSBFIRST)
      value |= digitalRead(dataPin) << i;
    else
      value |= digitalRead(dataPin) << (7 - i);
    digitalWrite(clockPin, (clock_type ? HIGH : LOW));
  }
  return value;
}

uint8_t readButtons(void)
{
  uint8_t buttons = 0;
  digitalWrite(strobe_pin, LOW);
  shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, 0x42);

  pinMode(data_pin, INPUT);

  for (uint8_t i = 0; i < 4; i++)
  {
    uint8_t v = shiftInMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US) << i;
    buttons |= v;
  }

  pinMode(data_pin, OUTPUT);
  digitalWrite(strobe_pin, HIGH);
  return buttons;
}

void setLed(uint8_t position, uint8_t value)
{
  pinMode(data_pin, OUTPUT);

  sendCommand(0x44);
  digitalWrite(strobe_pin, LOW);
  shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, 0xC1 + (position << 1));
  shiftOutMod(data_pin, clock_pin, LSBFIRST, CLOCK_TYPE, CLOCK_DELAY_US, value);
  digitalWrite(strobe_pin, HIGH);
}
