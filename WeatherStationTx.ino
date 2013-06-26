/* Weather Station - Sensor Transmitter
// This is the second version of the weather station, this version will consist of a 
// sensor transmitter and a base receiver.
// 
// Transmitter must gather sensor data, calculate any data necessary and transmit.
// Receiver must gather data and post to the internet and webserver, etc.
//
// The circuit:
*/
/*
25/06/13 - Reworked for ATMega328 Jeenode V6

Port Connections
LDR 
LDR
DHT22 Data

	ATTiny 84
1  VCC
2  PB0(PCINT8/XTAL1/CLKI)			D0	- 
3  PB1(PCINT9/XTAL2)				D1	- **nSel
4  PB3(PCINT11/RESET/dW)				- (ISP D10)
5  PB2(PCINT10/INT0/OC0A/CKOUT)		D2	- **nIRQ
6  PA7(PCINT7/ICP/OC0B/ADC7)		D3	- DHT22 Data
7  PA6(PCINT6/OC1A/SDA/MOSI/DI/ADC6)D4	- (ISP D11) **SDO
8  PA5(ADC5/DO/MISO/OC1B/PCINT5)	D5	- (ISP D12) **SDI
9  PA4(ADC4/USCK/SCL/T1/PCINT4)		D6	- (ISP D13) **SCK
10 PA3(ADC3/T0/PCINT3)				D7	- RX
11 PA2(ADC2/AIN1/PCINT2)			D8	- TX
12 PA1(ADC1/AIN0/PCINT1)			D9	- Pullup LDR
13 PA0(ADC0/AREF/PCINT0) 			D10	- LDR
14 GND
*/

#define DEBUG 1

//includes
#include <JeeLib.h>
#include <avr/sleep.h>
#include <math.h> 
#include <SoftwareSerial.h>


//Constants
#define SEND_PERIOD 30000			// Minimum transmission period of sensor values
#define LDR_PORT 0   				// Defined if LDR is connected to a port's AIO pin
#define SMOOTH 2   					// LDR smoothing factor used for running averages
#define DHT22_PORT 3 				// DHT22 Port
#define FREQ RF12_868MHZ        	// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
#define NODEID 30           		
#define GROUP 212  

SoftwareSerial mySerial(7, 8); // RX, TX


//Humidity and LDR Sensor Port
DHTxx dht (DHT22_PORT);
Port ldr (LDR_PORT);


//General Variables
struct {
	int light;     // light sensor: 0..255
	int humidity;
	int temperature;
	int dewpoint;
	int cloudbase;
	int vcc;		// 0 - 250,  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250, etc. 20mV Steps
} payload;

volatile bool adcDone;
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
// End of config


// Setup Routine
void setup()
{   
#if DEBUG
	Serial.begin(9600);
	Serial.println("\n[WXTX]");
#endif
	
	rf12_initialize(NODEID, FREQ, GROUP);                    
	rf12_sleep(RF12_SLEEP);
	
	pinMode(9, OUTPUT);
}


// Main Loop
void loop()
{
		//Get Sensor Readings
		//Ambient Light Readings
		doLDRMeasure();
		
		//Humdity Sensor Readings
		int h, t;
		!dht.reading(t,h) ? t = h = 0 : t = t;
		
		//Vcc
		payload.vcc = readVcc();
		
		if (!(h == 0 && t == 0))
		{
			payload.humidity = payload.humidity + (h - payload.humidity)*0.50;
			payload.temperature = payload.temperature + (t - payload.temperature)*0.50;
		
			payload.dewpoint = calculateDewpoint (payload.humidity, payload.temperature);
			payload.cloudbase = calculateCloudbase (payload.temperature, payload.dewpoint);
				
			//send sensor readings		
			send_rf_data();  
		}
#if DEBUG
		printData();
#endif
	Sleepy::loseSomeTime(SEND_PERIOD);
}


void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &payload, sizeof payload);//RF12_HDR_ACK
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);

  rf12_sleep(RF12_SLEEP);  
}

void printData()
{
#if DEBUG
	Serial.print(" l: ");
	Serial.print(payload.light, DEC);
	Serial.print(" h ");
	Serial.print(payload.humidity);
	Serial.print(" t ");
	Serial.print(payload.temperature);
	Serial.print(" d ");
	Serial.print(payload.dewpoint);
	Serial.print(" c ");
	Serial.print(payload.cloudbase);
	Serial.print(" v: ");
	Serial.println(payload.vcc, DEC);
#endif
}


int readVcc() {
  int result;

  ADMUX = _BV(MUX5) | _BV(MUX0);

  delay(2);					// Wait for Vref to settle
  ADCSRA |= _BV(ADSC);				// Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;			//1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
}


// utility code to perform simple smoothing as a running average
static int smoothedAverage(int prev, int next, byte firstTime =0) {
    if (firstTime)
        return next;
    return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
}


//Light Reading
static void doLDRMeasure() {
    byte firstTime = payload.humidity == 0; // special case to init running avg
	
	digitalWrite(14+LDR_PORT, HIGH);
	delay(10);
    int light = analogRead(0);
	digitalWrite(14+LDR_PORT, LOW);

	light = map(light, 150, 1023, 100, 0);
	light = constrain(light, 0, 100);

    payload.light = light;//smoothedAverage(payload.light, light, firstTime);
}

//cloudBase calculation
float calculateCloudbase (float t, float d)
{ 
	t *= 0.1;
  return ( (t - d) * 400 );
}

float calculateDewpoint(float h, float t)
{
	h *= 0.1;
	t *= 0.1;
	float k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t); 
	
	return 243.12*k/(17.62-k); 
}