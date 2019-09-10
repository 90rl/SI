/*
This code is based on the examples at http://forum.arduino.cc/index.php?topic=396450
*/


// Example 5 - parsing text and numbers with start and end markers in the stream

#include "helper.h"

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

dataPacket packet;


boolean newData = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////


QMC5883 qmc;

void SetPowerLevel(PowerSideEnum side, int level)
{
	level = constrain(level, -255, 255);

	if (side == PowerSideEnum::Right) {
		if (level > 0) {
			// do przodu
			digitalWrite(A_PHASE, 1);
			analogWrite(A_ENABLE, level);
		} else if (level < 0) {
			// do tyłu
			digitalWrite(A_PHASE, 0);
			analogWrite(A_ENABLE, -level);
		} else {
			// stop
			digitalWrite(A_PHASE, 0);
			analogWrite(A_ENABLE, 0);
		}
	}

	if (side == PowerSideEnum::Left) {
		if (level > 0) {
			// do przodu
			digitalWrite(B_PHASE, 1);
			analogWrite(B_ENABLE, level);
		} else if (level < 0) {
			// do tyłu
			digitalWrite(B_PHASE, 0);
			analogWrite(B_ENABLE, -level);
		} else {
			// stop
			digitalWrite(B_PHASE, 0);
			analogWrite(B_ENABLE, 0);
		}
	}
}



/////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//============

void setup() {
//    Serial.begin(9600);
//    Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
//    Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
//    Serial.println();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////
	// Czujniki ultradźwiekowe
	for (int i = (int)UltraSoundSensor::__first; i <= (int)UltraSoundSensor::__last; i++)
	{
		pinMode(ultrasound_trigger_pin[i], OUTPUT);
		pinMode(ultrasound_echo_pin[i], INPUT);

		digitalWrite(ultrasound_trigger_pin[i], 0);
	}

	// Silniki
	pinMode(A_PHASE, OUTPUT);
	pinMode(A_ENABLE, OUTPUT);
	pinMode(B_PHASE, OUTPUT);
	pinMode(B_ENABLE, OUTPUT);

	//pinMode(MODE, OUTPUT); -- podłaczone na krótko ze stanem wysokim
	//digitalWrite(MODE, true);  -- podłaczone na krótko ze stanem wysokim

	SetPowerLevel(PowerSideEnum::Left, 0);
	SetPowerLevel(PowerSideEnum::Right, 0);

	// Wejścia enkoderowe
	pinMode(ENCODER_LEFT, INPUT);
	pinMode(ENCODER_RIGHT, INPUT);

	Serial.begin(9600);
	Serial.print("Test... ");

	Wire.begin();
	qmc.init();

	Serial1.begin(9600); // HC06

/////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
void loop(void)
{
	delay(1000);


	SetPowerLevel(Side_Left, 100);
	delay(2000);

	SetPowerLevel(Side_Left, 200);
	delay(2000);

	SetPowerLevel(Side_Left, 255);
	delay(2000);


	SetPowerLevel(Side_Left, 0);
	SetPowerLevel(Side_Right, 0);

	SetPowerLevel(Side_Left, -100);
	delay(2000);

	SetPowerLevel(Side_Left, -200);
	delay(2000);

	SetPowerLevel(Side_Left, -255);
	delay(2000);

	SetPowerLevel(Side_Left, 0);
	SetPowerLevel(Side_Right, 0);
	delay(4000);




	delay(1000);

	SetPowerLevel(Side_Right, 100);
	delay(2000);

	SetPowerLevel(Side_Right, 200);
	delay(2000);

	SetPowerLevel(Side_Right, 255);
	delay(2000);


	SetPowerLevel(Side_Right, 0);
	SetPowerLevel(Side_Right, 0);

	SetPowerLevel(Side_Right, -100);
	delay(2000);

	SetPowerLevel(Side_Right, -200);
	delay(2000);

	SetPowerLevel(Side_Right, -255);
	delay(2000);

	SetPowerLevel(Side_Right, 0);
	SetPowerLevel(Side_Right, 0);
	delay(4000);
}

int measureSoundSpeed(int trigger_pin, int echo_pin)
{
	digitalWrite(trigger_pin, false);
	delayMicroseconds(2);

	digitalWrite(trigger_pin, true);
	delayMicroseconds(10);
	digitalWrite(trigger_pin, false);

	// zmierz czas przelotu fali dźwiękowej
	int duration = pulseIn(echo_pin, true, 50 * 1000);


	// przelicz czas na odległość (1/2 Vsound(t=20st.C))
	int distance = (int)((float)duration * 0.03438f * 0.5f);
	return distance;
}




//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        packet = parseData();
        showParsedData(packet);
        newData = false;
    }
}

//============


void cmd_proximity(const char* msg, UltraSoundSensor sensor)
{
	if (sensor == UltraSoundSensor::All) {

		char buffer[128];

		int d[4][5] = {0};
		int sum[4] = {0};
		int id[4] = {0};
		int dist[4] = {0};

		while (Serial.available() == 0)
		{
			for (int sens = (int)UltraSoundSensor::Front; sens <= (int)UltraSoundSensor::Right; sens++) {
				dist[sens] = measureSoundSpeed(
					ultrasound_trigger_pin[sens],
					ultrasound_echo_pin[sens]);

				// średnia krocząca
				sum[sens] -= d[sens][id[sens]];
				sum[sens] += d[sens][id[sens]] = dist[sens];
				id[sens] = (id[sens] + 1) % 5;
				dist[sens] = sum[sens] / 5;

			}
			sprintf(buffer, "\nFRONT: %4dcm; BACK: %4dcm; LEFT: %4dcm; RIGHT: %4dcm; ",
				dist[(int)UltraSoundSensor::Front],
				dist[(int)UltraSoundSensor::Back],
				dist[(int)UltraSoundSensor::Left],
				dist[(int)UltraSoundSensor::Right]);
			Serial.print(buffer);
		}


	} else {

		char buffer[64];
		int d[5] = {};
		int sum = 0;
		int id = 0;

		while (Serial.available() == 0)
		{
			int dist = measureSoundSpeed(
				ultrasound_trigger_pin[(int)sensor],
				ultrasound_echo_pin[(int)sensor]);

			// średnia krocząca
			sum -= d[id];
			sum += d[id] = dist;
			id = (id + 1) % 5;
			dist = sum / 5;

			sprintf(buffer, "\n%s: %0dcm", msg, dist);
			Serial.print(buffer);
		}

	}

	while (Serial.available())
		Serial.read();
}

void cmd_encoders(void)
{
	pinMode(ENCODER_LEFT, INPUT);
	pinMode(ENCODER_RIGHT, INPUT);

	char buffer[] = {'\n', 'L', '-','R','-','\x0'}; // 2, 4

	while (Serial.available() == 0)
	{
		buffer[2] = '0' + digitalRead(50);
		buffer[4] = '0' + digitalRead(51);

		Serial.print(buffer);
	}

	while (Serial.available())
		Serial.read();
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

dataPacket parseData() {      // split the data into its parts

    dataPacket tmpPacket;

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(tmpPacket.message, strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    tmpPacket.packet_int = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    tmpPacket.packet_float = atof(strtokIndx);     // convert this part to a float

    return tmpPacket;
}


void showParsedData(dataPacket packet) {
    Serial.print("Message ");
    Serial.println(packet.message);
    Serial.print("Integer ");
    Serial.println(packet.packet_int);
    Serial.print("Float ");
    Serial.println(packet.packet_float);
}

#define TEMP_READ_DELAY 800

//pid settings and gains
#define OUTPUT_MIN -5
#define OUTPUT_MAX 5
#define KP 0.12
#define KI 0.0003
#define KD 0

double input, setPoint, outputVal=0;

AutoPID myPID(&input, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long lastTempUpdate;


void loop(void)
{
	delay(800);
	qmc.measure();
//  int16_t x = qmc.getX();
//  int16_t y = qmc.getY();



//Sprawdzenie gdzie jesteśmy z odczytami
//    char buffer[64];
//    sprintf(buffer, "\n X=%5d Y=%5d", x, y);
//    Serial.print(buffer);
  // heading jest podany w radianach
//  float heading = atan2(y, x);
  //przeliczenie z radianów na stopnie
//  float  headingRad=heading*180/M_PI;

//    sprintf(buffer, "\nheadingRad  X=%5f", headingRad);
//    Serial.print(buffer);

   //wartość wejściowa
  setPoint = 0; // dążymy do środka
  input= (double)packet.packet_int; // wejście do PID

  myPID.run();
//    sprintf(buffer, "\noutputVal  X=%5f", outputVal);
//    Serial.print(buffer);

float normalSpeed=150;
float s=10;


if(dist[(int)UltraSoundSensor::Front] >30){
  if((float)outputVal>0 ){
   SetPowerLevel(PowerSideEnum::Right, normalSpeed+outputVal*s);
   SetPowerLevel(PowerSideEnum::Left, normalSpeed-outputVal*s);
  } else if((float)outputVal<0 ){
   SetPowerLevel(PowerSideEnum::Right, normalSpeed+outputVal*s);
   SetPowerLevel(PowerSideEnum::Left, normalSpeed-outputVal*s);
  } else{
   SetPowerLevel(PowerSideEnum::Right, normalSpeed);
   SetPowerLevel(PowerSideEnum::Left, normalSpeed);
  }
} else {
   SetPowerLevel(PowerSideEnum::Right, 0);
   SetPowerLevel(PowerSideEnum::Left, 0);
}
}