#include <arduinoFFT.h>

#define STEPPER_PIN 2 //output pin for Stepper
#define LIGHT_INTENSITY_ANALOG A0// value for analog input to read light intensity

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

unsigned int  stepperResistance = 224;// value to set speed of Stepper 

int delayTime = 63; // 8s/samples=63ms, samples-- number of data for FFT, 8s-- raotation Time 2Pi okres T = 8s
int startStepper = 0;
/**************************intialized values for FFT******************************/
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 200; //sampling Frequency 128/8s = 16
unsigned int counterData = 0; //value to count number of data
int sensorValue = 0;
/***********************************************
 * Those are the input and output vectors      *
 *Input vectors receive computed results from FFT
 ***********************************************/
double vReal[samples];
double vImag[samples];
double valuesToSFunction[5];
double a0, a4, b4, b2;
double stokesVector[4];
double resultArray[4];

/***************intialized Button values and values to start measurement***************/
#define BUTTON_PIN 4
#define DIODE_PIN 6
#define CHECK_STEPPER_POSITION_ANALOG A1 
int buttonState = 0; 
int qWavePosition = 0;
int qWavePositionLimit = 900;
int startMeasurement = 0;
/*****************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initialized done");
  pinMode(STEPPER_PIN,OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(DIODE_PIN, OUTPUT);
  noTone(STEPPER_PIN);
  
  for(int ii = 0; ii<4;ii++){
    stokesVector[ii] = 0;
    resultArray[ii] = 0;
  }
}

void loop() {
  /******If button has been pushed start measurement and stepper********/
  if(digitalRead(BUTTON_PIN) == HIGH){
    delay(20);
    startStepper = 1;
    delay(20);
  }

  if(startStepper == 1){
    digitalWrite(DIODE_PIN,HIGH);// turn on diode to set Q-wave position
    if(counterData<samples){ 
      qWavePosition = analogRead(CHECK_STEPPER_POSITION_ANALOG);// read light intesity from sensor to set position of quarter wave 
      //Serial.println(qWavePosition);
      tone(STEPPER_PIN,stepperResistance );//Start stepper
      if(qWavePosition>qWavePositionLimit && startMeasurement == 0){//set position of quater wave (vertical o q-wave axis)
        Serial.println("Start measurement..............");
        startMeasurement = 1;
        }
      if(startMeasurement == 1){//if possition is vertical start measurement
        sensorValue = analogRead(LIGHT_INTENSITY_ANALOG);
        //Serial.println(sensorValue);
        vReal[counterData] = sensorValue;// collecting data for FFT
        vImag[counterData] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
        counterData++;     
      }
      delay(delayTime);
    }
    else { /********* compute FFT and set Stokes Vector *************/
      digitalWrite(DIODE_PIN,HIGH);
      noTone(STEPPER_PIN);// stop Stepper
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
      //Serial.println("Computed Real values:");PrintVector(vReal, samples, SCL_INDEX);Serial.println("Computed Imaginary values:");PrintVector(vImag, samples, SCL_INDEX);
      counterData = 0;
      /******** getting special values to set Stokes Vector *********/
      a0= vReal[1];
      a4= vReal[5];
      b4 = vImag[5];
      b2 = vImag[3];
      /******** count Stokes Vector ************/
      stokesVector[0] = (a0/2 - a4);
      stokesVector[1] = (2*a4);
      stokesVector[2] = (2*b4);
      stokesVector[3] = b2;
      for(int ii =0; ii<4; ii++){
        Serial.print("S(");Serial.print(ii);Serial.print(") = "); Serial.println(stokesVector[ii]);
      }
      Serial.println("------------------------------------------------------------");
      startMeasurement = 0;
      startStepper = 0;      
    }
  }
  else noTone(STEPPER_PIN);// stop Stepper
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType){
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(vData[i], 4);
    Serial.println();
  }
  Serial.println();
}
