/********************************************************
 * PID Proportional on measurement Example
 * Setting the PID to use Proportional on measurement will 
 * make the output move more smoothly when the setpoint 
 * is changed.  In addition, it can eliminate overshoot
 * in certain processes like sous-vides.
 ********************************************************/
#include <Wire.h>
#include <INA219_WE.h>
#define I2C_ADDRESS 0x40


#include <PID_v1.h>

#define MCP4725_ADDR    0x60		// The address depends on the state of pin A0

static union {
  uint16_t dacValue = 4095;
  uint8_t data[2];
};
/* There are several ways to create your INA219 object:
 * INA219_WE ina219 = INA219_WE(); -> uses Wire / I2C Address = 0x40
 * INA219_WE ina219 = INA219_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
 * INA219_WE ina219 = INA219_WE(&Wire); -> you can pass any TwoWire object
 * INA219_WE ina219 = INA219_WE(&Wire, I2C_ADDRESS); -> all together
 */
INA219_WE ina219 = INA219_WE(I2C_ADDRESS);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

    Serial.begin(115200);
  Wire.begin();
  if(!ina219.init()){
    Serial.println("INA219 not connected!"); 
    while(1);
  }


 
  /* Set ADC Mode for Bus and ShuntVoltage
  *   * Mode *          * Res / Samples *     * Conversion Time *
    BIT_MODE_9        9 Bit Resolution             84 µs
    BIT_MODE_10       10 Bit Resolution            148 µs  
    BIT_MODE_11       11 Bit Resolution            276 µs
    BIT_MODE_12       12 Bit Resolution            532 µs  (DEFAULT)
    SAMPLE_MODE_2     Mean Value 2 samples         1.06 ms
    SAMPLE_MODE_4     Mean Value 4 samples         2.13 ms
    SAMPLE_MODE_8     Mean Value 8 samples         4.26 ms
    SAMPLE_MODE_16    Mean Value 16 samples        8.51 ms     
    SAMPLE_MODE_32    Mean Value 32 samples        17.02 ms
    SAMPLE_MODE_64    Mean Value 64 samples        34.05 ms
    SAMPLE_MODE_128   Mean Value 128 samples       68.10 ms
  */
  //ina219.setADCMode(SAMPLE_MODE_128); // choose mode and uncomment for change of default
  
  /* Set measure mode
    POWER_DOWN  - INA219 switched off
    TRIGGERED   - measurement on demand
    ADC_OFF     - Analog/Digital Converter switched off
    CONTINUOUS  - Continuous measurements (DEFAULT)
  */
  // ina219.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  
 /* Set PGain
  * Gain *  * Shunt Voltage Range *         * Max Current *
    PG_40          40 mV               0.4 A * 0.1 / shuntSizeInOhms 
    PG_80          80 mV               0.8 A * 0.1 / shuntSizeInOhms 
    PG_160        160 mV               1.6 A * 0.1 / shuntSizeInOhms 
    PG_320        320 mV               3.2 A * 0.1 / shuntSizeInOhms (DEFAULT)
  */
 //ina219.setPGain(PG_320); // choose gain and uncomment for change of default
  
  /* Set Bus Voltage Range
    BRNG_16   -> 16 V
    BRNG_32   -> 32 V (DEFAULT)
  */
  ina219.setBusRange(BRNG_16); // choose range and uncomment for change of default

  /* If the current values delivered by the INA219 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA219
  */
  ina219.setCorrectionFactor(0.95360824742); // insert your correction factor if necessary

  /* If you experience a shunt voltage offset, that means you detect a shunt voltage which is not 
     zero, although the current should be zero, you can apply a correction. For this, uncomment the 
     following function and apply the offset you have detected.   
  */
  // ina219.setShuntVoltOffset_mV(0.0); // insert the shunt voltage (millivolts) you detect at zero current

  /* Set shunt size
     If you don't use a module with a shunt of 0.1 ohms (R100) you can change set the shunt size 
     here. 
  */
  ina219.setShuntSizeInOhms(0.1); // Insert your shunt size in ohms
  
  Serial.println("INA219 Set Shunt Size"); 


  //initialize the variables we're linked to
  Input = ina219.getCurrent_mA();
  Setpoint = 100;   // 120 is SET Current load = 0.5A for inbuild ADC 

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {

  // DAC wiper control
  setRegisterValue();
  
  if (dacValue == 0) {
    dacValue = 4095;
  } else {
    dacValue--;
  }
  
 // Get register value 
 uint16_t getRegisterValue() {
   Wire.requestFrom(MCP4725_ADDR, 2, false);
   data[1] = Wire.read();
   data[0] = Wire.read();
  } 

  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0; 
  bool ina219_overflow = false;
  
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
  
  Serial.print("Shunt Voltage [mV]: "); Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: "); Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: "); Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: "); Serial.println(current_mA);
  Serial.print("Bus Power [mW]: "); Serial.println(power_mW);
  if(!ina219_overflow){
    Serial.println("Values OK - no overflow");
  }
  else{
    Serial.println("Overflow! Choose higher PGAIN");
  }
  Serial.println();


  Input = current_mA;
  myPID.Compute();
  
  // Set register value
void setRegisterValue() {
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(data[1]);
  Wire.write(data[0]);
  Wire.endTransmission(true);
  analogWrite(3,Output);
}