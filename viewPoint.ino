/*
MarvinBase

Basic controls of IoT Academy Marvin LoRa Development board.

This version supports:
  - Sending LoRa uplink messages using ABP 
  - Blink three times when sending data
  - Power control to RN2483 module

Instructions:
  - Get the latest version of the Arduino software
  - In Arduino IDE select Arduino Leonardo and com port of your device
  - Please adjust ABP adresses and key below to match yours
  - The loop() is where the actual stuff happens. Adjust input of send_lora_data() in void loop() to send your own data.
*/
#include "CayenneLPP.h";

// set max size
int     MAX_SIZE = 52;
CayenneLPP lpp(MAX_SIZE);

// Port to assign the type of lora data (any port can be used between 1 and 223)
int     set_port  = 1;

// Some standard ports that depend on the layout of the Marvin
long    defaultBaudRate = 57600;
int     reset_port = 5;
int     RN2483_power_port = 6; //Note that an earlier version of the Marvin doesn't support seperate power to RN2483
int     led_port = 13;

//** Set parameters here BEGIN ---->
String  set_nwkskey = "ce0853d32c393bcb3fd3896c89de370e";
String  set_appskey = "82815ae75fe7442359020b23984ae60f";
String  set_devaddr = "04001E0C";
//** <---- END Set parameters here


// Some global items
String reader = ""; 

// counter
int   count = 1; 


//* Set thigs right for the Grove temperature / humidity sensor
#include "DHT.h"      //download it here: https://github.com/Seeed-Studio/Grove_Temperature_And_Humidity_Sensor
                      // press clone/download and then download as .zip
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

uint16_t errorsAndWarnings = 0;
int counter = 0;

int xOld = 0;
int yOld = 0;
int zOld = 0;

float dX = 0;
float dY =0;
float dZ =0;

float SOS = 0;
float SOSmin = 0;
float SOSmax =0;

float accel = 0;

//Create instance of LSM6DS3Core
LSM6DS3Core myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

#define DHTPIN A3     // A3 is closes to the usb port of Marvin

// define the type of sensor used (there are others)
#define DHTTYPE DHT11   // DHT 11 

DHT dht(DHTPIN, DHTTYPE);

/*
 * Setup() function is called when board is started. Marvin uses a serial connection to talk to your pc and a serial
 * connection to talk to the RN2483, these are both initialized in seperate functions. Also some Arduino port are 
 * initialized and a LED is called to blink when everything is done. 
 */
void setup() {

  Serial.begin(defaultBaudRate);
  Serial1.begin(defaultBaudRate);
  InitializeSerials(defaultBaudRate);

    //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("\nDevice Error.\n");
  }
  else
  {
    Serial.print("\nDevice OK.\n");
  }

  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  //Now, write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  
  initializeRN2483(RN2483_power_port, reset_port);
  pinMode(led_port, OUTPUT); // Initialize LED port  
  dht.begin();
  blinky();
}

void loop() {
    SOSmin = 0;
    SOSmax = 0;
    while(counter < 31) {

      int16_t xAxis;
      int16_t yAxis;
      int16_t zAxis;
      
      //Acelerometer axis X
      if( myIMU.readRegisterInt16(&xAxis, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0 )
      {
        errorsAndWarnings++;
      }
      dX = sq(xAxis - xOld)/1000;
      Serial.print("dX: ");
      Serial.println(dX);
      xOld = xAxis;
  
      //Acelerometer axis Y 
      if( myIMU.readRegisterInt16(&yAxis, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0 )
      {
        errorsAndWarnings++;
      }
      dY = sq(yAxis - yOld)/1000;
  
      yOld = yAxis;
      
      //Acelerometer axis Z  
      if( myIMU.readRegisterInt16(&zAxis, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0 )
      {
        errorsAndWarnings++;
      }
      dZ = sq(zAxis - zOld)/1000;
  
      zOld = zAxis; 
    
      SOS = (dX + dY + dZ);
      
      if(SOS<SOSmin){
        SOSmin = SOS;
      };
      if(SOS>SOSmax){
        SOSmax = SOS;
     };
    
    counter++;
    delay(100);
  };
    counter = 0;
    accel = (SOSmax - SOSmin);
    Serial.print("Acceleration: ");
    Serial.println(accel);
    
    char payload[MAX_SIZE] = ""; 
    lpp.reset(); 

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(t) || isnan(h)) 
    {
        Serial.println("Failed to read from DHT");
    } 
    else 
    {
        Serial.print("Humidity: "); 
        Serial.print(h);
        Serial.print(" %\t");
        Serial.print("Temperature: "); 
        Serial.print(t);
        Serial.println(" *C");
    }
  int temp = (int) t;
  int hum = (int) h;  
  int tempdec = t * 100;
  int humdec = h * 100;
 
  lpp.addDigitalInput(1,accel);
  lpp.addTemperature(2,temp); 
  lpp.addRelativeHumidity(3,hum); 

  uint8_t buff = *lpp.getBuffer();
  
  Serial.print("Buffer size:" );
    Serial.println(lpp.getSize());

    for (int i = 0; i < lpp.getSize(); i++) {
      char tmp[16];

      sprintf(tmp, "%.2X",(lpp.getBuffer())[i]);
      strcat(payload, tmp);
    }

    Serial.print("Buffer content:" );
    Serial.println(payload);

    send_LoRa_data(set_port, payload);

    delay(10000);
    Serial.println(count);
    count = count + 1;

  //} // end if

 // end loop()

  
//  send_LoRa_data(set_port, 67 + String(temp) + "F" + String(hum));      //send temp / hum as rounded int over lora
  //send_LoRa_data(set_port, String(tempdec) + "F" + String(humdec)); //send temp / hum as 4 digit integer (decimals included)

  blinky();
  delay(1000);
  read_data_from_LoRa_Mod();
  delay(30000);
}

void InitializeSerials(int baudrate)
{
  delay(1000);
  print_to_console("Serial ports initialised");
}

void initializeRN2483(int pwr_port, int rst_port)
{
  //Enable power to the RN2483
  pinMode(pwr_port, OUTPUT);
  digitalWrite(pwr_port, HIGH);
  print_to_console("RN2483 Powered up");
  delay(1000);
  
  //Disable reset pin
  pinMode(rst_port, OUTPUT);
  digitalWrite(rst_port, HIGH);

  //Configure LoRa module
  send_LoRa_Command("sys reset");
  read_data_from_LoRa_Mod();

  send_LoRa_Command("radio set crc off");
  delay(1000);
  read_data_from_LoRa_Mod();

  send_LoRa_Command("mac set nwkskey " + set_nwkskey);
  read_data_from_LoRa_Mod();

  send_LoRa_Command("mac set appskey " + set_appskey);
  read_data_from_LoRa_Mod();

  send_LoRa_Command("mac set devaddr " + set_devaddr);
  read_data_from_LoRa_Mod();

  //For this commands some extra delay is needed.
  send_LoRa_Command("mac set adr on");
  delay(1000);
  read_data_from_LoRa_Mod();

  send_LoRa_Command("mac save");
  delay(1000);
  read_data_from_LoRa_Mod();

  send_LoRa_Command("mac join abp");
  delay(1000);
  read_data_from_LoRa_Mod();

}

void print_to_console(String message)
{
  Serial.println(message);
}

void read_data_from_LoRa_Mod()
{
  if (Serial1.available()) {
    String inByte = Serial1.readString();
    Serial.println(inByte);
  }

}

void send_LoRa_Command(String cmd)
{
  print_to_console("Now sending: " + cmd);
  Serial1.println(cmd);
  delay(500);
}

void send_LoRa_data(int tx_port, String rawdata)
{
  send_LoRa_Command("mac tx uncnf " + String(tx_port) + String(" ") + rawdata);
}


void blinky()
{
  digitalWrite(led_port, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                     // wait for a second
  digitalWrite(led_port, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                     // wait for a second
  digitalWrite(led_port, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                     // wait for a second
  digitalWrite(led_port, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                     // wait for a second

}

