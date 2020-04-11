#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <LS7366.h>
#include <Adafruit_NeoPixel.h>


#define VBAT_MV_PER_LSB             (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER                (0.5F)          // 100K + 100K voltage divider on VBAT = (100K / (100K + 100K))
#define VBAT_DIVIDER_COMP           (2.0F)          // Compensation factor for the VBAT divider
#define ADC_SAMPLES                 (10)            // Number of samples in ADC read
#define VBAT_READ_INTERVAL          (300000)        // 300000 ms = 5 minutes read battery interval
#define CHRONOJUMP_SERIAL_INTERVAL  (1)             // 1ms interval Serial Output data to chronojump
#define TIMEOUT_POWEROFF            (300000)        // 300000 ms = 5 minutes before to power off

#define QUADMOD1X                   (1)
#define QUADMOD2X                   (2)
#define QUADMOD4X                   (4)

#define NUMBYTES1                   (1)
#define NUMBYTES2                   (2)
#define NUMBYTES3                   (3)
#define NUMBYTES4                   (4)

/**
 * @brief PIN definitions
 * 
 */
#define VBAT_PIN                    (20)            // A6 - P0.29 Voltage divider to read battery voltage 
#define SERIAL_TX_PIN               (A1)
#define SERIAL_RX_PIN               (A0)
#define LS7366_CS_PIN               (13)            // D13 - P1.09 This is the Slave Select PIN for LS7366 chip
#define NEOMATRIX_LED_PIN           (8)             // P0.16 RGB Led Data Pin.    
#define VBAT_ENABLE_PIN             (11)            // D11 - P0.06 Put this PIN LOW to measure the battery voltage level. Keep voltage HIGH while not reading, then put LOW, read and come back to HIGH. If we missed to put high, the voltage divider disconnect in about 2ms.
#define PWREN_PIN                   (9)             // D9 - P0.26 Output is LOW after the device has been configured by USB, then high during USB suspend mode. This output can be used to control power to external logic P-Channel logic level MOSFET switch. Enable the interface pull-down option when using the PWREN# in this way.  
#define BCD_CHARGER_PIN             (14)            // A0 - P0.04 Output is LOW when Battery Charge Detect, indicates when the device is connected to a dedicated battery charger. 
#define INT_PIN                     (12)            // D12 - P0.08 This PIN is input we must put Pull-UP. When The push button received a power-off signal go LOW. We can use to read short pulse signal. {1=> Ignore, 2=> Enter Serial Mode, 3=> Exit Serial Mode, 4=> Enter DFU Mode, LONG=> Force power OFF}
#define DFLAG_PIN                   (5)             // D5 - P1.08 This PIN goes LOW with every INDEX encoder count.
#define KILL_PIN                    (6)             // D6 - P0.07 This PIN can shutdown the system. Must be tied to HIGH while system is poweron, and go to LOW whe we want to shutdown.

/**
 * @brief PIN DEFINITIONS 
 * #define PWREN_PIN                26            // P0.26 Output is LOW after the device has been configured by USB, then high during USB suspend mode. This output can be used to control power to external logic P-Channel logic level MOSFET switch. Enable the interface pull-down option when using the PWREN# in this way.  
 * #define BCD_CHARGER_PIN          4             // P0.04 Output is LOW when Battery Charge Detect, indicates when the device is connected to a dedicated battery charger. 
 * #define ANALOG_ENABLE_PIN        6             // P0.06 Put this PIN LOW to measure the battery voltage level. Keep voltage HIGH while not reading, then put LOW, read and come back to HIGH. If we missed to put high, the voltage divider disconnect in about 2ms.
 * #define LIPO_LEVEL_ANALOG_PIN    29            // P0.29 Voltage divider to read battery voltage
 * #define INT_PIN                  7             // P0.07 This PIN is input we must put Pull-UP. When The push button received a power-off signal go LOW. We can use to read short pulse signal. {1=> Ignore, 2=> Enter Serial Mode, 3=> Exit Serial Mode, 4=> Enter DFU Mode, LONG=> Force power OFF}
 * #define DFLAG_PIN                40            // P1.08 This PIN goes LOW with every INDEX encoder count.
 * #define SS_LS7366_PIN            41            // P1.09 This is the Slave Select PIN for LS7366 chip
 * #define SERIAL_TX_PIN            
 * #define SERIAL_RX_PIN            
 * #define NEOPIX_PIN               16            // P0.16 RGB Led Data Pin.
 *    
 */

/**
 * @brief BLUETOOTH COMMANDS definitions 
 *  
 */
#define COMMAND_GET_BATTERY         (0x42)        // 'B','66' Command to do a battery voltage read
#define COMMAND_OTA                 (0x40)        // '@','64' Command to enter DFU OTA mode    
#define COMMAND_RESET_COUNTER       (0x23)        // '#','35' Command to Reset Encoder counter to Cero 
#define COMMAND_SERIAL_OFF          (0x29)        // ')','41' Command to Disable Serial Output
#define COMMAND_SERIAL_ON           (0x28)        // '(','40' Command to enable Serial output
#define COMMAND_TURN_LED            (0x4C)        // 'L','76' Command to ENABLE/DISABLE the Led
#define COMMAND_ENCODER_1X          (0x3C)        // '<','60' Command to put encoder on 1X mode
#define COMMAND_ENCODER_2X          (0x3D)        // '=','61' Command to put encoder on 2X mode
#define COMMAND_ENCODER_4X          (0x3E)        // '>','62' Command to put encoder on 3X mode
#define COMMAND_ENCODER_OUTPUT_2    (0x32)        // '2','50' Command to put output encoder data to 2 bytes
#define COMMAND_ENCODER_OUTPUT_3    (0x33)        // '3','51' Command to put output encoder data to 3 bytes
#define COMMAND_ENCODER_OUTPUT_4    (0x34)        // '4','52' Command to put output encoder data to 4 bytes
#define COMMAND_DEFAULT_CONFIG      (0x21)        // '!','33' Command to put default configuration values
#define COMMAND_SIMULATE_DATA       (0x25)        // '%'.'37' Command to put device in ouput simulated data

#define COMMAND_SHUTDOWN            (0xDE)        // ''.'' Command to shutdown device


/**
 * @brief Characteristics define not in Bluefruit libraries
 */
#define UUID16_CHR_BATTERY_POWER_STATE (0x2A1A)

/**
 * @brief CHRONOJUMP Serial commands
 * 
 */
#define SER_COMMAND_ENCODER         (0x4A)        // 'J','74' ChronoJump Software asking for encoder presence
#define SER_COMMAND_SOFT_VER        (0x56)        // 'V','86' ChronoJump Get software version


#define MAX_PRPH_CONNECTION   1


LS7366 myLS7366(LS7366_CS_PIN);  
//SoftwareSerial mySerial(SERIAL_RX_PIN, SERIAL_TX_PIN); // RX, TX
Adafruit_NeoPixel signalLed = Adafruit_NeoPixel(5, NEOMATRIX_LED_PIN, NEO_GRB + NEO_KHZ800);


// Application variables
uint32_t encoderPosition = 0, lastEncoderPosition = 0;
uint32_t timeelapsed = 0;
uint8_t lastBatteryVoltagePer = 0;
int8_t volatile IncEncoderData = 0;
uint8_t batteryPowerState = 0B10001111;//org.bluetooth.characteristic.battery_power_state
int volatile batteryVoltageRaw;
bool isDeviceNotifyingEncoderData = false;
bool isDeviceNotifyingBatteryData = false;
bool isDeviceNotifyingBatteryPowerStateData = false;
bool usbPowerStatus = false;
bool batteryDischargingStatus = true;
bool usbConnectedStatus = false;

uint8_t quadMode = QUADMOD4X;
bool isIndexMode = false; //TODO: Change to false to production ready
uint8_t numBytesMode = NUMBYTES4;

bool isSerialEnable = false;
bool isSimulatedState = false;

uint8_t connection_count = 0;
uint16_t connectionHandles[MAX_PRPH_CONNECTION];
uint8_t encoderNotifyConn = 0;
uint16_t encoderNotifyConnHdls[MAX_PRPH_CONNECTION];
uint8_t batteryNotifyConn = 0;
uint16_t batteryNotifyConnHdls[MAX_PRPH_CONNECTION];
uint8_t batteryPowerStateConn = 0;
uint16_t batteryPowerStateNotifyConnHdls[MAX_PRPH_CONNECTION];

//
SoftwareTimer tm_blinkSignalLed;
SoftwareTimer tm_readBattery;
SoftwareTimer tm_encoderChronoJumpSerial;
SoftwareTimer tm_watchDogPowerOff;

// SemaphoreHandle_t xMutex;


/**
 * @brief Initialize LS7366 Chip with index and quadrature parameters
 * 
 * @param quadratureMode is the quadrature read mode 1x,2x,4x, value permited are 1,2,4
 * @param isIndexEnable if Index are used, must be true
 */

void initEncoderChip(uint8_t quadratureMode = 4, bool isIndexEnable = false, uint8_t numBytes = 4)
{
  uint8_t register_0 = FILTER_1 | FREE_RUN | NQUAD;
  uint8_t register_1 = NO_FLAGS | EN_CNTR | BYTE_4;

  if(!isIndexEnable) {
    register_0 += DISABLE_INDX; 
  } else {
    register_0 += INDX_LOADO;
    register_1 += IDX_FLAG;
  }

  if(quadratureMode == 1){
    register_0 += QUADRX1;  //1x Quadrature mode
  } else if(quadratureMode == 2){
    register_0 += QUADRX2;  //2x Quadrature mode
  } else if(quadratureMode == 4){
    register_0 += QUADRX4;  //4x Quadrature mode
  }

  if(numBytes == 1) {
    register_1 += BYTE_1;
  } else if(numBytes == 2) {
    register_1 += BYTE_2;
  } else if(numBytes == 3) {
    register_1 += BYTE_3;
  } else if(numBytes == 4) {
    register_1 += BYTE_4;
  }

  
  myLS7366.write_mode_register_0(register_0);
  myLS7366.write_mode_register_1(register_1);
  myLS7366.clear_counter();
  myLS7366.clear_status_register();
  myLS7366.write_data_register(4);
}

/**
 * @brief Read Battery Voltage Value {numRead} times
 * 
 * @param numRead Number of analog reads
 * @return int Return raw value 
 */

int readVBAT(uint8_t numRead) {
  int raw = 0;
  int rawVoltage = 0;
 
  //digitalWrite(VBAT_ENABLE_PIN,LOW);
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
 
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
 
  // Let the ADC settle
  digitalWrite(VBAT_ENABLE_PIN,LOW);
  //delay(3);
  for(uint8_t i=0;i<numRead;i++) {
    // Get the raw 12-bit, 0..3000mV ADC value
    int aux = analogRead(VBAT_PIN);
    Serial.println(aux);
    rawVoltage += aux;
  }
  
  raw = rawVoltage / numRead;
  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  digitalWrite(VBAT_ENABLE_PIN,HIGH);
  
  Serial.println(raw);
  return raw;
}

/**
 * @brief Callback from timer to read battery
 * 
 * @param xTimerID 
 */

void read_battery_callback(TimerHandle_t xTimerID)
{
  // freeRTOS timer ID, ignored if not used
  (void) xTimerID;

  batteryVoltageRaw = readVBAT(ADC_SAMPLES);
}

/**
 * @brief  Callback from timer to write on serial port ChronoJump data 
 * 
 * @param xTimerID 
 */

void serial_chronojump_callback(TimerHandle_t xTimerID)
{
  // freeRTOS timer ID, ignored if not used
  (void) xTimerID;
  taskENTER_CRITICAL();
  Serial1.write(IncEncoderData);
  Serial1.flush();
  IncEncoderData &= 0;
  taskEXIT_CRITICAL();
}
/**
 * @brief Callback from Timer to blink Signal Led
 * 
 * @param xTimerID 
 */

void blinkSignalLed_callback(TimerHandle_t xTimerID)
{
  static bool ledOn=false;
  (xTimerID);
  uint8_t r=0,g=0,b=0;

  switch(connection_count) {
    case 0 : 
      r = g = 0;
      b = 255;
    break;
    
    case 1 :
      r = 41;
      g = 121;
      b = 255;
    break;

    case 2 :
      r = 101;
      g = 31;
      b = 255;
    break;

    case 3 : //Not happens
      r = 213;
      g = 0;
      b = 249;
    break;

  }

  if(ledOn){
    ledAdvertisingOff(); 
  } else {
    signalLed.setPixelColor(0, r, g, b);
    signalLed.setBrightness(16);
    signalLed.show();  
  }
  ledOn = !ledOn;
  //Serial.print('/');
}

/**
 * @brief PowerOff Microcontroller lowering ltc2955 pin 
 * 
 * @param  
 * @return void 
 */
void powerOff(TimerHandle_t xTimerID)
{
  Serial.println("Powering OFF");
  Serial.flush();
  ledAdvertisingOff();
  delay(100);
  digitalWrite(KILL_PIN, LOW);  
}


/**
 * @brief Convert battery voltage from Milivolts to Percentage 
 * 
 * @param mvolts 
 * @return uint8_t 
 */

//TODO: Refactor this function to use a 3 lines aproximation
uint8_t mvToPercent(float mvolts) {
    uint8_t battery_level;

    if(mvolts > 4100) {
        battery_level = 100;
    } else if (mvolts >= 4000)
    {
        battery_level = 100 - (4100 - mvolts) / 10;
    }
    else if (mvolts > 3900)
    {
        battery_level = 90 - (4000 - mvolts) / 10;
    }
    else if (mvolts > 3700)
    {
        battery_level = 80 - (3900 - mvolts) / 10;
    }
    else if (mvolts > 3500)
    {
        battery_level = 60 - (3700 - mvolts) / 10;
    }
    else if (mvolts > 3200)
    {
        battery_level = 40 - (3500 - mvolts) / 8;
    }
    else
    {
        battery_level = 0;
    }
 
    return battery_level;
}

// UUUI from service, it must be in reverse order
const uint8_t encoderservice[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x38,0x05,0x39,0xad}; //"ad390538-d18b-11e6-bf26-cec0c932ce01"
const uint8_t encoderreadcharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x39,0x05,0x39,0xad}; // Encoder data
const uint8_t encoderwritecharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x3a,0x05,0x39,0xad}; // Command receive
const uint8_t encoderwritelegacycharac[16] = {0x01,0xce,0x32,0xc9,0xc0,0xce,0x26,0xbf,0xe6,0x11,0x8b,0xd1,0x3b,0x05,0x39,0xad}; // Legacy to emulate RFDuino

// Battery Service               : 0000180F-0000-1000-8000-00805F9B34FB

const uint8_t BMU_UUID_SERVICE[]
{
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00
};

// BLE Services
BLEDis            bledis; // DIS (Device Information Service) helper class instance
BLEService        blebas = BLEService(BMU_UUID_SERVICE); //Need to connect to callbacks
BLEService        bleencoders = BLEService(encoderservice);


BLECharacteristic encoderread = BLECharacteristic(encoderreadcharac);
BLECharacteristic encoderwrite1 = BLECharacteristic(encoderwritecharac);
BLECharacteristic encoderwrite2 = BLECharacteristic(encoderwritelegacycharac);

BLECharacteristic batteryBleChar = BLECharacteristic(UUID16_CHR_BATTERY_LEVEL);
BLECharacteristic batteryBlePowerStateChar = BLECharacteristic(UUID16_CHR_BATTERY_POWER_STATE);

void write_command(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  Serial.print("Received a value: ");
  Serial.println(data[0]);

  BaseType_t active = xTimerIsTimerActive(tm_encoderChronoJumpSerial.getHandle());

  if(len>0){
    switch (data[0])
    {

    case COMMAND_DEFAULT_CONFIG:
      isSerialEnable = true;
      if(!active) {
        tm_encoderChronoJumpSerial.start();
      }
      numBytesMode = NUMBYTES4;
      isIndexMode = false;
      quadMode = QUADMOD4X;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      isSimulatedState = false;
      break;

    case COMMAND_SIMULATE_DATA:
      isSimulatedState = !isSimulatedState;
      break;

    case COMMAND_TURN_LED:
      break;

    case COMMAND_GET_BATTERY:
      break;

    case COMMAND_OTA:
      enterOTADfu();
      break;
    
    case COMMAND_SERIAL_ON:    
      if(!active) {
        tm_encoderChronoJumpSerial.start();
      }
      isSerialEnable = true;
      break;

    case COMMAND_SERIAL_OFF:
      if(active) {
        tm_encoderChronoJumpSerial.stop();
      }
      isSerialEnable = false;
      break;

    case COMMAND_RESET_COUNTER:
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;
      
    case COMMAND_ENCODER_1X:
      quadMode = QUADMOD1X;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_ENCODER_2X:
      quadMode = QUADMOD2X;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_ENCODER_4X:
      quadMode = QUADMOD4X;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_ENCODER_OUTPUT_2:
      numBytesMode = NUMBYTES2;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_ENCODER_OUTPUT_3:
      numBytesMode = NUMBYTES3;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_ENCODER_OUTPUT_4:
      numBytesMode = NUMBYTES4;
      initEncoderChip(quadMode,isIndexMode,numBytesMode);
      break;

    case COMMAND_SHUTDOWN:
      powerOff(0);
      break;
    
    default:
      break;
    }
  }

}

/**
 * @brief Callback to handle connections 
 * 
 * @param conn_handle 
 */

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  //Bluefruit.getPeerName(conn_handle, central_name, sizeof(central_name));

  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  // EXPERIMENTAL: request PHY changed to 2MB
    Serial.println("Request to change PHY");
    connection->requestPHY();

    // request to update data length
    Serial.println("Request to change Data Length");
    connection->requestDataLengthUpdate();
    
    // request mtu exchange
    Serial.println("Request to change MTU");
    connection->requestMtuExchange(25);
    
    // request connection interval of 7.5 ms
    //connection->requestConnectionParameter(20); // in unit of 1.25
  
    delay(1000);
  
  connection->getPeerName(central_name,sizeof(central_name));
  
  // off Blue LED for lowest power consumption
  Bluefruit._setConnLed(false);
  Bluefruit.autoConnLed(false);
  Bluefruit._stopConnLed();
  //Bluefruit.setEventCallback()

  //Read battery Value
  batteryVoltageRaw = readVBAT(ADC_SAMPLES);
  
  uint8_t batteryVoltagePer = mvToPercent(batteryVoltageRaw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);

  //batteryBleChar.notify8(batteryVoltagePer);
  batteryBleChar.write8(batteryVoltagePer);

  Serial.print("Battery value mv: ");
  Serial.println(batteryVoltageRaw);
  Serial.print("Battery value %: ");
  Serial.println(batteryVoltagePer);
  Serial.print("Connected to ");
  Serial.println(central_name);
  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    if(connectionHandles[i] == BLE_CONN_HANDLE_INVALID) {
      connectionHandles[i] = conn_handle;
      break;
    }
  }

  if(connection_count>=MAX_PRPH_CONNECTION){
    tm_blinkSignalLed.stop();
    //Max connection reached
    ledAvertisingConnected();
  }

  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(connectionHandles[i]);
  }

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION) {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }

  //Disable Power Off
  tm_watchDogPowerOff.stop();
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  
  // ON Blue LED for advertising indicate
  Bluefruit.autoConnLed(true);
  Bluefruit._startConnLed();

  Serial.print("Disconnected reason: ");
  Serial.println(reason);
  Serial.println("Advertising!");

  connection_count--;
  
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    if(connectionHandles[i] == conn_handle) {
      connectionHandles[i] = BLE_CONN_HANDLE_INVALID;
    }
    if(encoderNotifyConnHdls[i] == conn_handle) {
      encoderNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
    }
    if(batteryNotifyConnHdls[i] == conn_handle) {
      batteryNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
    }
  }

  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    Serial.print("Element ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(connectionHandles[i]);
  }
  
  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
    Serial.println("Keep Blinking!");
    tm_blinkSignalLed.start();
  }

  //Stop actives timers
  if(connection_count < 1){
    BaseType_t active = xTimerIsTimerActive(tm_readBattery.getHandle());
    if(active) {
      tm_readBattery.stop();
    }
    tm_watchDogPowerOff.start();
  }

}

void batteryReadCallback(BLECharacteristic* chr, ble_gatts_evt_read_t * request)
{
  Serial.println("Read battery request received");
  //sd_ble_gatts_rw_authorize_reply(request->handle, )
  //request->handle
  //chr.write8(100);
  batteryBleChar.write8(100);
}

/**
 * @brief Callback to handle Characteristics subscription
 * 
 * @param chr 
 * @param cccd_value 
 */

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");
    
    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == encoderread.uuid) {
        if (cccd_value) {
            Serial.println("Encoder Data Measurement 'Notify' enabled");
            isDeviceNotifyingEncoderData = true;
            encoderNotifyConn++;
            for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
              if(encoderNotifyConnHdls[i] == BLE_CONN_HANDLE_INVALID) {
                encoderNotifyConnHdls[i] = conn_hdl;
                break;
              }
            }
        } else {
            Serial.println("Encoder Data Measurement 'Notify' disabled");
            (encoderNotifyConn > 0) ? encoderNotifyConn-- : encoderNotifyConn &= 0;
            for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
              if(encoderNotifyConnHdls[i] == conn_hdl) {
                encoderNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
              }
            }
            if(encoderNotifyConn < 1) {
              isDeviceNotifyingEncoderData = false;
            }
        }
    } else if (chr->uuid == batteryBleChar.uuid) {
      if (cccd_value) {
        Serial.println("Battery Data Measurement 'Notify' enabled");
        isDeviceNotifyingBatteryData = true;
        batteryVoltageRaw = readVBAT(ADC_SAMPLES); 
        uint8_t batteryVoltagePer = mvToPercent(batteryVoltageRaw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);
        batteryBleChar.notify8(batteryVoltagePer);
        batteryBleChar.write8(batteryVoltagePer);
        batteryNotifyConn++;
        for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
          if(batteryNotifyConnHdls[i] == BLE_CONN_HANDLE_INVALID) {
            batteryNotifyConnHdls[i] = conn_hdl;
            break;
          }
        }
        BaseType_t active = xTimerIsTimerActive(tm_readBattery.getHandle()); 
        if(!active) {
          tm_readBattery.start(); //Enable a timer to read battery value and notify value
        }          
      } else {
        Serial.println("Battery Data Measurement 'Notify' disabled");
        (batteryNotifyConn > 0) ? batteryNotifyConn-- : batteryNotifyConn &= 0;
        for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
          if(batteryNotifyConnHdls[i] == conn_hdl) {
            batteryNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
          }
        }
        if(batteryNotifyConn < 1) {
          isDeviceNotifyingBatteryData = false;
          tm_readBattery.stop(); //Disable a timer to read battery value
        }
      } 
    } else if(chr->uuid == batteryBlePowerStateChar.uuid) {
      if(cccd_value) { //subscribe
        Serial.println("Battery Power State 'Notify' enabled");
        isDeviceNotifyingBatteryPowerStateData = true;
        batteryPowerStateConn++;
        for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
          if(batteryPowerStateNotifyConnHdls[i] == BLE_CONN_HANDLE_INVALID) {
            batteryPowerStateNotifyConnHdls[i] = conn_hdl;
            break;
          }
        }
      } else { //unsubscribe
        Serial.println("Battery Power State 'Notify' disabled");
        (batteryPowerStateConn > 0) ? batteryPowerStateConn-- : batteryPowerStateConn &= 0;
        for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
          if(batteryPowerStateNotifyConnHdls[i] == conn_hdl) {
            batteryPowerStateNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
          }
        }
        if(batteryPowerStateConn < 1) {
          isDeviceNotifyingBatteryPowerStateData = false;
        }
      }
    }
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    Serial.print("Battery Power State Element ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(batteryPowerStateNotifyConnHdls[i]);
  }
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    Serial.print("Encoder Data Element ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(encoderNotifyConnHdls[i]);
  }
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include Encoder 128-bit uuid
  Bluefruit.Advertising.addService(bleencoders,blebas,bledis);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  

  delay(500);

  ledAvertisingInit();
}
/**
 * @brief Function to print out one byte in a readable, left-padded binary format 
 * 
 * @param val 
 */

void print_binary(byte val)
{
  byte i=0;
  for (i=0;i<8;i++){
    if (val & (0x01 << (7-i))) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    if (i==3) Serial.print("_");
  }
}

/**
 * @brief Output a simulated sinusoidal value
 * 
 * @param time 
 * @param period Repetition duration in milliseconds
 * @param heigth Max number of points  
 * @return int32_t 
 */

int32_t fn(uint32_t actualTime, float period, uint16_t heigth)
{
  
  float iteration = fmod((float) actualTime , period);
  float angle = ((iteration / period) * 2 * PI) + PI;
  float result = ((cos(angle) + 1) / 2) * heigth;
  
  return result;
}

void setup() {

  // Configure Signal LED
  signalLed.begin();
  signalLed.show(); // Initialize all pixels to 'off'
  signalLed.setPixelColor(0, 255, 255, 255);
  signalLed.setBrightness(255);
  signalLed.show();
  delay(100);
  ledAdvertisingOff();
  signalLed.setPixelColor(0, 0, 255, 0);
  signalLed.setBrightness(16);
  signalLed.show();

  //Clean Connected devices table
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    connectionHandles[i] = BLE_CONN_HANDLE_INVALID;
    encoderNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
    batteryNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
    batteryPowerStateNotifyConnHdls[i] = BLE_CONN_HANDLE_INVALID;
  }

  // configure D7 as input with a pullup (pin is active low)
  //pinMode(34, INPUT_PULLUP); ???? MISTAKE
  pinMode(KILL_PIN, OUTPUT); //Configuring PIN to shutdown the system via software
  digitalWrite(KILL_PIN, HIGH);
  pinMode(LS7366_CS_PIN, OUTPUT); //Configuring PIN fro LS7366 Chip select
  pinMode(VBAT_ENABLE_PIN, OUTPUT); //Configuring Pin to activate the battery resistor bridge
  digitalWrite(VBAT_ENABLE_PIN,HIGH);
  pinMode(INT_PIN, INPUT); //Configuring ON/OFF push button interrupt pin. It must have a external pullup resistor
  attachInterrupt(INT_PIN, digital_pushbutton_callback, ISR_DEFERRED | CHANGE);
  pinMode(DFLAG_PIN, INPUT);
  attachInterrupt(DFLAG_PIN, encoderEventCbk, ISR_DEFERRED | CHANGE);
  pinMode(PWREN_PIN, INPUT);
  attachInterrupt(PWREN_PIN, powerenableEventCbk, ISR_DEFERRED | CHANGE);
  pinMode(BCD_CHARGER_PIN, INPUT);
  attachInterrupt(BCD_CHARGER_PIN, chargerEventCbk, ISR_DEFERRED | CHANGE);


  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial1.begin(115200);
  //mySerial.begin(115200);
  //mySerial.println("Chronojump serial port enabled!!");

  SPI.begin();
  // Init encoder chip
  initEncoderChip();
  //myLS7366.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  //myLS7366.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  //myLS7366.clear_counter();
  //myLS7366.clear_status_register();
  //myLS7366.write_data_register(4);

  //Battery
  // Get a raw ADC reading
  batteryVoltageRaw = readVBAT(ADC_SAMPLES);
 
  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(batteryVoltageRaw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);

  //Experimental to view if troughput go HIGH
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  

  // Init Bluefruit
  Bluefruit.begin();

  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(100);
  
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(8);
  Bluefruit.setName("Isoinercial");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  //Experimental
  Bluefruit.Periph.setConnInterval(6, 12);

  
  // Start the BLE Battery Service and set it to their value
  Serial.println("Configuring the Battery Service");
  blebas.begin();

  batteryBleChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY); // could support notify
  batteryBleChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryBleChar.setFixedLen(1);
  batteryBleChar.setCccdWriteCallback(cccd_callback, true);
  //batteryBleChar.setReadAuthorizeCallback(batteryReadCallback);
  batteryBleChar.begin();
  batteryBleChar.write8(vbat_per);
  batteryBleChar.notify8(vbat_per);
  

  batteryBlePowerStateChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY); // could support notify
  batteryBlePowerStateChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryBlePowerStateChar.setFixedLen(1);
  batteryBlePowerStateChar.setCccdWriteCallback(cccd_callback, true);
  batteryBlePowerStateChar.begin();
  batteryBlePowerStateChar.write8(B10001111);


  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Isoinercial");
  bledis.setModel("Isoinercial Squat v1.0");
  bledis.setSoftwareRev("0.8.9");
  bledis.setHardwareRev("1.0.0"); 
  bledis.begin();

  //Configure and start the encoder Service
  Serial.println("Configuring the Encoder Service");
  bleencoders.begin();

  //Add Characteristics to Encoder Service
  Serial.println("Adding Characteristics");
  encoderread.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  encoderread.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  encoderread.setUserDescriptor("Data from encoder");
  encoderread.setFixedLen(8);
  encoderread.setCccdWriteCallback(cccd_callback, true);  // Optionally capture CCCD updates
  encoderread.begin();

  encoderwrite1.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  encoderwrite1.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  encoderwrite1.setUserDescriptor("Commands");
  encoderwrite1.setFixedLen(1);
  encoderwrite1.setWriteCallback(write_command, true);
  encoderwrite1.begin();

  encoderwrite2.setProperties(CHR_PROPS_WRITE);
  encoderwrite2.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  encoderwrite2.setUserDescriptor("Legacy");
  encoderwrite2.setFixedLen(1);
  encoderwrite2.begin();
 
  //Timer Creation related
  tm_blinkSignalLed.begin(500,blinkSignalLed_callback);
  Serial.print("Led timer handle ");
  Serial.println((int32_t) tm_blinkSignalLed.getHandle());
  
  tm_readBattery.begin(VBAT_READ_INTERVAL,read_battery_callback);
  //tm_readBattery.begin(500,blinkSignalLed_callback);
  Serial.print("Battery timer handle ");
  Serial.println((int32_t) tm_readBattery.getHandle());
  
  tm_encoderChronoJumpSerial.begin(CHRONOJUMP_SERIAL_INTERVAL, serial_chronojump_callback);
  Serial.print("Serial timer handle ");
  Serial.println((int32_t) tm_encoderChronoJumpSerial.getHandle());
  if(isSerialEnable) {
    tm_encoderChronoJumpSerial.start();
  }

  tm_watchDogPowerOff.begin(TIMEOUT_POWEROFF, powerOff);
  Serial.println("Power Off Timer enable");
  tm_watchDogPowerOff.start();

  // Call FTDI & USB poer state callbacks
  powerenableEventCbk();
  chargerEventCbk();

  
  // Set up and start advertising
  startAdv();
  Bluefruit.printInfo();
  Serial.println("Advertising!");
  
}

void loop() {

  timeelapsed = millis();
  uint32_t dummy_data[2] = {timeelapsed , encoderPosition};
  // Convert from raw mv to percentage (based on LIPO chemistry)  
  uint8_t batteryVoltagePer = mvToPercent(batteryVoltageRaw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);

  if(batteryVoltagePer != lastBatteryVoltagePer) {
      batteryBleChar.write8(batteryVoltagePer);
      //batteryBleChar.notify8(batteryVoltagePer);  
  }

  if(!isSimulatedState) {
    taskENTER_CRITICAL();
    encoderPosition = myLS7366.read_counter();
    IncEncoderData += encoderPosition - lastEncoderPosition;
    taskEXIT_CRITICAL();
  } else {
    taskENTER_CRITICAL();
    encoderPosition = fn(timeelapsed,2500,5000);
    IncEncoderData += encoderPosition - lastEncoderPosition;
    taskEXIT_CRITICAL();
  }

  //if (!Bluefruit.Advertising.isRunning()) {
    
    //Encoder related changes of data
    if(isDeviceNotifyingEncoderData) { 
      if(encoderPosition != lastEncoderPosition) {          
        //encoderread.notify(dummy_data,sizeof(dummy_data));
        notifyAllDevices(dummy_data, sizeof(dummy_data));
        //Serial.print(".");
      }
    }

    //Battery related changes of data
    if(batteryVoltagePer != lastBatteryVoltagePer) {
      if(isDeviceNotifyingBatteryData) {
        //batteryBleChar.notify8(batteryVoltagePer);
        notifyBatteryAllDevices(lastBatteryVoltagePer);
      }
    } 
  //}

  lastEncoderPosition = encoderPosition;
  lastBatteryVoltagePer = batteryVoltagePer;

}

void notifyAllDevices(const void* data, uint16_t len) {
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    if(encoderNotifyConnHdls[i] != BLE_CONN_HANDLE_INVALID) {
      encoderread.notify(encoderNotifyConnHdls[i], data, len);
    }
  }  
}

void notifyBatteryAllDevices(uint8_t data) 
{
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    if(batteryNotifyConnHdls[i] != BLE_CONN_HANDLE_INVALID) {
      batteryBleChar.notify8(batteryNotifyConnHdls[i], data);
    }
  }
}

void notifyBatteryPowerStateAllDevices(uint8_t data)
{
  for(uint8_t i=0;i<MAX_PRPH_CONNECTION;i++){
    if(batteryPowerStateNotifyConnHdls[i] != BLE_CONN_HANDLE_INVALID) {
      batteryBlePowerStateChar.notify8(batteryPowerStateNotifyConnHdls[i], data);
    }
  }
}

void digital_pushbutton_callback(void)
{
  uint8_t value;

  value = digitalRead(INT_PIN);
  Serial.print("Pin value: ");
  Serial.println(value);
  if(value) {
    BaseType_t active = xTimerIsTimerActive(tm_watchDogPowerOff.getHandle()); 
    if(active) {
      tm_watchDogPowerOff.reset();
    }
  }
}

void encoderEventCbk(void)
{
  Serial.print("DFLAG value: ");
  Serial.println(digitalRead(DFLAG_PIN));
}

void powerenableEventCbk(void)
{
  uint8_t value = digitalRead(PWREN_PIN);
  Serial.print("PWREN_PIN value: ");
  Serial.println(value);
  usbConnectedStatus = true;
  if(value == 0) {
    if(digitalRead(BCD_CHARGER_PIN) == 0) {
      usbConnectedStatus = false;
    }
  }
  
  //TODO: Notify about power State
  //batteryPowerState |= (usbConnectedStatus ? ); 
  //notifyBatteryPowerStateAllDevices(data);
}

void chargerEventCbk(void)
{
  uint8_t value = digitalRead(BCD_CHARGER_PIN);
  Serial.print("BCD_CHARGER_PIN value: ");
  Serial.println(value);
  if(value == 1) { //USB Power Enable
    usbPowerStatus = true;
    batteryDischargingStatus = false;
    //Disable PowerOFF Timer
    BaseType_t active = xTimerIsTimerActive(tm_watchDogPowerOff.getHandle()); 
    if(active) {
      tm_watchDogPowerOff.stop();
    }

  } else {
    usbPowerStatus = false;
    batteryDischargingStatus = true;
    //Enable PowerOff Timer
    tm_watchDogPowerOff.start();
  } 

  //Notify about power State
  batteryPowerState &= 0B11001111; //Clear Charging bits
  batteryPowerState |= (usbPowerStatus ? 0B00110000 : 0B00100000);
  batteryPowerState &= 0B11110011; //Clear Discharging bits
  batteryPowerState |= (batteryDischargingStatus ? 0B00001100 : 0B00001000);
  notifyBatteryPowerStateAllDevices(batteryPowerState);
}

void ledAvertisingInit(void)
{
  Serial.println("Init blinking!!");
  BaseType_t active = xTimerIsTimerActive(tm_blinkSignalLed.getHandle());
  
  if(!active) {
    tm_blinkSignalLed.start();
  }
  //Serial.println("Init blinking!!");
}

void ledAvertisingConnected(void)
{
  signalLed.setPixelColor(0, 0, 0, 255);
  signalLed.setBrightness(16);
  signalLed.show();    
}

void ledAdvertisingOff(void)
{
  signalLed.setPixelColor(0, 0, 0, 0);
  signalLed.setBrightness(255);
  signalLed.show();
}
