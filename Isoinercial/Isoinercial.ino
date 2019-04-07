#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <LS7366.h>
#include <SoftwareSerial.h> //TODO: Lets try hardware Serial1
#include <Adafruit_NeoPixel.h>


#define VBAT_MV_PER_LSB             (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER                (0.5F)          // 100K + 100K voltage divider on VBAT = (100K / (100K + 100K))
#define VBAT_DIVIDER_COMP           (2.0F)          // Compensation factor for the VBAT divider
#define ADC_SAMPLES                 (20)            // Number of samples in ADC read
#define VBAT_READ_INTERVAL          (300000)        // 300000 ms = 5 minutes read battery interval
#define CHRONOJUMP_SERIAL_INTERVAL  (1)             // 1ms interval Serial Output data to chronojump
/**
 * @brief PIN definitions
 * 
 */
#define VBAT_PIN                    (A6)            //Battery voltage measure -A6 in nrf52840 -A7 in nrf52832
#define SERIAL_TX_PIN               (A1)
#define SERIAL_RX_PIN               (A0)
#define LS7366_CS_PIN               (10)
#define NEOMATRIX_LED_PIN           (5)            // D2 Pin for Led Strip          

/**
 * @brief PIN DEFINITIONS 
 * #define PWREN_PIN                26            // P0.26 Output is LOW after the device has been configured by USB, then high during USB suspend mode. This output can be used to control power to external logic P-Channel logic level MOSFET switch. Enable the interface pull-down option when using the PWREN# in this way.  
 * #define BCD_CHARGER_PIN          4             // P0.04 Output is LOW when Battery Charge Detect, indicates when the device is connected to a dedicated battery charger. 
 * #define ANALOG_ENABLE_PIN        6             // P0.06 Put this PIN LOW to measure the battery voltage level. Keep voltage HIGH while not reading, then put LOW, read and come back to HIGH. If we missed to put high, the voltage divider disconnect in about 2ms.
 * #define LIPO_LEVEL_ANALOG_PIN    29            // P0.29 Voltage divider to read battery voltage
 * #define INT_PIN                  7             // P0.07 This PIN is input we must put Pull-UP. When The push button received a power-off signal go LOW. We can use to read short pulse signal.
 * #define DFLAG_PIN                40            // P1.08 This PIN goes LOW with every INDEX encoder count.
 * #define SS_LS7366_PIN            41            // P1.09 This is the Slave Select PIN for LS7366 chip
 * #define SERIAL_TX_PIN            
 * #define SERIAL_RX_PIN            
 * #define NEOPIX_PIN               16            // P0.16 RGB Led Data Pin.
 *    
 */

LS7366 myLS7366(LS7366_CS_PIN);  //10 is the chip select pin.
SoftwareSerial mySerial(SERIAL_RX_PIN, SERIAL_TX_PIN); // RX, TX
Adafruit_NeoPixel signalLed = Adafruit_NeoPixel(5, NEOMATRIX_LED_PIN, NEO_GRB + NEO_KHZ800);


// Application variables
uint32_t encoderPosition = 0, lastEncoderPosition = 0;
uint32_t timeelapsed = 0;
uint8_t lastBatteryVoltagePer = 0;
int8_t volatile IncEncoderData;
int batteryVoltageRaw;
bool isDeviceNotifyingEncoderData = false;
bool isDeviceNotifyingBatteryData = false;

//
SoftwareTimer readBatteryTimer;
SoftwareTimer encoderChronoJumpSerial;
//TaskHandle_t xHandleBatteryRead = NULL;   //Handler for Battery read Task;

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
  int raw;
  int rawVoltage;
 
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
 
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
 
  // Let the ADC settle
  delay(1);
  for(uint8_t i=0;i<numRead;i++) {
    // Get the raw 12-bit, 0..3000mV ADC value
    rawVoltage += analogRead(VBAT_PIN);
  }
  
  raw = rawVoltage / numRead;
  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  
  //Serial.println(raw);
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

  mySerial.write(IncEncoderData);
}

/**
 * @brief Convert battery voltage from Milivolts to Percentage 
 * 
 * @param mvolts 
 * @return uint8_t 
 */

uint8_t mvToPercent(float mvolts) {
    uint8_t battery_level;

    if(mvolts > 4200) {
        battery_level = 100;
    } else if (mvolts >= 4000)
    {
        battery_level = 100 - (4180 - mvolts) / 20;
    }
    else if (mvolts > 3900)
    {
        battery_level = 90 - (4000 - mvolts) / 10;
    }
    else if (mvolts > 3700)
    {
        battery_level = 80 - (3900 - mvolts) / 20;
    }
    else if (mvolts > 3500)
    {
        battery_level = 60 - (3700 - mvolts) / 20;
    }
    else if (mvolts > 3200)
    {
        battery_level = 40 - (3500 - mvolts) / 30;
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

void write_command(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  Serial.print("Received a value: ");
  Serial.println(data[0]);
}

/**
 * @brief Callback to handle connections 
 * 
 * @param conn_handle 
 */

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));
  
  // off Blue LED for lowest power consumption
  Bluefruit._setConnLed(false);
  Bluefruit.autoConnLed(false);
  Bluefruit._stopConnLed();
  //Bluefruit.setEventCallback()

  //Read battery Value
  batteryVoltageRaw = readVBAT(ADC_SAMPLES);
  Serial.print("Battery value: ");
  Serial.println(batteryVoltageRaw);
  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  // ON Blue LED for advertising indicate
  Bluefruit.autoConnLed(true);
  Bluefruit._startConnLed();

  Serial.print("Disconnected reason: ");
  Serial.println(reason);
  Serial.println("Advertising!");

  //TODO: Stop actives timers
  BaseType_t active = xTimerIsTimerActive(readBatteryTimer.getHandle());
  if(active) {
    readBatteryTimer.stop();
  }

}

void batteryReadCallback(BLECharacteristic& chr, ble_gatts_evt_read_t * request)
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

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");
    
    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr.uuid == encoderread.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Encoder Data Measurement 'Notify' enabled");
            isDeviceNotifyingEncoderData = true;
        } else {
            Serial.println("Encoder Data Measurement 'Notify' disabled");
            isDeviceNotifyingEncoderData = false;
        }
    } else if (chr.uuid == batteryBleChar.uuid) {
      if (chr.notifyEnabled()) {
            Serial.println("Battery Data Measurement 'Notify' enabled");
            isDeviceNotifyingBatteryData = true;
            readBatteryTimer.start(); //Enable a timer to read battery value and notify value
        } else {
            Serial.println("Battery Data Measurement 'Notify' disabled");
            isDeviceNotifyingBatteryData = false;
            readBatteryTimer.stop(); //Disable a timer to read battery value
            //NOTE: Disconnect dont disable the timer.
        } 
    }
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include Encoder 128-bit uuid
  Bluefruit.Advertising.addService(bleencoders);

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
}

void setup() {

  // configure D7 as input with a pullup (pin is active low)
  //pinMode(34, INPUT_PULLUP); ???? MISTAKE
  pinMode(LS7366_CS_PIN, OUTPUT);

  // Configure Signal LED
  signalLed.begin();
  signalLed.show(); // Initialize all pixels to 'off'
  signalLed.setPixelColor(0, 255, 0, 255);
  signalLed.setBrightness(16);
  signalLed.show();
  
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb
  mySerial.begin(115200);
  mySerial.println("Chronojump serial port enabled!!");

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
  int vbat_raw = readVBAT(ADC_SAMPLES);
 
  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);

  // Init Bluefruit
  Bluefruit.begin();

  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(100);
  
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(8);
  Bluefruit.setName("Isoinercial");

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  
  // Start the BLE Battery Service and set it to their value
  Serial.println("Configuring the Battery Service");
  blebas.begin();

  batteryBleChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY); // could support notify
  batteryBleChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryBleChar.setFixedLen(1);
  batteryBleChar.setCccdWriteCallback(cccd_callback);
  //batteryBleChar.setReadAuthorizeCallback(batteryReadCallback);
  batteryBleChar.begin();
  batteryBleChar.write8(vbat_per);
  
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
  encoderread.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  encoderread.begin();

  encoderwrite1.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  encoderwrite1.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  encoderwrite1.setUserDescriptor("Commands");
  encoderwrite1.setFixedLen(1);
  encoderwrite1.setWriteCallback(write_command);
  encoderwrite1.begin();

  encoderwrite2.setProperties(CHR_PROPS_WRITE);
  encoderwrite2.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  encoderwrite2.setUserDescriptor("Legacy");
  encoderwrite2.setFixedLen(1);
  encoderwrite2.begin();
 
  // Set up and start advertising
  startAdv();
  Bluefruit.printInfo();
  Serial.println("Advertising!");

  //Timer Creation related
  readBatteryTimer.begin(VBAT_READ_INTERVAL,read_battery_callback);
  encoderChronoJumpSerial.begin(CHRONOJUMP_SERIAL_INTERVAL, serial_chronojump_callback);
  encoderChronoJumpSerial.start();
  
  //Task Creation related
  //BaseType_t xReturned;

  /* Create the task, storing the handle. */
  
  //xReturned = xTaskCreate(
  //                vTaskCode,       /* Function that implements the task. */
  //                "READ_BAT",          /* Text name for the task. */
  //                STACK_SIZE,      /* Stack size in words, not bytes. */
  //                ( void * ) 1,    /* Parameter passed into the task. */
  //                tskIDLE_PRIORITY,/* Priority at which the task is created. */
  //                &xHandleBatteryRead );      /* Used to pass out the created task's handle. */

  //  if( xReturned == pdPASS ) {
      /* The task was created.  Use the task's handle to pause the task. */
  //    vTaskSuspend( xHandleBatteryRead );
  //  }

  
}

void loop() {

  timeelapsed = millis();
  uint32_t dummy_data[2] = {timeelapsed , encoderPosition};
  // Convert from raw mv to percentage (based on LIPO chemistry)
  
  uint8_t batteryVoltagePer = mvToPercent(batteryVoltageRaw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP);

  //Serial.println(batteryVoltagePer);

  if(batteryVoltagePer != lastBatteryVoltagePer) {
      Serial.println("Battery value changed!");
      batteryBleChar.write8(batteryVoltagePer);
      
  }

  encoderPosition = myLS7366.read_counter();
  IncEncoderData = encoderPosition - lastEncoderPosition;

  if (!Bluefruit.Advertising.isRunning()) {
    
    //Encoder related changes of data
    if(isDeviceNotifyingEncoderData) { 
      if(encoderPosition != lastEncoderPosition) {          
        encoderread.notify(dummy_data,sizeof(dummy_data));
      }
    }

    //Battery related changes of data
    if(batteryVoltagePer != lastBatteryVoltagePer) {
      if(isDeviceNotifyingBatteryData) {
        batteryBleChar.notify8(batteryVoltagePer);
      }
    } 
  }

  lastEncoderPosition = encoderPosition;
  lastBatteryVoltagePer = batteryVoltagePer;

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

int32_t fn(uint32_t time, float period, uint16_t heigth)
{
  
  float iteration = fmod((float) time , period);
  float angle = ((iteration / period) * 2 * PI) + PI;
  float result = ((cos(angle) + 1) / 2) * heigth;
  
  return result;
}

