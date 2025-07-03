// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp_app_builder.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
// #include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;

// Global definitions
const float kAnalogInputScale = 29. / 2.048; // ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
const float kMeasurementCurrent = 0.01; // Engine Hat constant measurement current (A)
const uint sensor_read_interval = 1000; // ms, the delay / interval to read the sensors, used for all sensors in this program

// Pin definitions
#define ONEWIRE_PIN 4 // 1-Wire data pin on SH-ESP32
const int kSDAPin = 16; // I2C pin on SH-ESP32
const int kSCLPin = 17; // I2C pin on SH-ESP32
const int kADS1115Address = 0x4b; // ADS1115 I2C address
const int kCANRxPin = GPIO_NUM_34; // CAN bus (NMEA 2000) pin on SH-ESP32
const int kCANTxPin = GPIO_NUM_32; // CAN bus (NMEA 2000) pin on SH-ESP32
const int kDigitalInputPin1 = GPIO_NUM_15; // Engine hat digital input pin
const int kDigitalInputPin2 = GPIO_NUM_13; // Engine hat digital input pin
const int kDigitalInputPin3 = GPIO_NUM_14; // Engine hat digital input pin
const int kDigitalInputPin4 = GPIO_NUM_12; // Engine hat digital input pin

// Specific defintions per sensor
// - config_* is the configuration path (see SensESP concepts documentations) that is shown (and editable at runtime) in the UI of the Engine Hat at http://enginehat.local
// - value_* is the default setting for a sensor configuration (also available through engine hat UI)
// - sk_path_* is the signalk path that the sensor reading is assigned to; this will show up in the signalk-server Data Browser.
// - SKMetadata is SignalK metadata parameters: unit of measure, display name, description, short name, timeout in secs

// Definitions: Engine RPM (W-terminal on alternator)
const char* config_engine1_freq_multiplier = "/engine1/tach/frequency multiplier"; // the alternator report a frequency that needs to be multiplied/divided to represent the actual RPM.
const char* config_engine2_freq_multiplier = "/engine2/tach/frequency multiplier"; // define a virtual second engine to define alternative parameters, see moving avg 2.
const char* config_engine1_moving_avg = "/engine1/tach/moving avg samples"; // the rpm might 'jump' around, use this number of samples to calculate a smooth rpm.
const char* config_engine2_moving_avg = "/engine2/tach/moving avg samples"; // the first moving_avg might be slow, configure a faster (but more jumpy moving avg on a virtual second engine).
const char* config_engine1_revolutions_sk_path = "/engine1/tach/revolutions/sk_path";
const char* config_engine2_revolutions_sk_path = "/engine2/tach/revolutions/sk_path";
auto metadata_engine_tach_revolutions = new SKMetadata("Hz", "Revolutions", "Revolutions per second", "Revolutions", 3);
const char* value_engine1_tach_revolutions_sk_path = "propulsion.1.revolutions";
const char* value_engine2_tach_revolutions_sk_path = "propulsion.2.revolutions";
const float value_engine_freq_multiplier = 1. / 9.5238; // 2022-08-26: Multiplier for Volvo Penta MD2020 is 0.105, so 1/ 9.5238
const float value_engine1_moving_avg = 15;
const float value_engine2_moving_avg = 2;

//auto engine1_tacho_frequency = new Frequency(value_engine1_freq_multiplier, config_engine1_freq_multiplier);
//auto engine1_tacho_mov_avg = new MovingAverage(value_engine1_moving_avg, 1.0, config_engine1_moving_avg);
// TODO: check if SKMetadata might interfere with stuff. Otherwise remove this line.
//auto engine1_tacho_frequency_sk_output = new SKOutputFloat(value_engine1_tach_revolutions_sk_path, config_engine1_revolutions_sk_path,metadata_engine1_tach_revolutions);

// Definitions: 1-wire sensors
const char* config_onewire_1_address = "/onewire/1/address"; 
const char* config_onewire_2_address = "/oneWire/2/address"; 
const char* config_onewire_3_address = "/oneWire/3/address";
const char* config_onewire_1_sk_path = "/onewire/1/signalk path";
const char* config_onewire_2_sk_path = "/oneWire/2/signalk path";
const char* config_onewire_3_sk_path = "/oneWire/3/signalk path";
auto onewire_3_metadata = new SKMetadata("K", "Engine Oil Temperature", "Engine Oil Temperature", "Oil Temperature", 10);
auto onewire_2_metadata = new SKMetadata("K", "Engine Room Temperature", "Engine Room Temperature", "Engine Room Temperature", 10);
auto onewire_1_metadata = new SKMetadata("K", "Cabin Room Temperature", "Cabin Room Temperature; also used for outside temperature", "Cabin Room Temperature", 10);
const char* value_onewire_3_sk_path = "propulsion.1.oilTemperature";
const char* value_onewire_2_sk_path = "environment.inside.engineRoom.temperature";
const char* value_onewire_1_sk_path = "environment.outside.temperature";

// Definition: Engine temperature sender
uint8_t enginehat_pin_temp = 0; // attached to pin A on the Engine Hat.
const char* config_engine1_temp_resistance_sk_path = "/engine1/temp/resistance/sk_path"; // the sk_path of the resistance of the temp sender in the engine.
const char* config_engine1_temp_temperature_sk_path = "/engine1/temp/temperature/sk_path"; // the sk_path of the temperature of the temp sender in the engine.
const char* config_engine1_temp_level_curve = "/engine1/temp/Level Curve";
const char* value_engine1_temp_resistance_sk_path = "propulsion.1.temperature.senderResistance";
const char* value_engine1_temp_temperature_sk_path = "propulsion.1.temperature";
auto metadata_engine1_temp_resistance = new SKMetadata("ohm", "Resistance", "Measured resistance of temperature sender", "Resistance", 10);
auto metadata_engine1_temp_temperature = new SKMetadata("K", "Engine temperature", "Temperature of cooling water sender in engine", "Temperature", 10);

// Definition: Engine temperature alarm (relay)
const char* config_engine1_temp_alarm_sk_path = "/engine1/temp/alarm/sk_path";
const char* value_engine1_temp_alarm_sk_path = "notifications.propulsion.1.overTemperature";

// Definition: Engine oil pressure sender
// Definition: Engine temperature sender
uint8_t enginehat_pin_oil = 1; // attached to pin B on the Engine Hat.
const char* config_engine1_oil_resistance_sk_path = "/engine1/oil/resistance/sk_path"; // the sk_path of the resistance of the temp sender in the engine.
const char* config_engine1_oil_pressure_sk_path = "/engine1/oil/pressure/sk_path"; // the sk_path of the temperature of the temp sender in the engine.
const char* config_engine1_oil_pressure_curve = "/engine1/oil/Level Pressure";
const char* value_engine1_oil_resistance_sk_path = "propulsion.1.oil.senderResistance";
const char* value_engine1_oil_pressure_sk_path = "propulsion.1.oilPressure";
auto metadata_engine1_oil_resistance = new SKMetadata("ohm", "Resistance", "Measured resistance of oil sender", "Resistance", 10);
auto metadata_engine1_oil_pressure = new SKMetadata("Pa", "Engine Oil Pressure", "Oil pressure in engine", "Pressure", 10);

// Definition: Engine oil pressure alarm (relay)
const char* config_engine1_oil_alarm_sk_path = "/engine1/oil/alarm/sk_path";
const char* value_engine1_oil_alarm_sk_path = "notifications.propulsion.1.lowOilPressure";

// Definition: Fuel tank sender
// This is for  VDO 145 Fuel lever arm sender adjustable 3-180 Ohm (empty-full)), tank capacity is 80l
uint8_t enginehat_pin_fuel = 2; // attached to pin C on the Engine Hat.
const float tank_fuel1_capacity = 80. / 1000; // in m3, divide by 1000 to get from litres to m3), fuel = 80l, water = 170l
const char* config_tank1_resistance_sk_path = "/tank/fuel1/resistance/sk_path"; // the sk_path of the resistance of the level sender in the tank.
const char* config_tank1_capacity_sk_path = "/tank/fuel1/capacity/sk_path"; // the sk_path of the total volume (in m3) of the tank
const char* config_tank1_remaining_sk_path = "/tank/fuel1/remaining/sk_path"; // the sk_path of the remaining volume (in m3) of the tank
const char* config_tank1_level_sk_path = "/tank/fuel1/level/sk_path"; // the sk_path of the level of fuel (%, ratio) in the tank.
const char* config_tank1_level_curve = "/tank/fuel1/Level Curve";
const char* config_tank1_moving_avg = "/tank/fuel1/moving avg samples"; // the fuel level might 'jump' around, use this number of samples to calculate a smooth fuel level.
const float value_tank1_moving_avg = 3;
const char* value_tank1_resistance_sk_path = "tanks.fuel.1.senderResistance";
const char* value_tank1_capacity_sk_path = "tanks.fuel.1.capacity";
const char* value_tank1_remaining_sk_path = "tanks.fuel.1.currentVolume";
const char* value_tank1_level_sk_path = "tanks.fuel.1.currentLevel";
auto metadata_tank1_level_resistance = new SKMetadata("ohm", "Resistance", "Measured resistance of fuel tank sender", "Resistance", 10);
auto metadata_tank1_capacity = new SKMetadata("m3", "Total volume", "The total volume (capacity) of the fuel tank (m3)", "Capacity", 10);
auto metadata_tank1_remaining = new SKMetadata("m3", "Remaining volume", "Remaining volume of fuel in tank (m3)", "Level", 10);
auto metadata_tank1_level = new SKMetadata("ratio", "Current level", "Current level of fuel in tank (%)", "Level", 10);

// Definition: BME280 sensors (temperature, barometric pressure, relative humidy)
Adafruit_BME280 bme280;
TwoWire* i2c;

// TODO: moet nog gefixed wordem, dit levert null op. Zie ook calls onderaan in setup()
float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
float read_pressure_callback() { return (bme280.readPressure());}
float read_humidity_callback() { return (bme280.readHumidity());}

// TODO: Ook nog onewires aansluiten.

// Convenience: print the addresses found on the I2C bus
void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("sh-esp32-engine-hat")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

 // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);
  ScanI2C(i2c);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

// Activate the 1-wires senders
  // DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);
  // auto onewire_1_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_1_address);
  // auto onewire_2_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_2_address);
  // auto onewire_3_temp = new OneWireTemperature(dts, sensor_read_interval, config_onewire_3_address);
  // onewire_1_temp->connect_to(new SKOutput<float>(value_onewire_1_sk_path, config_onewire_1_sk_path, onewire_1_metadata)); 
  // onewire_2_temp->connect_to(new SKOutput<float>(value_onewire_2_sk_path, config_onewire_2_sk_path, onewire_2_metadata));
  // onewire_3_temp->connect_to(new SKOutput<float>(value_onewire_3_sk_path, config_onewire_3_sk_path, onewire_3_metadata));



  // Temperature sensor setup
  // Could be done better using the example for sensesp 3.x: https://github.com/SignalK/SensESP/blob/main/examples/temperature_sender.cpp
  // Now it is not configurable in the web IO.
  auto engine1_temp_sender_resistance = new RepeatSensor<float>(sensor_read_interval, [ads1115]() {
    int16_t adc_output = ads1115->readADC_SingleEnded(enginehat_pin_temp);  
    float adc_output_volts = ads1115->computeVolts(adc_output);
    return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
  });
  auto temp_curve = (new CurveInterpolator(nullptr, config_engine1_temp_level_curve))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Temp Level (ratio)");
  if (temp_curve->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    temp_curve->clear_samples();
    temp_curve->add_sample(CurveInterpolator::Sample(800, 273.15)); // 0C, K is C + 273.15
    temp_curve->add_sample(CurveInterpolator::Sample(760, 288.15)); // 15C
    temp_curve->add_sample(CurveInterpolator::Sample(484, 293.15)); // 20C - this table taken from: https://www.dedieseldokter.nl/c-4226659/koelwater-sensor-controleren/
    temp_curve->add_sample(CurveInterpolator::Sample(304, 303.15)); // 30C
    temp_curve->add_sample(CurveInterpolator::Sample(209, 313.15)); // 40C
    temp_curve->add_sample(CurveInterpolator::Sample(149, 323.15)); // 50C
    temp_curve->add_sample(CurveInterpolator::Sample(104, 333.15)); // 60C
    temp_curve->add_sample(CurveInterpolator::Sample(79, 343.15)); // 70C
    temp_curve->add_sample(CurveInterpolator::Sample(60, 353.15)); // 80C  - aangepast van 72 naar 60, smoother curve
    temp_curve->add_sample(CurveInterpolator::Sample(40, 363.15)); // 90C
    temp_curve->add_sample(CurveInterpolator::Sample(29, 368.15));  // 95C
    temp_curve->add_sample(CurveInterpolator::Sample(23, 373.15));  // 100C - added, not measured, also not in spec, just guessed
    temp_curve->add_sample(CurveInterpolator::Sample(0, 393.15));  // 120C - added, not measured, also not in spec, just guessed
  }
engine1_temp_sender_resistance->connect_to(new SKOutputFloat(value_engine1_temp_resistance_sk_path, config_engine1_temp_resistance_sk_path, metadata_engine1_temp_resistance));
engine1_temp_sender_resistance->connect_to(temp_curve)->connect_to(new SKOutputFloat(value_engine1_temp_temperature_sk_path, config_engine1_temp_temperature_sk_path, metadata_engine1_temp_temperature));


// ===== Engine Oil Pressure Sender =====
  // 10 - 184 ohm = 0 tot 10 bar
  auto engine1_oil_sender_resistance = new RepeatSensor<float>(sensor_read_interval, [ads1115]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(enginehat_pin_oil);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });
  //auto engine1_temp_sender_resistance_sk_output = new SKOutputFloat(value_engine1_temp_resistance_sk_path, config_engine1_temp_temperature_sk_path, metadata_engine1_temp_resistance);
  auto oil_curve = (new CurveInterpolator(nullptr, config_engine1_oil_pressure_curve))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Oil Pressure Curve (ratio)");
  if (oil_curve->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    oil_curve->clear_samples();
    oil_curve->add_sample(CurveInterpolator::Sample(0, 0)); 
    oil_curve->add_sample(CurveInterpolator::Sample(10, 0)); // KUS oliedrukmete is 10-184 ohm, wat staat voor 0 - 10 bar.
    oil_curve->add_sample(CurveInterpolator::Sample(184, 100000000)); // 100 kPa = 1 bar, 1000kPa = 10 bar. 
    oil_curve->add_sample(CurveInterpolator::Sample(300, 100000000)); // Added 2 0's, than it works with signalk-to-n2k plugin
  }
  engine1_oil_sender_resistance->connect_to(new SKOutputFloat(value_engine1_oil_resistance_sk_path, config_engine1_oil_resistance_sk_path, metadata_engine1_oil_resistance));
  engine1_oil_sender_resistance->connect_to(oil_curve)->connect_to(new SKOutputFloat(value_engine1_oil_pressure_sk_path, config_engine1_oil_pressure_sk_path, metadata_engine1_oil_pressure));

// ===== Fuel Tank Sender =====
  auto tank_fuel1_sender_resistance = new RepeatSensor<float>(sensor_read_interval, [ads1115]() {
      int16_t adc_output = ads1115->readADC_SingleEnded(enginehat_pin_fuel);
      float adc_output_volts = ads1115->computeVolts(adc_output);
      return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
    });
  //auto tank_fuel1_sender_resistance_sk_output = new SKOutputFloat(value_tank1_resistance_sk_path, config_tank1_resistance_sk_path, metadata_tank1_level_resistance);
  auto fuel_curve = (new CurveInterpolator(nullptr, config_tank1_level_curve))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Fuel Level (ratio)");
  if (fuel_curve->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    fuel_curve->clear_samples();
    fuel_curve->add_sample(CurveInterpolator::Sample(0, 0)); // Empty (the range of similar VDO meter is 3 - 180 ohm)
    fuel_curve->add_sample(CurveInterpolator::Sample(350., 0.6));
    fuel_curve->add_sample(CurveInterpolator::Sample(400., 0.8));  // Full. (in practice i use 180 = 0.4 and 500 = 1, through manual config screen).
    fuel_curve->add_sample(CurveInterpolator::Sample(500., 1));  // Range of my pot meter for tests.
  }
  // 2023-10-01: 375 ohm na toevoegen 18 liter. Zit nu 90-95% vol.
  // 400= 0.850000024 -> heb ik 1 van gemaakt, scherm staat nu op 90% (was 80%)
  // 2023-09-16: 370 ohm = 80% full, change manual configured from x -> y:
  // 350=0.45 -> 0.70
  // 400=0.6 -> 0.85

  auto tank_fuel1_volume = new Linear(tank_fuel1_capacity, 0, config_tank1_capacity_sk_path);
  new SKOutputFloat(value_tank1_capacity_sk_path, config_tank1_capacity_sk_path, metadata_tank1_capacity);

  tank_fuel1_sender_resistance
    ->connect_to(new SKOutputFloat(value_tank1_resistance_sk_path, config_tank1_resistance_sk_path, metadata_tank1_level_resistance));

// tank_fuel1_sender_resistance
//       ->connect_to(fuel_curve)
//       ->connect_to(new MovingAverage(value_tank1_moving_avg, 1.0, config_tank1_moving_avg))
//       ->connect_to(new SKOutputFloat(value_tank1_level_sk_path, config_tank1_level_sk_path, metadata_tank1_level));
//   fuel_curve
//       ->connect_to(tank_fuel1_volume)
//       ->connect_to(new MovingAverage(value_tank1_moving_avg, 1.0, config_tank1_moving_avg))
//       ->connect_to(new SKOutputFloat(value_tank1_remaining_sk_path, config_tank1_remaining_sk_path, metadata_tank1_remaining));
  

  tank_fuel1_sender_resistance
      ->connect_to(fuel_curve)
      ->connect_to(new SKOutputFloat(value_tank1_level_sk_path, config_tank1_level_sk_path, metadata_tank1_level));
  fuel_curve
      ->connect_to(tank_fuel1_volume)
      ->connect_to(new SKOutputFloat(value_tank1_remaining_sk_path, config_tank1_remaining_sk_path, metadata_tank1_remaining));
  

  // ===== Engine Temperature Alarm/Relay =====
  auto* alarm_input = new DigitalInputChange(kDigitalInputPin1, INPUT, CHANGE);
  auto jsonify = new LambdaTransform<bool, String>([](bool input) -> String {
    DynamicJsonDocument doc(1024);
    if (input == true) {
      JsonArray alarm_methods = doc.createNestedArray("method");
      alarm_methods.add("visual");
      alarm_methods.add("sound");
      doc["state"] = "alarm";
      doc["message"] = "Engine overheating";
    } 
    String output;
    serializeJson(doc, output);
    return output;
  });
  alarm_input->connect_to(jsonify);
  jsonify->connect_to(new SKOutputRawJson(value_engine1_temp_alarm_sk_path, config_engine1_temp_alarm_sk_path));


  // ===== Engine Tacho (RPM) Sender (W-terminal on alternator) =====
  auto engine1_tacho_sender = new DigitalInputCounter(kDigitalInputPin2, INPUT, RISING, 500, "");
  engine1_tacho_sender
    ->connect_to(new Frequency(value_engine_freq_multiplier, config_engine1_freq_multiplier))
    ->connect_to(new MovingAverage(value_engine1_moving_avg, 1.0, config_engine1_moving_avg))
    ->connect_to(new SKOutputFloat(value_engine1_tach_revolutions_sk_path, config_engine1_revolutions_sk_path,metadata_engine_tach_revolutions));

   engine1_tacho_sender
    ->connect_to(new Frequency(value_engine_freq_multiplier, config_engine2_freq_multiplier))
    ->connect_to(new MovingAverage(value_engine2_moving_avg, 1.0, config_engine2_moving_avg))
    ->connect_to(new SKOutputFloat(value_engine2_tach_revolutions_sk_path, config_engine2_revolutions_sk_path,metadata_engine_tach_revolutions));



  

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }


