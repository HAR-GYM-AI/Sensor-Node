/**
 * HAR System - Sensor Node (Arduino Nano 33 IoT)
 * 
 * This code implements a wearable sensor node for Human Activity Recognition
 * Features:
 * - LSM6DS3 IMU data collection (accelerometer + gyroscope)
 * - Adafruit AHRS Madgwick filter for sensor fusion (quaternion output)
 * - BLE communication with Raspberry Pi
 * - 100Hz sampling rate with hardware timer
 * - Calibration support
 * - Circular buffer for transmission recovery
 * 
 * Placement: Wrist, Bicep, Chest, or Thigh
 * 
 * Library Required: Adafruit AHRS (install via Arduino Library Manager) + BLE
 * I think we should do a script but I could not figure it out on my desktop
 */

#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <Adafruit_AHRS.h>

// CONFIG

// OPERATING MODE - CHANGE THIS FOR TESTING vs DEPLOYMENT
#define DEBUG_MODE true  // true = USB debugging with serial output
                         // false = standalone mode (no USB, for deployment)

// Node Configuration - CHANGE THIS FOR EACH DEVICE
#define NODE_ID 1  // 0=Wrist, 1=Bicep, 2=Chest, 3=Thigh

// Sampling Configuration
#define SAMPLING_RATE_HZ 100
#define SAMPLING_PERIOD_MS 4  // 1000ms / 208Hz ≈ 4.8ms, use 4ms for the software timer
#define BUFFER_SIZE 20

// IMU Configuration
#define ACCEL_RANGE 8   // ±8g
#define GYRO_RANGE 500  // ±500 dps


// Sliding Window Configuration
#define SHORT_WINDOW_SIZE 150      // 1.5s @ 100Hz for classification
#define SHORT_WINDOW_OVERLAP 0.75  // 75% overlap
#define SHORT_WINDOW_STEP 38       // 150 * (1 - 0.75) = 37.5 ≈ 38 samples
#define LONG_WINDOW_SIZE 300       // 3.0s @ 100Hz for rep counting
#define LONG_WINDOW_OVERLAP 0.5    // 50% overlap
#define LONG_WINDOW_STEP 150       // 300 * (1 - 0.5) = 150 samples



// BLE Configuration
#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define ORIENTATION_CHAR_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define CONTROL_CHAR_UUID   "19B10002-E8F2-537E-4F6C-D104768A1214"
#define FEATURES_SHORT_CHAR_UUID "19B10003-E8F2-537E-4F6C-D104768A1214"
#define FEATURES_LONG_CHAR_UUID  "19B10004-E8F2-537E-4F6C-D104768A1214"


// GLOBAL VARIABLES

// BLE Service and Characteristics
BLEService sensorService(SERVICE_UUID);
BLECharacteristic orientationChar(ORIENTATION_CHAR_UUID, BLERead | BLENotify, 20);
BLECharacteristic controlChar(CONTROL_CHAR_UUID, BLERead | BLEWrite | BLENotify, 20);
BLECharacteristic featuresShortChar(FEATURES_SHORT_CHAR_UUID, BLERead | BLENotify, 20);
BLECharacteristic featuresLongChar(FEATURES_LONG_CHAR_UUID, BLERead | BLENotify, 20);

// Adafruit AHRS Madgwick Filter
Adafruit_Madgwick filter;

// Timing
unsigned long lastSampleTime = 0;
unsigned long systemStartTime = 0;
uint16_t timestamp = 0;  // 2-byte timestamp in 10ms ticks
uint8_t sequenceNumber = 0;

// IMU Data
float ax, ay, az;  // Accelerometer (g)
float gx, gy, gz;  // Gyroscope (dps)

// Calibration Offsets
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;
bool isCalibrated = false;

// Quaternion Output
float qw, qx, qy, qz;

// Sliding Window Buffers
struct QuaternionSample {
  uint16_t timestamp;
  float qw, qx, qy, qz;
};


QuaternionSample shortWindowBuffer[SHORT_WINDOW_SIZE];
QuaternionSample longWindowBuffer[LONG_WINDOW_SIZE];
uint16_t shortWindowIndex = 0;
uint16_t longWindowIndex = 0;
uint16_t shortWindowSampleCount = 0;
uint16_t longWindowSampleCount = 0;
uint16_t samplesSinceLastShortWindow = 0;
uint16_t samplesSinceLastLongWindow = 0;


// Feature Extraction Results
struct WindowFeatures {
  uint8_t windowType;      // 0=short, 1=long
  uint8_t nodeId;
  uint16_t windowId;
  float qw_mean;
  float qx_mean;
  float qy_mean;
  float qz_mean;
  float qw_std;
  float qx_std;
  float qy_std;
  float qz_std;
  float sma;               // Signal Magnitude Area
  float dominantFreq;      // Dominant frequency component
};


uint16_t shortWindowId = 0;
uint16_t longWindowId = 0;


// Circular Buffer for BLE Recovery
struct SensorData {
  uint8_t seqNum;
  uint16_t timestamp;
  float qw, qx, qy, qz;
};
SensorData dataBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

// System State
bool isRunning = false;
bool bleConnected = false;
bool enableWindowing = true;  // Enable/disable windowing mode

// PACKET STRUCTURE

struct OrientationPacket {
  uint8_t sequenceNumber;  // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t timestamp;      // 2 bytes
  float qw;                // 4 bytes
  float qx;                // 4 bytes
  float qy;                // 4 bytes
  float qz;                // 4 bytes
  // Total: 20 bytes
};

struct FeaturePacket {
  uint8_t windowType;      // 1 byte (0=short, 1=long)
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float qw_mean;           // 4 bytes
  float qx_mean;           // 4 bytes
  float qy_mean;           // 4 bytes
  float qz_mean;           // 4 bytes
  // Total: 20 bytes (first packet)
};

struct FeaturePacket2 {
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float qw_std;            // 4 bytes
  float qx_std;            // 4 bytes
  float qy_std;            // 4 bytes
  float qz_std;            // 4 bytes
  // Total: 20 bytes (second packet)
};

struct FeaturePacket3 {
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float sma;               // 4 bytes
  float dominantFreq;      // 4 bytes
  uint8_t padding[8];      // 8 bytes padding
  // Total: 20 bytes (third packet)
};


// SETUP

void setup() {
  #if DEBUG_MODE
    Serial.begin(115200);
    // Wait for serial connection in debug mode
    while (!Serial);
    delay(1000);
    
    Serial.println("========================================");
    Serial.println("HAR Sensor Node - DEBUG MODE");
    Serial.println("========================================");
    Serial.print("Node ID: ");
    Serial.println(NODE_ID);
    Serial.print(SHORT_WINDOW_SIZE);
    Serial.print(" samples (");
    Serial.print(SHORT_WINDOW_SIZE * 10);
    Serial.print("ms) with ");
    Serial.print(SHORT_WINDOW_OVERLAP * 100);
    Serial.println("% overlap");
    Serial.print("Long Window: ");
    Serial.print(LONG_WINDOW_SIZE);
    Serial.print(" samples (");
    Serial.print(LONG_WINDOW_SIZE * 10);
    Serial.print("ms) with ");
    Serial.print(LONG_WINDOW_OVERLAP * 100);
    Serial.println("% overlap");
  #else
    // In standalone mode, still initialize serial but don't wait
    Serial.begin(115200);
    delay(100);
  #endif
  
  // Initialize IMU
  if (!IMU.begin()) {
    #if DEBUG_MODE
      Serial.println("Failed to initialize IMU!");
    #endif
    while (1);
  }

  
  #if DEBUG_MODE
    Serial.println("IMU initialized successfully");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in g's");
    Serial.println("X\tY\tZ");
  #endif
  
  // Initialize Adafruit AHRS Madgwick Filter
  filter.begin(SAMPLING_RATE_HZ);
  
  #if DEBUG_MODE
    Serial.println("AHRS Filter initialized");
  #endif
  
  // Initialize BLE
  if (!BLE.begin()) {
    #if DEBUG_MODE
      Serial.println("Failed to initialize BLE!");
    #endif
    while (1);
  }
  
  // Set up BLE
  setupBLE();
  
  #if DEBUG_MODE
    Serial.println("System ready. Waiting for BLE connection...");
    Serial.println("========================================");
    Serial.println();
    printDebugMenu();
  #endif
}

// MAIN LOOP
void loop() {
  #if DEBUG_MODE
    // Check for serial commands in debug mode
    handleSerialCommand();
  #endif
  
  // Wait for BLE central to connect
  BLEDevice central = BLE.central();
  
  if (central) {
    #if DEBUG_MODE
      Serial.println();
      Serial.print("Connected to central: ");
      Serial.println(central.address());
      Serial.println("Starting data stream...");
      Serial.println();
    #endif
    
    bleConnected = true;
    
    // Reset timing when connection established
    systemStartTime = millis();
    lastSampleTime = millis();
    timestamp = 0;
    sequenceNumber = 0;
    
    #if DEBUG_MODE
      // In debug mode, automatically start data collection
      isRunning = true;
      Serial.println("Auto-started data collection (DEBUG MODE)");
    #endif
    
    while (central.connected()) {
      BLE.poll();

      // Check for control commands
      if (controlChar.written()) {
        handleControlCommand();
      }
      
      // Sample and transmit data if running
      if (isRunning) {
        unsigned long currentTime = millis();
        
        // Check if it's time for next sample (100Hz = 10ms period)
        if (currentTime - lastSampleTime >= SAMPLING_PERIOD_MS) {
          lastSampleTime = currentTime;
          
          // Read IMU and process
          if (readIMU()) {
            applyCalibration();
            updateOrientation();
            if (enableWindowing) {
              // Add to sliding windows
              addToWindows();
              
              // Check if windows are ready for feature extraction
              if (samplesSinceLastShortWindow >= SHORT_WINDOW_STEP && shortWindowSampleCount >= SHORT_WINDOW_SIZE) {
                extractAndTransmitFeatures(0);  // Short window (classification)
                samplesSinceLastShortWindow = 0;
                shortWindowId++;
              }
              
              if (samplesSinceLastLongWindow >= LONG_WINDOW_STEP && longWindowSampleCount >= LONG_WINDOW_SIZE) {
                extractAndTransmitFeatures(1);  // Long window (rep counting)
                samplesSinceLastLongWindow = 0;
                longWindowId++;
              }
            } else {
              storeInBuffer();
              transmitData();
            }
            
            #if DEBUG_MODE
              // In debug mode, print detailed packet info every 50 samples (0.5 sec)
              if (sequenceNumber % 50 == 0) {
                printDetailedPacketInfo();
              }
            #endif
          } else {
            #if DEBUG_MODE
              Serial.println("Warning: IMU read failed");
            #endif
          }
          
          // Increment timestamp (in 10ms ticks)
          timestamp++;
        }
      }
    }
    
    #if DEBUG_MODE
      Serial.println();
      Serial.println("Disconnected from central");
    #endif
    
    bleConnected = false;
    isRunning = false;
  }
  
  // Small delay to prevent tight loop when not connected
  delay(100);
}

// BLE SETUP

void setupBLE() {
  // Set device name
  String deviceName = "HAR_Node_" + String(NODE_ID);
  BLE.setLocalName(deviceName.c_str());
  BLE.setDeviceName(deviceName.c_str());
  
  // Set advertised service
  BLE.setAdvertisedService(sensorService);
  
  // Add characteristics to service
  sensorService.addCharacteristic(orientationChar);
  sensorService.addCharacteristic(controlChar);
  sensorService.addCharacteristic(featuresShortChar);
  sensorService.addCharacteristic(featuresLongChar);
  
  // Add service
  BLE.addService(sensorService);
  
  // Start advertising
  BLE.advertise();
  
  #if DEBUG_MODE
    Serial.println("BLE advertising started");
    Serial.println("Device name: " + deviceName);
    Serial.println("Service UUID: " SERVICE_UUID);
  #endif
}

// IMU FUNCTIONS
bool readIMU() {
  // Check if IMU data is available
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    return false;
  }
  
  // Read accelerometer (g)
  IMU.readAcceleration(ax, ay, az);
  
  // Read gyroscope (dps)
  IMU.readGyroscope(gx, gy, gz);
  
  return true;
}

void applyCalibration() {
  // Apply gyroscope bias correction
  if (isCalibrated) {
    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;
  }
}

void updateOrientation() {
  // Update Adafruit AHRS filter with gyro (dps) and accel (g)
  // Note: Adafruit AHRS expects gyro in radians/sec, so convert from dps
  float gx_rad = gx * DEG_TO_RAD;
  float gy_rad = gy * DEG_TO_RAD;
  float gz_rad = gz * DEG_TO_RAD;
  
  // Update filter (6-axis: gyro + accel, no magnetometer)
  filter.update(gx_rad, gy_rad, gz_rad, ax, ay, az, 0, 0, 0);
  
  // Get quaternion output - getQuaternion() fills the provided variables
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  
  // The Adafruit library already normalizes quaternions internally,
  // but we'll add a safety check
  float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (norm > 0.0001 && abs(norm - 1.0) > 0.01) {  // Only normalize if needed
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
  }
}

// CALIBRATION

// SLIDING WINDOW FUNCTIONS

void resetWindows() {
  shortWindowIndex = 0;
  longWindowIndex = 0;
  shortWindowSampleCount = 0;
  longWindowSampleCount = 0;
  samplesSinceLastShortWindow = 0;
  samplesSinceLastLongWindow = 0;
  shortWindowId = 0;
  longWindowId = 0;
  
  #if DEBUG_MODE
    Serial.println("Sliding windows reset");
  #endif
}

void addToWindows() {
  // Add current quaternion sample to both windows
  QuaternionSample sample;
  sample.timestamp = timestamp;
  sample.qw = qw;
  sample.qx = qx;
  sample.qy = qy;
  sample.qz = qz;
  
  // Add to short window (circular buffer)
  shortWindowBuffer[shortWindowIndex] = sample;
  shortWindowIndex = (shortWindowIndex + 1) % SHORT_WINDOW_SIZE;
  if (shortWindowSampleCount < SHORT_WINDOW_SIZE) {
    shortWindowSampleCount++;
  }
  samplesSinceLastShortWindow++;
  
  // Add to long window (circular buffer)
  longWindowBuffer[longWindowIndex] = sample;
  longWindowIndex = (longWindowIndex + 1) % LONG_WINDOW_SIZE;
  if (longWindowSampleCount < LONG_WINDOW_SIZE) {
    longWindowSampleCount++;
  }
  samplesSinceLastLongWindow++;
}

void extractAndTransmitFeatures(uint8_t windowType) {
  WindowFeatures features;
  features.windowType = windowType;
  features.nodeId = NODE_ID;
  
  QuaternionSample* buffer;
  uint16_t windowSize;
  
  if (windowType == 0) {
    // Short window
    buffer = shortWindowBuffer;
    windowSize = SHORT_WINDOW_SIZE;
    features.windowId = shortWindowId;
  } else {
    // Long window
    buffer = longWindowBuffer;
    windowSize = LONG_WINDOW_SIZE;
    features.windowId = longWindowId;
  }
  
  // Calculate mean
  float sum_qw = 0, sum_qx = 0, sum_qy = 0, sum_qz = 0;
  for (uint16_t i = 0; i < windowSize; i++) {
    sum_qw += buffer[i].qw;
    sum_qx += buffer[i].qx;
    sum_qy += buffer[i].qy;
    sum_qz += buffer[i].qz;
  }
  
  features.qw_mean = sum_qw / windowSize;
  features.qx_mean = sum_qx / windowSize;
  features.qy_mean = sum_qy / windowSize;
  features.qz_mean = sum_qz / windowSize;
  
  // Calculate standard deviation
  float sum_sq_qw = 0, sum_sq_qx = 0, sum_sq_qy = 0, sum_sq_qz = 0;
  for (uint16_t i = 0; i < windowSize; i++) {
    sum_sq_qw += pow(buffer[i].qw - features.qw_mean, 2);
    sum_sq_qx += pow(buffer[i].qx - features.qx_mean, 2);
    sum_sq_qy += pow(buffer[i].qy - features.qy_mean, 2);
    sum_sq_qz += pow(buffer[i].qz - features.qz_mean, 2);
  }
  
  features.qw_std = sqrt(sum_sq_qw / windowSize);
  features.qx_std = sqrt(sum_sq_qx / windowSize);
  features.qy_std = sqrt(sum_sq_qy / windowSize);
  features.qz_std = sqrt(sum_sq_qz / windowSize);
  
  // Calculate Signal Magnitude Area (SMA)
  // SMA = sum of absolute values of all quaternion components
  features.sma = 0;
  for (uint16_t i = 0; i < windowSize; i++) {
    features.sma += abs(buffer[i].qw) + abs(buffer[i].qx) + abs(buffer[i].qy) + abs(buffer[i].qz);
  }
  features.sma /= windowSize;
  
  // Calculate dominant frequency (simplified FFT alternative)
  // Using autocorrelation peak detection as a lightweight alternative
  features.dominantFreq = calculateDominantFrequency(buffer, windowSize);
  
  // Transmit features via BLE (split into 3 packets due to 20-byte limit)
  transmitFeatures(features);
  
  #if DEBUG_MODE
    if (windowType == 0) {
      Serial.println();
      Serial.println("=== SHORT WINDOW FEATURES ===");
    } else {
      Serial.println();
      Serial.println("=== LONG WINDOW FEATURES ===");
    }
    Serial.print("Window ID: "); Serial.println(features.windowId);
    Serial.print("Mean: ["); 
    Serial.print(features.qw_mean, 4); Serial.print(", ");
    Serial.print(features.qx_mean, 4); Serial.print(", ");
    Serial.print(features.qy_mean, 4); Serial.print(", ");
    Serial.print(features.qz_mean, 4); Serial.println("]");
    Serial.print("Std: [");
    Serial.print(features.qw_std, 4); Serial.print(", ");
    Serial.print(features.qx_std, 4); Serial.print(", ");
    Serial.print(features.qy_std, 4); Serial.print(", ");
    Serial.print(features.qz_std, 4); Serial.println("]");
    Serial.print("SMA: "); Serial.println(features.sma, 4);
    Serial.print("Dominant Freq: "); Serial.print(features.dominantFreq, 2); Serial.println(" Hz");
    Serial.println("=============================");
    Serial.println();
  #endif
}

float calculateDominantFrequency(QuaternionSample* buffer, uint16_t windowSize) {
  // Simplified frequency estimation using zero-crossing rate
  // on the quaternion magnitude changes
  
  float prevMagnitude = sqrt(pow(buffer[0].qw, 2) + pow(buffer[0].qx, 2) + 
                             pow(buffer[0].qy, 2) + pow(buffer[0].qz, 2));
  int zeroCrossings = 0;
  
  for (uint16_t i = 1; i < windowSize; i++) {
    float magnitude = sqrt(pow(buffer[i].qw, 2) + pow(buffer[i].qx, 2) + 
                          pow(buffer[i].qy, 2) + pow(buffer[i].qz, 2));
    
    float diff = magnitude - prevMagnitude;
    float prevDiff = prevMagnitude - sqrt(pow(buffer[i > 1 ? i-1 : 0].qw, 2) + 
                                         pow(buffer[i > 1 ? i-1 : 0].qx, 2) + 
                                         pow(buffer[i > 1 ? i-1 : 0].qy, 2) + 
                                         pow(buffer[i > 1 ? i-1 : 0].qz, 2));
    
    if ((diff > 0 && prevDiff < 0) || (diff < 0 && prevDiff > 0)) {
      zeroCrossings++;
    }
    
    prevMagnitude = magnitude;
  }
  
  // Frequency = (zero crossings / 2) / window duration
  float windowDuration = windowSize / (float)SAMPLING_RATE_HZ;
  float frequency = (zeroCrossings / 2.0) / windowDuration;
  
  return frequency;
}

void transmitFeatures(WindowFeatures& features) {
  if (!bleConnected) return;
  
  // Packet 1: Means
  FeaturePacket packet1;
  packet1.windowType = features.windowType;
  packet1.nodeId = features.nodeId;
  packet1.windowId = features.windowId;
  packet1.qw_mean = features.qw_mean;
  packet1.qx_mean = features.qx_mean;
  packet1.qy_mean = features.qy_mean;
  packet1.qz_mean = features.qz_mean;
  
  // Packet 2: Standard Deviations
  FeaturePacket2 packet2;
  packet2.windowType = features.windowType;
  packet2.nodeId = features.nodeId;
  packet2.windowId = features.windowId;
  packet2.qw_std = features.qw_std;
  packet2.qx_std = features.qx_std;
  packet2.qy_std = features.qy_std;
  packet2.qz_std = features.qz_std;
  
  // Packet 3: SMA and Dominant Frequency
  FeaturePacket3 packet3;
  packet3.windowType = features.windowType;
  packet3.nodeId = features.nodeId;
  packet3.windowId = features.windowId;
  packet3.sma = features.sma;
  packet3.dominantFreq = features.dominantFreq;
  memset(packet3.padding, 0, 8);
  
  // Send via appropriate BLE characteristic
  if (features.windowType == 0) {
    // Short window (classification)
    featuresShortChar.writeValue((uint8_t*)&packet1, sizeof(FeaturePacket));
    delay(2);  // Small delay between packets
    featuresShortChar.writeValue((uint8_t*)&packet2, sizeof(FeaturePacket2));
    delay(2);
    featuresShortChar.writeValue((uint8_t*)&packet3, sizeof(FeaturePacket3));
  } else {
    // Long window (rep counting)
    featuresLongChar.writeValue((uint8_t*)&packet1, sizeof(FeaturePacket));
    delay(2);
    featuresLongChar.writeValue((uint8_t*)&packet2, sizeof(FeaturePacket2));
    delay(2);
    featuresLongChar.writeValue((uint8_t*)&packet3, sizeof(FeaturePacket3));
  }
}




void performGyroCalibration() {
  #if DEBUG_MODE
    Serial.println();
    Serial.println("========================================");
    Serial.println("Starting gyroscope calibration...");
    Serial.println("Keep sensor stationary!");
    Serial.println("========================================");
  #endif
  
  const int numSamples = 200;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  int validSamples = 0;
  
  delay(1000);  // Wait 1 second before starting
  
  for (int i = 0; i < numSamples; i++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sumGx += gx;
      sumGy += gy;
      sumGz += gz;
      validSamples++;
      
      #if DEBUG_MODE
        if (i % 50 == 0) {
          Serial.print("Calibration progress: ");
          Serial.print((i * 100) / numSamples);
          Serial.println("%");
        }
      #endif
    }
    delay(10);  // 100Hz sampling
  }
  
  if (validSamples > 0) {
    gyroBiasX = sumGx / validSamples;
    gyroBiasY = sumGy / validSamples;
    gyroBiasZ = sumGz / validSamples;
    isCalibrated = true;
    
    #if DEBUG_MODE
      Serial.println();
      Serial.println("Calibration complete!");
      Serial.print("Valid samples: "); Serial.println(validSamples);
      Serial.print("Gyro Bias - X: "); Serial.print(gyroBiasX, 4);
      Serial.print(" Y: "); Serial.print(gyroBiasY, 4);
      Serial.print(" Z: "); Serial.println(gyroBiasZ, 4);
      Serial.println("========================================");
      Serial.println();
    #endif
    
    // Send calibration values back to RPI
    sendCalibrationData();
  } else {
    #if DEBUG_MODE
      Serial.println("Calibration failed - no valid samples!");
    #endif
  }
}

void sendCalibrationData() {
  // Package calibration data
  uint8_t calData[13];
  calData[0] = 0x02;  // Command ID: Calibration Data
  memcpy(&calData[1], &gyroBiasX, 4);
  memcpy(&calData[5], &gyroBiasY, 4);
  memcpy(&calData[9], &gyroBiasZ, 4);
  
  controlChar.writeValue(calData, 13);
}

// BUFFER MANAGEMENT
void storeInBuffer() {
  // Store current data in circular buffer
  dataBuffer[bufferIndex].seqNum = sequenceNumber;
  dataBuffer[bufferIndex].timestamp = timestamp;
  dataBuffer[bufferIndex].qw = qw;
  dataBuffer[bufferIndex].qx = qx;
  dataBuffer[bufferIndex].qy = qy;
  dataBuffer[bufferIndex].qz = qz;
  
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

// BLE TRANSMISSION
void transmitData() {
  if (!bleConnected) return;
  
  // Create packet
  OrientationPacket packet;
  packet.sequenceNumber = sequenceNumber;
  packet.nodeId = NODE_ID;
  packet.timestamp = timestamp;
  packet.qw = qw;
  packet.qx = qx;
  packet.qy = qy;
  packet.qz = qz;
  
  // Send via BLE notification
  orientationChar.writeValue((uint8_t*)&packet, sizeof(OrientationPacket));
  
  // Increment sequence number
  sequenceNumber++;
  
  #if !DEBUG_MODE
    // In standalone mode, only occasional status output
    if (sequenceNumber % 1000 == 0) {
      Serial.print("Running - Seq: ");
      Serial.println(sequenceNumber);
    }
  #endif
}

// CONTROL COMMAND HANDLING
void handleControlCommand() {
  uint8_t command[20];
  int len = controlChar.readValue(command, 20);
  
  if (len < 1) return;
  
  uint8_t commandId = command[0];
  
  #if DEBUG_MODE
    Serial.println();
    Serial.print(">>> Received command: 0x");
    Serial.print(commandId, HEX);
    Serial.print(" (");
    Serial.print(len);
    Serial.println(" bytes)");
  #endif
  
  switch (commandId) {
    case 0x01:  // Start Data Collection
      handleStartCommand(command, len);
      break;
      
    case 0x02:  // Stop Data Collection
      handleStopCommand();
      break;
      
    case 0x03:  // Calibrate Gyroscope
      handleCalibrateCommand();
      break;
      
    case 0x04:  // Set Exercise Type
      handleSetExerciseCommand(command, len);
      break;
      
    case 0x05:  // Reset System
      handleResetCommand();
      break;
      
    case 0x06:  // Sync Time
      handleTimeSyncCommand(command, len);
      break;
    case 0x07:
      handleToggleWindowingCommand();
      break;
      
    case 0xFF:  // Heartbeat/Ping
      handleHeartbeat();
      break;
      
    default:
      #if DEBUG_MODE
        Serial.println("Unknown command!");
      #endif
      sendAck(commandId, 0xFF);  // NACK
      break;
  }
}

void handleStartCommand(uint8_t* command, int len) {
  #if DEBUG_MODE
    Serial.println(">>> Starting data collection...");
  #endif
  
  // Reset counters
  timestamp = 0;
  sequenceNumber = 0;
  systemStartTime = millis();
  lastSampleTime = millis();
  
  isRunning = true;
  sendAck(0x01, 0x00);  // ACK
}

void handleStopCommand() {
  #if DEBUG_MODE
    Serial.println(">>> Stopping data collection...");
  #endif
  
  isRunning = false;
  sendAck(0x02, 0x00);  // ACK
}

void handleCalibrateCommand() {
  bool wasRunning = isRunning;
  isRunning = false;  // Stop data collection during calibration
  
  performGyroCalibration();
  
  isRunning = wasRunning;  // Resume if was running
  sendAck(0x03, 0x00);  // ACK
}

void handleSetExerciseCommand(uint8_t* command, int len) {
  if (len < 2) {
    sendAck(0x04, 0xFF);  // NACK - invalid length
    return;
  }
  
  uint8_t exerciseType = command[1];
  
  #if DEBUG_MODE
    Serial.print(">>> Exercise type set to: ");
    Serial.println(exerciseType);
  #endif
  
  // Note: Exercise type is just for logging, stored on RPI side
  sendAck(0x04, 0x00);  // ACK
}

void handleResetCommand() {
  #if DEBUG_MODE
    Serial.println(">>> Resetting system...");
  #endif
  
  // Reset all state
  isRunning = false;
  timestamp = 0;
  sequenceNumber = 0;
  isCalibrated = false;
  gyroBiasX = gyroBiasY = gyroBiasZ = 0.0;
  
  // Reset filter
  filter.begin(SAMPLING_RATE_HZ);
  
  sendAck(0x05, 0x00);  // ACK
}

void handleTimeSyncCommand(uint8_t* command, int len) {
  if (len < 5) {
    sendAck(0x06, 0xFF);  // NACK
    return;
  }
  
  // Extract new timestamp from command
  uint32_t newTime;
  memcpy(&newTime, &command[1], 4);
  
  // Update timestamp (convert from milliseconds to 10ms ticks)
  timestamp = (uint16_t)(newTime / 10);
  
  #if DEBUG_MODE
    Serial.print(">>> Time synchronized to: ");
    Serial.print(timestamp);
    Serial.println(" (10ms ticks)");
  #endif
  
  sendAck(0x06, 0x00);  // ACK
}

void handleToggleWindowingCommand() {
  enableWindowing = !enableWindowing;
  
  #if DEBUG_MODE
    Serial.print(">>> Windowing mode: ");
    Serial.println(enableWindowing ? "ENABLED" : "DISABLED (raw mode)");
  #endif
  
  if (enableWindowing) {
    resetWindows();
  }
  
  sendAck(0x07, 0x00);  // ACK
}

void handleHeartbeat() {
  // Respond to heartbeat/ping
  uint8_t response[6];
  response[0] = 0xFF;  // Heartbeat response
  response[1] = NODE_ID;
  response[2] = isRunning ? 1 : 0;
  response[3] = isCalibrated ? 1 : 0;
  response[4] = (uint8_t)(timestamp >> 8);
  response[5] = (uint8_t)(timestamp & 0xFF);
  
  controlChar.writeValue(response, 6);
  
  #if DEBUG_MODE
    Serial.println(">>> Heartbeat response sent");
  #endif
}

void sendAck(uint8_t commandId, uint8_t status) {
  uint8_t ack[3];
  ack[0] = 0xAA;  // ACK/NACK identifier
  ack[1] = commandId;
  ack[2] = status;  // 0x00 = success, 0xFF = failure
  
  controlChar.writeValue(ack, 3);
}

// printing functions  that will allow us to track the data easier It is 
void printIMUData() {
  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(" | Gyro: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
}

void printQuaternion() {
  Serial.print("Quaternion: [");
  Serial.print(qw, 4); Serial.print(", ");
  Serial.print(qx, 4); Serial.print(", ");
  Serial.print(qy, 4); Serial.print(", ");
  Serial.print(qz, 4); Serial.println("]");
}

// DEBUG MODE UTILITY FUNCTIONS so that we can test without PI
#if DEBUG_MODE

void printWindowStatus() {
  Serial.println();
  Serial.println("--- Window Status ---");
  Serial.print("Short Window: "); Serial.print(shortWindowSampleCount);
  Serial.print("/"); Serial.print(SHORT_WINDOW_SIZE);
  Serial.print(" | Samples since last: "); Serial.println(samplesSinceLastShortWindow);
  Serial.print("Long Window: "); Serial.print(longWindowSampleCount);
  Serial.print("/"); Serial.print(LONG_WINDOW_SIZE);
  Serial.print(" | Samples since last: "); Serial.println(samplesSinceLastLongWindow);
  Serial.print("Short Window ID: "); Serial.println(shortWindowId);
  Serial.print("Long Window ID: "); Serial.println(longWindowId);
  Serial.print("Uptime: "); Serial.print(millis() - systemStartTime); Serial.println(" ms");
  Serial.println("---------------------");
  Serial.println();
}


void printDetailedPacketInfo() {
  Serial.println();
  Serial.println("--- Packet Details ---");
  Serial.print("Sequence: "); Serial.println(sequenceNumber);
  Serial.print("Node ID: "); Serial.println(NODE_ID);
  Serial.print("Timestamp: "); Serial.print(timestamp); Serial.println(" (10ms ticks)");
  Serial.print("Uptime: "); Serial.print(millis() - systemStartTime); Serial.println(" ms");
  Serial.println();
  
  Serial.println("Raw IMU Data:");
  Serial.print("  Accel (g): ");
  Serial.print(ax, 3); Serial.print(", ");
  Serial.print(ay, 3); Serial.print(", ");
  Serial.println(az, 3);
  
  Serial.print("  Gyro (dps): ");
  Serial.print(gx, 2); Serial.print(", ");
  Serial.print(gy, 2); Serial.print(", ");
  Serial.println(gz, 2);
  Serial.println();
  
  Serial.println("Quaternion:");
  Serial.print("  [W, X, Y, Z] = [");
  Serial.print(qw, 4); Serial.print(", ");
  Serial.print(qx, 4); Serial.print(", ");
  Serial.print(qy, 4); Serial.print(", ");
  Serial.print(qz, 4); Serial.println("]");
  
  // Calculate and show quaternion magnitude
  float qMag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  Serial.print("  Magnitude: "); Serial.println(qMag, 6);
  Serial.println();
  
  Serial.println("BLE Packet (bytes):");
  OrientationPacket packet;
  packet.sequenceNumber = sequenceNumber;
  packet.nodeId = NODE_ID;
  packet.timestamp = timestamp;
  packet.qw = qw;
  packet.qx = qx;
  packet.qy = qy;
  packet.qz = qz;
  
  uint8_t* bytes = (uint8_t*)&packet;
  Serial.print("  ");
  for (int i = 0; i < sizeof(OrientationPacket); i++) {
    if (bytes[i] < 0x10) Serial.print("0");
    Serial.print(bytes[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 8 == 0) Serial.println();
  }
  Serial.println();
  Serial.print("  Total: "); Serial.print(sizeof(OrientationPacket)); Serial.println(" bytes");
  
  Serial.println("--- End Packet ---");
  Serial.println();
}

// Print BLE connection status
void printConnectionStatus() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("BLE Status:");
  Serial.print("  Connected: "); Serial.println(bleConnected ? "YES" : "NO");
  Serial.print("  Running: "); Serial.println(isRunning ? "YES" : "NO");
  Serial.print("  Calibrated: "); Serial.println(isCalibrated ? "YES" : "NO");
  Serial.print("  Sequence Number: "); Serial.println(sequenceNumber);
  Serial.print("  Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
  Serial.println("========================================");
  Serial.println();
}

// Test function to verify packet structure
void testPacketStructure() {
  Serial.println();
  Serial.println("PACKET STRUCTURE TEST");
  
  // Create a test packet with known values
  OrientationPacket testPacket;
  testPacket.sequenceNumber = 0xAB;  // 171
  testPacket.nodeId = NODE_ID;
  testPacket.timestamp = 0x1234;     // 4660
  testPacket.qw = 1.0f;
  testPacket.qx = 0.0f;
  testPacket.qy = 0.0f;
  testPacket.qz = 0.0f;
  
  Serial.println("Test Packet Contents:");
  Serial.print("  Sequence: 0x"); Serial.println(testPacket.sequenceNumber, HEX);
  Serial.print("  Node ID: "); Serial.println(testPacket.nodeId);
  Serial.print("  Timestamp: 0x"); Serial.println(testPacket.timestamp, HEX);
  Serial.print("  Quaternion: ["); 
  Serial.print(testPacket.qw); Serial.print(", ");
  Serial.print(testPacket.qx); Serial.print(", ");
  Serial.print(testPacket.qy); Serial.print(", ");
  Serial.print(testPacket.qz); Serial.println("]");
  Serial.println();
  
  Serial.print("Packet Size: "); Serial.print(sizeof(OrientationPacket)); Serial.println(" bytes");
  Serial.println();
  
  Serial.println("Raw Bytes (Hex):");
  uint8_t* bytes = (uint8_t*)&testPacket;
  Serial.print("  ");
  for (int i = 0; i < sizeof(OrientationPacket); i++) {
    if (bytes[i] < 0x10) Serial.print("0");
    Serial.print(bytes[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 8 == 0 && i < sizeof(OrientationPacket) - 1) {
      Serial.println();
      Serial.print("  ");
    }
  }
  Serial.println();
  Serial.println();
  
  Serial.println("Memory Layout:");
  Serial.print("  Byte 0: sequenceNumber\n");
  Serial.print("  Byte 1: nodeId\n");
  Serial.print("  Bytes 2-3: timestamp (uint16_t, little-endian)\n");
  Serial.print("  Bytes 4-7: qw (float)\n");
  Serial.print("  Bytes 8-11: qx (float)\n");
  Serial.print("  Bytes 12-15: qy (float)\n");
  Serial.print("  Bytes 16-19: qz (float)\n");
  Serial.println();
  Serial.println();
}

// Interactive debug menu
void printDebugMenu() {
  Serial.println();
  Serial.println("DEBUG MENU");
  Serial.println("Send commands via Serial Monitor:");
  Serial.println("  '1' - Print status");
  Serial.println("  '2' - Print Window Status");
  Serial.println("  '3' - Calibrate gyroscope");
  Serial.println("  '4' - Print current IMU reading");
  Serial.println("  '5' - Print current quaternion");
  Serial.println("  '6' - Start data collection");
  Serial.println("  '7' - Stop data collection");
  Serial.println("  '8' - Reset system");
  Serial.println("  '9' - Show this menu");
  Serial.println("========================================");
  Serial.println();
}

// Handle serial commands in debug mode
void handleSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Clear any remaining characters
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case '1':
        printConnectionStatus();
        break;
        
      case '2':
        printWindowStatus();
        break;
        
      case '3':
        {
          bool wasRunning = isRunning;
          isRunning = false;
          performGyroCalibration();
          isRunning = wasRunning;
        }
        break;
        
      case '4':
        if (readIMU()) {
          printIMUData();
        } else {
          Serial.println("Failed to read IMU");
        }
        break;
        
      case '5':
        printQuaternion();
        break;
        
      case '6':
        Serial.println(">>> Starting data collection via serial command");
        isRunning = true;
        timestamp = 0;
        sequenceNumber = 0;
        lastSampleTime = millis();
        break;
        
      case '7':
        Serial.println(">>> Stopping data collection via serial command");
        isRunning = false;
        break;
        
      case '8':
        Serial.println(">>> Resetting system via serial command");
        isRunning = false;
        timestamp = 0;
        sequenceNumber = 0;
        isCalibrated = false;
        gyroBiasX = gyroBiasY = gyroBiasZ = 0.0;
        filter.begin(SAMPLING_RATE_HZ);
        Serial.println("System reset complete");
        break;
        
      case '9':
        printDebugMenu();
        break;
        
      case '\n':
      case '\r':
        // Ignore newlines
        break;
        
      default:
        Serial.print("Unknown command: '");
        Serial.print(cmd);
        Serial.println("' - Type 'h' for help");
        break;
    }
  }
}
#endif