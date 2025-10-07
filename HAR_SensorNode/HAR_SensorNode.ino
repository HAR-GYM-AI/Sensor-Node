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
 * - Window-based feature extraction with raw sample bundling
 * 
 * NODE CONFIGURATION:
 * Each device MUST be configured with a unique placement before deployment:
 *   - Set NODE_PLACEMENT to: WRIST, BICEP, CHEST, or THIGH
 *   - This determines the node's identity across the 4-node system
 *   - BLE device name will be: HAR_WRIST_0, HAR_BICEP_1, etc.
 *   - All packets include nodeId to identify source during simultaneous operation
 * 
 * MULTI-NODE OPERATION:
 * All 4 nodes transmit simultaneously. Each node is uniquely identified by:
 *   1. BLE device name (e.g., "HAR_BICEP_1")
 *   2. Node ID in every data packet (WRIST=0, BICEP=1, CHEST=2, THIGH=3)
 *   3. Initial identification packet (0xFE) sent on BLE connection
 * 
 * Library Required: Adafruit AHRS (install via Arduino Library Manager) + BLE
 */

#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <Adafruit_AHRS.h>

// CONFIG

// OPERATING MODE - CHANGE THIS FOR TESTING vs DEPLOYMENT
#define DEBUG_MODE true  // true = USB debugging with serial output
                         // false = standalone mode (no USB, for deployment)

// Node Placement Enumeration
// ┌──────────┬────────┬───────────────────┐
// │ Location │ NodeID │ BLE Device Name   │
// ├──────────┼────────┼───────────────────┤
// │ WRIST    │   0    │ HAR_WRIST_0       │
// │ BICEP    │   1    │ HAR_BICEP_1       │
// │ CHEST    │   2    │ HAR_CHEST_2       │
// │ THIGH    │   3    │ HAR_THIGH_3       │
// └──────────┴────────┴───────────────────┘
enum NodePlacement : uint8_t {
  WRIST = 0,
  BICEP = 1,
  CHEST = 2,
  THIGH = 3
};

// IMPORTANT: CHANGE THIS FOR EACH DEVICE 
// Set to WRIST, BICEP, CHEST, or THIGH based on physical placement
#define NODE_PLACEMENT BICEP  // ← CHANGE THIS
#define NODE_ID NODE_PLACEMENT  // For backward compatibility

// Sampling Configuration
#define SAMPLING_RATE_HZ 100
#define SAMPLING_PERIOD_MS 10  // 1000ms / 100Hz = 10ms
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

// Packet Type Constants
#define PKT_TYPE_FEATURES_1 0x11
#define PKT_TYPE_FEATURES_2 0x12
#define PKT_TYPE_FEATURES_3 0x13
#define PKT_TYPE_SAMPLE_1   0x21
#define PKT_TYPE_SAMPLE_2   0x22


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
//
// WINDOWING MODE (enableWindowing = true):
//   When a window is ready, sends:
//   1. FeaturePacket (means)
//   2. FeaturePacket2 (std devs)
//   3. FeaturePacket3 (SMA, freq, sample count)
//   4. WindowSamplePacket1 + WindowSamplePacket2 (repeated for each sample in window)
//
//   Short Window: 3 + (150 * 2) = 303 packets every ~380ms
//   Long Window:  3 + (300 * 2) = 603 packets every ~1500ms
//
// RAW MODE (enableWindowing = false):
//   Sends OrientationPacket continuously at 100Hz
//

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
  uint8_t packetType;      //e.g., PKT_TYPE_FEATURES_1
  uint8_t windowType;      // 0=short, 1=long
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float qw_mean;           // 4 bytes
  float qx_mean;           // 4 bytes
  float qy_mean;           // 4 bytes
  float qz_mean;           // 4 bytes
  uint8_t padding[1];      // 1 byte padding to keep total 20 bytes
  // Total: 20 bytes (first packet)
};

struct FeaturePacket2 {
  uint8_t packetType;      // e.g., PKT_TYPE_FEATURES_2
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float qw_std;            // 4 bytes
  float qx_std;            // 4 bytes
  float qy_std;            // 4 bytes
  float qz_std;            // 4 bytes
  uint8_t padding[1];      // 1 byte padding to keep total 20 bytes
  // Total: 20 bytes (second packet)
};

struct FeaturePacket3 {
  uint8_t packetType;      // e.g., PKT_TYPE_FEATURES_3
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  float sma;               // 4 bytes
  float dominantFreq;      // 4 bytes
  uint16_t totalSamples;   // 2 bytes - number of samples in window
  uint8_t padding[5];      // 5 bytes padding to keep total 20 bytes
  // Total: 20 bytes (third packet)
};

// Packet for raw quaternion samples within a window
struct WindowSamplePacket {
  uint8_t windowType;      // 1 byte (0=short, 1=long)
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  uint16_t sampleIndex;    // 2 bytes - index within window (0 to windowSize-1)
  uint16_t timestamp;      // 2 bytes
  float qw;                // 4 bytes
  float qx;                // 4 bytes
  float qy;                // 4 bytes
  float qz;                // 4 bytes
  // Total: 24 bytes - EXCEEDS 20! Need to split
};

// Split into two packets due to 20-byte BLE limit
struct WindowSamplePacket1 {
  uint8_t packetType;      // PKT_TYPE_SAMPLE_1
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  uint16_t sampleIndex;    // 2 bytes
  uint16_t timestamp;      // 2 bytes
  float qw;                // 4 bytes
  float qx;                // 4 bytes
  uint8_t padding[3];      // 3 bytes padding to keep total 20 bytes
  // Total: 20 bytes
};

struct WindowSamplePacket2 {
  uint8_t packetType;      //PKT_TYPE_SAMPLE_2
  uint8_t windowType;      // 1 byte
  uint8_t nodeId;          // 1 byte
  uint16_t windowId;       // 2 bytes
  uint16_t sampleIndex;    // 2 bytes
  float qy;                // 4 bytes
  float qz;                // 4 bytes
  uint8_t padding[5];      // 5 bytes padding to keep total 20 bytes
  // Total: 20 bytes
};

// Node identification packet sent on connection
struct NodeIdentificationPacket {
  uint8_t packetType;      // 1 byte - 0xFE for identification
  uint8_t nodeId;          // 1 byte - WRIST=0, BICEP=1, CHEST=2, THIGH=3
  uint8_t nodePlacement;   // 1 byte - same as nodeId (for clarity)
  uint8_t samplingRate;    // 1 byte - 100Hz
  uint16_t shortWindowSize; // 2 bytes - 150
  uint16_t longWindowSize;  // 2 bytes - 300
  uint8_t firmwareVersion; // 1 byte - version number
  uint8_t padding[11];     // 11 bytes padding
  // Total: 20 bytes
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
    Serial.print("Node Placement: ");
    switch(NODE_PLACEMENT) {
      case WRIST: Serial.println("WRIST (0)"); break;
      case BICEP: Serial.println("BICEP (1)"); break;
      case CHEST: Serial.println("CHEST (2)"); break;
      case THIGH: Serial.println("THIGH (3)"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    Serial.print("Node ID: ");
    Serial.println(NODE_ID);
    Serial.print("Short Window: ");
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
    
    // Send node identification immediately after connection
    delay(100);  // Small delay to ensure connection is stable
    sendNodeIdentification();
    
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
              // When windowing is disabled, send raw data continuously
              storeInBuffer();
              transmitData();
            }
            
            #if DEBUG_MODE
              // In debug mode, print detailed packet info every 50 samples (0.5 sec)
              // if (sequenceNumber % 50 == 0) {
              //   printDetailedPacketInfo();
              // }
            #endif
          }
          // Note: Silently skip failed reads - they're usually just timing mismatches
          // and will succeed on the next cycle
          
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

// Helper function to get placement name
const char* getPlacementName(NodePlacement placement) {
  switch(placement) {
    case WRIST: return "WRIST";
    case BICEP: return "BICEP";
    case CHEST: return "CHEST";
    case THIGH: return "THIGH";
    default: return "UNKNOWN";
  }
}

void setupBLE() {
  // Set device name with placement identifier
  String deviceName = "HAR_";
  deviceName += getPlacementName(NODE_PLACEMENT);
  deviceName += "_";
  deviceName += String(NODE_ID);
  
  BLE.setLocalName(deviceName.c_str());
  BLE.setDeviceName(deviceName.c_str());
  

    //descriptors to make characteristics show readable names in nRF Connect
  BLEDescriptor orientationDescriptor("2901", "Orientation Data");
  orientationChar.addDescriptor(orientationDescriptor);
  
  BLEDescriptor controlDescriptor("2901", "Control Channel");
  controlChar.addDescriptor(controlDescriptor);
  
  BLEDescriptor featuresShortDescriptor("2901", "Features Short Window");
  featuresShortChar.addDescriptor(featuresShortDescriptor);
  
  BLEDescriptor featuresLongDescriptor("2901", "Features Long Window");
  featuresLongChar.addDescriptor(featuresLongDescriptor);
  
  // Add characteristics to service
  sensorService.addCharacteristic(orientationChar);
  sensorService.addCharacteristic(controlChar);
  sensorService.addCharacteristic(featuresShortChar);
  sensorService.addCharacteristic(featuresLongChar);

  
  // Add service
  BLE.addService(sensorService);
    // Set advertised service
  BLE.setAdvertisedService(sensorService);
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
  
  #if DEBUG_MODE
    Serial.println();
    Serial.println("=== BLE TRANSMISSION START ===");
    Serial.print("Window Type: ");
    Serial.println(features.windowType == 0 ? "SHORT (Classification)" : "LONG (Rep Counting)");
    Serial.print("Window ID: ");
    Serial.println(features.windowId);
    Serial.print("Timestamp: ");
    Serial.println(millis() - systemStartTime);
  #endif
  
  // Packet 1: Means
  FeaturePacket packet1;
  packet1.packetType = PKT_TYPE_FEATURES_1; // **SET THE TYPE**
  packet1.windowType = features.windowType;
  packet1.nodeId = features.nodeId;
  packet1.windowId = features.windowId;
  packet1.qw_mean = features.qw_mean;
  packet1.qx_mean = features.qx_mean;
  packet1.qy_mean = features.qy_mean;
  packet1.qz_mean = features.qz_mean;
  memset(packet1.padding, 0, 1);
  
  // Packet 2: Standard Deviations
  FeaturePacket2 packet2;
  packet2.packetType = PKT_TYPE_FEATURES_2; // **SET THE TYPE**
  packet2.windowType = features.windowType;
  packet2.nodeId = features.nodeId;
  packet2.windowId = features.windowId;
  packet2.qw_std = features.qw_std;
  packet2.qx_std = features.qx_std;
  packet2.qy_std = features.qy_std;
  packet2.qz_std = features.qz_std;
  memset(packet2.padding, 0, 1);
  
  // Packet 3: SMA and Dominant Frequency
  FeaturePacket3 packet3;
  packet3.packetType = PKT_TYPE_FEATURES_3; // **SET THE TYPE**
  packet3.windowType = features.windowType;
  packet3.nodeId = features.nodeId;
  packet3.windowId = features.windowId;
  packet3.sma = features.sma;
  packet3.dominantFreq = features.dominantFreq;
  
  // Get window buffer and size
  QuaternionSample* buffer;
  uint16_t windowSize;
  if (features.windowType == 0) {
    buffer = shortWindowBuffer;
    windowSize = SHORT_WINDOW_SIZE;
  } else {
    buffer = longWindowBuffer;
    windowSize = LONG_WINDOW_SIZE;
  }
  packet3.totalSamples = windowSize;
  memset(packet3.padding, 0, 5);
  
  // Send via appropriate BLE characteristic
  if (features.windowType == 0) {
    // Short window (classification)
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 1 (Means) via SHORT window characteristic...");
    #endif
    featuresShortChar.writeValue((uint8_t*)&packet1, sizeof(FeaturePacket));
    delay(2);  // Small delay between packets
    
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 2 (Std Devs) via SHORT window characteristic...");
    #endif
    featuresShortChar.writeValue((uint8_t*)&packet2, sizeof(FeaturePacket2));
    delay(2);
    
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 3 (SMA & Freq) via SHORT window characteristic...");
    #endif
    featuresShortChar.writeValue((uint8_t*)&packet3, sizeof(FeaturePacket3));
  } else {
    // Long window (rep counting)
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 1 (Means) via LONG window characteristic...");
    #endif
    featuresLongChar.writeValue((uint8_t*)&packet1, sizeof(FeaturePacket));
    delay(2);
    
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 2 (Std Devs) via LONG window characteristic...");
    #endif
    featuresLongChar.writeValue((uint8_t*)&packet2, sizeof(FeaturePacket2));
    delay(2);
    
    #if DEBUG_MODE
      Serial.println("Transmitting Packet 3 (SMA & Freq) via LONG window characteristic...");
    #endif
    featuresLongChar.writeValue((uint8_t*)&packet3, sizeof(FeaturePacket3));
  }
  
  #if DEBUG_MODE
    Serial.println("✓ All 3 feature packets transmitted");
    Serial.print("Now transmitting ");
    Serial.print(windowSize);
    Serial.println(" raw quaternion samples...");
  #endif
  
  // Now send all raw quaternion samples from the window
  // Each sample requires 2 packets (due to 20-byte BLE limit)
  for (uint16_t i = 0; i < windowSize; i++) {
    // Packet 1: windowType, nodeId, windowId, sampleIndex, timestamp, qw, qx
    WindowSamplePacket1 samplePkt1;
    samplePkt1.packetType = PKT_TYPE_SAMPLE_1; // **SET THE TYPE**
    samplePkt1.windowType = features.windowType;
    samplePkt1.nodeId = features.nodeId;
    samplePkt1.windowId = features.windowId;
    samplePkt1.sampleIndex = i;
    samplePkt1.timestamp = buffer[i].timestamp;
    samplePkt1.qw = buffer[i].qw;
    samplePkt1.qx = buffer[i].qx;
    memset(samplePkt1.padding, 0, 3);
    
    // Packet 2: windowType, nodeId, windowId, sampleIndex, qy, qz
    WindowSamplePacket2 samplePkt2;
    samplePkt2.packetType = PKT_TYPE_SAMPLE_2; // **SET THE TYPE**
    samplePkt2.windowType = features.windowType;
    samplePkt2.nodeId = features.nodeId;
    samplePkt2.windowId = features.windowId;
    samplePkt2.sampleIndex = i;
    samplePkt2.qy = buffer[i].qy;
    samplePkt2.qz = buffer[i].qz;
    memset(samplePkt2.padding, 0, 5);
    
    // Send via appropriate BLE characteristic
    if (features.windowType == 0) {
      featuresShortChar.writeValue((uint8_t*)&samplePkt1, sizeof(WindowSamplePacket1));
      delay(2);
      featuresShortChar.writeValue((uint8_t*)&samplePkt2, sizeof(WindowSamplePacket2));
      delay(2);
    } else {
      featuresLongChar.writeValue((uint8_t*)&samplePkt1, sizeof(WindowSamplePacket1));
      delay(2);
      featuresLongChar.writeValue((uint8_t*)&samplePkt2, sizeof(WindowSamplePacket2));
      delay(2);
    }
    
    #if DEBUG_MODE
      // Print progress every 50 samples
      if ((i + 1) % 50 == 0 || i == windowSize - 1) {
        Serial.print("  Transmitted sample ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.println(windowSize);
      }
    #endif
  }
  
  #if DEBUG_MODE
    Serial.println("✓ All window data transmitted successfully");
    Serial.print("Total packets sent: 3 (features) + ");
    Serial.print(windowSize * 2);
    Serial.print(" (samples) = ");
    Serial.println(3 + windowSize * 2);
    Serial.println("=== BLE TRANSMISSION END ===");
    Serial.println();
  #endif
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

// Send node identification packet
void sendNodeIdentification() {
  if (!bleConnected) return;
  
  NodeIdentificationPacket idPacket;
  idPacket.packetType = 0xFE;  // Identification packet marker
  idPacket.nodeId = NODE_ID;
  idPacket.nodePlacement = NODE_PLACEMENT;
  idPacket.samplingRate = SAMPLING_RATE_HZ;
  idPacket.shortWindowSize = SHORT_WINDOW_SIZE;
  idPacket.longWindowSize = LONG_WINDOW_SIZE;
  idPacket.firmwareVersion = 1;  // Version 1.0
  memset(idPacket.padding, 0, 11);
  
  // Send via control characteristic
  controlChar.writeValue((uint8_t*)&idPacket, sizeof(NodeIdentificationPacket));
  
  #if DEBUG_MODE
    Serial.println();
    Serial.println("=== NODE IDENTIFICATION SENT ===");
    Serial.print("Placement: ");
    Serial.println(getPlacementName(NODE_PLACEMENT));
    Serial.print("Node ID: ");
    Serial.println(NODE_ID);
    Serial.print("Sampling Rate: ");
    Serial.print(SAMPLING_RATE_HZ);
    Serial.println(" Hz");
    Serial.print("Window Sizes: Short=");
    Serial.print(SHORT_WINDOW_SIZE);
    Serial.print(", Long=");
    Serial.println(LONG_WINDOW_SIZE);
    Serial.println("================================");
    Serial.println();
  #endif
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
  
  #if DEBUG_MODE
    // Print raw quaternion transmission every 100 samples (1 second)
    if (sequenceNumber % 100 == 0) {
      Serial.print("Raw Quat #");
      Serial.print(sequenceNumber);
      Serial.print(" [");
      Serial.print(qw, 4); Serial.print(", ");
      Serial.print(qx, 4); Serial.print(", ");
      Serial.print(qy, 4); Serial.print(", ");
      Serial.print(qz, 4); Serial.print("] @ ");
      Serial.print(timestamp * 10); Serial.println("ms");
    }
  #endif
  
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
  Serial.println("NODE INFORMATION:");
  Serial.print("  Placement: ");
  Serial.print(getPlacementName(NODE_PLACEMENT));
  Serial.print(" (ID: ");
  Serial.print(NODE_ID);
  Serial.println(")");
  Serial.println();
  Serial.println("BLE Status:");
  Serial.print("  Connected: "); Serial.println(bleConnected ? "YES" : "NO");
  Serial.print("  Running: "); Serial.println(isRunning ? "YES" : "NO");
  Serial.print("  Calibrated: "); Serial.println(isCalibrated ? "YES" : "NO");
  Serial.print("  Windowing: "); Serial.println(enableWindowing ? "ENABLED" : "DISABLED");
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