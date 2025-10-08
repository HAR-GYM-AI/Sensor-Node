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
 * - Sliding window buffers maintained for RPI-side feature extraction
 * - Raw quaternion transmission only (features extracted on RPI)
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

// Force struct packing to 1 byte alignment
#pragma pack(1)

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

uint16_t shortWindowId = 0;
uint16_t longWindowId = 0;

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


// GLOBAL VARIABLES

// BLE Service and Characteristics
BLEService sensorService(SERVICE_UUID);
BLECharacteristic orientationChar(ORIENTATION_CHAR_UUID, BLERead | BLENotify, 20);
BLECharacteristic controlChar(CONTROL_CHAR_UUID, BLEWrite | BLENotify, 20);

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





// Circular Buffer for BLE Recovery
struct SensorData {
  uint8_t seqNum;
  uint16_t timestamp;
  float qw, qx, qy, qz;
};
SensorData dataBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

// BLE TRANSMISSION QUEUE
// All packets are 20 bytes, so we can use a generic structure
#define BLE_PACKET_SIZE 20
#define BLE_QUEUE_SIZE 256  // Buffer up to 1024 packets (20KB RAM usage)

enum BLECharacteristicType {
  ORIENTATION_CHAR = 0,
  CONTROL_CHAR = 1
};

struct BLEPacket {
  BLECharacteristicType charType;  // Which BLE characteristic to send on
  uint8_t data[BLE_PACKET_SIZE];   // The packet data
  uint32_t enqueueTime;            // When packet was queued (for timeout handling)
};

struct BLETransmissionQueue {
  BLEPacket queue[BLE_QUEUE_SIZE];
  uint16_t head;                   // Index of next packet to transmit
  uint16_t tail;                   // Index of next free slot
  uint16_t count;                  // Number of packets in queue
  uint32_t lastTransmitTime;       // Last time we transmitted a packet
  bool isTransmitting;             // Flag to prevent concurrent transmission
};

// Global queue instance
BLETransmissionQueue bleQueue;

// System State
bool isRunning = false;
bool bleConnected = false;

// BLE TRANSMISSION QUEUE FUNCTIONS
void initBLEQueue() {
  bleQueue.head = 0;
  bleQueue.tail = 0;
  bleQueue.count = 0;
  bleQueue.lastTransmitTime = 0;
  bleQueue.isTransmitting = false;
}

bool enqueueBLEPacket(BLECharacteristicType charType, const uint8_t* data, uint8_t dataSize) {
  if (dataSize != BLE_PACKET_SIZE) {
    #if DEBUG_MODE
      Serial.print("ERROR: Invalid packet size ");
      Serial.println(dataSize);
    #endif
    return false;
  }
  
  if (bleQueue.count >= BLE_QUEUE_SIZE) {
    #if DEBUG_MODE
      Serial.println("ERROR: BLE queue full! Packet dropped.");
    #endif
    return false;  // Queue full
  }
  
  // Copy data to queue
  bleQueue.queue[bleQueue.tail].charType = charType;
  memcpy(bleQueue.queue[bleQueue.tail].data, data, BLE_PACKET_SIZE);
  bleQueue.queue[bleQueue.tail].enqueueTime = millis();
  
  // Update queue pointers
  bleQueue.tail = (bleQueue.tail + 1) % BLE_QUEUE_SIZE;
  bleQueue.count++;
  
  #if DEBUG_MODE
    if (bleQueue.count % 50 == 0) {  // Log every 50 packets to avoid spam
      Serial.print("BLE Queue: ");
      Serial.print(bleQueue.count);
      Serial.print("/");
      Serial.println(BLE_QUEUE_SIZE);
    }
  #endif
  
  return true;
}

bool dequeueBLEPacket(BLEPacket* packet) {
  if (bleQueue.count == 0) {
    return false;  // Queue empty
  }
  
  // Copy packet data
  *packet = bleQueue.queue[bleQueue.head];
  
  // Update queue pointers
  bleQueue.head = (bleQueue.head + 1) % BLE_QUEUE_SIZE;
  bleQueue.count--;
  
  return true;
}

uint16_t getBLEQueueCount() {
  return bleQueue.count;
}

bool isBLEQueueFull() {
  return bleQueue.count >= BLE_QUEUE_SIZE;
}

bool isBLEQueueEmpty() {
  return bleQueue.count == 0;
}

void processBLEQueue() {
  if (!bleConnected || bleQueue.isTransmitting) {
    return;  // Not connected or already transmitting
  }
  
  unsigned long currentTime = millis();
  
  // Rate limiting: don't transmit more than one packet every 5ms to avoid BLE congestion
  // This allows up to 200 packets/second, which should handle our data rates:
  // - Raw mode: 100 packets/sec (well within limit)
  // - Window mode: bursts of 300-600 packets, but spaced out by window intervals
  if (currentTime - bleQueue.lastTransmitTime < 5) {
    return;  // Too soon since last transmission
  }
  
  BLEPacket packet;
  if (!dequeueBLEPacket(&packet)) {
    return;  // No packets to send
  }
  
  bleQueue.isTransmitting = true;
  
  // Transmit based on characteristic type
  bool success = false;
  switch (packet.charType) {
    case ORIENTATION_CHAR:
      success = orientationChar.writeValue(packet.data, BLE_PACKET_SIZE);
      break;
    case CONTROL_CHAR:
      success = controlChar.writeValue(packet.data, BLE_PACKET_SIZE);
      break;
    default:
      #if DEBUG_MODE
        Serial.print("ERROR: Unknown characteristic type ");
        Serial.println(packet.charType);
      #endif
      success = false;
      break;
  }
  
  if (success) {
    bleQueue.lastTransmitTime = currentTime;
    
    #if DEBUG_MODE
      // Occasional status logging
      static uint16_t packetCount = 0;
      packetCount++;
      if (packetCount % 100 == 0) {
        Serial.print("BLE: Transmitted packet ");
        Serial.print(packetCount);
        Serial.print(" (queue: ");
        Serial.print(bleQueue.count);
        Serial.println(" remaining)");
      }
    #endif
  } else {
    #if DEBUG_MODE
      Serial.println("ERROR: BLE transmission failed! Re-queuing packet.");
    #endif
    // Re-queue the packet at the front (higher priority)
    // Note: This is a simplified approach. In production, you might want
    // more sophisticated retry logic with backoff.
    if (bleQueue.count < BLE_QUEUE_SIZE) {
      // Shift head back to re-insert at front
      bleQueue.head = (bleQueue.head - 1 + BLE_QUEUE_SIZE) % BLE_QUEUE_SIZE;
      bleQueue.queue[bleQueue.head] = packet;
      bleQueue.count++;
    } else {
      #if DEBUG_MODE
        Serial.println("ERROR: Could not re-queue failed packet - queue still full!");
      #endif
    }
  }
  
  bleQueue.isTransmitting = false;
}

void monitorBLEQueue() {
  static unsigned long lastMonitorTime = 0;
  unsigned long currentTime = millis();
  
  // Monitor queue health every 5 seconds
  if (currentTime - lastMonitorTime < 5000) {
    return;
  }
  lastMonitorTime = currentTime;
  
  uint16_t queueCount = getBLEQueueCount();
  float queueUtilization = (float)queueCount / BLE_QUEUE_SIZE * 100.0;
  
  #if DEBUG_MODE
    if (queueUtilization > 80.0) {
      Serial.print("WARNING: BLE queue utilization high: ");
      Serial.print(queueUtilization, 1);
      Serial.print("% (");
      Serial.print(queueCount);
      Serial.println(" packets)");
    } else if (queueCount > 0) {
      Serial.print("BLE queue status: ");
      Serial.print(queueCount);
      Serial.print("/");
      Serial.print(BLE_QUEUE_SIZE);
      Serial.print(" packets (");
      Serial.print(queueUtilization, 1);
      Serial.println("%)");
    }
  #endif
  
  // Check for stale packets (older than 30 seconds)
  // This helps prevent memory issues if transmission gets stuck
  if (queueCount > 0) {
    unsigned long oldestPacketTime = bleQueue.queue[bleQueue.head].enqueueTime;
    unsigned long age = currentTime - oldestPacketTime;
    
    if (age > 30000) {  // 30 seconds
      #if DEBUG_MODE
        Serial.print("WARNING: Removing stale packet aged ");
        Serial.print(age / 1000);
        Serial.println(" seconds");
      #endif
      
      // Remove the stale packet
      BLEPacket dummy;
      dequeueBLEPacket(&dummy);
    }
  }
}

// PACKET STRUCTURE
//
// RAW MODE:
//   Sends OrientationPacket continuously at 100Hz
//   Windowing is maintained in buffers for RPI-side processing
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
  
  // Initialize BLE transmission queue
  initBLEQueue();
  
  #if DEBUG_MODE
    Serial.println("BLE transmission queue initialized");
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
      
      // Process BLE transmission queue
      processBLEQueue();
      
      // Monitor queue health
      monitorBLEQueue();

      // Check for control commands
      if (controlChar.written()) {
        handleControlCommand();
      }
      
      // Sample and transmit data if running
      if (isRunning) {
        // Check if BLE queue is too full - apply backpressure to prevent data loss
        if (getBLEQueueCount() > BLE_QUEUE_SIZE * 0.9) {  // 90% full
          #if DEBUG_MODE
            static unsigned long lastBackpressureWarning = 0;
            if (millis() - lastBackpressureWarning > 1000) {  // Warn once per second
              Serial.println("WARNING: BLE queue nearly full - applying backpressure (skipping sample)");
              lastBackpressureWarning = millis();
            }
          #endif
          // Skip this sample to allow queue to drain
          continue;
        }
        
        unsigned long currentTime = millis();
        
        // Check if it's time for next sample (100Hz = 10ms period)
        if (currentTime - lastSampleTime >= SAMPLING_PERIOD_MS) {
          lastSampleTime = currentTime;
          
          // Read IMU and process
          if (readIMU()) {
            applyCalibration();
            updateOrientation();
            
            // Add to sliding windows for RPI-side processing
            addToWindows();
            
            // Always transmit raw quaternion data
            storeInBuffer();
            transmitData();
            
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
  
  // Add characteristics to service
  sensorService.addCharacteristic(orientationChar);
  sensorService.addCharacteristic(controlChar);

  
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
  
  // Check for NaN or infinite values and replace with safe defaults
  if (isnan(qw) || isinf(qw)) qw = 1.0;
  if (isnan(qx) || isinf(qx)) qx = 0.0;
  if (isnan(qy) || isinf(qy)) qy = 0.0;
  if (isnan(qz) || isinf(qz)) qz = 0.0;
  
  // Always normalize quaternions to ensure unit length
  float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (norm > 0.0001) {
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
  } else {
    // If norm is too small, reset to identity quaternion
    qw = 1.0;
    qx = 0.0;
    qy = 0.0;
    qz = 0.0;
  }
  
  #if DEBUG_MODE
    // Debug: Print quaternion values every 100 samples
    static uint16_t debug_counter = 0;
    debug_counter++;
    if (debug_counter % 100 == 0) {
      Serial.print("Quaternion: [");
      Serial.print(qw, 4); Serial.print(", ");
      Serial.print(qx, 4); Serial.print(", ");
      Serial.print(qy, 4); Serial.print(", ");
      Serial.print(qz, 4); Serial.println("]");
      float q_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
      Serial.print("Norm: "); Serial.println(q_norm, 6);
    }
  #endif
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
  
  // Queue calibration response for transmission
  if (!enqueueBLEPacket(CONTROL_CHAR, calData, 13)) {
    #if DEBUG_MODE
      Serial.println("ERROR: Failed to queue calibration data!");
    #endif
  }
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
  
  // Queue identification packet for transmission
  if (!enqueueBLEPacket(CONTROL_CHAR, (uint8_t*)&idPacket, sizeof(NodeIdentificationPacket))) {
    #if DEBUG_MODE
      Serial.println("ERROR: Failed to queue node identification!");
    #endif
  }
  
  #if DEBUG_MODE
    Serial.println();
    Serial.println("=== NODE IDENTIFICATION QUEUED ===");
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
  
  // Queue packet for transmission instead of sending directly
  if (!enqueueBLEPacket(ORIENTATION_CHAR, (uint8_t*)&packet, sizeof(OrientationPacket))) {
    #if DEBUG_MODE
      Serial.println("ERROR: Failed to queue orientation packet!");
    #endif
  }
  
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
      
    case 0xFF:  // Heartbeat/Ping
      handleHeartbeat();
      break;
      
    default:
      #if DEBUG_MODE
        Serial.println("Unknown command!");
      #endif
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
}

void handleStopCommand() {
  #if DEBUG_MODE
    Serial.println(">>> Stopping data collection...");
  #endif
  
  isRunning = false;
}

void handleCalibrateCommand() {
  bool wasRunning = isRunning;
  isRunning = false;  // Stop data collection during calibration
  
  performGyroCalibration();
  
  isRunning = wasRunning;  // Resume if was running
}

void handleSetExerciseCommand(uint8_t* command, int len) {
  if (len < 2) {
    return;
  }
  
  uint8_t exerciseType = command[1];
  
  #if DEBUG_MODE
    Serial.print(">>> Exercise type set to: ");
    Serial.println(exerciseType);
  #endif
  
  // Note: Exercise type is just for logging, stored on RPI side
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
}

void handleTimeSyncCommand(uint8_t* command, int len) {
  if (len < 5) {
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
}

void handleHeartbeat() {
  #if DEBUG_MODE
    Serial.println(">>> Received heartbeat request");
  #endif
  
  // Small delay to ensure client is ready to receive
  delay(10);
  
  // Respond to heartbeat/ping
  uint8_t response[6];
  response[0] = 0xFF;  // Heartbeat response
  response[1] = NODE_ID;
  response[2] = isRunning ? 1 : 0;
  response[3] = isCalibrated ? 1 : 0;
  response[4] = (uint8_t)(timestamp >> 8);
  response[5] = (uint8_t)(timestamp & 0xFF);
  
  // Queue heartbeat response for transmission
  if (!enqueueBLEPacket(CONTROL_CHAR, response, 6)) {
    #if DEBUG_MODE
      Serial.println("ERROR: Failed to queue heartbeat response!");
    #endif
  }
  
  #if DEBUG_MODE
    Serial.println(">>> Heartbeat response queued");
    Serial.print(">>> Response bytes (");
    Serial.print(sizeof(response));
    Serial.print("): ");
    for (int i = 0; i < 6; i++) {
      if (response[i] < 0x10) Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print(">>> Node ID: ");
    Serial.println(NODE_ID);
    Serial.print(">>> Running: ");
    Serial.println(isRunning ? "YES" : "NO");
    Serial.print(">>> Calibrated: ");
    Serial.println(isCalibrated ? "YES" : "NO");
    Serial.print(">>> Timestamp: ");
    Serial.println(timestamp);
  #endif
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

// Restore default struct packing
#pragma pack(pop)