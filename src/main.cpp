#include <Arduino.h>
#include <LittleFS.h>

// Motor Pins
#define enB 14
#define in3 26
#define in4 27

// Encoder Pins
#define encoderA 33
#define encoderB 32

volatile long encoderTicks = 0;
volatile int lastEncoded = 0;
int pwm_value = 0;
int pulsesPerRevolution = 1; // Default value, can be calibrated
float pwmLogging[256];
float speedLogging[256]; // Array to store speed characteristics

float rpm = 0;
float maxRPM = 1;
float minLinRPM = 1;
float maxLinRPM = 1;
int minLinPWM = 0;
int maxLinPWM = 255;

// PID Controller variables
float integral = 0;
float lastError = 0;
unsigned long lastPidTime = 0;

#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderTicks--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderTicks++;

  lastEncoded = encoded;
}

//File handling functions
#define CALIBRATION_FILE "/motor/ppr.bin"
#define MOTOR_CHAR_FILE "/motor/speedLogging.bin"
#define MOTOR_LIN_CHAR "/motor/linearCharacteristics.bin"

// Save integer PPR
bool savePPR() {
    File file = LittleFS.open(CALIBRATION_FILE, "w");
    if (!file) {
        Serial.println("Failed to open ppr.bin for writing");
        return false;
    }
    file.write((uint8_t*)&pulsesPerRevolution, sizeof(pulsesPerRevolution));
    file.close();
    Serial.println("PPR saved.");
    return true;
}

// Load integer PPR
bool loadPPR() {
  File file = LittleFS.open(CALIBRATION_FILE);
  if(!file || file.isDirectory()){
    Serial.println("Failed to open file for reading");
    return false;
  }
  file.read((uint8_t*)&pulsesPerRevolution, sizeof(pulsesPerRevolution));
  file.close();
  return true;
}

// Save float array
bool saveSpeedLogging() {
  File file = LittleFS.open(MOTOR_CHAR_FILE, FILE_WRITE);
  int len = sizeof(speedLogging) / sizeof(speedLogging[0]);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return false;
  }
  file.write((uint8_t*)pwmLogging, sizeof(float) * len); 
  file.write((uint8_t*)speedLogging, sizeof(float) * len);
  file.close();
  //Serial.println("Float array saved");
  return true;
}

// Load float array
bool loadSpeedLogging() {
  File file = LittleFS.open(MOTOR_CHAR_FILE);
  int len = sizeof(speedLogging) / sizeof(speedLogging[0]);
  if(!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }
  file.read((uint8_t*)pwmLogging, sizeof(float) * len);
  file.read((uint8_t*)speedLogging, sizeof(float) * len);
  file.close();
  maxRPM = speedLogging[len-1];
  Serial.println("Float array loaded");
  return true;
}

//Save float linear confguration
bool saveLinChar(){
  File file = LittleFS.open(MOTOR_LIN_CHAR, "w");
  if (!file) {
      Serial.println("Failed to open file for reading");
      return false;
  }

  file.write((uint8_t*)&minLinPWM, sizeof(minLinPWM));
  file.write((uint8_t*)&maxLinPWM, sizeof(maxLinPWM));
  file.write((uint8_t*)&minLinRPM, sizeof(minLinRPM));
  file.write((uint8_t*)&maxLinRPM, sizeof(maxLinRPM));

  file.close();
  Serial.println("Linear characteristics saved");
  return true;
}

//Load float linear configuration
bool loadLinChar(){
  File file = LittleFS.open(MOTOR_LIN_CHAR);
  if (!file) {
      Serial.println("Failed to open file for reading");
      return false;
  }

  file.read((uint8_t*)&minLinPWM, sizeof(minLinPWM));
  file.read((uint8_t*)&maxLinPWM, sizeof(maxLinPWM));
  file.read((uint8_t*)&minLinRPM, sizeof(minLinRPM));
  file.read((uint8_t*)&maxLinRPM, sizeof(maxLinRPM));

  file.close();
  Serial.println("Linear characteristics loaded");
  return true;

}

void initFileSystem() {
    Serial.println("=== Initializing LittleFS ===");
    
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }
    
    // Create motor directory if it doesn't exist
    if (!LittleFS.exists("/motor")) {
        LittleFS.mkdir("/motor");
        Serial.println("Created /motor directory");
    }

    if (LittleFS.exists(CALIBRATION_FILE)) {
      loadPPR();
    }

    if (LittleFS.exists(MOTOR_CHAR_FILE)) {
      loadSpeedLogging();
    }

    if (LittleFS.exists(MOTOR_LIN_CHAR)) {
      loadLinChar();
    }
}

void listLittleFS(const char * dirname = "/", uint8_t levels = 2) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = LittleFS.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels) {
                // --- FIX: Ensure path starts with '/' ---
                String subdir = String(dirname);
                if (!subdir.endsWith("/")) subdir += "/";
                subdir += file.name();
                // Remove duplicate '/' if needed
                subdir.replace("//", "/");
                listLittleFS(subdir.c_str(), levels - 1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void eraseLittleFS() {
    if (LittleFS.begin()) {
        Serial.println("Formatting LittleFS...");
        if (LittleFS.format()) {
            Serial.println("LittleFS formatted successfully!");
        } else {
            Serial.println("LittleFS format failed!");
        }
    } else {
        Serial.println("LittleFS mount failed!");
    }
}

void setup() {
  // Initialize encoder pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, CHANGE);

  // Initialize motor pins
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enB, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // Start with motor off
  digitalWrite(in3, HIGH); //Motor forward
  digitalWrite(in4, LOW);

  Serial.begin(115200);
  delay(1000); // Wait for Serial to initialize
  Serial.setTimeout(50);

  // Initialize file system
  initFileSystem();

  Serial.println("Motor Control System Ready");
  Serial.println("Commands: 'c' for calibration, 's' for speed test");
}

//Motor control functions
void moveMotor(int PWM, int direction) {
  if (direction == 1) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (direction == -1) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW); // Stop motor
  }
  ledcWrite(PWM_CHANNEL, PWM);
}

void calibrateEncoder() {
    int calibrationSpeed = 127; // Default calibration speed in PWM
    int revolutions = 0;
    Serial.println("Calibrating encoder...");
    noInterrupts();
    encoderTicks = 0;
    interrupts();

    // Wait input
    while (true) {
        if (Serial.available()) {
            int command = Serial.parseInt();
            switch(command) {
                case -1:
                case 1:
                  moveMotor(calibrationSpeed, command);
                  //Serial.println("Mov 1 or -1");
                  break;
                case 0:
                  moveMotor(0, 0);
                  //Serial.println("Motor stopped.");
                  break;
                case 2:
                  Serial.println(encoderTicks);
                  break;
                case 3:
                  revolutions = Serial.parseFloat();

                  if (revolutions > 0) {
                      pulsesPerRevolution = abs(encoderTicks) / revolutions;
                      //Serial.print("Calibration result: ");
                      //Serial.println(pulsesPerRevolution);
                      Serial.println(2);
                      Serial.printf("%d %f\n", pulsesPerRevolution, maxRPM);
                      savePPR();
                      //Serial.println(" pulses per revolution.");
                  } else {
                      Serial.println("Calibration failed.");
                  }
                  break;
                case 4:
                  Serial.println(2);
                  Serial.printf("%d %f\n", pulsesPerRevolution, maxRPM);
                  return;
                  break;
                default:
                  Serial.println("Invalid command. Use -1 for reverse, 1 for forward, or 0 to stop.");
                  break;
            }
        }
        //Serial.println(encoderTicks);
    }
    return;
}

float readMotorSpeed(float samplingTimeMillis) {
    if (samplingTimeMillis <= 0) return 0.0;
    
    float totalRevolutions = (float)abs(encoderTicks) / pulsesPerRevolution;
    float speedRPM = totalRevolutions / (samplingTimeMillis / 60000.0);
    return speedRPM;
}

float medianFilter(float* array, int size){
  float temp;
  float arrcpy[size];
  memcpy(arrcpy, array, size * sizeof(float));
  // Sort the array
  for(int i = 0; i < size - 1; i++){
    for(int j = i + 1; j < size; j++){
      if(arrcpy[j] < arrcpy[i]){
        temp = arrcpy[i];
        arrcpy[i] = arrcpy[j];
        arrcpy[j] = temp;
      }
    }
  }
  // Return the median
  if(size % 2 == 0){
    return (arrcpy[size/2 - 1] + arrcpy[size/2]) / 2.0;
  } else {
    return arrcpy[size/2];
  }
}

float MAFilter(float* array, int size) {
  float sum = 0.0;
  for(int i = 0; i < size; i++){
    sum += array[i];
  }
  return sum / size;
}

float findMax(float* array, int size) {
  float max = array[0];
  for(int i = 1; i < size; i++){
    if(array[i] > max){
      max = array[i];
    }
  }
  return max;
}

float findMin(float* array, int size) {
  float min = array[0];
  for(int i = 1; i < size; i++){
    if(array[i] < min){
      min = array[i];
    }
  }
  return min;
}

void readMotorCharacteristic(){
  if (!LittleFS.exists(MOTOR_CHAR_FILE)) {
    Serial.println(0);
  } else {
    Serial.println(1);
    int lenInit = sizeof(speedLogging) / sizeof(speedLogging[0]);
    for (int i = 0; i < lenInit; i++) {
      Serial.printf("%f %f\n", pwmLogging[i], speedLogging[i]);
    }
  }

  while (true) {
    if (Serial.available()){
      int command = Serial.parseInt();
          
      // Clear buffer
      while(Serial.available()){
          Serial.read();
      }

      switch(command) {
          case 1: {
              int PWM = 0;
              int PWM_STEP = 1;
              float progress = 0.0;

              //Times used
              unsigned long startTime, currentTime;
              unsigned long SETTLE_TIME = 1000;
              unsigned long LOGGING_TIME = 8000;
              unsigned long SAMPLING_TIME = 100;

              moveMotor(0,0); //Ensure the motor didn't move when starting
              delay(SETTLE_TIME);

              for(PWM; PWM <= 255; PWM += PWM_STEP){
                pwmLogging[PWM/PWM_STEP] = PWM;
                float medBuff[3] = {0};
                float MABuff[5] = {0};
                float speedHistory[30] = {0};
                int medLen = sizeof(medBuff) / sizeof(medBuff[0]);
                int MALen = sizeof(MABuff) / sizeof(MABuff[0]);
                int speedLen = sizeof(speedHistory) / sizeof(speedHistory[0]);
                int i;

                moveMotor(PWM,1);
                delay(SETTLE_TIME);    

                noInterrupts();
                encoderTicks = 0;
                interrupts();

                startTime = millis();
                currentTime = millis();
                int counter = 0;

                while(millis() - startTime <= LOGGING_TIME){
                  if(millis() - currentTime >= SAMPLING_TIME){
                    float speed = readMotorSpeed(float(millis()-currentTime));
                    noInterrupts();
                    encoderTicks = 0;
                    interrupts();
                    currentTime = millis();
                    

                    for(i = medLen - 1; i > 0; i--){
                      medBuff[i] = medBuff[i - 1];
                    }

                    for(i = MALen - 1; i > 0; i--){
                      MABuff[i] = MABuff[i - 1];
                    }

                    for(i = speedLen - 1; i > 0; i--){
                      speedHistory[i] = speedHistory[i - 1];
                    }

                    medBuff[0] = speed;
                    MABuff[0] = medianFilter(medBuff, medLen);
                    speedHistory[0] = MAFilter(MABuff, MALen);


                    //Check convergence
                    if (counter >= speedLen) {
                      float maxSpeed = findMax(speedHistory, speedLen);
                      float minSpeed = findMin(speedHistory, speedLen);
                      float tolerance = 0.0;
                      if (maxSpeed < 20.0) {
                          tolerance = maxSpeed * 0.10;  // 10% tolerance below 20 RPM
                          tolerance = max(tolerance, 0.5f);  // Minimum 0.5 RPM tolerance
                      } else if (maxSpeed < 100.0) {
                          tolerance = maxSpeed * 0.05;  // 5% tolerance for 20-100 RPM
                      } else {
                          tolerance = maxSpeed * 0.02;  // 2% tolerance above 100 RPM
                          tolerance = min(tolerance, 3.0f);  // Cap at 3 RPM for very high speeds
                      }

                      if (maxSpeed > 0.0 && abs(maxSpeed - minSpeed) <= tolerance) {
                          speedLogging[PWM/PWM_STEP] = speedHistory[0];
                          break;
                      }
                    } else if (counter >= 5 && speedHistory[0] == 0.0 && speedHistory[1] == 0.0 && speedHistory[2] == 0.0) {
                      // Only exit if we have multiple consecutive zero readings
                      speedLogging[PWM/PWM_STEP] = 0.0;
                      break;
                    }
                    counter++;
                  }
                }

                if(speedLogging[PWM/PWM_STEP] != 0.0){
                  moveMotor(0,0);
                  delay(SETTLE_TIME);
                }
                
                //Update progress
                progress = (PWM / 255.0) * 100.0;
                Serial.println(progress);
              }

              moveMotor(0,0);
              int len = sizeof(speedLogging) / sizeof(speedLogging[0]);
              maxRPM = speedLogging[len-1];
              //Serial.println(speedLogging);
              saveSpeedLogging();
              Serial.println(1);
              len = sizeof(speedLogging) / sizeof(speedLogging[0]);
              for (int i = 0; i < len; i++) {
                Serial.printf("%f %f\n", pwmLogging[i], speedLogging[i]);
              }
              break;
          }
          case 2: {
              int len = sizeof(speedLogging) / sizeof(speedLogging[0]);
              Serial.println(1);
              for (int i = 0; i < len; i++) {
                Serial.printf("%f %f\n", pwmLogging[i], speedLogging[i]);
              }
              break;
          }
          case 4:
              Serial.println(2);
              Serial.printf("%d %f\n", pulsesPerRevolution, maxRPM);
              return;
          default:
              Serial.println("Invalid command. Use 1 to start the test or 4 to exit.");
              break;
      }
    }
  }
}

void defineLinearPWMSpeedRelation(){
  if(LittleFS.exists(CALIBRATION_FILE)&&LittleFS.exists(MOTOR_CHAR_FILE)){
      Serial.println(2);
      Serial.printf("%d %f\n", pulsesPerRevolution, maxRPM);
  } else if (LittleFS.exists(CALIBRATION_FILE)) {
      Serial.println(1);
      Serial.printf("%d\n", pulsesPerRevolution);
  } else {
      Serial.println(0);
  }

  while(true){
    if(Serial.available()){
      int command = Serial.parseInt();

      switch(command) {
        case 2:
            if (!LittleFS.exists(MOTOR_LIN_CHAR)) {
              Serial.println(0);
            } else {
              Serial.println(1);
              Serial.printf("%d %d %f %f\n", minLinPWM, maxLinPWM, minLinRPM, maxLinRPM);
            }
            break;
        case 3: {
            minLinPWM = Serial.parseInt();
            maxLinPWM = Serial.parseInt();
            minLinRPM = Serial.parseFloat();
            maxLinRPM = Serial.parseFloat();
            saveLinChar();
            Serial.println(1);
            Serial.printf("%d %d %f %f\n", minLinPWM, maxLinPWM, minLinRPM, maxLinRPM);
            break;
        }
        case 4:
            return;
        default:
            Serial.println("Invalid command. Use 3 to set linear relation or 4 to exit.");
            break;
      }
    }
  }
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  if (value > fromHigh) return toHigh;
  else if (value < fromLow) return toLow;
  else return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void motorOpenLoopControl(){
  while(true){
    if(Serial.available()){
      int command = Serial.parseInt();
      //Serial.println(command);

      switch(command) {
          case 1: {
              float targetSPEED;
              int targetPWM;
              int i;

              //Times used
              unsigned long startTime, currentTime;
              unsigned long SETTLE_TIME = 1000;
              unsigned long LOGGING_TIME = 3000;
              unsigned long SAMPLING_TIME = 10;

              float medBuff[3] = {0};
              float MABuff[5] = {0};
              int medLen = sizeof(medBuff) / sizeof(medBuff[0]);
              int MALen = sizeof(MABuff) / sizeof(MABuff[0]);

              if(Serial.available()){
                targetSPEED = Serial.parseFloat();
                //Serial.println(targetSPEED);
                if(abs(maxRPM - targetSPEED) < 0.1){
                  targetPWM = 255.0;
                } else {
                  targetPWM = (int)mapFloat(targetSPEED, minLinRPM, maxLinRPM, minLinPWM, maxLinPWM);
                }
                moveMotor(0,0);
                delay(SETTLE_TIME);

                noInterrupts();
                encoderTicks = 0;
                interrupts();

                startTime = millis();
                currentTime = millis();

                //Print header
                Serial.println("time_ms,rpm,targetrpm,pwm");
                moveMotor(targetPWM,1);

                while(millis() - startTime <= LOGGING_TIME){
                  if(millis() - currentTime >= SAMPLING_TIME){
                    float speed = readMotorSpeed(float(millis()-currentTime));
                    noInterrupts();
                    encoderTicks = 0;
                    interrupts();
                    currentTime = millis();

                    for(i = medLen - 1; i > 0; i--){
                      medBuff[i] = medBuff[i - 1];
                    }

                    for(i = MALen - 1; i > 0; i--){
                      MABuff[i] = MABuff[i - 1];
                    }

                    medBuff[0] = speed;
                    MABuff[0] = medianFilter(medBuff, medLen);

                    float olSpeed = MAFilter(MABuff, MALen);

                    Serial.printf("%lu,%.2f,%.2f,%d\n", millis()-startTime, olSpeed, targetSPEED, targetPWM);

                  }
                }

                moveMotor(0,0);
                delay(SETTLE_TIME);
                Serial.println("DONE");
              }
            break;
          } 
          case 2:
              if (LittleFS.exists(MOTOR_LIN_CHAR)&&LittleFS.exists(MOTOR_CHAR_FILE)) {
                Serial.println(2);
                Serial.printf("%f %f %f\n", minLinRPM, maxLinRPM, maxRPM);
              } else if (LittleFS.exists(MOTOR_CHAR_FILE)) {
                Serial.println(1);
                Serial.printf("%f\n",maxRPM);
              } else {
                Serial.println(0);
              }
              break;
          case 4:
              return;
          default:
              Serial.println("Invalid command. Use 1 to set PWM and direction, 2 to stop, or 3 to exit.");
              break;
      }
    }
  }
}

void motorClosedLoopControl(){
  while(true){
    if(Serial.available()){
      int command = Serial.parseInt();

      switch(command) {
          case 1: {
              float targetSPEED, currentSPEED;
              float k1, k2, k3;
              int targetPWM;
              float olSpeed, speed;
              int i;

              //Times used
              unsigned long startTime, currentTime;
              unsigned long SETTLE_TIME = 1000;
              unsigned long LOGGING_TIME = 5000;
              unsigned long SAMPLING_TIME = 10;

              float medBuff[5] = {0};
              float MABuff[10] = {0};
              float c[3] = {0};
              float e[3] = {0};
              int medLen = sizeof(medBuff) / sizeof(medBuff[0]);
              int MALen = sizeof(MABuff) / sizeof(MABuff[0]);
              int eLen = sizeof(e) / sizeof(e[0]);
              int cLen = sizeof(c) / sizeof(c[0]);

              if(Serial.available()){
                targetSPEED = Serial.parseFloat();
                SAMPLING_TIME = Serial.parseInt();
                k1 = Serial.parseFloat();
                k2 = Serial.parseFloat();
                k3 = Serial.parseFloat();

                moveMotor(0,0);
                delay(SETTLE_TIME);

                noInterrupts();
                encoderTicks = 0;
                interrupts();

                startTime = millis();
                currentTime = millis();

                //Print header
                Serial.println("time_ms,rpm,targetrpm,pwm");

                //Discrete PID control initializer
                e[2] = targetSPEED; e[1] = targetSPEED; e[0] = targetSPEED;
                c[2] = c[1] + k1 * e[2] + k2 * e[1] + k3 * e[0];
                moveMotor((int)constrain(c[2], 0, 255),1);

                while(millis() - startTime <= LOGGING_TIME){
                  if(millis() - currentTime > SAMPLING_TIME){
                    speed = readMotorSpeed(float(SAMPLING_TIME));
                    noInterrupts();
                    encoderTicks = 0;
                    interrupts();
                    currentTime = millis();

                    for(i = medLen - 1; i > 0; i--){
                      medBuff[i] = medBuff[i - 1];
                    }

                    for(i = MALen - 1; i > 0; i--){
                      MABuff[i] = MABuff[i - 1];
                    }

                    medBuff[0] = speed;
                    MABuff[0] = medianFilter(medBuff, medLen);

                    olSpeed = MAFilter(MABuff, MALen);

                    //Control
                    for (i = 0; i < eLen - 1; i++){
                      e[i] = e[i + 1];
                    }
                    e[eLen - 1] = targetSPEED - olSpeed;
                    for (i = 0; i < cLen - 1; i++){
                      c[i] = c[i + 1];
                    }
                    c[cLen - 1] = c[cLen - 2] + k1 * e[eLen - 1] + k2 * e[eLen - 2] + k3 * e[eLen - 3];
                    targetPWM = (int)constrain(c[cLen - 1], 0, 255);
                    moveMotor(targetPWM,1);

                    Serial.printf("%lu,%.2f,%.2f,%d\n", millis()-startTime, olSpeed, targetSPEED, targetPWM);

                  }
                }
                moveMotor(0,0);
                delay(SETTLE_TIME);
                Serial.println("DONE");
              }
              break;
          }
          case 4:
              return;
  
      }
    }
  }
}

void loop() {
    if(Serial.available()){
        char command = Serial.read();
        
        // Clear buffer
        while(Serial.available()){
            Serial.read();
        }
        
        switch(command) {
            case 'e':
                calibrateEncoder();
                break;
            case 's':  // Speed characteristics
                readMotorCharacteristic();
                break;
            case 'o':  // Open loop
                motorOpenLoopControl();
                break;
            case 'c':  // PID control (your educational approach)
                motorClosedLoopControl();
                break;
            case 'l':  // Linear relation
                defineLinearPWMSpeedRelation();
                break;
            case 'i':
                if(LittleFS.exists(CALIBRATION_FILE)&&LittleFS.exists(MOTOR_CHAR_FILE)){
                    Serial.println(2);
                    Serial.printf("%d %f\n", pulsesPerRevolution, maxRPM);
                } else if (LittleFS.exists(CALIBRATION_FILE)) {
                    Serial.println(1);
                    Serial.printf("%d\n", pulsesPerRevolution);
                } else {
                    Serial.println(0);
                }
                break;
            case 'w':
              listLittleFS();
              break;
            case 'd':
              eraseLittleFS();
              break;
            /*
            default:
                Serial.println("📚 Educational Motor Control Commands:");
                Serial.println("  c = Calibrate encoder");
                Serial.println("  s = Speed characteristics test");
                Serial.println("  o = Open loop control");
                Serial.println("  p = PID control (speed-based)");
                Serial.println("  q = PID control (PWM-based)");
                Serial.println("  l = Define linear speed relation");
                break;
            */
        }
    }
}