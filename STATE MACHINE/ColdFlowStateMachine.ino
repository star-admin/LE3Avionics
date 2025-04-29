#include <MCP23S17.h>
#include <ADS126X.h>
#include <SPI.h>

#define SENSE_CS_1 40
#define SENSE_DRDY_1 4 
#define SENSE_CS_2 42
#define SENSE_DRDY_2 35
#define PYRO_CS_1 16
#define PYRO_CS_2 4
#define MOSI 5
#define MISO 41
#define CLK 13

#define VALVE_FUP 36    // Fuel upstream solenoid open
#define VALVE_FDP 35  // Fuel downstream solenoid open
#define VALVE_OUP 34    // LOX upstream solenoid open
#define VALVE_ODP 33  // LOX downstream solenoid open
#define FUELVENT 37 
#define LOXVENT 38
#define PRESSURELINE 41

#define FUELMAIN 39 //actuators
#define LOXMAIN 40 //actuators

#define UP_PRESSURE 10

// Pressure transducers
// Fuel path
#define PT_I 13  // INJECTOR
#define PT_P 14 //UPSTREAM PRESSURE FOR BOTH FUEL AND OXIDIZER

#define PT_O1 9   // OX TANK PRESSURE 
#define PT_F1 6 // FUEL TANK PRESSURE

// LOX path - new pins need to be defined
#define PT_O2 10   // LOX DOWNSTREAM pressure
#define PT_F2 12 // FUEL DOWNSTREAM pressure

enum STATES { 
  IDLE,
  ARMED,
  PRESS,
  PRESS2,
  ABORT,
  FILL,
  FIRE
};

int STATES[] = {IDLE, ARMED, PRESS, PRESS2, ABORT, FILL, FIRE};

String state_names[] = { "Idle", "Armed", "Press","Press2", "Abort", "Fill", "Fire" };

int DAQState = IDLE;
// int COMState = IDLE;
// int FlightState = IDLE;

bool ethComplete = false;
bool oxComplete = false;
bool oxVentComplete = false;
bool ethVentComplete = false;

bool pressureTankComplete = false;
bool fuelPressComplete = false;
bool oxPressComplete = false;
bool ventComplete = false;
bool pressureReregComplete = false;

float p_fuel_up_filtered;
float p_fuel_down_filtered;


//bool flight_toggle = false;

// Pressure thresholds
//float pressureTankTarget = 150;    // Target pressure for pressure tank
//float pressureFuel = 495;          // Target pressure for fuel tank
// float pressureOx = 465;            // Target pressure for LOX tank
// float abortPressure = 800;         // Pressure threshold for abort condition
// float threshold = 0.98;            // Re-pressurization threshold
// float ventTo = 5;                  // Close solenoids at this pressure to preserve lifetime


ADS126X SENSE_1;
ADS126X SENSE_2;
MCP23S17* PYRO_1_MCP;
MCP23S17* PYRO_2_MCP;


// System parameters from optimization paper
const float ALPHA = 0.1;        // Natural pressure loss rate
const float BETA = 0.5;         // Upstream control penalty
const float DELTA = 0.5;        // Downstream control penalty
const float GAMMA = 0.3;        // Oscillatory control weight
const float LAMBDA = 0.2;       // Instability penalty

// Control states
enum ControlState {
    NOMINAL_REGULATION,
    RECOVERY_DAMPING,
    HIGH_FREQUENCY_OSCILLATION
};

const char* stateStrings[] = {
    "NOMINAL",
    "RECOVERY",
    "OSCILLATION"
};


// Moving average filter
const int FILTER_WINDOW = 20;
float pressure_buffer[8][FILTER_WINDOW];
int buffer_index = 0;

// System state variables
ControlState current_fuel_state = NOMINAL_REGULATION;
float P_threshold_fuel_base = 50.0;
float P_threshold_fuel_down;
float P_threshold_fuel_current;
float P_fuel;
float dP_fuel_dt;
float P_threshold_fuel_up;  // Upstream fuel pressure threshold
float P_threshold_fuel_up_base = P_threshold_fuel_base;

ControlState current_lox_state = NOMINAL_REGULATION;
float P_threshold_lox_base = 50.0; // Adjust base threshold for LOX as needed
float P_threshold_lox_down;
float P_threshold_lox_up;
float P_threshold_lox_current;
float p_lox_up_filtered;
float p_lox_down_filtered;
float P_lox;
float dP_lox_dt;
float P_threshold_lox_up_base = P_threshold_lox_base; // Higher base threshold for upstream LOX


unsigned long last_update = 0;
const int UPDATE_INTERVAL = 50; // 50ms update interval
const unsigned long deltaT = 1000;  // 0.1s in milliseconds

// Threshold optimization parameters
const float DT = UPDATE_INTERVAL / 1000.0;  // Convert to seconds
const float K_P = 0.8;  // Proportional gain
const float K_I = 0.2;  // Integral gain
const float K_D = 0.3;  // Derivative gain
const float MAX_THRESHOLD_CHANGE = 25;
const float STABILITY_EPSILON = 5.0;

// PID state variables
float last_pressure_error_fuel = 0;
float integral_error_fuel = 0;
float last_pressure_error_fuel_up = 0;
float integral_error_fuel_up = 0;

float last_pressure_error_lox = 0;
float integral_error_lox = 0;
float last_pressure_error_lox_up = 0;
float integral_error_lox_up = 0;

// Calibration coefficients
// Pressure Transducer Calibration Coefficients
const float PT_P_A = -5.44825017487633E-09;
const float PT_P_B = 0.0000165145288612233;
const float PT_P_C = 0.17643609939829;
const float PT_P_D = -99.5214963852045;

const float PT_F1_A = -3.05918799222961E-08;
const float PT_F1_B = 0.0000865416133942164;
const float PT_F1_C = 0.123902391819258;
const float PT_F1_D = -80.7759908639505;

const float PT_O1_A = -5.59024700297979E-10;
const float PT_O1_B =  2.95030660576259E-06;
const float PT_O1_C = 0.192622978835518;
const float PT_O1_D = -86.2447994854917;

const float PT_F2_A = 1.76851377283004E-08;
const float PT_F2_B = -0.0000463856993688005;
const float PT_F2_C = 0.23275478320493;
const float PT_F2_D = -97.2678935373566;

const float PT_O2_A = 8.37809910938362E-10;
const float PT_O2_B =  8.49581367227712E-06;
const float PT_O2_C = 0.183688920248558;
const float PT_O2_D =  -70.1194944330298;

// LOX pressure transducers calibration - replace with actual coefficients
const float PT_I_A = -5.44825017487633E-09; // Replace with actual calibration data
const float PT_I_B = 0.0000165145288612233;
const float PT_I_C = 0.17643609939829;
const float PT_I_D = -99.5214963852045;


float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D) {
    return (PT_A * pow(raw_value, 3)) +
           (PT_B * pow(raw_value, 2)) +
           (PT_C * raw_value) + PT_D;
}

bool upstream_solenoid_state = false; //CHANGE TO FUEL
bool downstream_solenoid_state = false;
bool lox_upstream_solenoid_state = false;
bool lox_downstream_solenoid_state = false;

float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D);
float updateMovingAverage(float new_value, float* buffer);
void calculatePressureDerivative(float current_pressure);
float calculateInstability(float error, float error_derivative);

void determineControlStateFuel();
void determineControlStateLOX();
void updateDynamicThresholdFuel();
void updateDynamicThresholdLOX();
void applyOptimalControlFuel();
void applyOptimalControlLOX();

// float calculatePressure(float raw_value, float PT_A, float PT_B, float PT_C, float PT_D) {
//     return (PT_A * pow(raw_value, 3)) +
//            (PT_B * pow(raw_value, 2)) +
//            (PT_C * raw_value) + PT_D;
// }

float updateMovingAverage(float new_value, float* buffer) {
    buffer[buffer_index] = new_value;
    float sum = 0;
    for(int i = 0; i < FILTER_WINDOW; i++) {
        sum += buffer[i];
    }
    return sum / FILTER_WINDOW;
}

void calculatePressureDerivative(float current_pressure) {
    static float last_pressure = current_pressure;
    static unsigned long last_time = millis();
    
    if (isnan(current_pressure)) {
        //Serial.println("NaN detected in current_pressure");
        return;
    }
    
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    
    // Prevent division by zero or very small dt
    if (dt > 0.001) {
        float new_derivative = (current_pressure - last_pressure) / dt;
        if (!isnan(new_derivative)) {
            dP_fuel_dt = new_derivative;
        }
    }
    
    last_pressure = current_pressure;
    last_time = current_time;
}

float calculateInstability(float error, float error_derivative) {
    // Add bounds checking
    if (isnan(error) || isnan(error_derivative)) {
        return 0.0;
    }
    
    if (STABILITY_EPSILON == 0 || MAX_THRESHOLD_CHANGE == 0) {
        return 0.0;
    }
    
    float term1 = pow(error/STABILITY_EPSILON, 2);
    float term2 = pow(error_derivative/MAX_THRESHOLD_CHANGE, 2);
    
    if (isnan(term1) || isnan(term2)) {
        return 0.0;
    }
    
    return sqrt(term1 + term2);
}

void updateDynamicThresholdFuel() {
    // Calculate pressure error and rate of change
    float pressure_error = P_fuel - P_threshold_fuel_down;
    float target_adjustment = 0.0;
    
    // If system pressure is consistently away from threshold, gradually move threshold
    if (abs(pressure_error) > STABILITY_EPSILON) {
        // Calculate desired threshold movement based on current pressure
        target_adjustment = pressure_error * 0.05; // 5% movement toward actual pressure
        
        // Add derivative term to anticipate pressure trends
        target_adjustment += (dP_fuel_dt * DT * 0.35 / 2); // 35% weight on trend, divided by 2 for derivative correction
        
        // Limit maximum adjustment per cycle
        target_adjustment = constrain(target_adjustment, -MAX_THRESHOLD_CHANGE * DT, MAX_THRESHOLD_CHANGE * DT);
        
        // Update threshold with smoothing
        P_threshold_fuel_down = P_threshold_fuel_down + target_adjustment;
        
        // Ensure threshold stays within acceptable bounds of base threshold
        P_threshold_fuel_down = constrain(P_threshold_fuel_down,
                                   P_threshold_fuel_base * 0.5,  // Allow more downward movement
                                   P_threshold_fuel_base * 1.7); // Allow more upward movement
    }
    
    // Update upstream threshold with wider bounds
    float pressure_error_up = P_fuel - P_threshold_fuel_up;
    if (abs(pressure_error_up) > STABILITY_EPSILON * 1.2) {
        float target_adjustment_up = pressure_error_up * 0.25; // 25% movement toward actual pressure
        target_adjustment_up += (dP_fuel_dt * DT * 0.10 / 2); // 10% weight on trend, divided by 2 for derivative correction
        
        target_adjustment_up = constrain(target_adjustment_up, 
                                       -MAX_THRESHOLD_CHANGE * DT * 1.5,
                                       MAX_THRESHOLD_CHANGE * DT * 1.5);
        
        P_threshold_fuel_up = P_threshold_fuel_up + target_adjustment_up;
        P_threshold_fuel_up = constrain(P_threshold_fuel_up,
                                 P_threshold_fuel_up_base * 0.6,
                                 P_threshold_fuel_up_base * 1.5);
    }
    
    P_threshold_fuel_up_base = P_threshold_fuel_up;
    P_threshold_fuel_base = P_threshold_fuel_down;
    
    // Update current threshold
    P_threshold_fuel_current = P_threshold_fuel_down;
}


void updateDynamicThresholdLOX() {
    // Calculate pressure error and rate of change for LOX
    float pressure_error = P_lox - P_threshold_lox_down;
    float target_adjustment = 0.0;
    
    // If system pressure is consistently away from threshold, gradually move threshold
    if (abs(pressure_error) > STABILITY_EPSILON) {
        // Calculate desired threshold movement based on current pressure
        target_adjustment = pressure_error * 0.05; // 5% movement toward actual pressure
        
        // Add derivative term to anticipate pressure trends
        target_adjustment += (dP_lox_dt * DT * 0.35 / 2); // 35% weight on trend, divided by 2 for derivative correction
        
        // Limit maximum adjustment per cycle
        target_adjustment = constrain(target_adjustment, -MAX_THRESHOLD_CHANGE * DT, MAX_THRESHOLD_CHANGE * DT);
        
        // Update threshold with smoothing
        P_threshold_lox_down = P_threshold_lox_down + target_adjustment;
        
        // Ensure threshold stays within acceptable bounds of base threshold
        P_threshold_lox_down = constrain(P_threshold_lox_down,
                                   P_threshold_lox_base * 0.5,  // Allow more downward movement
                                   P_threshold_lox_base * 1.7); // Allow more upward movement
    }
    
    // Update upstream threshold with wider bounds
    float pressure_error_up = P_lox - P_threshold_lox_up;
    if (abs(pressure_error_up) > STABILITY_EPSILON * 1.2) {
        float target_adjustment_up = pressure_error_up * 0.25; // 25% movement toward actual pressure
        target_adjustment_up += (dP_lox_dt * DT * 0.10 / 2); // 10% weight on trend, divided by 2 for derivative correction
        
        target_adjustment_up = constrain(target_adjustment_up, 
                                       -MAX_THRESHOLD_CHANGE * DT * 1.5,
                                       MAX_THRESHOLD_CHANGE * DT * 1.5);
        
        P_threshold_lox_up = P_threshold_lox_up + target_adjustment_up;
        P_threshold_lox_up = constrain(P_threshold_lox_up,
                                 P_threshold_lox_up_base * 0.6,
                                 P_threshold_lox_up_base * 1.5);
    }
    
    P_threshold_lox_up_base = P_threshold_lox_up;
    P_threshold_lox_base = P_threshold_lox_down;
    
    // Update current threshold
    P_threshold_lox_current = P_threshold_lox_down;
}

void determineControlStateFuel() {
    float instability = calculateInstability(P_fuel - P_threshold_fuel_current, dP_fuel_dt);
    
    if (instability < 0.5) {
        current_fuel_state = NOMINAL_REGULATION;
    } else if (dP_fuel_dt < -8.0 || instability > 0.5) {
        current_fuel_state = RECOVERY_DAMPING;
    } else if (abs(dP_fuel_dt) > 10.0 || instability > 1) {
        current_fuel_state = HIGH_FREQUENCY_OSCILLATION;
    }
}

void determineControlStateLOX() {
    float instability = calculateInstability(P_lox - P_threshold_lox_current, dP_lox_dt);
    
    if (instability < 0.5) {
        current_lox_state = NOMINAL_REGULATION;
    } else if (dP_lox_dt < -8.0 || instability > 0.5) {
        current_lox_state = RECOVERY_DAMPING;
    } else if (abs(dP_lox_dt) > 10.0 || instability > 1) {
        current_lox_state = HIGH_FREQUENCY_OSCILLATION;
    }
}

void applyOptimalControlFuel() {
    static unsigned long closeStartTime_fuel = 0;
    
    // Calculate pressure errors for both thresholds
    float pressure_error_down = P_threshold_fuel_down - P_fuel;
    float pressure_error_up = P_threshold_fuel_up - P_fuel;
    
    // Calculate optimal control actions using both errors
    float u_up = -pressure_error_up / (2 * BETA);  // Use upstream error for upstream control
    float u_down = pressure_error_down / (2 * DELTA);  // Use downstream error for downstream control
    
    // Add oscillatory control term with modified behavior
    if (current_fuel_state == HIGH_FREQUENCY_OSCILLATION) {
        float oscillation_term = GAMMA * (dP_fuel_dt > 0 ? 1 : -1);
        
        // Apply different oscillation factors based on pressure region
        if (P_fuel > P_threshold_fuel_up) {
            u_up += oscillation_term * 1.2;  // Stronger oscillation for high pressure
            u_down += oscillation_term * 0.8;
        } else if (P_fuel < P_threshold_fuel_down) {
            u_up += oscillation_term * 0.8;
            u_down += oscillation_term * 1.2;  // Stronger oscillation for low pressure
        } else {
            u_up += oscillation_term;
            u_down += oscillation_term;
        }
    }
    
    // Apply state-based control strategy with enhanced logic
    switch (current_fuel_state) {
        case NOMINAL_REGULATION:
            if (P_fuel > P_threshold_fuel_up * 1.1) {  // 10% tolerance
                PYRO_1_MCP->write1(VALVE_FUP, 0);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, 1);
            } else if (P_fuel < P_threshold_fuel_down * 0.9) {  // 10% tolerance
                PYRO_1_MCP->write1(VALVE_FUP, 1);
                PYRO_1_MCP->write1(VALVE_FDP, 0);// Close upstream
      // Close downstream
            } else {
              if (millis() - closeStartTime_fuel >= deltaT) {
                  PYRO_1_MCP->write1(VALVE_FUP, u_up > 0 ? 1.0 : 0);// Close upstream
                  PYRO_1_MCP->write1(VALVE_FDP, u_down > 0 ? 1.0 : 0);// Close upstream
              } else {
                  PYRO_1_MCP->write1(VALVE_FUP, u_up > 0 ? 1.0 : 0);// Close upstream
                  PYRO_1_MCP->write1(VALVE_FDP, u_down > 0 ? 1.0 : 0);// Close upstream

              }
            }
            break;
            
        case RECOVERY_DAMPING:
            // Enhanced recovery control
            closeStartTime_fuel = 0;

            if (dP_fuel_dt < -10) {  // Fast pressure drop
                PYRO_1_MCP->write1(VALVE_FUP, 1);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, 0);// Close upstream
                closeStartTime_fuel = millis();    // Store time when closed
            } else {
                // Check if the solenoid has been closed long enough
                if (millis() - closeStartTime_fuel >= deltaT) {
                    PYRO_1_MCP->write1(VALVE_FUP, u_up > 0 ? 1.0 : 0);// Close upstream
                    PYRO_1_MCP->write1(VALVE_FDP, u_down > 0 ? 1.0 : 0);// Close upstream
                } else {
                    PYRO_1_MCP->write1(VALVE_FDP, 0);// Close upstream

                }
            }
            break;
            
        case HIGH_FREQUENCY_OSCILLATION:
            // Implement bang-bang control
            if (P_fuel > P_threshold_fuel_up) {
                PYRO_1_MCP->write1(VALVE_FUP, 0);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, 1);// Close upstream
            } else if (P_fuel < P_threshold_fuel_down) {
                PYRO_1_MCP->write1(VALVE_FUP, 1);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, 0);// Close upstream

            } else {
              if (millis() - closeStartTime_fuel >= deltaT) {
                PYRO_1_MCP->write1(VALVE_FUP, u_up > 0 ? 1.0 : 0);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, u_down > 0 ? 1.0 : 0);// Close upstream
              } else {
                PYRO_1_MCP->write1(VALVE_FUP, u_up > 0 ? 1.0 : 0);// Close upstream
                PYRO_1_MCP->write1(VALVE_FDP, u_down > 0 ? 1.0 : 0);// Close upstream
              }
            }
            break;
    }
}

void applyOptimalControlLOX() {
    static unsigned long closeStartTime_lox = 0;
    
    // Calculate pressure errors for both thresholds
    float pressure_error_down = P_threshold_lox_down - P_lox;
    float pressure_error_up = P_threshold_lox_up - P_lox;
    
    // Calculate optimal control actions using both errors
    float u_up = -pressure_error_up / (2 * BETA);  // Use upstream error for upstream control
    float u_down = pressure_error_down / (2 * DELTA);  // Use downstream error for downstream control
    
    // Add oscillatory control term with modified behavior
    if (current_lox_state == HIGH_FREQUENCY_OSCILLATION) {
        float oscillation_term = GAMMA * (dP_lox_dt > 0 ? 1 : -1);
        
        // Apply different oscillation factors based on pressure region
        if (P_lox > P_threshold_lox_up) {
            u_up += oscillation_term * 1.2;  // Stronger oscillation for high pressure
            u_down += oscillation_term * 0.8;
        } else if (P_lox < P_threshold_lox_down) {
            u_up += oscillation_term * 0.8;
            u_down += oscillation_term * 1.2;  // Stronger oscillation for low pressure
        } else {
            u_up += oscillation_term;
            u_down += oscillation_term;
        }
    }
    
    // Apply state-based control strategy with enhanced logic
    switch (current_lox_state) {
        case NOMINAL_REGULATION:
            if (P_lox > P_threshold_lox_up * 1.1) {  // 10% tolerance
              PYRO_1_MCP->write1(VALVE_OUP, 0);// Close upstream
              PYRO_1_MCP->write1(VALVE_ODP, 1);// Close upstream

            } else if (P_lox < P_threshold_lox_down * 0.9) {  // 10% tolerance
                PYRO_1_MCP->write1(VALVE_OUP, 1);// Close upstream
                PYRO_1_MCP->write1(VALVE_ODP, 0);// Close upstream
            } else {
              if (millis() - closeStartTime_lox >= deltaT) {
                PYRO_1_MCP->write1(VALVE_OUP, u_up > 0 ? 1.0 : 0);// Close upstream
                PYRO_1_MCP->write1(VALVE_ODP, u_down > 0 ? 1.0 : 0);

              } else {
                PYRO_1_MCP->write1(VALVE_OUP, u_up > 0 ? 1.0 : 0);
                PYRO_1_MCP->write1(VALVE_ODP, u_down > 0 ? 1.0 : 0);
              }
            }
            break;
            
        case RECOVERY_DAMPING:
            // Enhanced recovery control
            closeStartTime_lox = 0;

            if (dP_lox_dt < -10) {  // Fast pressure drop
              PYRO_1_MCP->write1(VALVE_OUP, 1);
              PYRO_1_MCP->write1(VALVE_ODP, 0);

              closeStartTime_lox = millis();    // Store time when closed
            } else {
                // Check if the solenoid has been closed long enough
                if (millis() - closeStartTime_lox >= deltaT) {
                    PYRO_1_MCP->write1(VALVE_OUP, u_up > 0 ? 1.0 : 0);
                    PYRO_1_MCP->write1(VALVE_ODP, u_down> 0 ? 1.0 : 0);

                } else {
                    PYRO_1_MCP->write1(VALVE_ODP, 0);
                }
            }
            break;
            
        case HIGH_FREQUENCY_OSCILLATION:
            // Implement bang-bang control
            if (P_lox > P_threshold_lox_up) {
                PYRO_1_MCP->write1(VALVE_OUP, 0);
                PYRO_1_MCP->write1(VALVE_ODP, 1);
            } else if (P_lox < P_threshold_lox_down) {
                PYRO_1_MCP->write1(VALVE_OUP, 1);
                PYRO_1_MCP->write1(VALVE_ODP, 0);

            } else {
              if (millis() - closeStartTime_lox >= deltaT) {
                PYRO_1_MCP->write1(VALVE_OUP, u_up > 0 ? 1.0 : 0);
                PYRO_1_MCP->write1(VALVE_ODP, u_down > 0 ? 1.0 : 0);
              } else {
                PYRO_1_MCP->write1(VALVE_OUP, u_up > 0 ? 1.0 : 0);
                PYRO_1_MCP->write1(VALVE_ODP, u_down > 0 ? 1.0 : 0);

              }
            }
            break;
    }
}

float readPT(int channel) {
  delay(10);
  SENSE_1.readADC1(channel, ADS126X_AINCOM);
  delay(10);
  long raw = SENSE_1.readADC1(channel, ADS126X_AINCOM);
  float voltage = (float)raw * 5.0 / 2147483648.0;
  return voltage;
}


void SyncSerial() {
  if (Serial.available() > 0) {
  // Serial.read reads a single character as ASCII. Number 1 is 49 in ASCII.
  // Serial sends character and new line character "\n", which is 10 in ASCII.
  int SERIALState = Serial.read() - 48;
  if (SERIALState >= 0 && SERIALState <= sizeof(STATES) / sizeof(STATES[0])) {

    DAQState = SERIALState;
  }
  }
  Serial.print("DAQState:   ");
  Serial.println(DAQState);
}

void mosfetCloseAllValves() {
    for (int i = 0; i < 9; i++) {
      PYRO_1_MCP->write1(i, 0);
    }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SPI.begin(CLK, MISO, MOSI, -1);
  delay(500);

  PYRO_1_MCP = new MCP23S17(PYRO_CS_1, 0x00, &SPI);
  PYRO_2_MCP = new MCP23S17(PYRO_CS_2, 0x00, &SPI);

  SENSE_1.begin(SENSE_CS_1);
  SENSE_1.startADC1();
  SENSE_1.setRate(ADS126X_RATE_1200);
  SENSE_1.setReference(ADS126X_REF_NEG_VSS, ADS126X_REF_POS_VDD);
  uint8_t power = SENSE_1.readRegister(ADS126X_POWER);
  power &= ~(1 << 2);
  SENSE_1.writeRegister(ADS126X_POWER);

  bool status = PYRO_1_MCP->begin();
  Serial.println(status ? "Started Pyro 1 GPIO EX" : "Failed to start Pyro 1 GPIO EX");
  delay(100);
  for (int pin = 0; pin < 7; pin++) {
    PYRO_1_MCP->pinMode8(pin, 0x00);
    PYRO_1_MCP->write1(pin, 0);
  }

  status = PYRO_2_MCP->begin();
  Serial.println(status ? "Started Pyro 2 GPIO EX" : "Failed to start Pyro 2 GPIO EX");
  delay(100);
  for (int pin = 0; pin < 7; pin++) {
    PYRO_2_MCP->pinMode8(pin, 0x00);
    PYRO_2_MCP->write1(pin, 0);
  }

  P_threshold_fuel_current = P_threshold_fuel_base;
  P_threshold_fuel_down = P_threshold_fuel_base;

  P_threshold_lox_current = P_threshold_lox_base;
  P_threshold_lox_down = P_threshold_lox_base;  


}

void loop() {
  //applyOptimalControl();
  SyncSerial(); //sends states from gui
  stringConstructor();



// PT readings (FUEL UPSTREAM, FUEL DOWNSTREAM, LOX UPSTREAM, LOX DOWNSTREAM, PT-P, PT-I)
  switch (DAQState) {
    case (IDLE):
      idle();
      break;

    case (ARMED):
      armed();
      break;

    case (PRESS):
      press();
      break;

    case (PRESS2):
      press2();
      break;

    case (ABORT):
      abort_sequence();
      break;
      
    case (FILL):
      fill();
      break;
      
    case (FIRE):
      fire();
      break;
  }

  delay(50);
}

void reset() {
  oxComplete = false;
  ethComplete = false;
  oxVentComplete = false;
  ethVentComplete = false;
}

void idle() {
  mosfetCloseAllValves();
  oxComplete = false;
  ethComplete = false;
  reset(); 
  PYRO_1_MCP->write1(LOXVENT, 0);
  PYRO_1_MCP->write1(FUELVENT, 0);
  delay(50);
  PYRO_1_MCP->write1(FUELMAIN, 0);
  delay(50);
  PYRO_1_MCP->write1(LOXMAIN, 0);
}

void armed() {
  mosfetCloseAllValves();
  delay(50);
  PYRO_1_MCP->write1(LOXVENT, 0);
  delay(50);
  PYRO_1_MCP->write1(FUELVENT, 0);
  delay(50);
  PYRO_1_MCP->write1(FUELMAIN, 0);
  delay(50);
  PYRO_1_MCP->write1(LOXMAIN, 0);
}

void press(){

    PYRO_1_MCP->write1(LOXVENT, 0);
    delay(50);
    PYRO_1_MCP->write1(FUELVENT, 0);
    delay(50);
    PYRO_1_MCP->write1(FUELMAIN, 0);
    delay(50);
    PYRO_1_MCP->write1(LOXMAIN, 0);
    delay(50);
    PYRO_1_MCP->write1(PRESSURELINE, 1);
    //float p_fuel_up = calculatePressure(readPT(PT_P), PT_FUEL_1_A, PT_FUEL_1_B, PT_FUEL_1_C, PT_FUEL_1_D);

}

void press2() {
  PYRO_1_MCP->write1(LOXVENT, 0);
  delay(50);
  PYRO_1_MCP->write1(FUELVENT, 0);
  delay(50);
  PYRO_1_MCP->write1(PRESSURELINE, 0); 
  delay(50);
  PYRO_1_MCP->write1(VALVE_FUP, 1);
  delay(50);
  PYRO_1_MCP->write1(VALVE_OUP, 1);
  delay(50);
  PYRO_1_MCP->write1(FUELMAIN, 0);
  delay(50);
  PYRO_1_MCP->write1(LOXMAIN, 0);

  while (p_fuel_up_filtered <= UP_PRESSURE) {
    PYRO_1_MCP->write1(VALVE_FUP, 1);
    delay(50);
    PYRO_1_MCP->write1(VALVE_OUP, 1);
    PYRO_1_MCP->write1(LOXVENT, 0);
    delay(50);
    PYRO_1_MCP->write1(FUELVENT, 0);
    ethComplete = true;
    oxComplete = true;
    delay(50);
  }
  PYRO_1_MCP->write1(VALVE_FUP, 0);
  delay(50);
  PYRO_1_MCP->write1(VALVE_OUP, 0);
  delay(50);
  PYRO_1_MCP->write1(PRESSURELINE, 1); //reopen pressure tank back

}

void abort_sequence(){
  //dump all fluids
  //mains (actuators) need to be open! dump all fluids
  PYRO_1_MCP->write1(FUELMAIN, 1);
  delay(50);
  PYRO_1_MCP->write1(LOXMAIN, 1);
  delay(50);
  PYRO_1_MCP->write1(VALVE_FDP, 1);
  delay(50);
  vent();
  //PYRO_2_MCP->write1(VALVE_ODP, 1); not for cold from 4/26
  //downstream solenoids for lox and fuel need to be open

  oxVentComplete = true;
  ethVentComplete = true;

}

void vent(){
  //all other solenoids are closed
    PYRO_1_MCP->write1(PRESSURELINE, 0); //WILL BE REMOVED FROM THE SYSTEM
    delay(50);
    PYRO_1_MCP->write1(LOXVENT, 1);
    delay(50);
    PYRO_1_MCP->write1(FUELVENT, 1);
    

    while (P_fuel > 0){
      if (P_fuel > 800) {
        PYRO_1_MCP->write1(VALVE_FUP, 0);
        delay(50);
        PYRO_1_MCP->write1(VALVE_OUP, 0);
    }
      vent();
    }
}

void fill(){
    PYRO_1_MCP->write1(LOXVENT, 1);
    PYRO_1_MCP->write1(FUELVENT, 1);
    PYRO_1_MCP->write1(VALVE_FUP, 0);
    PYRO_1_MCP->write1(VALVE_OUP, 0);

    PYRO_1_MCP->write1(FUELMAIN, 0);
    PYRO_1_MCP->write1(LOXMAIN, 0);

    //PYRO_2_MCP->write1(VALVE_ODP, 1); COLD FLOW 4/26
    PYRO_1_MCP->write1(VALVE_FDP, 1);
}

void fire() {
  calculatePressureDerivative(P_fuel);
  calculatePressureDerivative(P_lox);
  updateDynamicThresholdLOX();
  updateDynamicThresholdFuel();
  determineControlStateLOX();
  determineControlStateFuel();
  applyOptimalControlLOX();
  applyOptimalControlFuel();
  PYRO_1_MCP->write1(LOXVENT, 0);
  PYRO_1_MCP->write1(FUELVENT, 0);

  PYRO_1_MCP->write1(FUELMAIN, 1);
  PYRO_1_MCP->write1(LOXMAIN, 1);
}

void stringConstructor() {
  float p_fuel_up = calculatePressure(readPT(PT_P), PT_P_A, PT_P_B, PT_P_C, PT_P_D);
  float p_fuel_tank = calculatePressure(readPT(PT_F1), PT_F1_A, PT_F1_B, PT_F1_C, PT_F1_D);
  float p_fuel_down = calculatePressure(readPT(PT_F2), PT_F2_A, PT_F2_B, PT_F2_C, PT_F2_D);
  // Read and calculate pressures for LOX path
  float p_lox_up = calculatePressure(readPT(PT_P), PT_P_A, PT_P_B, PT_P_C, PT_P_D);
  float p_lox_tank = calculatePressure(readPT(PT_O1),PT_O1_A, PT_O1_B, PT_O1_C, PT_O1_D);
  float p_lox_down = calculatePressure(readPT(PT_O2), PT_O2_A, PT_O2_B, PT_O2_C, PT_O2_D);
  float p_inj = calculatePressure(readPT(PT_I),PT_I_A, PT_I_B, PT_I_C, PT_I_D);
  
  P_fuel = updateMovingAverage(p_fuel_tank, pressure_buffer[1]);
  p_fuel_down_filtered = updateMovingAverage(p_fuel_down, pressure_buffer[2]);

  float p_lox_up_filtered = updateMovingAverage(p_lox_up, pressure_buffer[3]);
  P_lox = updateMovingAverage(p_lox_tank, pressure_buffer[4]);
  float p_lox_down_filtered = updateMovingAverage(p_lox_down, pressure_buffer[5]);
  float p_i_filtered = updateMovingAverage(p_inj, pressure_buffer[6]);

  buffer_index = (buffer_index + 1) % FILTER_WINDOW;

  Serial.print(p_fuel_up_filtered); Serial.print(", ");
  Serial.print(P_fuel); Serial.print(", ");
  Serial.print(p_fuel_down_filtered);Serial.print(", ");
  Serial.print(P_threshold_fuel_down);Serial.print(", ");
  Serial.print(P_threshold_fuel_up); Serial.print(", ");

  Serial.print(p_lox_up_filtered); Serial.print(", ");
  Serial.print(p_lox_down_filtered); Serial.print(", ");
  Serial.print(P_lox); Serial.print(" ");
  Serial.print(P_threshold_lox_down); Serial.print(", ");
  Serial.print(P_threshold_lox_up); Serial.print(", ");


  calculatePressureDerivative(P_fuel);
  Serial.print(dP_fuel_dt); Serial.print(", ");
  Serial.print(calculateInstability(P_fuel - P_threshold_fuel_current, dP_fuel_dt)); Serial.print(", ");
  
  calculatePressureDerivative(P_lox);
  Serial.print(dP_lox_dt); Serial.print(", ");
  Serial.print(calculateInstability(P_lox - P_threshold_lox_current, dP_lox_dt)); Serial.print(", ");

  Serial.print(stateStrings[current_fuel_state]); Serial.print(", ");
  Serial.print(stateStrings[current_lox_state]); Serial.print(", ");

}






