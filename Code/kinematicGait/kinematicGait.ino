// analog input from pot wiper
const int potA   = A0; 
const int potB   = A2; 
const int potC   = A5; 

const int adcMinA = 75;     // ADC at 0°
const int adcMaxA = 387;    // ADC at 180°

// MOTOR NAMES:
// FR: ABC
// BR: GHI
// BL: PQR
// FL: JKL

// PORT NAMES:
// A: 0, B: 1, C: 2
// G: 4, H: 5, I: 6
// P: 20, Q: 21, R: 22
// J: 24, K: 25, L: 26

// EVERY DISTANCE IS IN CENTIMETERS, IN CONTRAST TO MATLAB IN WHICH IT IS IN METERS
// these work for all four legs, just need a different sign for y offset
const int x_offset = -2;
const int y_offset = 5;
// const int z_offset = 0;

// link lengths
const float L1 = 3.1;
const float L2 = 4.5;
const float L3 = 3.0;
const float L4 = 9.0;

// // x axis to the right of the robot, y axis to the front
// float X_dist_fr = 5;
// float X_dist_br = 4.5;
// float X_dist_fl = -5;
// float X_dist_bl = -4.5;
// float Y_dist_f = 10;
// float Y_dist_b = -10;

const int T_STALL = 2; // transition time bw swing and stance
const int NUM_DATA_POINTS = 16; // total samples in swing and stance

// bezier curve parameters
const float X = 12;
const float S = -11;
const float A = 4;
const float T = 10;

// bezier curve points
const float P1[2] = {-1*T/2, S};
const float P2[2] = {0, S + 2*A};
const float P3[2] = {T/2, S};

// move one motor with a pwm position and speed
void moveOne(int motor, int position, int duration = 1000) 
{
  Serial.print("#");
  Serial.print(motor);
  Serial.print(" P");
  Serial.print(position);
  Serial.print(" T");
  Serial.print(duration);
  Serial.print("\r");
  // delay(duration + 50);
}

// generate linearly spaced array
void linspace(float start, float end, int num, float* arr) 
{
  if (num == 1) {
    arr[0] = start;
    return;
  }
  float step = (end - start) / (num - 1);
  for (int i = 0; i < num; i++) {
    arr[i] = start + step * i;
  }
}

void theta_to_pwm(float theta1, float theta2, float theta4, int* pwm, int leg_ind) 
{ 
  const int PERCENT_TO_MICROSECONDS = 200; // 1% duty cycle is 200 microseconds
  const int hip_pwm_lower_bounds[4] = {4*PERCENT_TO_MICROSECONDS, 4*PERCENT_TO_MICROSECONDS, 4*PERCENT_TO_MICROSECONDS, 4*PERCENT_TO_MICROSECONDS};
  const int hip_pwm_upper_bounds[4] = {11.5*PERCENT_TO_MICROSECONDS, 11.5*PERCENT_TO_MICROSECONDS, 11*PERCENT_TO_MICROSECONDS, 11*PERCENT_TO_MICROSECONDS};
  const int knee_pwm_lower_bounds[4] = {3*PERCENT_TO_MICROSECONDS, 2.7*PERCENT_TO_MICROSECONDS, 2.5*PERCENT_TO_MICROSECONDS, 3*PERCENT_TO_MICROSECONDS};
  const int knee_pwm_upper_bounds[4] = {12.8*PERCENT_TO_MICROSECONDS, 12.2*PERCENT_TO_MICROSECONDS, 12.2*PERCENT_TO_MICROSECONDS, 12.5*PERCENT_TO_MICROSECONDS};
  const int ankle_pwm_lower_bounds[4] = {2.8*PERCENT_TO_MICROSECONDS, 3*PERCENT_TO_MICROSECONDS, 2.5*PERCENT_TO_MICROSECONDS, 2.5*PERCENT_TO_MICROSECONDS};
  const int ankle_pwm_upper_bounds[4] = {13*PERCENT_TO_MICROSECONDS, 13*PERCENT_TO_MICROSECONDS, 12.5*PERCENT_TO_MICROSECONDS, 12*PERCENT_TO_MICROSECONDS};

  int hip_pwm_mid = (hip_pwm_lower_bounds[leg_ind] + hip_pwm_upper_bounds[leg_ind]) / 2;
  int knee_pwm_mid = (knee_pwm_lower_bounds[leg_ind] + knee_pwm_upper_bounds[leg_ind]) / 2;
  int ankle_pwm_mid = (ankle_pwm_lower_bounds[leg_ind] + ankle_pwm_upper_bounds[leg_ind]) / 2;
  float pwm_per_radian_hip = (hip_pwm_upper_bounds[leg_ind] - hip_pwm_lower_bounds[leg_ind]) / PI;
  float pwm_per_radian_knee = (knee_pwm_upper_bounds[leg_ind] - knee_pwm_lower_bounds[leg_ind]) / PI;
  float pwm_per_radian_ankle = (ankle_pwm_upper_bounds[leg_ind] - ankle_pwm_lower_bounds[leg_ind]) / PI;

  if (hip_pwm_mid + theta1 * pwm_per_radian_hip < hip_pwm_lower_bounds[leg_ind]) {
    pwm[0] = hip_pwm_lower_bounds[leg_ind];
  } else if (hip_pwm_mid + theta1 * pwm_per_radian_hip > hip_pwm_upper_bounds[leg_ind]) {
    pwm[0] = hip_pwm_upper_bounds[leg_ind];
  } else {
    pwm[0] = hip_pwm_mid + theta1 * pwm_per_radian_hip;
  }

  if (knee_pwm_mid + theta2 * pwm_per_radian_knee < knee_pwm_lower_bounds[leg_ind]) {
    pwm[1] = knee_pwm_lower_bounds[leg_ind];
  } else if (knee_pwm_mid - theta2 * pwm_per_radian_knee > knee_pwm_upper_bounds[leg_ind]) {
    pwm[1] = knee_pwm_upper_bounds[leg_ind];
  } else {
    pwm[1] = knee_pwm_mid + theta2 * pwm_per_radian_knee;
  }

  if (ankle_pwm_mid + theta4 * pwm_per_radian_ankle < ankle_pwm_lower_bounds[leg_ind]) {
    pwm[2] = ankle_pwm_lower_bounds[leg_ind];
  } else if (ankle_pwm_mid - theta4 * pwm_per_radian_ankle > ankle_pwm_upper_bounds[leg_ind]) {
    pwm[2] = ankle_pwm_upper_bounds[leg_ind];
  } else {
    pwm[2] = ankle_pwm_mid + theta4 * pwm_per_radian_ankle;
  }

}

void theta_to_pwm_array(float* theta1, float* theta2, float* theta4, int pwm_array[NUM_DATA_POINTS + 2*T_STALL][3], int leg_ind) 
{
  for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL; i++) 
  {
    int pwm[3];
    theta_to_pwm(theta1[i], theta2[i], theta4[i], pwm, leg_ind);
    pwm_array[i][0] = pwm[0];
    pwm_array[i][1] = pwm[1];
    pwm_array[i][2] = pwm[2];
  }
}

void inv_kin(float x, float y, float z, float& theta1, float& theta2, float& theta4, int leg_ind) {
  
  if (leg_ind < 2)
  {
    theta1 = atan2(y, x);
    float theta3 = PI/4.0;

    float LHS = (pow(x*cos(theta1) + y*sin(theta1) - L1, 2) + z*z - L2*L2 - L3*L3 - L4*L4 - 2*L2*L3*cos(theta3))/(2*L4);
    float A_1 = L2*cos(theta3)+L3;
    float B_1 = L2*sin(theta3);
    float phi1 = atan2(A_1, B_1);
    float a1 = sqrt(pow(A_1, 2) + pow(B_1, 2));
    // float a1 = A_1/sin(phi1);
    theta4 = phi1 - asin(LHS/a1);
    
    float A_2 = L2 + L3*cos(theta3) + L4*cos(theta3 + theta4);
    float B_2 = L4*sin(theta3 + theta4) + L3*sin(theta3);
    float phi2 = atan2(B_2, A_2);
    float a2 = sqrt(pow(A_2, 2) + pow(B_2, 2));
    // float a2 = B_2/sin(phi2);
    theta2 = asin(z/a2) + phi2;
  }

  else 
  {
    theta1 = atan2(y, x);
    float theta3 = -PI/4.0;

    float LHS = (pow(x*cos(theta1) + y*sin(theta1) - L1, 2) + z*z - L2*L2 - L3*L3 - L4*L4 - 2*L2*L3*cos(theta3))/(2*L4);
    float A_1 = L2*cos(theta3)+L3;
    float B_1 = L2*sin(theta3);
    float phi1 = atan2(B_1, A_1);
    float a1 = sqrt(pow(A_1, 2) + pow(B_1, 2));
    // float a1 = B_1/sin(phi1);
    theta4 = -1*acos(LHS/a1) - phi1;
    
    float A_2 = L2 + L3*cos(theta3) + L4*cos(theta3 + theta4);
    float B_2 = L4*sin(theta3 + theta4) + L3*sin(theta3);
    float phi2 = atan2(A_2, B_2);
    float a2 = sqrt(pow(A_2, 2) + pow(B_2, 2));
    // float a2 = A_2/sin(phi2);
    theta2 = acos(z/a2) - phi2;
  }

}

void inv_kin_array(float xyz[NUM_DATA_POINTS + 2*T_STALL][3], float* theta1, float* theta2, float* theta4, int leg_ind) {
  for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL; i++) {
    float x = xyz[i][0];
    float y = xyz[i][1];
    float z = xyz[i][2];
    
    float the1;
    float the2;
    float the4; 

    inv_kin(x, y, z, the1, the2, the4, leg_ind);

    theta1[i] = the1;
    theta2[i] = the2;
    theta4[i] = the4;
  }
}

void generate_trajectory(float xyz[NUM_DATA_POINTS + 2*T_STALL][3]) 
{
  int quarter = NUM_DATA_POINTS / 4; 
  float t[quarter];
  float y_stance[quarter*3];

  linspace(0, 1, quarter, t);

  for (int i = 0; i < quarter; i++) {
    xyz[i][0] = X;
    xyz[i][1] = pow(1 - t[i], 2) * P1[0] + 2 * (1 - t[i]) * t[i] * P2[0] + pow(t[i], 2) * P3[0];
    xyz[i][2] = pow(1 - t[i], 2) * P1[1] + 2 * (1 - t[i]) * t[i] * P2[1] + pow(t[i], 2) * P3[1];
  }

  for (int i = 0; i < T_STALL; i++)
  {
    xyz[quarter + i][0] = xyz[quarter + i - 1][0];
    xyz[quarter + i][1] = xyz[quarter + i - 1][1];
    xyz[quarter + i][2] = xyz[quarter + i - 1][2];
  }

  linspace(T/2, -T/2, quarter*3, y_stance);

  for (int i = 0; i < quarter*3; i++) {
    xyz[quarter + T_STALL + i][0] = X;
    xyz[quarter + T_STALL + i][1] = y_stance[i];
    xyz[quarter + T_STALL + i][2] = S;
  }

  for (int i = 0; i < T_STALL; i++)
  {
    xyz[NUM_DATA_POINTS + T_STALL + i][0] = xyz[NUM_DATA_POINTS + T_STALL + i - 1][0];
    xyz[NUM_DATA_POINTS + T_STALL + i][1] = xyz[NUM_DATA_POINTS + T_STALL + i - 1][1];
    xyz[NUM_DATA_POINTS + T_STALL + i][2] = xyz[NUM_DATA_POINTS + T_STALL + i - 1][2];
  }
}

void rotate_trajectory(int leg_ind, float xyzK[NUM_DATA_POINTS + 2*T_STALL][3]) 
{

  const float beta[4] = {
    -PI/4.0,             // Leg 0: RF
    PI/4.0,              // Leg 1: RB
    -PI/4.0,      // Leg 2: LB
    PI/4.0        // Leg 3: LF
  };
  const int y_pos_signs[4] = {1, 1, -1, -1};
  const int y_offset_signs[4] = {1, -1, 1, -1};
  // const int x_offset_on[4] = {1, 0, 0, 1};
  // const int z_offset_on[4] = {0, 1, 1, 0};

  float angle = beta[leg_ind];
  float cosB = cos(angle);
  float sinB = sin(angle);
  
  for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL; i++) {
    float x_old = xyzK[i][0];
    float y_old = y_pos_signs[leg_ind] * xyzK[i][1];

    xyzK[i][0] = (x_old + x_offset) * cosB - (y_old + (y_offset_signs[leg_ind]*y_offset)) * sinB;
    xyzK[i][1] = (x_old + x_offset) * sinB + (y_old + (y_offset_signs[leg_ind]*y_offset)) * cosB;
  }
}

void shift_trajectory(int leg_ind, float xyzK[NUM_DATA_POINTS + 2*T_STALL][3])
{
  const int schedule[4] = {0, 2, 1, 3}; // Order of swing: FR, BL, BR, FL

  for (int swing = 0; swing < 4; swing++) 
  {
    if (leg_ind == schedule[swing]) 
    {
      float xyzK_copy[NUM_DATA_POINTS + 2*T_STALL][3];

      // Copy original trajectory
      for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL; i++) 
      {
        xyzK_copy[i][0] = xyzK[i][0];
        xyzK_copy[i][1] = xyzK[i][1];
        xyzK_copy[i][2] = xyzK[i][2];
      }

      int xyzK_ind = 0;
      for (int i = NUM_DATA_POINTS + 2*T_STALL - swing * (NUM_DATA_POINTS / 4); i < NUM_DATA_POINTS + 2*T_STALL; i++) 
      {
        xyzK[xyzK_ind][0] = xyzK_copy[i][0];
        xyzK[xyzK_ind][1] = xyzK_copy[i][1];
        xyzK[xyzK_ind][2] = xyzK_copy[i][2];
        xyzK_ind++;
      }

      for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL - swing * (NUM_DATA_POINTS / 4); i++) 
      {
        xyzK[xyzK_ind][0] = xyzK_copy[i][0];
        xyzK[xyzK_ind][1] = xyzK_copy[i][1];
        xyzK[xyzK_ind][2] = xyzK_copy[i][2];
        xyzK_ind++;
      }

      break;
    }
  
  }
}

void stand()
{
  float xyz[3] = {X, 0, S};

  float xyz0[3];
  float xyz1[3];
  float xyz2[3];
  float xyz3[3];

  // this is giving x offset to everything and not using z offset
  xyz0[0]=cos(-PI/4.0)*(xyz[0] + x_offset) - sin(-PI/4.0)*(xyz[1] + y_offset);
  xyz0[1]=sin(-PI/4.0)*(xyz[0] + x_offset) + cos(-PI/4.0)*(xyz[1] + y_offset);
  xyz0[2]=xyz[2];

  xyz1[0]=cos(PI/4.0)*(xyz[0] + x_offset) - sin(PI/4.0)*(xyz[1] - y_offset);
  xyz1[1]=sin(PI/4.0)*(xyz[0] + x_offset) + cos(PI/4.0)*(xyz[1] - y_offset);
  xyz1[2]=xyz[2];

  xyz2[0]=cos(PI/4.0)*(xyz[0] + x_offset) - sin(PI/4.0)*(-1*xyz[1] - y_offset);
  xyz2[1]=sin(PI/4.0)*(xyz[0] + x_offset) + cos(PI/4.0)*(-1*xyz[1] - y_offset);  
  xyz2[2]=xyz[2];

  xyz3[0]=cos(-PI/4.0)*(xyz[0] + x_offset) - sin(-PI/4.0)*(-1*xyz[1] + y_offset);
  xyz3[1]=sin(-PI/4.0)*(xyz[0] + x_offset) + cos(-PI/4.0)*(-1*xyz[1] + y_offset);
  xyz3[2]=xyz[2];

  float thetaA;
  float thetaB;
  float thetaC;

  float thetaG;
  float thetaH;
  float thetaI;
  
  float thetaP;
  float thetaQ;
  float thetaR;
  
  float thetaJ;
  float thetaK;
  float thetaL;

  inv_kin(xyz[0], xyz[1], xyz[2], thetaA, thetaB, thetaC, 0); // FR
  inv_kin(xyz[0], xyz[1], xyz[2], thetaG, thetaH, thetaI, 1); // BR
  inv_kin(xyz[0], xyz[1], xyz[2], thetaP, thetaQ, thetaR, 2); // BL
  inv_kin(xyz[0], xyz[1], xyz[2], thetaJ, thetaK, thetaL, 3); // FL

  int pwmFR[3];
  int pwmBR[3];
  int pwmBL[3];
  int pwmFL[3];

  theta_to_pwm(thetaA, thetaB, thetaC, pwmFR, 0); // FR
  theta_to_pwm(thetaG, thetaH, thetaI, pwmBR, 1); // BR
  theta_to_pwm(thetaP, thetaQ, thetaR, pwmBL, 2); // BL
  theta_to_pwm(thetaJ, thetaK, thetaL, pwmFL, 3); // FL

  // Hips
  moveOne(0, pwmFR[0], 200);  // hip servo
  moveOne(4, pwmBR[0], 200);  // hip servo
  moveOne(20, pwmBL[0], 200);  // hip servo
  moveOne(24, pwmFL[0], 200);  // hip servo
  delay(1000);
  
  // Knees
  moveOne(1, pwmFR[1], 200);  // knee servo
  moveOne(5, pwmBR[1], 200);  // knee servo
  moveOne(21, pwmBL[1], 200);  // knee servo
  moveOne(25, pwmFL[1], 200);  // knee servo
  delay(1000);
  
  // Ankles
  moveOne(2, pwmFR[2], 1000);  // ankle servo
  moveOne(6, pwmBR[2], 1000);  // ankle servo
  moveOne(22, pwmBL[2], 1000);  // ankle servo
  moveOne(26, pwmFL[2], 1000);  // ankle servo
  delay(1000);
  
}

void setup() {
  Serial.begin(9600);
  delay(3000);
  stand();
  delay(3000);
  Serial.print("Setup done.");
}

void loop() 
{
  Serial.print("\nLooping\n");
  // // TEST THAT ONE JOINT MOVES
  // moveOne(2, 2600, 1000);

  // // IK test for a specific leg
  // int leg_ind = 0;
  // float x = 13;
  // float y = 0;
  // float z = -6.7;

  // float theta1;
  // float theta2;
  // float theta4;

  // int pwm[3];

  // inv_kin(x, y, z, theta1, theta2, theta4, leg_ind); // leg index 0 for FR
  // theta_to_pwm(theta1, theta2, theta4, pwm, leg_ind); // leg index 0 for FR
  
  // Serial.println("");
  // Serial.print(": XYZ = ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  // Serial.print(", ");
  // Serial.println(z);
  // Serial.print(": Theta = ");
  // Serial.print(theta1 * 180 / PI);
  // Serial.print(", ");
  // Serial.print(theta2 * 180 / PI);
  // Serial.print(", ");
  // Serial.println(theta4 * 180 / PI);
  // Serial.print(": PWM = ");
  // Serial.print(pwm[0]);
  // Serial.print(", ");
  // Serial.print(pwm[1]);
  // Serial.print(", ");
  // Serial.println(pwm[2]);

  // moveOne(0, pwm[0], 200);  // hip servo
  // moveOne(1, pwm[1], 200);  // knee servo
  // moveOne(2, pwm[2], 200);  // ankle servo

  // // read feedback
  // int adcA = analogRead(potA);
  // int adcB = analogRead(potB);
  // int adcC = analogRead(potC);

  // int angleMapA = map(adcA, adcMinA, adcMaxA, 0, 180);
  // int angleMapB = map(adcB, adcMinA, adcMaxA, 0, 180); // all are using A's range, update later
  // int angleMapC = map(adcC, adcMinA, adcMaxA, 0, 180);

  // // logging feedback
  // Serial.print("\nFeedback [SERVO A ANGLE]:");
  // Serial.print(angleMapA-90); // because mapping between 0 to 180
  // Serial.print("\n");
  // Serial.print("Feedback [SERVO B ANGLE]:");
  // Serial.print(angleMapB-90);
  // Serial.print("\n");
  // Serial.print("Feedback [SERVO C ANGLE]:");
  // Serial.print(angleMapC-90);
  // Serial.print("\n");

  // Gait testing

  // trajectory points
  float xyz[NUM_DATA_POINTS + 2*T_STALL][3];
  generate_trajectory(xyz);

  float xyz0[NUM_DATA_POINTS + 2*T_STALL][3];
  float xyz1[NUM_DATA_POINTS + 2*T_STALL][3];
  float xyz2[NUM_DATA_POINTS + 2*T_STALL][3];
  float xyz3[NUM_DATA_POINTS + 2*T_STALL][3];

  for (int i=0; i<NUM_DATA_POINTS + 2*T_STALL; ++i) 
  {
    xyz0[i][0]=xyz[i][0]; 
    xyz0[i][1]=xyz[i][1]; 
    xyz0[i][2]=xyz[i][2];

    xyz1[i][0]=xyz[i][0]; 
    xyz1[i][1]=xyz[i][1]; 
    xyz1[i][2]=xyz[i][2];
    
    xyz2[i][0]=xyz[i][0]; 
    xyz2[i][1]=xyz[i][1]; 
    xyz2[i][2]=xyz[i][2];
    
    xyz3[i][0]=xyz[i][0]; 
    xyz3[i][1]=xyz[i][1]; 
    xyz3[i][2]=xyz[i][2];
  }

  rotate_trajectory(0, xyz0);
  rotate_trajectory(1, xyz1);
  rotate_trajectory(2, xyz2);
  rotate_trajectory(3, xyz3);

  shift_trajectory(0, xyz0);
  shift_trajectory(1, xyz1);
  shift_trajectory(2, xyz2);
  shift_trajectory(3, xyz3);

  for (int i = 0; i < NUM_DATA_POINTS + 2*T_STALL; i++) 
  {

    float thetaA;
    float thetaB;
    float thetaC;

    float thetaG;
    float thetaH;
    float thetaI;
    
    float thetaP;
    float thetaQ;
    float thetaR;
    
    float thetaJ;
    float thetaK;
    float thetaL;
    
    // unrotated test
    // inv_kin(xyz[i][0], xyz[i][1], xyz[i][2], thetaA, thetaB, thetaC, 0); // FR

    inv_kin(xyz0[i][0], xyz0[i][1], xyz0[i][2], thetaA, thetaB, thetaC, 0); // FR
    inv_kin(xyz1[i][0], xyz1[i][1], xyz1[i][2], thetaG, thetaH, thetaI, 1); // BR
    inv_kin(xyz2[i][0], xyz2[i][1], xyz2[i][2], thetaP, thetaQ, thetaR, 2); // BL
    inv_kin(xyz3[i][0], xyz3[i][1], xyz3[i][2], thetaJ, thetaK, thetaL, 3); // FL

    int pwmFR[3];
    int pwmBR[3];
    int pwmBL[3];
    int pwmFL[3];

    theta_to_pwm(thetaA, thetaB, thetaC, pwmFR, 0); // FR
    theta_to_pwm(thetaG, thetaH, thetaI, pwmBR, 1); // BR
    theta_to_pwm(thetaP, thetaQ, thetaR, pwmBL, 2); // BL
    theta_to_pwm(thetaJ, thetaK, thetaL, pwmFL, 3); // FL

    // logging
    Serial.print("\n\nFront Right Step ");
    Serial.print(i);
    Serial.print(": Thetas = ");
    Serial.print(thetaA * 180.0 / PI);
    Serial.print(", ");
    Serial.print(thetaB * 180.0 / PI);
    Serial.print(", ");
    Serial.println(thetaC * 180.0 / PI);
    Serial.print(": PWM = ");
    Serial.print(pwmFR[0]);
    Serial.print(", ");
    Serial.print(pwmFR[1]);
    Serial.print(", ");
    Serial.println(pwmFR[2]);
    Serial.print("Coordinates: ");
    Serial.print(xyz0[i][0]);
    Serial.print(", ");
    Serial.print(xyz0[i][1]);
    Serial.print(", ");
    Serial.print(xyz0[i][2]);
    Serial.print("\n");

    Serial.print("\n\nBack Left Step ");
    Serial.print(i);
    Serial.print(": Thetas = ");
    Serial.print(thetaP * 180.0 / PI);
    Serial.print(", ");
    Serial.print(thetaQ * 180.0 / PI);
    Serial.print(", ");
    Serial.println(thetaR * 180.0 / PI);
    Serial.print(": PWM = ");
    Serial.print(pwmBL[0]);
    Serial.print(", ");
    Serial.print(pwmBL[1]);
    Serial.print(", ");
    Serial.println(pwmBL[2]);
    Serial.print("Coordinates: ");
    Serial.print(xyz2[i][0]);
    Serial.print(", ");
    Serial.print(xyz2[i][1]);
    Serial.print(", ");
    Serial.print(xyz2[i][2]);
    Serial.print("\n");

    // FR 
    moveOne(0, pwmFR[0], 200);  // hip servo
    moveOne(1, pwmFR[1], 200);  // knee servo
    moveOne(2, pwmFR[2], 200);  // ankle servo

    // // read feedback
    // int adcA = analogRead(potA);
    // int adcB = analogRead(potB);
    // int adcC = analogRead(potC);

    // int angleMapA = map(adcA, adcMinA, adcMaxA, 0, 180);
    // int angleMapB = map(adcB, adcMinA, adcMaxA, 0, 180); // all are using A's range, update later
    // int angleMapC = map(adcC, adcMinA, adcMaxA, 0, 180);

    // // logging feedback
    // Serial.print("\nFeedback [SERVO A ANGLE]:");
    // Serial.print(angleMapA-90); // because mapping between 0 to 180
    // Serial.print("\n");
    // Serial.print("Feedback [SERVO B ANGLE]:");
    // Serial.print(angleMapB-90);
    // Serial.print("\n");
    // Serial.print("Feedback [SERVO C ANGLE]:");
    // Serial.print(angleMapC-90);
    // Serial.print("\n");

    // BR 
    moveOne(4, pwmBR[0], 200);  // hip servo
    moveOne(5, pwmBR[1], 200);  // knee servo
    moveOne(6, pwmBR[2], 200);  // ankle servo
  
    // BL 
    moveOne(20, pwmBL[0], 200);  // hip servo
    moveOne(21, pwmBL[1], 200);  // knee servo
    moveOne(22, pwmBL[2], 200);  // ankle servo

    // FL
    moveOne(24, pwmFL[0], 200);  // hip servo
    moveOne(25, pwmFL[1], 200);  // knee servo
    moveOne(26, pwmFL[2], 200);  // ankle servo

    delay(250);
  }

  // delay loop
  delay(1000);
}
