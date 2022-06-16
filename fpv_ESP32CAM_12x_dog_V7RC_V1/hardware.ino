/* - ::: HARDWARE :::

  This file is part of warp_kinematics.
  [hardware] This file manages the basic hardware functions.

  [BACK] [LEFT], LOWER JOINT (0, 0) : servo00,
  UPPER JOINT (0, 1) : servo02, SHLDR JOINT (0, 2) :servo08

  [FRONT][LEFT], LOWER JOINT (1, 0) : servo04,
  UPPER JOINT (1, 1) : servo06, SHLDR JOINT (1, 2) :servo09

  [FRONT][RIGHT], LOWER JOINT (2, 0) : servo05,
  UPPER JOINT (2, 1) : servo07, SHLDR JOINT (2, 2) :servo10

  [BACK] [RIGHT], LOWER JOINT (3, 0) : servo01,
  UPPER JOINT (3, 1) : servo03, SHLDR JOINT (3, 2) :servo11

*/

/*
  ==============================
  HARDWARE - SERVO PARAMETERS
  ==============================
*/

const int s_output[4][3] = {
  {8, 2, 0},  // ## {chnl, chnl, chnl}
  {9, 6, 4},  // ## {chnl, chnl, chnl}
  {10, 7, 5}, // ## {chnl, chnl, chnl}
  {11, 3, 1}  // ## {chnl, chnl, chnl}
};

const int s_optinv[4][3] = {
  {0, 0, 0}, // ## {dir, dir, dir}
  {1, 0, 0}, // ## {dir, dir, dir}
  {0, 1, 1}, // ## {dir, dir, dir}
  {1, 1, 1}  // ## {dir, dir, dir}
};

const int s_offset_min[4][3] = {
  {500, 500, 500},    // ## {pulse-width, pulse-width, pulse-width}
  {500, 500, 500},  // ## {pulse-width, pulse-width, pulse-width}
  {500, 500, 500},  // ## {pulse-width, pulse-width, pulse-width}
  {500, 500, 500}    // ## {pulse-width, pulse-width, pulse-width}
};

const int s_offset_max[4][3] = {
  {2400, 2400, 2400}, // ## {pulse-width, pulse-width, pulse-width}
  {2400, 2400, 2400}, // ## {pulse-width, pulse-width, pulse-width}
  {2400, 2400, 2400}, // ## {pulse-width, pulse-width, pulse-width}
  {2400, 2400, 2400}  // ## {pulse-width, pulse-width, pulse-width}
};

// const int d_constraint_min[] { -50, 30, 60}; // ## {deg, deg, deg}
// const int d_constraint_max[] {80, 150, 140}; // ## {deg, deg, deg}
const int d_constraint_min[] { 0, 0, 0}; // ## {deg, deg, deg}
const int d_constraint_max[] {180, 180, 180}; // ## {deg, deg, deg}

/*
  ==============================
  HARDWARE - MPU VARIABLES
  ==============================
*/

uint16_t packetSize; // expected DMP packet size (default 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Quaternion q;           // [w, x, y, z]         quaternion container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//VectorFloat gravity;    // [x, y, z]            gravity vector

/*
  ::: SETUP :::
*/


/*
  ::: HANDLE LOOP :::
*/

/*
  ::: [SERVO] FUNCTIONS :::
*/

void set_leg(int leg, datatypes::Rotator rot) {
  set_joint(leg, 0, rot.yaw);
  set_joint(leg, 1, rot.pitch);
  set_joint(leg, 2, rot.roll);
  //Serial.println("");
}

void set_joint(int leg, int joint, float deg) {
  int _min = s_offset_min[leg][joint];
  int _max = s_offset_max[leg][joint];
  int _num = s_output[leg][joint];
  int _inv = s_optinv[leg][joint];

  int _minC = d_constraint_min[joint];
  int _maxC = d_constraint_max[joint];


  if (deg < _minC)
    deg = _minC;
  else if (deg > _maxC)
    deg = _maxC;

      float t_deg;     
  if (_inv == 0)
     t_deg = deg+offset_map[_num];
  else if (_inv == 1)
     t_deg = 180-deg+offset_map[_num];

    if ((_num == 8)||(_num == 9)||(_num == 10)||(_num == 11)){
 ////       t_deg = 90+deg;
        t_deg = 90+deg+offset_map[_num];
 /// xxx      t_deg = 90-deg+offset_map[_num];
   //   Serial.println(t_deg);
      }
   pwm.setPWM(_num, 0, floor(((t_deg) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)*0.2048 ) );
}
