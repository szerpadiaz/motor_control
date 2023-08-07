#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

#include <SPI.h>
/*SAMD core*/ 
#ifdef ARDUINO_SAD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB 
#else
  #define SERIAL Serial 
#endif
// Define Joystick connection pins
#define UP A1
#define DOWN A3
#define LEFT A2
#define RIGHT A5
#define CLICK A4

//Define LED pins 
#define LED2 8
#define LED3 7

// Value Limits 
#define P_MIN -12.5f 
#define P_MAX 12.5f 
#define V_MIN -65.0f 
#define V_MAX 65.0f 
#define KP_MIN 0.0f 
#define KP_MAX 500.0f 
#define KD_MIN 0.0f 
#define KD_MAX 5.0f 
#define T_MIN - 18.0f 
#define T_MAX 18.0f

// Set values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 100.0f;
float kd_in = 1.0f;
float t_in = 0.0f;

// measured values
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

// the cs pin of the version after v1.
// v0.9b and v1.0 is default D10
//const int SPI_CS_PIN = 10;

//MCP_CAN CAN(SPI_CS_PIN); 

void setup()
{
  SERIAL.begin(115200);
  delay (1000);

  if(Canbus.init(CANSPEED_500))  //Initialise MCP2515 CAN controller at the specified speed
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");

  //CAN.init_Mask(0, 0, 0xFFF); // Mask for filtering the message ID
  //CAN.init_Filt(0, 0, 0);     // Filter for accepting the message ID

  // Initialize pins as necessary
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(CLICK, INPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  // Pull andog pins high to enable reading of joystick moveme
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  digitalWrite(CLICK, HIGH);
  // Write LED pins low to turn them off by default
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
}

long previousMillis = 0;
void loop()
{
  //do something
  float p_step = 0.01;
  if (digitalRead(UP)==LOW)
  {
    //move motor forward
    p_in = p_in + p_step;
  }
  if (digitalRead(DOWN)==LOW)
  {
    //move motor backward
    p_in = p_in - p_step;
  }
  p_in = constrain(p_in, P_MIN, P_MAX);
  if (digitalRead(RIGHT)==LOW)
  {
    EnterMotorMode();
    digitalWrite(LED2, HIGH);
  }
  if (digitalRead(LEFT)==LOW)
  { 
    // Disable
    ExitMotorMode();
    digitalWrite(LED2, LOW);
  }
  // send CAN
  pack_cmd();

  // receive CAN
  //if(CAN_MSGAVAIL == CAN.checkReceive())
  //{ 
  //  unpack_reply();
  //}

  //print data
  SERIAL.print(millis()-previousMillis);
  previousMillis = millis();
  SERIAL.print(" ");
  SERIAL.print (p_in);
  SERIAL.print(" ");
  SERIAL.print (p_out);
  SERIAL.print(" ");
  SERIAL.print(v_out);
  SERIAL.print(" ");
  SERIAL.println(t_out);
  }

void EnterMotorMode(){
  // Enter Motor Mode (enable)

  tCAN message;

  message.id = 0x01; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = 0xFF;
	message.data[1] = 0xFF;
	message.data[2] = 0xFF;
	message.data[3] = 0xFF; //formatted in HEX
	message.data[4] = 0xFF;
	message.data[5] = 0xFF;
	message.data[6] = 0xFF;
	message.data[7] = 0xFC;
 
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);

}

void ExitMotorMode(){
  // Exit Motor Mode (enable)

  tCAN message;

  message.id = 0x01; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = 0xFF;
	message.data[1] = 0xFF;
	message.data[2] = 0xFF;
	message.data[3] = 0xFF; //formatted in HEX
	message.data[4] = 0xFF;
	message.data[5] = 0xFF;
	message.data[6] = 0xFF;
	message.data[7] = 0xFD;
 
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);

}

void pack_cmd(){
  byte buf[8];

  float p_des = constrain(p_in, P_MIN, P_MAX); //fminf(fmaxf(P.
  float v_des = constrain(v_in, V_MIN, V_MAX); //fminf(fmaxf(V.
  float kp = constrain(kp_in, KP_MIN, KP_MAX); //fminf(fmaxf(KF
  float kd = constrain(kd_in, KD_MIN, KD_MAX); //fminf(fmaxf(KI
  float t_ff = constrain(t_in, T_MIN, T_MAX); //fminf(fmaxf(T_N
  // convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  tCAN message;

  message.id = 0x01; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = p_int >> 8;
	message.data[1] = p_int & 0xFF;
	message.data[2] = v_int >> 4;
	message.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); //formatted in HEX
	message.data[4] = kp_int & 0xFF;
	message.data[5] = kd_int >> 4;
	message.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
	message.data[7] = t_int & 0xFF;
 
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);
}

void unpack_reply(){
  byte len = 0;
  byte buf[8];
  unsigned long canId = 0;
  //CAN.readMsgBuf(canId, &len, buf);

  /// unpack ints from can buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf [1] << 8) | buf[2]; 
  unsigned int v_int = (buf [3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf [5];
}
unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float (unsigned int x_int, float x_min, float x_max, int bits)
{
// converts unsigned int to float, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}
// END FILE


