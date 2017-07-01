#include <Wire.h>
#include <I2Cdev.h>
#include <math.h>
#include <avr/eeprom.h>
extern "C"{
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#define DEFAULT_MPU_HZ  (100)
#define I2C_SPEED 400000L
#define q30  1073741824.0f
#define q0 quatf[0]     //四元数计算
#define q1 quatf[1]
#define q2 quatf[2]
#define q3 quatf[3]


uint8_t devStatus; 
char chRxTemp;
uint32_t currentTime = 0;
uint32_t lastTime = 0;

int throttle = 0;
int in_r = 0;
int in_p = 0;
int in_y = 0;

#define EEPLEN 6
int eepa[EEPLEN] = {0};


short gyro[3];
short accel[3];
long quat[4];
float quatf[4];
int Pitch,Roll,Yaw,Yaw_t,Yaw_b;
unsigned long timestamp;
short sensors;
unsigned char more;



static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
        
    return b;
}
static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

uint16_t mpu605dmp_init()
{
    uint16_t result = 0;
    long gyro_tmp[3], accel_tmp[3];
    float gyro_sens;
    unsigned short accel_sens;
    //uint16_t debug;
    
    if(mpu_init(NULL))
        result |= 1;
    
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        result |= 2; 
    
    if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        result |= 4;
    
    if(mpu_set_sample_rate(DEFAULT_MPU_HZ))
        result |= 8;
    
    if(dmp_load_motion_driver_firmware())
        result |= 0x10;
        //Serial.print("ERROR:");
        //Serial.print(debug);
        //Serial.print("\n\r");
    if(dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        result |= 0x20;
    
    if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
                  DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
        result |= 0x40;
    
    if(dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        result |= 0x80;
    
    #if 0
    if(mpu_run_self_test(gyro_tmp, accel_tmp))
    {
        mpu_get_gyro_sens(&gyro_sens);
        gyro_tmp[0] = (long)(gyro_tmp[0] * gyro_sens);
        gyro_tmp[1] = (long)(gyro_tmp[1] * gyro_sens);
        gyro_tmp[2] = (long)(gyro_tmp[2] * gyro_sens);
        Serial.print("bias:\n\r");   
        Serial.print(gyro_tmp[0]); Serial.print("\n\r"); 
        Serial.print(gyro_tmp[1]); Serial.print("\n\r"); 
        Serial.print(gyro_tmp[2]); Serial.print("\n\r"); 
        dmp_set_gyro_bias(gyro_tmp);
        
        mpu_get_accel_sens(&accel_sens);
        accel_tmp[0] *= accel_sens;
        accel_tmp[1] *= accel_sens;
        accel_tmp[2] *= accel_sens;
        Serial.print(accel_tmp[0]); Serial.print("\n\r"); 
        Serial.print(accel_tmp[1]); Serial.print("\n\r"); 
        Serial.print(accel_tmp[2]); Serial.print("\n\r"); 
        dmp_set_accel_bias(accel_tmp);        
    }
    else
        result |= 0x100;
    #else
    gyro_tmp[0] = -1320000;
    gyro_tmp[1] = -610000;
    gyro_tmp[2] = -400000;
    dmp_set_gyro_bias(gyro_tmp);
    
    accel_tmp[0] = 17000000;
    accel_tmp[1] = 2000000;
    accel_tmp[2] = -7500000;
    dmp_set_accel_bias(accel_tmp);  
    #endif
    
    if(mpu_set_dmp_state(1))
        result |= 0x200;

    if (mpu_set_bypass(1))
        result |= 0x400;
        
    return result;
}


#if 0
#define MAG_ADDRESS 0x1E
#define HMC58X3_R_CONFA 0x0
#define HMC58X3_R_CONFB 0x01
#define HMC58X3_R_MODE 0x02
#define MAG_DATA_REGISTER 0x03

int16_t mag_xyz[3];

int mag_init() {
    uint16_t result = 0;
    uint8_t reg_dat[3] = {0x70, 0x20, 0x01};

    //if (mpu_set_bypass(1))
    //    result |= 1;

    if (i2c_write(MAG_ADDRESS, HMC58X3_R_CONFA, 1, &reg_dat[0]))
        result |= 2;

    if (i2c_write(MAG_ADDRESS, HMC58X3_R_CONFB, 1, &reg_dat[1]))
        result |= 4;

    if (i2c_write(MAG_ADDRESS, HMC58X3_R_MODE, 1, &reg_dat[2]))
        result |= 8;

    //if (mpu_set_bypass(0))
    //    result |= 0x10;

    return result;
} //  mag_init().

int mag_get_dat(int16_t dat[3])
{
    uint8_t dat_tmp[6];
    
    //if (mpu_set_bypass(1))
    //    return -1;

    if (i2c_read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, dat_tmp))
        return -1;

    //if (mpu_set_bypass(0))
    //    return -1;
    
    dat[0] = dat_tmp[0] << 8 | dat_tmp[1];
    dat[1] = dat_tmp[4] << 8 | dat_tmp[5];
    dat[2] = dat_tmp[2] << 8 | dat_tmp[3];

    return 0;
}
#endif

#if 0
#define BMP085_ADDRESS 0x77
#define BARO_TAB_SIZE   21

static struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 3

int32_t baroPressure;
int32_t baroTemperature;
int32_t baroPressureSum;

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  uint8_t s_bytes = 22;//(uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  if (i2c_read(BMP085_ADDRESS, 0xAA, s_bytes, (uint8_t*)&bmp085_ctx.ac1))
    while(1);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void  Baro_init() {
  //delay(10);
  i2c_BMP085_readCalibration();
  delay(5);
  //i2c_BMP085_UT_Start(); 
  bmp085_ctx.deadline = currentTime + 5000;
}

/*
// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}
*/

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  /*
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
  */
  uint8_t tmp = 0x34+(OSS<<6);
  if (i2c_write(BMP085_ADDRESS, 0xf4, 1, &tmp))
    Serial.println("i2c_write(BMP085_ADDRESS, 0xf4, 1, &tmp)");  
  if (i2c_read(BMP085_ADDRESS, 0xF6, 3, (uint8_t*)bmp085_ctx.up.raw))
    Serial.println("i2c_read(BMP085_ADDRESS, 0xF6, 3, (uint8_t*)bmp085_ctx.up.raw)");  
  tmp = bmp085_ctx.up.raw[2];
  bmp085_ctx.up.raw[2] = bmp085_ctx.up.raw[0];
  bmp085_ctx.up.raw[0] = tmp;
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  /*
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
  */
  uint8_t tmp = 0x2e;
  if (i2c_write(BMP085_ADDRESS, 0xf4, 1, &tmp))
    Serial.println("i2c_write(BMP085_ADDRESS, 0xf4, 1, &tmp)");  
  if (i2c_read(BMP085_ADDRESS, 0xF6, 2, (uint8_t*)bmp085_ctx.ut.raw))
    Serial.println("i2c_read(BMP085_ADDRESS, 0xF6, 2, (uint8_t*)bmp085_ctx.ut.raw)");
  tmp = bmp085_ctx.ut.raw[1];
  bmp085_ctx.ut.raw[1] = bmp085_ctx.ut.raw[0];
  bmp085_ctx.ut.raw[0] = tmp;
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
}

void Baro_Common() {
  static int32_t baroHistTab[BARO_TAB_SIZE];
  static uint8_t baroHistIdx;

  uint8_t indexplus1 = (baroHistIdx + 1);
  if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
  baroHistTab[baroHistIdx] = baroPressure;
  baroPressureSum += baroHistTab[baroHistIdx];
  baroPressureSum -= baroHistTab[indexplus1];
  baroHistIdx = indexplus1;  
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                   // first UT conversion is started in init procedure
  if (currentTime < bmp085_ctx.deadline) return 0; 
  bmp085_ctx.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
  //TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (bmp085_ctx.state == 0) {
    i2c_BMP085_UT_Read(); 
    //i2c_BMP085_UP_Start(); 
    bmp085_ctx.state = 1; 
    Baro_Common();
    bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
    return 1;
  } else {
    i2c_BMP085_UP_Read(); 
    //i2c_BMP085_UT_Start(); 
    i2c_BMP085_Calculate(); 
    bmp085_ctx.state = 0; 
    return 2;
  }
}
#endif

volatile uint8_t atomicPWM_PIN4_lowState;
volatile uint8_t atomicPWM_PIN4_highState;
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;

int PWM_Out[6] = {0};

void PWM_Init()
{    
    pinMode(3, OUTPUT);     // pwm 
    pinMode(9, OUTPUT);     // pwm
    pinMode(10, OUTPUT);    // pwm
    pinMode(11, OUTPUT);    // pwm 
    TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    TCCR0A = 0; // initializeSoftPWM normal counting mode
    TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
    TIMSK0 |= (1<<OCIE0A);
    pinMode(A0, OUTPUT);    // pwm
    pinMode(A1, OUTPUT);    // pwm
}

void PWM_OutPut(int motor_in[6])
{
    int i = 0;
    uint8_t motor[6] = {0};

    for (; i < 6; i++)
    {
        if (motor_in[i] < 5)
        {
            motor[i] = 0;
        }
        else if(motor_in[i] > 255)
        {
            motor[i] = 255;
        }
        else
        {
            motor[i] = motor_in[i];
        }
    }

    OCR2B = motor[0]; //  pin 3
    OCR1A = motor[1]; //  pin 9
    OCR1B = motor[2]; //  pin 10
    OCR2A = motor[3]; //  pin 11    
    atomicPWM_PIN4_highState = motor[4];
    atomicPWM_PIN5_highState = motor[5];
    atomicPWM_PIN4_lowState  = 255 - atomicPWM_PIN4_highState;
    atomicPWM_PIN5_lowState  = 255 - atomicPWM_PIN5_highState; 
}

void PWM_OutPut_All(int mo)
{
    int motor[6];
    uint8_t i = 0;

    for (; i < 6; i++)
    {
        motor[i] = mo;
    }
    PWM_OutPut(motor);
}


#define SOFT_PWM_1_PIN_HIGH        PORTC |= 1<<0
#define SOFT_PWM_1_PIN_LOW         PORTC &= ~(1<<0)
#define SOFT_PWM_2_PIN_HIGH        PORTC |= 1<<1
#define SOFT_PWM_2_PIN_LOW         PORTC &= ~(1<<1)
ISR(TIMER0_COMPA_vect) { 
    static uint8_t state = 0;
    if(state == 0){
      if (atomicPWM_PIN4_highState>0) SOFT_PWM_1_PIN_HIGH;
      OCR0A += atomicPWM_PIN4_highState;
      state = 1;
    }else if(state == 1){
      OCR0A += atomicPWM_PIN4_highState;
      state = 2;
    }else if(state == 2){
      SOFT_PWM_1_PIN_LOW;
      OCR0A += atomicPWM_PIN4_lowState;
      state = 3;  
    }else if(state == 3){
      OCR0A += atomicPWM_PIN4_lowState;
      state = 0;   
    }
}
ISR(TIMER0_COMPB_vect) { 
    static uint8_t state = 0;
    if(state == 0){
      if (atomicPWM_PIN5_highState>0) SOFT_PWM_2_PIN_HIGH;
      OCR0B += atomicPWM_PIN5_highState;
      state = 1;
    }else if(state == 1){
      OCR0B += atomicPWM_PIN5_highState;
      state = 2;
    }else if(state == 2){
      SOFT_PWM_2_PIN_LOW;
      OCR0B += atomicPWM_PIN5_lowState;
      state = 3;  
    }else if(state == 3){
      OCR0B += atomicPWM_PIN5_lowState;
      state = 0;   
    }
}

uint8_t fly_sta = 0;

typedef struct pid_s{
    int Kp;
    int Ki;
    int Kd;
    int inte;
//    int e1;
//    int e2;
    int diff;
    int inte_max;
};

#define INTE_M 1000

struct pid_s pid_r;
struct pid_s pid_p;
struct pid_s pid_y;

void PID_Init(struct pid_s *pid)
{
    pid->diff = 0;
    pid->inte = 0;
    pid->inte_max = INTE_M;
    pid->Kd = 0;
    pid->Ki = 0;
    pid->Kp = 0;
}

int pin_led = 13;
int led_sta = 0;

void setup() {
    // put your setup code here, to run once:
    pinMode(pin_led, OUTPUT);    // led

    Serial.begin(115200);       // 15200 bps

    Serial.println("Initializing PWM...");
    PWM_Init();
    PWM_OutPut_All(0);

    Serial.println("Initializing I2C devices...");
    Wire.begin();
    
    // 4000Khz fast mode
    TWSR = 0;
    TWBR = ((16000000L/I2C_SPEED) - 16)/2;
    TWCR = 1<<TWEN;
    
    delay(1000);
    Serial.println("Initializing DMP...");
    devStatus = mpu605dmp_init();
    if (devStatus)
    {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");

        while(1);
    }
    Serial.println("DMP OK");

    #if 0
    delay(1);
    Serial.println("Initializing MAG...");
    devStatus = mag_init();
    if (devStatus)
    {
        Serial.print("MAG Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        
        while(1);
    }
    Serial.println("MAG OK");
    #endif

    #if 0
    delay(1);
    Serial.println("Initializing BMP...");
    Baro_init();
    #endif

//    PID_Init(&pid_r);
//    PID_Init(&pid_p);
//    PID_Init(&pid_y);
    pid_r.diff = pid_r.inte = 0;
    pid_p.diff = pid_p.inte = 0;
    pid_y.diff = pid_y.inte = 0;
    pid_r.inte_max = pid_p.inte_max = pid_y.inte_max = INTE_M;
    
    eeprom_read_block(&eepa[0], (void*)0, sizeof(int)*EEPLEN);
    pid_p.Kp = pid_r.Kp = eepa[0];
    pid_p.Ki = pid_r.Ki = eepa[1];
    pid_p.Kd = pid_r.Kd = eepa[2];
    pid_y.Kp = eepa[3];
    pid_y.Ki = eepa[4];
    pid_y.Kd = eepa[5]; 
    
    lastTime = currentTime = micros();
}

long PID_Cclt(int in,int fb, int td, struct pid_s pid)
{
    int err;
    long out;

    err = in - fb;
    
    if ((!((pid.inte > pid.inte_max) && (err > 0))) 
        &&(!((pid.inte < -pid.inte_max) && (err < 0)))){
        pid.inte += err;
    }
    
    out = (long)pid.Kp * err 
        + (long)pid.Ki * pid.inte * td
        + (long)pid.Kd * pid.diff;

    return out;
}

void Progress_Rx(uint8_t rx)
{
    if ('q' == rx)
    {
        pid_r.Kp--;
        pid_p.Kp--;    
    }
    else if ('w' == rx)
    {
        pid_r.Kp++;
        pid_p.Kp++;
    }
    else if ('e' == rx)
    {
        pid_r.Kd--;
        pid_p.Kd--;
    }
    else if ('r' == rx)
    {
        pid_r.Kd++;
        pid_p.Kd++;
    }
    else if ('a' == rx)
    {
        //pid_p.Kp--;
    }
    else if ('s' == rx)
    {
        //pid_p.Kp++;
    }
    else if ('d' == rx)
    {
        //pid_p.Kd--;
    }
    else if ('f' == rx)
    {
        //pid_p.Kd++;
    }
    else if ('z' == rx)
    {
        pid_y.Kp--;
    }
    else if ('x' == rx)
    {
        pid_y.Kp++;
    }
    else if ('c' == rx)
    {
        pid_y.Ki--;
    }
    else if ('v' == rx)
    {
        pid_y.Ki++;
    }
    else if ('g' == rx)
    {
        throttle++;
    }
    else if ('b' == rx)
    {
        throttle--;
    }
    else if ('l' == rx)
    {
        in_r++;
    }
    else if (',' == rx)
    {
        in_r--;
    }
    else if (';' == rx)
    {
        in_p++;
    }
    else if ('.' == rx)
    {
        in_p--;
    }
    else if ('/' == rx)
    {
        eepa[0] = pid_r.Kp;
        eepa[1] = pid_r.Ki;
        eepa[2] = pid_r.Kd;
        eepa[3] = pid_y.Kp;
        eepa[4] = pid_y.Ki;
        eepa[5] = pid_y.Kd;
        eeprom_write_block(&eepa[0], (void*)0, sizeof(int)*EEPLEN);
    }
    else if ('t' == rx)
    {
        mpu605dmp_init();
    }
    else if ('n' == rx)
    {
        fly_sta = 0;
        PWM_OutPut_All(0);
    }
    else if ('h' == rx)
    {
        fly_sta = 1;
        pid_r.diff = pid_r.inte = 0;
        pid_p.diff = pid_p.inte = 0;
        pid_y.diff = pid_y.inte = 0;
        throttle = 0;
        in_r = in_p = 0;
        //Yaw_b = Yaw_t;
    }
    
    digitalWrite(pin_led, (led_sta = !led_sta));
}

int pid_o_r = 0;
int pid_o_p = 0;
int pid_o_y = 0;
int dt = 0;

void loop() {
    // put your main code here, to run repeatedly: 
    if(Serial.available())
    {
        chRxTemp = Serial.read();
        Serial.write(chRxTemp);

        Progress_Rx(chRxTemp);
    }

    if (!dmp_read_fifo(gyro, accel, quat,&timestamp, &sensors, &more))
    {
        quatf[0] = quat[0]/q30;  //四元数归一化
        quatf[1] = quat[1]/q30;
        quatf[2] = quat[2]/q30;
        quatf[3] = quat[3]/q30;

        Roll  = (int)(asin(2*q1*q3- 2*q0*q2)*573.14);       //roll -90~90
      
        Pitch = (int)(atan2(2*q2*q3+ 2*q0*q1,
                    -2*q1*q1- 2*q2*q2+ 1)*573.14);          //pitch -180~180
        Yaw = (int)(atan2(2*(q1*q2+ q0*q3),
                    q0*q0+ q1*q1- q2*q2- q3*q3)*573.14);    //Yaw -180~180
        //Yaw = Yaw_t - Yaw_b;
        
        currentTime = micros();
        dt = currentTime - lastTime;
        lastTime = currentTime;

        pid_r.diff = gyro[1]/10;
        pid_p.diff = -gyro[0]/10;
        pid_y.diff = gyro[2]/10;
        pid_o_r = PID_Cclt(in_r, Roll, dt, pid_r);
        pid_o_p = PID_Cclt(in_p, Pitch, dt, pid_p);
        pid_o_y = PID_Cclt(in_y, Yaw, dt, pid_y);

        PWM_Out[0] = throttle - (pid_o_r/2 - pid_o_p/8*7 - pid_o_y)/100;
        PWM_Out[1] = throttle + (pid_o_r/2 - pid_o_p/8*7 - pid_o_y)/100;
        PWM_Out[2] = throttle + (pid_o_r/2 + pid_o_p/8*7 - pid_o_y)/100;
        PWM_Out[3] = throttle - (pid_o_r/2 + pid_o_p/8*7 - pid_o_y)/100;
        PWM_Out[4] = throttle - (pid_o_r + pid_o_y)/100;
        PWM_Out[5] = throttle + (pid_o_r + pid_o_y)/100;

        if ((Roll > 450) || (Roll < -450) || (Pitch > 450) || (Pitch < -450))
        {
            fly_sta = 0;
            PWM_OutPut_All(0);
        }

        if (fly_sta)
        {
            PWM_OutPut(PWM_Out);
        }
        #if 1
        //Serial.print("\r                                        ");
        Serial.print("\rRPY:\t");
        Serial.print(Roll);
        Serial.print("\t");
        Serial.print(Pitch);
        Serial.print("\t");
        Serial.print(Yaw);  
        Serial.print("\t");
        Serial.print(throttle);
        Serial.print("\t");
        Serial.print(in_r);
        Serial.print("\t");
        Serial.print(in_p);
        Serial.print("\t");
        Serial.print(pid_r.Kp);
        Serial.print("\t");
        Serial.print(pid_r.Kd);
        Serial.print("\t");
        Serial.print(pid_p.Kp);
        Serial.print("\t");
        Serial.print(pid_p.Kd);
        Serial.print("\t");
        Serial.print(pid_y.Kp);
        Serial.print("\t");
        Serial.print(pid_y.Ki);
        #endif
    }

    //if (!mag_get_dat(mag_xyz))
    {
        
    }

    //if (1 == Baro_update())
    {
        //Serial.print("\r                                        ");
        //Serial.print("\rBMP:\t");
        //Serial.print(baroPressure);
        //Serial.print("\r\n");
    }
}











