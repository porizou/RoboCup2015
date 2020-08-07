/*------------------------

* XBee1
    ID : 5102

    DH : 0013A200
    DL : 407ABE67

* XBee2
    ID : 5102

    DH : 0013A200
    DL : 407C04BD

------------------------*/

/*------------------------
mv /Users/muramatsunaoya/Downloads/RoboCup_2015_LPC1768.bin /Volumes/MBED/ && diskutil unmount /Volumes/MBED/
------------------------*/



#include "mbed.h"
// ライブラリ
#include "MotorDriver.h"
#include "PID.h"
// 自作のクラス
#include "xbeeAPI.h"


/*--- 定義 ---*/
#define PI 3.1415926
#define Deg2Rad (3.1415926/180)
#define Byte2Rad (3.1415926/255/2)

#define mC 261.626
#define mD 293.665
#define mE 329.628
#define mF 349.228
#define mG 391.995
#define mA 440.000
#define mB 493.883

// PIDに関する部分
#define IndividualValueL 0.4
#define IndividualValueR 0.4
#define IndividualValueB 0.4

// 0.4 300.0 0.00001
#define PID_Kc 0.6
#define PID_Ti 0.0  // <- 0.0でもいいかも？
#define PID_Td 0.03
#define RATE   0.00006

#define PID_Kp 0.2
#define PID_Ki 0.0
#define PID_Kd 0.0




// csピン <-> mbedピン　の対応
    #define cs01 p23
    #define cs02 p22
    #define cs03 p15
    #define cs04 p14
    #define cs05 p16
    #define cs06 p17
    #define cs07 p18
    #define cs08 p19
    #define cs09 p20
    #define cs10 p30
    #define cs11 p29
    #define cs12 p26
    #define cs13 p25

// デバッグ用の定義_
    // #define PING_RLB_  // 超音波（左,右,後）
    #define MOTOR_     // モータドライバ
    // #define KICK_DIV_  // キッカー, ドリブラー
    // #define IR_FR_     // IRボール距離（右前）
    // #define IR_BR_     // IRボール距離（右後）
    // #define IR_BL_     // IRボール距離（左前）
    // #define IR_FL_     // IRボール距離（左後）
    #define IR__       // IRボール方向
    // #define SDCARD_    // SDカード
    #define PING_F_    // 超音波（前,斜め）
    #define IMU_       // IMU（LSM9DS0）
    #define DEBUG_     // デバッグ
// #define MESSAGE
// #define PROG_TIME

// SPI通信に関するパラメータ
#define SPI_WAIT_US 2   // SPI通信の待ち時間(us)
#define t_SS_SCLK 0.002 // SSを指定してから、通信開始までの待ち時間(us)

// IRに関するマスク用数値
    #define DEG045 0x80
    #define DEG090 0x40
    #define DEG135 0x20
    #define DEG180 0x10
    #define DEG225 0x08
    #define DEG270 0x04
    #define DEG315 0x02
    #define DEG000 0x01



/*--- ピンの設定 ---*/
// LEDの設定
DigitalOut led_Motor(LED1, 0);
DigitalOut led_IR(LED2, 0);
DigitalOut led_IMU(LED3, 0);
DigitalOut led4(LED4, 0);
// SW
DigitalIn SW(p24);  // OFF then 1, ON then 0
// ラインセンサの許可信号
DigitalOut linePermission(p5, 1);
// ラインセンサのinputピン
DigitalIn lineLeft(p6);
DigitalIn lineRight(p7);
// Kickerのピン設定
DigitalIn kickerInput(cs03);
DigitalOut kickerOuput(p8, 1);
// SPI通信用のピン設定
    SPI spi(p11, p12, p13); // mosi, miso, clk
    DigitalOut cs_PingRLB(cs01, 1);         // 超音波（左,右,後）
    DigitalOut cs_Motor(cs02, 1);           // モータドライバ
    // DigitalOut cs_KickerDribbler(cs03, 1);  // キッカー, ドリブラー
    DigitalOut cs_Speed(cs04, 1);           // マウスセンサ（ADNS9800）
    DigitalOut cs_IRFrDistance(cs05, 1);    // IRボール距離（右前）
    DigitalOut cs_IRBrDistance(cs06, 1);    // IRボール距離（右後）
    DigitalOut cs_IRBlDistance(cs07, 1);    // IRボール距離（左前）
    DigitalOut cs_IRFlDistance(cs08, 1);    // IRボール距離（左後）
    DigitalOut cs_IR(cs09, 1);              // IRボール方向
    DigitalOut cs_SDcard(cs10, 1);          // SDカード
    DigitalOut cs_PingF(cs11, 1);           // 超音波（前,斜め）
    DigitalOut cs_IMU(cs12, 1);             // IMU（LSM9DS0）
    DigitalOut cs_Debug(cs13, 1);           // デバッグ
Ticker pidupdata;
// デバッグ用のSerial通信用のピン設定
PwmOut sound(p21);
#if 0
Serial pc(p9, p10);   // tx, rx
#else
Serial pc(USBTX, USBRX);   // tx, rx
#endif

// #define XBEE
#ifdef XBEE
Serial xbee(p9, p10);
#endif



/*--- グローバル変数 ---*/
int LoopCount = 0;
PID pidController(PID_Kc, PID_Ti, PID_Td, RATE);    // Kp, Ki, Kd, RATE
float moveDirection = 0.0;  // ロボットの進む方向
// PINGに関する変数
uint8_t pingFF, pingFR, pingFL;
uint8_t pingBR, pingBL, pingR, pingL;
// ロボットの行動に関する変数
int Integral=0;
int Direction = 0;
int8_t Angle=0, ex_Angle=0;
int dx=0, ex_dx=0, ex2_dx=0;
float R = 2.3;
float Omega = 0.0;
float Theta = 0.0;

// デバッグに関する変数
Timer t;
uint8_t MyRemoteAddress[] = {0x00,0x13,0xa2,0x00,0x40,0x7c,0x04,0xbd};    // コントローラのアドレス


// プロトタイプ宣言
    inline int bitCount(uint8_t bits);
    inline void lineMove();
    inline void lineCheck();
    inline bool SWcheck();
    void PingRead();
    inline int IRRead();
    void ImuRead();
    void PwmLRB1(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed);
    void PwmLRB2(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed);
    void PwmLRB3(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed);
    void RegulationHeading(float* pPwmL, float* pPwmR, float* pPwmB);
    inline void Motor(float direction, float speed);
    inline float MoveDirection(int iBallDirection);
    bool spiChecking();
    inline void ControlCommand(uint8_t command);
    inline void serialChecking();




/*--- メッセージ表示用関数 ---*/
void msgD(int iTab, char *cMsg, int iValue, bool bNew=true)
{
    #ifdef MESSAGE
    for (int i=0; i<iTab; i++) pc.printf(" ");
    pc.printf("%s : %d", cMsg, iValue);
    if (bNew) pc.printf("\n");
    else pc.printf("   ");
    return;
    #endif
}
void msgF(int iTab, char *cMsg, float fValue, bool bNew=true)
{
    #ifdef MESSAGE
    for (int i=0; i<iTab; i++) pc.printf(" ");
    pc.printf("%s : %7.4f", cMsg, fValue);
    if (bNew) pc.printf("\n");
    else pc.printf("   ");
    return;
    #endif
}
void msgX(int iTab, char *cMsg, uint8_t ucValue, bool bNew=true)
{
    #ifdef MESSAGE
    for (int i=0; i<iTab; i++) pc.printf(" ");
    pc.printf("%s : %02x", cMsg, ucValue);
    if (bNew) pc.printf("\n");
    else pc.printf("   ");
    return;
    #endif
}
/*--- エラー表示用関数 ---*/
void errorMsgHex(int iTab, char *cMsg, uint8_t ucValue, bool bNew=true) // tab, message, value, new line
{
    #ifdef MESSAGE
    for (int i=0; i<iTab; i++) pc.printf(" ");
    pc.printf("error: %s   rx_data: %02x", cMsg, ucValue);
    if (bNew) pc.printf("\n");
    else pc.printf("   ");
    return;
    #endif
}
/*--- BitCount関数 ---*/
inline int bitCount(uint8_t bits)
{
    bits = (bits & 0x55) + (bits >> 1 & 0x55);
    bits = (bits & 0x33) + (bits >> 2 & 0x33);
    bits = (bits & 0x0f) + (bits >> 4 & 0x0f);
    bits = (bits & 0xff) + (bits >> 8 & 0xff);
    return (bits & 0xff) + (bits >>16 & 0xff);
}

inline void lineMove()
{
#if 1
    /*--- 反対方向に進む ---*/
    // if (Direction == 1000.0)
    //  return;
    // else if (Direction > 0.0)
    //  Motor(Direction-180.0, 1.0);
    // else
    //  Motor(Direction+180.0, 1.0);
    // wait(0.4);
    int stopDirecetion = 0;
    if (Direction > 0.0)
        stopDirecetion = Direction-180.0;
    else
        stopDirecetion = Direction+180.0;
    for (int i=0; i<700; i++) {
        ImuRead();
        pidController.setProcessValue(Angle);
        Omega = pidController.compute();
        Motor(1000.0, 0.0);
    }
    Motor(stopDirecetion, 0.4);
    wait(0.3);
#endif

#if 0
    /*--- 中央に戻ろうとする ---*/
    if (Direction == 1000.0)
        return;
    else if (lineLeft == 1 && lineRight == 0) {
        Motor(-90.0, 0.5);
    }
    else if (lineLeft == 0 && lineRight == 1) {
        Motor(90.0, 0.5);
    }
    else if (lineLeft == 1 && lineRight == 1) {
        if (Direction > 0.0)
            Motor(Direction-180.0, 1.0);
        else
            Motor(Direction+180.0, 1.0);
    }
    else return;
#endif

#if 0
    /*--- IRsensorの状況を見て行動 ---*/
    while (IRRead() != 1000.0) {
        ImuRead();
        pidController.setProcessValue(Angle);
        Omega = pidController.compute();
        Motor(moveDirection, 0.5);
        Direction = moveDirection;
    }
#endif
}


inline void lineCheck()
{
    // ラインセンサの状態をチェック
    if (lineLeft || lineRight) {
        lineMove();
    }
}

inline bool SWcheck()
{
    /*---
    ボタンが押されているかをチェック
    - ON-> '1'
    - OFF-> '0'
    ---*/
    if (!SW) {
        wait(0.3);
        if (SW) {
            wait(0.2);
            return 1;
        }
    }
    return 0;
}

/*--- 超音波による距離の測定 ---*/
void PingRead()
{
    cs_PingF = 0;
    pingFF = spi.write(0x01);  // 開始信号
    cs_PingF = 1;
    cs_PingF = 0;
    pingFF = spi.write(0x02);  // 正面
    cs_PingF = 1;
    cs_PingF = 0;
    pingFR = spi.write(0x03);  // 右前
    cs_PingF = 1;
    cs_PingF = 0;
    pingFL = spi.write(0x04);  // 左前
    cs_PingF = 1;

    #if 0
    cs_PingRLB = 0;
    pingL = spi.write(0x01);    // 開始信号
    cs_PingRLB = 1;
    cs_PingRLB = 0;
    pingL = spi.write(0x02);    // 左
    cs_PingRLB = 1;
    cs_PingRLB = 0;
    pingR = spi.write(0x03);    // 右
    cs_PingRLB = 1;
    cs_PingRLB = 0;
    pingBL = spi.write(0x04);   // 左後ろ
    cs_PingRLB = 1;
    cs_PingRLB = 0;
    pingBR = spi.write(0x05);   // 右後ろ
    cs_PingRLB = 1;
    #endif
}

/*--- ロボットから見たボールの方向を返す ---*/
inline int IRRead()
{
    /*---

    - IRのnearとfarのデータを取得
    - ボールのある方向を返す

    ---*/

    uint8_t near, far;
    // データを取得する
    cs_IR = 0;  wait_us(t_SS_SCLK); far = spi.write(0x00);  cs_IR = 1;
    wait_us(SPI_WAIT_US);
    cs_IR = 0;  wait_us(t_SS_SCLK); near = ~spi.write(0xFF);    cs_IR = 1;
    wait_us(SPI_WAIT_US);
    // データを解析して、進行方向を決定
    int iDirection = 0;
    int directionNear = 0;
    int directionFar  = 0;
    // int bitCountNear = bitCount(near);
    // int bitCountFar  = bitCount(far);

    // directionNear = ( ((DEG045 & near)?1:0) *  45
    //              + ((DEG090 & near)?1:0) *  90
    //              + ((DEG135 & near)?1:0) * 135
    //              + ((DEG180 & near)?1:0) * 180
    //              + ((DEG225 & near)?1:0) * 225
    //              + ((DEG270 & near)?1:0) * 270
    //              + ((DEG315 & near)?1:0) * 315
    //              + ((DEG000 & near)?1:0) * 000)
    //              / bitCountNear;
    // directionFar  = ( ((DEG045 & far)?1:0) *  45
    //              + ((DEG090 & far)?1:0) *  90
    //              + ((DEG135 & far)?1:0) * 135
    //              + ((DEG180 & far)?1:0) * 180
    //              + ((DEG225 & far)?1:0) * 225
    //              + ((DEG270 & far)?1:0) * 270
    //              + ((DEG315 & far)?1:0) * 315
    //              + ((DEG000 & far)?1:0) * 000)
    //              / bitCountFar;


    int iBitCountNear = 0;
    bool bBitFlag = false;
    int aiAngleData[] = {0, -45, -90, -135, 180, 135, 90, 45};
    int iAngleSum = 0;
    int iBaseAngle = 0;
    for (int i=0, j=0; i<8; i++) {
        if (near & 0x01) {
            iBitCountNear++;
            if (bBitFlag) {
                iAngleSum += aiAngleData[i-j];
            }
            else {
                bBitFlag = true;
                iBaseAngle = aiAngleData[i];
                j = i;
            }
        }
        near >>= 1;
    }
    iAngleSum /= iBitCountNear;
    directionNear = iBaseAngle + iAngleSum;

    if (iBitCountNear > 0) {
        iDirection = directionNear;
    }
    // else if (bitCountFar > 0) {
    //  iDirection = directionFar;
    // }
    else {
        iDirection = 1000.0;
    }

    // 角度を -180~+180 に修正
    if (iDirection > 180.0 && iDirection != 1000.0){
        iDirection -= 360.0;
    }
    else if (iDirection < -180.0 && iDirection != 1000.0){
        iDirection += 360.0;
    }

    msgD(0, "Direction", iDirection);

    return iDirection;
}

/*--- Union ---*/
union uni_8bits {
    uint8_t uint8;
    int8_t  int8;
};
/*--- IMUのデータを取得、Angleに代入 ---*/
void ImuRead()
{
    // 変数宣言
    int count = 5;
    uni_8bits data = {0};
    // データを受信
    do {
        cs_IMU = 0;
        data.uint8 = spi.write(0x50);
        cs_IMU = 1;
        count--;
        wait_us(SPI_WAIT_US);
    } while ((data.uint8==0xFF || data.uint8==0x00) && count);
    // 0~255 -> -128~127
    Angle = data.int8;
    msgD(0, "IMU", Angle);
}
/*--- 偏差から操作量Omegaを求める関数 ---*/
#if 0
void GetOmega() {
    // 偏差の差を求める
    if (-3<=Angle && Angle<=3) Angle=0;

    dx = Angle - ex_Angle;

    if (-3<=dx && dx<=3) {
        dx = 0;
        Angle = ex_Angle = (Angle+ex_Angle) / 2;
    }
    Integral /= 10;
    Integral += (Angle + ex_Angle) / 2;

    #if 1
    // 操作量Omegaを求める（PID制御）
    Omega = - (PID_Kp * (float)(Angle / 127.0))
            - (PID_Ki * (float)(Integral / 127.0))
            - (PID_Kd * (float)(dx / 127.0));
    #else
    // webページものも
    Omega+=  PID_Kp * (float)(dx)
            +PID_Ki * (float)(Angle)
            +PID_Kd * (float)((dx - ex_dx));
    #endif

    if (Omega < -1.0) {
        Omega = -1.0;
    }
    else if (Omega > 1.0) {
        Omega = 1.0;
    }

    msgF(0, "Omega", Omega);

    // データをずらす
    ex_dx = dx;
    ex_Angle = Angle;
}
#endif

/*--- pwmLRBを求める関数 ---*/
void PwmLRB1(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed)
{
    /*---

    このサイトに載っている公式を使用 (http://wiki.tokor.org/index.php?%A5%AA%A5%E0%A5%CB%A5%DB%A5%A4%A1%BC%A5%EB%A4%CB%A4%E8%A4%EB%C1%B4%CA%FD%B8%FE%B0%DC%C6%B0%BC%D6%A4%CE%C0%A9%B8%E6%A1%A1-%A1%A1%B5%A1%B3%A3%C0%A9%B8%E6)

    ---*/
    float pwmL =  vx * cos(Theta+PI*5/6)+ vy * sin(Theta+PI*5/6) - R*Omega;
    float pwmR =  vx * cos(Theta+PI/6)  + vy * sin(Theta+PI/6)   - R*Omega;
    float pwmB =  vx * cos(Theta-PI/2)  + vy * sin(Theta-PI/2)   - R*Omega;
    /*--- 最大値を求める ---*/
    float pwmMAX = fabs(pwmL);
    float aPwmR = fabs(pwmR);
    float aPwmB = fabs(pwmB);
    pwmMAX = (pwmMAX > aPwmR) ? pwmMAX : aPwmR;
    pwmMAX = (pwmMAX > aPwmB) ? pwmMAX : aPwmB;
    /*--- 求めた最大値で正規化 ---*/
    pwmL = pwmL / pwmMAX * speed * IndividualValueL;
    pwmR = pwmR / pwmMAX * speed * IndividualValueR;
    pwmB = pwmB / pwmMAX * speed * IndividualValueB;

    /*--- 値を反映 ---*/
    *pPwmL = pwmL;
    *pPwmR = pwmR;
    *pPwmB = pwmB;
}
void PwmLRB2(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed)
{
    float pwmL =  vx * cos(Theta+PI*5/6)+ vy * sin(Theta+PI*5/6);
    float pwmR =  vx * cos(Theta+PI/6)  + vy * sin(Theta+PI/6)  ;
    float pwmB =  vx * cos(Theta-PI/2)  + vy * sin(Theta-PI/2)  ;
    /*--- 最大値を求める ---*/
    float pwmMAX = fabs(pwmL);
    float aPwmR = fabs(pwmR);
    float aPwmB = fabs(pwmB);
    pwmMAX = (pwmMAX > aPwmR) ? pwmMAX : aPwmR;
    pwmMAX = (pwmMAX > aPwmB) ? pwmMAX : aPwmB;
    /*--- 求めた最大値で正規化 ---*/
    pwmL = pwmL / pwmMAX * speed * IndividualValueL * 0.7 - 0.3*Omega;
    pwmR = pwmR / pwmMAX * speed * IndividualValueR * 0.7 - 0.3*Omega;
    pwmB = pwmB / pwmMAX * speed * IndividualValueB * 0.7 - 0.3*Omega;

    /*--- 値を反映 ---*/
    *pPwmL = pwmL;
    *pPwmR = pwmR;
    *pPwmB = pwmB;
}
void PwmLRB3(float* pPwmL, float* pPwmR, float* pPwmB, float vx, float vy, float speed)
{
    /*---

    このサイトに載っていた方法の応用を使う (http://yukispanicworld.tumblr.com/post/106053191409)
    - 旋回は、求めた値の割合の変化分を足し合わせることによって行う

    ---*/
    float pwmL =  vx * cos(Theta+PI*5/6)+ vy * sin(Theta+PI*5/6);
    float pwmR =  vx * cos(Theta+PI/6)  + vy * sin(Theta+PI/6)  ;
    float pwmB =  vx * cos(Theta-PI/2)  + vy * sin(Theta-PI/2)  ;
    /*--- 最大値を求める ---*/
    float pwmMAX = fabs(pwmL);
    float aPwmR = fabs(pwmR);
    float aPwmB = fabs(pwmB);
    pwmMAX = (pwmMAX > aPwmR) ? pwmMAX : aPwmR;
    pwmMAX = (pwmMAX > aPwmB) ? pwmMAX : aPwmB;
    /*--- 求めた最大値で正規化 ---*/
    pwmL = pwmL / pwmMAX * speed;
    pwmR = pwmR / pwmMAX * speed;
    pwmB = pwmB / pwmMAX * speed;
    /*--- 旋回を考慮する ---*/
    float k = 0.07;
    pwmL += pwmL * k*Omega;
    pwmR += pwmR * k*Omega;
    pwmB += pwmB * k*Omega;
    /*--- 値を反映 ---*/
    *pPwmL = pwmL;
    *pPwmR = pwmR;
    *pPwmB = pwmB;
}
void RegulationHeading(float* pPwmL, float* pPwmR, float* pPwmB)
{
    float k = 1.0;  // 調節用の係数
    float pwmL =  - Omega * k * IndividualValueL;
    float pwmR =  - Omega * k * IndividualValueR;
    float pwmB =  - Omega * k * IndividualValueB;

    /*--- 値を反映 ---*/
    *pPwmL = pwmL;
    *pPwmR = pwmR;
    *pPwmB = pwmB;
}
/*--- モータの計算と通信 ---*/
inline void Motor(float direction, float speed)
{
    uint8_t rx_data;

    /*--- 計算に必要となる値 ---
    * 方向 ... direction (deg)
    * 速度 ... speed (m/s)
    * 機体の回転数 ... Omega (rad/s)
    * 機体の半径 ... R (m)
    * 機体の基準に対する傾き ... Theta (rad)
    * pwmL(左), pwmB(後), pwmR(右)
    --*/

    // 変数宣言
    float pwmL, pwmB, pwmR;



    // 停止コマンドのときの処理
    if (direction == 1000.0) {
        RegulationHeading(&pwmL, &pwmR, &pwmB);
    }
    // 普通のときの処理
    else {
        // Theta = Angle * Byte2Rad;
        direction *= Deg2Rad;
        float vx = cos(direction);
        float vy = sin(direction);

        PwmLRB1(&pwmL, &pwmR, &pwmB, vx, vy, speed);
        // PwmLRB2(&pwmL, &pwmR, &pwmB, vx, vy, speed);
        // PwmLRB3(&pwmL, &pwmR, &pwmB, vx, vy, speed);
    }

    msgF(0, "pwmL", pwmL, false);
    msgF(0, "pwmR", pwmR, false);
    msgF(0, "pwmB", pwmB, false);

    // 回転方向を決定　と　マイナスだったらプラスに変える
    uint8_t data0 = 0x3F;   // 0011 1111
    if (pwmL!=0.0 && pwmR!=0.0 && pwmB!=0.0) {
        // 左モータ
        if (pwmL>0) {
            data0 &= 0x37;  // 0011 0111
        }
        else {
            data0 &= 0x3B;  // 0011 1011
            pwmL *= -1.0;
        }
        if (pwmL>1.0) pwmL=1.0;

        // 右モータ
        if (pwmR>0) {
            data0 &= 0x3D;  // 0011 1101
        }
        else {
            data0 &= 0x3E;  // 0011 1110
            pwmR *= -1.0;
        }
        if (pwmR>1.0) pwmR=1.0;

        // 後モータ
        if (pwmB>0) {
            data0 &= 0x1F;  // 0001 1111
        }
        else {
            data0 &= 0x2F;  // 0010 1111
            pwmB *= -1.0;
        }
        if (pwmB>1.0) pwmB=1.0;
    }
    else data0 = 0x00;

    uint8_t dataL = (uint8_t)(pwmL * 255.0);    // 左モータの回転数
    uint8_t dataR = (uint8_t)(pwmR * 255.0);    // 右モータの回転数
    uint8_t dataB = (uint8_t)(pwmB * 255.0);    // 後モータの回転数
    if (dataL == 0x00) {
        data0 &= 0x33;  // 0011 0011
        data0 |= 0x0C;
        dataL++;
    }
    if (dataR == 0x00) {
        data0 &= 0x3C;  // 0011 1100
        data0 |= 0x03;
        dataR++;
    }
    if (dataB == 0x00) {
        data0 &= 0x0F;  // 0000 1111
        data0 |= 0x30;
        dataB++;
    }

    // データ確認用の値を埋め込む
    dataL = dataL & 0xFC | 0x01;
    dataR = dataR & 0xFC | 0x02;
    dataB = dataB & 0xFC | 0x03;

    // デバッグのため、送信データを表示
    // pc.printf("  ");
    msgX(0, "data0", data0, false);
    msgX(0, "dataL", dataL, false);
    msgX(0, "dataR", dataR, false);
    msgX(0, "dataB", dataB, true);

    // データを送信
    int error = 0;
    int count = 0;
    while (1) {
        // 通信開始の合図
            cs_Motor=0; wait_us(t_SS_SCLK);
            rx_data=spi.write((uint8_t)(error%4));  wait_us(t_SS_SCLK);
            cs_Motor=1;
            if (rx_data!=0xAA) {
                errorMsgHex(0, "start", rx_data);
                // errorMsgHex(0, "start_data", (uint8_t)(error%4));
            }
            wait_us(SPI_WAIT_US);
        // 回転方向のデータ
            do {
                cs_Motor=0; wait_us(t_SS_SCLK);
                rx_data=spi.write(data0);   wait_us(t_SS_SCLK);
                cs_Motor=1;
                count++;
            } while(rx_data != 0x00 && count < 5);
            if (rx_data!=0x00) {
                errorMsgHex(0, "data0", rx_data);
                led_Motor = 0;
                error++;
                continue;
            } else led_Motor = 1;
            msgD(0, "count", count);
            count = 0;
            wait_us(SPI_WAIT_US);
        // MotorLのデータ
            do {
                cs_Motor=0; wait_us(t_SS_SCLK);
                rx_data=spi.write(dataL);   wait_us(t_SS_SCLK);
                cs_Motor=1;
                count++;
            } while(rx_data != data0 && count < 5);
            if (rx_data!=data0) {
                errorMsgHex(0, "dataL", rx_data);
                led_Motor = 0;
                error++;
                continue;
            } else led_Motor = 1;
            msgD(0, "count", count);
            count = 0;
            wait_us(SPI_WAIT_US);
        // MotorRのデータ
            do {
                cs_Motor=0; wait_us(t_SS_SCLK);
                rx_data=spi.write(dataR);   wait_us(t_SS_SCLK);
                cs_Motor=1;
                count++;
            } while(rx_data != dataL && count < 5);
            if (rx_data!=dataL) {
                errorMsgHex(0, "dataR", rx_data);
                led_Motor = 0;
                error++;
                continue;
            } else led_Motor = 1;
            msgD(0, "count", count);
            count = 0;
            wait_us(SPI_WAIT_US);
        // MotorBのデータ
            do {
                cs_Motor=0; wait_us(t_SS_SCLK);
                rx_data=spi.write(dataB);   wait_us(t_SS_SCLK);
                cs_Motor=1;
                count++;
            } while(rx_data != dataR && count < 5);
            if (rx_data!=dataR) {
                errorMsgHex(0, "dataB", rx_data);
                led_Motor = 0;
                error++;
                continue;
            } else led_Motor = 1;
            msgD(0, "count", count);
            count = 0;
            wait_us(SPI_WAIT_US);

        break;
    }

    return;
}

/*--- ボールの方向から、ロボットの進行方向を決める関数 ---*/
inline float MoveDirection(int iBallDirection)
{
    /*---

    - ボールの方向から、ロボットの進行方向を求める関数
    - -180 < iBallDirection < +180

    ---*/

    if (iBallDirection == 1000.0) return 1000.0;

    float fMoveDirection = 0.0;
    if (-30<iBallDirection && iBallDirection<30)    fMoveDirection = 0.0;
    else if (iBallDirection > 0)                    fMoveDirection = iBallDirection+60.0;
    else if (iBallDirection < 0)                    fMoveDirection = iBallDirection-60.0;
    else                                            fMoveDirection = 180.0;

    // 角度を -180~+180 に修正
    if (fMoveDirection > 180.0){
        return fMoveDirection - 360.0;
    }
    if (fMoveDirection < -180.0){
        return fMoveDirection + 360.0;
    }

    return fMoveDirection;
}


/*--- SPI通信確認 ---*/
bool spiChecking()
{
    int spicheck = 0;
    uint8_t data = 0;
    uint8_t missData1 = 0;
    uint8_t missData2 = 0;
    uint8_t missData3 = 0;
    uint8_t missData4 = 0;
    pc.printf("\nChecking SPI...\n");

    #ifdef PING_RLB_
        cs_PingRLB        = 0;  missData1 = spi.write(0x50);    cs_PingRLB        = 1;  wait_us(SPI_WAIT_US);
        cs_PingRLB        = 0;  missData2 = spi.write(0x50);    cs_PingRLB        = 1;  wait_us(SPI_WAIT_US);
        cs_PingRLB        = 0;  data = spi.write(0x50);         cs_PingRLB        = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##  mbed <-> LPC1114 <-> Ping(RLB)\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef MOTOR_
        led_Motor = 0;
        // ちゃんと値が入っているかをチェック
        cs_Motor          = 0;  missData1 = spi.write(0x51);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  missData2 = spi.write(0x51);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  missData3 = spi.write(0x51);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  missData4 = spi.write(0x51);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  data = spi.write(0x01);         cs_Motor          = 1;
        pc.printf("    ReturnValue1 = %x\n    ReturnValue2 = %x\n    ReturnValue3 = %x\n    ReturnValue4 = %x\n    ReturnValue5 = %x\n", missData1, missData2, missData3, missData4, data);
        // SPI通信のチェック
        cs_Motor          = 0;  missData1 = spi.write(0x50);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  missData2 = spi.write(0x50);    cs_Motor          = 1;  wait_us(SPI_WAIT_US);
        cs_Motor          = 0;  data = spi.write(0x01);         cs_Motor          = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> CY8C29466 <-> Motor\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
        else led_Motor=1;
    #endif

    #ifdef KICK_DIV_
        cs_KickerDribbler = 0;  missData1 = spi.write(0x50);    cs_KickerDribbler = 1;  wait_us(SPI_WAIT_US);
        cs_KickerDribbler = 0;  missData2 = spi.write(0x50);    cs_KickerDribbler = 1;  wait_us(SPI_WAIT_US);
        cs_KickerDribbler = 0;
        data = spi.write(0x01);
        cs_KickerDribbler = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> CY8C29466 <-> Kicker&Dribbler\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IR_FR_
        cs_IRFrDistance   = 0;  missData1 = spi.write(0x50);    cs_IRFrDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRFrDistance   = 0;  missData2 = spi.write(0x50);    cs_IRFrDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRFrDistance   = 0;  data = spi.write(0x01);         cs_IRFrDistance   = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> CY8C24123A <-> IR_FR_distance\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IR_BR_
        cs_IRBrDistance   = 0;  missData1 = spi.write(0x50);    cs_IRBrDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRBrDistance   = 0;  missData2 = spi.write(0x50);    cs_IRBrDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRBrDistance   = 0;  data = spi.write(0x01);         cs_IRBrDistance   = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> CY8C24123A <-> IR_BR_distance\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IR_BL_
        cs_IRBlDistance   = 0;  missData1 = spi.write(0x50);    cs_IRBlDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRBlDistance   = 0;  missData2 = spi.write(0x50);    cs_IRBlDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRBlDistance   = 0;  data = spi.write(0x01);         cs_IRBlDistance   = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed  <-> CY8C24123A <-> IR_BL_distance\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IR_FL_
        cs_IRFlDistance   = 0;  missData1 = spi.write(0x50);    cs_IRFlDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRFlDistance   = 0;  missData2 = spi.write(0x50);    cs_IRFlDistance   = 1;  wait_us(SPI_WAIT_US);
        cs_IRFlDistance   = 0;  data = spi.write(0x01);         cs_IRFlDistance   = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed  <-> CY8C24123A <-> IR_FL_distance\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IR__
        led_IR = 0;
        cs_IR             = 0;  missData1 = spi.write(0x50);    cs_IR             = 1;  wait_us(SPI_WAIT_US);
        cs_IR             = 0;  missData2 = spi.write(0x50);    cs_IR             = 1;  wait_us(SPI_WAIT_US);
        cs_IR             = 0;  data = spi.write(0x01);         cs_IR             = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed  <-> CY8C29466 <-> IR_direction\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
        else led_IR=1;
    #endif

    #ifdef PING_F_
        cs_PingF          = 0;  missData1 = spi.write(0x50);    cs_PingF          = 1;  wait_us(SPI_WAIT_US);
        cs_PingF          = 0;  missData2 = spi.write(0x50);    cs_PingF          = 1;  wait_us(SPI_WAIT_US);
        cs_PingF          = 0;  data = spi.write(0x01);         cs_PingF          = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> LPC1114 <-> Ping(F)\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    #ifdef IMU_
        led_IMU = 0;
        int count = 5;
        do {
            cs_IMU            = 0;  data = spi.write(0x50); cs_IMU            = 1;  wait_us(SPI_WAIT_US);
            pc.printf("data : %x\n", data);
            count--;
        } while ((data==0xFF || data==0x00) && count);

        if (count < 1) {
            spicheck++;
            pc.printf("##Bad##   mbed <-> LPC1114 <-> LSM9DS0\n");
        }
        else led_IMU=1;
        pc.printf("    IMU : %d\n", data);
    #endif

    #ifdef DEBUG_
        cs_Debug          = 0;  missData1 = spi.write(0x50);    cs_Debug          = 1;  wait_us(SPI_WAIT_US);
        cs_Debug          = 0;  missData2 = spi.write(0x50);    cs_Debug          = 1;  wait_us(SPI_WAIT_US);
        cs_Debug          = 0;  data = spi.write(0x01);         cs_Debug          = 1;
        if (data != 0xAA) {
            pc.printf("##Bad##   mbed <-> LPC1114_DEBUG\n");
            pc.printf("    MissData1 = %x\n    MissData2 = %x\n    MissData3 = %x\n", missData1, missData2, data);
            spicheck++;
        }
    #endif

    // 最後にメッセージを表示
    if (spicheck) {
        pc.printf("waring : %d\n", spicheck);
        led4 = 0;
        // while (!pc.readable());      // PCから指示があるまで待つ
        // pc.getc();
        return 0;
        wait(2);
    }
    else {
        pc.printf("All SPIdevices complete!!\n");
        led4 = 1;
        return 1;
        wait(1);
    }
    pc.printf("\n");
}


/*--- PCとの通信用関数 ---*/
inline void ControlCommand(uint8_t command)
{
    /*--- コマンド一覧 ---
    * 必要なparameter
    0x01    PID_Kp      PID制御の比例
    0x02    PID_Ki      PID制御の積分
    0x03    PID_Kd      PID制御の微分
    0x04    Robo_R      機体の半径

    * 必要なcommand
    0x10    modeStop        停止コマンド
    0x11    modeBattle1     戦闘モード1
    0x12    modeBattle2     戦闘モード2
    0x13    modeDebug       デバッグモード
    0x21    goFlont         前進
    0x22    goBack          後退
    0x23    goLeft          左
    0x24    goRight         右
    0x25    turnCW          時計回りに一回転
    0x26    turnCCW         反時計回りに一回転
    0x31    checkSensor     センサーチェック
    0x31    checkIMU        IMUからの情報を画面に表示
    0xAA    Reset           リセット（SDカードから、値を再読み込み）
    ---*/
    float wait_time = 1.0;
    switch (command)
    {
        case 0xAA:
            break;
        case 0x13:  // modeDebug
            spiChecking();
            wait(3);
            break;
        case 0x21:  // goFlont
            Motor(0.0, 0.3);
            wait(wait_time);
            break;
        case 0x22:  // goBack
            Motor(180.0, 0.3);
            wait(wait_time);
            break;
        case 0x23:  // goLeft
            Motor(-90.0, 0.3);
            wait(wait_time);
            break;
        case 0x24:  // goRight
            Motor(90.0, 0.3);
            wait(wait_time);
            break;
        default:
            break;
    }
}


#ifdef XBEE
/*--- 送信用関数 ---*/
void xbeeTx(uint8_t *address, uint8_t data) {
    int i=0;

    xbee.putc(0x7E);
    xbee.putc(0x00);    // フレーム長
    xbee.putc(0x0F);
    xbee.putc(0x10);    // フレームタイプ
    xbee.putc(0x01);    // フレームID
    for (int i=0; i<8; i++)
        xbee.putc(address[i]);  // 62bit宛先アドレス
    xbee.putc(0xFF);    // 16bit宛先ネットワークアドレス
    xbee.putc(0xFE);
    xbee.putc(0x00);    // ブロードキャスト半径
    xbee.putc(0x00);    // オプション
    xbee.putc(data);    // RFデータ
    // チェックサムの計算
    uint8_t checksum = 0xFF - (0x10+0x01+0xFF+0xFE+data);
    for (i=0; i<8; i++)  checksum -= address[i];
    xbee.putc(checksum);    // チェックサム

    // 送信ステータスの確認
    if (xbee.readable()) {
        if (xbee.getc() == 0x7E) {
            for (i=0; i<8; i++) xbee.getc();
            if (xbee.getc() == 0x00)
                // pc.printf("Send success!\n");
            else
                // pc.printf("Send missed.\n");
            for (i=9; i<10; i++) xbee.getc();
        }
    }
}
/*--- 受信用関数 ---*/
xbeeAPI xbeeRx() {
    xbeeAPI returnApi;  // 返り値用の変数
    returnApi.reset();
    if (xbee.readable() > 0) {
        // pc.printf("readable\n");
    if (xbee.getc() == 0x7E) {
        // pc.printf("startBit\n");
        int i=0;
        uint8_t buffer[18];

        buffer[0] = 0x7E;
        for (i=1; i<18; i++) {
            buffer[i] = xbee.getc();
        }
        // 送信元アドレスを抽出
        for (i=0; i<8; i++) {
            returnApi.address[i] = buffer[i+4];
        }
        // 受信データを抽出
        returnApi.direction = buffer[15];
        returnApi.turning   = buffer[16];

        // データをPCに表示
        // for (i=0; i<8; i++)  pc.printf("%x", returnApi.address[i]);
        // pc.printf("\n");
        // pc.printf("%x\n", returnApi.direction);
        // pc.printf("%x\n", returnApi.turning);
    }
    }
    return returnApi;
}
#endif

/*--- serialの通信チェック関数 ---*/
inline void serialChecking()
{
    if ( pc.readable() ) {
        uint8_t command = pc.getc();
        printf("command : %x\n", pc.getc());
        ControlCommand(command);
    }
}




int main()
{
    /*--- 変数宣言 ---*/
    int direction = 0;  // ボールのある方向
    float speed = 0.0;  // ロボットの速度

    /*--- 初期処理 ---*/
    // 音
    float mm[]={mC,mD,mE,mF,mG,mA,mB,mC*2}; // ドレミファソラシ
    sound.period(1.0/mm[0]);
    sound = 0.03;
    // Serialの設定
    pc.baud(115200);
    // SPIの初期設定
    spi.format(8, 3); // DataFlameのビット数(4~16)
    spi.frequency(1000000); // クロック周波数(デフォルト:1MHz)
    // ラインセンサに許可信号を
    linePermission=1;
    wait(1);  // 各モジュールのセットアップが完了するまで待つ

    // PIDライブラリの初期設定
    pidController.setInputLimits(-128.0, 127.0);
    pidController.setOutputLimits(-1.0, 1.0);
    pidController.setBias(0.0);
    pidController.setMode(AUTO_MODE);
    pidController.setSetPoint(0.0);
    // pidupdata.attach(&ImuRead, RATE);

    // lineの設定
    // lineLeft.rise(&lineMove);
    // lineRight.rise(&lineMove);

    // SPIの通信確認
    while ( !spiChecking() );


    /*--- スタートの合図を待つ ---*/
    kickerOuput = 1;
    // SWが押されるまで待つ
    sound.period(1.0/mm[1]);
    sound = 0.06;
    while(!SWcheck());
    // 音を止める
    sound = 0.00;


    /*--- メイン処理 ---*/
    while(true) {

        #ifdef PROG_TIME
        t.start();
        #endif

        /*--- pcとの通信があったかをCheck ---*/
        // serialChecking();
        /*--- ストップボタンをCheck ---*/
        if (SWcheck()) {
            Motor(0.0, 0.0);
            sound.period(1.0/mm[1]);
            sound = 0.06;
            while(!SWcheck());
            sound = 0.00;
        }
        lineCheck();

        // PingRead();

        /*--- 現在の方角を確認 ---*/
        // Kicker使用時は、IMUの値は参考にならないので無視する
        // if (kickerInput) {
        //  kickerOuput = 1;
        //  wait(0.8);
        //  kickerOuput = 0;
        // } else {
        //  ImuRead();
        //  pidController.setProcessValue(Angle);
        // }
        ImuRead();
        pidController.setProcessValue(Angle);
        lineCheck();

        /*--- ロボットから見たボールの方向を確認 ---*/
        direction = IRRead();
        lineCheck();

        /*--- ボールの方向の絶対値を算出 ---*/
        if (direction != 1000.0) {
            direction += Angle;
            if (direction > 180.0){
                direction -= 360.0;
            }
            else if (direction < -180.0){
                direction += 360.0;
            }
        }
        lineCheck();

        /*--- ロボットの進行方向を決定 ---*/
        moveDirection = MoveDirection(direction);
        // pc.printf("%f\n", moveDirection);
        lineCheck();

        /*--- 修正のための角速度を算出 ---*/
        // GetOmega();
        Omega = pidController.compute();
        if (-5<Angle && Angle<5) {
            Omega = 0.0;
        }
        lineCheck();

        /*--- 実際にロボットを動かす ---*/
        // moveDirection = 0.0;
        // moveDirection = 1000.0;
        Motor(moveDirection, 0.4);
        lineCheck();

        Direction = moveDirection;


        #ifdef PROG_TIME
        t.stop();
        pc.printf("%f\n", t.read());
        t.reset();
        #endif

        LoopCount++;
    }
}












