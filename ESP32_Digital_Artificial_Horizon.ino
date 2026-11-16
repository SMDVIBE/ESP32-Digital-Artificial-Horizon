#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ESP32-C3 + GC9A01A 240x240 + MPU6050
// Round-screen artificial horizon. Uses the proven SPI initialization
// from the working TurnCoordinator_GC9A01A.ino. No full-screen canvas.

#define TFT_SCLK 4
#define TFT_MOSI 6
#define TFT_CS   7
#define TFT_DC   3
#define TFT_RST  10
#define MPU_SDA  0
#define MPU_SCL  1
#define BUTTON_PIN 5

Adafruit_GC9A01A tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
Adafruit_MPU6050 mpu;
GFXcanvas16 frame(240, 240); // Compose a complete frame in RAM, then transfer it once.

const int W=240, H=240, CX=120, CY=120;
const float ROLL_LIMIT=60.0f;
float rollZero=0, pitchZero=0, rollDeg=0, pitchDeg=0;
float filteredRoll=0, filteredPitch=0;
uint8_t style=0;
bool lastButton=HIGH, holdHandled=false;
unsigned long pressStart=0, lastFrame=0;

const uint16_t SKY_BLUE=0x249F, GROUND_BROWN=0x9A60;
const uint16_t SKY_NIGHT=0x001F, GROUND_NIGHT=0x4208;
const uint16_t WHITE=0xFFFF, YELLOW=0xFFE0, BLACK=0x0000;
const uint16_t GREEN=0x07E0, CYAN=0x07FF;
uint16_t skyColor(){return style==0?SKY_BLUE:SKY_NIGHT;}
uint16_t groundColor(){return style==0?GROUND_BROWN:GROUND_NIGHT;}

uint16_t blend565(uint16_t a,uint16_t b,float t){
  t=constrain(t,0.0f,1.0f);
  int ar=(a>>11)&31, ag=(a>>5)&63, ab=a&31;
  int br=(b>>11)&31, bg=(b>>5)&63, bb=b&31;
  int r=(int)lroundf(ar+(br-ar)*t);
  int g=(int)lroundf(ag+(bg-ag)*t);
  int bl=(int)lroundf(ab+(bb-ab)*t);
  return (uint16_t)((r<<11)|(g<<5)|bl);
}

void drawPixelClipped(int x,int y,uint16_t c){
  int dx=x-CX, dy=y-CY;
  if(dx*dx+dy*dy <= 119*119) frame.drawPixel(x,y,c);
}
void drawLineClippedCircle(float x1,float y1,float x2,float y2,uint16_t c){
  float dx=x2-x1, dy=y2-y1;
  int steps=(int)ceilf(fmaxf(fabsf(dx),fabsf(dy)));
  if(steps<1) steps=1;
  for(int i=0;i<=steps;i++){
    float q=(float)i/steps;
    drawPixelClipped((int)lroundf(x1+dx*q),(int)lroundf(y1+dy*q),c);
  }
}

void drawCircularBezel(){
  // Keep the original circular face geometry; the perimeter is antialiased last.
  frame.fillScreen(BLACK);
  frame.fillCircle(CX,CY,119,skyColor());
}

void drawHorizonBackground(){
  // Original crisp, pixel-stepped horizon rendering; no antialiasing.
  float slope=tanf(rollDeg*DEG_TO_RAD);
  float pitchShift=constrain(pitchDeg*2.5f,-62.5f,62.5f);

  for(int y=CY-119;y<=CY+119;y++){
    int dy=y-CY;
    int half=(int)floorf(sqrtf(119.0f*119.0f-dy*dy));
    int xL=CX-half, xR=CX+half;
    frame.drawFastHLine(xL,y,xR-xL+1,skyColor());

    if(fabsf(slope)<0.00001f){
      if((float)y>=CY+pitchShift)
        frame.drawFastHLine(xL,y,xR-xL+1,groundColor());
    }else{
      float xCross=CX+((float)y-CY-pitchShift)/slope;
      if(slope>0.0f){
        int end=(int)floorf(xCross);
        if(end>xR) end=xR;
        if(end>=xL) frame.drawFastHLine(xL,y,end-xL+1,groundColor());
      }else{
        int start=(int)ceilf(xCross);
        if(start<xL) start=xL;
        if(start<=xR) frame.drawFastHLine(start,y,xR-start+1,groundColor());
      }
    }
  }
}

void drawPitchLadder(){
  float rad=rollDeg*DEG_TO_RAD;
  float ux=cosf(rad), uy=sinf(rad);
  float nx=-sinf(rad), ny=cosf(rad);
  float shift=constrain(pitchDeg*2.5f,-62.5f,62.5f);
  for(int deg=-30;deg<=30;deg+=10){
    if(deg==0) continue;
    float offset=-deg*2.5f+shift;
    float px=CX+nx*offset, py=CY+ny*offset;
    float halfLen=(deg%20==0)?27.0f:17.0f;
    drawLineClippedCircle(px-ux*halfLen,py-uy*halfLen,px+ux*halfLen,py+uy*halfLen,WHITE);
    if(deg%20==0){
      int tx=(int)(px+nx*5-7), ty=(int)(py+ny*5-4);
      if((tx-CX)*(tx-CX)+(ty-CY)*(ty-CY)<82*82){
        frame.setTextColor(WHITE); frame.setTextSize(1); frame.setTextWrap(false);
        frame.setCursor(tx,ty); frame.print(abs(deg));
      }
    }
  }
}

void drawRollPointer(){
  // Rotate the entire pointer radially around the dial centre.
  // The tip stays on the 91 px arc; the base stays at radius 81 px.
  float a=(constrain(rollDeg,-ROLL_LIMIT,ROLL_LIMIT)-90.0f)*DEG_TO_RAD;
  float ux=cosf(a), uy=sinf(a);
  float tx=-uy, ty=ux;
  int tipX=CX+(int)lroundf(ux*91.0f);
  int tipY=CY+(int)lroundf(uy*91.0f);
  int baseCX=CX+(int)lroundf(ux*81.0f);
  int baseCY=CY+(int)lroundf(uy*81.0f);
  int x1=(int)lroundf(baseCX-tx*6.0f);
  int y1=(int)lroundf(baseCY-ty*6.0f);
  int x2=(int)lroundf(baseCX+tx*6.0f);
  int y2=(int)lroundf(baseCY+ty*6.0f);
  frame.fillTriangle(tipX,tipY,x1,y1,x2,y2,YELLOW);
}

void drawRollScaleLabels(){
  // 20, 40 and 60 degree labels sit just outside the scale ticks on both sides.
  frame.setTextColor(WHITE);
  frame.setTextSize(1);
  frame.setTextWrap(false);
  for(int angle=-60;angle<=60;angle+=20){
    if(angle==0) continue;
    float a=(angle-90.0f)*DEG_TO_RAD;
    int x=CX+(int)lroundf(cosf(a)*108.0f);
    int y=CY+(int)lroundf(sinf(a)*108.0f);
    frame.setCursor(x-9,y-3);
    frame.print(abs(angle));
    frame.drawCircle(x+5,y-1,1,WHITE);
  }
}

void drawReadouts(){
  // Compact values in the lower part of the round face.
  frame.fillRoundRect(54,188,132,27,5,BLACK);
  frame.drawRoundRect(54,188,132,27,5,0x7BEF);
  frame.setTextSize(1); frame.setTextWrap(false);
  frame.setTextColor(GREEN); frame.setCursor(63,198);
  frame.print("R:"); frame.print(rollDeg,1); frame.print(" deg");
  frame.setTextColor(CYAN); frame.setCursor(127,198);
  frame.print("P:"); frame.print(pitchDeg,1);
}

void renderHorizon(){
  drawCircularBezel();
  drawHorizonBackground();
  // Redraw scale and aircraft reference over the moving horizon.
  for(int a=-60;a<=60;a+=10){
    float rad=(a-90)*DEG_TO_RAD;
    int x1=CX+(int)lroundf(cosf(rad)*96), y1=CY+(int)lroundf(sinf(rad)*96);
    int len=(a%30==0)?9:5;
    int x2=CX+(int)lroundf(cosf(rad)*(96-len)), y2=CY+(int)lroundf(sinf(rad)*(96-len));
    drawLineClippedCircle(x1,y1,x2,y2,WHITE);
  }
  drawPitchLadder();
  drawRollScaleLabels();
  // aircraft reference fixed at screen centre
  // Same aircraft reference geometry, thickened to 3 px by adding
  // one parallel pixel above and one below each existing yellow segment.
  frame.drawLine(55,119,92,119,YELLOW); frame.drawLine(148,119,185,119,YELLOW);
  frame.drawLine(92,119,105,129,YELLOW); frame.drawLine(148,119,135,129,YELLOW);
  frame.drawLine(105,129,135,129,YELLOW);
  frame.drawLine(55,120,92,120,YELLOW); frame.drawLine(148,120,185,120,YELLOW);
  frame.drawLine(92,120,105,130,YELLOW); frame.drawLine(148,120,135,130,YELLOW);
  frame.drawLine(105,130,135,130,YELLOW);
  frame.drawLine(55,121,92,121,YELLOW); frame.drawLine(148,121,185,121,YELLOW);
  frame.drawLine(92,121,105,131,YELLOW); frame.drawLine(148,121,135,131,YELLOW);
  frame.drawLine(105,131,135,131,YELLOW);
  frame.fillCircle(120,120,3,YELLOW);
  drawRollPointer();
  drawReadouts();
  // Single-pixel white perimeter around the existing circular display area.
  frame.drawCircle(CX, CY, 119, WHITE);
  tft.drawRGBBitmap(0, 0, frame.getBuffer(), 240, 240);
}

void calibrateLevel(){
  const int samples=80; float sumRoll=0,sumPitch=0;
  sensors_event_t a,g,temp;
  for(int i=0;i<samples;i++){
    mpu.getEvent(&a,&g,&temp);
    // Sensor axes are crossed relative to the instrument orientation:
    // MPU pitch axis drives displayed roll; MPU roll axis drives displayed pitch.
    float sensorRoll=atan2f(a.acceleration.y,a.acceleration.z)*RAD_TO_DEG;
    float sensorPitch=atan2f(-a.acceleration.x,sqrtf(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z))*RAD_TO_DEG;
    sumRoll+=sensorPitch;
    sumPitch+=sensorRoll;
    delay(8);
  }
  rollZero=sumRoll/samples; pitchZero=sumPitch/samples;
}
void readMPU(){
  sensors_event_t a,g,temp; mpu.getEvent(&a,&g,&temp);
  float sensorRoll=atan2f(a.acceleration.y,a.acceleration.z)*RAD_TO_DEG;
  float sensorPitch=atan2f(-a.acceleration.x,sqrtf(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z))*RAD_TO_DEG;
  // Swap axes to match the physical mounting of the MPU6050.
  rollDeg=sensorPitch-rollZero;
  pitchDeg=sensorRoll-pitchZero;
  filteredRoll=filteredRoll*0.78f+rollDeg*0.22f;
  filteredPitch=filteredPitch*0.78f+pitchDeg*0.22f;
  rollDeg=constrain(filteredRoll,-ROLL_LIMIT,ROLL_LIMIT);
  pitchDeg=constrain(filteredPitch,-45.0f,45.0f);
}
void handleButton(){
  bool now=digitalRead(BUTTON_PIN);
  if(lastButton==HIGH && now==LOW){pressStart=millis();holdHandled=false;}
  if(now==LOW && !holdHandled && millis()-pressStart>=2000){
    calibrateLevel(); filteredRoll=filteredPitch=rollDeg=pitchDeg=0; holdHandled=true;
  }
  if(lastButton==LOW && now==HIGH && !holdHandled) style=(style+1)%2;
  lastButton=now;
}

void setup(){
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  Wire.begin(MPU_SDA,MPU_SCL); Wire.setClock(400000);
  SPI.begin(TFT_SCLK,-1,TFT_MOSI,TFT_CS);
  tft.begin(40000000);
  tft.setRotation(0);
  if (frame.getBuffer() == nullptr) {
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE);
    tft.setCursor(20, 110);
    tft.print("NO FRAME RAM");
    while (1) delay(100);
  }
  tft.fillScreen(BLACK);
  if(!mpu.begin(0x68,&Wire)){
    tft.setTextColor(WHITE); tft.setCursor(45,115); tft.print("MPU6050 ERROR");
    while(1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(200); calibrateLevel(); filteredRoll=filteredPitch=0;
}
void loop(){
  handleButton();
  if(millis()-lastFrame>=50){lastFrame=millis(); readMPU(); renderHorizon();}
}
