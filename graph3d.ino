#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_ADDR 0x3C

constexpr double TAU = 2 * PI;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float Axcam = atan(M_SQRT2), Aycam = 0, Azcam = 0;
float Xcam = 0, Ycam = -6.0 * M_SQRT2, Zcam = 6;
float xdisp = 0, ydisp = 0, zdisp = 62.43;

void setup() {
  // Serial.begin(115200);

  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      ;
  }
}

void loop() {
  constexpr int FDELAY = 20; // 40 ms between frames
  constexpr int PIXTOTAL = SCREEN_WIDTH * SCREEN_HEIGHT;

  static int prevM = millis();
  const int timeDelta = millis() - prevM;

  oled.clearDisplay();
  constexpr int GSIZE = 5;
  for (int i = -GSIZE; i <= GSIZE; ++i) {
    line3d(i, -GSIZE, 0, i, GSIZE, 0, WHITE);
    line3d(-GSIZE, i, 0, GSIZE, i, 0, WHITE);
  }
  oled.display();

  // https://www.desmos.com/calculator/xhun5i7v6z
  Azcam = Azcam + 0.03;
  if (Azcam > TAU) {
    Azcam = Azcam - TAU;
  }
  Xcam = 6 * M_SQRT2 * cos(Azcam - PI / 2);
  Ycam = 6 * M_SQRT2 * sin(Azcam - PI / 2);
  if (timeDelta < FDELAY) {
    delay(FDELAY - timeDelta);
  }
  prevM = millis();
}

bool f3d(float x, float y, float z, float *xout, float *yout) {
  float Ccx = cos(Axcam);
  float Csx = sin(Axcam);
  float Ccy = cos(Aycam);
  float Csy = sin(Aycam);
  float Ccz = cos(Azcam);
  float Csz = sin(Azcam);
  float xo = Ccy * Ccz * (x - Xcam) + Ccy * Csz * (y - Ycam) + Csy * (Zcam - z);
  float yo = (Csx * Csy * Ccz - Ccx * Csz) * (x - Xcam) +
      (Csx * Csy * Csz + Ccx * Ccz) * (y - Ycam) + Csx * Ccy * (z - Zcam);
  float zo = (Ccx * Csy * Ccz + Csx * Csz) * (x - Xcam) +
      (Ccx * Csy * Csz - Csx * Ccz) * (y - Ycam) + Ccx * Ccy * (z - Zcam);
  return pr3d(xo, yo, zo, xout, yout);
}

bool pr3d(float x, float y, float z, float *xout, float *yout) {
  *xout = SCREEN_WIDTH / 2.0 + xdisp - zdisp * x / z;
  *yout = SCREEN_HEIGHT / 2.0 - ydisp + zdisp * y / z;
  return z < 0;
}

void line3d(float x0, float y0, float z0, float x1, float y1, float z1,
            uint16_t color) {
  float v0, w0;
  float v1, w1;
  bool trim;

  trim = f3d(x0, y0, z0, &v0, &w0);
  trim = trim && f3d(x1, y1, z1, &v1, &w1);

  if (trim) {
    oled.drawLine(v0, w0, v1, w1, color);
  }
}

void pixel3d(float x0, float y0, float z0, uint16_t color) {
  float v0, w0;
  bool trim;

  trim = f3d(x0, y0, z0, &v0, &w0);
  Serial.print(v0);
  Serial.print(", ");
  Serial.println(w0);

  if (trim) {
    oled.drawPixel(v0, w0, color);
  }
}