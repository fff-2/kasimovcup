/* =====================================================================
 *  KAsimov Cup 2026 — 컬러 나노 (뒤쪽 라인 센서 전담)
 *  ---------------------------------------------------------------
 *  TCS34725는 I2C 주소가 0x29로 고정이라 한 버스에 2개를 못 단다.
 *  → 이 나노가 뒤쪽 센서를 전담하고, 감지 결과만 디지털 신호로 전달.
 *
 *  [배선]
 *   뒤 TCS34725: SDA=A4, SCL=A5 (모듈 LED는 항상 켜둘 것)
 *   D2  → 메인 나노 D12 (HIGH = 뒤 라인 감지)
 *   D13 : 상태 LED
 *   ※ 메인 나노와 반드시 GND 공통 연결
 *
 *  [사용법]
 *   로봇을 흰 바닥 위에 올려놓은 상태에서 전원 인가.
 *   → 켜지고 2초 뒤 자동으로 흰 바닥 캘리브레이션 (LED 켜짐 = 진행 중)
 *   → 이후 라인 감지 시 D2 HIGH (최소 80ms 유지 — 메인이 놓치지 않도록 래치)
 * ===================================================================== */

#include <Wire.h>

#define DEBUG 0   // 1: 시리얼 디버그 (경기 때는 0)

/* ---- 튜닝 상수 (메인 나노와 동일 기준) ---- */
#define LINE_CLEAR_FRAC 0.45f  // 라인 판정: clear < 흰바닥 × 이 비율
#define RED_RATIO_DELTA 0.10f  // 라인 판정: R비율 > 흰바닥 R비율 + 이 값
#define LATCH_MS        80     // 감지 신호 최소 유지 시간
#define LOOP_MS         5      // 읽기 주기 (적분 2.4ms보다 길게)

const uint8_t PIN_OUT = 2;     // → 메인 나노 D12
const uint8_t PIN_LED = 13;

float whiteClear = 3000, whiteRratio = 0.33f;
uint8_t lineFrameCnt = 0;
unsigned long latchUntil = 0;

/* ---- TCS34725 최소 드라이버 (메인 나노와 동일) ---- */
#define TCS_ADDR 0x29
#define TCS_CMD  0x80

static void tcsWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(TCS_CMD | reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool tcsInit() {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(TCS_CMD | 0x12);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)1);
  uint8_t id = Wire.read();
  if (id != 0x44 && id != 0x4D && id != 0x10) return false;
  tcsWrite8(0x01, 0xFF);   // ATIME: 적분 2.4ms
  tcsWrite8(0x0F, 0x02);   // 게인 16x (흰 바닥 clear<2000이면 0x03=60x)
  tcsWrite8(0x00, 0x01);   // PON
  delay(3);
  tcsWrite8(0x00, 0x03);   // PON | AEN
  return true;
}

bool tcsRead(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(TCS_CMD | 0x20 | 0x14);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8) != 8) return false;
  c = Wire.read() | (Wire.read() << 8);
  r = Wire.read() | (Wire.read() << 8);
  g = Wire.read() | (Wire.read() << 8);
  b = Wire.read() | (Wire.read() << 8);
  return true;
}

bool classifyLine(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
  uint32_t sum = (uint32_t)r + g + b;
  if (sum == 0) return false;
  float rr = (float)r / sum;
  return (c < whiteClear * LINE_CLEAR_FRAC) && (rr > whiteRratio + RED_RATIO_DELTA);
}

void setup() {
#if DEBUG
  Serial.begin(115200);
#endif
  pinMode(PIN_OUT, OUTPUT); digitalWrite(PIN_OUT, LOW);
  pinMode(PIN_LED, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif

  if (!tcsInit()) {
    // 센서 인식 실패: 빠른 점멸 반복 (배선 확인 필요). 오탐 방지 위해 출력은 LOW 고정
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  // 전원 인가 2초 후 흰 바닥 캘리브레이션 (로봇을 흰 바닥에 올려둔 상태여야 함)
  delay(2000);
  digitalWrite(PIN_LED, HIGH);
  float cSum = 0, rrSum = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 20; i++) {
    uint16_t c, r, g, b;
    if (tcsRead(c, r, g, b)) {
      uint32_t sum = (uint32_t)r + g + b;
      if (sum > 0) { cSum += c; rrSum += (float)r / sum; n++; }
    }
    delay(25);
  }
  if (n >= 10 && cSum / n > 100) {
    whiteClear  = cSum / n;
    whiteRratio = rrSum / n;
  } else {
    // 캘리브레이션 실패: 경고 점멸 후 보수적 기본값으로 진행
    for (uint8_t i = 0; i < 10; i++) { digitalWrite(PIN_LED, i & 1); delay(80); }
  }
  digitalWrite(PIN_LED, LOW);
#if DEBUG
  Serial.print(F("calib clear=")); Serial.print(whiteClear);
  Serial.print(F(" rratio=")); Serial.println(whiteRratio, 3);
#endif
}

void loop() {
  unsigned long now = millis();

  uint16_t c, r, g, b;
  if (tcsRead(c, r, g, b) && classifyLine(c, r, g, b)) {
    if (lineFrameCnt < 255) lineFrameCnt++;
  } else {
    lineFrameCnt = 0;
  }
  if (lineFrameCnt >= 2) latchUntil = now + LATCH_MS;   // 2프레임 확정 → 래치 연장

  bool out = (now < latchUntil);
  digitalWrite(PIN_OUT, out);
  digitalWrite(PIN_LED, out);

#if DEBUG
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 250) {
    lastDbg = now;
    Serial.print(F("c:")); Serial.print(c);
    Serial.print(F(" r:")); Serial.print(r);
    Serial.print(F(" g:")); Serial.print(g);
    Serial.print(F(" b:")); Serial.print(b);
    Serial.print(F(" out:")); Serial.println(out);
  }
#endif

  delay(LOOP_MS);
}
