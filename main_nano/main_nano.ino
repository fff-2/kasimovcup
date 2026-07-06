/* =====================================================================
 *  KAsimov Cup 2026 — 메인 나노 (모든 연산 담당)
 *  ---------------------------------------------------------------
 *  경기장: 지름 110cm 옥타곤, 흰 바닥 + 빨간 경계선(#660000, 두께 7cm)
 *
 *  [배선]
 *   HC-SR04 초음파 4개 (trig/echo):
 *     오른쪽 D2/D3, 앞 D4/D5, 왼쪽 D6/D7, 뒤 D8/D9
 *   L298N: ENA=D10(PWM,왼쪽모터), ENB=D11(PWM,오른쪽모터)
 *          IN1=A0, IN2=A1 (왼쪽) / IN3=A2, IN4=A3 (오른쪽)
 *   앞 TCS34725: SDA=A4, SCL=A5 (I2C 직결, 모듈 LED는 항상 켜둘 것)
 *   D12: 컬러 나노의 뒤 라인 감지 신호 입력 (HIGH = 뒤 라인)
 *   D13: 상태 LED
 *   A6 : 시작 버튼 (버튼→5V, A6→10k 저항→GND 풀다운 필수!)
 *   ※ 두 나노는 반드시 GND 공통 연결
 *
 *  [현장 튜닝 체크리스트 — 경기 전 반드시 확인]
 *   1. TURN_45_MS  : 새 배터리 기준 45도가 되게 조정. 살짝 덜 도는 쪽이 안전
 *   2. LINE_CLEAR_FRAC / RED_RATIO_DELTA : DEBUG 켜고 흰 바닥/빨간 선 값 확인
 *   3. ATTACK_DIST_CM : 상대 로봇이 30cm에서 실제로 잡히는지 확인
 *   4. 모터가 반대로 돌면 MOTOR_L_INVERT / MOTOR_R_INVERT 를 1로
 *   5. 경기 업로드 전 DEBUG 0 확인!
 * ===================================================================== */

#include <Wire.h>

#define DEBUG 0   // 1: 시리얼 디버그 출력 (경기 때는 반드시 0)

/* ------------------- 튜닝 상수 ------------------- */
#define CIRCLE_DIR        1     // 1: 시작 때 오른쪽 45도, 라인을 오른쪽에 두고 선회 / -1: 반대
#define TURN_45_MS        350   // 45도 회전 시간(ms) — 배터리 전압 따라 현장 조정
#define SPEED_SEEK        160   // 라인 찾아 직진 속도 (0~255)
#define SPEED_FOLLOW      140   // 라인 추종 기본 속도
#define FOLLOW_BIAS       25    // 라인 쪽(바깥)으로 미는 조향량
#define SPEED_PIVOT       170   // 제자리 회전 속도
#define SPEED_PUSH        255   // 밀기 속도 (최대)
#define ATTACK_DIST_CM    30    // 앞 초음파 공격 트리거 거리
#define LOST_DIST_CM      40    // 밀기 중 상대 소실 판정 거리 (히스테리시스)
#define SIDE_DIST_CM      25    // 측면 초음파 반응 거리
#define REAR_DIST_CM      25    // 뒤 초음파 방어 트리거 거리
#define PRETURN_MS        150   // 공격 전 안쪽 선회 시간 (비스듬히 충돌용)
#define PUSH_TIMEOUT_MS   2500  // 최대 밀기 시간
#define DEFENSE_MS        1200  // 후방 방어(가속+안쪽 파고들기) 시간
#define BRAKE_MS          50    // 급제동 시간
#define ESCAPE_REV_MS     250   // 라인 탈출 후진 시간 (7cm 라인 폭 기준 충분)
#define ESCAPE_PIVOT_MS   300   // 라인 탈출 후 안쪽 회전 시간
#define PIVOT_CLEAR_MS    180   // 위빙: 라인이 시야에서 사라진 뒤 추가 회전 시간
#define LINE_TURN_MAX_MS  1200  // 위빙 회전이 이 시간을 넘으면 깊이 들어간 것 → 탈출
#define BORED_MS          8000  // 이 시간 동안 상대 미접촉 시 중앙 가로지르기
#define SONAR_TIMEOUT_US  7000  // 초음파 타임아웃 (110cm 왕복 ≈ 6.4ms)
#define ARENA_MAX_CM      60    // 이 거리 초과는 경기장 밖(벽/심판)으로 무시
#define LINE_CLEAR_FRAC   0.45f // 라인 판정: clear < 흰바닥 × 이 비율
#define RED_RATIO_DELTA   0.10f // 라인 판정: R비율 > 흰바닥 R비율 + 이 값
#define WATCHDOG_IDLE_MS  1500  // 정지 워치독 (행동불능 감점 방지)
#define START_DELAY_MS    0     // 버튼 후 대기 시간 (규정상 대기 필요 시 설정)
#define MOTOR_L_INVERT    0     // 왼쪽 모터가 반대로 돌면 1
#define MOTOR_R_INVERT    0     // 오른쪽 모터가 반대로 돌면 1

/* ------------------- 핀 정의 ------------------- */
// 초음파 인덱스
enum { S_FRONT = 0, S_RIGHT, S_LEFT, S_BACK };
const uint8_t TRIG[4] = {4, 2, 6, 8};
const uint8_t ECHO[4] = {5, 3, 7, 9};

const uint8_t PIN_ENA = 10;  // 왼쪽 모터 PWM
const uint8_t PIN_ENB = 11;  // 오른쪽 모터 PWM
const uint8_t PIN_IN1 = A0;
const uint8_t PIN_IN2 = A1;
const uint8_t PIN_IN3 = A2;
const uint8_t PIN_IN4 = A3;
const uint8_t PIN_BACK_LINE = 12;  // 컬러 나노 신호
const uint8_t PIN_LED = 13;
const uint8_t PIN_BTN = A6;        // analogRead 전용

#define NO_TARGET 999

/* ------------------- 상태 정의 ------------------- */
enum State {
  ST_IDLE, ST_CALIBRATE, ST_START_TURN, ST_SEEK_LINE, ST_LINE_FOLLOW,
  ST_LINE_TURN, ST_ATTACK_PRETURN, ST_PUSH, ST_REAR_DEFENSE,
  ST_LINE_ESCAPE, ST_EDGE_SPIN, ST_CROSS_CUT
};
#if DEBUG
const char* const ST_NAME[] = {
  "IDLE", "CALIB", "START_TURN", "SEEK", "FOLLOW", "LINE_TURN",
  "PRETURN", "PUSH", "REAR_DEF", "ESCAPE", "EDGE_SPIN", "CROSS_CUT"
};
#endif

State state = ST_IDLE;
unsigned long stateStart = 0;   // 현재 상태 진입 시각
uint8_t escapePhase = 0;        // LINE_ESCAPE/CROSS_CUT 내부 단계
int8_t preturnDir = 0;          // ATTACK_PRETURN 회전 방향 (+1 우 / -1 좌)

/* ------------------- 센서 전역 ------------------- */
int dist[4] = {NO_TARGET, NO_TARGET, NO_TARGET, NO_TARGET};
uint8_t rotIdx = 0;             // 좌/우/뒤 라운드로빈
uint8_t frontHitCnt = 0;        // 앞 초음파 연속 감지 횟수
uint8_t frontLostCnt = 0;
uint8_t rearHitCnt = 0;

// 앞 TCS34725
float whiteClear = 0, whiteRratio = 0.33f;
bool calibOk = false;
uint8_t lineFrameCnt = 0;       // 라인 연속 프레임 카운트
bool frontLine = false;         // 2프레임 확정 감지
unsigned long lastLineSeen = 0;

unsigned long lastContact = 0;      // 마지막 상대 감지 시각 (교착 판정용)
unsigned long lastMoveCmd = 0;      // 마지막 비정지 모터 명령 시각 (워치독)
unsigned long edgeBehindUntil = 0;  // 뒤 라인 플래그 유지 시각

/* =====================================================================
 *  TCS34725 최소 드라이버 (라이브러리 불필요)
 * ===================================================================== */
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
  Wire.write(TCS_CMD | 0x12);           // ID 레지스터
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)1);
  uint8_t id = Wire.read();
  if (id != 0x44 && id != 0x4D && id != 0x10) return false;
  tcsWrite8(0x01, 0xFF);                // ATIME: 적분 2.4ms (최속)
  tcsWrite8(0x0F, 0x02);                // 게인 16x (흰 바닥 clear<2000이면 0x03=60x)
  tcsWrite8(0x00, 0x01);                // PON
  delay(3);
  tcsWrite8(0x00, 0x03);                // PON | AEN — 이후 자유 구동, 읽기만 하면 됨
  return true;
}

// 논블로킹: 최신 적분 결과 레지스터를 읽기만 함 (~0.5ms)
bool tcsRead(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(TCS_CMD | 0x20 | 0x14);    // CDATAL부터 자동 증가
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8) != 8) return false;
  c = Wire.read() | (Wire.read() << 8);
  r = Wire.read() | (Wire.read() << 8);
  g = Wire.read() | (Wire.read() << 8);
  b = Wire.read() | (Wire.read() << 8);
  return true;
}

// 흰 바닥 대비 빨간 라인 판별 (2조건 AND)
bool classifyLine(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
  uint32_t sum = (uint32_t)r + g + b;
  if (sum == 0) return false;
  float rr = (float)r / sum;
  return (c < whiteClear * LINE_CLEAR_FRAC) && (rr > whiteRratio + RED_RATIO_DELTA);
}

/* =====================================================================
 *  모터 (L298N)
 * ===================================================================== */
static void setOneMotor(uint8_t inA, uint8_t inB, uint8_t en, int v, bool invert) {
  if (invert) v = -v;
  v = constrain(v, -255, 255);
  if (v > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW); }
  else if (v < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else            { digitalWrite(inA, LOW);  digitalWrite(inB, LOW); }  // 코스트
  analogWrite(en, abs(v));
}

void setMotors(int left, int right) {
  setOneMotor(PIN_IN1, PIN_IN2, PIN_ENA, left,  MOTOR_L_INVERT);
  setOneMotor(PIN_IN3, PIN_IN4, PIN_ENB, right, MOTOR_R_INVERT);
  if (left != 0 || right != 0) lastMoveCmd = millis();
}

// fwd: 전진(+)/후진(-), turn: +면 우회전(왼쪽 바퀴 빠름)
void drive(int fwd, int turn) { setMotors(fwd + turn, fwd - turn); }

// 급제동 (양쪽 IN HIGH = 쇼트 브레이크)
void brake() {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, 255); analogWrite(PIN_ENB, 255);
  lastMoveCmd = millis();  // 제동도 능동 동작으로 취급
}

/* =====================================================================
 *  초음파
 * ===================================================================== */
int pingCm(uint8_t idx) {
  digitalWrite(TRIG[idx], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG[idx], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[idx], LOW);
  unsigned long us = pulseIn(ECHO[idx], HIGH, SONAR_TIMEOUT_US);
  if (us == 0) return NO_TARGET;              // 타임아웃 = 타깃 없음 (0cm 아님!)
  int cm = (int)(us / 58);
  if (cm > ARENA_MAX_CM) return NO_TARGET;    // 경기장 밖 (벽/심판) 무시
  return cm;
}

/* =====================================================================
 *  상태 전이
 * ===================================================================== */
void enterState(State s) {
#if DEBUG
  Serial.print(F("-> ")); Serial.println(ST_NAME[s]);
#endif
  state = s;
  stateStart = millis();
  escapePhase = 0;
}

/* =====================================================================
 *  setup / 캘리브레이션
 * ===================================================================== */
void calibrateWhite() {
  // 시작 위치(흰 바닥)에서 20회 샘플 → 기준값 저장
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
  if (n >= 10) {
    whiteClear  = cSum / n;
    whiteRratio = rrSum / n;
    calibOk = (whiteClear > 100);   // 너무 어두우면 센서 높이/게인 문제
  }
  if (!calibOk) {
    // 경고 점멸 후 보수적 기본값으로라도 진행 (경기 중 멈추는 것보다 낫다)
    for (uint8_t i = 0; i < 10; i++) { digitalWrite(PIN_LED, i & 1); delay(80); }
    if (whiteClear <= 100) { whiteClear = 3000; whiteRratio = 0.33f; }
  }
  digitalWrite(PIN_LED, LOW);
#if DEBUG
  Serial.print(F("calib clear=")); Serial.print(whiteClear);
  Serial.print(F(" rratio=")); Serial.println(whiteRratio, 3);
#endif
}

void setup() {
#if DEBUG
  Serial.begin(115200);
#endif
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
  }
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_BACK_LINE, INPUT);
  pinMode(PIN_LED, OUTPUT);
  setMotors(0, 0);

  Wire.begin();
  Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);   // I2C 행 방지 (센서 이상 시에도 로봇은 계속 동작)
#endif
  if (!tcsInit()) {
    // 센서 인식 실패: 빠른 점멸로 경고 (배선 확인). 라인 감지 없이도 일단 구동은 가능
    for (uint8_t i = 0; i < 20; i++) { digitalWrite(PIN_LED, i & 1); delay(50); }
  }
  enterState(ST_IDLE);
}

/* =====================================================================
 *  메인 루프
 * ===================================================================== */
void loop() {
  unsigned long now = millis();

  /* ---- 1. 센서 읽기 ---- */
  // 앞 컬러센서 (매 루프, 논블로킹)
  uint16_t c, r, g, b;
  if (tcsRead(c, r, g, b) && classifyLine(c, r, g, b)) {
    if (lineFrameCnt < 255) lineFrameCnt++;
  } else {
    lineFrameCnt = 0;
  }
  frontLine = (lineFrameCnt >= 2);          // 2프레임 연속 → 확정
  if (frontLine) lastLineSeen = now;

  bool backLine = (digitalRead(PIN_BACK_LINE) == HIGH);
  if (backLine) edgeBehindUntil = now + 500;

  // 초음파: 앞은 매 루프 + 좌/우/뒤 중 1개 라운드로빈
  if (state >= ST_START_TURN) {
    dist[S_FRONT] = pingCm(S_FRONT);
    static const uint8_t rot[3] = {S_RIGHT, S_LEFT, S_BACK};
    uint8_t k = rot[rotIdx];
    rotIdx = (rotIdx + 1) % 3;
    dist[k] = pingCm(k);

    // 연속 감지 카운터 (노이즈 필터)
    if (dist[S_FRONT] <= ATTACK_DIST_CM) { if (frontHitCnt < 10) frontHitCnt++; }
    else frontHitCnt = 0;
    if (dist[S_FRONT] == NO_TARGET || dist[S_FRONT] > LOST_DIST_CM) { if (frontLostCnt < 10) frontLostCnt++; }
    else frontLostCnt = 0;
    if (dist[S_BACK] <= REAR_DIST_CM) { if (rearHitCnt < 10) rearHitCnt++; }
    else rearHitCnt = 0;

    for (uint8_t i = 0; i < 4; i++)
      if (dist[i] != NO_TARGET) { lastContact = now; break; }
  }

  /* ---- 2. 전역 오버라이드 (상태 로직보다 우선) ---- */
  if (state >= ST_START_TURN) {
    if (frontLine && backLine && state != ST_EDGE_SPIN) {
      enterState(ST_EDGE_SPIN);                       // 앞뒤 모두 라인 = 라인 위에 걸침
    } else if (frontLine) {
      bool weaving = (state == ST_LINE_FOLLOW || state == ST_LINE_TURN ||
                      state == ST_SEEK_LINE   || state == ST_CROSS_CUT);
      if (weaving) {
        if (state != ST_LINE_TURN) enterState(ST_LINE_TURN);   // 가벼운 대응: 제동+안쪽 회전
      } else if (state != ST_LINE_ESCAPE && state != ST_EDGE_SPIN) {
        enterState(ST_LINE_ESCAPE);                   // 밀기/방어 중 = 강한 대응: 후진 포함
      }
    }
    // 정지 워치독: 1.5초 이상 정지 명령이면 강제로 움직임 재개 (행동불능 감점 방지)
    if (now - lastMoveCmd > WATCHDOG_IDLE_MS) enterState(ST_SEEK_LINE);
  }

  unsigned long t = now - stateStart;   // 현재 상태 경과 시간

  /* ---- 3. 상태별 동작 ---- */
  switch (state) {

    case ST_IDLE:
      setMotors(0, 0);
      lastMoveCmd = now;                              // IDLE은 워치독 제외
      // A6는 아날로그 전용: 버튼(5V) + 10k 풀다운 필수
      if (analogRead(PIN_BTN) > 512) {
        delay(30);
        if (analogRead(PIN_BTN) > 512) enterState(ST_CALIBRATE);
      }
      break;

    case ST_CALIBRATE:
      calibrateWhite();                               // 약 0.5초 소요
#if START_DELAY_MS > 0
      delay(START_DELAY_MS);
#endif
      lastContact = millis();
      lastMoveCmd = millis();                         // 캘리브레이션 시간이 워치독에 걸리지 않게
      enterState(ST_START_TURN);
      break;

    case ST_START_TURN:                               // 45도 회전 (타이머)
      drive(0, SPEED_PIVOT * CIRCLE_DIR);
      if (t >= TURN_45_MS) enterState(ST_SEEK_LINE);
      break;

    case ST_SEEK_LINE:                                // 라인까지 직진
      drive(SPEED_SEEK, 0);
      if (frontHitCnt >= 2) {                         // 오는 길에 상대 발견 → 그대로 정면 돌격
        preturnDir = 0;
        enterState(ST_PUSH);
      }
      break;                                          // 라인 도달은 오버라이드가 처리

    case ST_LINE_FOLLOW: {                            // 위빙: 바깥쪽으로 살짝 밀며 전진
      int bias = FOLLOW_BIAS * CIRCLE_DIR;
      if (now < edgeBehindUntil) bias = -bias;        // 방금 뒤가 라인이었다면 안쪽으로
      drive(SPEED_FOLLOW, bias);
      if (frontHitCnt >= 2) {                         // 정면 조우 → 안쪽 선제 회전 후 밀기
        preturnDir = -CIRCLE_DIR;
        enterState(ST_ATTACK_PRETURN);
      } else if (rearHitCnt >= 2) {                   // 후방 추격 → 가속+안쪽 파고들기
        enterState(ST_REAR_DEFENSE);
      } else if (now - lastContact > BORED_MS) {      // 교착 (서로 안 만남) → 중앙 가로지르기
        lastContact = now;                            // 복귀 직후 곧바로 재발동하지 않게 리셋
        enterState(ST_CROSS_CUT);
      }
      break;
    }

    case ST_LINE_TURN:                                // 라인 히트: 제동 → 안쪽 회전
      if (t < BRAKE_MS) { brake(); break; }
      drive(0, -SPEED_PIVOT * CIRCLE_DIR);
      if (t > LINE_TURN_MAX_MS) enterState(ST_LINE_ESCAPE);          // 너무 오래 = 깊이 들어감
      else if (!frontLine && now - lastLineSeen > PIVOT_CLEAR_MS)    // 라인이 안 보인 지 충분
        enterState(ST_LINE_FOLLOW);
      break;

    case ST_ATTACK_PRETURN:                           // 비스듬히 충돌하도록 살짝 회전
      drive(SPEED_FOLLOW, preturnDir * SPEED_PIVOT);
      if (t >= PRETURN_MS) enterState(ST_PUSH);
      break;

    case ST_PUSH: {                                   // 전속 밀기 + 측면 초음파 조향 보정
      int steer = 0;
      if (dist[S_LEFT]  <= SIDE_DIST_CM) steer = -60; // 상대가 왼쪽에 → 왼쪽으로
      else if (dist[S_RIGHT] <= SIDE_DIST_CM) steer = 60;
      drive(SPEED_PUSH, steer);
      // 밀착 시 초음파에서 상대가 사라질 수 있음(최소거리 한계) → 타임아웃까지는 접촉 유지로 간주
      if (t > PUSH_TIMEOUT_MS || frontLostCnt >= 4) enterState(ST_LINE_FOLLOW);
      break;                                          // 라인 도달 시 오버라이드가 ESCAPE 처리
    }

    case ST_REAR_DEFENSE:                             // 가속 + 안쪽(중앙) 파고들기
      drive(SPEED_PUSH, -60 * CIRCLE_DIR);
      {
        int outward = (CIRCLE_DIR > 0) ? dist[S_RIGHT] : dist[S_LEFT];
        if (outward <= SIDE_DIST_CM) {                // 상대가 바깥쪽으로 오버슛 → 측면 밀기
          preturnDir = CIRCLE_DIR;                    // 바깥쪽(상대 쪽)으로 살짝 회전 후 밀기
          enterState(ST_ATTACK_PRETURN);
        } else if (t >= DEFENSE_MS) {
          enterState(ST_LINE_FOLLOW);
        }
      }
      break;

    case ST_LINE_ESCAPE:                              // 강한 라인 대응: 제동→후진→안쪽 회전
      switch (escapePhase) {
        case 0:                                       // 제동
          brake();
          if (t >= BRAKE_MS) { escapePhase = 1; stateStart = now; }
          break;
        case 1:                                       // 후진 (뒤 라인 감지 시 즉시 중단!)
          drive(-SPEED_SEEK, 0);
          if (backLine || t >= ESCAPE_REV_MS) { escapePhase = 2; stateStart = now; }
          break;
        case 2:                                       // 안쪽 회전
          drive(0, -SPEED_PIVOT * CIRCLE_DIR);
          if (t >= ESCAPE_PIVOT_MS && !frontLine) enterState(ST_LINE_FOLLOW);
          else if (t >= 2UL * ESCAPE_PIVOT_MS) enterState(ST_LINE_FOLLOW);
          break;
      }
      break;

    case ST_EDGE_SPIN:                                // 앞뒤 모두 라인: 제자리 회전으로 복귀
      drive(0, -SPEED_PIVOT * CIRCLE_DIR);
      if (t >= 300 && !frontLine) enterState(ST_SEEK_LINE);
      break;

    case ST_CROSS_CUT:                                // 교착 해소: 90도 안쪽 회전 후 중앙 가로지르기
      if (escapePhase == 0) {
        drive(0, -SPEED_PIVOT * CIRCLE_DIR);
        if (t >= 2UL * TURN_45_MS) { escapePhase = 1; stateStart = now; }
      } else {
        drive(SPEED_SEEK, 0);
        if (frontHitCnt >= 2) { preturnDir = 0; enterState(ST_PUSH); }
      }
      break;                                          // 라인 도달은 오버라이드(LINE_TURN)가 처리
  }

  /* ---- 4. 디버그 ---- */
#if DEBUG
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 250) {
    lastDbg = now;
    Serial.print(ST_NAME[state]);
    Serial.print(F(" F:")); Serial.print(dist[S_FRONT]);
    Serial.print(F(" R:")); Serial.print(dist[S_RIGHT]);
    Serial.print(F(" L:")); Serial.print(dist[S_LEFT]);
    Serial.print(F(" B:")); Serial.print(dist[S_BACK]);
    Serial.print(F(" c:")); Serial.print(c);
    Serial.print(F(" line:")); Serial.print(frontLine);
    Serial.print(F("/")); Serial.println(backLine);
  }
#endif

  digitalWrite(PIN_LED, (state == ST_PUSH || frontLine));  // 상태 표시
}
