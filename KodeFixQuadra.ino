#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR_1 0x3C
#define OLED_ADDR_2 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int JUMLAH_SERVO = 6;

// S1 = kaki kiri depan
// S2 = kaki kanan depan
// S3 = kaki kiri belakang
// S4 = kaki kanan belakang
// S5 = slider maju/mundur
// S6 = pengarah lurus/belok kanan/belok kiri
int servoPin[JUMLAH_SERVO] = {32, 33, 25, 26, 27, 14};

String namaServo[JUMLAH_SERVO] = {
  "S1 Kiri Depan",
  "S2 Kanan Depan",
  "S3 Kiri Belakang",
  "S4 Kanan Belakang",
  "S5 Slider Maju Mundur",
  "S6 Pengarah Belok"
};

int POS_BERDIRI[JUMLAH_SERVO] = {1800, 1200, 1800, 1200, 1500, 1500};
int POS_JONGKOK[JUMLAH_SERVO] = {1200, 1800, 1200, 1800, 1500, 1500};

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const bool AUTO_START = false;

// ==============================
// MPU6050
// ==============================
const byte MPU_ADDR_1 = 0x68;
const byte MPU_ADDR_2 = 0x69;
byte mpuAddr = MPU_ADDR_1;

bool imuOK = false;
bool oledOK = false;
byte oledAddr = OLED_ADDR_1;

const unsigned long IMU_UPDATE_MS = 80;
const unsigned long OLED_UPDATE_MS = 220;
const unsigned long TELEMETRY_MS = 160;

unsigned long lastImuUpdate = 0;
unsigned long lastOledUpdate = 0;
unsigned long lastTelemetry = 0;

int16_t accXraw = 0;
int16_t accYraw = 0;
int16_t accZraw = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;

float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float rollDeg = 0.0;
float pitchDeg = 0.0;

const float BATAS_MIRING_DERAJAT = 18.0;

// ==============================
// POSISI KAKI
// ==============================
const int S1_TURUN = 1800;
const int S2_TURUN = 1200;
const int S3_TURUN = 1800;
const int S4_TURUN = 1200;

int amplitudoAngkatDepan = 650;
int amplitudoAngkatBelakang = 900;

// ==============================
// SLIDER DAN PENGARAH
// ==============================
const int S5_TENGAH = 1500;

// GUI: S5A, S5B, S6MAJU, S6MUNDUR, S6KANAN, S6KIRI
int s5MajuA = 1200;
int s5MajuB = 1800;

const int S6_CENTER_US = 1500;

int s6TargetLurusMajuUs = 1450;    
int s6TargetLurusMundurUs = 1500;  
int s6TargetKananUs = 1330;        
int s6TargetKiriUs = 1100;         
const bool BELOK_REFERENSI_JAMES = true;

// ==============================
// KECEPATAN
// ==============================
const unsigned long AWAL_BERDIRI_MS = 900;
const unsigned long UPDATE_SERVO_MS = 7;

const int LEG_STEP_US = 55;
const int SLIDER_STEP_US = 75;
const int ARAH_STEP_US = 70;

// GUI: SPEED
int speedBaseMs = 220;

int posNow[JUMLAH_SERVO];
int posTarget[JUMLAH_SERVO];

bool autoStarted = false;
unsigned long waktuStart = 0;
unsigned long lastServoUpdate = 0;
unsigned long waktuFase = 0;
int fase = 0;
String serialBuffer = "";

// ==============================
// MODE ROBOT
// ==============================
enum RobotMode {
  MODE_STAND,
  MODE_MAJU,
  MODE_MUNDUR,
  MODE_KIRI,
  MODE_KANAN,
  MODE_STOP,
  MODE_JONGKOK
};

RobotMode modeRobot = MODE_STAND;

// ==============================
// SETUP DAN LOOP
// ==============================
void setup() {
  Serial.begin(115200);
  delay(250);

  Wire.begin(21, 22);
  Wire.setTimeOut(50);

  initOLED();
  tampilBootOLED("BOOT", "Init servo...");

  initServoPWM();
  setServoLangsung(POS_BERDIRI);

  tampilBootOLED("BOOT", "Init MPU6050...");
  imuOK = initMPU6050();
  if (imuOK) bacaMPU6050();

  modeRobot = MODE_STAND;
  setSemuaBerdiri();
  waktuStart = millis();
  waktuFase = millis();

  Serial.println();
  Serial.println("READY=QUADRA_PAW_REV10_LIFT_BALANCE");
  Serial.println("BAUD=115200");
  Serial.println("COMMAND=MAJU,MUNDUR,KIRI,KANAN,STOP,STAND,JONGKOK,PRINT,MPU,PING,PARAM?,I2C");
  Serial.println("TUNING=SPEED=220,LIFTF=650,LIFTB=900,S5A=1200,S5B=1800,S6MAJU=1450,S6MUNDUR=1500,S6KANAN=1330,S6KIRI=1100");
  Serial.println("S6_NOTE=REV10: lift depan/belakang dipisah. LIFTF untuk S1/S2, LIFTB untuk S3/S4.");
  Serial.println("NAME=QUADRA-PAW");
  Serial.print("OLED=");
  Serial.println(oledOK ? "OK" : "ERROR");
  if (oledOK) {
    Serial.print("OLED_ADDR=0x");
    Serial.println(oledAddr, HEX);
  }
  Serial.print("MPU=");
  Serial.println(imuOK ? "OK" : "ERROR");
  if (imuOK) {
    Serial.print("MPU_ADDR=0x");
    Serial.println(mpuAddr, HEX);
  }

  printParameter();
  tampilOLED("BERDIRI AWAL");
  kirimTelemetry();
}

void loop() {
  unsigned long now = millis();

  bacaCommandSerial();

  if (AUTO_START && !autoStarted && now - waktuStart >= AWAL_BERDIRI_MS) {
    autoStarted = true;
    mulaiGerak(MODE_MAJU);
  }

  if (modeSedangJalan()) {
    if (now - waktuFase >= durasiFase(fase)) {
      fase++;
      if (fase > 11) fase = 0;
      waktuFase = now;
      setFaseJalan(fase);
    }
  }

  if (now - lastServoUpdate >= UPDATE_SERVO_MS) {
    lastServoUpdate = now;
    paksaS6KoreksiLurusAktif();
    updateSemuaServo();
  }

  if (now - lastImuUpdate >= IMU_UPDATE_MS) {
    lastImuUpdate = now;
    if (imuOK) {
      imuOK = bacaMPU6050();
    } else {
      imuOK = initMPU6050();
      if (imuOK) bacaMPU6050();
    }
  }

  if (oledOK && now - lastOledUpdate >= OLED_UPDATE_MS) {
    lastOledUpdate = now;
    tampilOLED(namaFaseDisplay());
  }

  if (now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    kirimTelemetry();
  }
}

// ==============================
// INIT SERVO
// ==============================
void initServoPWM() {
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    ledcAttach(servoPin[i], 50, 16);
    posNow[i] = POS_BERDIRI[i];
    posTarget[i] = POS_BERDIRI[i];
    writeServo(i, posNow[i]);
  }
}

void setServoLangsung(int dataPosisi[]) {
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    posNow[i] = dataPosisi[i];
    posTarget[i] = dataPosisi[i];
    writeServo(i, posNow[i]);
  }
}

// ==============================
// SERIAL GUI
// ==============================
void bacaCommandSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        prosesCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      if (serialBuffer.length() < 120) serialBuffer += c;
    }
  }
}

void prosesCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String rawCmd = cmd;
  cmd.toUpperCase();
  cmd.replace(" ", "");

  Serial.print("RX=");
  Serial.println(rawCmd);

  if (cmd == "PING") {
    Serial.println("ACK=PING");
  }
  else if (cmd == "PARAM?" || cmd == "PARAM" || cmd == "READ") {
    printParameter();
    Serial.println("ACK=PARAM");
  }
  else if (cmd == "START" || cmd == "GO" || cmd == "MAJU" || cmd == "FORWARD") {
    mulaiGerak(MODE_MAJU);
    Serial.println("ACK=MAJU");
  }
  else if (cmd == "MUNDUR" || cmd == "BACK" || cmd == "BACKWARD") {
    mulaiGerak(MODE_MUNDUR);
    Serial.println("ACK=MUNDUR");
  }
  else if (cmd == "KIRI" || cmd == "LEFT" || cmd == "BELOKKIRI" || cmd == "TURNLEFT") {
    mulaiGerak(MODE_KIRI);
    Serial.println("ACK=KIRI");
  }
  else if (cmd == "KANAN" || cmd == "RIGHT" || cmd == "BELOKKANAN" || cmd == "TURNRIGHT") {
    mulaiGerak(MODE_KANAN);
    Serial.println("ACK=KANAN");
  }
  else if (cmd == "LURUS") {
    mulaiGerak(MODE_MAJU);
    Serial.println("ACK=LURUS_MAJU");
  }
  else if (cmd == "STOP") {
    stopRobot();
    Serial.println("ACK=STOP");
  }
  else if (cmd == "STAND" || cmd == "BERDIRI" || cmd == "HOME") {
    modeRobot = MODE_STAND;
    setSemuaBerdiri();
    Serial.println("ACK=STAND");
  }
  else if (cmd == "JONGKOK" || cmd == "SQUAT") {
    modeRobot = MODE_JONGKOK;
    setJongkok();
    Serial.println("ACK=JONGKOK");
  }
  else if (cmd == "PRINT") {
    printPosisi("POSISI SEKARANG");
    printParameter();
    Serial.println("ACK=PRINT");
  }
  else if (cmd == "MPU") {
    printMPU();
    Serial.println("ACK=MPU");
  }
  else if (cmd == "I2C") {
    scanI2C();
    Serial.println("ACK=I2C");
  }
  else if (cmd.startsWith("SPEED=") || cmd.startsWith("DURASI=")) {
    speedBaseMs = constrain(ambilNilaiSetelahSamaDengan(cmd), 80, 350);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=SPEED,VALUE=");
    Serial.println(speedBaseMs);
  }
  else if (cmd.startsWith("LIFT=") || cmd.startsWith("TINGGI=")) {
    int v = constrain(ambilNilaiSetelahSamaDengan(cmd), 400, 1000);
    amplitudoAngkatDepan = v;
    amplitudoAngkatBelakang = v;
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=LIFT,VALUE=");
    Serial.println(v);
    Serial.print("ACK=LIFTF,VALUE=");
    Serial.println(amplitudoAngkatDepan);
    Serial.print("ACK=LIFTB,VALUE=");
    Serial.println(amplitudoAngkatBelakang);
  }
  else if (cmd.startsWith("LIFTF=") || cmd.startsWith("LIFTDEPAN=") || cmd.startsWith("FRONTLIFT=") || cmd.startsWith("ANGKATDEPAN=")) {
    amplitudoAngkatDepan = constrain(ambilNilaiSetelahSamaDengan(cmd), 400, 1000);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=LIFTF,VALUE=");
    Serial.println(amplitudoAngkatDepan);
  }
  else if (cmd.startsWith("LIFTB=") || cmd.startsWith("LIFTBELAKANG=") || cmd.startsWith("BACKLIFT=") || cmd.startsWith("ANGKATBELAKANG=")) {
    amplitudoAngkatBelakang = constrain(ambilNilaiSetelahSamaDengan(cmd), 400, 1050);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=LIFTB,VALUE=");
    Serial.println(amplitudoAngkatBelakang);
  }
  else if (cmd.startsWith("S5A=") || cmd.startsWith("S5_A=")) {
    s5MajuA = constrain(ambilNilaiSetelahSamaDengan(cmd), 1000, 1500);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=S5A,VALUE=");
    Serial.println(s5MajuA);
  }
  else if (cmd.startsWith("S5B=") || cmd.startsWith("S5_B=")) {
    s5MajuB = constrain(ambilNilaiSetelahSamaDengan(cmd), 1500, 2000);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=S5B,VALUE=");
    Serial.println(s5MajuB);
  }
  else if (cmd.startsWith("S6MAJU=") || cmd.startsWith("S6TRIM=") || cmd.startsWith("TRIMMAJU=") || cmd.startsWith("S6LURUSMAJU=")) {
    int v = ambilNilaiSetelahSamaDengan(cmd);
    s6TargetLurusMajuUs = normalisasiS6Lurus(v);
    refreshTargetFaseJikaJalan();
    paksaS6KoreksiLurusAktif();
    Serial.print("ACK=S6MAJU,VALUE=");
    Serial.println(s6TargetLurusMajuUs);
    Serial.print("ACK=S6MAJUTARGET,VALUE=");
    Serial.println(s6TargetLurusMajuUs);
  }
  else if (cmd.startsWith("S6MUNDUR=") || cmd.startsWith("TRIMMUNDUR=") || cmd.startsWith("S6LURUSMUNDUR=")) {
    int v = ambilNilaiSetelahSamaDengan(cmd);
    s6TargetLurusMundurUs = normalisasiS6Lurus(v);
    refreshTargetFaseJikaJalan();
    paksaS6KoreksiLurusAktif();
    Serial.print("ACK=S6MUNDUR,VALUE=");
    Serial.println(s6TargetLurusMundurUs);
    Serial.print("ACK=S6MUNDURTARGET,VALUE=");
    Serial.println(s6TargetLurusMundurUs);
  }
  else if (cmd.startsWith("S6KANAN=") || cmd.startsWith("S6_RIGHT=") || cmd.startsWith("S6KANANTARGET=")) {
    int v = ambilNilaiSetelahSamaDengan(cmd);
    // REV7: nilai GUI adalah target langsung. Contoh 1330.
    // Kalau terkirim nilai kecil seperti 170, tetap didukung sebagai offset gaya lama.
    if (abs(v) <= 500) s6TargetKananUs = constrain(S6_CENTER_US - abs(v), 900, 2100);
    else s6TargetKananUs = constrain(v, 900, 2100);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=S6KANAN,VALUE=");
    Serial.println(s6TargetKananUs);
  }
  else if (cmd.startsWith("S6KIRI=") || cmd.startsWith("S6_LEFT=") || cmd.startsWith("S6KIRITARGET=")) {
    int v = ambilNilaiSetelahSamaDengan(cmd);
    // REV7: nilai GUI adalah target langsung. Contoh 1100 atau 1200.
    // Kalau terkirim nilai kecil seperti 300, dipakai sebagai offset ke sisi kiri dari 1500.
    if (abs(v) <= 500) s6TargetKiriUs = constrain(S6_CENTER_US - abs(v), 900, 2100);
    else s6TargetKiriUs = constrain(v, 900, 2100);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=S6KIRI,VALUE=");
    Serial.println(s6TargetKiriUs);
  }
  else if (cmd.startsWith("S6BELOK=") || cmd.startsWith("S6ANGLE=") || cmd.startsWith("S6K=") || cmd.startsWith("S6KOREKSI=")) {
    int v = constrain(abs(ambilNilaiSetelahSamaDengan(cmd)), 0, 500);
    // Kompatibilitas: satu nilai membuat kanan gaya V5 dan kiri lebih tajam gaya V6.
    s6TargetKananUs = constrain(S6_CENTER_US - v, 900, 2100);
    s6TargetKiriUs = constrain(S6_CENTER_US - max(v + 120, v), 900, 2100);
    refreshTargetFaseJikaJalan();
    Serial.print("ACK=S6KANAN,VALUE=");
    Serial.println(s6TargetKananUs);
    Serial.print("ACK=S6KIRI,VALUE=");
    Serial.println(s6TargetKiriUs);
  }
  else if (cmd == "TESTS6LURUS" || cmd == "S6LURUS") {
    posTarget[5] = s6Center();
    constrainSemuaTarget();
    Serial.print("ACK=S6LURUS,VALUE=");
    Serial.println(posTarget[5]);
  }
  else if (cmd == "TESTS6KIRI" || cmd == "S6KIRI") {
    posTarget[5] = s6Kiri();
    constrainSemuaTarget();
    Serial.print("ACK=TESTS6KIRI,VALUE=");
    Serial.println(posTarget[5]);
  }
  else if (cmd == "TESTS6KANAN" || cmd == "S6KANAN") {
    posTarget[5] = s6Kanan();
    constrainSemuaTarget();
    Serial.print("ACK=TESTS6KANAN,VALUE=");
    Serial.println(posTarget[5]);
  }
  else {
    Serial.print("ERR=UNKNOWN_COMMAND,CMD=");
    Serial.println(rawCmd);
  }

  kirimTelemetry();
}

int ambilNilaiSetelahSamaDengan(String cmd) {
  int idx = cmd.indexOf('=');
  if (idx < 0) return 0;
  String nilai = cmd.substring(idx + 1);
  nilai.trim();
  return nilai.toInt();
}

void refreshTargetFaseJikaJalan() {
  if (modeSedangJalan()) {
    setFaseJalan(fase);
  } else if (modeRobot == MODE_STAND || modeRobot == MODE_STOP) {
    setSemuaBerdiri();
  } else if (modeRobot == MODE_JONGKOK) {
    setJongkok();
  }
}

// ==============================
// MPU6050
// ==============================
bool initMPU6050() {
  if (cekI2C(MPU_ADDR_1)) {
    mpuAddr = MPU_ADDR_1;
  } else if (cekI2C(MPU_ADDR_2)) {
    mpuAddr = MPU_ADDR_2;
  } else {
    return false;
  }

  tulisMPU(0x6B, 0x00);
  delay(20);
  tulisMPU(0x1B, 0x00);
  tulisMPU(0x1C, 0x00);
  tulisMPU(0x1A, 0x03);
  return true;
}

bool cekI2C(byte alamat) {
  Wire.beginTransmission(alamat);
  return (Wire.endTransmission() == 0);
}

void tulisMPU(byte reg, byte data) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool bacaMPU6050() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  int jumlahByte = Wire.requestFrom((int)mpuAddr, 14, true);
  if (jumlahByte < 14) return false;

  accXraw = baca16bitWire();
  accYraw = baca16bitWire();
  accZraw = baca16bitWire();
  baca16bitWire();
  gyroXraw = baca16bitWire();
  gyroYraw = baca16bitWire();
  gyroZraw = baca16bitWire();

  accX = accXraw / 16384.0;
  accY = accYraw / 16384.0;
  accZ = accZraw / 16384.0;

  gyroX = gyroXraw / 131.0;
  gyroY = gyroYraw / 131.0;
  gyroZ = gyroZraw / 131.0;

  rollDeg = atan2(accY, accZ) * 180.0 / PI;
  pitchDeg = atan2(-accX, sqrt((accY * accY) + (accZ * accZ))) * 180.0 / PI;
  return true;
}

int16_t baca16bitWire() {
  int16_t highByte = Wire.read();
  int16_t lowByte = Wire.read();
  return (highByte << 8) | lowByte;
}

const char* statusIMU() {
  if (!imuOK) return "ERROR";
  if (fabs(rollDeg) > BATAS_MIRING_DERAJAT || fabs(pitchDeg) > BATAS_MIRING_DERAJAT) return "MIRING";
  return "OK";
}

void printMPU() {
  Serial.print("MPU=");
  Serial.print(statusIMU());
  Serial.print(",ROLL=");
  Serial.print(rollDeg, 2);
  Serial.print(",PITCH=");
  Serial.print(pitchDeg, 2);
  Serial.print(",AZ=");
  Serial.println(accZ, 2);
}

// ==============================
// GERAK ROBOT
// ==============================
int s1Angkat() { return S1_TURUN - amplitudoAngkatDepan; }
int s2Angkat() { return S2_TURUN + amplitudoAngkatDepan; }
int s3Angkat() { return S3_TURUN - amplitudoAngkatBelakang; }
int s4Angkat() { return S4_TURUN + amplitudoAngkatBelakang; }
int s6Center() { return S6_CENTER_US; }
int normalisasiS6Lurus(int v) {

  if (v == 0) return S6_CENTER_US;
  if (v >= 900 && v <= 2100) return constrain(v, 900, 2100);
  if (v >= -500 && v <= 500) return constrain(S6_CENTER_US + v, 900, 2100);
  return constrain(v, 900, 2100);
}

int s6LurusMaju() { return s6TargetLurusMajuUs; }
int s6LurusMundur() { return s6TargetLurusMundurUs; }

int s6LurusUntukMode() {
  if (modeRobot == MODE_MAJU) return s6LurusMaju();
  if (modeRobot == MODE_MUNDUR) return s6LurusMundur();
  return s6Center();
}

int s6Kanan() { return s6TargetKananUs; }
int s6Kiri() { return s6TargetKiriUs; }

void constrainSemuaTarget() {
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    posTarget[i] = constrain(posTarget[i], SERVO_MIN_US, SERVO_MAX_US);
  }
}

bool modeSedangJalan() {
  return modeRobot == MODE_MAJU || modeRobot == MODE_MUNDUR || modeRobot == MODE_KIRI || modeRobot == MODE_KANAN;
}

int targetS6UntukMode() {
  if (modeRobot == MODE_KIRI) return s6Kiri();
  if (modeRobot == MODE_KANAN) return s6Kanan();
  return s6LurusUntukMode();
}

void paksaS6KoreksiLurusAktif() {
  if (modeRobot == MODE_MAJU) {
    posTarget[5] = constrain(s6LurusMaju(), SERVO_MIN_US, SERVO_MAX_US);
  } else if (modeRobot == MODE_MUNDUR) {
    posTarget[5] = constrain(s6LurusMundur(), SERVO_MIN_US, SERVO_MAX_US);
  }
}

int targetS5Pertama() {
  if (modeRobot == MODE_MUNDUR) return s5MajuB;
  return s5MajuA;
}

int targetS5Kedua() {
  if (modeRobot == MODE_MUNDUR) return s5MajuA;
  return s5MajuB;
}

bool modeBelok() {
  return modeRobot == MODE_KIRI || modeRobot == MODE_KANAN;
}

bool pasanganPertamaAdalahP2() {
  if (modeRobot == MODE_KANAN && BELOK_REFERENSI_JAMES) return true;
  return false;
}

void setPasanganPertamaAngkat() {
  if (pasanganPertamaAdalahP2()) setPasangan2Angkat();
  else setPasangan1Angkat();
}

void setPasanganKeduaAngkat() {
  if (pasanganPertamaAdalahP2()) setPasangan1Angkat();
  else setPasangan2Angkat();
}

void mulaiGerak(RobotMode modeBaru) {
  modeRobot = modeBaru;
  fase = 0;
  waktuFase = millis();
  setFaseJalan(fase);
  paksaS6KoreksiLurusAktif();
}

void stopRobot() {
  modeRobot = MODE_STOP;
  setSemuaBerdiri();
}

void setKakiTurunDasar() {
  posTarget[0] = S1_TURUN;
  posTarget[1] = S2_TURUN;
  posTarget[2] = S3_TURUN;
  posTarget[3] = S4_TURUN;
}

void setSemuaBerdiri() {
  setKakiTurunDasar();
  posTarget[4] = S5_TENGAH;
  posTarget[5] = s6Center();
  constrainSemuaTarget();
}

void setJongkok() {
  for (int i = 0; i < JUMLAH_SERVO; i++) posTarget[i] = POS_JONGKOK[i];
  posTarget[4] = S5_TENGAH;
  posTarget[5] = s6Center();
  constrainSemuaTarget();
}

void setDasarFaseJalan() {
  setKakiTurunDasar();
  posTarget[4] = S5_TENGAH;
  posTarget[5] = targetS6UntukMode();
}

void setPasangan1Angkat() {
  // Pasangan 1: S2 kanan depan + S3 kiri belakang
  posTarget[1] = s2Angkat();
  posTarget[2] = s3Angkat();
}

void setPasangan2Angkat() {
  // Pasangan 2: S1 kiri depan + S4 kanan belakang
  posTarget[0] = s1Angkat();
  posTarget[3] = s4Angkat();
}

void setFaseJalan(int f) {
  setDasarFaseJalan();

  int s5Pertama = targetS5Pertama();
  int s5Kedua = targetS5Kedua();
  int s6L = s6Center();
  int s6Turn = targetS6UntukMode();

  if (modeBelok() && BELOK_REFERENSI_JAMES) {
  
    posTarget[4] = S5_TENGAH;

    if (f == 0) {
      setPasanganPertamaAngkat();
      posTarget[5] = s6L;
    } else if (f == 1) {
      setPasanganPertamaAngkat();
      posTarget[5] = s6L;
    } else if (f == 2) {
      setPasanganPertamaAngkat();
      posTarget[5] = s6Turn;
    } else if (f == 3) {
      posTarget[5] = s6Turn;
    } else if (f == 4) {
      posTarget[5] = s6Turn;
    } else if (f == 5) {
      posTarget[5] = s6Turn;
    } else if (f == 6) {
      setPasanganKeduaAngkat();
      posTarget[5] = s6Turn;
    } else if (f == 7) {
      setPasanganKeduaAngkat();
      posTarget[5] = s6Turn;
    } else if (f == 8) {
      setPasanganKeduaAngkat();
      posTarget[5] = s6L;
    } else if (f == 9) {
      posTarget[5] = s6L;
    } else if (f == 10) {
      posTarget[5] = s6L;
    } else if (f == 11) {
      posTarget[5] = s6L;
    }
  } else {
    
    posTarget[5] = targetS6UntukMode();

    if (f == 0) {
      setPasanganPertamaAngkat();
      posTarget[4] = S5_TENGAH;
    } else if (f == 1) {
      setPasanganPertamaAngkat();
      posTarget[4] = S5_TENGAH;
    } else if (f == 2) {
      setPasanganPertamaAngkat();
      posTarget[4] = s5Pertama;
    } else if (f == 3) {
      posTarget[4] = s5Pertama;
    } else if (f == 4) {
      posTarget[4] = s5Pertama;
    } else if (f == 5) {
      posTarget[4] = s5Pertama;
    } else if (f == 6) {
      setPasanganKeduaAngkat();
      posTarget[4] = s5Pertama;
    } else if (f == 7) {
      setPasanganKeduaAngkat();
      posTarget[4] = s5Pertama;
    } else if (f == 8) {
      setPasanganKeduaAngkat();
      posTarget[4] = s5Kedua;
    } else if (f == 9) {
      posTarget[4] = s5Kedua;
    } else if (f == 10) {
      posTarget[4] = s5Kedua;
    } else if (f == 11) {
      posTarget[4] = s5Kedua;
    }
  }

  constrainSemuaTarget();
  printTargetFase();
}

unsigned long durasiFase(int f) {
  int durasiAngkat = speedBaseMs;
  int durasiTahan = max(40, (speedBaseMs * 36) / 100);
  int durasiSlider = speedBaseMs + 30;
  int durasiTurun = max(120, speedBaseMs - 10);
  int durasiTepak = max(50, (speedBaseMs * 39) / 100);
  int durasiStabil = max(30, (speedBaseMs * 23) / 100);

  if (f == 0 || f == 6) return durasiAngkat;
  if (f == 1 || f == 7) return durasiTahan;
  if (f == 2 || f == 8) return durasiSlider;
  if (f == 3 || f == 9) return durasiTurun;
  if (f == 4 || f == 10) return durasiTepak;
  return durasiStabil;
}

void updateSemuaServo() {
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    int stepGerak = LEG_STEP_US;
    if (i == 4) stepGerak = SLIDER_STEP_US;
    if (i == 5) stepGerak = ARAH_STEP_US;
    posNow[i] = majuKeTarget(posNow[i], posTarget[i], stepGerak);
    writeServo(i, posNow[i]);
  }
}

int majuKeTarget(int sekarang, int target, int stepGerak) {
  if (sekarang < target) {
    sekarang += stepGerak;
    if (sekarang > target) sekarang = target;
  } else if (sekarang > target) {
    sekarang -= stepGerak;
    if (sekarang < target) sekarang = target;
  }
  return sekarang;
}

void writeServo(int index, int pulseUS) {
  pulseUS = constrain(pulseUS, SERVO_MIN_US, SERVO_MAX_US);
  int duty = (pulseUS * 65535L) / 20000L;
  ledcWrite(servoPin[index], duty);
}

// ==============================
// OLED
// ==============================
void initOLED() {
  oledOK = false;

  if (cekI2C(OLED_ADDR_1)) {
    oledAddr = OLED_ADDR_1;
    oledOK = display.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  } else if (cekI2C(OLED_ADDR_2)) {
    oledAddr = OLED_ADDR_2;
    oledOK = display.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  }

  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.print("ADDR 0x");
    display.println(oledAddr, HEX);
    display.display();
  }
}

void tampilBootOLED(const char* line1, const char* line2) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("QUADRA-PAW");
  display.println(line1);
  display.println(line2);
  display.display();
}

void tampilOLED(const char* status) {
  if (!oledOK) return;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("QUADRA-PAW");
  display.print("Mode: ");
  display.println(namaMode());
  display.print("Fase: ");
  display.println(status);
  display.print("S5: ");
  display.print(posNow[4]);
  display.print("/");
  display.println(posTarget[4]);
  display.print("S6: ");
  display.print(posNow[5]);
  display.print("/");
  display.println(posTarget[5]);
  display.print("MPU: ");
  display.println(statusIMU());
  display.print("R:");
  display.print(rollDeg, 1);
  display.print(" P:");
  display.println(pitchDeg, 1);
  display.print("Az:");
  display.print(accZ, 2);
  display.display();
}

// ==============================
// DISPLAY, TELEMETRY, DEBUG
// ==============================
const char* namaMode() {
  if (modeRobot == MODE_MAJU) return "MAJU";
  if (modeRobot == MODE_MUNDUR) return "MUNDUR";
  if (modeRobot == MODE_KIRI) return "KIRI";
  if (modeRobot == MODE_KANAN) return "KANAN";
  if (modeRobot == MODE_STOP) return "STOP";
  if (modeRobot == MODE_JONGKOK) return "JONGKOK";
  return "STAND";
}

const char* namaFaseDisplay() {
  if (!modeSedangJalan()) return namaMode();

  bool firstP2 = pasanganPertamaAdalahP2();
  if (fase == 0) return firstP2 ? "P2 ANGKAT" : "P1 ANGKAT";
  if (fase == 1) return firstP2 ? "P2 TAHAN" : "P1 TAHAN";
  if (fase == 2) return modeBelok() ? "S6 BELOK 1" : "S5 GERAK 1";
  if (fase == 3) return firstP2 ? "P2 TURUN" : "P1 TURUN";
  if (fase == 4) return firstP2 ? "P2 TEPAK" : "P1 TEPAK";
  if (fase == 5) return "STABIL 1";
  if (fase == 6) return firstP2 ? "P1 ANGKAT" : "P2 ANGKAT";
  if (fase == 7) return firstP2 ? "P1 TAHAN" : "P2 TAHAN";
  if (fase == 8) return modeBelok() ? "S6 LURUS" : "S5 GERAK 2";
  if (fase == 9) return firstP2 ? "P1 TURUN" : "P2 TURUN";
  if (fase == 10) return firstP2 ? "P1 TEPAK" : "P2 TEPAK";
  if (fase == 11) return "STABIL 2";
  return "JALAN";
}

const char* namaFaseSerial() {
  if (!modeSedangJalan()) return namaMode();

  bool firstP2 = pasanganPertamaAdalahP2();
  if (fase == 0) return firstP2 ? "P2_ANGKAT" : "P1_ANGKAT";
  if (fase == 1) return firstP2 ? "P2_TAHAN" : "P1_TAHAN";
  if (fase == 2) return modeBelok() ? "S6_BELOK_1" : "S5_GERAK_1";
  if (fase == 3) return firstP2 ? "P2_TURUN" : "P1_TURUN";
  if (fase == 4) return firstP2 ? "P2_TEPAK" : "P1_TEPAK";
  if (fase == 5) return "STABIL_1";
  if (fase == 6) return firstP2 ? "P1_ANGKAT" : "P2_ANGKAT";
  if (fase == 7) return firstP2 ? "P1_TAHAN" : "P2_TAHAN";
  if (fase == 8) return modeBelok() ? "S6_LURUS" : "S5_GERAK_2";
  if (fase == 9) return firstP2 ? "P1_TURUN" : "P2_TURUN";
  if (fase == 10) return firstP2 ? "P1_TEPAK" : "P2_TEPAK";
  if (fase == 11) return "STABIL_2";
  return "JALAN";
}

void kirimTelemetry() {
  Serial.print("MODE=");
  Serial.print(namaMode());
  Serial.print(",PHASE=");
  Serial.print(namaFaseSerial());
  Serial.print(",MPU=");
  Serial.print(statusIMU());
  Serial.print(",ROLL=");
  Serial.print(rollDeg, 2);
  Serial.print(",PITCH=");
  Serial.print(pitchDeg, 2);
  Serial.print(",AZ=");
  Serial.print(accZ, 2);
  Serial.print(",S5=");
  Serial.print(posNow[4]);
  Serial.print("/");
  Serial.print(posTarget[4]);
  Serial.print(",S6=");
  Serial.print(posNow[5]);
  Serial.print("/");
  Serial.print(posTarget[5]);
  Serial.print(",LIFTF=");
  Serial.print(amplitudoAngkatDepan);
  Serial.print(",LIFTB=");
  Serial.print(amplitudoAngkatBelakang);
  Serial.print(",LIFT=");
  Serial.print((amplitudoAngkatDepan + amplitudoAngkatBelakang) / 2);
  Serial.print(",SPEED=");
  Serial.print(speedBaseMs);
  Serial.print(",S5A=");
  Serial.print(s5MajuA);
  Serial.print(",S5B=");
  Serial.print(s5MajuB);
  Serial.print(",S6MAJU=");
  Serial.print(s6TargetLurusMajuUs);
  Serial.print(",S6MUNDUR=");
  Serial.print(s6TargetLurusMundurUs);
  Serial.print(",S6MAJUTARGET=");
  Serial.print(s6LurusMaju());
  Serial.print(",S6MUNDURTARGET=");
  Serial.print(s6LurusMundur());
  Serial.print(",S6TRIM=");
  Serial.print(s6TargetLurusMajuUs);
  Serial.print(",S6KANAN=");
  Serial.print(s6TargetKananUs);
  Serial.print(",S6KIRI=");
  Serial.print(s6TargetKiriUs);
  Serial.print(",S6K=");
  Serial.println(s6TargetKananUs);
}

void printTargetFase() {
  Serial.print("PHASE_CHANGE=");
  Serial.print(namaFaseSerial());
  Serial.print(",MODE=");
  Serial.print(namaMode());
  Serial.print(",S5T=");
  Serial.print(posTarget[4]);
  Serial.print(",S6T=");
  Serial.println(posTarget[5]);
}

void printParameter() {
  Serial.println("=== PARAMETER ===");
  Serial.print("SPEED=");
  Serial.println(speedBaseMs);
  Serial.print("LIFTF=");
  Serial.println(amplitudoAngkatDepan);
  Serial.print("LIFTB=");
  Serial.println(amplitudoAngkatBelakang);
  Serial.print("LIFT=");
  Serial.println((amplitudoAngkatDepan + amplitudoAngkatBelakang) / 2);
  Serial.print("S5A=");
  Serial.println(s5MajuA);
  Serial.print("S5B=");
  Serial.println(s5MajuB);
  Serial.print("S6MAJU=");
  Serial.println(s6TargetLurusMajuUs);
  Serial.print("S6MUNDUR=");
  Serial.println(s6TargetLurusMundurUs);
  Serial.print("S6TRIM=");
  Serial.println(s6TargetLurusMajuUs);
  Serial.print("S6KANAN=");
  Serial.println(s6TargetKananUs);
  Serial.print("S6KIRI=");
  Serial.println(s6TargetKiriUs);
  Serial.print("S6K=");
  Serial.println(s6TargetKananUs);
  Serial.println("REV10_FIX=LIFT_DEPAN_BELAKANG_DIPISAH_DAN_S6_DIRECT_TETAP_AKTIF");
  Serial.print("TURN_REF=");
  Serial.println(BELOK_REFERENSI_JAMES ? 1 : 0);
  Serial.print("S6_CENTER=");
  Serial.println(s6Center());
  Serial.print("S6_MAJU_TARGET=");
  Serial.println(s6LurusMaju());
  Serial.print("S6_MUNDUR_TARGET=");
  Serial.println(s6LurusMundur());
  Serial.print("S6_KANAN_TARGET=");
  Serial.println(s6Kanan());
  Serial.print("S6_KIRI_TARGET=");
  Serial.println(s6Kiri());
}

void printPosisi(String judul) {
  Serial.println("=== " + judul + " ===");
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(" ");
    Serial.print(namaServo[i]);
    Serial.print(" = ");
    Serial.print(posNow[i]);
    Serial.print(" target ");
    Serial.println(posTarget[i]);
  }
  Serial.print("Array now: {");
  for (int i = 0; i < JUMLAH_SERVO; i++) {
    Serial.print(posNow[i]);
    if (i < JUMLAH_SERVO - 1) Serial.print(", ");
  }
  Serial.println("}");
}

void scanI2C() {
  Serial.println("I2C_SCAN_BEGIN");
  int found = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C_FOUND=0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    }
  }
  Serial.print("I2C_TOTAL=");
  Serial.println(found);
  Serial.println("I2C_SCAN_END");
}
