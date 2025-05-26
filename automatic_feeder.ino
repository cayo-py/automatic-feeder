#define BLYNK_TEMPLATE_ID "[Your_Blynk_Template_ID]"
#define BLYNK_TEMPLATE_NAME "[Your_Blynk_Template_Name]"
#define BLYNK_AUTH_TOKEN "[Your_Blynk_Auth_Token]"

#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// ====== Watchdog =======
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 10 // Timeout in seconds
esp_err_t ESP32_ERROR;

// ========== Blynk configuration ==========
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "[Your_SSID]";
char pass[] = "[Your_Password]";

// ========== component ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
Servo myServo;

// ========== Pin ==========
const int trigPin = 4;
const int echoPin = 5;
const int buzzerPin = 14;
const int servoPin = 13;
const int pushButton = 12;

// ========== default schedule ==========
int hourSch1 = 6, minuteSch1 = 0, secondSch1 = 0;
int hourSch2 = 18, minuteSch2 = 0, secondSch2 = 0;

// ========== Variabel ==========
bool alarmActive = false;
int state = 0;
int curAngle = 0;
float lastDist = 0.0;
bool notif = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  rtc.begin();
  lcd.begin(16, 2);
  lcd.backlight();

  // ======== Watchdog Setup start ============ (source: https://f1atb.fr/esp32-watchdog-example/)
  esp_task_wdt_deinit();
  // Task Watchdog configuration
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,                 // Convertin ms
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores, https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/task_watchdog/main/task_watchdog_example_main.c
    .trigger_panic = true                             // Enable panic to restart ESP32
  };
  // WDT Init
  ESP32_ERROR = esp_task_wdt_init(&wdt_config);
   Serial.println("Last Reset : " + String(esp_err_to_name(ESP32_ERROR)));
   esp_task_wdt_add(NULL);  //add current thread to WDT watch
  // ======== Watchdog Setup end ============

  if (!rtc.begin()) {
    while (1);
  }

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(0);

  lcd.setCursor(2, 0);
  lcd.print("Auto  Feeder");
  delay(1500);
  lcd.clear();

  Blynk.begin(auth, ssid, pass);

  Blynk.syncVirtual(V1); // Re-request data from TimeInput 1
  Blynk.syncVirtual(V2); // Re-request data from TimeInput 2
  
  esp_task_wdt_reset();
}

void loop() {
  Blynk.run();

  switch (state) {
    case 0: state0(); break;
    case 1: state1(); break;
    case 2: state2(); break;
  }

  esp_task_wdt_reset();
}

// ========== Getting Started with Blynk Functions ==========
BLYNK_WRITE(V0) { // Switch
  int pushButtonVirtual = param.asInt();
  if (pushButtonVirtual == 1 && state == 0) {
    alarmActive = true;
    notif = true;
    state = 1;
  }
}

BLYNK_WRITE(V1) { // Set time1
  TimeInputParam t(param);
  if (t.hasStartTime()) {
    hourSch1 = t.getStartHour();
    minuteSch1 = t.getStartMinute();
    secondSch1 = t.getStartSecond();
  }
}

BLYNK_WRITE(V2) { // Set time2
  TimeInputParam t(param);
  if (t.hasStartTime()) {
    hourSch2 = t.getStartHour();
    minuteSch2 = t.getStartMinute();
    secondSch2 = t.getStartSecond();
  }
}
// ========== End of Blynk Functions ==========

// ========== Rotate the servo per PWM ==========
void rotateServo(int angle) {
  angle = constrain(angle, 0, 180); // angle restrictions

  if (curAngle < angle) {
    for (int i = curAngle; i <= angle; i++) {
      myServo.write(i);
      delay(1);
    }
  } else if (curAngle > angle) {
    for (int i = curAngle; i >= angle; i--) {
      myServo.write(i);
      delay(1);
    }
  }

  // Redundancy to ensure servos are in target positions
  for (int j = 0; j < 3; j++) {
    myServo.write(angle);
    delay(10);
  }

  curAngle = angle;
}

// ========== Open and close the Servo to feed it ==========
void feedFunc() {
  rotateServo(80);
  delay(50);
  rotateServo(0);
  delay(1500);
  esp_task_wdt_reset();
}

// ========== Ultrasonic distance process ==========
float readDist() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long durasi = pulseIn(echoPin, HIGH);
  float distance = durasi * 0.034 / 2;
  return distance;
}

void showNextSch() {
  DateTime now = rtc.now();
  DateTime sched1(now.year(), now.month(), now.day(), hourSch1, minuteSch1, secondSch1);
  DateTime sched2(now.year(), now.month(), now.day(), hourSch2, minuteSch2, secondSch2);

  if (sched1 <= now) sched1 = sched1 + TimeSpan(86400); // Add 1 day if it has passed
  if (sched2 <= now) sched2 = sched2 + TimeSpan(86400);

  DateTime nextSch = sched1 < sched2 ? sched1 : sched2;

  lcd.setCursor(0, 1);
  lcd.print("Next: ");
  lcd.print(nextSch.timestamp(DateTime::TIMESTAMP_TIME));
}


// ========== Initial State Function ==========
/*
Main State
starting from real time reading, printing to LCD for time and stock based on RTC and distance
the part that is looped continuously from the start of the system to check the schedule and condition of the push button
*/
void state0() {
  DateTime now = rtc.now();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();

  float distance = readDist();
  lastDist = distance;

  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.setCursor(6, 0);
  if (hour < 10) lcd.print("0"); lcd.print(hour); lcd.print(":");
  if (minute < 10) lcd.print("0"); lcd.print(minute); lcd.print(":");
  if (second < 10) lcd.print("0"); lcd.print(second);

  esp_task_wdt_reset();
  
  showNextSch();

  if ((hour == hourSch1 && minute == minuteSch1 && second == secondSch1 && !alarmActive) ||
      (hour == hourSch2 && minute == minuteSch2 && second == secondSch2 && !alarmActive)) {
    alarmActive = true;
    notif = true;
    state = 1;
  }
  else if (digitalRead(pushButton) == LOW) {
    alarmActive = true;
    notif = true;
    state = 1;
  }
}

/*
state when the feeding schedule has been entered or the push button is pressed to feed
will activate the buzzer as a sign of feeding
update text on the LCD
open and close the servo to feed
*/
void state1() {
  digitalWrite(buzzerPin, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time to");
  lcd.setCursor(0, 1);
  lcd.print("Feed!");
  
  esp_task_wdt_reset();

  feedFunc();
  
  lcd.clear();

  state = 2;
}

/*
feeding process
reading distance to re-confirm feed stock
if stock is available, turn off buzzer and return to state 0 (send notification to blynk)
if not, turn buzzer on and off every 0.5 seconds repeatedly without returning to state 0 (send notification to blynk)
*/
void state2() {
  lastDist = readDist();
  Serial.print("dist: ");
  Serial.println(lastDist);

  if (lastDist <= 3.5) {
    lcd.setCursor(0, 0);
    lcd.print("STOCK");
    lcd.setCursor(0, 1);
    lcd.print("AVAIL!    ");
    if (notif) {
      Blynk.logEvent("[event_name1]", "Messages");
      notif = false;
    }

    esp_task_wdt_reset();

    delay(1000);
    lcd.clear();
    digitalWrite(buzzerPin, LOW);
    alarmActive = false;
    state = 0;
  }
  else {
    lcd.setCursor(0, 0);
    lcd.print("STOCK");
    lcd.setCursor(0, 1);
    lcd.print("NOT AVAIL!");
    if (notif) {
      Blynk.logEvent("[event_name2", "Messages");
      notif = false;
    }

    esp_task_wdt_reset();

    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW); 
    delay(500);
  }
}
