// Manual servo angle calibration for the Pet Dog quadruped.
// Key-based control over Serial Monitor -- no trim, no FR/BL inversion --
// so you can find the true angle at which each leg hangs perpendicular
// to the ground.
//
// Uses the ESP-IDF `driver/ledc.h` API directly (same as the real firmware's
// servo.c), with one hardcoded LEDC channel per leg. This intentionally
// avoids the Arduino "ESP32Servo" library: on ESP32-S3 that library's
// automatic channel/timer allocation can let two Servo objects end up
// sharing one physical PWM channel, so moving one leg visibly drags
// another leg with it. Explicit channels here guarantee each leg is
// fully independent, exactly like the working animation firmware.
//
// Board: XIAO_ESP32S3 (or "ESP32S3 Dev Module"), matching GPIOs from servo.h.
// Serial Monitor: 115200 baud, line ending doesn't matter (key-by-key).
//
// Keys:
//   1 / 2 / 3 / 4   -> attach + select FL / FR / BL / BR
//   x               -> detach all four servos (goes limp, free to rotate by hand)
//   +               -> selected leg +1 degree
//   -               -> selected leg -1 degree
//   [               -> selected leg +5 degrees
//   ]               -> selected leg -5 degrees
//
// After every keypress the current angle of all four legs is printed.
//
// Once you find the raw angle where a leg is perpendicular to the ground,
// that becomes the new *_TRIM for that leg in servo.c:
//   TRIM = raw_angle_found - 90
// (This holds for all four legs, including FR/BL, because the inversion
// is applied before the trim in servo.c's set_fr/set_bl.)

#include "driver/ledc.h"

#define SERVO_FL_GPIO 3
#define SERVO_FR_GPIO 4
#define SERVO_BL_GPIO 5
#define SERVO_BR_GPIO 6

#define SERVO_FL_CH LEDC_CHANNEL_0
#define SERVO_FR_CH LEDC_CHANNEL_1
#define SERVO_BL_CH LEDC_CHANNEL_2
#define SERVO_BR_CH LEDC_CHANNEL_3

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ_HZ 50

struct Leg {
  int gpio;
  ledc_channel_t channel;
  int angle;
  bool attached;
  const char *name;
};

Leg legs[4] = {
  {SERVO_FL_GPIO, SERVO_FL_CH, 90, false, "FL"},
  {SERVO_FR_GPIO, SERVO_FR_CH, 90, false, "FR"},
  {SERVO_BL_GPIO, SERVO_BL_CH, 90, false, "BL"},
  {SERVO_BR_GPIO, SERVO_BR_CH, 90, false, "BR"},
};

int selected = -1; // index into legs[], -1 = none selected yet

static uint32_t angleToDuty(int degrees) {
  int pulse_us = SERVO_MIN_US + (degrees * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
  return (uint32_t)((pulse_us * 16383) / 20000); // 14-bit duty, 50 Hz (20000 us period)
}

static void writeAngle(Leg &leg, int degrees) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, leg.channel, angleToDuty(degrees));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, leg.channel);
}

static void pwmOff(Leg &leg) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, leg.channel, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, leg.channel);
}

void printAngles() {
  for (int i = 0; i < 4; i++) {
    Serial.print(legs[i].name);
    Serial.print("=");
    Serial.print(legs[i].angle);
    Serial.print(legs[i].attached ? "* " : "  ");
  }
  if (selected >= 0) {
    Serial.print(" [selected: ");
    Serial.print(legs[selected].name);
    Serial.print("]");
  }
  Serial.println();
}

void selectLeg(int idx) {
  Leg &leg = legs[idx];
  if (!leg.attached) {
    writeAngle(leg, leg.angle); // resume PWM at its last known angle
    leg.attached = true;
  }
  selected = idx;
  printAngles();
}

void detachAll() {
  for (int i = 0; i < 4; i++) {
    if (legs[i].attached) {
      pwmOff(legs[i]);
      legs[i].attached = false;
    }
  }
  selected = -1;
  printAngles();
}

void nudge(int delta) {
  if (selected < 0) {
    Serial.println("No leg selected. Press 1/2/3/4 first.");
    return;
  }
  Leg &leg = legs[selected];
  leg.angle = constrain(leg.angle + delta, 0, 180);
  writeAngle(leg, leg.angle);
  printAngles();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = SERVO_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer);

  for (int i = 0; i < 4; i++) {
    ledc_channel_config_t ch = {};
    ch.gpio_num = legs[i].gpio;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = legs[i].channel;
    ch.timer_sel = LEDC_TIMER_0;
    ch.intr_type = LEDC_INTR_DISABLE;
    ch.duty = 0; // start with PWM off (detached) on all legs
    ch.hpoint = 0;
    ledc_channel_config(&ch);
  }

  Serial.println();
  Serial.println("=== Servo calibration ready (raw ledc driver, one channel per leg) ===");
  Serial.println("1/2/3/4 = attach+select FL/FR/BL/BR");
  Serial.println("x       = detach all (free to rotate by hand)");
  Serial.println("+ / -   = selected leg +1 / -1 degree");
  Serial.println("[ / ]   = selected leg +5 / -5 degrees");
  printAngles();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();

  switch (c) {
    case '1': selectLeg(0); break;
    case '2': selectLeg(1); break;
    case '3': selectLeg(2); break;
    case '4': selectLeg(3); break;
    case 'x': case 'X': detachAll(); break;
    case '+': nudge(1); break;
    case '-': nudge(-1); break;
    case '[': nudge(5); break;
    case ']': nudge(-5); break;
    default: break; // ignore newlines, carriage returns, anything else
  }
}
