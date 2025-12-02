#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <AceButton.h>

using namespace ace_button;

class FilteredDigitalInput {
public:
  FilteredDigitalInput(int pin, uint32_t filter_period_us) : pin_(pin), filter_period_us_(filter_period_us) {}

  void begin() {
    pinMode(pin_, INPUT);
  }

  void update(uint32_t time_us) {
    bool value = !!digitalRead(pin_);
    if (value != pending_value_) {
      pending_value_ = value;
      pending_change_time_us_ = time_us;
    } else if (time_us - pending_change_time_us_ > filter_period_us_) {
      filtered_value_ = value;
    }
  }

  bool value() const { return filtered_value_; }

private:
  const int pin_;
  const uint32_t filter_period_us_;

  bool pending_value_{};
  uint32_t pending_change_time_us_{};
  bool filtered_value_{};
};

class QuadratureDecoder {
public:
  int32_t update(bool phase1, bool phase2) {
    uint8_t bits = (phase1 ? 1 : 0) | (phase2 ? 2 : 0);
    int32_t change = 0;
    if (bits != bits_) {
      constexpr int8_t TRANSITIONS[] = {
        0, 1, -1, 0 /*error*/,
        -1, 0, 0 /*error*/, 1,
        1, 0 /*error*/, 0, -1,
        0 /*error*/, -1, 1, 0,
      };
      change = int32_t(TRANSITIONS[(bits_ << 2) | bits]);
      count_ += change;
      bits_ = bits;
    }
    return change;
  }

  int32_t count() const { return count_; }

  void reset_count() { count_ = 0; }

private:
  uint8_t bits_{};
  int32_t count_{};
};

class MotorTracker {
public:
  int32_t update(uint32_t time_us, bool hall1, bool hall2, bool enabled) {
    int32_t change = decoder_.update(hall1, hall2);
    if (change) {
      last_not_stalled_time_us_ = time_us;
      stalled_ = false;
    } else if (!enabled) {
      last_not_stalled_time_us_ = time_us;
      stalled_ = false;
    } else {
      // TODO: Ideally we would measure the motor current to detect activation
      // of the limit switch but we don't have enough pins for that so instead
      // we wait to observe no movement while the motor driver is continously enabled
      constexpr uint32_t STALL_DURATION_US = 400000;
      if (time_us - last_not_stalled_time_us_ > STALL_DURATION_US) {
        stalled_ = true;
      }
    }
    return change;
  }

  bool stalled() const { return stalled_; }

  int32_t position() const { return decoder_.count(); }

  void reset_position() { decoder_.reset_count(); }

private:
  QuadratureDecoder decoder_;
  uint32_t last_not_stalled_time_us_{};
  bool stalled_{};
};

constexpr int INDICATOR1_POWER_PIN = 11;
constexpr int INDICATOR1_DATA_PIN = 12;
Adafruit_NeoPixel indicator1(1, INDICATOR1_DATA_PIN, NEO_GRB + NEO_KHZ800);

constexpr int INDICATOR2_R_PIN = 17;
constexpr int INDICATOR2_G_PIN = 16;
constexpr int INDICATOR2_B_PIN = 25;

constexpr int MOTOR_NFAULT_PIN = 27;
constexpr int MOTOR_A_IN1_PIN = 29;
constexpr int MOTOR_A_IN2_PIN = 28;
constexpr int MOTOR_B_IN1_PIN = 0;
constexpr int MOTOR_B_IN2_PIN = 26;
constexpr int MOTOR_A_HALL1_PIN = 7;
constexpr int MOTOR_A_HALL2_PIN = 6;
constexpr int MOTOR_B_HALL1_PIN = 2;
constexpr int MOTOR_B_HALL2_PIN = 1;
constexpr uint32_t MOTOR_HALL_FILTER_PERIOD_US = 1000; // 2000 is too big, 800 is fine, 1000 is fine
FilteredDigitalInput motor_a_hall1(MOTOR_A_HALL1_PIN, MOTOR_HALL_FILTER_PERIOD_US);
FilteredDigitalInput motor_a_hall2(MOTOR_A_HALL2_PIN, MOTOR_HALL_FILTER_PERIOD_US);
FilteredDigitalInput motor_b_hall1(MOTOR_B_HALL1_PIN, MOTOR_HALL_FILTER_PERIOD_US);
FilteredDigitalInput motor_b_hall2(MOTOR_B_HALL2_PIN, MOTOR_HALL_FILTER_PERIOD_US);

constexpr int DIR_UP_PIN = 3;
constexpr int DIR_DOWN_PIN = 4;
ButtonConfig button_config;
AceButton dir_up_button(&button_config, DIR_UP_PIN, HIGH /*default_released_state*/);
AceButton dir_down_button(&button_config, DIR_DOWN_PIN, HIGH  /*default_released_state*/);

void handle_button(AceButton* button, uint8_t event_type, uint8_t button_state);

void set_indicator1(uint32_t color) {
  static uint32_t last_color = 0xffffffff; // invalid color
  if (color != last_color) {
    indicator1.setPixelColor(0, color);
    indicator1.show();
    last_color = color;
  }
}

void set_indicator1_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  set_indicator1(Adafruit_NeoPixel::Color(red, green, blue));
}

void set_indicator1_hue(uint16_t hue) {
  set_indicator1(Adafruit_NeoPixel::ColorHSV(hue));
}

void set_indicator2(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(INDICATOR2_R_PIN, 255 - red);
  analogWrite(INDICATOR2_G_PIN, 255 - green);
  analogWrite(INDICATOR2_B_PIN, 255 - blue);
}

void setup() {
  analogWriteResolution(8);

  pinMode(INDICATOR1_POWER_PIN, OUTPUT);
  digitalWrite(INDICATOR1_POWER_PIN, true);
  indicator1.begin();
  indicator1.setBrightness(96);
  set_indicator1_rgb(0, 0, 0);

  pinMode(INDICATOR2_R_PIN, OUTPUT);
  pinMode(INDICATOR2_G_PIN, OUTPUT);
  pinMode(INDICATOR2_B_PIN, OUTPUT);
  set_indicator2(0, 0, 0);

  motor_a_hall1.begin();
  motor_a_hall2.begin();
  motor_b_hall1.begin();
  motor_b_hall2.begin();

  pinMode(MOTOR_NFAULT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  digitalWrite(MOTOR_A_IN1_PIN, false);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_A_IN2_PIN, false);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);
  digitalWrite(MOTOR_B_IN1_PIN, false);
  pinMode(MOTOR_B_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_A_IN2_PIN, false);

  pinMode(DIR_UP_PIN, INPUT);
  pinMode(DIR_DOWN_PIN, INPUT);
  button_config.setLongPressDelay(500);
  button_config.setFeature(ButtonConfig::kFeatureLongPress);
  button_config.setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  button_config.setEventHandler(handle_button);
}

constexpr uint8_t KEY_UP_LONG = 1u << 0;
constexpr uint8_t KEY_DOWN_LONG = 1u << 1;
uint8_t key_state = 0;

void handle_button(AceButton* button, uint8_t event_type, uint8_t button_state) {
  switch (event_type) {
    case AceButton::kEventLongPressed:
      key_state |= button->getPin() == DIR_UP_PIN ? KEY_UP_LONG : KEY_DOWN_LONG;
      break;
    case AceButton::kEventLongReleased:
      key_state &= ~(button->getPin() == DIR_UP_PIN ? KEY_UP_LONG : KEY_DOWN_LONG);
      break;
  }
}

MotorTracker motor_a_tracker;
MotorTracker motor_b_tracker;

void loop() {
  static bool error = false;
  static int32_t motor_a_movement = 0;
  static int32_t motor_b_movement = 0;

  dir_down_button.check();
  dir_up_button.check();

  const uint32_t time_us = micros();
  motor_a_hall1.update(time_us);
  motor_a_hall2.update(time_us);
  motor_b_hall1.update(time_us);
  motor_b_hall2.update(time_us);

  const bool motor_fault = !digitalRead(MOTOR_NFAULT_PIN);
  if (motor_fault) {
    error = true;
  }

  const bool motor_a_enabled = digitalRead(MOTOR_A_IN1_PIN) ^ digitalRead(MOTOR_A_IN2_PIN);
  const bool motor_b_enabled = digitalRead(MOTOR_B_IN1_PIN) ^ digitalRead(MOTOR_B_IN2_PIN);
  motor_a_movement += motor_a_tracker.update(time_us, motor_a_hall1.value(), motor_a_hall2.value(), motor_a_enabled);
  motor_b_movement += motor_b_tracker.update(time_us, motor_b_hall1.value(), motor_b_hall2.value(), motor_b_enabled);

  constexpr int32_t SYNC_STEPS = 2; // Maximum step delta before coasting to synchronize
  constexpr int32_t CATCH_UP_STEPS = 100; // Maximum step delta to catch up after one motor hits the limit switch and stalls
  constexpr int32_t MINIMUM_MOVEMENT = 20; // Minimum motor movement before requiring synchronization
  const int32_t delta = motor_a_tracker.position() - motor_b_tracker.position();
  if (abs(delta) > CATCH_UP_STEPS) {
    error = true;
  }
  if (motor_a_tracker.stalled() && motor_b_tracker.stalled()) {
    motor_a_tracker.reset_position();
    motor_b_tracker.reset_position();
  }

  if (!error && key_state == KEY_UP_LONG) {
    digitalWrite(MOTOR_A_IN1_PIN, delta <= SYNC_STEPS || motor_b_tracker.stalled() || abs(motor_a_movement) < MINIMUM_MOVEMENT);
    digitalWrite(MOTOR_A_IN2_PIN, false);
    digitalWrite(MOTOR_B_IN1_PIN, -delta <= SYNC_STEPS || motor_a_tracker.stalled() || abs(motor_b_movement) < MINIMUM_MOVEMENT);
    digitalWrite(MOTOR_B_IN2_PIN, false);
  } else if (!error && key_state == KEY_DOWN_LONG) {
    digitalWrite(MOTOR_A_IN1_PIN, false);
    digitalWrite(MOTOR_A_IN2_PIN, -delta <= SYNC_STEPS || motor_b_tracker.stalled() || abs(motor_a_movement) < MINIMUM_MOVEMENT);
    digitalWrite(MOTOR_B_IN1_PIN, false);
    digitalWrite(MOTOR_B_IN2_PIN, delta <= SYNC_STEPS || motor_a_tracker.stalled() || abs(motor_b_movement) < MINIMUM_MOVEMENT);
  } else {
    digitalWrite(MOTOR_A_IN1_PIN, true);
    digitalWrite(MOTOR_A_IN2_PIN, true);
    digitalWrite(MOTOR_B_IN1_PIN, true);
    digitalWrite(MOTOR_B_IN2_PIN, true);
  }
  if (key_state == 0) {
    if (error) {
      // FIXME: dubious!
      motor_a_tracker.reset_position();
      motor_b_tracker.reset_position();
    }
    error = false;
    motor_a_movement = 0;
    motor_b_movement = 0;
  }

  if (key_state == 0) {
    set_indicator1_rgb(0, 0, 0);
  } else if (error && (time_us / 250000) % 2) {
    set_indicator1_hue(0); // blink red when faulted
  } else {
    set_indicator1_hue(motor_a_tracker.position() * 10);
  }

  set_indicator2(
      motor_a_enabled ? 255 : 0,
      motor_b_enabled ? 128 : 0,
      motor_a_tracker.stalled() && motor_b_tracker.stalled() ? 128 : 0);
}
