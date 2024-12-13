// --------------------------------------------
// Freezer X Controller Firmware
// Yaseen M. Twati - 2024 | https://yaseen.ly
// --------------------------------------------

#include <Arduino.h>
#include <Adafruit_GC9A01A.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <NTC_Thermistor.h>
#include <RtcDS1302.h>
#include <SPI.h>
#include <Thermistor.h>
#include <avr/wdt.h>

// --------------------------------------
// Pins

// Thermistors
const int THERMISTOR_1_PIN = A8;
const int THERMISTOR_2_PIN = A9;

//// RTC
const int RTC_CLK = 4;
const int RTC_IO = 5;
const int RTC_CE = 6;

// Rotary Encoder
const int ROTARY_CLK = 2;
const int ROTARY_DT = 3;
const int ROTARY_SW = 7;

// LCD
const int LCD_RST = 20;
const int LCD_DC = 21;
const int LCD_DIN = 16;
const int LCD_SCK = 15;
const int LCD_CS = 19;

// Micro SD
const int MICRO_SD_CS = 10;
const int MICRO_SD_MOSI = 16;
const int MICRO_SD_MISO = 14;
const int MICRO_SD_SCK = 15;

// Compressor Relay
const int COMPRESSOR_RELAY = 18;

// LEDs
const int BLUE_LED_PIN = 0;
const int RED_LED_PIN = 1;

// --------------------------------------
// Constants

// the int16_t don't actually have to be signed, they can be uint16_t
// but having them as signed makes it easier for me to have them wrap around in the code.

// Config Menu Constants
const float MIN_TARGET_TEMPERATURE = -22.0;
const float MAX_TARGET_TEMPERATURE = -10.0;
const float TARGET_TEMPERATURE_INCREMENT = 1.0;

const int16_t MIN_TARGET_TEMPERATURE_HYSTERESIS_TIME = 5;
const int16_t MAX_TARGET_TEMPERATURE_HYSTERESIS_TIME = 350;
const int16_t TARGET_TEMPERATURE_HYSTERESIS_TIME_INCREMENT = 10;

const int16_t MIN_COMPRESSOR_DEAD_TIME = 2;
const int16_t MAX_COMPRESSOR_DEAD_TIME = 20;
const int16_t COMPRESSOR_DEAD_TIME_INCREMENT = 1;

const int16_t MIN_COMPRESSOR_MAX_RUNTIME = 60;
const int16_t MAX_COMPRESSOR_MAX_RUNTIME = 600; // 10 hours ?
const int16_t COMPRESSOR_MAX_RUNTIME_INCREMENT = 10;

const float MIN_COMPRESSOR_MAX_TEMP = 30.0;
const float MAX_COMPRESSOR_MAX_TEMP = 60.0;
const float COMPRESSOR_MAX_TEMP_INCREMENT = 1.0;

const int16_t MIN_FREEZER_STARTUP_DELAY = 0;
const int16_t MAX_FREEZER_STARTUP_DELAY = 20;
const int16_t FREEZER_STARTUP_DELAY_INCREMENT = 1;

// NTC Constants
const int REFERENCE_RESISTANCE = 10000;
const int NOMINAL_TEMPERATURE = 25;
const int B_VALUE_NTC_1 = 3950;
const int B_VALUE_NTC_2 = 3950;

const int ANALOG_RESOLUTION = 1023;

const uint32_t NOMINAL_RESISTANCE_NTC_1 = 10000;
const uint32_t NOMINAL_RESISTANCE_NTC_2 = 100000;

// SD CARD SECTOR SIZE
const uint16_t SD_BLOCK_SIZE  = 512;

// LCD Constants
// these are mainly here to save a few bytes instead of calling tft.width() & tft.height()
const int16_t LCD_SIZE = 240; // width is same as height
const int16_t LCD_CENTER = 120;

// --------------------------------------
// Structures & Enums

struct freezer_config
{
  char magic_bytes[4] = {'F', '0', '0', '2'}; // 4

  float target_temperature = -18.0;  // Degrees Celsius // 4

  int16_t target_temperature_hysteresis_time = 60;  // 15 seconds // 2

  int16_t compressor_dead_time = 5;     // minutes // 2
  int16_t compressor_max_run_time = 300; // 5 hours // in minutes // 2

  float compressor_max_temp = 45.0; // 4

  int16_t freezer_startup_delay = 2;  // minutes; // 2
};

enum freezer_status
{
  off = 0,
  cooling = 1,
//  hysteresis = 2,
  reached_target = 2,
  dead_time = 3,
  compressor_max_runtime = 4,
  startup_delay = 5,
  overheat = 6,
};

const char* freezer_status_strings[] = {
    "",
    "   Cooling   ",
//    "",
    "Within Target",
    "  Dead Time  ",
    " Max Runtime ",
    "Startup Delay",
    "  Overheat   ",
};

uint16_t status_colors[][2] = {
    {GC9A01A_WHITE, GC9A01A_WHITE},
    {GC9A01A_BLUE, GC9A01A_PURPLE},
//    {GC9A01A_WHITE, GC9A01A_WHITE},
    {GC9A01A_GREEN, GC9A01A_GREEN},
    {GC9A01A_WHITE, GC9A01A_WHITE},
    {GC9A01A_ORANGE, GC9A01A_YELLOW},
    {GC9A01A_ORANGE, GC9A01A_YELLOW},
    {GC9A01A_RED, GC9A01A_ORANGE},
};

struct freezer_state
{
  double current_ntc1_temperature = 0.0; // 4 bytes
  double current_ntc2_temperature = 0.0; // 4 bytes

  bool target_compressor_state = LOW;
  bool actual_compressor_state = LOW;

  freezer_status status = freezer_status::off;
};

struct freezer_state_data_point
{
  volatile uint8_t start_magic_bytes[4] = {'F', 'Z', 'Z', 'X'}; // 4 bytes

//  volatile uint32_t unix_time = 0; // 8 bytes
  volatile uint32_t ms_since_startup = 0;

  freezer_config config; // 24 bytes

  freezer_state state; // 11 bytes

  volatile uint8_t crc = 0; // 4 bytes
};

struct menu_entry
{
  const char* name;
  const char* unit;

  void* value;
  bool is_float;
  float increment;
  float min;
  float max;
};

// --------------------------------------
// Peripherals

Adafruit_GC9A01A tft(LCD_CS, LCD_DC, LCD_RST);

Encoder input_rotary_encoder(ROTARY_DT, ROTARY_CLK);

Thermistor *ntc_1;
Thermistor *ntc_2;

ThreeWire rtc_three_wire(RTC_IO,RTC_CLK,RTC_CE);
RtcDS1302<ThreeWire> rtc_clock(rtc_three_wire);

// --------------------------------------
// Global Variables

bool logging_enabled = false;
bool config_is_dirty = false;
bool edit_mode = false;
bool screen_should_refresh = false;
bool showing_new_screen = false;

freezer_config active_config {};
freezer_config dirty_config {};

freezer_state current_state {};

uint32_t compressor_turned_on_at;
uint32_t compressor_turned_off_at;
uint32_t reached_target_temperature_at;
uint32_t state_updated_at = 0;
uint32_t edit_mode_toggled_at = 0;
uint32_t exceeded_max_runtime_at = 0;

int8_t screen_index;

int32_t last_encoder_position = 0;

bool startup_delay_over = false;

freezer_status previous_status = freezer_status::off;

menu_entry menu_entries[] ={
    {"Target Temp", "Degrees C", &dirty_config.target_temperature, true, TARGET_TEMPERATURE_INCREMENT, MIN_TARGET_TEMPERATURE, MAX_TARGET_TEMPERATURE},
    {"Hysteresis", "Seconds", &dirty_config.target_temperature_hysteresis_time, false, TARGET_TEMPERATURE_HYSTERESIS_TIME_INCREMENT, MIN_TARGET_TEMPERATURE_HYSTERESIS_TIME, MAX_TARGET_TEMPERATURE_HYSTERESIS_TIME},
    {"Dead Time", "Minutes", &dirty_config.compressor_dead_time, false, COMPRESSOR_DEAD_TIME_INCREMENT, MIN_COMPRESSOR_DEAD_TIME, MAX_COMPRESSOR_DEAD_TIME},
    {"Max Runtime", "Minutes", &dirty_config.compressor_max_run_time, false, COMPRESSOR_MAX_RUNTIME_INCREMENT, MIN_COMPRESSOR_MAX_RUNTIME, MAX_COMPRESSOR_MAX_RUNTIME},
    {"Max Comp Temp", "Degrees C", &dirty_config.compressor_max_temp, true, COMPRESSOR_MAX_TEMP_INCREMENT, MIN_COMPRESSOR_MAX_TEMP, MAX_COMPRESSOR_MAX_TEMP},
    {"Startup Delay", "Minutes", &dirty_config.freezer_startup_delay, false, FREEZER_STARTUP_DELAY_INCREMENT, MIN_FREEZER_STARTUP_DELAY, MAX_FREEZER_STARTUP_DELAY},
};

bool reset_by_watchdog = false;

const uint8_t HOME_SCREEN = 0;
const uint8_t INFO_SCREEN = 7;

// --------------------------------------
// Function Definitions

void initialize_thermistors();

void initialize_display();

void initialize_status_leds();

void initialize_logging();

void initialize_rtc();

void initialize_watchdog();

void initialize_compressor();

void initialize_input();

void load_config();

void save_dirty_config();

void read_sensors();

volatile void log_state();

void update_state();

void validate_temperatures();

void halt(const char* error);

bool is_compressor_overheating();

bool is_startup_delay_over();

bool is_compressor_on();

bool is_within_target_temperature();

bool has_hysteresis_time_elapsed();

bool has_compressor_exceeded_max_runtime();

bool has_dead_time_elapsed();

void set_compressor(bool state);

void handle_input();

void on_button_pressed();

void show_centered_text(const char *text, uint8_t font_size, int16_t x_offset = 0, int16_t y_offset = 0, uint16_t color = GC9A01A_WHITE);

void refresh_display();

void display_home_screen();

void display_info_screen();

bool init_sd_card();

bool save_data_point(const freezer_state_data_point* data);

bool wait_card_busy();

void send_card_command(uint8_t cmd, uint32_t arg, uint8_t crc);

uint8_t read_card_response();

bool read_card_block(uint32_t block_addr, uint8_t* buffer, uint16_t buffer_size);

bool write_card_block(uint32_t block_addr, const void* buffer, uint16_t buffer_size);

uint8_t calculate_crc8(const uint8_t *data, uint8_t len);

// --------------------------------------

void setup()
{
  initialize_watchdog();
  initialize_status_leds();

  load_config();

  initialize_logging();
  initialize_display();
  initialize_thermistors();
  initialize_compressor();
//  initialize_rtc();
  initialize_input();


  showing_new_screen = true;
  screen_should_refresh = true;
}

void loop()
{
  if(millis() - state_updated_at >= 1000) // we refresh once a second, which is more than needed
  {
    if(config_is_dirty)
    {
      active_config = dirty_config;
      config_is_dirty = false;
    }

    update_state();

    state_updated_at = millis();
  }

  handle_input();
  refresh_display();

  wdt_reset();
}

// --------------------------------------

void initialize_thermistors()
{
  // Serial.println("initialize_thermistors()");

  ntc_1 = new NTC_Thermistor(
      THERMISTOR_1_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE_NTC_1,
      NOMINAL_TEMPERATURE,
      B_VALUE_NTC_1,
      ANALOG_RESOLUTION);

  ntc_2 = new NTC_Thermistor(
      THERMISTOR_2_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE_NTC_2,
      NOMINAL_TEMPERATURE,
      B_VALUE_NTC_2,
      ANALOG_RESOLUTION);
}

void initialize_display()
{
  // Serial.println("initialize_display()");

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(GC9A01A_BLACK);
  tft.setTextWrap(false);
}

void initialize_status_leds()
{
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
}

void initialize_logging()
{
  // Serial.println("initialize_logging()");

  if(init_sd_card())
  {
    // Serial.println("initialize_logging(): successfully initialized sd card, enabling logging");
    logging_enabled = true;
  }
  else
  {
    // Serial.println("initialize_logging(): could not initialize sd card, logging is disabled");
  }
}

void initialize_compressor()
{
  // Serial.println("initialize_compressor()");

  pinMode(COMPRESSOR_RELAY, OUTPUT);
  digitalWrite(COMPRESSOR_RELAY, LOW);
}

void initialize_input()
{
  // Serial.println("initialize_input()");

  pinMode(ROTARY_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTARY_SW), on_button_pressed, RISING);
}

void initialize_watchdog()
{
  reset_by_watchdog = (MCUSR & (1 << WDRF));

  MCUSR = 0;

  wdt_enable(WDTO_2S);
}

//void initialize_rtc()
//{
// // Serial.println("initialize_rtc()");
//
// rtc_clock.Begin();
//
// RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
//
// if (!rtc_clock.IsDateTimeValid())
// {
//   // Serial.println("initialize_rtc(): RTC lost confidence in the DateTime!");
//   rtc_clock.SetDateTime(compiled);
// }
//
// if (rtc_clock.GetIsWriteProtected())
// {
//     // Serial.println("initialize_rtc(): RTC was write protected, enabling writing now");
//     rtc_clock.SetIsWriteProtected(false);
// }
//
// if (!rtc_clock.GetIsRunning())
// {
//     // Serial.println("initialize_rtc(): RTC was not actively running, starting now");
//     rtc_clock.SetIsRunning(true);
// }
//
// RtcDateTime now = rtc_clock.GetDateTime();
// if (now < compiled)
// {
//     // Serial.println("initialize_rtc(): RTC is older than compile time!  (Updating DateTime)");
//     rtc_clock.SetDateTime(compiled);
// }
//}

// --------------------------------------

void load_config()
{
  // Serial.println("load_config()");

  freezer_config config;
  EEPROM.get(0, config);

  // make sure we have a valid config by comparing the magic bytes, otherwise reset to the default state
  if(memcmp(config.magic_bytes, "F002", 4) != 0)
  {
    // we flash the LEDs just to indicate that we are resetting the config

    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);

    delay(1000);

    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);

    // Serial.println("load_config(): config in eeprom is invalid, resetting back to default values");

    active_config = freezer_config();
    dirty_config = freezer_config();

    save_dirty_config();
  }
  else
  {
    // Serial.println("load_config(): successfully loaded config from eeprom");

    active_config = config;
    dirty_config = config;
  }
}

void save_dirty_config()
{
  // Serial.println("save_dirty_config()");

  EEPROM.put(0, dirty_config);
}

// --------------------------------------

bool is_compressor_overheating()
{
  return current_state.current_ntc2_temperature >= active_config.compressor_max_temp;
}

bool is_startup_delay_over()
{
  return millis() >= ((uint32_t)active_config.freezer_startup_delay * 60 * 1000);
}

bool is_compressor_on()
{
  return current_state.actual_compressor_state;
}

bool is_within_target_temperature()
{
  // todo: add a 'range'
  return current_state.current_ntc1_temperature <= active_config.target_temperature;
}

bool has_hysteresis_time_elapsed()
{
  return millis() - reached_target_temperature_at >= ((uint32_t)active_config.target_temperature_hysteresis_time * 1000);
}

bool has_compressor_exceeded_max_runtime()
{
  if(exceeded_max_runtime_at > 0)
  {
    // we already exceeded the max runtime, we need to make sure we have stopped for enough time
    if(has_dead_time_elapsed())
    {
      exceeded_max_runtime_at = 0;
      return false;
    }

    return true;
  }
  else
  {
    if(is_compressor_on())
    {
      if(millis() - compressor_turned_on_at >= ((uint32_t)active_config.compressor_max_run_time * 1000 * 60))  // 28612 bytes ( THIS IS SMALLER )
      {
        exceeded_max_runtime_at = millis();
        return true;
      }
    }
  }

  exceeded_max_runtime_at = 0;
  return false;
}

bool has_dead_time_elapsed()
{
  if(compressor_turned_off_at == 0)
  {
    return true;
  }

  return millis() - compressor_turned_off_at >= ((uint32_t)active_config.compressor_dead_time * 1000 * 60);
}

void update_state()
{
  read_sensors();
  validate_temperatures();

  // we are saving startup_delay_over into a variable to make we dont get affected by millis() rolling over after 50 days or so
  if(!startup_delay_over)
  {
    startup_delay_over = is_startup_delay_over();

    if(screen_index == HOME_SCREEN || screen_index == INFO_SCREEN)
    {
      screen_should_refresh = true;
    }

    current_state.status = freezer_status::startup_delay;
    log_state();

    return;
  }

  // base decision-making -----

  if(is_within_target_temperature())
  {
    if(reached_target_temperature_at == 0)
    {
      reached_target_temperature_at = millis();
    }

    if(has_hysteresis_time_elapsed())
    {
      current_state.target_compressor_state = LOW;
      current_state.status = freezer_status::reached_target;
    }
    else
    {
      current_state.target_compressor_state = HIGH;
      current_state.status = freezer_status::cooling;
    }
  }
  else
  {
    reached_target_temperature_at = 0;

    current_state.target_compressor_state = HIGH;
    current_state.status = freezer_status::cooling;
  }

  // overrides -------

  bool override = false;

  // we only need to override if we are planning on turning on the compressor
  if(current_state.target_compressor_state == HIGH)
  {
    if(is_compressor_overheating())
    {
      current_state.status = freezer_status::overheat;
      override = true;
    }
    else if(has_compressor_exceeded_max_runtime())
    {
      current_state.status = freezer_status::compressor_max_runtime;
      override = true;
    }
    else if(!is_compressor_on() && !has_dead_time_elapsed())
    {
      current_state.status = freezer_status::dead_time;
      override = true;
    }
  }

  set_compressor(current_state.target_compressor_state && !override);
  log_state();

  screen_should_refresh = true;
}

void read_sensors()
{
//  delay(10);

  analogRead(THERMISTOR_1_PIN); // dummy read to stabilize the value

  double ntc_1_avg = 0;
  for(int i = 0; i < 10; i++)
  {
    ntc_1_avg += ntc_1->readCelsius();
  }

  current_state.current_ntc1_temperature = ntc_1_avg / 10;

//  delay(10);

  analogRead(THERMISTOR_2_PIN); // dummy read to stabilize the value

  double ntc_2_avg = 0;
  for(int i = 0; i < 10; i++)
  {
    ntc_2_avg += ntc_2->readCelsius();
  }

  current_state.current_ntc2_temperature = ntc_2_avg / 10;
}

void halt(const char* error)
{
  set_compressor(false);

  digitalWrite(RED_LED_PIN, HIGH);

  tft.fillScreen(GC9A01A_RED);

  show_centered_text("ERROR", 4, 0, -30);
  show_centered_text(error, 3, 0, 20);

  for(;;);
}

void validate_temperatures()
{
  if(current_state.current_ntc1_temperature < -100 || current_state.current_ntc1_temperature > 100)
  {
    halt("N1");
  }

  // for the compressor ntc, we don't really care if its disconnected since it'll return -273.15
  // we do care if its shorted though which would return MAX_TEMP
  if(current_state.current_ntc2_temperature > 100)
  {
    halt("N2");
  }
}

// --------------------------------------

void set_compressor(bool state)
{
  if(state != current_state.actual_compressor_state)
  {
    // if we off and going on
    if(state == HIGH)
    {
      compressor_turned_on_at = millis();
      compressor_turned_off_at = 0;
    }
    else
    {
      compressor_turned_off_at = millis();
      compressor_turned_on_at = 0;
    }

    digitalWrite(COMPRESSOR_RELAY, state);
    digitalWrite(BLUE_LED_PIN, state);

    current_state.actual_compressor_state = state;
  }
}

// --------------------------------------

void show_centered_text(const char *text, uint8_t font_size, int16_t x_offset, int16_t y_offset, uint16_t color)
{
  tft.setTextSize(font_size);
  tft.setTextColor(color, GC9A01A_BLACK);

  // Get text dimensions
  int16_t x1, y1;
  uint16_t text_width, text_height;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &text_width, &text_height);

  // start with the center of the screen
  int16_t cursor_x = LCD_CENTER;
  int16_t cursor_y = LCD_CENTER;

  // shift by the offset, so a +100 means on the positive edge
  cursor_x += cursor_x * ((float)x_offset / 100);
  cursor_y += cursor_y * ((float)y_offset / 100);

  // shift back by half of the text size
  cursor_x -= text_width / 2;
  cursor_y -= text_height / 2;

  tft.setCursor(cursor_x, cursor_y);
  tft.print(text);
}

void draw_outer_ring(int16_t start_color, int16_t end_color)
{
  int16_t start_r = (start_color >> 11) & 0x1F;
  int16_t start_g = (start_color >> 5) & 0x3F;
  int16_t start_b = start_color & 0x1F;

  int16_t end_r = (end_color >> 11) & 0x1F;
  int16_t end_g = (end_color >> 5) & 0x3F;
  int16_t end_b = end_color & 0x1F;

  for (int i = 0; i < 8; i++)
  {
    float progress = (float)i / 7;

    int16_t r = start_r + (end_r - start_r) * progress;
    int16_t g = start_g + (end_g - start_g) * progress;
    int16_t b = start_b + (end_b - start_b) * progress;

    // Convert RGB to 16-bit color
    uint16_t color = (r << 11) | (g << 5) | b;

    int current_radius = LCD_CENTER - i;

    tft.drawCircle(LCD_CENTER, LCD_CENTER, current_radius, color);
  }
}

void display_home_screen()
{
  if(showing_new_screen || current_state.status != previous_status)
  {
    previous_status = current_state.status;
    showing_new_screen = false;
    draw_outer_ring(status_colors[current_state.status][0], status_colors[current_state.status][1]);
  }

  if(screen_should_refresh)
  {
    char target_str[16];
    dtostrf(active_config.target_temperature, 4, 1, target_str);

    char target_text[32];
    sprintf(target_text, "Target: %s", target_str);

    show_centered_text(target_text, 2, 0, -50);

    char current_temp_str[16];
    dtostrf(current_state.current_ntc1_temperature, 5, 2, current_temp_str);

    show_centered_text(current_temp_str, 5, 0, 0);
    show_centered_text(freezer_status_strings[current_state.status], 2, 0, 50, status_colors[current_state.status][0]);

    screen_should_refresh = false;
  }
}

void display_info_screen()
{
  if(showing_new_screen)
  {
    showing_new_screen = false;
  }

  if(screen_should_refresh)
  {
    char display_str[16];
    char ntc_str[16];

    dtostrf(current_state.current_ntc1_temperature, 4, 1, ntc_str);
    sprintf(display_str, "NTC1: %s", ntc_str);
    show_centered_text(display_str, 2, 0, -55);

    dtostrf(current_state.current_ntc2_temperature, 4, 1, ntc_str);
    sprintf(display_str, "NTC2: %s", ntc_str);
    show_centered_text(display_str, 2, 0, -30);

//    RtcDateTime now = rtc_clock.GetDateTime();
//
//    char date_str[16];
//    sprintf(date_str, "%04d-%02d-%02d", now.Year(), now.Month(), now.Day());
//
//    char time_str[16];
//    sprintf(time_str, "%02d:%02d:%02d", now.Hour(), now.Minute(), now.Second());
//
//    show_centered_text(date_str, 2, 0, 0);
//    show_centered_text(time_str, 2, 0, 20);
//

    char millis_str[16];
    sprintf(millis_str, "MS: %lu", millis());

    show_centered_text(reset_by_watchdog ? "W RESET: YES" : "W-RESET: NO", 2, 0, 0, reset_by_watchdog ? GC9A01A_RED : GC9A01A_GREEN);

    show_centered_text(millis_str, 2, 0, 20);

    show_centered_text(logging_enabled ? "LOG (TRUE)" : "LOG (FALSE)", 2, 0, 50, logging_enabled ? GC9A01A_GREEN : GC9A01A_RED);

    screen_should_refresh = false;
  }
}

void display_config_screen(int32_t config_index)
{
  if(showing_new_screen)
  {
    showing_new_screen = false;
  }

  if(screen_should_refresh)
  {
    // clear
    menu_entry entry = menu_entries[config_index];

    show_centered_text(entry.name, 2, 0, -50);
    show_centered_text(entry.unit, 2, 0, 50);

    char value_str[32];

    if (entry.is_float)
    {
      dtostrf(*(float*)entry.value, 5, 2, value_str);
    } else {
      itoa(*(uint32_t*)entry.value, value_str, 10);
    }

    tft.fillRect(0, 80, 400, 70, GC9A01A_BLACK);

    show_centered_text(value_str, 5, 0, 0, edit_mode ? GC9A01A_YELLOW : GC9A01A_WHITE);

    screen_should_refresh = false;
  }
}

void refresh_display()
{
  if(showing_new_screen)
  {
    tft.fillScreen(GC9A01A_BLACK);
  }

  if(screen_index == HOME_SCREEN)
  {
    display_home_screen();
  }
  else if(screen_index == INFO_SCREEN)
  {
    display_info_screen();
  }
  else
  {
    display_config_screen(screen_index - 1);
  }
}

// --------------------------------------

void handle_input()
{
  int32_t current_position = input_rotary_encoder.read();

  if(current_position == last_encoder_position)
  {
    return;
  }

  if(current_position % 4 != 0)
  {
    return;
  }

  bool direction = current_position - last_encoder_position < 0;

  if(!edit_mode)
  {
    screen_index += direction ? 1 : -1;

    if(screen_index < 0)
    {
      screen_index = 7;
    }
    else if(screen_index > 7)
    {
      screen_index = 0;
    }

    showing_new_screen = true;
    screen_should_refresh = true;
  }
  else
  {
    menu_entry entry = menu_entries[screen_index -1];

    if(entry.is_float)
    {
      if(direction)
      {
        *(float*)entry.value += entry.increment;
      }
      else
      {
        *(float*)entry.value -= entry.increment;
      }

      if(*(float*)entry.value < entry.min)
      {
        *(float*)entry.value = entry.max;
      }
      else if(*(float*)entry.value > entry.max)
      {
        *(float*)entry.value = entry.min;
      }
    }
    else
    {
      if(direction)
      {
        *(int16_t*)entry.value += (int16_t)entry.increment;
      }
      else
      {
        *(int16_t*)entry.value -= (int16_t)entry.increment;
      }

      if(*(int16_t*)entry.value < (int16_t)entry.min)
      {
        *(int16_t*)entry.value = (int16_t)entry.max;
      }
      else if(*(int16_t*)entry.value > (int16_t)entry.max)
      {
        *(int16_t*)entry.value = (int16_t)entry.min;
      }
    }

    screen_should_refresh = true;
  }

  last_encoder_position = current_position;
}

void on_button_pressed()
{
  if(screen_index != HOME_SCREEN && screen_index != INFO_SCREEN)
  {
    if(millis() - edit_mode_toggled_at < 500)
    {
      return;
    }

    // if we are just leaving edit mode then save teh config and mark it as dirty
    if(edit_mode)
    {
      save_dirty_config();
      config_is_dirty = true;
    }

    edit_mode = !edit_mode;
    edit_mode_toggled_at = millis();

    screen_should_refresh = true;
  }
}

// --------------------------------------

uint8_t calculate_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

volatile void log_state()
{
  if(!logging_enabled)
  {
    return;
  }

  freezer_state_data_point data_point {};

//  if(rtc_clock.IsDateTimeValid())
//  {
//    data_point.unix_time = rtc_clock.GetDateTime().Unix32Time();
//  }

  data_point.ms_since_startup = millis();
  data_point.state = current_state;

  data_point.config = active_config;

  data_point.crc = calculate_crc8((uint8_t*)&data_point, sizeof(freezer_state_data_point));

  save_data_point(&data_point);
}

struct sd_count_sector
{
  uint32_t count;
  uint8_t crc;
};

bool save_data_point(const freezer_state_data_point* data)
{
  // we read the 0th and 1st sectors, both are used to save the current datapoint count
  // they should always be the same, if they are not we choose the one with the correct crc
  // if both are invalid, we choose 0th .. hopefully that never happens

  sd_count_sector count_sector_0 {};
  sd_count_sector count_sector_1 {};

  if (!read_card_block(0, (uint8_t*)&count_sector_0, sizeof(sd_count_sector)))
  {
    return false;
  }

  if (!read_card_block(1, (uint8_t*)&count_sector_1, sizeof(sd_count_sector)))
  {
    return false;
  }

  if(count_sector_0.count != count_sector_1.count)
  {
    if(count_sector_0.crc == calculate_crc8((uint8_t*)&count_sector_0.count, sizeof(uint32_t)))
    {
      count_sector_1.count = count_sector_0.count;
      count_sector_1.crc = count_sector_0.crc;
    }
    else if(count_sector_1.crc == calculate_crc8((uint8_t*)&count_sector_1.count, sizeof(uint32_t)))
    {
      count_sector_0.count = count_sector_1.count;
      count_sector_0.crc = count_sector_1.crc;
    }
    else
    {
      // both are invalid, we choose 0
      count_sector_1.count = count_sector_0.count;
      count_sector_1.crc = count_sector_0.crc;
    }

    if (!write_card_block(1, (uint8_t*)&count_sector_1, sizeof(sd_count_sector)))
    {
      return false;
    }
  }

  // increment the count and write it back
  count_sector_0.count++;
  count_sector_0.crc = calculate_crc8((uint8_t*)&count_sector_0.count, sizeof(uint32_t));

  // write it to both sectors
  if (!write_card_block(0, (uint8_t*)&count_sector_0, sizeof(sd_count_sector)))
  {
    return false;
  }

  if (!write_card_block(1, (uint8_t*)&count_sector_0, sizeof(sd_count_sector)))
  {
    return false;
  }

  // write the data at the correct sector ( current_count + 2 )
  if (!write_card_block(count_sector_0.count+2, data, sizeof(freezer_state_data_point)))
  {
    return false;
  }

//  // read the 4 bytes the first (0th) sector
//  uint32_t current_count = 0;
//  if (!read_card_block(0, (uint8_t*)&current_count, 4))
//  {
//    return false;
//  }
//
//
//  // increment the count and write it back
//  current_count++;
//
//  if (!write_card_block(0, (uint8_t*)&current_count, 4))
//  {
//    return false;
//  }

//  // write the data at the correct sector ( current_count + 1 )
//  if (!write_card_block(current_count+1, data, sizeof(freezer_state_data_point)))
//  {
//    return false;
//  }

  return true;
}

// ---------------

bool init_sd_card()
{
  SPI.begin();
  SPISettings sdSpiSettings(250000, MSBFIRST, SPI_MODE0);

  pinMode(MICRO_SD_CS, OUTPUT);
  digitalWrite(MICRO_SD_CS, HIGH);

  delay(300);

  // at least 74 clock cycles after power up
  SPI.beginTransaction(sdSpiSettings);
  for (uint8_t i = 0; i < 10; i++)
  {
    SPI.transfer(0xFF);
  }
  SPI.endTransaction();

  digitalWrite(MICRO_SD_CS, LOW);

  // Reset the card ( CMD0 )
  send_card_command(0, 0, 0x95); // 0x95 is the pre-calculated crc
  if (read_card_response() != 0x01)
  {
    digitalWrite(MICRO_SD_CS, HIGH);

    // Serial.println("init_sd_card() : invalid response after sending CMD0");
    return false;
  }

  // Check the voltage range ( CMD8 )
  // or more accurately, make sure this is a v2 card
  // we don't really care about the voltage rangee
  send_card_command(8, 0x01AA, 0x87);
  if (read_card_response() != 0x01)
  {
    digitalWrite(MICRO_SD_CS, HIGH);

    // Serial.println("init_sd_card() : invalid response after sending CMD0");

    return false;
  }

  uint8_t retries = 3;
  for (uint8_t i = 0; i < retries; i++)
  {
    // inform the card that the next command is an ACMD
    send_card_command(55, 0, 0x01);
    if (read_card_response() != 0x01)
    {
      digitalWrite(MICRO_SD_CS, HIGH);

      // Serial.println("init_sd_card() : invalid response after sending CMD55");
      return false;
    }

    // Initializes the SD card in high-capacity mode (> 2GB)
    send_card_command(41, 0x40000000, 0x01);

    // if we receive 0x00 that means the initialization was successful,
    // otherwise we keep retrying
    if (read_card_response() == 0x00)
    {
      break;
    }
  }

  digitalWrite(MICRO_SD_CS, HIGH);

  // write an extra dunmmy byte after pulling the cs line high
  // this might be needed if you have other devices on the same
  // spi bus as the sd card.
  SPI.transfer(0xFF);

  return true;
}

void send_card_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  SPISettings sdSpiSettings(250000, MSBFIRST, SPI_MODE0);

  wait_card_busy();

  // SD Card Command Frame: [Command | Argument | CRC] (48 bits)
  // Command  => | start bit (always 0) | command bit (always 1) | 6 bits command index |
  // Argument => | 32 bit argument |
  // CRC      => | CRC7 (7 bits) | end bit (always 1) |

  SPI.beginTransaction(sdSpiSettings);

  SPI.transfer(0x40 | cmd); // 0x40 sets the command bit to 1

  SPI.transfer(arg >> 24);
  SPI.transfer(arg >> 16);
  SPI.transfer(arg >> 8);
  SPI.transfer(arg);

  SPI.transfer(crc);

  SPI.endTransaction();
}

uint8_t read_card_response()
{
  SPISettings sdSpiSettings(250000, MSBFIRST, SPI_MODE0);

  SPI.beginTransaction(sdSpiSettings);

  // keep receiving bytes until we receive something that isn't 0xFF
  for (uint8_t i = 0; i < 10; i++)
  {
    uint8_t response = SPI.transfer(0xFF);
    if (response != 0xFF)
    {
      SPI.endTransaction();

      return response;
    }
  }

  SPI.endTransaction();
  return 0xFF; // no response, something is probably wrong
}

bool wait_card_busy()
{
  auto start_time = millis();

  // wait until the card pulls the DO high ( we receive a 0xFF )
  // or until we time out after 300ms, I don't have a particular reason
  // for choosing 300ms as a timeout value other than that it was the
  // value used in the SDFat library.

  for(;;)
  {
    auto r_byte = SPI.transfer(0xFF);

    if(r_byte == 0xFF)
    {
      return true;
    }

    if(millis() - start_time > 300)
    {
      return false;
    }
  }
}

bool read_card_block(uint32_t block_addr, uint8_t* buffer, uint16_t buffer_size)
{
  SPISettings sdSpiSettings(250000, MSBFIRST, SPI_MODE0);

  digitalWrite(MICRO_SD_CS, LOW);

  // send CMD17 ( read block ) with the block address as the parameter
  send_card_command(17, block_addr, 0x01);
  if (read_card_response() != 0x00)
  {
    digitalWrite(MICRO_SD_CS, HIGH);
    return false;
  }

  SPI.beginTransaction(sdSpiSettings);

  // we keep receiving bytes until we get the 'data token' ( 0xFE )
  while (SPI.transfer(0xFF) != 0xFE);

  // we need to read the entire 512 byte block regardless of if we need it or not
  // we are ignoring the bytes read once we have the buffer_size amount of bytes
  for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
  {
    uint8_t r_byte = SPI.transfer(0xFF);

    if (i < buffer_size)
    {
      buffer[i] = r_byte;
    }
  }

  // we aren't doing any CRC validation so we're ignoring the response
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);

  SPI.endTransaction();

  digitalWrite(MICRO_SD_CS, HIGH);

  // write an extra dunmmy byte after pulling the cs line high
  // this might be needed if you have other devices on the same
  // spi bus as the sd card.
  SPI.transfer(0xFF);

  return true;
}

bool write_card_block(uint32_t block_addr, const void* buffer, uint16_t buffer_size)
{
  SPISettings sdSpiSettings(250000, MSBFIRST, SPI_MODE0);

  digitalWrite(MICRO_SD_CS, LOW);

  // send CMD14 (block write)
  send_card_command(24, block_addr, 0x01);
  if (read_card_response() != 0x00)
  {
    digitalWrite(MICRO_SD_CS, HIGH);
    return false;
  }

  SPI.beginTransaction(sdSpiSettings);

  // Send the data start token
  SPI.transfer(0xFE);

  // we must write the entire 512 byte block
  // we fill the rest with zeros
  for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
  {
    if (i < buffer_size)
    {
      SPI.transfer(((uint8_t*)buffer)[i]);
    }
    else
    {
      SPI.transfer(0x00);
    }
  }

  // just a dummy crc
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);

  uint8_t response = SPI.transfer(0xFF);

  // check if the data was accepted
  if ((response & 0x1F) != 0x05)
  {
    SPI.endTransaction();
    digitalWrite(MICRO_SD_CS, HIGH);
    return false;
  }

  // wait until the card is ready
  while (SPI.transfer(0xFF) == 0x00);

  SPI.endTransaction();

  digitalWrite(MICRO_SD_CS, HIGH);

  // write an extra dunmmy byte after pulling the cs line high
  // this might be needed if you have other devices on the same
  // spi bus as the sd card.
  SPI.transfer(0xFF);

  return true;
}

// --------------------------------------
