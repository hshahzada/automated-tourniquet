#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_MPRLS.h"
#include <PID_v1.h>
//#include <examples/lv_examples.h>
//#include <demos/lv_demos.h>

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   240
#define TFT_VER_RES   320
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

static uint32_t lvgl_refresh_timestamp = 0u;
#define LVGL_REFRESH_TIME 5u

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define UP_BUTTON 42
#define DOWN_BUTTON 41
#define SOLENOID_PIN  37
#define MOTOR_PIN  39
#define LED1_PIN -1
#define LED2_PIN -1
#define SLEEP_PIN 35
#define BL_PIN 9
#define STATUS_LED 19
#define POWER_LED 20

const float VENEPUNCTURE_PRESSURE = 60;
const float BICEP_PRESSURE = 203;
const float THIGH_PRESSURE = 300;
float startPressure;
float currentPressure;
float targetPressure;

const unsigned long VENEPUNCTURE_TIME = 60000;
const unsigned long BICEP_TIME = 5400000;
const unsigned long THIGH_TIME = 7200000;
unsigned long startTime = 0;
// unsigned long currentTime = 0;
unsigned long targetTime = 0;

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// PID Setup
double PIDPressure;
double Input, Output;
float calibration_value;
double Kp = 3, Ki = 0.05, Kd = 1; // Example PID tuning values, adjust as needed
PID myPID(&Input, &Output, &PIDPressure, Kp, Ki, Kd, DIRECT);

void actuationScreen(const char *mode);
void occlusionScreen();
void bicepOcclusionScreen();
void thighOcclusionScreen();

LV_IMAGE_DECLARE(arrow_up);
LV_IMAGE_DECLARE(arrow_down);

lv_obj_t *pressure_val;
lv_obj_t *vene_label;
lv_obj_t * bar;
lv_timer_t * bar_timer;
int bar_value;
char test2[6];
bool timerOn = false;
int pressureTime;
bool timeStarted = false;

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
  /*Call it to tell LVGL you are ready*/
  lv_display_flush_ready(disp);
}

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void)
{
  return millis();
}

static void occlusion_up_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    targetPressure = BICEP_PRESSURE;
    bar_value = (int)BICEP_TIME/1000;
    targetTime = BICEP_TIME;
    actuationScreen("Bicep Occlusion");
  }
}

static void occlusion_down_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    targetPressure = THIGH_PRESSURE;
    bar_value = (int)THIGH_TIME/1000;
    targetTime = THIGH_TIME;
    actuationScreen("Thigh Occlusion");
  }
}

static void up_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    targetPressure = VENEPUNCTURE_PRESSURE;
    bar_value = (int)VENEPUNCTURE_TIME/1000;
    targetTime = VENEPUNCTURE_TIME;
    actuationScreen("Venepuncture");
  }
}

static void down_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    occlusionScreen();
  }
}

static void timer_labelupdate(lv_timer_t* timer){
  sprintf(test2, "%d", (int)currentPressure);
  lv_label_set_text(pressure_val, test2);
}

static void timer_barupdate(lv_timer_t* timer){
  bar_value--;
}

static void timer_pressureupdate(lv_timer_t* timer){
  currentPressure = (mpr.readPressure() - calibration_value) * 0.75;

  PIDPressure = targetPressure;
  bool buttonUp = digitalRead(UP_BUTTON);
  bool buttonDown = digitalRead(DOWN_BUTTON);
  if (buttonUp == HIGH) {
    targetPressure = targetPressure + 1;
  } else if (buttonDown == HIGH) {
    targetPressure = targetPressure - 1;
  }

  char pressure_target[21]; 
  sprintf(pressure_target, "%d", (int)targetPressure);
  String target_pressure_label = (String)"Target Pressure: " + pressure_target + (String)" mmHg";
  char arr[target_pressure_label.length() + 1]; 
	strcpy(arr, target_pressure_label.c_str());
  lv_label_set_text(vene_label, arr);

  Input = currentPressure;
  myPID.Compute();
  
  if (timerOn == false) {
    analogWrite(MOTOR_PIN, Output);
  }
  
  if (currentPressure >= targetPressure-3 && timeStarted == false){
    startTime = millis();
    timeStarted = true;
  } else if (currentPressure < targetPressure-3){
    startTime = 0;
    timeStarted = false;
  }
  
  if (millis()-startTime > 5000 && timeStarted == true){
    lv_timer_resume(bar_timer);
    pressureTime = millis();
    analogWrite(MOTOR_PIN, 0);
    timerOn = true;
  }

  if (millis()-pressureTime >= targetTime && timerOn){
    lv_timer_pause(bar_timer);
    timerOn = false;
  }

  if (currentPressure > targetPressure+10){
    digitalWrite(SOLENOID_PIN, HIGH);
  } else {
    digitalWrite(SOLENOID_PIN, LOW);
  }
  
  Serial.println(timerOn);
}

bool buttonPressed(int button) {
  return digitalRead(button) == HIGH && digitalRead(button) == HIGH;
}

bool buttonNotPressed(int button) {
  return digitalRead(button) == LOW && digitalRead(button) == LOW;
}

int my_btn_read(){
  if(buttonPressed(UP_BUTTON) && buttonNotPressed(DOWN_BUTTON)){
    return 0;
  }
  if(buttonPressed(DOWN_BUTTON) && buttonNotPressed(UP_BUTTON)){
    return 1;
  } else {
    return -1;
  }
}

void button_read( lv_indev_t * indev, lv_indev_data_t * data ){
  static uint32_t last_btn = 0;   /*Store the last pressed button*/
  int btn_pr = my_btn_read();     /*Get the ID (0,1,2...) of the pressed button*/
  if(btn_pr >= 0) {               /*Is there a button press? (E.g. -1 indicated no button was pressed)*/
      last_btn = btn_pr;           /*Save the ID of the pressed button*/
      data->state = LV_INDEV_STATE_PRESSED;  /*Set the pressed state*/
  } else {
      data->state = LV_INDEV_STATE_RELEASED; /*Set the released state*/
  }

  data->btn_id = last_btn;         /*Save the last button*/
}

static void set_arc_angle(void * obj, int32_t v)
{
  lv_arc_set_value((lv_obj_t*) obj, currentPressure);
}

static void set_bar_value(void * bar, int32_t v)
{
  lv_bar_set_value((lv_obj_t*) bar, bar_value, LV_ANIM_OFF);
}

void occlusionScreen(){
  lv_obj_t *screen = lv_obj_create(lv_screen_active());
  lv_obj_set_size(screen, TFT_HOR_RES, TFT_VER_RES);
  lv_obj_center(screen);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * up = lv_imagebutton_create(screen);
  lv_imagebutton_set_src(up, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &arrow_up, NULL);
  lv_obj_align(up, LV_ALIGN_CENTER, -50, -30);
  lv_obj_add_event_cb(up, occlusion_up_pressed, LV_EVENT_ALL, NULL);

  lv_obj_t * down = lv_imagebutton_create(screen);
  lv_imagebutton_set_src(down, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &arrow_down, NULL);
  lv_obj_align(down, LV_ALIGN_BOTTOM_MID, -50, -20);
  lv_obj_add_event_cb(down, occlusion_down_pressed, LV_EVENT_ALL, NULL);

  lv_obj_t *menu_label = lv_label_create( screen );
  lv_label_set_text( menu_label, "Occlusion" );
  lv_obj_align( menu_label, LV_ALIGN_TOP_MID, -3, 0);

  static lv_style_t menu_style;
  lv_style_init(&menu_style);
  lv_style_set_text_font(&menu_style, &lv_font_montserrat_20); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(menu_label, &menu_style, 0);  // <--- obj is the label

  lv_obj_t *choose_label = lv_label_create( screen );
  lv_label_set_text( choose_label, "Please choose an");
  lv_obj_align( choose_label, LV_ALIGN_TOP_MID, -3, 30);
  lv_label_set_long_mode(choose_label, LV_LABEL_LONG_WRAP);

  lv_obj_t *device_label = lv_label_create( screen );
  lv_label_set_text( device_label, "occlusion mode");
  lv_obj_align( device_label, LV_ALIGN_TOP_MID, -3, 45);
  lv_label_set_long_mode(device_label, LV_LABEL_LONG_WRAP);

  static lv_style_t choose_style;
  lv_style_init(&choose_style);
  lv_style_set_text_font(&choose_style, &lv_font_montserrat_16); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(choose_label, &choose_style, 0);  // <--- obj is the label
  lv_obj_add_style(device_label, &choose_style, 0);  // <--- obj is the label

  lv_obj_t *vene_label = lv_label_create( screen );
  lv_label_set_text( vene_label, "Bicep Occlusion" );
  lv_obj_align( vene_label, LV_ALIGN_CENTER, 50, -35);

  lv_obj_t *occlusion_label = lv_label_create( screen );
  lv_label_set_text( occlusion_label, "Thigh Occlusion" );
  lv_obj_align( occlusion_label, LV_ALIGN_BOTTOM_MID, 50, -55);
  
  lv_obj_t *select_label = lv_label_create( screen );
  lv_label_set_text( select_label, "Select with up/down arrows" );
  lv_obj_align( select_label, LV_ALIGN_BOTTOM_MID, 0, 3);

  lv_obj_t *battery_label = lv_label_create( screen );
  lv_label_set_text( battery_label, "73% " LV_SYMBOL_BATTERY_3);
  lv_obj_align( battery_label, LV_ALIGN_TOP_RIGHT, 3, 0);

  lv_obj_t *time_label = lv_label_create( screen );
  lv_label_set_text( time_label, "13:26");
  lv_obj_align( time_label, LV_ALIGN_TOP_LEFT, 0, 0);

  static lv_style_t vene_style;
  lv_style_init(&vene_style);
  lv_style_set_text_font(&vene_style, &lv_font_montserrat_14); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(vene_label, &vene_style, 0);  // <--- obj is the label
  lv_obj_add_style(occlusion_label, &vene_style, 0);
  lv_obj_add_style(select_label, &vene_style, 0);

  static lv_style_t time_style;
  lv_style_init(&time_style);
  lv_style_set_text_font(&time_style, &lv_font_montserrat_12); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(battery_label, &time_style, 0);  // <--- obj is the label
  lv_obj_add_style(time_label, &time_style, 0);
}

void homeScreen(){
  lv_obj_t *screen = lv_obj_create(lv_screen_active());
  lv_obj_set_size(screen, TFT_HOR_RES, TFT_VER_RES);
  lv_obj_center(screen);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t * up = lv_imagebutton_create(screen);
  lv_imagebutton_set_src(up, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &arrow_up, NULL);
  lv_obj_align(up, LV_ALIGN_CENTER, -50, -30);
  lv_obj_add_event_cb(up, up_pressed, LV_EVENT_ALL, NULL);

  lv_obj_t * down = lv_imagebutton_create(screen);
  lv_imagebutton_set_src(down, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &arrow_down, NULL);
  lv_obj_align(down, LV_ALIGN_BOTTOM_MID, -50, -20);
  lv_obj_add_event_cb(down, down_pressed, LV_EVENT_ALL, NULL);

  lv_obj_t *menu_label = lv_label_create( screen );
  lv_label_set_text( menu_label, "Main Menu" );
  lv_obj_align( menu_label, LV_ALIGN_TOP_MID, -3, 0);

  static lv_style_t menu_style;
  lv_style_init(&menu_style);
  lv_style_set_text_font(&menu_style, &lv_font_montserrat_24); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(menu_label, &menu_style, 0);  // <--- obj is the label

  lv_obj_t *choose_label = lv_label_create( screen );
  lv_label_set_text( choose_label, "Please choose a");
  lv_obj_align( choose_label, LV_ALIGN_TOP_MID, -3, 30);
  lv_label_set_long_mode(choose_label, LV_LABEL_LONG_WRAP);

  lv_obj_t *device_label = lv_label_create( screen );
  lv_label_set_text( device_label, "device mode");
  lv_obj_align( device_label, LV_ALIGN_TOP_MID, -3, 45);
  lv_label_set_long_mode(device_label, LV_LABEL_LONG_WRAP);

  static lv_style_t choose_style;
  lv_style_init(&choose_style);
  lv_style_set_text_font(&choose_style, &lv_font_montserrat_16); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(choose_label, &choose_style, 0);  // <--- obj is the label
  lv_obj_add_style(device_label, &choose_style, 0);  // <--- obj is the label

  lv_obj_t *vene_label = lv_label_create( screen );
  lv_label_set_text( vene_label, "Venepuncture" );
  lv_obj_align( vene_label, LV_ALIGN_CENTER, 50, -35);

  lv_obj_t *occlusion_label = lv_label_create( screen );
  lv_label_set_text( occlusion_label, "Occlusion" );
  lv_obj_align( occlusion_label, LV_ALIGN_BOTTOM_MID, 40, -70);
  
  lv_obj_t *select_label = lv_label_create( screen );
  lv_label_set_text( select_label, "Select with up/down arrows" );
  lv_obj_align( select_label, LV_ALIGN_BOTTOM_MID, 0, 3);

  lv_obj_t *battery_label = lv_label_create( screen );
  lv_label_set_text( battery_label, "73% " LV_SYMBOL_BATTERY_3);
  lv_obj_align( battery_label, LV_ALIGN_TOP_RIGHT, 3, 0);

  lv_obj_t *time_label = lv_label_create( screen );
  lv_label_set_text( time_label, "13:26");
  lv_obj_align( time_label, LV_ALIGN_TOP_LEFT, 0, 0);

  static lv_style_t vene_style;
  lv_style_init(&vene_style);
  lv_style_set_text_font(&vene_style, &lv_font_montserrat_14); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(vene_label, &vene_style, 0);  // <--- obj is the label
  lv_obj_add_style(occlusion_label, &vene_style, 0);
  lv_obj_add_style(select_label, &vene_style, 0);

  static lv_style_t time_style;
  lv_style_init(&time_style);
  lv_style_set_text_font(&time_style, &lv_font_montserrat_12); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(battery_label, &time_style, 0);  // <--- obj is the label
  lv_obj_add_style(time_label, &time_style, 0);
}

void actuationScreen(const char *mode){
  lv_timer_t * pressure_timer = lv_timer_create(timer_pressureupdate, 50, NULL);
  lv_timer_ready(pressure_timer);

  lv_obj_t *screen = lv_obj_create(lv_screen_active());
  lv_obj_set_size(screen, TFT_HOR_RES, TFT_VER_RES);
  lv_obj_center(screen);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *menu_label = lv_label_create( screen );
  lv_label_set_text( menu_label, mode );
  lv_obj_align( menu_label, LV_ALIGN_TOP_MID, 0, 0);

  static lv_style_t menu_style;
  lv_style_init(&menu_style);
  lv_style_set_text_font(&menu_style, &lv_font_montserrat_24); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(menu_label, &menu_style, 0);  // <--- obj is the label

  vene_label = lv_label_create( screen );
  lv_label_set_text( vene_label, "");
  lv_obj_align( vene_label, LV_ALIGN_CENTER, 0, 120);

  lv_obj_t * arc = lv_arc_create(screen);
  lv_arc_set_rotation(arc, 270);
  lv_arc_set_bg_angles(arc, 0, 360);
  lv_arc_set_range(arc, 0, targetPressure);
  lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
  lv_obj_align(arc, LV_ALIGN_CENTER, 0, -20);

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, arc);
  lv_anim_set_exec_cb(&a, set_arc_angle);
  lv_anim_set_duration(&a, 1000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_repeat_delay(&a, 500);
  lv_anim_start(&a);

  pressure_val = lv_label_create( screen );
  lv_label_set_text(pressure_val, "");
  lv_obj_align(pressure_val, LV_ALIGN_CENTER, 0, -20);
  
  static lv_style_t pressure_label_style;
  lv_style_init(&pressure_label_style);
  lv_style_set_text_font(&pressure_label_style, &lv_font_montserrat_24); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(pressure_val, &pressure_label_style, 0);  // <--- obj is the label

  bar = lv_bar_create(lv_screen_active()); 
  lv_obj_set_size(bar, 200, 20);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -70);
  lv_bar_set_value(bar, 100, LV_ANIM_OFF);
  lv_bar_set_range(bar, 0, bar_value);
  // lv_obj_add_event_cb(bar, event_cb, LV_EVENT_DRAW_MAIN_END, NULL);

  lv_anim_t bar_anim;
  lv_anim_init(&bar_anim);
  lv_anim_set_var(&bar_anim, bar);
  lv_anim_set_values(&bar_anim, 0, bar_value);
  lv_anim_set_exec_cb(&bar_anim, set_bar_value);
  lv_anim_set_duration(&bar_anim, 4000);
  lv_anim_set_playback_duration(&bar_anim, 4000);
  lv_anim_set_repeat_count(&bar_anim, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&bar_anim);

  lv_timer_t * label_timer = lv_timer_create(timer_labelupdate, 500, NULL);
  lv_timer_ready(label_timer);

  bar_timer = lv_timer_create(timer_barupdate, 1000,  NULL);
  lv_timer_pause(bar_timer);
}

void setup()
{
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(POWER_LED, OUTPUT);

  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BL_PIN, HIGH);
  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(POWER_LED, HIGH);

  Serial.begin(115200);
  Wire.begin(1,2);
  Serial.println(LVGL_Arduino);

  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
        digitalWrite(STATUS_LED, HIGH);
        delay(10);
    }
  }

  calibration_value = mpr.readPressure();

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range for motor control

  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick);

  lv_display_t * disp;
  /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
  disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, TFT_ROTATION);

  lv_indev_t * button_indev = lv_indev_create();
  lv_indev_set_type(button_indev, LV_INDEV_TYPE_BUTTON);
  lv_indev_set_read_cb(button_indev, button_read);
  static lv_point_t button_array[2] = { { 70, 130 }, {70, 280} };
  lv_indev_set_button_points(button_indev, button_array);

  Serial.println( "Setup done" );

  lvgl_refresh_timestamp = millis();

  homeScreen();
}

static void LVGL_TaskMng( void )
{
  uint32_t now = millis();
  // LVGL Refresh Timed Task
  if( (now - lvgl_refresh_timestamp) >= LVGL_REFRESH_TIME )
  {
    lvgl_refresh_timestamp = now;
    // let the GUI does work
    lv_timer_handler();
  }
}

void loop()
{
  LVGL_TaskMng(); /* let the GUI do its work */
}
