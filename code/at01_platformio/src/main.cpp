#include <lvgl.h>
#include <TFT_eSPI.h>
//#include <examples/lv_examples.h>
//#include <demos/lv_demos.h>

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   240
#define TFT_VER_RES   320
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

#define UP_BUTTON 42
#define DOWN_BUTTON 41
#define SOLENOID_PIN  37
#define MOTOR_PIN  39
#define LED1_PIN -1
#define LED2_PIN -1
#define SLEEP_PIN 35
#define BL_PIN 9

void venepunctureScreen();
void occlusionScreen();
void bicepOcclusionScreen();
void thighOcclusionScreen();

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

static void up_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    venepunctureScreen();
  }
}

static void down_pressed(lv_event_t * event)
{
  //Original Code
  lv_event_code_t code = lv_event_get_code(event);

  if(code == LV_EVENT_PRESSED) 
  {
    Serial.println("Down pressed");
  }
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

void homeScreen(){
  lv_obj_t *screen = lv_obj_create(lv_screen_active());
  lv_obj_set_size(screen, TFT_HOR_RES, TFT_VER_RES);
  lv_obj_center(screen);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  LV_IMAGE_DECLARE(arrow_up);
  lv_obj_t * up = lv_imagebutton_create(screen);
  lv_imagebutton_set_src(up, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &arrow_up, NULL);
  lv_obj_align(up, LV_ALIGN_CENTER, -50, -30);
  lv_obj_add_event_cb(up, up_pressed, LV_EVENT_ALL, NULL);

  LV_IMAGE_DECLARE(arrow_down);
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

void venepunctureScreen(){
  lv_obj_t *screen = lv_obj_create(lv_screen_active());
  lv_obj_set_size(screen, TFT_HOR_RES, TFT_VER_RES);
  lv_obj_center(screen);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *menu_label = lv_label_create( screen );
  lv_label_set_text( menu_label, "Venepuncture" );
  lv_obj_align( menu_label, LV_ALIGN_TOP_MID, 0, 0);

  static lv_style_t menu_style;
  lv_style_init(&menu_style);
  lv_style_set_text_font(&menu_style, &lv_font_montserrat_24); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(menu_label, &menu_style, 0);  // <--- obj is the label

  lv_obj_t *vene_label = lv_label_create( screen );
  lv_label_set_text( vene_label, "Target Pressure: 60 mmHg" );
  lv_obj_align( vene_label, LV_ALIGN_CENTER, 0, 100);

  LV_IMAGE_DECLARE(img_hand);

  lv_obj_t * needle_line;
  lv_obj_t * needle_img;
  
  lv_obj_t * scale_img = lv_scale_create(lv_screen_active());

  lv_obj_set_size(scale_img, 150, 150);
  lv_scale_set_mode(scale_img, LV_SCALE_MODE_ROUND_INNER);
  lv_obj_set_style_bg_opa(scale_img, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(scale_img, lv_palette_lighten(LV_PALETTE_RED, 5), 0);
  lv_obj_set_style_radius(scale_img, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_clip_corner(scale_img, true, 0);
  lv_obj_align(scale_img, LV_ALIGN_CENTER, 0, 0);

  lv_scale_set_label_show(scale_img, true);

  lv_scale_set_total_tick_count(scale_img, 31);
  lv_scale_set_major_tick_every(scale_img, 5);

  lv_obj_set_style_length(scale_img, 5, LV_PART_ITEMS);
  lv_obj_set_style_length(scale_img, 10, LV_PART_INDICATOR);
  lv_scale_set_range(scale_img, 0, 60);

  lv_scale_set_angle_range(scale_img, 270);
  lv_scale_set_rotation(scale_img, 135);

  /* image must point to the right. E.g. -O------>*/
  needle_img = lv_img_create(scale_img);
  lv_image_set_src(needle_img, &img_hand);
  lv_obj_align(needle_img, LV_ALIGN_CENTER, 47, -2);
  lv_image_set_pivot(needle_img, 3, 4);

  static lv_style_t vene_style;
  lv_style_init(&vene_style);
  lv_style_set_text_font(&vene_style, &lv_font_montserrat_14); // <--- you have to enable other font sizes in menuconfig
  lv_obj_add_style(vene_label, &vene_style, 0);  // <--- obj is the label
  // lv_obj_add_style(occlusion_label, &vene_style, 0);
}

void setup()
{
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);

  Serial.begin( 115200 );
  Serial.println( LVGL_Arduino );

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

  homeScreen();

  Serial.println( "Setup done" );
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(5); /* let this time pass */
}
