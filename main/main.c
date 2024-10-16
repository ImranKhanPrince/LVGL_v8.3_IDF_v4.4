#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <lvgl.h>
#include "lvgl_helpers.h"

#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include <esp_log.h>
#include "esp_err.h"

#define LED_BACKLIT_PIN GPIO_NUM_4
#define TAG "Main.c"
static lv_disp_t *lv_display = NULL;
static lv_color_t *lv_buf_1 = NULL;
static lv_color_t *lv_buf_2 = NULL;
const size_t LV_BUFFER_SIZE = LV_HOR_RES_MAX * 25; // Default to 25 rows chunk of frame

/* gui task parameters */
TaskHandle_t GUI_TASK_HANDLE = NULL;
#define GUI_TASK_STACK_SIZE 5 * 1024
#define GUI_TASK_PRIORITY 10
#define GUI_TASK_CORE 1

#define LV_TICK_PERIOD_MS 10

// dunno why
esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
esp_lcd_panel_handle_t lcd_handle = NULL;

// static lv_disp_draw_buf_t lv_disp_buf;
lv_disp_drv_t disp_drv;

void dispBacklitEn()
{
  /* Config tft display backlight signal */
  ESP_LOGI(TAG, "Configuring IO for  LCD Backlit PWM");

  configTftBacklit(1024);
  // Initialize fade service
  ESP_ERROR_CHECK(ledc_fade_func_install(0));

  ESP_LOGI(TAG, "PWM signal started on GPIO4");
}

void configTftBacklit(uint32_t duty_c)
{
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,   // High speed mode
      .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit duty resolution
      .timer_num = LEDC_TIMER_0,            // Timer 0
      .freq_hz = 2000,                      // Frequency of PWM signal 2khz
      .clk_cfg = LEDC_AUTO_CLK              // Auto select the source clock
  };
  // Set configuration of timer0 for high speed channels
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {
      .gpio_num = LED_BACKLIT_PIN, // Set GPIO4 as PWM signal output
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = duty_c, // Set duty cycle to 50%
      .hpoint = 0};
  // Set LEDC channel configuration
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void event_handler(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *btn = lv_event_get_target(e);

  if (code == LV_EVENT_CLICKED)
  {
    printf("Button clicked!\n");
  }
}

void draw_label()
{
  lv_disp_t *dispp = lv_disp_get_default();
  // Get the active screen (the default parent object)
  lv_obj_t *scr = lv_scr_act();

  lv_obj_t *btn = lv_btn_create(scr);

  // Set the size and position of the button
  lv_obj_set_size(btn, 120, 50);            // Width: 120, Height: 50
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0); // Center the button

  // Attach an event handler to the button
  lv_obj_add_event_cb(btn, event_handler, LV_EVENT_ALL, NULL);

  // Create a label on the button
  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, "Click Me");
  lv_obj_center(label); // Center the label within the button
}

static void lv_tick_task(void *arg)
{
  lv_tick_inc(LV_TICK_PERIOD_MS); // 10ms
}

static void ui_task(void *pvParameter)
{
  dispBacklitEn();
  lv_init();
  lvgl_driver_init();
  /*----OUTPUT DEVICE-----*/
  // Allocate buffer single or double for faster processing - requires "esp_heap_caps.h"
  lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

  // define[could be glbal] and initialize the buffer
  lv_disp_draw_buf_t lv_disp_buf;
  lv_disp_draw_buf_init(&lv_disp_buf, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE);

  // define[could be global] and Initialize the display driver (struct)
  // lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  // disp_drv = {
  //     .hor_res = LV_HOR_RES,
  //     .ver_res = LV_VER_RES,
  //     .flush_cb = disp_driver_flush,
  //     .draw_buf = &lv_disp_buf,
  //     .user_data = lcd_handle,
  // };

  // define the resolution
  disp_drv.hor_res = LV_HOR_RES_MAX;
  disp_drv.ver_res = LV_VER_RES_MAX;

  // Register the flush callback for lvgl to access
  disp_drv.flush_cb = disp_driver_flush; // this is configured to call ili9488 flush

  // Register the buffer pointer for lvgl to access
  disp_drv.draw_buf = &lv_disp_buf;

  // dunno why
  disp_drv.user_data = lcd_handle;

  // Register the display driver struct
  lv_display = lv_disp_drv_register(&disp_drv);

  // initialize the input drivers
  lv_indev_drv_t my_indev_drv;
  lv_indev_drv_init(&my_indev_drv);
  my_indev_drv.read_cb = touch_driver_read;  // Set the read callback function
  my_indev_drv.type = LV_INDEV_TYPE_POINTER; // Set the input device type (e.g., pointer)
  lv_indev_drv_register(&my_indev_drv);      // Register the input device driver

  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &lv_tick_task,
      .name = "periodic_gui"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

  draw_label();

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(LV_TICK_PERIOD_MS));
    // take this semaphore to call lvgl related function on success
    lv_task_handler();
  }
  vTaskDelete(NULL);
}

void app_main(void)
{

  xTaskCreatePinnedToCore(ui_task, "gui", GUI_TASK_STACK_SIZE, NULL, GUI_TASK_PRIORITY, &GUI_TASK_HANDLE, GUI_TASK_CORE);
  /*INPUT DEVICE*/
}
