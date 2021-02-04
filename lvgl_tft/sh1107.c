/**
 * @file sh1107.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "sh1107.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_i2c_conf.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "SH1107"




#if CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT
#define DIEDIEDIE abort()
#else
#warning "Consider changing system panic behavior to CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT."
#define DIEDIEDIE while (1)
#endif

#define OK_OR_RETURN(x, ret)                                                                  \
  do {                                                                                        \
    const esp_err_t err = (x);                                                                \
    if (err != ESP_OK) {                                                                      \
      ESP_LOGE("", "%s:%d TRY(" #x ") fail => %s", __FILE__, __LINE__, esp_err_to_name(err)); \
      return ret;                                                                             \
    }                                                                                         \
  } while (0)

#define TRY(x) OK_OR_RETURN(x, err)

#define CHECK(x)                                                    \
  do {                                                              \
    if (!(x)) {                                                     \
      ESP_LOGE("", "%s:%d CHECK(" #x ") fail", __FILE__, __LINE__); \
      DIEDIEDIE;                                                    \
    }                                                               \
  } while (0)

#define CHECK_OK(x)                                                                              \
  do {                                                                                           \
    const esp_err_t result = (x);                                                                \
    if (result != ESP_OK) {                                                                      \
      ESP_LOGE(                                                                                  \
          "", "%s:%d CHECK_OK(" #x ") fail => %s", __FILE__, __LINE__, esp_err_to_name(result)); \
      DIEDIEDIE;                                                                                 \
    }                                                                                            \
  } while (0)



/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void sh1107_send_cmd(uint8_t cmd);
static esp_err_t sh1107_send_cmds_i2c(uint8_t* cmd, uint16_t length);
static esp_err_t sh1107_send_data(void * data, uint16_t length);
static esp_err_t sh1107_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void sh1107_init(void)
{
    // Use Double Bytes Commands if necessary, but not Command+Data
    // Initialization taken from https://github.com/nopnop2002/esp-idf-m5stick
    lcd_init_cmd_t init_cmds[]={
        {0xAE, {0}, 0},	// Turn display off
        {0xDC, {0}, 0},	// Set display start line
        {0x00, {0}, 0},	// ...value
        {0x81, {0}, 0},	// Set display contrast
        {0x2F, {0}, 0},	// ...value
        {0x20, {0}, 0},	// Set memory mode
        {0xA0, {0}, 0},	// Non-rotated display  
#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE			
        {0xC0, {0}, 0},	// flipped vertical
#elif defined CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT
        {0xC0, {0}, 0},	// flipped vertical
#endif
// NOTE: value from 7F to 3F
        {0xA8, {0}, 0},	// Set multiplex ratio
        {0x3F, {0}, 0},	// ...value        
        {0xD3, {0}, 0},	// Set display offset to zero
        {0x60, {0}, 0},	// ...value
        {0xD5, {0}, 0},	// Set display clock divider
        {0x51, {0}, 0},	// ...value
        {0xD9, {0}, 0},	// Set pre-charge
        {0x22, {0}, 0},	// ...value
        {0xDB, {0}, 0},	// Set com detect
        {0x35, {0}, 0},	// ...value
        {0xB0, {0}, 0},	// Set page address
        {0xDA, {0}, 0},	// Set com pins
        {0x12, {0}, 0},	// ...value
        {0xA4, {0}, 0},	// output ram to display
#if defined CONFIG_LV_INVERT_DISPLAY
        {0xA7, {0}, 0},	// inverted display
#else
        {0xA6, {0}, 0},	// Non-inverted display
#endif 
        {0xAF, {0}, 0},	// Turn display on
        {0, {0}, 0xff},
    };

#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    //Initialize non-SPI GPIOs
    gpio_pad_select_gpio(SH1107_DC);
    gpio_set_direction(SH1107_DC, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(SH1107_RST);
    gpio_set_direction(SH1107_RST, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(SH1107_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(SH1107_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    uint16_t cmd = 0;
    while (init_cmds[cmd].databytes!=0xff) {
        sh1107_send_cmd(init_cmds[cmd].cmd);
        sh1107_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes&0x1F);
        if (init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI

#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    const uint16_t n = sizeof(init_cmds) / sizeof(init_cmds[0]) - 1;
    uint8_t cmd_list[sizeof(init_cmds) / sizeof(init_cmds[0]) - 1];
    for (int i = 0; i < n; i++) {
        cmd_list[i] = init_cmds[i].cmd;
    }
    sh1107_send_cmds_i2c(cmd_list, n);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
}

void sh1107_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa) 
{
    /* buf_w will be ignored, the configured CONFIG_LV_DISPLAY_HEIGHT and _WIDTH,
       and CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE and _PORTRAIT will be used. */ 		
    uint16_t byte_index = 0;
    uint8_t  bit_index = 0;

#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE			
    byte_index = (LV_VER_RES_MAX - y - 1) + (( x>>3 ) * LV_VER_RES_MAX);
    bit_index  = x & 0x7;
#elif defined CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT
    byte_index = x + (( y>>3 ) * LV_HOR_RES_MAX);
    bit_index  = y & 0x7;
#endif

    // ESP_LOGI(TAG, "(%d, %d) %d [%d][%d]", (int)x, (int)y, (int)color.full, (int)byte_index, (int)bit_index);

    if ((color.full == 0) && (LV_OPA_TRANSP != opa)) {
        BIT_SET(buf[byte_index], bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], bit_index);
    }
}

void sh1107_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t columnLow = area->x1 & 0x0F;
    uint8_t columnHigh = (area->x1 >> 4) & 0x0F;
    uint8_t row1 = 0, row2 = 0;
    uint32_t size = 0;
    void *ptr;

#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE		
    row1 = area->x1>>3;
    row2 = area->x2>>3;
#else 
    row1 = area->y1>>3;
    row2 = area->y2>>3;
#endif

    for(int i = row1; i <= row2; i++){

#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
        sh1107_send_cmd(0x10 | columnHigh);         // Set Higher Column Start Address for Page Addressing Mode
        sh1107_send_cmd(0x00 | columnLow);          // Set Lower Column Start Address for Page Addressing Mode
        sh1107_send_cmd(0xB0 | i);                  // Set Page Start Address for Page Addressing Mode
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
        uint8_t cmds[3] = {
            0xB0 | i,
            0x10 | columnHigh,
            0x00 | columnLow,
        };
        CHECK_OK(sh1107_send_cmds_i2c(cmds, 3));
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C

#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE	
        size = area->x2 - area->x1 + 1;	
        ptr = color_map + i * LV_VER_RES_MAX;
#else
        size = area->y2 - area->y1 + 1;
        ptr = color_map + i * LV_HOR_RES_MAX;
#endif
        if (i != row2) {
            sh1107_send_data((uint8_t*)ptr, size);
        } else {
            // complete sending data by sh1107_send_color() and thus call lv_flush_ready()
            sh1107_send_color((uint8_t*)ptr, size);
        }
    }

#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C	
    lv_disp_flush_ready(drv);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
}

void sh1107_rounder(struct _disp_drv_t * disp_drv, lv_area_t *area)
{
    // workaround: always send complete size display buffer
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = LV_HOR_RES_MAX-1;
    area->y2 = LV_VER_RES_MAX-1;
}

void sh1107_sleep_in()
{
    sh1107_send_cmd(0xAE);
}

void sh1107_sleep_out()
{
    sh1107_send_cmd(0xAF);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void sh1107_send_cmd(uint8_t cmd)
{
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    disp_wait_for_pending_transactions();	
    gpio_set_level(SH1107_DC, 0);	 /*Command mode*/
    disp_spi_send_data(&cmd, 1);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    i2c_cmd_handle_t link = i2c_cmd_link_create();
    i2c_master_start(link);

    i2c_master_write_byte(link, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, 0x00, true);
    i2c_master_write_byte(link, cmd, true);

    i2c_master_stop(link);
    i2c_master_cmd_begin(DISP_I2C_PORT, link, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(link);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
}
static esp_err_t sh1107_send_cmds_i2c(uint8_t* cmds, uint16_t length) {
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    i2c_cmd_handle_t link = i2c_cmd_link_create();
    TRY(i2c_master_start(link));

    TRY(i2c_master_write_byte(link, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    TRY(i2c_master_write_byte(link, 0x00, true));
    TRY(i2c_master_write(link, cmds, length, true));

    TRY(i2c_master_stop(link));
    TRY(i2c_master_cmd_begin(DISP_I2C_PORT, link, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(link);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    return ESP_OK;
}

static esp_err_t sh1107_send_data(void* data, uint16_t length)
{
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    disp_wait_for_pending_transactions();
    gpio_set_level(SH1107_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    i2c_cmd_handle_t link = i2c_cmd_link_create();
    TRY(i2c_master_start(link));

    TRY(i2c_master_write_byte(link, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    TRY(i2c_master_write_byte(link, 0x40, true));
    TRY(i2c_master_write(link, data, length, true));

    TRY(i2c_master_stop(link));
    TRY(i2c_master_cmd_begin(DISP_I2C_PORT, link, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(link);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    return ESP_OK;
}

static esp_err_t sh1107_send_color(void * data, uint16_t length)
{
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    disp_wait_for_pending_transactions();
    gpio_set_level(SH1107_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
#if CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    TRY(sh1107_send_data(data, length));
#endif  // CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C
    return ESP_OK;
}
