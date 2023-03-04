#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ili9341.h"


#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left


//#define TTGO_V12	// Ревизия платы TTGO v1.2
#define TTGO_V13	// Ревизия платы TTGO v1.3

#ifdef TTGO_V12
#define PIN_MISO 	12
#define PIN_MOSI 	23
#define PIN_CLK  	18
#define PIN_CS   	27
#define PIN_DC   	26
#define PIN_RST  	5
#define PIN_BL		4	// Управление подсветкой
#endif

#ifdef TTGO_V13
#define PIN_MISO 	12
#define PIN_MOSI 	23
#define PIN_CLK  	18
#define PIN_CS   	27
#define PIN_DC   	32
#define PIN_RST  	5
#define PIN_BL		4	// Управление подсветкой
#endif


// The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;


static uint16_t *ScreenBuff;
static spi_device_handle_t _spi;
static uint8_t rotation = 0;
static uint16_t _width, _height;


DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] =
{
    // Power contorl B, power control = 0, DC_ENA = 1
    {0xCF, {0x00, 0x83, 0X30}, 3},
    // Power on sequence control,
    // cp1 keeps 1 frame, 1st frame enable
    // vcl = 0, ddvdh=3, vgh=1, vgl=2
    // DDVDH_ENH=1
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    // Driver timing control A,
    // non-overlap=default +1
    // EQ=default - 1, CR=default
    // pre-charge=default - 1
    {0xE8, {0x85, 0x01, 0x79}, 3},
    // Power control A, Vcore=1.6V, DDVDH=5.6V
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    // Pump ratio control, DDVDH=2xVCl
    {0xF7, {0x20}, 1},
    // Driver timing control, all=0 unit
    {0xEA, {0x00, 0x00}, 2},
    // Power control 1, GVDD=4.75V
    {0xC0, {0x26}, 1},
    // Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3
    {0xC1, {0x11}, 1},
    // VCOM control 1, VCOMH=4.025V, VCOML=-0.950V
    {0xC5, {0x35, 0x3E}, 2},
    // VCOM control 2, VCOMH=VMH-2, VCOML=VML-2
    {0xC7, {0xBE}, 1},
    // Memory access control, MX=MY=0, MV=1, ML=0, BGR=1, MH=0
    {0x36, {0x28}, 1},
    // Pixel format, 16bits/pixel for RGB/MCU interface
    {ILI9341_PIXFMT, {0x55}, 1},
    // Frame rate control, f=fosc, 70Hz fps
    {0xB1, {0x00, 0x1B}, 2},
    // Enable 3G, disabled
    {0xF2, {0x08}, 1},
    // Gamma set, curve 1
    {0x26, {0x01}, 1},
    // Positive gamma correction
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    // Negative gamma correction
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    // Column address set, SC=0, EC=0xEF
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    // Page address set, SP=0, EP=0x013F
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    // Memory write
    {0x2C, {0}, 0},
    // Entry mode set, Low vol detect disabled, normal display
    {0xB7, {0x07}, 1},
    // Display function control
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    // Sleep out
    {ILI9341_SLPOUT, {0}, 0x80},
    // Display on
    {ILI9341_DISPON, {0}, 0x80},
	{ILI9341_RAMWR, {0}, 0x80},
    {0, {0}, 0xff},
};


//==============================================================================
//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
//==============================================================================
void lcd_cmd(const uint8_t cmd)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    t.user = (void*) 0;                //D/C needs to be set to 0
    ret = spi_device_transmit(_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//==============================================================================


//==============================================================================
//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
//==============================================================================
void lcd_data(const uint8_t *data, int len)
{
    if (len == 0)
    	return;             //no need to send anything

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;               //Data
    t.user = (void*) 1;                //D/C needs to be set to 1
    ret = spi_device_transmit(_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//==============================================================================


//==============================================================================
// This function is called (in irq context!) just before a transmission starts. It will set the D/C line to the value indicated in the user field.
//==============================================================================
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int) t->user;
    gpio_set_level(PIN_DC, dc);
}
//==============================================================================


//==============================================================================
// Установка окна на дисплее, которое будем заполнять
//==============================================================================
void ili9341_setWindow(uint16_t x, uint16_t y, uint16_t x_end, uint16_t y_end)
{
	uint8_t Buff[4];

    lcd_cmd(ILI9341_CASET); // Column addr set
    Buff[0] = (x >> 8) & 0xFF;
    Buff[1] = x & 0xFF;
    Buff[2] = (x_end >> 8) & 0xFF;
    Buff[3] = x_end & 0xFF;
    lcd_data(Buff, 4);

    lcd_cmd(ILI9341_PASET); // Row addr set
    Buff[0] = (y >> 8) & 0xFF;
    Buff[1] = y & 0xFF;
    Buff[2] = (y_end >> 8) & 0xFF;
    Buff[3] = y_end & 0xFF;
    lcd_data(Buff, 4);

    lcd_cmd(ILI9341_RAMWR); // write to RAM
}
//==============================================================================


//==============================================================================
// Установка направления заполнения дисплея
//==============================================================================
void ili9341_setRotation(uint8_t m)
{
    rotation = m % 4; // can't be higher than 3
    switch (rotation)
    {
        case 0:
            m = (MADCTL_MX | MADCTL_BGR);
            _width  = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 1:
            m = (MADCTL_MV | MADCTL_BGR);
            _width  = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
        case 2:
            m = (MADCTL_MY | MADCTL_BGR);
            _width  = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 3:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            _width  = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
    }

    lcd_cmd(ILI9341_MADCTL);
    lcd_data(&m, 1);
}
//==============================================================================


//==============================================================================
// Процедура управления подсветкой
//==============================================================================
void ili9341_SetBL(uint8_t value)
{
#ifdef PIN_BL
	if (value)
		gpio_set_level(PIN_BL, 1);
	else
		gpio_set_level(PIN_BL, 0);
#endif
}
//==============================================================================


//==============================================================================
// Инициализация интерфейса дисплея и отправка команд инициализации
//==============================================================================
static void lcd_init()
{
    int cmd = 0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
#ifdef PIN_BL
    gpio_set_direction(PIN_BL, GPIO_MODE_OUTPUT);
#endif

    //Reset the display
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //printf("LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff)
    {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);

        if (lcd_init_cmds[cmd].databytes & 0x80)
            vTaskDelay(100 / portTICK_RATE_MS);

        cmd++;
    }

    // Включаем подсветку
    ili9341_SetBL(100);
}
//==============================================================================


//==============================================================================
// Инициализация дисплея
//==============================================================================
void ili9341_init(uint16_t width, uint16_t height)
{
	_width = width;
	_height = height;


  spi_bus_config_t buscfg =
  {
    .miso_io_num = PIN_MISO,
    .mosi_io_num = PIN_MOSI,
    .sclk_io_num = PIN_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = ILI9341_TFTHEIGHT * ILI9341_TFTWIDTH * 2
  };

  spi_device_interface_config_t devcfg =
  {
    .clock_speed_hz = 24 * 1000 * 1000,           //Clock out at 26 MHz
    .mode=0,                                //SPI mode 0
    .spics_io_num = PIN_CS,             //CS pin
    .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
    .pre_cb = lcd_spi_pre_transfer_callback,//Specify pre-transfer callback to handle D/C line
  };

  esp_err_t ret;

  //Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  ESP_ERROR_CHECK(ret);
  //Attach the LCD to the SPI bus
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &_spi);
  ESP_ERROR_CHECK(ret);
  //Initialize the LCD
  lcd_init();

  ScreenBuff = heap_caps_malloc(ILI9341_TFTHEIGHT * ILI9341_TFTWIDTH * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  printf("*ScreenBuff=0x%08X\n", (uint32_t) ScreenBuff);

  ili9341_setRotation(1);
}
//==============================================================================


//==============================================================================
// Процедура смены порядка байт в 2-байтном слове
//==============================================================================
static void SwapBytes(uint16_t *color)
{
	uint8_t temp = *color >> 8;
	*color = (*color << 8) | temp;
}
//==============================================================================


#if (ILI9341_MODE == ILI9341_DIRECT_MODE)

//==============================================================================
// Процедура окрашивает 1 пиксель дисплея
//==============================================================================
void ili9341_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height))
		return;

	SwapBytes(&color);

	ili9341_setWindow(x, y, x, y);
	lcd_data((uint8_t *) &color, 2);
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольника цветом color
//==============================================================================
void ili9341_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	if ((w <= 0) || (h <= 0) || (x >= _width) || (y >= _height))
		return;

	if ((x + w) > _width)
		w = _width - x;
	if ((y + h) > _height)
		h = _height - y;

	SwapBytes(&color);

	uint16_t *Buff = &ScreenBuff[y * _width + x];
	for (uint32_t i = 0; i < h * w; i++)
		Buff[i] = color;

	ili9341_setWindow(x, y, x + w - 1, y + h - 1);
	lcd_data((uint8_t *) Buff, w * h * 2);
}
//==============================================================================
#endif


#if (ILI9341_MODE == ILI9341_BUFFER_MODE)
//==============================================================================
// Процедура окрашивает 1 пиксель в буфере кадра дисплея
//==============================================================================
void ili9341_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height))
		return;

	SwapBytes(&color);

	ScreenBuff[y * _width + x] = color;
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольника в буфере кадра цветом color
//==============================================================================
void ili9341_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	if ((w <= 0) || (h <= 0) || (x >= _width) || (y >= _height))
		return;

	if ((x + w) > _width)
		w = _width - x;
	if ((y + h) > _height)
		h = _height - y;

	SwapBytes(&color);

	for (uint16_t row = 0; row < h; row++)
	{
		for (uint16_t col = 0; col < w; col++)
			ScreenBuff[(y + row) * _width + x + col] = color;
	}
}
//==============================================================================


//==============================================================================
// Процедура обновляет дисплей из буфера кадра
//==============================================================================
void ili9341_update(void)
{
	ili9341_setWindow(0, 0, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH);
	lcd_data((uint8_t *) ScreenBuff, ILI9341_TFTHEIGHT * ILI9341_TFTWIDTH * 2);
}
//==============================================================================
#endif
