#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "console/console.h"
#include "esp_spi_flash.h"
#include "esp_spiram.h"
#include "palette/palette.h"
#include "display/dispcolor.h"
#include "mlx90640/mlx90640_api.h"
#include "adc/adc.h"


#define SW_VERSION_MAJOR	1
#define SW_VERSION_MINOR	0


#define CALC_MODE_DIRECT		0	// � �������� ���������� (��� ������������)
#define CALC_MODE_INTERPOL		1	// � �������������

#define CALC_MODE				CALC_MODE_INTERPOL


#define MIN_TEMP				-40
#define MAX_TEMP				300
#define MIN_TEMPSCALE_DELTA		20		// ����������� ������ �� �����
#define SCALE_DEFAULT_MIN		10		// ����������� ����������� ����� �� ���������
#define SCALE_DEFAULT_MAX		50		// ������������ ����������� ����� �� ���������
#define AUTOSCALE_MODE					// ����� �������������� �����
// ���������� ������
#define dispWidth 				320
#define dispHeight				240
// ���������� �������� �����������
#define termWidth				32
#define termHeight				24

#if (CALC_MODE == CALC_MODE_DIRECT)
// ���������� ��������� ��������
#define imageWidth				(9 * termWidth)
#define imageHeight 			(9 * termHeight)
#endif

#if (CALC_MODE == CALC_MODE_INTERPOL)
#define iSteps					9		// ���������� ������������� ����� ��� ������������ �� �����������/���������
#define INT_MODE						// ������������ � ������������� ����������. ���� ���������������� - ����� ������������ ����������
// ���������� ����������������� �����������
#define HQtermWidth				((termWidth - 1) * iSteps)
#define HQtermHeight			((termHeight - 1) * iSteps)
// ���������� ��������� ��������
#define imageWidth 				HQtermWidth
#define	imageHeight 			HQtermHeight
#endif


static paramsMLX90640 params;
static uint16_t *Frame;
static uint16_t *eeMLX90640;
static float *TermoImage;
static int16_t *TermoImage16;
#if (CALC_MODE == CALC_MODE_INTERPOL)
static int16_t *TermoHqImage16;
#endif
static tRGBcolor *pPalette;
static uint16_t PaletteSteps = 0;

const float FPS_rates[] = {0.5, 1, 2, 4, 8, 16, 32, 64};

#if (CALC_MODE == CALC_MODE_DIRECT)
//==============================================================================
// ��������� ��������� ����������� � �������� ����������
//==============================================================================
void DrawImage(int16_t *pImage, tRGBcolor *pPalette, uint16_t PaletteSize, uint16_t X, uint16_t Y, uint8_t pixelWidth, uint8_t pixelHeight, float minTemp)
{
	int cnt = 0;
	for (int row = 0; row < 24; row++)
	{
		for (int col = 0; col < 32; col++, cnt++)
		{
			int16_t colorIdx = pImage[cnt] - (minTemp * 10);

			if (colorIdx < 0)
				colorIdx = 0;
			if (colorIdx >= PaletteSize)
				colorIdx = PaletteSize - 1;

	    	uint16_t color = RGB565(pPalette[colorIdx].r, pPalette[colorIdx].g, pPalette[colorIdx].b);
			dispcolor_FillRect((31 - col) * pixelWidth + X, row * pixelHeight + Y, pixelWidth, pixelHeight, color);
		}
	}
}
//==============================================================================
#endif


#if (CALC_MODE == CALC_MODE_INTERPOL)
//==============================================================================
// ��������� ��������� ����������������� �����������
//==============================================================================
void DrawHQImage(int16_t *pImage, tRGBcolor *pPalette, uint16_t PaletteSize, uint16_t X, uint16_t Y, float minTemp)
{
	int cnt = 0;
	for (int row = 0; row < HQtermHeight; row++)
	{
		for (int col = 0; col < HQtermWidth; col++, cnt++)
		{
			int16_t colorIdx = pImage[cnt] - (minTemp * 10);

			if (colorIdx < 0)
				colorIdx = 0;
			if (colorIdx >= PaletteSize)
				colorIdx = PaletteSize - 1;

	    	uint16_t color = RGB565(pPalette[colorIdx].r, pPalette[colorIdx].g, pPalette[colorIdx].b);
	    	dispcolor_DrawPixel((HQtermWidth - col - 1) + X, row + Y, color);
		}
	}
}
//==============================================================================
#endif


//==============================================================================
// ��������� ��������� �������� �����
//==============================================================================
void DrawScale(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, float minTemp, float maxTemp)
{
	tRGBcolor *Buffer = getPalette(PALETTE_IRON, Height);
	if (!Buffer)
	    return;

    for (int i = 0; i < Height; i++)
	{
		uint16_t color = RGB565(Buffer[i].r, Buffer[i].g, Buffer[i].b);
		dispcolor_FillRect(X, Y + Height - i - 1, Width, 1, color);
	}

    freePalette(Buffer);

    // ����� ������������� �������� � ����� (�� ����������� - �� ������)
    int16_t TextWidth = dispcolor_getFormatStrWidth(FONTID_6X8M, "%.0f�C", maxTemp);
    dispcolor_printf(X + (Width - TextWidth) / 2, Y + 2, FONTID_6X8M, BLACK, "%.0f�C", maxTemp);
    // ����� ������������ �������� � ����� (�� ����������� - �� ������)
    TextWidth = dispcolor_getFormatStrWidth(FONTID_6X8M, "%.0f�C", minTemp);
    dispcolor_printf(X + (Width - TextWidth) / 2, Y + Height - 10, FONTID_6X8M, WHITE, "%.0f�C", minTemp);
}
//==============================================================================


//==============================================================================
// ��������� ������ "�������" � �������� ����������� � ������ ����������� �������� ������
//==============================================================================
void DrawCenterTempColor(uint16_t cX, uint16_t cY, float Temp, tRGBcolor *color)
{
	uint8_t offMin = 5;		// ���������� �� ������ �� ������ �����
	uint8_t offMax = 10;	// ���������� �� ������ �� ����� �����
	uint8_t offTwin = 1;	// ���������� ����� 2 ������������� �������

	// ������� ����� �����������
	dispcolor_DrawLine(cX - offTwin, cY - offMin, cX - offTwin, cY - offMax, RGB565(color->r, color->g, color->b));
	dispcolor_DrawLine(cX + offTwin, cY - offMin, cX + offTwin, cY - offMax, RGB565(color->r, color->g, color->b));
	// ������ ����� �����������
	dispcolor_DrawLine(cX - offTwin, cY + offMin, cX - offTwin, cY + offMax, RGB565(color->r, color->g, color->b));
	dispcolor_DrawLine(cX + offTwin, cY + offMin, cX + offTwin, cY + offMax, RGB565(color->r, color->g, color->b));
	// ����� ����� �����������
	dispcolor_DrawLine(cX - offMin, cY - offTwin, cX - offMax, cY - offTwin, RGB565(color->r, color->g, color->b));
	dispcolor_DrawLine(cX - offMin, cY + offTwin, cX - offMax, cY + offTwin, RGB565(color->r, color->g, color->b));
	// ������ ����� �����������
	dispcolor_DrawLine(cX + offMin, cY - offTwin, cX + offMax, cY - offTwin, RGB565(color->r, color->g, color->b));
	dispcolor_DrawLine(cX + offMin, cY + offTwin, cX + offMax, cY + offTwin, RGB565(color->r, color->g, color->b));

	if ((Temp > -100) && (Temp < 500))
		dispcolor_printf(cX + 8, cY + 8, FONTID_6X8M, RGB565(color->r, color->g, color->b), "%.1f�C", Temp);
}
//==============================================================================


//==============================================================================
// ��������� ������ "�������" � �������� ����������� � ������ ����������� ����� � ������ �����
//==============================================================================
void DrawCenterTemp(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, float Temp)
{
	uint16_t cX = (Width >> 1) + X;
	uint16_t cY = (Height >> 1) + Y;
	tRGBcolor colorBlack = {0, 0, 0};
	tRGBcolor colorWhite = {255, 255, 255};

	// ��������� ���� ������
	DrawCenterTempColor(cX + 1, cY + 1, Temp, &colorBlack);
	// ��������� �����
	DrawCenterTempColor(cX, cY, Temp, &colorWhite);
}
//==============================================================================


#if (CALC_MODE == CALC_MODE_INTERPOL)
//==============================================================================
// ��������� ������������ �����������
//==============================================================================
void InterpolateImage(int16_t *pImage, int16_t *pHdImage)
{
	// ����������� ����� �� ����������� 32 -> 279 (288)
	for (uint16_t row = 0; row < termHeight; row++)
	{
		for (uint16_t col = 0; col < (termWidth - 1); col++)
		{
			uint16_t ImageIdx = row * termWidth + col;
			int16_t tempStart = pImage[ImageIdx];
			int16_t tempEnd = pImage[ImageIdx + 1];

			for (uint16_t step = 0; step < iSteps; step++)
			{
#ifdef INT_MODE
				uint32_t Idx = (row * HQtermWidth + col) * iSteps + step;
				pHdImage[Idx] = tempStart * (iSteps - step) / iSteps + tempEnd * step / iSteps;
#else
				float n = (float)step / (float) (iSteps - 1);
				uint32_t Idx = (row * HQtermWidth + col) * iSteps + step;
				pHdImage[Idx] = tempStart * (1.0f - n) + tempEnd * n;
#endif
			}
		}
	}

	// ����������� ����� �� ��������� 24 -> 207 (216)
	for (uint16_t col = 0; col < HQtermWidth; col++)
	{
		for (uint16_t row = 0; row < termHeight; row++)
		{
			int16_t tempStart = pHdImage[row * iSteps * HQtermWidth + col];
			int16_t tempEnd = pHdImage[(row + 1) * iSteps * HQtermWidth + col];

			for (uint16_t step = 1; step < iSteps; step++)
			{
#ifdef INT_MODE
				uint32_t Idx = (row * iSteps + step) * HQtermWidth + col;
				pHdImage[Idx] = tempStart * (iSteps - step) / iSteps + tempEnd * step / iSteps;
#else
				float n = (float)step / (float) (iSteps - 1);
				uint32_t Idx = (row * iSteps + step) * HQtermWidth + col;
				pHdImage[Idx] = tempStart * (1.0f - n) + tempEnd * n;
#endif
			}
		}
	}
}
//==============================================================================
#endif


//==============================================================================
// ��������� ������ ������ �������
//==============================================================================
void DrawBattery(uint16_t X, uint16_t Y, float capacity)
{
	// ���������� ���� �������
	uint16_t Color = GREEN;
	if (capacity < 80)
		Color = RGB565(249, 166, 2);
	if (capacity < 50)
		Color = RED;

	// ������ ������ ���������
	dispcolor_DrawRectangle(X, Y, X + 17, Y + 9, WHITE);
	dispcolor_DrawRectangleFilled(X + 17, Y + 2, X + 19, Y + 6, WHITE);
	// ������ �������
	dispcolor_DrawRectangleFilled(X + 12, Y + 2, X + 15, Y + 7, capacity < 80 ? BLACK : Color);
	dispcolor_DrawRectangleFilled(X + 7, Y + 2, X + 10, Y + 7, capacity < 50 ? BLACK : Color);
	dispcolor_DrawRectangleFilled(X + 2, Y + 2, X + 5, Y + 7, capacity < 25 ? BLACK : Color);
}
//==============================================================================


//==============================================================================
void app_main()
{
	int result;
    float minTemp = 0;
    float maxTemp = 0;
    float minTempNew = SCALE_DEFAULT_MIN;
    float maxTempNew = SCALE_DEFAULT_MAX;

	// ������������� �������
    printf("Display init\n");
    dispcolor_Init(dispWidth, dispHeight);
    dispcolor_SetBrightness(100);

    // ����� ����� ���������� � CPU
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    console_printf(MsgInfo, "ESP32 rev. %d (%d CPU cores, WiFi%s%s), ", chip_info.revision, chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    console_printf(MsgInfo, " %d MB %s SPI FLASH\n", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    console_printf(MsgInfo, " %d MB SPI PSRAM\n", esp_spiram_get_size() / (1024 * 1024));
    console_pause(300);

    // ������������� ���
    init_adc();
    uint32_t BatteryVoltage = getBatteryVoltage();
    console_printf(MsgInfo, " VBAT=%d mV\n", BatteryVoltage);
    console_pause(100);

    console_printf(MsgInfo, "������������� Melexis mlx90640\r\n");
    MLX90640_Init();

    console_printf(MsgInfo, " ��������� ��� ��� ������� mlx90640\r\n");
    Frame = heap_caps_malloc(834 << 1, MALLOC_CAP_8BIT);
    if (!Frame)
        FatalErrorMsg("������ ��������� ��� ��� ������ mlx90640\r\n");

	eeMLX90640 = heap_caps_malloc(832 << 1, MALLOC_CAP_8BIT);
    if (!eeMLX90640)
    	FatalError("������ ��������� ��� ��� �������� �������� mlx90640, ��������� �� EEPROM\r\n");

    TermoImage = heap_caps_malloc(768 << 2, MALLOC_CAP_8BIT);
    if (!TermoImage)
    	FatalError("������ ��������� ��� ��� �����������\r\n");
    TermoImage16 = heap_caps_malloc(768 << 1, MALLOC_CAP_8BIT);
    if (!TermoImage16)
    	FatalError("������ ��������� ��� (int16) ��� �����������\r\n");

#if (CALC_MODE == CALC_MODE_INTERPOL)
    TermoHqImage16 = heap_caps_malloc((HQtermWidth * HQtermHeight) << 1, MALLOC_CAP_8BIT);
    if (!TermoHqImage16)
    	FatalError("������ ��������� ��� (int16) ��� ����������������� �����������\r\n");
#endif

    console_printf(MsgInfo, "������ ���������� mlx90640 �� EEPROM\r\n");
	result = MLX90640_DumpEE (0x33, eeMLX90640); //the whole EEPROM is stored in the eeMLX90640 array
    if (result < 0)
    	FatalError("������ ������ EEPROM �� mlx90640, ��� ������ %d\r\n", result);

    console_printf(MsgInfo, " ���������� ���������� mlx90640 �� EEPROM\r\n");
    result = MLX90640_ExtractParameters(eeMLX90640, &params);
    if (result < 0)
    	FatalError("������ ���������� ���������� mlx90640, ��� ������ %d\r\n", result);

    uint8_t FPS_Idx = 3;
    console_printf(MsgInfo, " ��������� ������� ���������� ����������� (%.1f FPS)\r\n", FPS_rates[FPS_Idx]);
    result = MLX90640_SetRefreshRate(0x33, FPS_Idx);
    if (result < 0)
       	console_printf(MsgWarning, "������ ��������� ������� ���������� �����������, ��� ������ %d\r\n", result);


    console_printf(MsgInfo, "������������� ������� ���������\r\n", result);
    console_pause(500);

    dispcolor_ClearScreen();
    dispcolor_printf(2, 4, FONTID_6X8M, WHITE, "���������� (esp32 + mlx90640)  %.1f FPS\r\n", FPS_rates[FPS_Idx]);

    while (1)
    {
    	// ������������ ����� ����� ���� ����������
        if ((minTempNew != minTemp) || (maxTempNew != maxTemp))
        {
        	// ��������� ����� ���� �������� ��������� ������ MIN_TEMPSCALE_DELTA
        	float Delta = maxTempNew - minTempNew;
        	if (Delta < MIN_TEMPSCALE_DELTA)
        	{
        		minTempNew -= (MIN_TEMPSCALE_DELTA - Delta) / 2;
        		maxTempNew += (MIN_TEMPSCALE_DELTA - Delta) / 2;
        	}

        	minTemp = minTempNew;
        	maxTemp = maxTempNew;

        	// ��������� ����� �������� �����
        	freePalette(pPalette);
        	PaletteSteps = (uint16_t)((maxTemp - minTemp) * 10);
        	pPalette = getPalette(PALETTE_IRON, PaletteSteps);
        	// ��������� ����� � ������ ����� ������
        	DrawScale(imageWidth + 2, (dispHeight - imageHeight) >> 1, dispWidth - imageWidth - 2, imageHeight, minTemp, maxTemp);
        }

        // ������ ������ �� �������
        result = MLX90640_GetFrameData(0x33, Frame);

        // ������� � ������� ��������� ����������, ��������� �� mlx90640
    	float Vdd = MLX90640_GetVdd(Frame, &params);
    	float Ta = MLX90640_GetTa(Frame, &params);
    	dispcolor_printf_Bg(205, 228, FONTID_6X8M, RGB565(0, 160, 160), BLACK, "Ta=%.1f�C ", Ta);
    	dispcolor_printf_Bg(266, 228, FONTID_6X8M, RGB565(96, 160, 0), BLACK, "Vdd=%.2fV ", Vdd);

    	// ������� � ������� ����� ������������ � ��� ����������
        float VBAT = ((float)getBatteryVoltage()) / 1000;
    	float capacity = VBAT * 125 - 400;
    	if (capacity > 100)
    		capacity = 100;
    	if (capacity < 0)
    		capacity = 0;
//    	dispcolor_printf_Bg(68, 228, FONTID_6X8M, RGB565(160, 96, 96), BLACK, "BAT=%.0f%% ", capacity);
    	dispcolor_printf_Bg(137, 228, FONTID_6X8M, RGB565(160, 96, 0), BLACK, "VBAT=%.2fV ", VBAT);
    	DrawBattery(300, 2, capacity++);

    	// ������ ������� ����������
    	float emissivity = 0.95;
    	float tr = Ta - 8;
    	MLX90640_CalculateTo(Frame, &params, emissivity, tr, TermoImage);

    	// �������� ����������� � ������������� ������ ��� ��������� ���������� ��������
    	for (uint16_t i = 0; i < 768; i++)
        	TermoImage16[i] = TermoImage[i] * 10;

#if (CALC_MODE == CALC_MODE_DIRECT)
    	// ������� ����������
    	if (pPalette)
    		DrawImage(TermoImage16, pPalette, PaletteSteps, 0, (dispHeight - imageHeight) >> 1, 9, 9, minTemp);
#endif
#if (CALC_MODE == CALC_MODE_INTERPOL)
    	// ������� ����������
    	InterpolateImage(TermoImage16, TermoHqImage16);
    	if (pPalette)
    		DrawHQImage(TermoHqImage16, pPalette, PaletteSteps, 0, (dispHeight - imageHeight) >> 1, minTemp);
#endif

    	// ������� � ������� ����������� � ������ ������
    	float MainTemp =
    			TermoImage[termWidth * ((termHeight >> 1) - 1) + ((termWidth >> 1) - 1)] +
    			TermoImage[termWidth * ((termHeight >> 1) - 1) + (termWidth >> 1)] +
    			TermoImage[termWidth * (termHeight >> 1) + ((termWidth >> 1) - 1)] +
				TermoImage[termWidth * (termHeight >> 1) + (termWidth >> 1)];
    	MainTemp /= 4;
    	DrawCenterTemp(0, (dispHeight - imageHeight) >> 1, imageWidth, imageHeight, MainTemp);

    	// ����� � ����� ����������� � ������������ ����������� � �����
        float minT = 300;
        float maxT = -40;
    	for (uint16_t i = 0; i < 768; i++)
    	{
    		if (maxT < TermoImage[i])
    			maxT = TermoImage[i];
    		if (minT > TermoImage[i])
    			minT = TermoImage[i];
    	}
		if (maxT > MAX_TEMP)
			maxT = MAX_TEMP;
		if (minT < MIN_TEMP)
			minT = MIN_TEMP;

		dispcolor_printf_Bg(1, 228, FONTID_6X8M, RGB565(32, 32, 192), BLACK, "MIN=%.1f�C ", minT);
		dispcolor_printf_Bg(69, 228, FONTID_6X8M, RGB565(192, 32, 32), BLACK, "MAX=%.1f�C ", maxT);

#ifdef AUTOSCALE_MODE
		minTempNew = minT;
        maxTempNew = maxT;
#endif

    	// ��������� ������� �� ������ �����
    	dispcolor_Update();
    }
}
//==============================================================================
