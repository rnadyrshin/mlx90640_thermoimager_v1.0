#ifndef MAIN_PALETTE_PALETTE_H_
#define MAIN_PALETTE_PALETTE_H_

#include "../display/rgbcolor.h"

#define PALETTE_IRON	0


// ������� ���������� ��������� �� ������ ����� ������� ���������� ����
tRGBcolor *getPalette(uint8_t paletteNum, uint16_t steps);
// ��������� ������������ ������, ��������������� ��� �������
void freePalette(tRGBcolor *pPalette);


#endif /* MAIN_PALETTE_PALETTE_H_ */
