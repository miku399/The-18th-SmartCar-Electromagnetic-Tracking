#ifndef __UI_H
#define __UI_H
#include "common.h"
#include "headfile.h"
typedef struct
{
    void (*Disp)(void);
    int  cursor[1], page, enter;
} UI_CLASS;

void UI_Disp(void);
void UI_DispUIStrings(uint8 strings[10][17]);
#endif
