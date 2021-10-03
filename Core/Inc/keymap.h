//
// Created by NonSense on 2021/10/03.
//

#ifndef EHHHKB_KEYMAP_H
#define EHHHKB_KEYMAP_H

#include "keycode.h"
#include "main.h"
#include <stdint.h>

#define ROW_NUM 9
#define COL_NUM 9
#define LAYER_NUM 2


#define XXX KC_NO
#define LAYOUT( \
    K11, K12, K13, K14, K15, K16, K17, K18, K19, K61, K62, K63, K64, K65, K69,   K67, K68,  \
      K21, K22, K23, K24, K25, K26, K27, K28, K29, K71, K72, K73, K74,  K75,     K77, K78,  \
       K31, K32, K33, K34, K35, K36, K37, K38, K39, K81, K82, K83,     K84,      K87, K88,  \
        K41,  K42, K43, K44, K45, K46, K47, K48, K49, K91, K92,    K93,   K95,   K97, K98,  \
     K51,  K52,  K53,             K54,                K57,  K58,   K59,   K55,   K99, K89   \
) { \
    { K11, K12, K13, K14, K15, K16, K17, K18, K19 }, \
    { K21, K22, K23, K24, K25, K26, K27, K28, K29 }, \
    { K31, K32, K33, K34, K35, K36, K37, K38, K39 }, \
    { K41, K42, K43, K44, K45, K46, K47, K48, K49 }, \
    { K51, K52, K53, K54, K55, XXX, K57, K58, K59 }, \
    { K61, K62, K63, K64, K65, XXX, K67, K68, K69 }, \
    { K71, K72, K73, K74, K75, XXX, K77, K78, XXX}, \
    { K81, K82, K83, K84, XXX, XXX, K87, K88, K89 }, \
    { K91, K92, K93, XXX, K95, XXX, K97, K98, K99 } \
}

extern uint8_t keymap[LAYER_NUM][ROW_NUM][COL_NUM];
extern uint16_t matrixRowPin[ROW_NUM];
extern uint16_t matrixColPin[COL_NUM];
extern GPIO_TypeDef* matrixRowPort[ROW_NUM];
extern GPIO_TypeDef* matrixColPort[COL_NUM];

#endif//EHHHKB_KEYMAP_H
