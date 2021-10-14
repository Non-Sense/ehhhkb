//
// Created by NonSense on 2021/10/03.
//

#include "keymap.h"

#define _____ _______
#define ____ _______

enum layers {
  _BASE,
//  _ENJP,
//  _ENJPS,
  _FN
};

uint8_t keymap[LAYER_NUM][ROW_NUM][COL_NUM] = {
  [_BASE] = LAYOUT(
    KC_ESC, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0, KC_MINS, KC_EQL, KC_BSLS, KC_GRV,  KC_HOME , KC_END,  \
     KC_TAB,  KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P, KC_LBRC, KC_RBRC, KC_BSPC,       KC_DEL  , KC_INS,  \
      KC_LCTL,  KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_K, KC_L, KC_SCLN, KC_QUOT,   KC_ENT,          KC_PSCR , KC_PGUP, \
      KC_LSFT,    KC_Z, KC_X, KC_C, KC_V, KC_B, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_RSFT, KC_FN1,     KC_UP   , KC_PGDN, \
       KC_LCTL,  KC_FN1,  KC_LALT,          KC_SPC,                  KC_RGUI, KC_RALT, KC_RCTL, KC_LEFT,    KC_DOWN , KC_RIGHT ),

//  [_ENJP] = LAYOUT(
//    ____  , ____, KC_2, ____, ____, ____, KC_6, KC_7, KC_8, KC_9, KC_0, KC_MINS, KC_EQL, KC_BSLS, KC_GRV,  _____ , _____, \
//     ____  ,  ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, KC_RBRC, KC_BSLS, _____,         _____ , _____, \
//      ____   ,  ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, KC_QUOT,   _____,           _____ , _____, \
//      KC_FN3   ,    ____, ____, ____, ____, ____, ____, ____, ____   , ____  , ____   , KC_FN3, _____,     _____ , _____, \
//       ____   ,  ____ ,  ____   ,          ____  ,                  ____   , ____   , ____   , ____   ,    _____ , _____  ),
//
//  [_ENJP] = LAYOUT(
//    ____  , ____, KC_LBRC, ____, ____, ____, KC_EQL, KC_6, KC_QUOT, KC_8, KC_9, KC_INT1, KC_SCLN, KC_INT3, KC_EQL,  _____ , _____, \
//     ____  ,  ____, ____  , ____, ____, ____, ____, ____, ____, ____, ____, KC_LBRC, KC_RBRC, _____,                _____ , _____, \
//      ____   ,  ____, ____  , ____, ____, ____, ____, ____, ____, ____, KC_SCLN, KC_QUOT,   KC_ENT,                 _____ , _____, \
//      ____     ,    ____  , ____, ____, ____, ____, ____, ____, ____   , ____  , ____   , ____  , _____,            _____ , _____, \
//       ____   ,  ____ ,  ____   ,          ____  ,                  ____   , ____   , ____   , ____   ,             _____ , _____  ),

  [_FN] = LAYOUT(
     _____, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, KC_F12, _____, KC_DEL,    _____ , _____ , \
       KC_CAPS,  _____, _____, _____, _____, _____, _____, _____, _____, _____, _____, KC_UP, _____,     _____,      _____ , _____ , \
        _____,    _____, _____, _____, _____, _____, _____, _____, _____, _____, KC_LEFT, KC_RIGHT,    _____,        _____ , _____ , \
         _____,     _____, _____, _____, _____, _____, _____, _____, _____, _____,   KC_DOWN,     KC_FN2,  _____,    _____ , _____ , \
         _____,  _____,    _____,            _____,                  _____, _____, _____,                  _____,    _____ , _____  )
};

uint16_t matrixRowPin[ROW_NUM] = { ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin, ROW8_Pin, ROW9_Pin };
uint16_t matrixColPin[COL_NUM] = { COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin, COL5_Pin, COL6_Pin, COL7_Pin, COL8_Pin, COL9_Pin };
GPIO_TypeDef* matrixRowPort[ROW_NUM] = { ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port, ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port, ROW8_GPIO_Port, ROW9_GPIO_Port };
GPIO_TypeDef* matrixColPort[COL_NUM] = { COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port, COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port, COL9_GPIO_Port };