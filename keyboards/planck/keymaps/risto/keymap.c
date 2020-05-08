/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"
#include "keymap_finnish.h"

extern keymap_config_t keymap_config;

enum planck_layers {
  _QWERTY,
  _LOWER,
  _RAISE,
  _ADJUST,
  _OMA
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  BACKLIT,
};

enum unicode_names {
  DOT,
  COLON,
  COMMA,
  SMCLN, //semicolon
  DASH,
  UNDRSCR //underscore
};
const uint32_t PROGMEM unicode_map[] = {
  [DOT] = 0x002E,
  [COLON] = 0x003A,
  [COMMA] = 0x002C,
  [SMCLN] = 0x003B, //semicolon
  [DASH] = 0x002D,
  [UNDRSCR] = 0x005F //underscore
};
#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define OMA MO(_OMA)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Esc  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * | Ctrl |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   Ö  |  Ä   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   -  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | TAB | Ctrl  | GUI  | Alt  |Lower | spce | bksp |Raise | Oma  | Caps | Shift|ctrl |
 * `-----------------------------------------------------------------------------------'
 *                 ^ super
 */
[_QWERTY] = LAYOUT_planck_grid(
    KC_ESC,  FI_Q,    FI_W,    FI_E,    FI_R,    FI_T,    FI_Y,    FI_U,    FI_I,    FI_O,    FI_P,    KC_BSPC,
    KC_LCTL,  FI_A,    FI_S,    FI_D,    FI_F,    FI_G,    FI_H,    FI_J,    FI_K,    FI_L,    FI_ODIA, FI_ADIA,
    KC_LSFT, FI_Z,    FI_X,    FI_C,    FI_V,    FI_B,    FI_N,    FI_M,  FI_COMM, FI_DOT, FI_MINS,  KC_ENT,
    KC_TAB, KC_LCTL, KC_LGUI, KC_LALT, LOWER,   KC_SPC,  KC_BSPC,  RAISE,   OMA, KC_CAPS, KC_RCTL, KC_RSFT // KC_LOCK
),


/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |      |   ^  | home | pgup | pgdn | end  |  left | down |  up  | right|   *   |      |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |      |      |      |   \  |   '  |  `  |      |   ~ |   @   |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |     |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
    _______, FI_CIRC, KC_HOME, KC_PGUP,KC_PGDOWN, KC_END, KC_LEFT, KC_DOWN, KC_UP,KC_RIGHT, FI_ASTR, _______,
    _______, _______, _______, FI_BSLS, FI_QUOT,  FI_GRV, _______, FI_TILD,   FI_AT, _______, _______, _______,
    _______, _______, _______, _______, _______, _______,  KC_DEL, _______, _______, _______, _______, _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |      |   !  |   "  |   #  |   $  |   %  |   &  |   /  |   =  |   ?  |   +  |      |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  |      |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |      |   |  |   <  |   >  |   [  |   ]  |   (  |   )  |   {  |   }  |   @  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_grid(
    _______,  FI_EXLM, FI_DQUO, FI_HASH, FI_DLR, FI_PERC, FI_AMPR, FI_SLSH,   FI_EQL, FI_QUES,  FI_PLUS, _______,
    _______,  FI_1,    FI_2,    FI_3,    FI_4,    FI_5,    FI_6,    FI_7,    FI_8,    FI_9,    FI_0,   _______,
    _______, FI_PIPE,   FI_LABK, FI_RABK, FI_LBRC, FI_RBRC, FI_LPRN, FI_RPRN, FI_LCBR, FI_RCBR, FI_AT, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),

/* Adjust (Lower + Raise)
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|      |      |      |      |      |      |      |      |      |  Del |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |      |      |      |Aud on|Audoff|AGnorm|AGswap|Qwerty|      |      |      |      |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
//    _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, KC_DEL ,
//    _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, QWERTY,  _______,  _______, _______, _______,
//    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
//    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
       KC_1, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
    ),

[_OMA] = LAYOUT_planck_grid(
       KC_1, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
)
};

uint32_t layer_state_set_user(uint32_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          PORTE &= ~(1<<6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          PORTE |= (1<<6);
        #endif
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
}

void dip_update(uint8_t index, bool active) {
  switch (index) {
    case 0:
      if (active) {
        layer_on(_ADJUST);
      } else {
        layer_off(_ADJUST);
      }
      break;
    case 1:
      if (active) {
        muse_mode = true;
      } else {
        muse_mode = false;
        #ifdef AUDIO_ENABLE
          stop_all_notes();
        #endif
      }
   }
}

void matrix_scan_user(void) {
  #ifdef AUDIO_ENABLE
    if (muse_mode) {
      if (muse_counter == 0) {
        uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
        if (muse_note != last_muse_note) {
          stop_note(compute_freq_for_midi_note(last_muse_note));
          play_note(compute_freq_for_midi_note(muse_note), 0xF);
          last_muse_note = muse_note;
        }
      }
      muse_counter = (muse_counter + 1) % muse_tempo;
    }
  #endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
