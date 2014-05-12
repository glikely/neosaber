/*
 * NeoSaber - A lightsaber based on ws2812 LED pixels (NeoPixels).
 *
 * Copyright (c) 2014 by Grant Likely <grant.likely@secretlab.ca>
 * Portions of rotary encoder support Copyright (c) 2013 Adafruit Industries
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the license, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with TrinketHidCombo. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <Adafruit_NeoPixel.h>

enum PINS {
  PIN_PWR_BTN = 0,
  PIN_ENCODER_BTN = 1,
  PIN_LED = 2,
  PIN_ENCODER_A = 3,
  PIN_ENCODER_B = 4,

  // Configuration of lightsaber
  LENGTH_MAX = 56,
  LENGTH_MIN = 8,
  BRIGHTNESS_MIN = 8,
  BRIGHTNESS_MAX = 192,
  BRIGHTNESS_STEP = 8,
  COLOR_STEP = 4,
  BLADE_SPEED_MS = 500,  // Time for blade growing
};

// Events reported by the get_event() function
enum EVENTS {
  EVT_NONE,
  EVT_PWR_BTN_DOWN,
  EVT_PWR_BTN_UP,
  EVT_ENCODER_CCW,
  EVT_ENCODER_CW,
  EVT_ENCODER_BTN_DOWN,
  EVT_ENCODER_BTN_UP,
  NUM_EVTS,
};

enum MODES {
  MODE_COLOR,
  MODE_BRIGHTNESS,
  MODE_LENGTH,
  NUM_MODES,
};


// Global variables
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LENGTH_MAX, PIN_LED, NEO_GRB + NEO_KHZ800);

uint8_t length = LENGTH_MAX;
uint8_t brightness = BRIGHTNESS_MAX;
uint8_t color = 0;
uint8_t mode = 0;


uint8_t get_event()
{
  static uint8_t enc_prev_pos = 0;
  static uint8_t enc_flags;
  
  uint8_t enc_cur_pos = 0;
  uint8_t event = EVT_NONE;

  if (bit_is_set(PINB, PIN_ENCODER_BTN) && bit_is_clear(enc_flags, 5)) {
    enc_flags |= (1 << 5);
    delay(5);
    return EVT_ENCODER_BTN_DOWN;
  } else if (bit_is_clear(PINB, PIN_ENCODER_BTN) && bit_is_set(enc_flags, 5)) {  
    enc_flags &= ~(1 << 5);
    delay(5);
    return EVT_ENCODER_BTN_UP;
  }

  if (bit_is_clear(PINB, PIN_PWR_BTN) && bit_is_clear(enc_flags, 6)) {
    enc_flags |= (1 << 6);
    delay(5);
    return EVT_PWR_BTN_DOWN;
  } else if (bit_is_set(PINB, PIN_PWR_BTN) && bit_is_set(enc_flags, 6)) {  
    enc_flags &= ~(1 << 6);
    delay(5);
    return EVT_PWR_BTN_UP;
  }
  
  // read in the encoder state first
  if (bit_is_clear(PINB, PIN_ENCODER_A))
    enc_cur_pos |= (1 << 0);
  if (bit_is_clear(PINB, PIN_ENCODER_B))
    enc_cur_pos |= (1 << 1);
  

  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos) {
    // this is the first edge
    if (enc_prev_pos == 0x00) {
      if (enc_cur_pos == 0x01)
        enc_flags |= (1 << 0);
      else if (enc_cur_pos == 0x02)
        enc_flags |= (1 << 1);
    }
 
    // this is when the encoder is in the middle of a "step"
    if (enc_cur_pos == 0x03) {
      enc_flags |= (1 << 4);
    } else if (enc_cur_pos == 0x00) {
      // this is the final edge
      if (enc_prev_pos == 0x02)
        enc_flags |= (1 << 2);
      else if (enc_prev_pos == 0x01)
        enc_flags |= (1 << 3);
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4)))
        event = EVT_ENCODER_CW;
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4)))
        event = EVT_ENCODER_CW;
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4)))
        event = EVT_ENCODER_CCW;
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4)))
        event = EVT_ENCODER_CCW;
 
      enc_flags = 0; // reset for next time
    }
  }

  enc_prev_pos = enc_cur_pos;
  return event;
} 

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


void setup()
{
  pinMode(PIN_PWR_BTN, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ENCODER_A, INPUT);
  pinMode(PIN_ENCODER_B, INPUT);
  pinMode(PIN_ENCODER_BTN, INPUT);

  digitalWrite(PIN_ENCODER_A, HIGH);
  digitalWrite(PIN_ENCODER_B, HIGH);
  digitalWrite(PIN_PWR_BTN, HIGH);
  
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();
} 

void loop ()
{
  static bool blade_on = false;
  bool clockwise = false;
  int i;
  uint8_t event = get_event();
  
  switch (event) {
    case EVT_PWR_BTN_DOWN:
      if (blade_on == false) {
        for (i = 0; i < length; i++) {
          strip.setPixelColor(i, Wheel(color));
          strip.show();
          delay(BLADE_SPEED_MS/length);
        }
        blade_on = true;
      } else {
        for (i = 0; i < length; i++) {
          strip.setPixelColor((length - 1) - i, 0);
          strip.show();
          delay(BLADE_SPEED_MS/length);
        }
        blade_on = false;
      }
      break;
      
    case EVT_ENCODER_BTN_DOWN:
      mode++;
      if (mode >= NUM_MODES)
        mode = 0;
      break;
      
    case EVT_ENCODER_CW:
      clockwise = true;
      // fall through
    case EVT_ENCODER_CCW:
      switch (mode) {
        case MODE_COLOR:
          if (clockwise)
            color += COLOR_STEP;
          else
            color -= COLOR_STEP;
          break;
          
        case MODE_LENGTH:
          if (clockwise && length < LENGTH_MAX)
            length++;
          if (!clockwise && length > LENGTH_MIN)
            length--;
          break;
          
        case MODE_BRIGHTNESS:
          if (clockwise && brightness < BRIGHTNESS_MAX - BRIGHTNESS_STEP)
            brightness += BRIGHTNESS_STEP;
          if (!clockwise && brightness > BRIGHTNESS_MIN + BRIGHTNESS_STEP)
            brightness -= BRIGHTNESS_STEP;
          strip.setBrightness(brightness);
          break;
      }
      break;
  }

  if (blade_on) {
    for (i = 0; i < length; i++)
      strip.setPixelColor(i, Wheel(color));
    for (; i < strip.numPixels(); i++)
      strip.setPixelColor(i, 0);
    strip.show();
  }
}



