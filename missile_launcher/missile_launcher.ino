/*
 *    Crash's Eurobot 2017 UI and Missile Launcher
 *    Copyright (C) 2017  Crash Racing Team <crt@turag.de>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FastLED.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

FASTLED_USING_NAMESPACE

static ros::NodeHandle nh;
static void led_cb(const std_msgs::String&);
static void rocket_cb(const std_msgs::Bool&);
static ros::Subscriber<std_msgs::String> led_sub("lighthouse", led_cb);
static ros::Subscriber<std_msgs::Bool> rocket_sub("rocket", rocket_cb);

static unsigned game_state = 0;
static unsigned long game_start_time = 0;

/****************************************************
 *** WLToys Missile Controller                    ***
 ****************************************************/

class MissileLauncher
{
public:
  MissileLauncher(unsigned pinMotor, unsigned pinSwitch) :
    pinMotor(pinMotor),
    pinSwitch(pinSwitch)
  {

  }

  void setup() {
    pinMode(pinMotor, OUTPUT);
    setMotor(false);
    pinMode(pinSwitch, INPUT);
  }

  void setMotor(bool state) {
    digitalWrite(pinMotor, state ? HIGH : LOW);
  }

  bool wasLaunched() {
    return !digitalRead(pinSwitch);
  }

  bool waitShootTimeout(unsigned timeout) {
    const unsigned start = millis();
    bool wasRunning = false;

    while (true) {
      // check timeout
      const unsigned tdiff = millis() - start;
      if (tdiff > timeout) {
        return false;
      }

      if (!wasRunning) {
        if (!wasLaunched()) {
          // motor moved out of initial position
          wasRunning = true;
        }
      } else {
        // motor is moving
        if (wasLaunched()) {
          // missile launched
          break;
        }
      }
    }

    return true; // no timeout
  }

  void shootOne() {
    setMotor(true);
    const bool ok = waitShootTimeout(1000);
    if (ok) {
      // wait to move motor out of triggering position
      delay(10);
    }
    setMotor(false);
  }

  void shootMany(unsigned count, unsigned shootDelay_ms=0) {
    for (unsigned i = 0; i < count; i++) {
      shootOne();

      if (shootDelay_ms) {
        delay(shootDelay_ms);
      }
    }
  }

protected:
  unsigned pinMotor;
  unsigned pinSwitch;
};

/****************************************************
 *** LED Lighthouse                               ***
 ****************************************************/

template<unsigned DOUT, unsigned NUM_LEDS>
class Lighthouse
{
public:
    void setup(unsigned brightness)
    {
        FastLED.addLeds<WS2811, DOUT, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
        FastLED.setBrightness(brightness);

        off();
        setManual(leds);
        FastLED.show();
    }

    void loop()
    {
        static unsigned last_run = 0;

        const unsigned now = millis();

        if ((now - last_run) >= TICK_DELAY_ms) {
            last_run = now;
            tick();
        }
    }

    void tick()
    {
        switch (currentMode) {
        case Mode::INIT:
            init();
            break;

        case Mode::PREPARE:
            manual();
            break;

        case Mode::READY:
            rainbowWithGlitter();
            break;

        case Mode::GAME:
            rainbow();
            break;

        case Mode::RAINBOW:
            rainbow();
            break;

        case Mode::RAINBOW_GLITTER:
            rainbowWithGlitter();
            break;

        case Mode::CONFETTI:
            confetti();
            break;

        case Mode::SINELON:
            sinelon();
            break;

        case Mode::BPM:
            bpm();
            break;

        case Mode::JUGGLE:
            juggle();
            break;

        case Mode::OFF:
        default:
            off();
        }

        if (currentMode == Mode::READY) {
            const bool blink = ((millis() - game_start_time) % 1000) < 500;
            if (blink) {
                leds[2] = manualLeds[2];
            }
        }

        if (currentMode == Mode::GAME) {
            const bool blink = ((millis() - game_start_time) % 2000) < 1000;
            if (blink) {
                leds[2] = manualLeds[2];
            }
        }

        FastLED.show();

        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
    }

    void nextPattern()
    {
        unsigned cur = static_cast<unsigned>(currentMode);
        unsigned mode_count = static_cast<unsigned>(Mode::count);

        currentMode = static_cast<Mode>((cur + 1) % mode_count);
    }

    void off()
    {
        for (unsigned i = 0; i < NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }
    }

    void init()
    {
        // front LED
        fill_rainbow( leds+2, 1, gHue, 7);

        // others manually
        for (unsigned i = 0; i < NUM_LEDS; i++) {
            if (i != 2) {
                leds[i] = manualLeds[i];
            }
        }
    }

    void manual()
    {
        for (unsigned i = 0; i < NUM_LEDS; i++) {
            leds[i] = manualLeds[i];
        }
    }

    void commitManual()
    {
        manual();
        FastLED.show();
    }

    void rainbow()
    {
        // FastLED's built-in rainbow generator
        fill_rainbow( leds, NUM_LEDS, gHue, 7);
    }

    void rainbowWithGlitter()
    {
        // built-in FastLED rainbow, plus some random sparkly glitter
        rainbow();
        addGlitter(80);
    }

    void addGlitter( fract8 chanceOfGlitter)
    {
        if( random8() < chanceOfGlitter) {
            leds[ random16(NUM_LEDS) ] += CRGB::White;
        }
    }

    void confetti()
    {
        // random colored speckles that blink in and fade smoothly
        fadeToBlackBy( leds, NUM_LEDS, 10);
        int pos = random16(NUM_LEDS);
        leds[pos] += CHSV( gHue + random8(64), 200, 255);
    }

    void sinelon()
    {
        // a colored dot sweeping back and forth, with fading trails
        fadeToBlackBy( leds, NUM_LEDS, 20);
        int pos = beatsin16(13,0,NUM_LEDS);
        leds[pos] += CHSV( gHue, 255, 192);
    }

    void bpm()
    {
        // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
        uint8_t BeatsPerMinute = 62;
        CRGBPalette16 palette = PartyColors_p;
        uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
        for( unsigned i = 0; i < NUM_LEDS; i++) { //9948
            leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
        }
    }

    void juggle() {
        // eight colored dots, weaving in and out of sync with each other
        fadeToBlackBy( leds, NUM_LEDS, 20);
        byte dothue = 0;
        for( unsigned i = 0; i < 8; i++) {
            leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
            dothue += 32;
        }
    }

    void setManual(CRGB ledValues[])
    {
        for (unsigned i = 0; i < NUM_LEDS; i++) {
            manualLeds[i] = ledValues[i];
        }
    }

    void setManual(unsigned index, CRGB color)
    {
        if (index < NUM_LEDS) {
            manualLeds[index] = color;
        }
    }

    void setManual(CRGB ledValue)
    {
        for (unsigned i = 0; i < NUM_LEDS; i++) {
            manualLeds[i] = ledValue;
        }
    }

    enum class Mode {
        OFF,
        INIT,
        PREPARE,
        READY,
        GAME,
        RAINBOW,
        RAINBOW_GLITTER,
        CONFETTI,
        SINELON,
        BPM,
        JUGGLE,
        count
    };

    void setMode(Mode mode)
    {
        currentMode = mode;
    }

    Mode getMode()
    {
        return currentMode;
    }

protected:
    static constexpr unsigned FPS = 50, TICK_DELAY_ms = 1000/FPS;

    Mode currentMode = Mode::OFF;

    CRGB leds[NUM_LEDS];
    CRGB manualLeds[NUM_LEDS];

    uint8_t gHue = 0; // rotating "base color" used by many of the patterns
};

/****************************************************
 *** Simple button interface                      ***
 ****************************************************/

class Button
{
public:
    Button(const char *topic, const unsigned pin, const bool inverted=true) :
        pin(pin),
        inverted(inverted),
        pub_button({topic, &pushed_msg})
    {
        pinMode(pin, INPUT_PULLUP);

        last_reading = state = getRaw();
    }

    void setup(ros::NodeHandle *nh)
    {
        nh->advertise(pub_button);
    }

    bool getRaw()
    {
        if (inverted) {
            return !digitalRead(pin);
        } else {
            return digitalRead(pin);
        }
    }

    bool get() {
        return state;
    }

    void loop()
    {
        const bool reading = getRaw();

        if (last_reading != reading){
            last_debounce_time = millis();
            published = false;
        }

        //if the button value has not changed for the debounce delay, we know its stable
        if (!published && (millis() - last_debounce_time) > debounce_delay) {
            state = reading;
            pushed_msg.data = reading;
            pub_button.publish(&pushed_msg);
            published = true;
        }

        last_reading = reading;
    }

protected:
    const unsigned pin;
    const bool inverted;

    bool state;
    bool last_reading;
    unsigned long last_debounce_time = 0;
    unsigned long debounce_delay = 50;

    bool published = false;
    std_msgs::Bool pushed_msg;
    ros::Publisher pub_button;
};

class InputController
{
public:
    InputController(const unsigned buttonUp,
                    const unsigned buttonDown,
                    const unsigned buttonStart) :
        up({"up",    buttonUp}),
        down({"down",  buttonDown}),
        start({"start", buttonStart})
    {
    }

    void setup(ros::NodeHandle *nh)
    {
        up.setup(nh);
        down.setup(nh);
        start.setup(nh);
    }

    void loop()
    {
        up.loop();
        down.loop();
        start.loop();
    }

    bool getUp()
    {
        return up.get();
    }

    bool getDown()
    {
        return down.get();
    }

    bool getStart()
    {
        return start.get();
    }

protected:
    Button up, down, start;
};

/****************************************************
 *** Configuration                                ***
 ****************************************************/

MissileLauncher missile(15 /*motor_en: A1*/, 16 /*switch: A2*/);

typedef Lighthouse<6 /*WS2812 DOut: D6*/, 5 /*LEDs*/> CrashLighthouse;
CrashLighthouse lighthouse;

InputController inputs(17 /*up/red: A3*/, 18 /*down: A4*/, 12 /*start: D12*/);

static inline CRGB charToColor(const char c)
{
    switch (c) {
    case 'r':
        return CRGB::Red;
    case 'g':
        return CRGB::Green;
    case 'b':
        return CRGB::Blue;
    case 'y':
        return CRGB::Yellow;
    case 'c':
        return CRGB::Cyan;
    case 'm':
        return CRGB::Magenta;
    case 'l':
        return CRGB::Lime;
    case 'p':
        return CRGB::Pink;
    case 'w':
        return CRGB::White;

    case '0':
    default:
        return CRGB::Black;
    }
}

static void led_cb(const std_msgs::String& ledColors)
{
    if (strlen((const char*) ledColors.data) == 5) {
        for (int i=0; i < 5; i++) {
            lighthouse.setManual(i, charToColor(ledColors.data[i]));
        }
    }
}

static void rocket_cb(const std_msgs::Bool &doLaunch)
{
    if (doLaunch.data == true) {
        missile.shootOne();
    }
}

void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(led_sub);
    nh.subscribe(rocket_sub);
    inputs.setup(&nh);

    // full brightness
    lighthouse.setup(255);
    missile.setup();

    lighthouse.setMode(CrashLighthouse::Mode::INIT);
}

void loop() {
    // read buttons
    inputs.loop();

    // trigger pose reset
    if (inputs.getUp()) {
        game_state = 0;

        lighthouse.setMode(CrashLighthouse::Mode::PREPARE);
        lighthouse.setManual(CRGB::Red);
        lighthouse.commitManual();
    }

    // we run an own state machine for pulling the start jumper and launching
    // the missile after the game.
    if (millis() > 500) {
        static bool last_state = inputs.getStart();

        const bool new_state = inputs.getStart();
        const bool changed = (last_state != new_state);
        const unsigned long now = millis();

        switch (game_state) {
        case 0:
            if (changed && true == new_state) {
                // inserted
                game_state = 1;

                lighthouse.setMode(CrashLighthouse::Mode::READY);
            }
            break;

        case 1:
            if (changed && false == new_state) {
                // game start
                game_state = 2;
                game_start_time = now;

                lighthouse.setMode(CrashLighthouse::Mode::GAME);
            }
            break;

        case 2:
            // game running

            if ((now - game_start_time) > 91000) {
                // game over, launch missile after 91s to be safe
                game_state = 3;

                for (unsigned i = 0; i < 5; i++) {
                    lighthouse.setManual(i, i&1 ? CRGB::White : CRGB::Red);
                }
                lighthouse.commitManual();
                missile.shootMany(3, 200);
                delay(200);

                for (unsigned i = 0; i < 5; i++) {
                    lighthouse.setManual(i, i&1 ? CRGB::Red : CRGB::White);
                }
                lighthouse.commitManual();
                missile.shootMany(3, 100);
            }
            break;

        case 3:
            // game done
            if (false == new_state) {
                // back to start if jumper released
                game_state = 0;

                lighthouse.setMode(CrashLighthouse::Mode::RAINBOW);
            }
            break;
        }

        last_state = new_state;
    }

    // update LEDs
    lighthouse.loop();

    nh.spinOnce();
}
