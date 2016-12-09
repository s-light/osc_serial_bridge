/******************************************************************************

    serial2DMX
        simple serial to dmx bridge

        serial format
            dmxSS/_____ [2byte size] [channels each 1byte]


    hardware
        DMX Output:  UART (Pin 0 RX, Pin 1 TX, Pin 2 EN )  DMX-Transeiver PCB
        pcb see schematics.


    libraries used
        ~ DMXSerial
            Copyright (c) 2005-2012 by Matthias Hertel,
            http://www.mathertel.de
        ~ slight_DebugMenu

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/


   changelog / history
    30.11.2016 15:46 based on Analog2DMX_USART_06ch_v03

    TO DO:
        ~ enjoy the sun


******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2015 Stefan Krüger

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
******************************************************************************/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

#include <slight_DebugMenu.h>
#include <DMXSerial.h>


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Info
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sketchinfo_print(Print &out) {
    out.println();
    //             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|                       ^ ^                      |"));
    out.println(F("|                      (0,0)                     |"));
    out.println(F("|                      ( _ )                     |"));
    out.println(F("|                       \" \"                      |"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("| serial2DMX.ino"));
    out.println(F("|   simple serial to dmx bridge"));
    out.println(F("|"));
    out.println(F("| This Sketch has a debug-menu:"));
    out.println(F("| send '?'+Return for help"));
    out.println(F("|"));
    out.println(F("| dream on & have fun :-)"));
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|"));
    //out.println(F("| Version: Nov 11 2013  20:35:04"));
    out.print(F("| version: "));
    out.print(F(__DATE__));
    out.print(F("  "));
    out.print(F(__TIME__));
    out.println();
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    //out.println(__DATE__); Nov 11 2013
    //out.println(__TIME__); 20:35:04
}


// Serial.print to Flash: Notepad++ Replace RegEx
//     Find what:        Serial.print(.*)\("(.*)"\);
//     Replace with:    Serial.print\1\(F\("\2"\)\);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions (global)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Debug Output

boolean infoled_state = 0;
const byte infoled_pin = 9; //D9

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = 0;
boolean debugOut_LiveSign_LED_Enabled = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu

// slight_DebugMenu(Stream &in_ref, Print &out_ref, uint8_t input_length_new);
slight_DebugMenu myDebugMenu(Serial, Serial, 15);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// DMXSerial

const uint16_t DMX_channel_count = 40;
const byte DMX_direction_PIN = 2;

bool DMX_valid = false;

uint16_t DMX_channel_start = 0;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//

// const uint8_t cbFaderCount = 6;
//
// // Pin for Analog In reading
// const uint8_t pin_Fader[cbFaderCount]	= {A0, A1, A2, A3, A4, A5};
// const uint8_t pin_VCC					= A6; // DigitalPin 4
//
// // Raw Values
// uint16_t wValue_Fader[cbFaderCount] = {0, 0, 0, 0, 0, 0};
// uint16_t wValue_VCC = 0;
//
// // Map
// const uint8_t bMap_Count			= 4;
// uint16_t wMap_InFader [bMap_Count]	= {    0,   40, 1020, 1023};
// uint16_t wMap_OutDMX  [bMap_Count]	= {    0,   10,  255,  255};


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// other things..

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things

// freeRam found at
// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
// posted by mrburnette
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// multiMap
/**
  * MultiMap
  *
  * http://arduino.cc/playground/Main/MultiMap
  * My calibrated distance sensor - SHARP 2Y0A02 F 9Y
  *   // out[] holds the values wanted in cm
  *   uint16_t out[] = {150,140,130,120,110,100, 90, 80, 70, 60, 50, 40, 30, 20};
  *   // in[] holds the measured analogRead() values for defined distances
  *   uint16_t in[]  = { 90, 97,105,113,124,134,147,164,185,218,255,317,408,506};
  *   val = analogRead(A0);
  *   cm = multiMap(val, in, out, 14);
  *
  *  modified by Stefan Krueger for 16bit uint16_t datatype
  **/
uint16_t multiMap(uint16_t val, uint16_t* _in, uint16_t* _out, size_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return map(val, _in[pos-1], _in[pos], _out[pos-1], _out[pos]);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// init

void DMX_init(Print &out) {
    // init all subparts

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup DMXSerial
        out.println(F("setup DMXSerial:")); {
            // pin for direction
            pinMode(DMX_direction_PIN, OUTPUT);

            // set to receive mode
            // out.println(F("\t set direction pin to Low = 'Receive' "));
            // digitalWrite(DMX_direction_PIN, LOW);
            // out.println(F("\t init as DMXReceiver"));
            // DMXSerial.init(DMXReceiver);

            // set to send mode
            out.println(F("\t set direction pin to High = 'Send' "));
            digitalWrite(DMX_direction_PIN, HIGH);
            out.println(F("\t init as DMXController"));
            DMXSerial.init(DMXController);
            DMXSerial.maxChannel(DMX_channel_count);

            // out.println(F("\t set first Pixel to Yellow"));
            // DMXSerial.write(10, 255);
            // DMXSerial.write(11, 255);
            // DMXSerial.write(12, 1);
            // read dmx values
            // DMXSerial.read(1)
        }
        out.println(F("\t finished."));
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup ??
        // out.println(F("setup fader:")); {
        //     fader.begin();
        // }
        // out.println(F("\t finished."));

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // -
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// serial DMX receiveing


void handle_dmxascii(char *command){
    // command = 'dmx/512:255'
    // Serial.print(F("\t received dmx data: "));

    // find '/'
    // char *pointer_nextsection = strtok(&command[0], '/');
    char *pointer_nextsection = strchr(&command[0], '/') + 1;

    // convert part of string to int
    // (up to first char that is not a number)
    uint16_t channel = atoi(pointer_nextsection)+1;

    // find value after ':'
    // pointer_nextsection = strtok(NULL, ':');
    pointer_nextsection = strchr(pointer_nextsection, ':') + 1;

    // extract value
    uint8_t value = atoi(pointer_nextsection);

    // Serial.print(channel);
    // Serial.print(F(" : "));
    // Serial.print(value);
    // Serial.println();

    DMXSerial.write(channel, value);
}

void handle_dmxbinary(char *command){
    // command = 'DmxSS/_____'
    // SS = 2byte, size
    uint16_t data_size = 0;
    data_size = command[3];
    data_size |= command[4] << 8;
    Serial.print("data_size: ");
    Serial.println(data_size);
    // check for size data divider
    if (command[5] == '/') {
        char *data_start_pointer = &command[5];
        // data_start_pointer points to the start of the array.
        uint8_t * raw_buffer = DMXSerial.getBuffer();
        slight_DebugMenu::print_uint8_array(
            Serial,
            data_start_pointer,
            data_size
        );
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

// Main Menu
void handleMenu_Main(slight_DebugMenu *pInstance) {
    Print &out = pInstance->get_stream_out_ref();
    char *command = pInstance->get_command_current_pointer();
    // out.print("command: '");
    // out.print(command);
    // out.println("'");
    switch (command[0]) {
        case 'h':
        case 'H':
        case '?': {
            // help
            out.println(F("____________________________________________________________"));
            out.println();
            out.println(F("Help for Commands:"));
            out.println();
            out.println(F("\t '?': this help"));
            out.println(F("\t 'i': sketch info"));
            out.println(F("\t 'y': toggle DebugOut livesign print"));
            out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            out.println(F("\t 'x': tests"));
            out.println();
            out.println(F("\t 'A': Show 'HelloWorld' "));
            out.println(F("\t 'f': DemoFadeTo(ID, value) 'f1:65535'"));
            out.println(F("\t 'd': dmx channels 'dmx/512:255'"));
            out.println(F("\t 'D': Dmx channels B=byte 'DmxSS/[BBB...]'"));
            out.println();
            out.println(F("____________________________________________________________"));
        } break;
        case 'i': {
            sketchinfo_print(out);
        } break;
        case 'y': {
            out.println(F("\t toggle DebugOut livesign Serial:"));
            debugOut_LiveSign_Serial_Enabled = !debugOut_LiveSign_Serial_Enabled;
            out.print(F("\t debugOut_LiveSign_Serial_Enabled:"));
            out.println(debugOut_LiveSign_Serial_Enabled);
        } break;
        case 'Y': {
            out.println(F("\t toggle DebugOut livesign LED:"));
            debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
            out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
            out.println(debugOut_LiveSign_LED_Enabled);
        } break;
        case 'x': {
            // get state
            out.println(F("__________"));
            out.println(F("Tests:"));

            out.println(F("nothing to do."));

            // uint16_t wTest = 65535;
            uint16_t wTest = atoi(&command[1]);
            out.print(F("wTest: "));
            out.print(wTest);
            out.println();

            out.print(F("1: "));
            out.print((byte)wTest);
            out.println();

            out.print(F("2: "));
            out.print((byte)(wTest>>8));
            out.println();

            out.println();

            out.println(F("__________"));
        } break;
        //---------------------------------------------------------------------
        case 'A': {
            out.println(F("\t Hello World! :-)"));
        } break;
        case 'f': {
            out.print(F("\t DemoFadeTo "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t id = atoi(&command[1]);
            // convert single character to int representation
            // uint8_t id = &command[1] - '0';
            out.print(id);
            out.print(F(" : "));
            uint16_t value = atoi(&command[3]);
            out.print(value);
            out.println();
            //demo_fadeTo(id, value);
            out.println(F("\t demo for parsing values --> finished."));
        } break;
        case 'd': {
            handle_dmxascii(&command[0]);
        } break;
        case 'D': {
            handle_dmxbinary(&command[0]);
        } break;
        //---------------------------------------------------------------------
        default: {
            if(strlen(command) > 0) {
                out.print(F("command '"));
                out.print(command);
                out.println(F("' not recognized. try again."));
            }
            pInstance->get_command_input_pointer()[0] = '?';
            pInstance->set_flag_EOC(true);
        }
    } // end switch

    // end Command Parser
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// other things..




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs

        //LiveSign
        pinMode(infoled_pin, OUTPUT);
        digitalWrite(infoled_pin, HIGH);

        // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // init DMX
    DMX_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // wait for arduino IDE to release all serial ports after upload.
            delay(2000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup XXX1

        // Serial.print(F("# Free RAM = "));
        // Serial.println(freeRam());
        //
        // Serial.println(F("setup XXX1:")); {
        //
        //     Serial.println(F("\t sub action"));
        // }
        // Serial.println(F("\t finished."));

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // inits
    init();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_user_EOC_char(';');
        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("Loop:"));

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // menu input
        myDebugMenu.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // timed things

        // every XXXXms
        // if ( ( millis() - ulTimeStamp_LastAction ) > cwUpdateInterval) {
        //     ulTimeStamp_LastAction =  millis();
        //     // do something
        // }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // debug output

        if (
            (millis() - debugOut_LiveSign_TimeStamp_LastAction) >
            debugOut_LiveSign_UpdateInterval
        ) {
            debugOut_LiveSign_TimeStamp_LastAction = millis();

            if ( debugOut_LiveSign_Serial_Enabled ) {
                Serial.print(millis());
                Serial.print(F("ms;"));
                Serial.print(F("  free RAM = "));
                Serial.println(freeRam());
            }

            if ( debugOut_LiveSign_LED_Enabled ) {
                infoled_state = ! infoled_state;
                if (infoled_state) {
                    //set LED to HIGH
                    digitalWrite(infoled_pin, HIGH);
                } else {
                    //set LED to LOW
                    digitalWrite(infoled_pin, LOW);
                }
            }

        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // other things

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
