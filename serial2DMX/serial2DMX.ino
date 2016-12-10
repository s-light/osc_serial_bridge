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
// analog 2 digital mapping

const uint8_t fader_count = 6;

// Pin for Analog In reading
const uint8_t fader_pins[fader_count]  = {A0, A1, A2, A3, A4, A5};
const uint8_t voltage_pin              = A6; // DigitalPin 4

// Raw Values
uint16_t fader_value_raw[fader_count] = {0, 0, 0, 0, 0, 0};
// uint8_t  fader_value_mapped[fader_count] = {0, 0, 0, 0, 0, 0};
uint16_t fader_voltage_value = 0;

// Map
const uint8_t map_count      = 4;
uint16_t map_InFader [map_count]  = {    0,   40, 1010, 1023};
uint16_t map_OutDMX  [map_count]  = {    0,   10,  255,  255};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// dataout: output fader values over serial

uint32_t dataout_timestamp_lastsend = 0;
uint16_t dataout_refresh_interval = 5000;

uint16_t dataout_values[fader_count] = {0, 0, 0, 0, 0, 0};
// uint8_t dataout_values[fader_count] = {0, 0, 0, 0, 0, 0};

const uint8_t dataout_filter_count = 20;
uint8_t dataout_filter_index = 0;
uint16_t dataout_filter[fader_count][dataout_filter_count];

bool dataout_enabled = true;

bool debugOut_Dataout_Serial_Enabled = false;

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
            out.println(F("\t 'X': toggle debug dataout print"));
            out.println(F("\t 'x': tests"));
            out.println();
            out.println(F("\t 'p': print current fader values. "));
            out.println(F("\t 's': send current fader values. "));
            out.println(F("\t 'S': toggle send dataout. "));
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
        case 'X': {
            out.println(F("\t toggle Debugout dataout:"));
            debugOut_Dataout_Serial_Enabled = !debugOut_Dataout_Serial_Enabled;
            out.print(F("\t debugOut_Dataout_Serial_Enabled:"));
            out.println(debugOut_Dataout_Serial_Enabled);
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
        case 'p': {
            out.println(F("\t Current Values:"));

            // temporary deactivate dataout_enabled
            // save current data out state:
            bool temp = dataout_enabled;
            dataout_enabled = false;
            // update calculations
            dataout_update();
            // restore enable state
            dataout_enabled = temp;

            out.print("fv:");
            // slight_DebugMenu::print_uint8_array(
            //     out,
            //     dataout_values,
            //     fader_count
            // );
            slight_DebugMenu::print_uint16_array(
                out,
                dataout_values,
                fader_count
            );

            // endline
            out.println();
        } break;
        case 's': {
            // temporary deactivate dataout_enabled
            // save current data out state:
            bool temp = dataout_enabled;
            dataout_enabled = false;
            // update calculations
            dataout_update();
            // restore enable state
            dataout_enabled = temp;

            dataout_print(out);
        } break;
        case 'S': {
            out.println(F("\t toggle dataout:"));
            dataout_enabled = !dataout_enabled;
            out.print(F("\t dataout_enabled:"));
            out.println(dataout_enabled);
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
// faders

void readAnalogFader() {
    //unsigned long ulTimeStamp = millis();

      // Read VCC; -13=correction: difference between VCC and max Fader Voltage.
      fader_voltage_value = analogRead(voltage_pin) - 13 ;
      // read fader
      // uint16_t analog_in_raw = 0;
      for (uint8_t fader_index = 0; fader_index < fader_count ; fader_index++) {
        uint16_t analog_in_raw = analogRead(fader_pins[fader_index]);
        // no remapping.
        fader_value_raw[fader_index] = analog_in_raw;
        // remap to accomendate for input-voltage range.
        // unsigned long lTemp = (unsigned long)analog_in_raw * 1024;
        // fader_value_raw[fader_index] = (uint16_t) (lTemp / fader_voltage_value);
      }

    //unsigned long ulTimeStamp2 = millis();
    // Serial.print("analog_in_raw: ");
    // Serial.print(analog_in_raw);

    // Serial.print("VCC: ");
    // Serial.print(fader_voltage_value);

    // Serial.print(", ");
    // Serial.print( (fader_voltage_value*50) / 1024 );
    // Serial.print("V ");

    // Serial.print(" ms:");
    // Serial.println(ulTimeStamp2 - ulTimeStamp);

    // DEBUG AUSGABE
    // Serial.print("; Fader: ");
    // for (uint8_t fader_index = 0; fader_index < fader_count ; fader_index++) {
    //
    //   Serial.print(fader_value_raw[fader_index]);
    //   Serial.print(", ");
    // }
    // Serial.println();

      //Serial.print("A0: ");
      //Serial.println(analogRead(A0));

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// dataout

uint8_t dataout_map_value(uint16_t value_raw) {
    uint8_t value =  multiMap(
        value_raw,
        map_InFader,
        map_OutDMX,
        map_count
    );
    return value;
}


uint16_t dataout_filter_channel(uint8_t fader_id, uint16_t value_new) {
   uint16_t value_result = 0;
   // median filter
   // based on
   // http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/
   // mode = median + averag
   // median
   // int sortedValues[NUM_READS];
   // for(int i=0;i<NUM_READS;i++){
   //   int value = analogRead(sensorpin);
   //   int j;
   //   if(value<sortedValues[0] || i==0){
   //      j=0; //insert at first position
   //   }
   //   else{
   //     for(j=1;j<i;j++){
   //        if(sortedValues[j-1]<=value && sortedValues[j]>=value){
   //          // j is insert position
   //          break;
   //        }
   //     }
   //   }
   //   for(int k=i;k>j;k--){
   //     // move all values higher than current reading up one position
   //     sortedValues[k]=sortedValues[k-1];
   //   }
   //   sortedValues[j]=value; //insert current reading
   // }
   // averag
   //return scaled mode of 10 values
   // float returnval = 0;
   // for(int i=NUM_READS/2-5;i<(NUM_READS/2+5);i++){
   //   returnval +=sortedValues[i];
   // }
   // returnval = returnval/10;
   // ##########################
   // 480,  477,  476,  476,  476,  475,  476,  475,  475,  478,  475,  477,  480,  475,  476,  476,  474,  478,  473,  475,  472,  478,  472,  470,  476,  475,  479,  476,  476,  473,  474,  475,  473,  476,  477,  474,  472,  474,  477,  474,  476,  477,  478,  473,  472,  475,  476,  475,  474,    0



   // median
   uint8_t insert_pos = 0;
   if ( (value_new < dataout_filter[fader_id][0]) || (dataout_filter_index > 0) ) {
      insert_pos = 0;
   } else {
      while (
         !((dataout_filter[fader_id][insert_pos-1] <= value_new) &&
         (dataout_filter[fader_id][insert_pos] >= value_new))
      ) {
         /* code */
      }


      for (insert_pos = 1; insert_pos < dataout_filter_index; insert_pos++) {
         if (
            (dataout_filter[fader_id][insert_pos-1] <= value_new) &&
            (dataout_filter[fader_id][insert_pos] >= value_new)
         ) {
            // insert_pos is the correct position.
            break;
         }
      }
   }
   // move all values higher than current insert position up one position.
   for (size_t k = dataout_filter_index; k > insert_pos; k--) {
      dataout_filter[fader_id][k] = dataout_filter[fader_id][k-1];
   }
   // set new value
   dataout_filter[fader_id][insert_pos] = value_new;

   // increase index
   dataout_filter_index = dataout_filter_index +1;
   // wrap around
   // dataout_filter_index = dataout_filter_index % dataout_filter_count;
   if (dataout_filter_index >= dataout_filter_count) {
      dataout_filter_index = 0;
   }


   // averag
   // get center area
   uint16_t value_sum = 0;
   // uint8_t filter_count_one_third = dataout_filter_count/3;
   uint8_t filter_count_one_fifth = dataout_filter_count/5;
   for (
      // size_t i = filter_count_one_third;
      // i < (filter_count_one_third*2);
      size_t i = (filter_count_one_fifth*2);
      i < (filter_count_one_fifth*3);
      i++
   ) {
      value_sum += dataout_filter[fader_id][i];
   }
   // value_result = value_sum / filter_count_one_third;
   value_result = value_sum / filter_count_one_fifth;

   return value_result;
}

bool dataout_update() {
    bool flag_changed = false;

    for (size_t fader_index = 0; fader_index < fader_count; fader_index++) {
        // raw values (10bit)
        // size_t value = fader_value_raw[fader_index];
        // mapped values (8bit)
        // Serial.print(i);
        // Serial.print(" value new: ");
        // Serial.print(value);
        // Serial.println();
        // Serial.print(i);
        // Serial.print(" value old: ");
        // Serial.print(value);
        // Serial.println();

        size_t value = fader_value_raw[fader_index];

        // size_t value = dataout_map_value(fader_value_raw[fader_index]);

        // size_t value = dataout_filter_channel(
        //     fader_index,
        //     fader_value_raw[fader_index]
        // );

        // value = dataout_map_value(value);

        // value = dataout_filter_channel(i, value);

        if (value != dataout_values[fader_index]) {
            dataout_values[fader_index] = value;
            flag_changed = true;
            if (dataout_enabled) {
                send_fader_message(
                    Serial,
                    fader_index,
                    value
                );
            }
        }
    }
    return flag_changed;
}

void send_fader_message(Print &out, uint8_t id, uint16_t value) {
    out.print("/fader/");
    out.print(id);
    out.print(":");
    out.print(value);
    out.println();
}

void dataout_print(Print &out) {
    // out.print("fv:");
    //
    // // slight_DebugMenu::print_uint8_array(out, dataout_values , fader_count);
    // slight_DebugMenu::print_uint16_array(out, dataout_values , fader_count);
    //
    // // endline
    // out.println();
    for (size_t fader_index = 0; fader_index < fader_count; fader_index++) {
        send_fader_message(
            out,
            fader_index,
            dataout_values[fader_index]
        );
    }
}

void dataout_handle() {
    if (dataout_enabled) {
        if(dataout_update()){
            dataout_timestamp_lastsend = millis();
            // will be handled on channel basis inside of dataout_update
            // dataout_print(Serial);
        } else {
            uint32_t duration = millis() - dataout_timestamp_lastsend;
            if (duration > dataout_refresh_interval) {
                dataout_timestamp_lastsend = millis();
                dataout_print(Serial);
            }
        }
    }
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
    // init DMX

        DMX_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup faders

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        Serial.println(F("fader / analog inputs:")); {
            Serial.println(F("\t nothing to do. defaults to input."));
        }
        Serial.println(F("\t finished."));


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
    // analog in
        readAnalogFader();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // dataout
        dataout_handle();

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

            if ( debugOut_Dataout_Serial_Enabled ) {
                Serial.println(F("fader_value_raw"));
                slight_DebugMenu::print_uint16_array(
                   Serial,
                   fader_value_raw,
                   fader_count
                );
                Serial.println();
                Serial.println(F("dataout_values"));
                slight_DebugMenu::print_uint16_array(
                   Serial,
                   dataout_values,
                   fader_count
                );
                Serial.println();
                // Serial.println(F("dataout_filter"));
                // for (size_t i = 0; i < fader_count; i++) {
                //    slight_DebugMenu::print_uint16_array(
                //       Serial,
                //       dataout_filter[i] ,
                //       dataout_filter_count
                //    );
                //    Serial.println();
                // }
                Serial.println();
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
