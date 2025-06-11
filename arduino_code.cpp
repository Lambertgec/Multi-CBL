#include <Arduino.h>
#include <math.h>
#include <string.h>

// Variables to change manually depending on setup
const int pinsUsed = 8; //number of analog pins used
const int analogPins[pinsUsed] = {A0, A1, A2, A3, A4, A5, A6, A7}; //analog pins to read (for sensors)
const int LED_PIN[3] = {9, 10, 11}; //pin for the LED, as its RGB it will also probably have 3 pins?
const int buzzerPins[2] = {2, 3}; //pins for the buzzers (one for each sensor)
const int serial_baud_rate = 9600; // Serial communication baud rate
const int exit_button_pin = 13; // change this to wherever it is

// Variables we have to play around with
const int window_size = 20; //size of the rolling window for each sensor
const float consistency_threshold = 0.65; //amount of readings that must be < MAD*multiplier of median of the rolling window 
const int debounce_length = 3; //amount of consecutive readings that must be consistent
const float mad_multiplier = 3.0; //multiplier for filtering noise
const unsigned long stability_seconds = 5 * 1000; // amount of time in seconds that the posture must be stable for
const int delay_time = 500; //delay between readings in milliseconds, adjust as needed
const int pulse_duration = 100; // duration of the pulse for the light nudge in milliseconds
const float at_chair_threshold = 0.05; // threshold for determining if a person is at the chair, so minimum value when sitting down
const float drift_threshold = 0.15; // the value the median of readings can drift from the reference (which is the median of when the posture was stable)

// Global variables
bool debounce_buffer[debounce_length] = {true, true, true}; //buffer for consistency checks (noise filtering)
float reference_median[pinsUsed] = {0}; // a reference median is saved for each sensor when the posture becomes stable, and is used later to detect change over time
float datas[pinsUsed][window_size]; //rolling window for each sensor
int data_idx[pinsUsed] = {0}; //current index for each sensor's rolling window
bool buffer_full[pinsUsed] = {false}; //whether there are rolling window data for each sensor
unsigned long last_unstable_time = 0; // last time the posture was unstable
const int timestamp = 10 * 1000; // timestamps for the light nudge function (in seconds)
bool reported = false; // whether a stable posture has been reported, for now no functionality
const int red_sensors[4] = {2, 3, 4, 5};
const int cyan_sensors[2] = {6, 7}; // sensors that trigger cyan light nudges

unsigned long color_set_time = 0; // time when the color was last set (for starting pulsing)
String last_color = ""; // last color set for the light nudge function
bool led_pulsing = false; // whether the LED is currently pulsing
unsigned long pulse_start_time = 0; // time when the pulse started, for determining when to turn on and off, this is done concurrently right now
void light_nudge(String color) {
  if (color != last_color) { //If the color has changed, for pulsing we want to take note of it so that we know when to start pulsing
    // Color changed, update immediately and reset timer
    last_color = color;
    color_set_time = millis();
    led_pulsing = false; // reset pulsing state
    if (color == "red")        light_values(100, 0, 0);
    else if (color == "orange")light_values(255, 165, 0);
    else if (color == "cyan")light_values(0, 100, 100);
    else                       Serial.println("Invalid color for blinking");
    return;
  }

  // Color is the same as before and enough time has passed since setting it
  if (millis() - color_set_time >= timestamp) {
    if (!led_pulsing) { //if its not yet pulsing we turn it off 
      light_values(0, 0, 0); // Turn off LED
      pulse_start_time = millis();
      led_pulsing = true;
    } else if (led_pulsing && (millis() - pulse_start_time >= pulse_duration)) { // if we started pulsing (so LED is off) and pulse duration has passed, we can turn it back on
      if (color == "red")        light_values(100, 0, 0); //red
      else if (color == "orange")light_values(255, 165, 0); //orange
      else if (color == "cyan")  light_values(0, 100, 100); //cyan
      else                       Serial.println("Invalid color for blinking");
      led_pulsing = false; // Ready for next pulse cycle
    }
  } else { // we might not need this but it doesnt hurt to keep it, in case not enough time passed we keep the same color
    // Keep the LED on with the current color
    if (color == "red")        light_values(100, 0, 0);
    else if (color == "orange")light_values(255, 165, 0);
    else if (color == "cyan")  light_values(0, 100, 100);
    else                       Serial.println("Invalid color for blinking");
    led_pulsing = false; // indicate that the LED is pulsing
  }
}

float median(float arr[]) { // Calculate the median of an array
  float temp[window_size]; //temporary array to hold the sorted values
  memcpy(temp, arr, window_size * sizeof(float)); //copy arr onto temp
  // Simple insertion sort
  for (int i = 1; i < window_size; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }
  if (window_size % 2 == 0) //return the middle element (median) of the sorted array
    return (temp[window_size/2 - 1] + temp[window_size/2]) / 2.0;
  else
    return temp[window_size/2];
}

float mad(float arr[], float med) { //returns the Median Absolute Deviation (MAD) of an array
  float deviations[window_size]; //array for the absolute deviations from the median
  for (int i = 0; i < window_size; i++)
    deviations[i] = fabs(arr[i] - med);
  return median(deviations); //return the median of the absolute deviations
}

void setup() {
  Serial.begin(serial_baud_rate); //this has to be the same as in platformio.ini OR in the serial monitor
  //pinMode(exit_button_pin, INPUT_PULLUP); // Use internal pull-up resistor for exit button
  for (int i = 0; i < 3; i++) {
    pinMode(LED_PIN[i], OUTPUT); // Set the LED pin as output
  }
  for (int i = 0; i < pinsUsed; i++) {
    pinMode(analogPins[i], INPUT); // Optional for clarity
    for (int j = 0; j < window_size; j++) {
      datas[i][j] = 0.0; //initialize rolling windows to 0.0
    }
  }  
  //pinMode(buzzerPins[0], OUTPUT); // Set buzzer pins as output
  //pinMode(buzzerPins[1], OUTPUT); // Set buzzer pins as output
  last_unstable_time = millis(); //initialize last unstable time
}

// this might have to be changed once its actually in the seating, because padding might cause unintentional small readings, also set the sensors to bottom 4
bool at_chair() {
  // Check if any of the sensors have a positive reading 
  if (datas[0][data_idx[0]] > at_chair_threshold || datas[1][data_idx[1]] > at_chair_threshold || // we have to make these the bottom 4 sensors
      datas[2][data_idx[2]] > at_chair_threshold || datas[3][data_idx[3]] > at_chair_threshold) {
    return true; // at least one sensor has a reading above the threshold, meaning a person is at the chair
  } 
  return false; // all sensors had a reading of 0, meaning no person is at the chair
}

// returns false if posture is acceptable, true if lower back and rear sensors are all mostly 0
bool incorrect_posture_red() { // we should check all the relevant sensors, not just first 4 so we have to change these to the corresponding ones
  for (int i = 0; i < 4; i++) { // Check lower back and rear sensors    
    float med = median(datas[red_sensors[i]]);
    float mad_val = mad(datas[red_sensors[i]], med);
    if (med > at_chair_threshold || mad_val > at_chair_threshold) { 
        return false;
    }
  }
  return true;
}

// returns false if posture is acceptable, true if upper back sensors are all mostly 0
bool incorrect_posture_cyan() { // we should check all the relevant sensors, not just last 2 so we have to change these to the corresponding ones
  for (int i = 0; i < 2; i++) { // Check upper back sensors
    float med = median(datas[cyan_sensors[i]]);
    float mad_val = mad(datas[cyan_sensors[i]], med);
    if (med > at_chair_threshold || mad_val > at_chair_threshold) {
        return false;
    }
  }
  return true;
}

// Change light values
void light_values(int red, int green, int blue) {
  analogWrite(LED_PIN[0], red);   // Set red LED value
  analogWrite(LED_PIN[1], green); // Set green LED value
  analogWrite(LED_PIN[2], blue);  // Set blue LED value
  Serial.print("LED values set to: R=");
  Serial.print(red);
  Serial.print(", G=");
  Serial.print(green);
  Serial.print(", B=");
  Serial.println(blue);
}

bool reference_set = false; // to keep track of whether the reference median has been set, technically same functionaly as last_debounced
bool last_debounced = false; // to keep track of the last debounced state, so we only update reference median when the posture becomes stable for the first time
void debounce(bool all_consistent) {
  if (!at_chair()) { // if no person is at the chair, we can immediately report that
    Serial.println("No person at the chair");
    light_values(150, 150, 150); // set it to white / gray, just for debugging 
    last_unstable_time = millis(); 
    return;
  }

  if (incorrect_posture_red()) { // we can immediately report all postures that we deem incorrect in incorrect_posture()
    // RED if sensors 2-5 are all mostly 0
    Serial.println("Red poor posture detected, should be changed");
    light_nudge("red"); // Blink red light  
    last_unstable_time = millis(); // update last unstable time
    return;
  } else if (incorrect_posture_cyan()) { // if sensors 6-7 are all mostly 0
    // CYAN if sensors 4-7 are all mostly 0
    Serial.println("Cyan poor posture detected, should be changed");
    light_nudge("cyan"); // Blink cyan light
    last_unstable_time = millis(); // update last unstable time
    return;
  }

  // Debounce logic - after any of the sensors were inconsistent, the next debounce_length readings must be consistent for all sensors (currently)
  for (int i = 0; i < debounce_length - 1; i++) 
    debounce_buffer[i] = debounce_buffer[i + 1];
  debounce_buffer[debounce_length - 1] = all_consistent;

  bool debounced = true;
  for (int i = 0; i < debounce_length; i++) {
    if (!debounce_buffer[i]) { // if any of the readings in any of the sensor buffers were false (inconsistent (large change)) 
      debounced = false;  //then debouncing did not happen yet (as we wait for buffers to only have true (stable)) values
      break;
    }
  }

  if (debounced) { // set consecutive readings were stable
    if (!last_debounced) { // For each pins, only set reference values (medians) when the posture becomes stable for the first time
        for (int i = 0; i < pinsUsed; i++) {
            float window[window_size];
            for (int j = 0; j < window_size; j++)
                window[j] = datas[i][(data_idx[i] + j) % window_size];
            reference_median[i] = median(window);
          }
        reference_set = true; // Set reference median for each sensor
        last_debounced = true;
    }
    if (millis() - last_unstable_time > stability_seconds) {
        // ORANGE if there was a timeout with sensors 0-3 not all being 0
        Serial.println("Posture timeout");
        light_nudge("orange"); // Set LED to orange
    } else {
      // GREEN if there was no timeout and acceptable posture
      Serial.println("Posture can still be kept and is stable");
      light_values(0, 100, 0); // Set LED to green 
    }
  } else { // if at least one of the last debounce_length readings was inconsistent, we can report that the posture is not stable 
    // brown when not stable and also "acceptable" posture (meaning lowerback 2 sensors and seat 2 rear sensors are not all 0)
    Serial.println("Posture not stable, but acceptable");
    last_debounced = false; // reset debounced state
    reference_set = false; // reset reference set state as we expect a new reference median soon, meaning no slow change over time can happen
    last_unstable_time = millis(); // update last unstable time
    light_values(150, 100, 0); // make led brown
  }
}

bool running = true; // boolean to have the board "turn off". It cant be properly turned off, but we can make it seem like it is
void loop() {
  if (digitalRead(exit_button_pin) == LOW && running == true) { // Check if exit button is being pressed
    running = false; // Stop the loop
    light_values(60, 0, 100); // Turn off the LED
    last_color = "purple"; // Reset last color
    Serial.println("Exiting...");
    delay(1000); // Give some time for the user to see the LED color
    return; // Exit the loop
  }
  if (!running) return; // If running is false (exit button pressed was registered), keep exiting the loop (so "stopping")

  // Read sensors and update their rolling windows
  for (int i = 0; i < pinsUsed; i++) {
    datas[i][data_idx[i]] = analogRead(analogPins[i]) / 1023.0; // readings are analogous (0-1023), we convert it to a 0.0-1.0
    data_idx[i]++; //increment the index for the rolling window
    if (data_idx[i] >= window_size) { // if we reached the end of the rolling window
      data_idx[i] = 0; //reset the index
      buffer_full[i] = true; //mark the buffer as full so we can start checking posture
    }
  }

  // Only check stability if all buffers are full (so there is enough data)
  bool enough_data = true;
  for (int i = 0; i < pinsUsed; i++) {
    if (!buffer_full[i]) { //if any buffer is not full, we cannot check stability yet
      enough_data = false;
      break;
    }
  }

  if (enough_data && at_chair()) {
    bool all_consistent = true;
    for (int i = 0; i < pinsUsed; i++) { // Build window for each sensor
      float window[window_size];
      for (int j = 0; j < window_size; j++) // where each window's values are the last window_size readings of the sensor
        window[j] = datas[i][(data_idx[i] + j) % window_size]; // wrap around the index with modulo to keep it going

      float med = median(window); // get the median of the current window for calculating the MAD
      float mad_val = mad(window, med); // calculate the MAD of the current window

      // Consistency: % of readings within MAD*multiplier of median
      int within_range = 0; // number of elements considered as consistent
      for (int j = 0; j < window_size; j++)
        if (fabs(window[j] - med) < mad_multiplier * (mad_val + 1e-6)) 
          within_range++;
      float consistency_score = (float)within_range / window_size;

      // Check for drift from reference
      if (consistency_score < consistency_threshold) {
        all_consistent = false;
        light_values(0, 0, 100); // Set LED to blue for inconsistency
        last_color = "blue";
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" inconsistent: ");
        Serial.print(consistency_score, 2);
        Serial.print(" < ");
        Serial.println(consistency_threshold, 2);
        // break; // we probably don't want to break here, because we want to check all sensors 
      } else if (fabs(med - reference_median[i]) > drift_threshold && reference_set) { 
        all_consistent = false; // if the median of the current window is too far from the reference median, we consider it inconsistent
        light_values(100, 0, 100); // Set LED to cyan for drift over time
        last_color = "magenta";
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" drifted: ");
        Serial.print(med, 2);
        Serial.print(" vs reference ");
        Serial.println(reference_median[i], 2);
      }
    }
    debounce(all_consistent); // Call debounce function to handle consistency checks
  } else if (enough_data && !at_chair()) {
    light_values(100, 100, 100); // Set LED to white if no person is at the chair
    last_color = "white"; // Update last color to white
    Serial.println("No person at the chair");
  }
  delay(delay_time); // may operate too fast without this, we'll see if we need it or not
}