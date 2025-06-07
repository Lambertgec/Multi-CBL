#include <Arduino.h>
#include <math.h>
#include <string.h>

// Variables to change manually depending on setup
const int pinsUsed = 8; //number of analog pins used
const int analogPins[pinsUsed] = {A0, A1, A2, A3, A4, A5, A6, A7}; //analog pins to read (for sensors)
const int LED_PIN[3] = {9, 10, 11}; //pin for the LED, as its RGB it will also probably have 3 pins?
const int buzzerPins[pinsUsed] = {2, 3, 4, 5}; //pins for the buzzers (one for each sensor)
const int serial_baud_rate = 9600; // Serial communication baud rate

// Variables we have to play around with
const int window_size = 20; //size of the rolling window for each sensor
const float consistency_threshold = 0.65; //amount of readings that must be < MAD*multiplier of median of the rolling window 
const int debounce_length = 3; //amount of consecutive readings that must be consistent
const float mad_multiplier = 3.0; //multiplier for filtering noise
const unsigned long stability_seconds = 5 * 1000; // amount of time in seconds that the posture must be stable for
const int delay_time = 100; //delay between readings in milliseconds, adjust as needed
const int pulse_duration = 100; // duration of the pulse for the light nudge in milliseconds
const float at_chair_threshold = 0.05; // threshold for determining if a person is at the chair, so value when sitting down
float reference_median[pinsUsed] = {0}; // 
const float drift_threshold = 0.15; // the value the median of readings can drift from the reference (which is the median of when the posture was stable)

// Global variables
float datas[pinsUsed][window_size]; //rolling window for each sensor
int data_idx[pinsUsed] = {0}; //current index for each sensor's rolling window
bool buffer_full[pinsUsed] = {false}; //whether there are rolling window data for each sensor
bool debounce_buffer[debounce_length] = {true, true, true}; //buffer for consistency checks (noise filtering)
unsigned long last_unstable_time = 0; // last time the posture was unstable
const int timestamp = 10 * 1000; // timestamps for the light nudge function
bool reported = false; // whether a stable posture has been reported, for now no functionality

unsigned long color_set_time = 0; // time when the color was last set
String last_color = ""; // last color set for the light nudge function
bool led_pulsing = false;
unsigned long pulse_start_time = 0; // time when the pulse started
void light_nudge(String color) {
  if (color != last_color) {
    // Color changed, update immediately and reset timer
    last_color = color;
    color_set_time = millis();
    led_pulsing = false; // reset pulsing state
    if (color == "red")        light_values(200, 0, 0);
    else if (color == "green") light_values(0, 200, 0);
    else if (color == "orange")light_values(200, 100, 0);
    else                       Serial.println("Invalid color");
    return;
  }

  // Color is the same as before
  if (millis() - color_set_time >= timestamp) {
    if (!led_pulsing) {
      light_values(0, 0, 0); // Turn off LED
      pulse_start_time = millis();
      led_pulsing = true;
    } else if (led_pulsing && (millis() - pulse_start_time >= pulse_duration)) {
      if (color == "red")        light_values(200, 0, 0);
      else if (color == "green") light_values(0, 200, 0);
      else if (color == "orange")light_values(200, 100, 0);
      else                       Serial.println("Invalid color");
      led_pulsing = false; // Ready for next pulse cycle
    }
  } else { // we might not need this but it doesnt hurt to keep it
    // Keep the LED on with the current color
    if (color == "red")        light_values(200, 0, 0);
    else if (color == "green") light_values(0, 200, 0);
    else if (color == "orange")light_values(200, 100, 0);
    else                       Serial.println("Invalid color");
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
  for (int i = 0; i < 3; i++) {
    pinMode(LED_PIN[i], OUTPUT); // Set the LED pin as output
  }
  for (int i = 0; i < pinsUsed; i++) {
    pinMode(analogPins[i], INPUT); // Optional for clarity
    //pinMode(buzzerPins[i], OUTPUT); // Set buzzer pins as output
    for (int j = 0; j < window_size; j++) {
        datas[i][j] = 0.0; //initialize rolling windows to 0.0
      }
    }  
  last_unstable_time = millis(); //initialize last unstable time
}

// this might have to be changed once its actually in the seating, because padding might cause unintentional small readings
bool at_chair() {
  // Check if any of the sensors have a positive reading 
  if (datas[0][data_idx[0]] > at_chair_threshold || datas[1][data_idx[1]] > at_chair_threshold || // we have to make these the bottom 4 sensors
      datas[2][data_idx[2]] > at_chair_threshold || datas[3][data_idx[3]] > at_chair_threshold) {
    return true; // at least one sensor has a reading above the threshold, meaning a person is at the chair
  } 
  return false; // all sensors had a reading of 0, meaning no person is at the chair
}

bool incorrect_posture() { // we should check all the relevant sensors, not just first 4
  for (int i = 0; i < 4; i++) { 
    float med = median(datas[i]);
    float mad_val = mad(datas[i], med);
    if (med > 0.01 || mad_val > 0.01) { // adjust threshold as needed
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

void debounce(bool all_consistent) {
  if (!at_chair()) { // if no person is at the chair, we can immediately report that
    Serial.println("No person at the chair");
    light_values(150, 150, 150); // set it to white, just for debugging 
    last_unstable_time = millis(); 
    return;
  }

  if (incorrect_posture()) { // we can immediately report all postures that we deem incorrect in incorrect_posture()
    // RED if sensors 0-3 are all mostly 0
    Serial.println("Poor posture detected, should be changed");
    light_nudge("red"); // Blink red light  
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

  if (debounced) {
    if (millis() - last_unstable_time > stability_seconds) {
        // ORANGE if there was a timeout with sensors 0-3 not all being 0
        Serial.println("Posture timeout");
        light_nudge("orange"); // Set LED to orange
    } else {
      // GREEN if there was no timeout and acceptable posture
      Serial.println("Posture can still be kept and is stable");
      light_nudge("green"); // Set LED to green 
    }

    // Update reference median when posture becomes stable
    for (int i = 0; i < pinsUsed; i++) {
      float window[window_size];
      for (int j = 0; j < window_size; j++)
          window[j] = datas[i][(data_idx[i] + j) % window_size];
      reference_median[i] = median(window);
    }
  } else {
    // GREEN when not stable and also "acceptable" posture (meaning lowerback 2 sensors and seat 2 rear sensors are not all 0)
    Serial.println("Posture not stable, but acceptable");
    last_unstable_time = millis(); // update last unstable time
    light_nudge("green"); // make led green
  }
}

void loop() {
  // Read sensors and update their rolling windows
  for (int i = 0; i < pinsUsed; i++) {
    datas[i][data_idx[i]] = analogRead(analogPins[i]) / 1023.0; // readings are analogous (0-1023), we convert it to a 0.0-1.0
    data_idx[i]++; //increment the index for the rolling window
    if (data_idx[i] >= window_size) { // if we reached the end of the rolling window
      data_idx[i] = 0; //reset the index
      buffer_full[i] = true; //mark the buffer as full so we can start checking posture
    }
  }

  // Only check stability if all buffers are full
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
      if (consistency_score < consistency_threshold || fabs(med - reference_median[i]) > drift_threshold) {
        all_consistent = false;
        light_values(0, 0, 200); // Set LED to blue for inconsistency
        last_color = "blue";
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" inconsistent: ");
        Serial.print(consistency_score, 2);
        Serial.print(" < ");
        Serial.println(consistency_threshold, 2);
        // break; // we probably don't want to break here, because we want to check all sensors 
      }
    }
    debounce(all_consistent); // Call debounce function to handle consistency checks
  } else if (enough_data && !at_chair()) {
    light_values(150, 150, 150); // Set LED to white if no person is at the chair
    Serial.println("No person at the chair");
  }
  // delay(delay_time); // may operate too fast without this, we'll see if we need it or not
}