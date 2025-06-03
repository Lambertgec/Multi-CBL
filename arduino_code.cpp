#include <Arduino.h>
#include <math.h>
#include <string.h>

// Variables to change manually depending on setup
const int pinsUsed = 4; //number of analog pins used
const int analogPins[pinsUsed] = {A0, A1, A2, A3}; //analog pins to read (for sensors)
const int LED_PIN = 13; //pin for the LED, as its RGB it will also probably have 3 pins?
const int buzzerPins[pinsUsed] = {2, 3, 4, 5}; //pins for the buzzers (one for each sensor)

// Variables we have to play around with
const int window_size = 20; //size of the rolling window for each sensor
const float consistency_threshold = 0.65; //amount of readings that must be < MAD*multiplier of median of the rolling window 
const int debounce_length = 3; //amount of consecutive readings that must be consistent
const float mad_multiplier = 3.0; //multiplier for filtering noise
const unsigned long stability_seconds = 5 * 1000; // amount of time in seconds that the posture must be stable for
const int delay_time = 100; //delay between readings in milliseconds, adjust as needed

// Global variables
float datas[pinsUsed][window_size]; //rolling window for each sensor
int data_idx[pinsUsed] = {0}; //current index for each sensor's rolling window
bool buffer_full[pinsUsed] = {false}; //whether there are rolling window data for each sensor
bool debounce_buffer[debounce_length] = {true, true, true}; //buffer for consistency checks (noise filtering)
unsigned long last_unstable_time = 0; // last time the posture was unstable
bool stable_reported = false; // whether a stable posture has been reported, for now no functionality

float median(float arr[], int size) { // Calculate the median of an array
  float temp[window_size]; //temporary array to hold the sorted values
  memcpy(temp, arr, size * sizeof(float)); //copy arr onto temp
  // Simple insertion sort
  for (int i = 1; i < size; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }
  if (size % 2 == 0) //return the middle element (median) of the sorted array
    return (temp[size/2 - 1] + temp[size/2]) / 2.0;
  else
    return temp[size/2];
}

float mad(float arr[], int size, float med) { //returns the Median Absolute Deviation (MAD) of an array
  float deviations[window_size]; //array for the absolute deviations from the median
  for (int i = 0; i < size; i++)
    deviations[i] = fabs(arr[i] - med);
  return median(deviations, size); //return the median of the absolute deviations
}

void setup() {
  Serial.begin(115200); //this has to be the same as in platformio.ini OR in the serial monitor
  pinMode(LED_PIN, OUTPUT); // Set the LED pin as output
  for (int i = 0; i < pinsUsed; i++)
    pinMode(analogPins[i], INPUT); // Optional for clarity
    pinMode(buzzerPins[i], OUTPUT); // Set buzzer pins as output
    for (int j = 0; j < window_size; j++)
      datas[i][j] = 0.0; //initialize rolling windows to 0.0
  last_unstable_time = millis(); //initialize last unstable time
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

  bool all_consistent = true;
  if (enough_data) {
    for (int i = 0; i < pinsUsed; i++) { // Build window for each sensor
      float window[window_size];
      for (int j = 0; j < window_size; j++) // where each window's values are the last window_size readings of the sensor
        window[j] = datas[i][(data_idx[i] + j) % window_size]; // wrap around the index with modulo to keep it going

      float med = median(window, window_size); // get the median of the current window for calculating the MAD
      float mad_val = mad(window, window_size, med); // calculate the MAD of the current window

      // Consistency: % of readings within MAD*multiplier of median
      int within_range = 0; // number of elements considered as consistent
      for (int j = 0; j < window_size; j++)
        if (fabs(window[j] - med) < mad_multiplier * (mad_val + 1e-6)) 
          within_range++;
      float consistency_score = (float)within_range / window_size;

      if (consistency_score < consistency_threshold) { //check if readings were consistent enough
        all_consistent = false;
        char buf[64];
        snprintf(buf, sizeof(buf), "Sensor %d inconsistent: %.2f < %.2f", i, consistency_score, consistency_threshold);
        Serial.println(buf);
        // Serial.print("Sensor ");
        // Serial.print(i);
        // Serial.print(" inconsistent: ");
        // Serial.print(consistency_score, 2);
        // Serial.print(" < ");
        // Serial.println(consistency_threshold, 2);
        // break; // we probably don't want to break here, because we want to check all sensors 
      }
    }
  }

  // Debounce logic - after any of the sensors were inconsistent, the next debounce_length readings must be consistent for all sensors (currently)
  for (int i = 0; i < debounce_length - 1; i++)
    debounce_buffer[i] = debounce_buffer[i + 1];
  debounce_buffer[debounce_length - 1] = all_consistent; // shift the buffer and add the current consistency status

  bool debounced = true;
  for (int i = 0; i < debounce_length; i++)
    if (!debounce_buffer[i]) { // if any of the readings in any of the sensor buffers were false (inconsistent (large change)) 
      debounced = false;  //then debouncing did not happen yet (as we wait for buffers to only have true (stable)) values
      break;
    }

  if (debounced) { // if all sensors were consistent for the last debounce_length readings
    if (millis() - last_unstable_time > stability_seconds) { //if enough time elapsed and we didnt yet report posture as stable, this is where we light up the LED and make the buzzers pulse
        char buf[64];
        snprintf(buf, sizeof(buf), "Stable posture detected for %.2f seconds! \n", stability_seconds / 1000.0);
        Serial.println(buf);
        digitalWrite(LED_PIN, HIGH); // Turn on the LED, if it works this should be changed to a method
        digitalWrite(buzzerPins[0], HIGH); // Turn on the first buzzer, if it works this should be changed to a method
        // Serial.print("Stable posture detected for "); //kept in case snprintf doesnt work
        // Serial.print(stability_seconds / 1000);
        // Serial.println(" seconds!");
        stable_reported = true; // we probably don't need this
    }
  } else { // there was atleast one sensor having inconsistent readings hence we reset the timer
    last_unstable_time = millis();
    stable_reported = false;
    Serial.println("Timer reset");
  }

  delay(delay_time);
}