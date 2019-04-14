void setup() {
  // Initialize sensors

}

void loop() {
  static unsigned long timestamp;
  static unsigned long old_time = 0; //ms
  static unsigned long new_time = 0; //ms
  unsigned long delta_time;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms
  
  if (state == LANDED)
        time_interval = LANDED_POLLING_TIME_INTERVAL;
    else
        time_interval = NOMINAL_POLLING_TIME_INTERVAL;


    new_time = millis();
    if ((new_time - old_time) >= time_interval) {
        delta_time = new_time - old_time;
        old_time = new_time;

        pollSensors(&timestamp, &battery_voltage, acc_data, bar_data, &temp_sensor_data, IMU_data, GPS_data);
        calculateValues(acc_data, bar_data, &prev_altitude, &altitude, &delta_altitude, &prev_delta_altitude, &baseline_pressure, &delta_time, pressure_set);
        stateMachine(&altitude, &delta_altitude, &prev_altitude, bar_data, &baseline_pressure, &ground_altitude, ground_alt_arr, &state);
        logData(&timestamp, &battery_voltage, acc_data, bar_data, &temp_sensor_data, IMU_data, GPS_data, state, altitude, baseline_pressure);
    }

}
