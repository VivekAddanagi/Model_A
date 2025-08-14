/*void flight_control_loop() {
    // 1. Read IMU
    bmi323_read(&sensor_data);

    // 2. Update orientation
    update_orientation(sensor_data.ax, sensor_data.ay, sensor_data.az,
                       sensor_data.gx, sensor_data.gy, sensor_data.gz);

    // 3. Apply PID using gains from current_config
    float pitch_cmd = pitch_pid.update(estimated_pitch, current_config->pitch_gain);
    float roll_cmd  = roll_pid.update(estimated_roll, current_config->roll_gain);
    float yaw_cmd   = yaw_pid.update(estimated_yaw, current_config->yaw_gain);

    // 4. Motor mixing & altitude hold using current_config->hold_altitude & altitude_gain
}
*/