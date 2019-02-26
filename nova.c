// Command a Quaternion attitude with feedforward and smoothing
void AC_AttitudeControl::input_quaternion(Quaternion attitude_desired_quat)
{
    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    Quaternion attitude_error_quat = _attitude_target_quat.inverse() * attitude_desired_quat;
    Vector3f attitude_error_angle;
    attitude_error_quat.to_axis_angle(attitude_error_angle);

    if (_rate_bf_ff_enabled) {
        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _attitude_target_ang_vel.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _attitude_target_ang_vel.x, _dt);
        _attitude_target_ang_vel.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _attitude_target_ang_vel.y, _dt);
        _attitude_target_ang_vel.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _attitude_target_ang_vel.z, _dt);

        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        _attitude_target_quat = attitude_desired_quat;

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}
