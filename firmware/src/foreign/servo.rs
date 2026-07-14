/// 0 = closed 100 = fully open
pub fn percent_open_to_pwm_us(percent: f32, calib: &CalibServoValve) -> u16 {
    debug_assert!(!percent.is_nan());
    debug_assert!((0.0..=100.0).contains(&percent));
    let percent = percent.clamp(0.0, 100.0);
    let us_range = calib.closed_us.max(calib.opened_us) - calib.closed_us.min(calib.opened_us);
    // TODO: test what happens if someone cast NaN to u16
    let us_offset = ((us_range as f32 * percent) / 100.0).round() as u16;
    let us_offset = us_offset.clamp(0, us_range);

    match calib.closed_us.cmp(&calib.opened_us) {
        Ordering::Equal => calib.closed_us,
        Ordering::Less => calib.closed_us + us_offset,
        Ordering::Greater => calib.closed_us - us_offset,
    }
}

pub struct CalibServoValve {
    /// microsecond pwm for which the valve is closed
    pub closed_us: u16,
    /// microsecond pwm for which the valve is opened
    pub opened_us: u16,
}

pub fn valve_state_to_servo_us(state: ValveState, valve_id: ValveId) -> u16 {
    // TODO: replace placeholder
    let servo_placeholder: CalibServoValve = CalibServoValve {
        closed_us: 1000,
        opened_us: 2000,
    };
    let total_angle = 2700;
    // TODO: replace placeholder calculation

    let angle_deci = state.promille() * (3600 / 1000);
    let min_us = servo_placeholder.opened_us.min(servo_placeholder.closed_us);
    let max_us = servo_placeholder.opened_us.max(servo_placeholder.closed_us);

    let angle = angle_deci / 10;
    defmt::info!("angle: {}", angle);
    debug_assert!(angle_deci <= total_angle);
    let angle_deci = angle_deci.clamp(0, total_angle);
    debug_assert!(min_us < max_us);
    let span = max_us - min_us;

    let pwm =
        min_us + u16::try_from(((span as u32) * (angle_deci as u32)) / total_angle as u32).unwrap();
    debug_assert!(min_us <= pwm && pwm <= max_us);
    pwm
}
