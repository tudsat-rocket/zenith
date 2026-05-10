use state_estimator::StateEstimatorSettings;

#[derive(Debug, Default, Clone)]
pub struct Settings {
    pub state_estimator: StateEstimatorSettings,
    pub recovery: RecoverySettings,
}

#[derive(Debug, Clone)]
pub struct RecoverySettings {
    /// Altitude AGL (meters) at which to deploy the main parachute
    pub main_deploy_altitude: f32,
    /// Minimum time (ms) after launch before allowing drogue deployment
    pub min_time_to_drogue: u32,
    /// Minimum time (ms) after drogue before allowing main deployment
    pub min_time_to_main: u32,
}

impl Default for RecoverySettings {
    fn default() -> Self {
        Self {
            main_deploy_altitude: 400.0,
            min_time_to_drogue: 1000,
            min_time_to_main: 3000,
        }
    }
}
