use links::UplinkCommand;
use rapid_dialect::Rapid;

pub const HEARTBEAT_INTERVAL_MS: u32 = 500;
pub const SENSOR_INTERVAL_MS: u32 = 100;
pub const BATTERY_INTERVAL_MS: u32 = 200;

pub trait TelemetryLink {
    fn send_message(&mut self, message: Rapid);
    fn try_recv_command(&mut self) -> Option<UplinkCommand>;
}
