#![no_std]
#![no_main]
#![allow(
    clippy::unwrap_used,
    reason = "boot-time peripheral/task init; panic-on-failure is the embedded model"
)]

use embassy_executor::{InterruptExecutor, Spawner, raw};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Ticker};
use static_cell::StaticCell;

use firmware::bus::BusHandler;
use firmware::buzzer::alerts::Alerts;
use firmware::can::{CanRxSubscriber, CanTxPublisher};
use firmware::links::{Links, UplinkCommand};
use firmware::{Vehicle, buzzer};
use rapid_dialect::rapid::enums::MavResult;

use {defmt_rtt as _, panic_probe as _};

use firmware as fw;

/// Long enough for the ack to reach the ground, which over LoRa only rides along in heartbeats,
/// and for the ground station to finish retransmitting the request, which would otherwise reboot
/// the vehicle a second time.
const REBOOT_DELAY: Duration = Duration::from_secs(1);

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<raw::Executor> = StaticCell::new();

/// Hand-rolled `#[embassy_executor::main]`, so we can watch CPU utilization.
#[cortex_m_rt::entry]
fn main() -> ! {
    /// Context value the cortex-m pender uses to recognise the thread-mode executor.
    /// Not public in `embassy-executor`, but part of its ABI.
    const THREAD_PENDER: usize = usize::MAX;

    let executor: &'static raw::Executor =
        EXECUTOR_LOW.init(raw::Executor::new(THREAD_PENDER as *mut ()));
    executor.spawner().must_spawn(init(executor.spawner()));

    fw::cpu::init();

    loop {
        // SAFETY: the only `poll` for this executor, never called reentrantly - `sleep` below
        // cannot run any of this executor's tasks.
        unsafe { executor.poll() };
        fw::cpu::sleep();
    }
}

#[embassy_executor::task]
async fn init(low_priority_spawner: Spawner) {
    let board = fw::board::init(true).await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    let lora_params = board.params.lora.clone();
    let storage = fw::storage::spawn(
        board.flash,
        board.params,
        fw::links::interfaces::ethernet::ROCKET_SYSTEM_ID,
        &low_priority_spawner,
    );

    fw::sensors::power::spawn(board.adc, low_priority_spawner);
    #[cfg(not(feature = "gcs"))]
    fw::sensors::gps::spawn(board.gps, low_priority_spawner);

    // Spawn bus handling tasks
    let can1_rx = fw::can::CAN1_RX_CH.init(PubSubChannel::new());
    let can1_tx = fw::can::CAN1_TX_CH.init(PubSubChannel::new());

    fw::can::spawn_can1(
        board.can1,
        medium_priority_spawner,
        can1_rx.publisher().unwrap(),
        can1_tx.subscriber().unwrap(),
    )
    .await;

    fw::buzzer::spawn(board.buzzer, low_priority_spawner);

    let can_tx_pub: CanTxPublisher = can1_tx.publisher().unwrap();
    let can_rx_sub: CanRxSubscriber = can1_rx.subscriber().unwrap();
    let bus = BusHandler::new(can_tx_pub, can_rx_sub, &fw::can::CAN1_HEALTH);

    // Initialize main Vehicle & Linkss structs
    let vehicle = Vehicle::new(board.sensors, board.outputs, storage, bus).await;
    buzzer::set_volume(vehicle.misc_params().buzzer_volume);
    let links = Links::init(
        board.ethernet,
        board.seed,
        board.usb,
        board.lora1,
        board.lora2,
        &lora_params,
        (can1_tx.publisher().unwrap(), can1_rx.subscriber().unwrap()),
        medium_priority_spawner,
        low_priority_spawner,
    )
    .await;

    // Shorten the boot watchdog's timeout for the main loop and spawn it.
    defmt::info!("Boot took {} ms", Instant::now().as_millis());
    let mut iwdg = IndependentWatchdog::new(board.iwdg, 512_000);
    iwdg.unleash();
    high_priority_spawner
        .spawn(main_loop(vehicle, links, iwdg))
        .unwrap();
    buzzer::request_sound(buzzer::Sound::StartupTech);
}

#[embassy_executor::task]
#[allow(
    clippy::too_many_lines,
    reason = "command dispatch is inline until it moves out, see the TODO below"
)]
pub async fn main_loop(
    mut vehicle: Vehicle,
    mut links: Links,
    mut iwdg: IndependentWatchdog<'static, IWDG1>,
) -> ! {
    let mut cpu = fw::cpu::CpuMonitor::new();
    let mut alerts = Alerts::default();
    let mut flash_log = fw::storage::FlashLog::default();
    let mut ticker = Ticker::every(Duration::from_micros(1000));
    let mut reboot_at: Option<Instant> = None;
    loop {
        let tick_started = Instant::now();

        if reboot_at.is_some_and(|at| tick_started >= at) {
            cortex_m::peripheral::SCB::sys_reset();
        }

        if links::take_uplink_activity() {
            vehicle.note_uplink();
        }

        vehicle.tick().await;

        // Let the buzzer know about mode changes and the battery voltage.
        alerts.update(
            vehicle.mode(),
            vehicle.readings.power.as_ref().map(|p| p.bus_main_voltage),
        );

        // TODO: this belongs somewhere else
        if let Some((token, cmd)) = links.try_recv_command() {
            let result = match cmd {
                UplinkCommand::SetFlightMode(fm) => {
                    vehicle.set_mode(fm);
                    MavResult::Accepted
                }
                UplinkCommand::CommandValve(valve_id, valve_cmd) => {
                    match vehicle.try_command_valve(valve_id, valve_cmd) {
                        Ok(()) => MavResult::Accepted,
                        Err(e) => {
                            defmt::warn!(
                                "CommandValve {} {} rejected: {}",
                                defmt::Debug2Format(&valve_id),
                                defmt::Debug2Format(&valve_cmd),
                                defmt::Debug2Format(&e)
                            );
                            e.into()
                        }
                    }
                }
                UplinkCommand::SetServo {
                    servo, promille, ..
                } => match vehicle.try_command_servo(servo, promille) {
                    Ok(()) => MavResult::Accepted,
                    Err(e) => {
                        defmt::warn!("SetServo {} {} rejected", servo, promille);
                        e
                    }
                },
                UplinkCommand::Gripper { instance, grab } => {
                    match vehicle.try_command_gripper(instance, grab) {
                        Ok(()) => MavResult::Accepted,
                        Err(e) => {
                            defmt::warn!("Gripper {} {} rejected", instance, grab);
                            e
                        }
                    }
                }
                UplinkCommand::SetParam { id, raw } => {
                    vehicle.set_param(id, raw).await;
                    buzzer::set_volume(vehicle.misc_params().buzzer_volume);
                    MavResult::Accepted
                }
                UplinkCommand::PlayTune(name) => {
                    if alerts.alert_active() {
                        // The ground must not be able to silence the pad hazard
                        // warning or the landing beacon.
                        defmt::warn!("Ignoring tune request while a buzzer alert is active");
                        MavResult::TemporarilyRejected
                    } else if name.is_empty() {
                        // We take an empty tune to mean "stop the buzzer".
                        buzzer::request_stop();
                        MavResult::Accepted
                    } else if let Some(sound) = buzzer::Sound::from_name(&name) {
                        buzzer::request_sound(sound);
                        MavResult::Accepted
                    } else {
                        defmt::warn!("Unknown tune {}", name.as_str());
                        MavResult::Denied
                    }
                }
                UplinkCommand::EraseLog => {
                    if vehicle.erase_log() {
                        MavResult::Accepted
                    } else {
                        MavResult::Denied
                    }
                }
                UplinkCommand::Reboot => {
                    defmt::warn!("Rebooting");
                    flash_log.flush();
                    reboot_at = Instant::now().checked_add(REBOOT_DELAY);
                    MavResult::Accepted
                }
                _ => MavResult::Unsupported,
            };

            links.note_command_result(token, result);
        }

        links.send_telemetry_messages(&vehicle);
        flash_log.log(&vehicle.snapshot());

        cpu.update(tick_started.elapsed());
        iwdg.pet();
        ticker.next().await;
    }
}

#[interrupt]
unsafe fn I2C3_EV() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[interrupt]
unsafe fn I2C3_ER() {
    unsafe { EXECUTOR_MEDIUM.on_interrupt() }
}
