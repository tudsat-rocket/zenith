//! Buzzer driver and sound library.
//!
//! A single [`player`] task owns the PWM channel driving the buzzer. Everything
//! else talks to it through [`COMMANDS`], using the `request_*` functions below.
//! There are two kinds of sounds:
//!
//! * *One-shots* ([`request_sound`]) are played once and interrupt whatever is
//!   currently playing, e.g. the chirp emitted on every flight mode change.
//! * The *alert loop* ([`request_loop`]) repeats until it is replaced or
//!   cleared. It is used for the continuous warnings that have to outlast a
//!   single sound, e.g. the landing beacon.
//!
//! A one-shot temporarily takes over from the alert loop; once it has finished
//! playing, the loop resumes on its own.
//!
//! Which sound is played when is not decided here, but in [`alerts`]. Sounds
//! can additionally be triggered from the ground by name via `PLAY_TUNE_V2`,
//! see [`Sound::from_name`] for the names.

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use num_traits::Float;

#[allow(clippy::enum_glob_use, reason = "annoying")]
use Semitone::*;

pub mod alerts;
mod sounds;
use sounds::mario::MARIO;

/// Duty cycle of the PWM signal, in percent. A buzzer is loudest at 50%, this
/// trades some volume for a lower current draw.
const VOLUME_PERCENT: u8 = 30;

static STARTUP_TECH: [Note; 6] = [
    Note::new(E, 4, 100),
    Note::pause(20),
    Note::new(A, 4, 100),
    Note::pause(20),
    Note::new(E, 5, 200),
    Note::pause(100),
];

/// Three descending beeps, repeated every few seconds while the battery is low.
static BATTERY_LOW: [Note; 6] = [
    Note::new(E, 6, 120),
    Note::pause(60),
    Note::new(Cs, 6, 120),
    Note::pause(60),
    Note::new(A, 5, 250),
    Note::pause(150),
];

/// Same descending motif as [`BATTERY_LOW`], but preceded by a burst of fast
/// beeps, so the two warnings are easy to tell apart.
static BATTERY_EXTREME_LOW: [Note; 14] = [
    Note::new(A, 6, 70),
    Note::pause(50),
    Note::new(A, 6, 70),
    Note::pause(50),
    Note::new(A, 6, 70),
    Note::pause(50),
    Note::new(A, 6, 70),
    Note::pause(150),
    Note::new(E, 6, 120),
    Note::pause(60),
    Note::new(Cs, 6, 120),
    Note::pause(60),
    Note::new(A, 5, 350),
    Note::pause(150),
];

/// Short rising chirp, played whenever the flight mode changes.
static MODE_CHANGE: [Note; 4] = [
    Note::new(A, 5, 90),
    Note::pause(30),
    Note::new(E, 6, 140),
    Note::pause(120),
];

/// Two-tone hazard warning, looped while the vehicle is pressurized. Kept
/// deliberately unpleasant and gapless: it means "do not approach the rocket".
static PRESSURIZED: [Note; 4] = [
    Note::new(Ds, 6, 220),
    Note::pause(50),
    Note::new(A, 5, 220),
    Note::pause(50),
];

/// Locator beacon, looped after touchdown. Two short high beeps followed by a
/// long pause: high notes carry the furthest, the pause makes the direction the
/// sound comes from easier to make out (and saves some battery).
static LANDED: [Note; 4] = [
    Note::new(E, 6, 250),
    Note::pause(180),
    Note::new(E, 6, 250),
    Note::pause(1300),
];

static IGNITION: [Note; 28] = [
    Note::new(C, 4, 1000),
    Note::pause(10),
    Note::new(C, 4, 1000),
    Note::pause(10),
    Note::new(C, 4, 1000),
    Note::pause(10),
    Note::new(C, 4, 500),
    Note::pause(10),
    Note::new(C, 4, 500),
    Note::pause(10),
    Note::new(C, 4, 500),
    Note::pause(10),
    Note::new(C, 4, 250),
    Note::pause(10),
    Note::new(C, 4, 250),
    Note::pause(10),
    Note::new(C, 4, 250),
    Note::pause(10),
    Note::new(C, 4, 125),
    Note::pause(10),
    Note::new(C, 4, 125),
    Note::pause(10),
    Note::new(C, 4, 125),
    Note::pause(10),
    Note::new(C, 4, 125),
    Note::pause(10),
    Note::new(C, 4, 10000),
    Note::pause(10),
];

#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub enum Sound {
    StartupTech,
    BatteryLow,
    BatteryExtremeLow,
    ModeChange,
    Pressurized,
    Landed,
    Ignition,
    Mario,
}

impl Sound {
    /// The sounds that can be requested by name, e.g. over `PLAY_TUNE_V2`.
    const NAMED: [(&'static str, Sound); 8] = [
        ("startup", Sound::StartupTech),
        ("battery_low", Sound::BatteryLow),
        ("battery_extreme_low", Sound::BatteryExtremeLow),
        ("mode_change", Sound::ModeChange),
        ("pressurized", Sound::Pressurized),
        ("landed", Sound::Landed),
        ("ignition", Sound::Ignition),
        ("mario", Sound::Mario),
    ];

    /// Look a sound up by name, ignoring case.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::NAMED
            .iter()
            .find(|(known, _)| known.eq_ignore_ascii_case(name))
            .map(|(_, sound)| *sound)
    }
}

fn get_song_notes(sound: Sound) -> &'static [Note] {
    match sound {
        Sound::StartupTech => &STARTUP_TECH,
        Sound::BatteryLow => &BATTERY_LOW,
        Sound::BatteryExtremeLow => &BATTERY_EXTREME_LOW,
        Sound::ModeChange => &MODE_CHANGE,
        Sound::Pressurized => &PRESSURIZED,
        Sound::Landed => &LANDED,
        Sound::Ignition => &IGNITION,
        Sound::Mario => &MARIO,
    }
}

/// Commands accepted by the [`player`] task.
#[derive(Clone, Copy, PartialEq, Eq, Format)]
enum Command {
    /// Play a sound once, interrupting whatever is currently playing.
    PlayOnce(Sound),
    /// Replace the looping alert sound (`None` clears it).
    SetLoop(Option<Sound>),
    /// Stop the one-shot and the alert loop.
    Stop,
}

/// Commands are sent from tasks on both the thread mode and the interrupt
/// executors, hence the critical section mutex.
static COMMANDS: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();

pub fn spawn(buzzer: (SimplePwm<'static, TIM2>, embassy_stm32::timer::Channel), spawner: Spawner) {
    #[allow(
        clippy::unwrap_used,
        reason = "boot-time task init; panic-on-failure is the embedded model"
    )]
    spawner.spawn(player(buzzer)).unwrap();
}

/// The song player.
///
/// Plays the sound currently selected by the accumulated [`Command`]s, note by
/// note, and keeps looping it if it is the alert loop. New commands are applied
/// immediately, even in the middle of a note.
#[embassy_executor::task]
async fn player(buzzer: (SimplePwm<'static, TIM2>, embassy_stm32::timer::Channel)) -> ! {
    info!("Starting buzzer...");

    let (mut pwm, channel) = buzzer;
    let mut state = PlayerState::default();

    loop {
        let Some(playback) = state.desired() else {
            // Nothing to play, wait for something to do.
            pwm.channel(channel).disable();
            state.apply(COMMANDS.receive().await);
            continue;
        };

        'song: loop {
            for note in get_song_notes(playback.sound) {
                if let Some(frequency) = note.frequency() {
                    pwm.set_frequency(Hertz::hz(frequency as u32));
                    // Changing the frequency changes the timer period without
                    // touching the compare register, so the duty cycle - and
                    // with it the volume - has to be set again for every note.
                    pwm.channel(channel).set_duty_cycle_percent(VOLUME_PERCENT);
                    pwm.channel(channel).enable();
                } else {
                    pwm.channel(channel).disable();
                }

                // Wait out the note, but stay responsive: a new command may
                // have to interrupt us in the middle of a long note. Commands
                // that do not change what should be playing (e.g. arming an
                // alert loop while a one-shot is running) leave the note alone.
                let mut note_over = Timer::after(Duration::from_millis(u64::from(note.duration)));
                loop {
                    match select(&mut note_over, COMMANDS.receive()).await {
                        Either::First(()) => break,
                        Either::Second(command) => {
                            state.apply(command);
                            if state.desired() != Some(playback) {
                                info!("New sound selected - abort");
                                break 'song;
                            }
                        }
                    }
                }
            }

            if !playback.repeat {
                state.finished(playback);
                break;
            }
        }
    }
}

/// A sound the player is (or should be) playing.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Playback {
    sound: Sound,
    /// Keep repeating the sound until something else is requested.
    repeat: bool,
    /// Distinguishes two consecutive requests for the same one-shot sound, so
    /// that the second one restarts the sound instead of being swallowed.
    generation: u32,
}

/// What the player has been asked to play.
#[derive(Default)]
struct PlayerState {
    alert_loop: Option<Sound>,
    one_shot: Option<Sound>,
    generation: u32,
}

impl PlayerState {
    fn apply(&mut self, command: Command) {
        match command {
            Command::PlayOnce(sound) => {
                self.one_shot = Some(sound);
                self.generation = self.generation.wrapping_add(1);
            }
            Command::SetLoop(sound) => self.alert_loop = sound,
            Command::Stop => {
                self.one_shot = None;
                self.alert_loop = None;
            }
        }
    }

    /// The sound that should be playing right now, if any. One-shots take
    /// precedence over the alert loop.
    fn desired(&self) -> Option<Playback> {
        let (sound, repeat) = match (self.one_shot, self.alert_loop) {
            (Some(one_shot), _) => (one_shot, false),
            (None, Some(alert_loop)) => (alert_loop, true),
            (None, None) => return None,
        };

        Some(Playback {
            sound,
            repeat,
            generation: self.generation,
        })
    }

    /// Called once a one-shot has been played to the end, so the alert loop can
    /// take over again.
    fn finished(&mut self, playback: Playback) {
        if self.desired() == Some(playback) {
            self.one_shot = None;
        }
    }
}

/// Play a sound once, interrupting whatever is currently playing.
///
/// Once the sound has finished, the alert loop (if any) resumes.
pub fn request_sound(sound: Sound) {
    send(Command::PlayOnce(sound));
}

/// Set the sound looped whenever no one-shot is playing, or clear it by passing
/// `None`.
///
/// Note that an alert loop never interrupts a one-shot that is already playing.
/// When both are requested for the same event, request the loop first, so the
/// one-shot is not cut short.
pub fn request_loop(sound: Option<Sound>) {
    send(Command::SetLoop(sound));
}

/// Stop the one-shot and the alert loop.
pub fn request_stop() {
    send(Command::Stop);
}

fn send(command: Command) {
    if let Err(_) = COMMANDS.try_send(command) {
        warn!("Buzzer command queue full, dropping {}", command);
    }
}

struct Note {
    pitch: Option<Pitch>,
    frequency: Option<f32>,
    duration: u32,
}

struct Pitch {
    semitone: Semitone,
    octave: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum Semitone {
    C = 0,
    Cs = 1,
    D = 2,
    Ds = 3,
    E = 4,
    F = 5,
    Fs = 6,
    G = 7,
    Gs = 8,
    A = 9,
    As = 10,
    B = 11,
}

impl Note {
    /// Create a new note
    const fn new(semitone: Semitone, octave: u8, duration: u32) -> Self {
        Self {
            pitch: Some(Pitch { semitone, octave }),
            frequency: None,
            duration,
        }
    }

    /// Create a Note with a frequency
    #[allow(dead_code)]
    const fn with_frequency(frequency: f32, duration: u32) -> Self {
        Self {
            pitch: None,
            frequency: Some(frequency),
            duration,
        }
    }

    /// Create a new pause
    const fn pause(duration: u32) -> Self {
        Self {
            pitch: None,
            frequency: None,
            duration,
        }
    }

    /// return the frequency. In case only a pitch was set, calculate the frequency otherwise
    /// directly return the frequency
    fn frequency(&self) -> Option<f32> {
        self.frequency.or(self.pitch.as_ref().map(Pitch::frequency))
    }
}

impl Pitch {
    /// Calculate the frequency for a tone in Hertz
    ///
    /// The reference tone is A4 with 440 hz.
    /// One octavce is double the frequency and each octave is divided into 12 semitones.
    /// To get from one semitone to the next one, you multiply the frequency with the 12th
    /// squereroot of 2
    ///
    /// **Returns**
    /// A `f32` which represenmts the frequency in Hz
    #[allow(
        clippy::arithmetic_side_effects,
        reason = "semitone indices are far too small to overflow an i32"
    )]
    fn frequency(&self) -> f32 {
        // calculate the position of the reference tone A4 on a linear scale
        let a_i = 3 * 12 + (Semitone::A as i32);
        // calculate the position of the wanted note on the same scale
        let note_i = (self.octave as i32) * 12 + (self.semitone as i32);
        // apply frequency formula: f = 440 * 2^((n / 12))
        // to get the difference between the wanted tone and the reference tone A4
        440.0 * 2.0f32.powf((note_i - a_i) as f32 / 12.0)
    }
}
