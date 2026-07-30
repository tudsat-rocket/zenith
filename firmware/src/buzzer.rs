use core::ops::Not;

use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::can::CanTx;
use embassy_stm32::lptim::pwm::Pwm;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

use defmt::*;
use embassy_stm32::peripherals::TIM2;
use embassy_time::{Duration, Ticker, Timer};
use num_traits::Float;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use Semitone::*;

static STARTUP_TECH: [Note; 6] = [
    Note::new(E, 4, 100),
    Note::pause(20),
    Note::new(A, 4, 100),
    Note::pause(20),
    Note::new(E, 5, 200),
    Note::pause(100),
];

static BATTERY_LOW: [Note; 3] = [Note::new(A, 4, 100), Note::pause(20), Note::new(A, 4, 100)];

static BATTERY_EXTREM_LOW: [Note; 7] = [
    Note::new(A, 5, 100),
    Note::pause(40),
    Note::new(A, 5, 100),
    Note::pause(100),
    Note::new(A, 6, 200),
    Note::pause(40),
    Note::new(A, 6, 200),
];

static MODE_CHANGE: [Note; 1] = [Note::new(E, 4, 1000)];

#[derive(Clone, Copy, PartialEq, Format)]
pub enum Sound {
    StartupTech,
    BatteryLow,
    BatteryExtremLow,
    ModeChange,
}

fn get_song_notes(sound: Sound) -> &'static [Note] {
    match sound {
        Sound::StartupTech => &STARTUP_TECH,
        Sound::BatteryLow => &BATTERY_LOW,
        Sound::BatteryExtremLow => &BATTERY_EXTREM_LOW,
        Sound::ModeChange => &MODE_CHANGE,
    }
}

pub static SOUND_CHANNEL: Channel<ThreadModeRawMutex, Sound, 1> = Channel::new();

static STOP_SIGAL: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub fn spawn(buzzer: (SimplePwm<'static, TIM2>, embassy_stm32::timer::Channel), spawner: Spawner) {
    spawner.spawn(buzzer_controller()).unwrap();
    spawner.spawn(player(buzzer)).unwrap();
}

/// Buzzer buzzer controller
///
/// Listens for a new song and sets the stop signal as soon as a new song is selected
#[embassy_executor::task]
async fn buzzer_controller() {
    loop {
        let new_song = SOUND_CHANNEL.receive().await;
        info!("New Song selected {:?}", new_song);

        // stop the player to play the next song
        STOP_SIGAL.signal(());
    }
}

/// The song player
///
/// Listens for new a new song, and plays every note on the buzzer
/// The song can be aborted at any time using the stop_signal
#[embassy_executor::task]
pub async fn player(buzzer: (SimplePwm<'static, TIM2>, embassy_stm32::timer::Channel)) {
    info!("Starting buzzer...");

    let (mut pwm, channel) = buzzer;
    let max_duty = pwm.max_duty_cycle() as u16;

    // set the volume of the the buzzer
    let volume = 30u16;
    pwm.channel(channel).set_duty_cycle(volume * max_duty / 100);

    loop {
        let song = SOUND_CHANNEL.receive().await;
        let notes = get_song_notes(song);

        for note in notes.iter() {
            // stop playing the song when the stop signal is set
            if STOP_SIGAL.signaled() {
                info!("Sound aborted - Play new sound");
                pwm.channel(channel).disable();
                STOP_SIGAL.reset();
                break;
            }

            // play the next note
            if let Some(frequency) = note.frequency() {
                pwm.set_frequency(Hertz::hz(frequency as u32));
                pwm.channel(channel).enable();
            } else {
                pwm.channel(channel).disable();
            }
            Timer::after(Duration::from_millis(note.duration as u64)).await;
        }

        // turn off buzzer
        pwm.channel(channel).disable();
    }
}

/// Request a new song.
///
/// This will stop the currently playing song and
/// will start playing the requested song
pub fn request_sound(sound: Sound) {
    let _ = SOUND_CHANNEL.try_send(sound);
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
        self.frequency
            .or(self.pitch.as_ref().map(|p| p.frequency()))
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
