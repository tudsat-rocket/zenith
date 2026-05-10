//! Valve actuation simulation

use mission::propulsion::ValveCommand;

#[derive(Copy, Clone)]
pub struct Valve {
    /// Targeted state
    commanded: ValveCommand,
    /// Time [s]
    commanded_time: f32,
    /// Conductance [mol/(s*bar) for gas / L/(s*bar) for liquids]
    conductance: f32,
    /// Time to close/open [s]
    travel_time: f32,
    /// Current valve state [0..=1, 0=closed]
    state: f32,
}

impl Valve {
    pub fn new(conductance: f32, travel_time: f32) -> Self {
        Self {
            commanded: ValveCommand::Close,
            commanded_time: 0.0,
            conductance,
            travel_time,
            state: 0.0,
        }
    }

    pub fn state(&self) -> f32 {
        self.state
    }

    pub fn conductance(&self) -> f32 {
        self.state * self.conductance
    }

    pub fn command(&mut self, cmd: ValveCommand) {
        self.commanded = cmd;
        self.commanded_time = 0.0;
    }

    pub fn tick(&mut self, dt: f32) {
        let target = match self.commanded {
            ValveCommand::Open => 1.0,
            ValveCommand::Close => 0.0,
            ValveCommand::Partial(p) => p,
            ValveCommand::PulseOpen(dur) if dur.as_secs_f32() > self.commanded_time => 1.0,
            ValveCommand::PulseOpen(_) => 0.0,
        };

        let max_step = dt / self.travel_time;
        let error = target - self.state;
        self.state += error.clamp(-max_step, max_step);

        self.commanded_time += dt;
    }
}
