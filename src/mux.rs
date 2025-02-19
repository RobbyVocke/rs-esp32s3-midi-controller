// This module provides a debounced driver for a 4051 8 channel multiplexer chip. Only input is supported currently. 
//Should support up to 8 chips, but has only been tested with 4.
//The driver is designed to be used with the async/await pattern.

//Basic example:
//First make a list of the GPIO pins you will use for the 4051's select pins. These can be shared between multiple chips
//    let select = [
//    Output::new(peripherals.GPIO1, Level::Low),
//    Output::new(peripherals.GPIO2, Level::Low),
//    Output::new(peripherals.GPIO3, Level::Low),
//];
//Then create a new Multiplexer4051 instance with the select pins.
//    let mut mux = mux::Multiplexer4051::new(select);
//Next, create a MuxChipConfig for each chip you want to use. This will require a common GPIO pin for the chip's common pin.
//    let chip_config = mux::MuxChipConfig::new_digital_input(Input::new(peripherals.GPIO4, Pull::Up));
//Add the chip to the Multiplexer4051 instance.
//    mux.add_chip(chip_config);
//Finally, spawn the poll task.
//    spawner.spawn(mux_poll_task(mux)).unwrap();

use core::fmt::Debug;
use embassy_time::Duration;
use embassy_time::{Timer, Instant};
use esp_hal::gpio::{Input, Output, };
use heapless::Vec;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MuxMode {
    DigitalInput,
    DigitalOutput,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SwitchState {
    High,
    Low,
}

pub enum MuxChipConfig<'a> {
    DigitalInput {
        common: Input<'a>,
        states: Vec<SwitchState, 8>,
    },
    DigitalOutput {
        common: Output<'a>,
        states: Vec<bool, 16>,
    },
}

impl<'a> MuxChipConfig<'a> {
    pub fn new_digital_input(common: Input<'a>) -> Self { //This creates a new digital input chip with 8 states. Requires a common GPIO pin.
        let mut states: Vec<SwitchState, 8> = Vec::new();
        for _ in 0..8 {
            states.push(SwitchState::High).ok();
        }
        Self::DigitalInput { common, states }
    }

    pub fn new_digital_output(common: Output<'a>) -> Self { //Not yet implemented.
        let mut states: Vec<bool, 16> = Vec::new();
        for _ in 0..16 {
            states.push(false).ok();
        }
        Self::DigitalOutput { common, states }
    }

    pub fn mode(&self) -> MuxMode {
        match self {
            Self::DigitalInput { .. } => MuxMode::DigitalInput,
            Self::DigitalOutput { .. } => MuxMode::DigitalOutput,
        }
    }
}

pub struct Multiplexer4051<'a> {
    pub select: [Output<'a>; 3], //The GPIO pins for the 4051's select pins.
    pub chips: Vec<MuxChipConfig<'a>, 8>, //The multiplexing chips wired to the micro controller.
    pub digital_in: Vec<SwitchState, 64>, //The stable state of all 64 channels.
    last_change: [Instant; 64], //The last time each channel changed state.
    debounce_interval: Duration, //The debounce interval for all channels.
    pub falling_edge_callback: Option<fn(usize)>, //Callback for when a channel's state changes from high to low.
    pub rising_edge_callback: Option<fn(usize)>, //Callback for when a channel's state changes from low to high.
}

impl<'a> Multiplexer4051<'a> {
    pub fn new(select: [Output<'a>; 3]) -> Self {
        // Initialize the stable state for all 64 channels.
        let mut digital_in: Vec<SwitchState, 64> = Vec::new();
        for _ in 0..64 {
            digital_in.push(SwitchState::High);
        }
        // Default debounce interval is 20ms.
        let debounce_interval = Duration::from_millis(20);
        let now = Instant::now();
        // Initialize each channel's last-change timestamp to allow immediate changes.
        let last_change = [now - debounce_interval; 64];

        Self {
            select,
            chips: Vec::new(),
            digital_in,
            last_change,
            debounce_interval,
            falling_edge_callback: None,
            rising_edge_callback: None,
        }
    }

    /// Allows the main script to change the debounce interval.
    pub fn set_debounce_interval(&mut self, interval: Duration) {
        self.debounce_interval = interval;
    }

    pub fn set_falling_edge_callback(&mut self, callback: fn(usize)) { //Sets the callback for when a channel's state changes from high to low.
        self.falling_edge_callback = Some(callback);
    }

    pub fn set_rising_edge_callback(&mut self, callback: fn(usize)) { //Sets the callback for when a channel's state changes from low to high.
        self.rising_edge_callback = Some(callback);
    }

    pub fn add_chip(&mut self, chip: MuxChipConfig<'a>) { //Adds a chip to the multiplexer.
        self.chips.push(chip).ok();
    }

    fn set_channel(&mut self, channel: u8) { //Sets the channel on the 4051.
        let bits = [(channel >> 0) & 1, (channel >> 1) & 1, (channel >> 2) & 1];
        for (pin, &bit) in self.select.iter_mut().zip(&bits) {
            if bit == 0 {
                let _ = pin.set_low();
            } else {
                let _ = pin.set_high();
            }
        }
    }

    /// Debounced polling for a single multiplexer channel using a time-based debounce.
    ///
    /// - `reading`: the raw reading from the chip’s common pin (true if low, i.e. pressed).
    /// - `read_channel`: the multiplexer channel (0..7).
    /// - `chip_offset`: which chip (in our chips Vec) is being read.
    fn poll_digital_input_chip(
        &mut self,
        reading: bool,
        read_channel: usize,
        chip_offset: u8,
    ) {
        let index = read_channel + (8 * chip_offset as usize);
        let current_state = self.digital_in[index];
        // Map the raw reading into our stable state.
        // (true means the input is low/pressed → Low state;
        //  false means not pressed → High state)
        let expected_state = if reading { SwitchState::Low } else { SwitchState::High };

        let now = Instant::now();
        if current_state != expected_state {
            // Only accept the change if the debounce interval has elapsed.
            if now.duration_since(self.last_change[index]) >= self.debounce_interval {
                self.digital_in[index] = expected_state;
                self.last_change[index] = now;
                if expected_state == SwitchState::Low {
                    if let Some(callback) = self.falling_edge_callback {
                        callback(index);
                    }
                } else {
                    if let Some(callback) = self.rising_edge_callback {
                        callback(index);
                    }
                }
            }
        }
    }

    /// Continuously polls all channels on all chips. Currently only checks chips set to digital input.
    pub async fn poll_all(&mut self) {
        loop {
            for channel in 0..8 {
                let read_channel = channel as usize;
                self.set_channel(channel);
                Timer::after_micros(50).await; // Wait for the channel to change in the multiplexing IC.
                let mut common_states: Vec<bool, 8> = Vec::new();
                for chip in self.chips.iter_mut() {
                    if let MuxChipConfig::DigitalInput { common, .. } = chip {
                        // With Pull-Up inputs, a pressed button pulls the pin low.
                        common_states.push(common.is_low()).ok();
                    }
                }
                for (chip_index, &state) in common_states.iter().enumerate() {
                    self.poll_digital_input_chip(state, read_channel, chip_index as u8);
                }
            }
        }
    }
}
