#![no_std]
#![no_main]

mod mux;

use core::cell::RefCell;
use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Level, Output, Pull},
    otg_fs,
    timer::timg::TimerGroup,
    Config,
};
use esp_hal_embassy::main;
use heapless::Vec;
use midi_convert::midi_types::{Channel, MidiMessage, Note, Value7};
use midi_convert::render_slice::MidiRenderSlice;
use usb_device::prelude::*;
use usbd_midi::{CableNumber, UsbMidiClass, UsbMidiEventPacket};

// Key mapping for the 4051 multiplexer. "255" and "254" are the octave up and down buttons respectively. If you do not wire your buttons in this order, you can adjust this array.
const KEYS: [i32; 27] = [
    255, 254, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24,
];

//Needed for MIDI out
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

/// Global state for keys and octave.
/// "key_note" stores which note is currently being held on each key.
/// This is done so releasing the key will play the correct not off if you change octave.
/// 255 means no note is held.
/// "octave" stores the current octave.
#[derive(Debug)]
pub struct GlobalState {
    pub key_note: [i32; 25],
    pub octave: i32,
}

static GLOBAL_STATE: Mutex<CriticalSectionRawMutex, RefCell<GlobalState>> =
    Mutex::new(RefCell::new(GlobalState {
        key_note: [255; 25],
        octave: 4,
    }));

// Separate mutexes for note ON and note OFF events to prevent deadlock.
static ON_EVENTS: Mutex<CriticalSectionRawMutex, RefCell<Vec<i32, 128>>> =
    Mutex::new(RefCell::new(Vec::new()));
static OFF_EVENTS: Mutex<CriticalSectionRawMutex, RefCell<Vec<i32, 128>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Called on a falling edge (button pressed).
fn falling_edge_handler(index: usize) {
    GLOBAL_STATE.lock(|global_state| {
        // Lock the global state.
        let mut state = global_state.borrow_mut();
        if KEYS[index] == 255 {
            // Check for octave up button.
            if state.octave < 8 {
                state.octave += 1;
            }
        } else if KEYS[index] == 254 {
            // Check for octave down button.
            if state.octave > 0 {
                state.octave -= 1;
            }
        } else {
            // Otherwise, it's a note button.
            let note = KEYS[index] + (state.octave * 12); //Shifts note to current octave.
            state.key_note[KEYS[index] as usize] = note; // Store the note in the key_note array for note-off events.
            ON_EVENTS.lock(|on_events| {
                // Lock the note-on events.
                let mut events = on_events.borrow_mut();
                if events.len() < 128 {
                    events.push(note).ok(); // Push the note-on event. All events in this list will be sent to the MIDI device in the main loop.
                }
            });
        }
    });
}

/// Called on a rising edge (button released).
fn rising_edge_handler(index: usize) {
    GLOBAL_STATE.lock(|global_state| {
        // Lock the global state.
        let mut state = global_state.borrow_mut();
        if KEYS[index] < 254 {
            // If it's not an octave button.
            let note = state.key_note[KEYS[index] as usize]; // Get the note from the key_note array.
            OFF_EVENTS.lock(|off_events| {
                // Lock the note-off events.
                let mut events = off_events.borrow_mut();
                if events.len() < 128 {
                    events.push(note).ok(); // Push the note-off event. All events in this list will be sent to the MIDI device in the main loop.
                }
            });
            state.key_note[KEYS[index] as usize] = 255; // Reset the key_note array for this key.
        }
    });
}

#[embassy_executor::task]
async fn mux_poll_task(mut mux: mux::Multiplexer4051<'static>) {
    // Task for polling the multiplexer.
    mux.poll_all().await;
}

#[main]
async fn main(spawner: Spawner) {
    // Esp32S3 initialization.
    let mut config = Config::default();
    config.cpu_clock = CpuClock::max();
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // USB initialization.
    let usb_bus_allocator = otg_fs::UsbBus::new(
        otg_fs::Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19),
        unsafe { &mut *addr_of_mut!(EP_MEMORY) },
    );

    // Set up the GPIOs for the multiplexer and LEDs.
    let select = [
        Output::new(peripherals.GPIO1, Level::Low),
        Output::new(peripherals.GPIO2, Level::Low),
        Output::new(peripherals.GPIO3, Level::Low),
    ];
    let mut down_led = Output::new(peripherals.GPIO8, Level::Low);
    let mut up_led = Output::new(peripherals.GPIO9, Level::High);

    // Set up the multiplexer.
    let mut mux = mux::Multiplexer4051::new(select); // Create a new multiplexer with the select pins.
                                                     // Create a MuxChipConfig for each chip you want to use.
    let chip1_config =
        mux::MuxChipConfig::new_digital_input(Input::new(peripherals.GPIO4, Pull::Up));
    let chip2_config =
        mux::MuxChipConfig::new_digital_input(Input::new(peripherals.GPIO5, Pull::Up));
    let chip3_config =
        mux::MuxChipConfig::new_digital_input(Input::new(peripherals.GPIO6, Pull::Up));
    let chip4_config =
        mux::MuxChipConfig::new_digital_input(Input::new(peripherals.GPIO7, Pull::Up));
    // Add the chips to the multiplexer.
    mux.add_chip(chip1_config);
    mux.add_chip(chip2_config);
    mux.add_chip(chip3_config);
    mux.add_chip(chip4_config);
    // Set callbacks and spawn the poll task.
    mux.set_falling_edge_callback(falling_edge_handler);
    mux.set_rising_edge_callback(rising_edge_handler);
    spawner.spawn(mux_poll_task(mux)).unwrap();

    // Functions for LED timers for octave indication
    let mut down_led_timer = 0;
    let mut up_led_timer = 0;

    // Global state for keys and octave.
    let mut midi_class = UsbMidiClass::new(&usb_bus_allocator, 1, 1).unwrap();
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus_allocator, UsbVidPid(0x16c0, 0x5e4))
        .device_class(0x01)
        .device_sub_class(0x03)
        .max_packet_size_0(16)
        .unwrap()
        .build();

    loop {
        // Poll USB.
        if usb_dev.poll(&mut [&mut midi_class]) {}

        // --- Process Note ON events ---
        {
            let on_events_to_send = ON_EVENTS.lock(|on_events| {
                // Get the note-on events from the mutex.
                let mut events = on_events.borrow_mut();
                let events_to_send = events.clone();
                events.clear();
                events_to_send
            });
            for note_on in on_events_to_send.into_iter() {
                let mut bytes: [u8; 3] = [0; 3]; // Create a buffer for the MIDI message.
                let message =
                    MidiMessage::NoteOn(Channel::C1, Note::from(note_on as u8), Value7::from(127)); // Create a MIDI message. Currently set to channel 1/127 velocity.
                message.render_slice(&mut bytes); // Render the message to the buffer.
                let packet = // Create a MIDI packet from the buffer.
                    UsbMidiEventPacket::try_from_payload_bytes(CableNumber::Cable0, &bytes)
                        .unwrap();
                let result = midi_class.send_packet(packet); // Send the packet.
                // If sending fails, reinsert the event to prevent dropped MIDI messages.
                if result.is_err() {
                    ON_EVENTS.lock(|on_events| {
                        let mut events = on_events.borrow_mut();
                        events.push(note_on).ok();
                    });
                }
            }
        }

        // --- Process Note OFF events ---
        {
            let off_events_to_send = OFF_EVENTS.lock(|off_events| {
                // Get the note-off events from the mutex.
                let mut events = off_events.borrow_mut();
                let events_to_send = events.clone();
                events.clear();
                events_to_send
            });
            for note_off in off_events_to_send.into_iter() {
                let mut bytes: [u8; 3] = [0; 3]; // Create a buffer for the MIDI message.
                let message =
                    MidiMessage::NoteOff(Channel::C1, Note::from(note_off as u8), Value7::from(0)); // Create a MIDI message. Currently set to channel 1/0 velocity.
                message.render_slice(&mut bytes);// Render the message to the buffer.
                let packet = // Create a MIDI packet from the buffer.
                    UsbMidiEventPacket::try_from_payload_bytes(CableNumber::Cable0, &bytes)
                        .unwrap();
                let result = midi_class.send_packet(packet); // Send the packet.
                // If sending fails, reinsert the event to prevent dropped MIDI messages.
                if result.is_err() {
                    OFF_EVENTS.lock(|off_events| {
                        let mut events = off_events.borrow_mut();
                        events.push(note_off).ok();
                    });
                }
            }
        }

        // Update LED blink based on the current octave.
        let oct = GLOBAL_STATE.lock(|global_state| global_state.borrow().octave);
        if oct > 4 {
            up_led_timer += 1;
            if down_led.is_set_high() {
                down_led.set_low();
            }
            if up_led_timer > ((12 - oct) * 50) {
                up_led.toggle();
                up_led_timer = 0;
            }
        } else if oct < 4 {
            down_led_timer += 1;
            if up_led.is_set_high() {
                up_led.set_low();
            }
            if down_led_timer > ((4 + oct) * 50) {
                down_led.toggle();
                down_led_timer = 0;
            }
        } else {
            if down_led.is_set_high() {
                down_led.set_low();
            }
            if up_led.is_set_high() {
                up_led.set_low();
            }
        }

        Timer::after_millis(1).await;
    }
}
