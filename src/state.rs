use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration};

use crate::TouchData;

static INPUT_EVENT_CHANNEL: Channel<ThreadModeRawMutex, InputEvent, 2> = Channel::new();
static INPUT_CHANNEL: Channel<ThreadModeRawMutex, InputState, 2> = Channel::new();

pub enum InputEvent {
    TouchEvent(TouchData),
}

/// Records the complete state of all inputs
#[derive(Default, Clone, Copy)]
pub struct InputState {
    /// Data for the trackpad
    pub touch_data: TouchData,
}

#[embassy_executor::task]
pub async fn input_task() -> ! {
    let mut state = InputState::default();
    loop {
        if let Ok(event) =
            with_timeout(Duration::from_millis(500), INPUT_EVENT_CHANNEL.receive()).await
        {
            match event {
                InputEvent::TouchEvent(touch_data) => {
                    state.touch_data = touch_data;
                }
            }
            INPUT_CHANNEL.try_send(state);
        };
    }
}
