#![no_std]

///! Requests that can be sent *to* the sensor board
#[derive(Debug, Clone, defmt::Format)]
pub enum ToBoard {
    Heartbeat,
}

///! Requests that can be received *from* the sensor board
#[derive(Debug, Clone, defmt::Format)]
pub enum FromBoard {
    Heartbeat,
}
