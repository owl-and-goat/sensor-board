pub mod addr {
    #[expect(dead_code)]
    pub const DISTANCE: u8 = 0x52;
    #[expect(dead_code)]
    pub const COLOR: u8 = 0x38;
    #[expect(dead_code)]
    pub const TEMP_HUMIDITY: u8 = 0x44;
    #[expect(dead_code)]
    pub const ACCEL: u8 = 0x32;
    pub const CAPACITANCE: u8 = 0x2A;
}

#[derive(Clone, Copy, defmt::Format)]
#[expect(dead_code)]
pub enum Device {
    Distance,
    Color,
    TempHumidity,
    Accel,
    Capacitance,
}

impl Device {
    #[expect(dead_code)]
    pub fn addr(self) -> u8 {
        match self {
            Device::Distance => addr::DISTANCE,
            Device::Color => addr::COLOR,
            Device::TempHumidity => addr::TEMP_HUMIDITY,
            Device::Accel => addr::ACCEL,
            Device::Capacitance => addr::CAPACITANCE,
        }
    }
}
