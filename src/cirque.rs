use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output, Pin};
use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH3, SPI1};
use embassy_stm32::spi::Spi;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const PINNACLE_X_MAX: usize = 2047;
const PINNACLE_Y_MAX: usize = 1535;
const PINNACLE_X_LOWER: usize = 127;
const PINNACLE_X_UPPER: usize = 1919;
const PINNACLE_Y_LOWER: usize = 63;
const PINNACLE_Y_UPPER: usize = 1471;
const PINNACLE_X_RANGE: usize = PINNACLE_X_UPPER - PINNACLE_X_LOWER;
const PINNACLE_Y_RANGE: usize = PINNACLE_Y_UPPER - PINNACLE_Y_LOWER;
const ZONESCALE: usize = 256;
const ROWS_Y: usize = (PINNACLE_Y_MAX + 1) / ZONESCALE;
const COLS_X: usize = (PINNACLE_X_MAX + 1) / ZONESCALE;

const ZVALUE_MAP: [[u8; COLS_X]; ROWS_Y] = [
    [100, 100, 100, 100, 100, 100, 100, 100],
    [100, 100, 100, 100, 100, 100, 100, 100],
    [100, 100, 100, 100, 100, 100, 100, 100],
    [100, 100, 100, 100, 100, 100, 100, 100],
    [100, 100, 100, 100, 100, 100, 100, 100],
    [100, 100, 100, 100, 100, 100, 100, 100],
];

/// `CirqueRAP` provides asynchronous methods to communicate with a Cirque touch controller using the Read Access Protocol (RAP).
///
/// # Type Parameters
/// - `CS`: The type representing the Chip Select (CS) pin, which must implement the `Pin` trait for controlling the CS line.
/// - `DR`: The type representing the Data Ready (DR) pin, which must implement the `Pin` trait for monitoring the DR signal.
pub struct CirqueRAP<'a, CS: Pin, DR: Pin> {
    pub spi: Spi<'a, SPI1, DMA2_CH3, DMA2_CH2>,
    pub cs: Output<'a, CS>,
    pub dr: ExtiInput<'a, DR>,
}

/// Represents touch data received from the Cirque touch controller.
///
/// Contains the X and Y coordinates, Z (pressure) value, button flags, and status indicators.
///
/// # Fields
///
/// - `x_value`: The X-coordinate of the touch.
/// - `y_value`: The Y-coordinate of the touch.
/// - `z_value`: The Z (pressure) value of the touch.
/// - `button_flags`: Flags indicating the status of buttons.
/// - `touch_down`: Indicates whether the touch is currently active (touch down).
/// - `hovering`: Indicates whether the touch is considered hovering.
#[derive(defmt::Format, Default, Clone, Copy)]
pub struct TouchData {
    x_value: u16,
    y_value: u16,
    z_value: u8,
    button_flags: u8,
    touch_down: bool,
    hovering: bool,
}

impl<'a, CS: Pin, DR: Pin> CirqueRAP<'a, CS, DR> {
    /// Asynchronously writes a single byte to a given register address using the RAP protocol.
    ///
    /// # Parameters
    ///
    /// - `address`: The 8-bit register address to write to.
    /// - `data`: The 8-bit data to write to the register.
    pub async fn rap_write(&mut self, address: u8, data: u8) {
        self.cs.set_low();
        let mut buf = [0x80 | address, data];
        self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        Timer::after_micros(50).await;
    }

    /// Asynchronously reads data from a given register address using the RAP protocol.
    ///
    /// # Parameters
    ///
    /// - `address`: The 8-bit register address to read from.
    /// - `data`: A mutable slice to store the data. The length determines how many bytes are read.
    pub async fn rap_read(&mut self, address: u8, data: &mut [u8]) {
        for word in data.iter_mut() {
            *word = 0xfc;
        }
        self.cs.set_low();
        let mut buf = [0xA0 | address, 0xfc, 0xfc];
        self.spi.transfer_in_place(&mut buf).await;
        self.spi.transfer_in_place(data).await;
        self.cs.set_high();
        Timer::after_micros(50).await;
    }

    /// Enables or disables the data feed from the touch controller.
    ///
    /// # Parameters
    ///
    /// - `feed_enable`: A boolean indicating whether to enable (`true`) or disable (`false`) the feed.
    pub async fn enable_feed(&mut self, feed_enable: bool) {
        let mut temp = [0u8; 1];
        self.rap_read(0x04, &mut temp).await;

        if feed_enable {
            temp[0] |= 0x01;
        } else {
            temp[0] &= !0x01;
        }

        self.rap_write(0x04, temp[0]).await;
    }

    /// Clears the touch controller's flags by writing zero to the flags register.
    pub async fn clear_flags(&mut self) {
        self.rap_write(0x02, 0).await;
    }

    /// Waits until the Data Ready (DR) pin goes high, indicating that data is available to read.
    pub async fn ready(&mut self) {
        self.dr.wait_for_high().await;
    }

    /// Reads absolute touch data from the touch controller.
    ///
    /// This method reads the touch data, clears the flags, and processes the data into a `TouchData` struct.
    ///
    /// # Returns
    ///
    /// A `TouchData` instance containing the touch coordinates, pressure, and status flags.
    pub async fn read_absolute(&mut self) -> TouchData {
        let mut buf = [0, 0, 0, 0, 0, 0];
        self.rap_read(0x12, &mut buf).await;
        self.clear_flags().await;

        let button_flags = buf[0] & 0x3F;
        let x_value: u16 = buf[2] as u16 | ((buf[4] as u16 & 0x0f) << 8);
        let y_value: u16 = buf[3] as u16 | ((buf[4] as u16 & 0xf0) << 4);
        let z_value = buf[5];

        let touch_down = x_value != 0;

        let mut data = TouchData {
            x_value,
            y_value,
            z_value,
            button_flags,
            touch_down,
            hovering: false,
        };

        data.touch_down = data.z_idle_packet();
        data.hovering = data.is_hovering();
        data
    }

    /// Writes a single byte to the touch controller's ERA (Extended Register Access) memory space.
    /// The documentation for this is either incredibly well hidden or nonexistant, but we can figure out enough useful registers from the existing examples to get the trackpad working.
    ///
    /// # Usage Note
    /// This will disable the feed, but not re-enable it.
    /// Because they say to do it in the official docs, we *must* clear flags at the end of this. Unsure why.
    ///
    /// # Parameters
    ///
    /// - `address`: The 16-bit address in the ERA space to write to.
    /// - `data`: The 8-bit data to write.
    pub async fn era_write_byte(&mut self, address: u16, data: u8) {
        self.enable_feed(false).await;

        self.rap_write(0x1B, data).await;

        self.rap_write(0x1c, (address >> 8) as u8).await;
        self.rap_write(0x1d, (address & 0xFF) as u8).await;

        self.rap_write(0x1e, 0x02).await;

        let mut era_control_value = [0xffu8; 1];
        loop {
            self.rap_read(0x1e, &mut era_control_value).await;
            if era_control_value[0] == 0 {
                break;
            }
        }

        self.clear_flags().await;
    }

    /// Reads multiple bytes from the touch controller's ERA (Extended Register Access) memory space.
    ///
    /// # Usage Note
    /// This will disable the feed, but not re-enable it.
    /// Because they say to do it in the official docs, we *must* clear flags at the end of this. Unsure why.
    ///
    /// # Parameters
    ///
    /// - `address`: The 16-bit starting address in the ERA space to read from.
    /// - `data`: A mutable slice to store the read data.
    pub async fn era_read_bytes(&mut self, address: u16, data: &mut [u8]) {
        self.enable_feed(false).await;

        self.rap_write(0x1c, (address >> 8) as u8).await;
        self.rap_write(0x1d, (address & 0xff) as u8).await;

        for byte in data.iter_mut() {
            self.rap_write(0x1e, 0x05).await;

            let mut era_control_value = [0xffu8; 1];
            loop {
                self.rap_read(0x1e, &mut era_control_value).await;

                if era_control_value[0] == 0 {
                    break;
                }
            }

            self.rap_read(0x1b, &mut [*byte]).await;
            self.clear_flags().await;
        }
    }

    /// Sets the ADC attenuation setting for the touch controller.
    ///
    /// # Parameters
    ///
    /// - `adc_gain`: The ADC attenuation setting (e.g., `0x00` for 1x, `0x40` for 2x).
    pub async fn set_adc_attenuation(&mut self, adc_gain: u8) {
        let mut temp = [0u8; 1];

        self.era_read_bytes(0x0187, &mut temp).await;

        temp[0] &= 0x3F; // Clear top two bits
        temp[0] |= adc_gain;

        self.era_write_byte(0x0187, temp[0]).await;

        self.era_read_bytes(0x0187, &mut temp).await;

        info!("ADC gain set to: {:#X}", temp[0] & 0xC0);
    }

    /// Tunes the edge sensitivity settings of the touch controller.
    ///
    /// Adjusts the `WideZMin` parameters for the X and Y axes to improve edge detection sensitivity.
    /// This is ripped from the examples, there is very little explanation of how it works and no documentation :(.
    pub async fn tune_edge_sensitivity(&mut self) {
        let mut temp = [0u8; 1];

        self.era_read_bytes(0x0149, &mut temp).await;
        info!("Current xAxis.WideZMin: {:#X}", temp[0]);
        self.era_write_byte(0x0149, 0x04).await;
        self.era_read_bytes(0x0149, &mut temp).await;
        info!("New xAxis.WideZMin: {:#X}", temp[0]);

        self.era_read_bytes(0x0168, &mut temp).await;
        info!("Current yAxis.WideZMin: {:#X}", temp[0]);
        self.era_write_byte(0x0168, 0x03).await;
        self.era_read_bytes(0x0168, &mut temp).await;
        info!("New yAxis.WideZMin: {:#X}", temp[0]);
    }

    /// Initializes the touch controller with default settings.
    ///
    /// This method performs the following steps:
    /// - Clears any existing flags.
    /// - Configures control registers.
    /// - Enables the absolute output mode.
    /// - Adjusts Z-idle packet count.
    /// - Sets ADC attenuation.
    /// - Tunes edge sensitivity.
    /// - Enables the data feed.
    pub async fn init(&mut self) {
        // Clear flags
        self.clear_flags().await;

        // Configure registers 0x03 and 0x05
        self.rap_write(0x03, 0).await;
        self.rap_write(0x05, 0x1F).await;

        // Enable preferred output mode (absolute)
        self.rap_write(0x04, 0x03).await;

        // Set z-idle packet count to 5 (default is 30)
        self.rap_write(0x0A, 5).await;

        info!("Pinnacle Initialized...");

        // Set ADC attenuation to ADC_ATTENUATE_2X
        self.set_adc_attenuation(0x40).await;

        // Tune edge sensitivity
        self.tune_edge_sensitivity().await;

        // Enable the feed
        self.enable_feed(true).await;
    }
}

impl TouchData {
    /// Clips the touch coordinates to be within the defined bounds.
    ///
    /// Ensures that `x_value` and `y_value` are within the lower and upper limits defined by `PINNACLE_X_LOWER`, `PINNACLE_X_UPPER`, `PINNACLE_Y_LOWER`, and `PINNACLE_Y_UPPER`.
    pub fn clip_coordinates(&mut self) {
        if self.x_value < PINNACLE_X_LOWER as u16 {
            self.x_value = PINNACLE_X_LOWER as u16;
        } else if self.x_value > PINNACLE_X_UPPER as u16 {
            self.x_value = PINNACLE_X_UPPER as u16;
        }

        if self.y_value < PINNACLE_Y_LOWER as u16 {
            self.y_value = PINNACLE_Y_LOWER as u16;
        } else if self.y_value > PINNACLE_Y_UPPER as u16 {
            self.y_value = PINNACLE_Y_UPPER as u16;
        }
    }

    /// Scales the touch data to the specified resolution.
    ///
    /// This method adjusts the `x_value` and `y_value` based on the provided `x_resolution` and `y_resolution`, after clipping the coordinates.
    ///
    /// # Parameters
    ///
    /// - `x_resolution`: The desired horizontal resolution.
    /// - `y_resolution`: The desired vertical resolution.
    pub fn scale_data(&mut self, x_resolution: u16, y_resolution: u16) {
        self.clip_coordinates();

        let x_temp = (self.x_value - PINNACLE_X_LOWER as u16) as u32;
        let y_temp = (self.y_value - PINNACLE_Y_LOWER as u16) as u32;

        self.x_value = (x_temp * x_resolution as u32 / PINNACLE_X_RANGE as u32) as u16;
        self.y_value = (y_temp * y_resolution as u32 / PINNACLE_Y_RANGE as u32) as u16;
    }

    /// Determines whether the touch input is considered hovering based on the Z-value.
    ///
    /// Uses a predefined Z-value map (`ZVALUE_MAP`) and scales the X and Y coordinates to determine the hovering state.
    ///
    /// # Returns
    ///
    /// - `true` if the touch is considered hovering.
    /// - `false` otherwise.
    pub fn is_hovering(&self) -> bool {
        let zone_x = self.x_value as usize / ZONESCALE;
        let zone_y = self.y_value as usize / ZONESCALE;

        let zone_x = zone_x.min(COLS_X - 1);
        let zone_y = zone_y.min(ROWS_Y - 1);

        !(self.z_value > ZVALUE_MAP[zone_y][zone_x])
    }

    /// Checks if the current data represents a Z-idle packet.
    ///
    /// A Z-idle packet is identified when `x_value`, `y_value`, and `z_value` are all zero.
    ///
    /// # Returns
    ///
    /// - `true` if the data is a Z-idle packet.
    /// - `false` otherwise.
    pub fn z_idle_packet(&self) -> bool {
        self.x_value == 0 && self.y_value == 0 && self.z_value == 0
    }
}
