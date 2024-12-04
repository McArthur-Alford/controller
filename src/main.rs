#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull::{self};
use embassy_stm32::gpio::{Input, Level, Output, Pin, Speed};
use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH3, SPI1};
use embassy_stm32::spi::{BitOrder, Config, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

struct CirqueRAP<'a, CS: Pin, DR: Pin> {
    spi: Spi<'a, SPI1, DMA2_CH3, DMA2_CH2>,
    cs: Output<'a, CS>,
    dr: ExtiInput<'a, DR>,
}

#[derive(defmt::Format)]
struct TouchData {
    x_value: u16,
    y_value: u16,
    z_value: u8,
    button_flags: u8,
    touch_down: bool,
    hovering: bool,
}

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        // config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz(10_000_000);
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.mode = embassy_stm32::spi::MODE_1;

    let spi = Spi::new(
        p.SPI1, p.PB3, p.PB5, p.PB4, p.DMA2_CH3, p.DMA2_CH2, spi_config,
    );
    let cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);
    let dr = Input::new(p.PB0, Pull::Down);
    let dr = ExtiInput::new(dr, p.EXTI0);

    let mut cirque = CirqueRAP { spi, cs, dr };

    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    cirque.init().await;

    loop {
        cirque.ready().await;
        let result = cirque.read_absolute().await;
        if result.z_idle_packet() {
            led.set_low();
        } else {
            led.set_high();
        }

        info!("{}", result);
    }
}

impl<'a, CS: Pin, DR: Pin> CirqueRAP<'a, CS, DR> {
    async fn rap_write(&mut self, address: u8, data: u8) {
        self.cs.set_low();
        let mut buf = [0x80 | address, data];
        self.spi.transfer_in_place(&mut buf).await;
        self.cs.set_high();
        Timer::after_micros(50).await;
    }

    async fn rap_read(&mut self, address: u8, data: &mut [u8]) {
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

    async fn enable_feed(&mut self, feed_enable: bool) {
        let mut temp = [0u8; 1];
        self.rap_read(0x04, &mut temp).await;

        if feed_enable {
            temp[0] |= 0x01;
        } else {
            temp[0] &= !0x01;
        }

        self.rap_write(0x04, temp[0]).await;
    }

    async fn clear_flags(&mut self) {
        self.rap_write(0x02, 0).await;
    }

    async fn ready(&mut self) {
        self.dr.wait_for_high().await;
    }

    async fn read_absolute(&mut self) -> TouchData {
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

    async fn era_write_byte(&mut self, address: u16, data: u8) {
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

    async fn era_read_bytes(&mut self, address: u16, data: &mut [u8]) {
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

    async fn set_adc_attenuation(&mut self, adc_gain: u8) {
        let mut temp = [0u8; 1];

        self.era_read_bytes(0x0187, &mut temp).await;

        temp[0] &= 0x3F; // Clear top two bits
        temp[0] |= adc_gain;

        self.era_write_byte(0x0187, temp[0]).await;

        self.era_read_bytes(0x0187, &mut temp).await;

        info!("ADC gain set to: {:#X}", temp[0] & 0xC0);
    }

    async fn tune_edge_sensitivity(&mut self) {
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

    async fn init(&mut self) {
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
    fn clip_coordinates(&mut self) {
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

    fn scale_data(&mut self, x_resolution: u16, y_resolution: u16) {
        self.clip_coordinates();

        let x_temp = (self.x_value - PINNACLE_X_LOWER as u16) as u32;
        let y_temp = (self.y_value - PINNACLE_Y_LOWER as u16) as u32;

        self.x_value = (x_temp * x_resolution as u32 / PINNACLE_X_RANGE as u32) as u16;
        self.y_value = (y_temp * y_resolution as u32 / PINNACLE_Y_RANGE as u32) as u16;
    }

    fn is_hovering(&self) -> bool {
        let zone_x = self.x_value as usize / ZONESCALE;
        let zone_y = self.y_value as usize / ZONESCALE;

        let zone_x = zone_x.min(COLS_X - 1);
        let zone_y = zone_y.min(ROWS_Y - 1);

        !(self.z_value > ZVALUE_MAP[zone_y][zone_x])
    }

    fn z_idle_packet(&self) -> bool {
        self.x_value == 0 && self.y_value == 0 && self.z_value == 0
    }
}
