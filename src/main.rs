#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;

mod cirque;
mod spi_handler;
mod state;
use cirque::*;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    spi::{self, Spi},
    time::Hertz,
    Peripherals,
};
use state::input_task;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    // {
    //     use embassy_stm32::rcc::*;
    //     config.rcc.hse = Some(Hse {
    //         freq: Hertz(8_000_000),
    //         mode: HseMode::Bypass,
    //     });
    //     config.rcc.pll_src = PllSource::HSE;
    //     config.rcc.pll = Some(Pll {
    //         prediv: PllPreDiv::DIV4,
    //         mul: PllMul::MUL168,
    //         divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
    //         divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
    //         divr: None,
    //     });
    //     config.rcc.ahb_pre = AHBPrescaler::DIV1;
    //     config.rcc.apb1_pre = APBPrescaler::DIV4;
    //     config.rcc.apb2_pre = APBPrescaler::DIV2;
    //     config.rcc.sys = Sysclk::PLL1_P;
    //     // config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    // }
    let p = embassy_stm32::init(config);

    spawner.spawn(input_task());

    // let mut ep_out_buffer = [0u8; 256];
    // let mut config = embassy_stm32::usb_otg::Config::default();
    // config.vbus_detection = true;

    // let driver = Driver::new_fs(
    //     p.USB_OTG_FS,
    //     Irqs,
    //     p.PA12,
    //     p.PA11,
    //     &mut ep_out_buffer,
    //     config,
    // );

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(10_000_000);
    spi_config.bit_order = spi::BitOrder::MsbFirst;
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
