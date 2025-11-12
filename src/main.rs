#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{self as _, Config};

use defmt_rtt as _;
use embassy_time::Timer;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::default();
    let _peripherals = embassy_stm32::init(config);
    loop {
        info!("Hello");
        Timer::after_secs(1).await;
    }
}