#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::task;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    i2c,
    Async,
    i2c::master::I2c
};
use esp_println::println;
use esp_wifi::ble::controller::BleConnector;
use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

type I2cShared = Mutex<NoopRawMutex, Option<I2c<'static, Async>>>;
static I2C1_SHARED: StaticCell<I2cShared> = StaticCell::new();


#[task]
//async fn task_EEPROM(mut i2c1: I2c<'static, Async>) {
async fn task_EEPROM(i2c1: &'static I2cShared) {
    loop {
        println!("EEPROM Task");
        let address: u8 = 0x58;
        let write_buffer:[u8; 1] = [0x00];
        let mut read_buffer: [u8; 1] = [0x00];
        {
            let mut i2c1_mutex = i2c1.lock().await;
            i2c1_mutex.as_mut().unwrap().write_read_async(address, &write_buffer, &mut read_buffer).await;
        }
        
        println!("{:?}", read_buffer);
        Timer::after(Duration::from_millis(250)).await;
    }
}

#[task]
//async fn task_EEPROM(mut i2c1: I2c<'static, Async>) {
async fn task_HTSensor(i2c1: &'static I2cShared) {
    loop {
        println!("HT Task");
        let address: u8 = 0x5C;
        let write_buffer:[u8; 1] = [0x0F];
        let mut read_buffer: [u8; 1] = [0x00];
        {
            let mut i2c1_mutex = i2c1.lock().await;
            i2c1_mutex.as_mut().unwrap().write_read_async(address, &write_buffer, &mut read_buffer).await;
        }
        
        println!("{:?}", read_buffer);
        Timer::after(Duration::from_millis(500)).await;
    }
}


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    let _connector = BleConnector::new(&wifi_init, peripherals.BT);


    // Task 1: I2C Task
    let i2c1 = I2c::new(peripherals.I2C1, i2c::master::Config::default())
        .unwrap()
        .with_scl(peripherals.GPIO5)
        .with_sda(peripherals.GPIO4)
        .into_async();

    let i2c1_shared = I2C1_SHARED.init(Mutex::new(Some(i2c1)));

    spawner.spawn(task_HTSensor(i2c1_shared)).unwrap();
    spawner.spawn(task_EEPROM(i2c1_shared)).unwrap();
    // spawner.spawn(task_HTSensor(i2c1)).unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}
