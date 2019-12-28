use rppal::uart::*;
use sps30::{DeviceInfo, Sps30};
use std::thread;
use std::time::Duration;

// activate uart in raspi-config
fn main() {
    if let Err(e) = run() {
        eprintln!("Program exited early with error: {}", e);
    }
}

fn run() -> Result<()> {
    // Configure UART
    let mut serial = Uart::new(115_200, Parity::None, 8, 1)?;
    serial.set_hardware_flow_control(false).unwrap();
    serial.set_software_flow_control(false).unwrap();
    serial.set_rts(false).unwrap();
    serial.set_write_mode(true).unwrap();
    serial.set_read_mode(1, Duration::new(0, 0)).unwrap();

    let mut sps30 = Sps30::new(serial);
    sps30.reset().unwrap();
    thread::sleep(Duration::from_millis(10000));
    sps30.start_measurement().unwrap();

    for _ in 0..10 {
        thread::sleep(Duration::from_millis(10000));

        let res = sps30.read_measurement().unwrap();
        println!("Mass Concentration PM1.0 [μg/m³] {:?}", res[0]);
        println!("Mass Concentration PM2.5 [μg/m³] {:?} ", res[1]);
        println!("Mass Concentration PM4.0 [μg/m³] {}", res[2]);
        println!("Mass Concentration PM10 [μg/m³] {}", res[3]);
        println!("Number Concentration PM0.5 [#/cm³] {}", res[4]);
        println!("Number Concentration PM1.0 [#/cm³] {}", res[5]);
        println!("Number Concentration PM2.5 [#/cm³] {}", res[6]);
        println!("Number Concentration PM4.0 [#/cm³] {}", res[7]);
        println!("Number Concentration PM10 [#/cm³] {}", res[8]);
        println!("Typical Particle Size [μm] {}", res[9]);
    }

    println!(
        "Clean interval= {}",
        sps30.read_cleaning_interval().unwrap()
    );

    sps30.stop_measurement().unwrap();

    let product_name =
        String::from_utf8(sps30.device_info(DeviceInfo::ProductName).unwrap().to_vec()).unwrap();
    println!("Product Name = {}", product_name);

    let serial_number = String::from_utf8(
        sps30
            .device_info(DeviceInfo::SerialNumber)
            .unwrap()
            .to_vec(),
    )
    .unwrap();
    println!("Serial Number = {}", serial_number);

    let article_code =
        String::from_utf8(sps30.device_info(DeviceInfo::ArticleCode).unwrap().to_vec()).unwrap();
    println!("Article Code = {}", article_code);

    Ok(())
}
