#![no_std]
#![no_main]
use micromath::F32Ext;


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

pub extern "C" fn airbrake_calculate(altitude: f32, speed: f32) -> u32 {
    // some BS calculation to test it
    return ((altitude * 400.0 + speed.sqrt() * 2.0) * 100.0).round() as u32;
}