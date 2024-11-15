#![no_std]
#![no_main]

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

use micromath::F32Ext;

#[repr(C)]
pub struct AirbrakeData {
    altitude: f32,
    speed: f32,
    angle: f32,
}

#[no_mangle]
/// Calculates airbrake deployment percentage based on altitude and speed.
/// 
/// * `altitude` - Altitude in meters.
/// * `speed` - Speed in meters per second.
pub extern "C" fn airbrake_calculate(a: &AirbrakeData) -> u32 {
    // some BS calculation to test it
    return ((a.altitude * 400.0 + a.speed.sqrt() * 2.0) * 100.0).round() as u32;
}