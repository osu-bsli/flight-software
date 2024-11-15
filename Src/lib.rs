#![no_std]
use micromath::F32Ext;

#[no_mangle]
/// Calculates airbrake deployment percentage based on altitude and speed.
/// 
/// * `altitude` - Altitude in meters.
/// * `speed` - Speed in meters per second.
pub extern "C" fn airbrake_calculate(altitude: f32, speed: f32) -> u32 {
    // some BS calculation to test it
    return ((altitude * 400.0 + speed.sqrt() * 2.0) * 100.0).round() as u32;
}