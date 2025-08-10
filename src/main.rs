#![no_std]
#![no_main]

use core::sync::atomic::{AtomicI32, Ordering};
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::{TIM1, TIM10};
use embassy_stm32::time::hz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use {defmt_rtt as _, panic_probe as _};

type SimplePwmMutex = Mutex<CriticalSectionRawMutex, Option<SimplePwm<'static, TIM1>>>;
static PWM: SimplePwmMutex = Mutex::new(None);
static SERVO_POSITION: AtomicI32 = AtomicI32::new(0); // Initial position in angle (-90 to 90 degrees)

#[embassy_executor::task(pool_size = 4)]
async fn servo_control(pwm: &'static SimplePwmMutex, channel: Channel) {
    if let Some(pwm_ref) = pwm.lock().await.as_mut() {
        pwm_ref.channel(channel).enable();
        info!("Servo control started!");
    }

    loop {
        let pos = SERVO_POSITION.load(Ordering::Relaxed);

        if let Some(pwm_ref) = pwm.lock().await.as_mut() {
            let duty_cycle = ((pos + 90 + 45) as u32) * (pwm_ref.max_duty_cycle() as u32) / 1800;

            info!(
                "Updating servo position. Position: {} degrees, Duty Cycle: {}/{}",
                pos,
                duty_cycle,
                pwm_ref.max_duty_cycle()
            );

            pwm_ref.channel(channel).set_duty_cycle(duty_cycle as u16);
        }

        embassy_time::Timer::after(Duration::from_millis(100)).await; // Update every 100 ms
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let servo_pin = PwmPin::new_ch1(p.PE9, OutputType::PushPull); // D6

    // Initialize the PWM channel for the servo
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(servo_pin),
        None,
        None,
        None,
        hz(50), // 50 Hz for servo control
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    {
        *(PWM.lock().await) = Some(pwm);
    }

    spawner.spawn(servo_control(&PWM, Channel::Ch1)).unwrap();

    loop {
        // Simulate changing the servo position
        for pos in (-90..=90).step_by(45) {
            SERVO_POSITION.store(pos, Ordering::Relaxed);
            embassy_time::Timer::after(Duration::from_millis(1000)).await; // Wait 1 second between changes
        }

        embassy_time::Timer::after(Duration::from_secs(5)).await; // Wait 5 seconds before repeating
    }
}
