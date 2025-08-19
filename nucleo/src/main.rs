#![no_std]
#![no_main]

use defmt::{debug, info};
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::time::hz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use {defmt_rtt as _, panic_probe as _};
mod servo;

type TIMER = TIM1;

static PWM: servo::SimplePwmMutex<TIMER> = Mutex::new(None);
static SERVOS: Mutex<CriticalSectionRawMutex, [Option<servo::Servo<'static, TIMER>>; 2]> =
    Mutex::new([None, None]);

#[embassy_executor::task(pool_size = 4)]
async fn servo_control(
    servos: &'static Mutex<CriticalSectionRawMutex, [Option<servo::Servo<'static, TIMER>>; 2]>,
) {
    loop {
        // Simulate changing the servo position
        // for pos in (-90..=90).step_by(45) {
        //     info!("Setting servo position to {} degrees", pos);

        //     {
        //         let servos = &servos.lock().await;
        //         if let Some(servo) = &servos[0] {
        //             servo.set_position(pos).await;
        //         }
        //         if let Some(servo_2) = &servos[1] {
        //             servo_2.set_position(-pos).await;
        //         }
        //     }

        //     embassy_time::Timer::after(Duration::from_millis(1000)).await; // Wait 1 second between changes
        // }
        {
            let servos = &servos.lock().await;
            for (i, servo) in servos.iter().enumerate() {
                if let Some(servo) = servo {
                    info!("Setting servo {} position to 0 degrees", i + 1);
                    servo.set_position(0).await; // Set all servos to 0 degrees
                }
            }
        }

        embassy_time::Timer::after(Duration::from_secs(2)).await; // Wait 2 seconds before repeating
    }
}

async fn zero_servo_positions() {
    info!("Zeroing servo positions");
    let mut servos = SERVOS.lock().await;
    for servo in servos.iter_mut() {
        if let Some(s) = servo {
            s.set_position(0).await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let servo_pin = PwmPin::new_ch1(p.PE9, OutputType::PushPull); // D6
    let servo_pin_2 = PwmPin::new_ch2(p.PA9, OutputType::PushPull);

    // Initialize the PWM channel for the servo
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(servo_pin),
        Some(servo_pin_2),
        None,
        None,
        hz(50), // 50 Hz for servo control
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    {
        *(PWM.lock().await) = Some(pwm);
    }

    let servo_1 = servo::Servo::new(&PWM, Channel::Ch1).await;
    let servo_2 = servo::Servo::new(&PWM, Channel::Ch2).await;

    {
        let mut servos_lock = SERVOS.lock().await;
        servos_lock[0] = Some(servo_1);
        servos_lock[1] = Some(servo_2);
    }

    spawner.spawn(servo_control(&SERVOS)).unwrap();

    loop {
        debug!("HEARTBEAT");
        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}
