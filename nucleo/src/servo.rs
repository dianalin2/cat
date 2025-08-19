use defmt::info;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub type SimplePwmMutex<'a, T> = Mutex<CriticalSectionRawMutex, Option<SimplePwm<'a, T>>>;

pub struct Servo<'a, T>
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
    pub pwm: &'a SimplePwmMutex<'a, T>,
    pub channel: Channel,
}

impl<'a, T> Servo<'a, T>
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
    pub async fn new(pwm: &'a SimplePwmMutex<'a, T>, channel: Channel) -> Self {
        if let Some(pwm_ref) = pwm.lock().await.as_mut() {
            pwm_ref.channel(channel).enable();
            info!("Servo control started!");
        }

        Self { pwm: pwm, channel }
    }

    pub async fn set_position(&self, pos: i32) {
        if let Some(pwm_ref) = self.pwm.lock().await.as_mut() {
            let duty_cycle = ((pos + 90 + 45) as u32) * (pwm_ref.max_duty_cycle() as u32) / 1800;
            pwm_ref
                .channel(self.channel)
                .set_duty_cycle(duty_cycle as u16);
        }
    }
}
