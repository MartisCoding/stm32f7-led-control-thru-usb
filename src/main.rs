#![no_std]
#![no_main]


mod fmt;

use core::ops::Deref;
use cortex_m::prelude::_embedded_hal_Pwm;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::{bind_interrupts, Config, Peri};
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};

use embassy_stm32::pac::rcc::vals::{Plln, Ppre};
use embassy_stm32::peripherals;
use embassy_stm32::peripherals::{DMA2_CH5, TIM1};
use embassy_stm32::rcc::AHBPrescaler::DIV1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{SimplePwm, PwmPin};
use embassy_stm32::usb::{Driver, Instance, InterruptHandler};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use fmt::info;


bind_interrupts!(struct Irqs {
    OTG_FS => InterruptHandler<peripherals::USB_OTG_FS>;
});

static RGB_DATA: Mutex<ThreadModeRawMutex, [(u8, u8, u8); 8]> = Mutex::new([(0, 0, 0); 8]);


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello there!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: Plln::MUL216,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV9),
            divr: None,
        });
        config.rcc.ahb_pre = DIV1;
        config.rcc.apb1_pre = Ppre::DIV4;
        config.rcc.apb2_pre = Ppre::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();
    config.vbus_detection = true;

    let driver = Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, &mut ep_out_buffer, config);

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Embassy");
    usb_config.product = Some("Serial port study.");
    usb_config.serial_number = Some("12345678");

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    
    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class= CdcAcmClass::new(&mut builder, &mut state, 64);
    
    let mut usb = builder.build();
    let usb_fut = usb.run();
    
    let rgb_fut = async {
        loop {
            info!("Waiting for connection...");
            class.wait_connection().await;
            info!("Connection established... Listening.");
            let _ = rgb_listener(
                &mut class,
                &RGB_DATA
            );
            info!("Disconnected");
        }
    };
    
    let argb = ARGBLed::new_ch1(p.TIM1, p.PE9, Hertz(800_000));
    
    info!("Starting...");
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);
    for i in 0..20 {
        led.toggle();
        Timer::after_millis(200).await;
    }
    led.set_low();
    spawner.spawn(argb_task(argb, p.DMA2_CH5)).unwrap();
    join(usb_fut, rgb_fut).await;
}

struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(v: EndpointError) -> Self {
        match v {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected,
        }
    }
}

async fn rgb_listener<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
    mutex_ref: &Mutex<ThreadModeRawMutex, [(u8, u8, u8); 8]>) -> Result<(), Disconnected> {
    let mut buff = [0; 24];
    loop {
        let n = class.read_packet(&mut buff).await?;
        if n < 24 {
            continue;
        }
        let data = &buff[..n];
        let mut rgb_buff = mutex_ref.lock().await;
        for (i, tuple) in rgb_buff.iter_mut().enumerate() {
            let base = i * 3;
            if base + 2 < data.len() {
                break;
            }
            tuple.0 = data[base];
            tuple.1 = data[base + 1];
            tuple.2 = data[base + 2];
        }
    }
}

#[embassy_executor::task]
async fn argb_task(mut argb_led: ARGBLed<'static, TIM1>, dma: Peri<'static, DMA2_CH5>) {
    argb_led.run(dma).await;
}

struct ARGBLed<'a, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'a, T>,
    data: &'static Mutex<ThreadModeRawMutex, [(u8, u8, u8); 8]>,
    channel: Channel
}

impl<'a, T: GeneralInstance4Channel> ARGBLed<'a, T> {
    fn new_ch1(timer: Peri<'a, T>, pin: Peri<'a, impl TimerPin<T, Ch1>>, freq: Hertz,) -> Self
    {
        let pwm_pin = PwmPin::new(pin, OutputType::PushPull);
        let pwm_driver = SimplePwm::new(
            timer,
            Some(pwm_pin),
            None,
            None,
            None,
            freq,
            CountingMode::default(),
        );
        Self {
            pwm: pwm_driver,
            data: &RGB_DATA,
            channel: Channel::Ch1,
        }
    }

    fn new_ch2(timer: Peri<'a, T>, pin: Peri<'a, impl TimerPin<T, Ch2>>, freq: Hertz,) -> Self {
        let pwm_pin = PwmPin::new(pin, OutputType::PushPull);
        let pwm_driver = SimplePwm::new(
            timer,
            None,
            Some(pwm_pin),
            None,
            None,
            freq,
            CountingMode::default(),
        );
        Self {
            pwm: pwm_driver,
            data: &RGB_DATA,
            channel: Channel::Ch2,
        }
    }

    fn new_ch3(timer: Peri<'a, T>, pin: Peri<'a, impl TimerPin<T, Ch3>>, freq: Hertz,) -> Self {
        let pwm_pin = PwmPin::new(pin, OutputType::PushPull);
        let pwm_driver = SimplePwm::new(
            timer,
            None,
            None,
            Some(pwm_pin),
            None,
            freq,
            CountingMode::default(),
        );
        Self {
            pwm: pwm_driver,
            data: &RGB_DATA,
            channel: Channel::Ch3,
        }
    }

    fn new_ch4(timer: Peri<'a, T>, pin: Peri<'a, impl TimerPin<T, Ch4>>, freq: Hertz) -> Self {
        let pwm_pin = PwmPin::new(pin, OutputType::PushPull);
        let pwm_driver = SimplePwm::new(
            timer,
            None,
            None,
            None,
            Some(pwm_pin),
            freq,
            CountingMode::default(),
        );
        Self {
            pwm: pwm_driver,
            data: &RGB_DATA,
            channel: Channel::Ch4,
        }
    }

    async fn run(&mut self, mut dma: Peri<'a, impl UpDma<T>>) -> ! {
        loop {
            let lock = self.data.lock().await;
            let packet = self.produce_rgb_duty_packet(lock.deref());
            self.pwm.waveform_up(
                dma.reborrow(),
                self.channel,
                &packet,
            ).await;
        }
    }

    fn produce_rgb_duty_packet(&self, data: &[(u8, u8, u8)]) -> [u16; 192] {
        let high = (self.pwm.get_max_duty() as f32 * 0.7) as u16;
        let low = (self.pwm.get_max_duty() as f32 * 0.3) as u16;

        let mut res = [0u16; 192];
        let mut i = 0_usize;
        for &(r, g, b) in data {
            for byte in [g, r, b] {
                for bit in (0..8_u8).rev() {
                    let duty = if (byte >> bit) & 1 == 0 { high } else { low };
                    res[i] = duty;
                    i += 1;
                }
            }
        }
        res
    }
}

