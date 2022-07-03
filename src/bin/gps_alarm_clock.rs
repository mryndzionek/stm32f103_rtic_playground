//#![deny(warnings)]
#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [PVD, WWDG, RTC, SPI1])]
mod app {
    use bbqueue::BBBuffer;
    use bresenham::Bresenham;
    use core::fmt::{Debug, Write};
    use core::iter::Peekable;
    use core::slice::Iter;
    use defmt::{write, Format, Formatter};
    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use eeprom::{EEPROMExt, Params, EEPROM};
    use heapless::{String, Vec};
    use nmea0183::{
        coords::{Hemisphere, Latitude, Longitude},
        ParseResult, Parser, Sentence,
    };
    use oorandom::Rand32;
    use rgb::AsPixels;
    use rotary_encoder_hal::{Direction as EncDir, Rotary};
    use rtic::Monotonic;
    use smart_leds::{
        hsv::{hsv2rgb, Hsv},
        SmartLedsWrite, RGB, RGB8,
    };
    use stm32f103_rtic_playground as _;
    use stm32f1xx_hal::gpio::{
        gpioa::{PA1, PA4},
        gpiob::{PB0, PB1},
        gpioc::PC13,
        Alternate, Edge, ExtiPin, Input, Output, PullDown, PullUp, PushPull,
    };
    use stm32f1xx_hal::watchdog::IndependentWatchdog;
    use stm32f1xx_hal::{
        flash::{FlashSize, Parts, SectorSize},
        pac::{EXTI, TIM2, TIM3},
        prelude::*,
        pwm::{Channel, Pwm, C1},
        serial::{Config, Rx2, Serial},
        spi::{NoMiso, NoSck, Spi, Spi2NoRemap},
        timer::{CountDownTimer, Event, Tim3NoRemap, Timer},
    };
    use time::macros::time;
    use time::{Date, Duration, Month, PrimitiveDateTime, Time, Weekday};
    use ws2812::Ws2812;
    use ws2812_spi as ws2812;

    struct HemisphereWrapper(Hemisphere);
    impl Format for HemisphereWrapper {
        fn format(&self, fmt: Formatter) {
            let s = match &self.0 {
                Hemisphere::North => 'N',
                Hemisphere::South => 'S',
                Hemisphere::East => 'E',
                Hemisphere::West => 'W',
            };
            write!(fmt, "{}", s);
        }
    }
    struct LongitudeWrapper(Longitude);

    impl Format for LongitudeWrapper {
        fn format(&self, fmt: Formatter) {
            write!(
                fmt,
                "{}° {}' {}\" {:?}",
                self.0.degrees,
                self.0.minutes,
                self.0.seconds,
                HemisphereWrapper(self.0.hemisphere.clone()),
            );
        }
    }
    struct LatitudeWrapper(Latitude);

    impl Format for LatitudeWrapper {
        fn format(&self, fmt: Formatter) {
            write!(
                fmt,
                "{}° {}' {}\" {:?}",
                self.0.degrees,
                self.0.minutes,
                self.0.seconds,
                HemisphereWrapper(self.0.hemisphere.clone()),
            );
        }
    }

    const FREQ: u32 = 72_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    const SEG_LEN: usize = (5 * 4) + (2 * 3); // 7 segments, 5 with 4 LEDs, 2 with 3 LEDs
    const NUM_SEGS: usize = 7;
    const BUFF_SIZE: usize = 1024;
    const LED_ROWS: usize = 8;
    const LED_COLS: usize = 16;
    const NUM_LEDS: usize = SEG_LEN * 6 + (2 * 2);
    const UTC_OFFSET: Duration = Duration::HOUR;
    const SUNRISE_LEN: Duration = Duration::minutes(30);
    const ALARM_LEN: Duration = Duration::minutes(3);
    const BUZZER_BASE_FREQ_HZ: u32 = 6000;
    const LED_COLOR_PROJECTOR: RGB8 = RGB8::new(64, 2, 0);
    const LED_COLOR_DAY: RGB8 = RGB8::new(16, 2, 0);
    const LED_COLOR_NIGHT: RGB8 = RGB8::new(1, 0, 0);
    const DEFAULT_LAMP_COLOR: Hsv = Hsv {
        hue: 0,
        sat: 255,
        val: 10,
    };

    const EEPROM_PARAMS: Params = Params {
        first_page: 126,
        flash_size: FlashSize::Sz128K,
        page_size: SectorSize::Sz1K,
        page_count: 2,
    };

    const SYMBOL_TO_SEGS: [u8; 11] = [
        0b1111110, 0b1000010, 0b0110111, 0b1100111, 0b1001011, 0b1101101, 0b1111101, 0b1000110,
        0b1111111, 0b1101111, 0b1011011, // X
    ];

    const SEG_TO_OFFSET: [(usize, usize); NUM_SEGS] =
        [(0, 4), (4, 3), (7, 4), (11, 4), (15, 4), (19, 3), (22, 4)];

    const SUNRISE_CMAP: [u32; 256] = [
        0x20000, 0x70000, 0xD0000, 0x120000, 0x160000, 0x190000, 0x1C0000, 0x1F0000, 0x220000,
        0x240000, 0x260000, 0x280000, 0x2A0000, 0x2C0000, 0x2E0000, 0x300000, 0x320000, 0x330000,
        0x350000, 0x360000, 0x380000, 0x390000, 0x3B0000, 0x3C0000, 0x3E0000, 0x3F0000, 0x400000,
        0x420100, 0x430100, 0x450100, 0x460100, 0x480100, 0x490100, 0x4B0100, 0x4C0100, 0x4E0100,
        0x4F0100, 0x510100, 0x520100, 0x540100, 0x550100, 0x570100, 0x580100, 0x5A0100, 0x5B0100,
        0x5D0100, 0x5E0100, 0x600100, 0x610100, 0x630100, 0x640100, 0x660100, 0x670100, 0x690100,
        0x6B0200, 0x6C0200, 0x6E0200, 0x6F0200, 0x710200, 0x720200, 0x740200, 0x760200, 0x770200,
        0x790200, 0x7A0200, 0x7C0200, 0x7E0200, 0x7F0200, 0x810200, 0x830200, 0x840300, 0x860300,
        0x870300, 0x890300, 0x8B0300, 0x8C0300, 0x8E0300, 0x900300, 0x910300, 0x930300, 0x950300,
        0x960400, 0x980400, 0x9A0400, 0x9B0400, 0x9D0400, 0x9F0400, 0xA00400, 0xA20400, 0xA40400,
        0xA50500, 0xA70500, 0xA90500, 0xAB0500, 0xAC0500, 0xAE0500, 0xB00600, 0xB10600, 0xB30600,
        0xB50600, 0xB70600, 0xB80600, 0xBA0700, 0xBC0700, 0xBD0700, 0xBF0700, 0xC10700, 0xC30800,
        0xC40800, 0xC60800, 0xC80800, 0xCA0900, 0xCB0900, 0xCD0900, 0xCF0900, 0xD10A00, 0xD20A00,
        0xD40A00, 0xD60B00, 0xD80B00, 0xD90C00, 0xDB0C00, 0xDD0C00, 0xDF0D00, 0xE00D00, 0xE20E00,
        0xE40F00, 0xE60F00, 0xE71000, 0xE91100, 0xEB1200, 0xEC1300, 0xEE1500, 0xEF1800, 0xF01A00,
        0xF11D00, 0xF22100, 0xF32400, 0xF42700, 0xF42A00, 0xF52E00, 0xF63100, 0xF63400, 0xF73700,
        0xF73A00, 0xF73D00, 0xF83F00, 0xF84200, 0xF84500, 0xF94800, 0xF94A00, 0xF94D00, 0xF94F00,
        0xFA5200, 0xFA5400, 0xFA5600, 0xFA5900, 0xFA5B00, 0xFB5D00, 0xFB6000, 0xFB6200, 0xFB6400,
        0xFB6600, 0xFB6800, 0xFC6A00, 0xFC6C00, 0xFC6E00, 0xFC7000, 0xFC7200, 0xFC7400, 0xFC7600,
        0xFC7800, 0xFC7A00, 0xFC7C00, 0xFD7E00, 0xFD8000, 0xFD8200, 0xFD8300, 0xFD8500, 0xFD8700,
        0xFD8900, 0xFD8B00, 0xFD8C00, 0xFD8E00, 0xFD9000, 0xFD9200, 0xFD9300, 0xFE9500, 0xFE9700,
        0xFE9900, 0xFE9A00, 0xFE9C00, 0xFE9E00, 0xFE9F00, 0xFEA100, 0xFEA300, 0xFEA400, 0xFEA600,
        0xFEA800, 0xFEA900, 0xFEAB00, 0xFEAD00, 0xFEAE00, 0xFEB000, 0xFEB100, 0xFEB300, 0xFEB500,
        0xFEB600, 0xFEB800, 0xFEB900, 0xFEBB00, 0xFEBD00, 0xFFBE00, 0xFFC000, 0xFFC100, 0xFFC300,
        0xFFC400, 0xFFC600, 0xFFC700, 0xFFC900, 0xFFCB00, 0xFFCC00, 0xFFCE00, 0xFFCF00, 0xFFD100,
        0xFFD200, 0xFFD400, 0xFFD500, 0xFFD700, 0xFFD800, 0xFFDA00, 0xFFDB00, 0xFFDD00, 0xFFDE00,
        0xFFE000, 0xFFE100, 0xFFE300, 0xFFE400, 0xFFE600, 0xFFE700, 0xFFE900, 0xFFEA00, 0xFFEC00,
        0xFFED00, 0xFFEF00, 0xFFF000, 0xFFF200, 0xFFF300, 0xFFF500, 0xFFF600, 0xFFF800, 0xFFF900,
        0xFFFB00, 0xFFFC00, 0xFFFE00, 0xFFFF00,
    ];

    const GAMMA8: [u8; 256] = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
        4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12,
        13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24,
        24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
        64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93,
        95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124,
        126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158,
        160, 162, 164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
        200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244,
        247, 249, 252, 255,
    ];

    static BB: BBBuffer<BUFF_SIZE> = BBBuffer::new();

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum LightEvent {
        INIT,
        SOLID_COLOR { color: RGB8 },
        TICK { h: u8, m: u8, s: u8 },
        TICK_MODE { mode: LightTickMode },
        DIGITS { dg: [u8; 6], col1: bool, col2: bool },
        UPDATE,
        ALARM_CHANGE { set: bool },
    }

    #[derive(Debug, PartialEq, Clone)]
    #[allow(non_camel_case_types)]
    pub enum LightTickMode {
        DAY,
        NIGHT,
        PROJECTOR,
    }

    #[shared]
    struct Shared {
        datetime: Option<PrimitiveDateTime>,
        gps_locked: bool,
        pps_timeout: Option<pps_timeout::SpawnHandle>,
        eeprom: EEPROM<&'static mut stm32f1xx_hal::flash::Parts>,
        btn_pin: PA4<Input<PullUp>>,
        exti: EXTI,
        leds: [RGB8; NUM_LEDS],
    }

    type Leds = ws2812_spi::Ws2812<
        Spi<
            stm32f1xx_hal::pac::SPI2,
            Spi2NoRemap,
            (
                NoSck,
                NoMiso,
                stm32f1xx_hal::gpio::gpiob::PB15<stm32f1xx_hal::gpio::Alternate<PushPull>>,
            ),
            u8,
        >,
    >;

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        pps: PA1<Input<PullDown>>,
        rx: Rx2,
        parser: Parser,
        ws: Leds,
        prod: bbqueue::Producer<'static, BUFF_SIZE>,
        cons: bbqueue::Consumer<'static, BUFF_SIZE>,
        dst: Duration,
        buzz_pwm: Pwm<TIM3, Tim3NoRemap, C1, stm32f1xx_hal::gpio::gpioa::PA6<Alternate<PushPull>>>,
        wdg: IndependentWatchdog,
        enc_timer: CountDownTimer<TIM2>,
        encoder: Rotary<PB0<Input<PullUp>>, PB1<Input<PullUp>>>,
        rng: Rand32,
        alarm_enabled: bool,
        alarm_time: Time,
    }

    #[init(local = [flash: Option<Parts> = None])]
    fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Starting !!!");

        // workaround, see: https://github.com/knurling-rs/defmt/issues/322
        #[cfg(debug_assertions)]
        c.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        #[cfg(debug_assertions)]
        c.device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        let mut flash = c.device.FLASH.constrain();
        let rcc = c.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(FREQ.hz())
            .pclk1(36.mhz())
            .freeze(&mut flash.acr);
        let mut afio = c.device.AFIO.constrain();

        let flash = c.local.flash.insert(flash);

        let mut gpioc = c.device.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        // Setup USART
        let mut gpioa = c.device.GPIOA.split();
        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;
        let (_, mut rx) = Serial::usart2(
            c.device.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(38400.bps()),
            clocks,
        )
        .split();
        rx.listen();
        rx.listen_idle();

        let mut pps = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);
        pps.make_interrupt_source(&mut afio);
        pps.enable_interrupt(&mut c.device.EXTI);
        pps.trigger_on_edge(&mut c.device.EXTI, Edge::Rising);

        let mut btn = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);
        btn.make_interrupt_source(&mut afio);
        btn.enable_interrupt(&mut c.device.EXTI);
        btn.trigger_on_edge(&mut c.device.EXTI, Edge::Falling);

        let mut gpiob = c.device.GPIOB.split();
        let pins = (
            NoSck,
            NoMiso,
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );
        let spi = Spi::spi2(c.device.SPI2, pins, ws2812::MODE, 3500.khz(), clocks);
        let leds = [RGB8::default(); NUM_LEDS];

        let ws = Ws2812::new(spi);

        let mut dcb = c.core.DCB;
        let dwt = c.core.DWT;
        let systick = c.core.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        let parser = Parser::new().sentence_filter(Sentence::RMC | Sentence::RMC);
        let (prod, cons) = BB.try_split().unwrap();

        let mut eeprom = flash.eeprom(EEPROM_PARAMS);
        eeprom.init().unwrap();
        let dst = if let Some(v) = eeprom.read(0) {
            match v {
                0 => Duration::ZERO,
                1 => Duration::HOUR,
                _ => {
                    eeprom.write(0, 1).unwrap();
                    Duration::HOUR
                }
            }
        } else {
            eeprom.write(0, 1).unwrap();
            Duration::HOUR
        };
        defmt::info!("DST: {}", dst.whole_hours());

        let alarm_enabled = if let Some(v) = eeprom.read(1) {
            v != 0
        } else {
            eeprom.write(1, 0).unwrap();
            false
        };

        let hm = if let Some(v) = eeprom.read(2) {
            v
        } else {
            eeprom.write(2, 0).unwrap();
            0
        };

        let alarm_time = Time::from_hms(
            ((hm >> 8) % 24).try_into().unwrap(),
            ((hm & 0xFF) % 60).try_into().unwrap(),
            0,
        )
        .unwrap();

        let buzz_pin = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
        let mut buzz_pwm = Timer::new(c.device.TIM3, &clocks).pwm::<Tim3NoRemap, _, _, _>(
            buzz_pin,
            &mut afio.mapr,
            BUZZER_BASE_FREQ_HZ.hz(),
        );
        buzz_pwm.set_duty(Channel::C1, 2 * (buzz_pwm.get_max_duty() / 100));

        let enc_a = gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        let enc_b = gpiob.pb1.into_pull_up_input(&mut gpiob.crl);
        let encoder = Rotary::new(enc_a, enc_b);

        let mut enc_timer = Timer::tim2(c.device.TIM2, &clocks).start_count_down(1000.hz());
        enc_timer.listen(Event::Update);

        let wdg = IndependentWatchdog::new(c.device.IWDG);

        light::spawn(LightEvent::INIT).unwrap();
        light::spawn(LightEvent::DIGITS {
            dg: [10, 10, 10, 10, 10, 10],
            col1: true,
            col2: true,
        })
        .unwrap();
        light::spawn(LightEvent::ALARM_CHANGE { set: alarm_enabled }).unwrap();

        (
            Shared {
                datetime: None,
                gps_locked: false,
                pps_timeout: None,
                eeprom,
                btn_pin: btn,
                exti: c.device.EXTI,
                leds,
            },
            Local {
                led,
                pps,
                rx,
                parser,
                ws,
                prod,
                cons,
                dst,
                buzz_pwm,
                wdg,
                enc_timer,
                encoder,
                rng: Rand32::new(666),
                alarm_enabled,
                alarm_time,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = USART2, shared = [], local = [rx, prod], priority = 16)]
    fn usart2(cx: usart2::Context) {
        let rx = cx.local.rx;
        let prod = cx.local.prod;
        if rx.is_rx_not_empty() {
            if let Ok(b) = nb::block!(rx.read()) {
                let mut wgr = prod.grant_exact(1).unwrap();
                wgr[0] = b;
                wgr.commit(1);
                if b == '\n' as u8 {
                    parse::spawn().ok();
                }
            } else {
                defmt::error!("usart error!!!");
            }
            rx.listen_idle();
        } else if rx.is_idle() {
            rx.unlisten_idle();
        }
    }

    fn dst_adjust(dt: PrimitiveDateTime) -> Duration {
        let tm = dt.time();
        let dt = dt.date();
        static mut SM: bool = false;

        if (dt.month() == Month::October)
            && (dt.day() + 7 > 31)
            && (dt.weekday() == Weekday::Sunday)
            && (tm == time!(3:00:00))
        {
            unsafe {
                if !SM {
                    SM = true;
                    return -Duration::HOUR;
                } else {
                    SM = false;
                    return Duration::ZERO;
                }
            }
        } else if (dt.month() == Month::March)
            && (dt.day() + 7 > 31)
            && (dt.weekday() == Weekday::Sunday)
            && (tm == time!(2:00:00))
        {
            return Duration::HOUR;
        } else {
            return Duration::ZERO;
        }
    }

    #[task(shared = [datetime, pps_timeout, gps_locked], local = [cons, parser], priority = 2, capacity = 1)]
    fn parse(cx: parse::Context) {
        let cons = cx.local.cons;
        let parser = cx.local.parser;

        let parse::SharedResources {
            mut datetime,
            pps_timeout,
            gps_locked,
        } = cx.shared;

        let mut res1 = (gps_locked, pps_timeout);

        if let Some(rgr) = cons.read().ok() {
            defmt::trace!("{}", &rgr.buf());

            for result in parser.parse_from_bytes(&rgr) {
                match result {
                    Ok(ParseResult::RMC(Some(rmc))) => {
                        res1.lock(|gps_locked, pps_timeout| {
                            if *gps_locked {
                                if let Some(handle) = pps_timeout.take() {
                                    handle.cancel().ok();
                                }
                            } else {
                                *gps_locked = true;
                                defmt::info!("GPS lock acquired");
                            }
                        });

                        let nmea0183::datetime::Date { day, month, year } = rmc.datetime.date;
                        let nmea0183::datetime::Time {
                            hours,
                            minutes,
                            seconds,
                        } = rmc.datetime.time;
                        defmt::debug!(
                            "rmc {}:{}:{} {}-{}-{}",
                            hours,
                            minutes,
                            seconds,
                            day,
                            month,
                            year
                        );
                        let date = Date::from_calendar_date(
                            year.try_into().unwrap(),
                            month.try_into().unwrap(),
                            day.try_into().unwrap(),
                        )
                        .unwrap();
                        let time = Time::from_hms(hours, minutes, seconds as u8).unwrap();
                        let date_time = PrimitiveDateTime::new(date, time);
                        datetime.lock(|dt| match *dt {
                            Some(dt) => {
                                defmt::assert!(dt == date_time);
                            }
                            None => {
                                *dt = Some(date_time);
                            }
                        })
                    }
                    Ok(ParseResult::GGA(Some(gga))) => {
                        defmt::debug!(
                            "Lon: {:?} Lat: {:?}",
                            LongitudeWrapper(gga.longitude.clone()),
                            LatitudeWrapper(gga.latitude.clone())
                        );
                        defmt::debug!(
                            "sats in use {}, quality {}",
                            gga.sat_in_use,
                            gga.gps_quality as u8
                        );
                    }
                    Ok(_) => {
                        defmt::debug!("Incomplete NMEA")
                    }
                    Err(e) => {
                        defmt::debug!("NMEA parsing error: {}", e)
                    }
                }
            }

            let len = rgr.len();
            rgr.release(len);
        }
    }

    #[task(binds = EXTI1, shared = [datetime, gps_locked, pps_timeout, eeprom],
                           local = [pps, dst], priority = 16)]
    fn pps_isr(cx: pps_isr::Context) {
        let pps_isr::SharedResources {
            datetime,
            gps_locked,
            pps_timeout,
            eeprom,
        } = cx.shared;

        let pps = cx.local.pps;

        if pps.check_interrupt() {
            pps.clear_interrupt_pending_bit();

            defmt::debug!("pps");
            let (dt, gps) = (datetime, gps_locked, pps_timeout, eeprom).lock(
                |dt, gps_locked, pps_timeout, eeprom| {
                    if let Some(mut dt_) = *dt {
                        *dt = Some(dt_ + Duration::SECOND);
                        dt_ += Duration::SECOND;
                        // TODO local.offset needs to be saved into persistent memory
                        let dst_change = dst_adjust(dt_ + UTC_OFFSET + *cx.local.dst);
                        if dst_change != Duration::ZERO {
                            *cx.local.dst += dst_change;
                            eeprom
                                .write(0, cx.local.dst.whole_hours().try_into().unwrap())
                                .unwrap();
                        }
                        dt_ += UTC_OFFSET + *cx.local.dst;
                        if *gps_locked {
                            *pps_timeout = pps_timeout::spawn_after(900.millis()).ok();
                        }
                        (Some(dt_), *gps_locked)
                    } else {
                        (None, *gps_locked)
                    }
                },
            );
            main_sm::spawn(MainEvent::PPSTick { dt, gps_lock: gps }).unwrap();
        }
    }

    #[task(shared = [gps_locked], local = [], priority = 2)]
    fn pps_timeout(cx: pps_timeout::Context) {
        let mut gps_locked = cx.shared.gps_locked;
        defmt::warn!("GPS lock lost");
        gps_locked.lock(|gps_locked| *gps_locked = false)
    }

    fn leds_write(ws: &mut Leds, leds: &[RGB<u8>; NUM_LEDS]) {
        for retries in 0..10 {
            let ret = ws.write(leds.iter().cloned());
            match ret {
                Ok(_) => break,
                Err(_) => {
                    if retries == 9 {
                        defmt::panic!();
                    } else {
                        defmt::warn!("Writing to LEDs failed")
                    }
                }
            }
        }
    }

    #[task(shared = [leds], local = [ws, is_alarm: bool = false, mode: LightTickMode = LightTickMode::DAY],
                        priority = 16, capacity = 3)]
    fn light(cx: light::Context, ev: LightEvent) {
        let ws = cx.local.ws;
        let mut leds = cx.shared.leds;
        let curr_mode = cx.local.mode;

        let color = match *curr_mode {
            LightTickMode::NIGHT => LED_COLOR_NIGHT,
            LightTickMode::DAY => LED_COLOR_DAY,
            LightTickMode::PROJECTOR => LED_COLOR_PROJECTOR,
        };

        match ev {
            LightEvent::INIT => leds.lock(|leds| {
                ws.write(leds.iter().cloned()).unwrap();
            }),
            LightEvent::TICK { h, m, s } => {
                let mut buf: String<6> = String::new();
                buf.write_fmt(format_args!("{:02}{:02}{:02}", h, m, s))
                    .unwrap();

                leds.lock(|leds| {
                    *leds = leds.map(|_| RGB8::default());

                    for (x, mut d) in buf.bytes().enumerate() {
                        d -= '0' as u8;
                        leds_draw_digit(leds, x, d, color);
                    }
                    if *cx.local.is_alarm {
                        leds[leds_xy_to_n(0, 1, false)] = RGB8::new(0, 0, color.r);
                    }
                    if s % 2 == 1 {
                        leds_draw_colon(leds, 0, color);
                        leds_draw_colon(leds, 1, color);
                    }
                    leds_write(ws, leds);
                });
            }
            LightEvent::TICK_MODE { mode } => {
                *curr_mode = mode;
            }
            LightEvent::SOLID_COLOR { color } => {
                leds.lock(|leds| {
                    *leds = leds.map(|_| color);
                    leds_write(ws, leds);
                });
            }
            LightEvent::UPDATE => leds.lock(|leds| {
                leds_write(ws, leds);
            }),
            LightEvent::ALARM_CHANGE { set } => {
                *cx.local.is_alarm = set;
                leds.lock(|leds| {
                    let mirror = *curr_mode == LightTickMode::PROJECTOR;
                    if *cx.local.is_alarm {
                        leds[leds_xy_to_n(0, 1, mirror)] = RGB8::new(0, 0, color.r);
                    } else {
                        leds[leds_xy_to_n(0, 1, mirror)] = RGB8::new(0, 0, 0);
                    }
                    leds_write(ws, leds);
                });
            }
            LightEvent::DIGITS { dg, col1, col2 } => {
                leds.lock(|leds| {
                    *leds = leds.map(|_| RGB8::default());

                    for (x, d) in dg.iter().enumerate() {
                        leds_draw_digit(leds, x, *d % (SYMBOL_TO_SEGS.len() as u8), color);
                    }

                    if col1 {
                        leds_draw_colon(leds, 0, color);
                    }
                    if col2 {
                        leds_draw_colon(leds, 1, color);
                    }
                    leds_write(ws, leds);
                });
            }
        }
    }

    pub enum AmSensorState {
        START1,
        START2,
        ACK1,
        ACK2,
        RX,
    }
    pub enum BuzzerState {
        Chime {
            is_on: bool,
            chime: Option<Iter<'static, u32>>,
        },
        Alarm {
            is_pause: bool,
            dur: u16,
            alarm: Iter<'static, (u16, u16)>,
            iter: Iter<'static, (u16, u16)>,
        },
    }

    #[derive(Debug, PartialEq, Format)]
    pub enum BuzzerEvent {
        HourChime,
        QuarterChime,
        SecondTick,
        Next,
        Alarm { i: Option<usize> },
    }

    const ALARMS: &'static [&'static [(u16, u16)]] = &[
        &[
            (0, 42),
            (1175, 167),
            (784, 167),
            (784, 167),
            (1175, 167),
            (784, 167),
            (784, 167),
            (1175, 167),
            (784, 167),
            (1244, 167),
            (784, 167),
            (1175, 167),
            (784, 167),
            (784, 167),
            (1175, 167),
            (784, 167),
            (784, 167),
            (1175, 167),
            (784, 167),
            (1244, 167),
            (784, 167),
            (1109, 167),
            (740, 167),
            (740, 167),
            (1109, 167),
            (740, 167),
            (740, 167),
            (1109, 167),
            (740, 167),
            (1175, 167),
            (740, 167),
            (1109, 167),
            (740, 167),
            (740, 167),
            (1109, 167),
            (740, 167),
            (740, 167),
            (1109, 167),
            (740, 167),
            (1175, 167),
            (740, 167),
        ],
        &[
            (659, 333),
            (659, 333),
            (587, 167),
            (659, 333),
            (784, 333),
            (659, 1000),
            (0, 333),
            (1318, 500),
            (1976, 500),
            (1760, 500),
            (1568, 500),
            (1397, 333),
            (659, 333),
            (659, 333),
            (587, 167),
            (659, 333),
            (784, 333),
            (659, 1000),
            (0, 333),
            (1318, 500),
            (1976, 500),
            (1760, 500),
            (1568, 500),
            (1397, 333),
            (698, 333),
            (698, 333),
            (698, 167),
            (698, 333),
            (698, 333),
            (698, 1000),
            (698, 333),
            (698, 167),
            (698, 333),
            (698, 333),
            (698, 333),
            (698, 333),
            (698, 333),
            (784, 333),
            (698, 500),
            (659, 667),
            (659, 333),
            (659, 333),
            (587, 167),
            (659, 333),
            (784, 333),
            (659, 1000),
            (0, 333),
            (1318, 500),
            (1976, 500),
            (1760, 500),
            (1568, 500),
            (1397, 333),
            (659, 333),
            (659, 333),
            (587, 167),
            (659, 333),
            (784, 333),
            (659, 1000),
            (0, 333),
            (1318, 500),
            (1976, 500),
            (1760, 500),
            (1568, 500),
            (1397, 333),
            (698, 333),
            (698, 333),
            (698, 167),
            (698, 333),
            (698, 333),
            (698, 1000),
            (698, 333),
            (698, 167),
            (698, 333),
            (698, 333),
            (698, 333),
            (698, 333),
            (698, 333),
            (784, 333),
            (698, 500),
            (659, 667),
        ],
        &[
            (659, 480),
            (988, 480),
            (880, 480),
            (988, 480),
            (1175, 480),
            (988, 1440),
            (0, 1920),
            (659, 480),
            (988, 480),
            (880, 480),
            (988, 480),
            (1318, 480),
            (988, 1440),
            (1568, 480),
            (1397, 480),
            (1318, 480),
            (1175, 480),
            (1397, 480),
            (988, 1440),
            (0, 1920),
            (659, 480),
            (988, 480),
            (880, 480),
            (988, 480),
            (1175, 480),
            (988, 1440),
            (0, 1920),
            (1318, 480),
            (988, 1440),
        ],
        &[
            (0, 47),
            (1568, 188),
            (1568, 188),
            (1568, 188),
            (1568, 375),
            (1318, 375),
            (1568, 188),
            (1760, 188),
            (1568, 188),
            (1397, 188),
            (1568, 375),
            (1046, 375),
            (1568, 188),
            (1760, 188),
            (1568, 188),
            (1397, 188),
            (1568, 375),
            (1318, 375),
            (1046, 188),
            (1175, 1125),
            (0, 375),
            (1318, 562),
            (880, 375),
            (1046, 375),
            (1318, 188),
            (1480, 188),
            (1318, 188),
            (1175, 188),
            (1318, 375),
            (880, 375),
            (1318, 188),
            (1480, 188),
            (1318, 188),
            (1175, 188),
            (1318, 375),
            (880, 375),
            (784, 188),
            (880, 750),
        ],
        &[
            (880, 120),
            (659, 120),
            (0, 120),
            (659, 120),
            (0, 120),
            (659, 120),
            (784, 120),
            (0, 120),
            (698, 120),
            (698, 120),
            (698, 120),
            (698, 120),
            (698, 120),
            (740, 120),
            (0, 120),
            (740, 120),
            (988, 120),
            (659, 120),
            (0, 120),
            (659, 120),
            (0, 120),
            (659, 120),
            (880, 120),
            (0, 120),
            (784, 120),
            (784, 120),
            (784, 120),
            (784, 120),
            (784, 120),
            (831, 120),
        ],
        &[
            (1480, 375),
            (1480, 375),
            (1318, 188),
            (1318, 375),
            (1175, 375),
            (1175, 375),
            (988, 562),
            (740, 375),
            (988, 188),
            (988, 188),
            (988, 375),
            (988, 188),
            (1175, 375),
            (1175, 375),
            (1318, 375),
            (1175, 562),
            (988, 375),
            (1480, 375),
            (1480, 375),
            (1318, 188),
            (1318, 375),
            (1175, 375),
            (1175, 375),
            (988, 562),
            (740, 375),
            (988, 375),
            (988, 375),
            (988, 188),
            (1175, 375),
            (1175, 375),
            (1318, 375),
            (1175, 562),
            (988, 375),
            (1480, 375),
            (1480, 375),
            (1318, 188),
            (1318, 375),
            (1175, 375),
            (1175, 375),
            (988, 562),
            (740, 375),
            (988, 188),
            (988, 188),
            (988, 375),
            (988, 188),
            (1175, 375),
            (1175, 375),
            (1318, 375),
            (1175, 562),
            (988, 375),
            (1480, 375),
            (1480, 375),
            (1318, 188),
            (1318, 375),
            (1175, 375),
            (1175, 375),
            (988, 562),
            (740, 375),
            (988, 375),
            (988, 375),
            (988, 188),
            (1175, 375),
            (1175, 375),
            (1318, 375),
            (1175, 562),
            (988, 375),
        ],
    ];

    #[task(local = [buzz_pwm,
                    state: BuzzerState = BuzzerState::Chime {is_on: false, chime: None},
                    handle: Option<buzzer::SpawnHandle> = None],
           shared = [],
           priority = 4,
           capacity = 2)]
    fn buzzer(cx: buzzer::Context, ev: BuzzerEvent) {
        const HOUR_CHIME: [u32; 3] = [100, 80, 50];
        const QUARTER_CHIME: [u32; 1] = [50];
        const SECOND_TICK: [u32; 1] = [1];

        let buzzer = cx.local.buzz_pwm;
        let s = cx.local.state;
        let handle = cx.local.handle;

        match s {
            BuzzerState::Chime { is_on, chime } => match ev {
                BuzzerEvent::Alarm { i } => {
                    if let Some(i) = i {
                        if let Some(handle) = handle.take() {
                            handle.cancel().ok();
                        }
                        *s = BuzzerState::Alarm {
                            is_pause: false,
                            dur: 0,
                            alarm: ALARMS[i].iter(),
                            iter: ALARMS[i].iter(),
                        };
                        buzzer::spawn(BuzzerEvent::Next).unwrap();
                    }
                }
                _ => {
                    match ev {
                        BuzzerEvent::HourChime => *chime = Some(HOUR_CHIME.iter()),
                        BuzzerEvent::QuarterChime => *chime = Some(QUARTER_CHIME.iter()),
                        BuzzerEvent::SecondTick => *chime = Some(SECOND_TICK.iter()),
                        BuzzerEvent::Next => (),
                        BuzzerEvent::Alarm { .. } => {
                            defmt::panic!();
                        }
                    }

                    if let Some(i) = chime {
                        match i.next() {
                            None => {
                                buzzer.disable(Channel::C1);
                                *is_on = false;
                                *chime = None;
                            }
                            Some(delay) => {
                                if *is_on {
                                    buzzer.disable(Channel::C1);
                                    *is_on = false;
                                } else {
                                    buzzer.set_period(BUZZER_BASE_FREQ_HZ.hz());
                                    buzzer.enable(Channel::C1);
                                    *is_on = true;
                                }
                                *handle =
                                    buzzer::spawn_after(delay.millis(), BuzzerEvent::Next).ok();
                            }
                        }
                    }
                }
            },
            BuzzerState::Alarm {
                is_pause,
                dur,
                alarm,
                iter,
            } => match ev {
                BuzzerEvent::Next => {
                    if *is_pause {
                        *is_pause = false;
                        buzzer.disable(Channel::C1);
                        *handle =
                            buzzer::spawn_after(((*dur / 10) as u32).millis(), BuzzerEvent::Next)
                                .ok();
                    } else {
                        match alarm.next() {
                            Some((freq, dur_)) => {
                                defmt::debug!(
                                    "Playing: frequency = {}Hz, duration = {} ms",
                                    freq,
                                    dur
                                );
                                if *freq > 0 {
                                    buzzer.set_period((*freq as u32).hz());
                                    buzzer.enable(Channel::C1);
                                }
                                *is_pause = true;
                                *dur = *dur_;
                                *handle = buzzer::spawn_after(
                                    ((*dur_ - (*dur_ / 10)) as u32).millis(),
                                    BuzzerEvent::Next,
                                )
                                .ok();
                            }
                            None => {
                                *alarm = iter.clone();
                                *handle = buzzer::spawn_after(1.secs(), BuzzerEvent::Next).ok();
                            }
                        }
                    }
                }
                BuzzerEvent::Alarm { i } => {
                    if let Some(i) = i {
                        *s = BuzzerState::Alarm {
                            is_pause: false,
                            dur: 0,
                            alarm: ALARMS[i].iter(),
                            iter: ALARMS[i].iter(),
                        };
                        if let Some(handle) = handle.take() {
                            handle.cancel().ok();
                        }
                        buzzer::spawn(BuzzerEvent::Next).unwrap();
                    } else {
                        buzzer.disable(Channel::C1);
                        if let Some(handle) = handle.take() {
                            handle.cancel().ok();
                        }
                        *s = BuzzerState::Chime {
                            is_on: false,
                            chime: None,
                        };
                    }
                }
                _ => {}
            },
        }
    }

    fn leds_xy_to_n(x: usize, y: usize, m: bool) -> usize {
        let _y = 7 - y;
        let _x = if m { x } else { 15 - x };
        let n = if _x > 7 { NUM_LEDS / 2 } else { 0 };
        return n + (_y * 8) + (_x % 8);
    }

    fn leds_draw_digit(leds: &mut [RGB8; NUM_LEDS], x: usize, d: u8, color: RGB8) {
        let dx = x * SEG_LEN + (2 * (x / 2));
        let segs = SYMBOL_TO_SEGS[d as usize];
        for i in 0..NUM_SEGS {
            if segs & (1 << i) != 0 {
                let (o, l) = SEG_TO_OFFSET[i];
                for i in dx + o..dx + o + l {
                    leds[i] = color;
                }
            }
        }
    }

    fn leds_draw_colon(leds: &mut [RGB8; NUM_LEDS], x: usize, color: RGB8) {
        let offset = 2 * SEG_LEN + ((2 * SEG_LEN) + 2) * x;
        leds[offset] = color;
        leds[offset + 1] = color;
    }

    #[derive(Debug, PartialEq, Format)]
    pub enum ButtonEvent {
        Depressed,
        Pressed,
        Timeout,
    }

    pub enum ButtonState {
        Idle,
        ShortPressWait,
    }

    #[task(binds = EXTI4, shared = [btn_pin, exti],
        local = [], priority = 2)]
    fn button_isr(cx: button_isr::Context) {
        let btn = cx.shared.btn_pin;
        let exti = cx.shared.exti;

        (btn, exti).lock(|btn, exti| {
            btn.disable_interrupt(exti);
            btn.clear_interrupt_pending_bit();
            btn_debouncer::spawn_after(20.millis()).unwrap();
        });
    }

    #[task(local = [is_pressed: bool = false],
        shared = [btn_pin, exti],
        priority = 2)]
    fn btn_debouncer(cx: btn_debouncer::Context) {
        let btn = cx.shared.btn_pin;
        let exti = cx.shared.exti;
        let is_pressed = cx.local.is_pressed;

        (btn, exti).lock(|btn, exti| {
            if *is_pressed {
                if btn.is_high() {
                    btn.trigger_on_edge(exti, Edge::Falling);
                    *is_pressed = false;
                    btn_detector::spawn(ButtonEvent::Depressed).unwrap();
                }
            } else {
                if btn.is_low() {
                    btn.trigger_on_edge(exti, Edge::Rising);
                    *is_pressed = true;
                    btn_detector::spawn(ButtonEvent::Pressed).unwrap();
                }
            }
            btn.enable_interrupt(exti);
        });
    }

    #[task(local = [state: ButtonState = ButtonState::Idle,
                    handle: Option<btn_detector::SpawnHandle> = None],
        capacity = 2,
        priority = 2)]
    fn btn_detector(cx: btn_detector::Context, ev: ButtonEvent) {
        let state = cx.local.state;
        let handle = cx.local.handle;

        match state {
            ButtonState::Idle => match ev {
                ButtonEvent::Pressed => {
                    *handle = btn_detector::spawn_after(750.millis(), ButtonEvent::Timeout).ok();
                    *state = ButtonState::ShortPressWait;
                }
                ButtonEvent::Depressed => {}
                _ => defmt::panic!(),
            },
            ButtonState::ShortPressWait => match ev {
                ButtonEvent::Depressed => {
                    // we have a short press
                    if let Some(handle) = handle.take() {
                        handle.cancel().ok();
                    }
                    main_sm::spawn(MainEvent::ShortButtonPress).unwrap();
                    *state = ButtonState::Idle;
                }
                ButtonEvent::Timeout => {
                    // we have a long press
                    main_sm::spawn(MainEvent::LongButtonPress).unwrap();
                    *state = ButtonState::Idle;
                }
                _ => {
                    defmt::panic!();
                }
            },
        }
    }

    pub enum MainState {
        TimeDisplay,
        Sunrise {
            ramp: Peekable<Bresenham>,
            prev_val: Option<isize>,
        },
        Alarm,
        AlarmSetting,
        Lamp {
            hsv: Hsv,
        },
        Fire,
        Coals {
            is_rgb: bool,
        },
    }

    #[derive(Debug)]
    pub enum MainEvent {
        Timeout,
        Timeout2,
        PPSTick {
            dt: Option<PrimitiveDateTime>,
            gps_lock: bool,
        },
        ShortButtonPress,
        LongButtonPress,
        Encoder {
            dir: EncDir,
        },
    }

    fn pps_tick_handler(dt: Option<PrimitiveDateTime>, _gps_lock: bool) {
        if let Some(dt) = dt {
            let (h, s) = (dt.time().hour(), dt.time().second());
            if h < 22 && h > 7 {
                match dt.time().minute() {
                    0 => {
                        if s == 0 {
                            buzzer::spawn(BuzzerEvent::HourChime).unwrap();
                        }
                    }
                    59 => {
                        if s > 56 {
                            buzzer::spawn(BuzzerEvent::QuarterChime).unwrap();
                        }
                    }
                    15 | 30 | 45 => {
                        if s == 0 {
                            buzzer::spawn(BuzzerEvent::QuarterChime).unwrap()
                        }
                    }
                    _ => (),
                }
            }
        }
    }

    const FIRE_PALETTE: [u32; 11] = [
        0x000000, 0x300000, 0x600000, 0x800400, 0xA01000, 0xB02000, 0xC02000, 0xC04000, 0xC06000,
        0xC08000, 0x807080,
    ];

    fn fire_update(pixels: &mut [[u16; LED_COLS + 2]; LED_ROWS + 2], rng: &mut Rand32) {
        for i in (1..LED_ROWS + 1).rev() {
            for j in 1..LED_COLS + 1 {
                let mut n = 0;

                if pixels[i - 1][j] > 0 {
                    n = pixels[i - 1][j] - 1;
                }
                pixels[i][j] = n;
            }
        }

        for j in 1..LED_COLS + 1 {
            let range = ((FIRE_PALETTE.len() - 6) as u32)..((FIRE_PALETTE.len() - 2) as u32);
            pixels[0][j] = rng.rand_range(range) as u16;
        }
    }

    fn diff_skewed(a: u16, b: u16) -> u16 {
        if a > b {
            return a - b;
        } else {
            return 0;
        };
    }

    fn coals_transfer(pixels: &[[u16; LED_COLS + 2]; LED_ROWS + 2], x: u8, y: u8) -> u16 {
        let mut v: u32 = 0;

        for i in [-1, 0, 1] {
            for j in [-1, 0, 1] {
                let n = (y as i16 + i) as usize;
                let m = (x as i16 + j) as usize;
                v += diff_skewed(pixels[m][n], pixels[x as usize][y as usize]) as u32;
            }
        }

        return if v > 0xFFFF { 0xFFFF } else { v as u16 };
    }

    fn coals_update(pixels: &mut [[u16; LED_COLS + 2]; LED_ROWS + 2]) {
        for x in 1..LED_ROWS + 1 {
            for y in 1..LED_COLS + 1 {
                pixels[x][y] =
                    diff_skewed(pixels[x][y], 32) + (coals_transfer(&pixels, x as u8, y as u8) / 8);
            }
        }
    }

    fn disp_time(time: Time) {
        let (h, m, s) = time.as_hms();
        light::spawn(LightEvent::TICK { h, m, s }).unwrap();
    }

    #[task(shared = [leds, eeprom], local = [
                                 curr_al: usize = 0,
                                 state: MainState = MainState::TimeDisplay,
                                 handle: Option<main_sm::SpawnHandle> = None,
                                 handle2: Option<main_sm::SpawnHandle> = None,
                                 rng,
                                 bmp: [[u16; LED_COLS + 2]; LED_ROWS + 2] = [[0; LED_COLS + 2]; LED_ROWS + 2],
                                 mode: LightTickMode = LightTickMode::DAY,
                                 alarm_enabled,
                                 alarm_time,
                                 alarm_dur: Duration = Duration::ZERO,
                                 dt: Option<PrimitiveDateTime> = None,
                                 ],
                        priority = 4, capacity = 6)]
    fn main_sm(cx: main_sm::Context, ev: MainEvent) {
        let state = cx.local.state;
        let curr_mode = cx.local.mode;
        let alarm_time = cx.local.alarm_time;
        let datetime = cx.local.dt;

        match state {
            MainState::TimeDisplay => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    pps_tick_handler(dt, gps_lock);
                    *datetime = dt;
                    if let Some(dt) = *datetime {
                        if *cx.local.alarm_enabled {
                            if ![Weekday::Saturday, Weekday::Sunday].contains(&dt.date().weekday())
                            {
                                if dt.time() == (*alarm_time - SUNRISE_LEN) {
                                    defmt::info!("Starting sunrise");

                                    *state = MainState::Sunrise {
                                        ramp: Bresenham::new(
                                            (0, 0),
                                            (
                                                SUNRISE_LEN.whole_seconds().try_into().unwrap(),
                                                SUNRISE_CMAP.len().try_into().unwrap(),
                                            ),
                                        )
                                        .peekable(),
                                        prev_val: None,
                                    };

                                    main_sm::spawn(ev).unwrap();
                                }
                            }
                        }
                        let h = dt.time().hour();

                        match *curr_mode {
                            LightTickMode::DAY => {
                                if h > 20 || h < 7 {
                                    *curr_mode = LightTickMode::NIGHT;
                                    light::spawn(LightEvent::TICK_MODE {
                                        mode: (*curr_mode).clone(),
                                    })
                                    .unwrap();
                                }
                            }
                            LightTickMode::NIGHT => {
                                if h <= 20 && h >= 7 {
                                    *curr_mode = LightTickMode::DAY;
                                    light::spawn(LightEvent::TICK_MODE {
                                        mode: (*curr_mode).clone(),
                                    })
                                    .unwrap();
                                }
                            }
                            LightTickMode::PROJECTOR => {}
                        }
                        disp_time(dt.time());
                    }
                }
                MainEvent::LongButtonPress => {
                    disp_time(*alarm_time);
                    buzzer::spawn(BuzzerEvent::Alarm {
                        i: Some(*cx.local.curr_al),
                    })
                    .unwrap();
                    *state = MainState::AlarmSetting;
                }
                MainEvent::ShortButtonPress => {
                    *cx.local.alarm_enabled ^= true;
                    light::spawn(LightEvent::ALARM_CHANGE {
                        set: *cx.local.alarm_enabled,
                    })
                    .unwrap();
                    if *cx.local.alarm_enabled {
                        buzzer::spawn(BuzzerEvent::QuarterChime).unwrap();
                    }
                    let mut eeprom = cx.shared.eeprom;
                    eeprom.lock(|eeprom| {
                        eeprom.write(1, *cx.local.alarm_enabled as u16).unwrap();
                    });
                }
                _ => (),
            },
            MainState::Sunrise { ramp, prev_val } => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    pps_tick_handler(dt, gps_lock);
                    *datetime = dt;
                    let v;
                    loop {
                        match ramp.next() {
                            Some((cx, cy)) => match ramp.peek() {
                                Some((nx, _)) => {
                                    if cx != *nx {
                                        v = Some(cy);
                                        break;
                                    } else {
                                        continue;
                                    }
                                }
                                None => {
                                    v = Some(cy);
                                    break;
                                }
                            },
                            None => {
                                v = None;
                                break;
                            }
                        }
                    }

                    match v {
                        Some(y) => {
                            if let Some(y) = match prev_val {
                                None => Some(y),
                                Some(v) => {
                                    if *v == y {
                                        None
                                    } else {
                                        Some(y)
                                    }
                                }
                            } {
                                let rgb = SUNRISE_CMAP[TryInto::<usize>::try_into(y).unwrap()]
                                    .to_be_bytes()[1..4]
                                    .iter()
                                    .map(|v| GAMMA8[*v as usize])
                                    .collect::<Vec<_, 3>>();
                                let c = RGB8::new(rgb[0], rgb[1], rgb[2]) / 4;
                                light::spawn(LightEvent::SOLID_COLOR { color: c }).unwrap();
                                *prev_val = Some(y);
                            }
                        }
                        None => {
                            buzzer::spawn(BuzzerEvent::Alarm {
                                i: Some(*cx.local.curr_al),
                            })
                            .unwrap();
                            *state = MainState::Alarm;
                            *cx.local.alarm_dur = Duration::ZERO;
                        }
                    }
                }
                MainEvent::LongButtonPress | MainEvent::ShortButtonPress => {
                    *state = MainState::TimeDisplay;
                }
                _ => (),
            },
            MainState::Alarm => match ev {
                MainEvent::PPSTick { dt, gps_lock: _ } => {
                    *datetime = dt;
                    *cx.local.alarm_dur += Duration::SECOND;
                    if *cx.local.alarm_dur == ALARM_LEN {
                        buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                        *state = MainState::TimeDisplay;
                    }
                }
                MainEvent::LongButtonPress | MainEvent::ShortButtonPress => {
                    buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                    *state = MainState::TimeDisplay;
                }
                _ => {}
            },
            MainState::AlarmSetting => match ev {
                MainEvent::PPSTick { dt: _, gps_lock: _ } => {}
                MainEvent::LongButtonPress => {
                    let (h, m, _) = alarm_time.as_hms();
                    let hm: u16 = ((h as u16) << 8) | (m as u16);

                    let mut eeprom = cx.shared.eeprom;
                    eeprom.lock(|eeprom| {
                        eeprom.write(2, hm).unwrap();
                    });
                    *state = MainState::Lamp {
                        hsv: DEFAULT_LAMP_COLOR,
                    };
                    light::spawn(LightEvent::SOLID_COLOR {
                        color: hsv2rgb(DEFAULT_LAMP_COLOR),
                    })
                    .unwrap();
                    buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                }
                MainEvent::ShortButtonPress => {
                    *cx.local.curr_al += 1;
                    if *cx.local.curr_al >= ALARMS.len() {
                        *cx.local.curr_al = 0;
                    }
                    buzzer::spawn(BuzzerEvent::Alarm {
                        i: Some(*cx.local.curr_al),
                    })
                    .unwrap();
                }
                MainEvent::Encoder { dir } => match dir {
                    EncDir::Clockwise => {
                        *alarm_time += Duration::minutes(5);
                        disp_time(*alarm_time);
                    }
                    EncDir::CounterClockwise => {
                        *alarm_time -= Duration::minutes(5);
                        disp_time(*alarm_time);
                    }
                    _ => (),
                },
                _ => (),
            },
            MainState::Lamp { hsv } => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    *datetime = dt;
                    pps_tick_handler(dt, gps_lock);
                }
                MainEvent::ShortButtonPress => {
                    hsv.val += 5;
                    light::spawn(LightEvent::SOLID_COLOR {
                        color: hsv2rgb(*hsv),
                    })
                    .unwrap();
                }
                MainEvent::LongButtonPress => {
                    *state = MainState::Fire;
                    let mut leds = cx.shared.leds;
                    leds.lock(|leds| {
                        *leds = leds.map(|_| RGB8::default());
                    });
                    main_sm::spawn(MainEvent::Timeout).unwrap();
                }
                MainEvent::Encoder { dir } => match dir {
                    EncDir::Clockwise => {
                        hsv.hue += 1;
                        light::spawn(LightEvent::SOLID_COLOR {
                            color: hsv2rgb(*hsv),
                        })
                        .unwrap();
                    }
                    EncDir::CounterClockwise => {
                        hsv.hue -= 1;
                        light::spawn(LightEvent::SOLID_COLOR {
                            color: hsv2rgb(*hsv),
                        })
                        .unwrap();
                    }
                    _ => (),
                },
                _ => (),
            },
            MainState::Fire => {
                let mut leds = cx.shared.leds;
                let bmp = cx.local.bmp;
                let rng = cx.local.rng;

                match ev {
                    MainEvent::PPSTick { dt, gps_lock } => {
                        *datetime = dt;
                        pps_tick_handler(dt, gps_lock);
                    }
                    MainEvent::Timeout => {
                        fire_update(bmp, rng);

                        leds.lock(|leds| {
                            for i in 1..LED_ROWS + 1 {
                                for j in 1..LED_COLS + 1 {
                                    let k = if (bmp[i][j] as usize) < FIRE_PALETTE.len() {
                                        bmp[i][j]
                                    } else {
                                        (FIRE_PALETTE.len() - 1) as u16
                                    };
                                    let rgb: RGB8 =
                                        FIRE_PALETTE[k as usize].to_be_bytes()[1..4].as_pixels()[0];
                                    leds[leds_xy_to_n(
                                        j - 1,
                                        LED_ROWS - i,
                                        *curr_mode == LightTickMode::PROJECTOR,
                                    )] = rgb;
                                }
                            }
                        });

                        light::spawn(LightEvent::UPDATE).unwrap();
                        *cx.local.handle =
                            main_sm::spawn_after(50.millis(), MainEvent::Timeout).ok();
                    }
                    MainEvent::ShortButtonPress => {
                        if *curr_mode == LightTickMode::PROJECTOR {
                            *curr_mode = LightTickMode::DAY;
                        } else {
                            *curr_mode = LightTickMode::PROJECTOR;
                        }
                        light::spawn(LightEvent::TICK_MODE {
                            mode: (*curr_mode).clone(),
                        })
                        .unwrap();
                    }
                    MainEvent::LongButtonPress => {
                        if let Some(handle) = cx.local.handle.take() {
                            handle.cancel().ok();
                        }
                        leds.lock(|leds| {
                            *leds = leds.map(|_| RGB8::default());
                        });
                        for i in 0..LED_ROWS + 2 {
                            for j in 0..LED_COLS + 2 {
                                bmp[i][j] = 0;
                            }
                        }
                        *state = MainState::Coals { is_rgb: false };
                        main_sm::spawn(MainEvent::Timeout).unwrap();
                        main_sm::spawn(MainEvent::Timeout2).unwrap();
                    }
                    _ => (),
                }
            }
            MainState::Coals { is_rgb } => {
                let mut leds = cx.shared.leds;
                let bmp = cx.local.bmp;
                let rng = cx.local.rng;

                match ev {
                    MainEvent::PPSTick { dt, gps_lock } => {
                        *datetime = dt;
                        pps_tick_handler(dt, gps_lock);
                    }
                    MainEvent::Timeout => {
                        coals_update(bmp);

                        leds.lock(|leds| {
                            for i in 1..LED_ROWS + 1 {
                                for j in 1..LED_COLS + 1 {
                                    let v = if bmp[i][j] > 0xFF { 0xFF } else { bmp[i][j] };
                                    let c = if *is_rgb {
                                        if v == 0 {
                                            RGB8::default()
                                        } else {
                                            hsv2rgb(Hsv {
                                                hue: v.try_into().unwrap(),
                                                sat: 255,
                                                val: v.try_into().unwrap(),
                                            })
                                        }
                                    } else {
                                        let rgb = SUNRISE_CMAP
                                            [TryInto::<usize>::try_into(v).unwrap()]
                                        .to_be_bytes()[1..4]
                                            .iter()
                                            .map(|v| GAMMA8[*v as usize])
                                            .collect::<Vec<_, 3>>();
                                        RGB8::new(rgb[0], rgb[1], rgb[2]) / 2
                                    };

                                    leds[leds_xy_to_n(
                                        j - 1,
                                        LED_ROWS - i,
                                        *curr_mode == LightTickMode::PROJECTOR,
                                    )] = c;
                                }
                            }
                        });

                        light::spawn(LightEvent::UPDATE).unwrap();
                        *cx.local.handle =
                            main_sm::spawn_after(30.millis(), MainEvent::Timeout).ok();
                    }
                    MainEvent::Timeout2 => {
                        let x = rng.rand_range(1..(LED_COLS + 1) as u32) as usize;
                        let y = rng.rand_range(1..(LED_ROWS + 1) as u32) as usize;

                        if bmp[y][x] <= 0x1FF {
                            bmp[y][x] += rng.rand_range(0x300..0x3FF) as u16;
                        }

                        let tm = rng.rand_range(200..1300);
                        *cx.local.handle2 =
                            main_sm::spawn_after(tm.millis(), MainEvent::Timeout2).ok();
                    }
                    MainEvent::ShortButtonPress => *state = MainState::Coals { is_rgb: !*is_rgb },
                    MainEvent::LongButtonPress => {
                        if let Some(handle) = cx.local.handle.take() {
                            handle.cancel().ok();
                        }
                        if let Some(handle2) = cx.local.handle2.take() {
                            handle2.cancel().ok();
                        }
                        *state = MainState::TimeDisplay;
                        if let Some(dt) = *datetime {
                            disp_time(dt.time());
                        } else {
                            light::spawn(LightEvent::DIGITS {
                                dg: [10, 10, 10, 10, 10, 10],
                                col1: true,
                                col2: true,
                            })
                            .unwrap();
                        }
                    }
                    _ => (),
                }
            }
        }
    }

    #[task(binds = TIM2, local = [enc_timer, encoder, pos: isize = 0],
           priority = 2)]
    fn enc_timer(cx: enc_timer::Context) {
        let timer = cx.local.enc_timer;
        let encoder = cx.local.encoder;
        let pos = cx.local.pos;

        match encoder.update().unwrap() {
            EncDir::Clockwise => {
                *pos += 1;
                if *pos % 2 == 0 {
                    main_sm::spawn(MainEvent::Encoder {
                        dir: EncDir::Clockwise,
                    })
                    .unwrap();
                }
            }
            EncDir::CounterClockwise => {
                *pos -= 1;
                if *pos % 2 == 0 {
                    main_sm::spawn(MainEvent::Encoder {
                        dir: EncDir::CounterClockwise,
                    })
                    .unwrap();
                }
            }
            EncDir::None => {}
        }

        timer.clear_update_interrupt_flag();
    }

    #[idle(local = [led, wdg])]
    fn idle(cx: idle::Context) -> ! {
        cx.local.wdg.start(4000.ms());

        loop {
            cx.local.led.set_high();
            rtic::export::wfi();
            cx.local.led.set_low();
            cx.local.wdg.feed();
        }
    }
}
