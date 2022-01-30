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
    use embedded_graphics::{
        image::{Image, ImageRaw},
        mono_font::MonoTextStyleBuilder,
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use heapless::{String, Vec};
    use nmea0183::{
        coords::{Hemisphere, Latitude, Longitude},
        ParseResult, Parser, Sentence,
    };
    use profont::{PROFONT_18_POINT, PROFONT_9_POINT};
    use rtic::Monotonic;
    use smart_leds::{
        hsv::{hsv2rgb, Hsv},
        SmartLedsWrite, RGB, RGB8,
    };
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use stm32f103_rtic_playground as _;
    use stm32f1xx_hal::gpio::{
        gpioa::{PA1, PA4},
        gpioc::PC13,
        Alternate, Cr, Edge, ExtiPin, Input, OpenDrain, Output, PullDown, PullUp, PushPull, CRL,
    };
    use stm32f1xx_hal::watchdog::IndependentWatchdog;
    use stm32f1xx_hal::{
        flash::{FlashSize, Parts, SectorSize},
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac::{EXTI, I2C1, TIM3},
        prelude::*,
        pwm::{Channel, Pwm, C1},
        serial::{Config, Rx2, Serial},
        spi::{NoMiso, NoSck, Spi, Spi2NoRemap},
        timer::{Tim3NoRemap, Timer},
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

    type Display = Ssd1306<
        I2CInterface<
            BlockingI2c<
                I2C1,
                (
                    stm32f1xx_hal::gpio::gpiob::PB6<Alternate<OpenDrain>>,
                    stm32f1xx_hal::gpio::gpiob::PB7<Alternate<OpenDrain>>,
                ),
            >,
        >,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >;

    const FREQ: u32 = 72_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    const BUFF_SIZE: usize = 512;
    const NUM_LEDS: usize = 128;
    const UTC_OFFSET: Duration = Duration::HOUR;
    const ALARM_TIME: Time = time!(6:00:00);
    const SUNRISE_LEN: Duration = Duration::minutes(30);
    const BUZZER_BASE_FREQ_HZ: u32 = 4000;
    const LED_LEVEL_DAY: u8 = 16;
    const LED_LEVEL_NIGHT: u8 = 1;

    const EEPROM_PARAMS: Params = Params {
        first_page: 126,
        flash_size: FlashSize::Sz128K,
        page_size: SectorSize::Sz1K,
        page_count: 2,
    };

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

    pub enum DispState {
        INIT,
        READY,
    }

    #[derive(Debug)]
    pub enum DispEvent {
        INIT,
        UPDATE { dt: PrimitiveDateTime, gps: bool },
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum LightEvent {
        SOLID_COLOR { color: RGB8 },
        TICK { h: u8, m: u8, s: u8 },
    }

    #[shared]
    struct Shared {
        datetime: Option<PrimitiveDateTime>,
        gps_locked: bool,
        pps_timeout: Option<pps_timeout::SpawnHandle>,
        eeprom: EEPROM<&'static mut stm32f1xx_hal::flash::Parts>,
        env_data: Option<(u16, i16)>,
        am_pin: stm32f1xx_hal::gpio::Pin<stm32f1xx_hal::gpio::Dynamic, CRL, 'A', 0_u8>,
        am_reset: bool,
        btn_pin: PA4<Input<PullUp>>,
        exti: EXTI,
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
        display: Display,
        ws: Leds,
        leds: [RGB8; NUM_LEDS],
        prod: bbqueue::Producer<'static, BUFF_SIZE>,
        cons: bbqueue::Consumer<'static, BUFF_SIZE>,
        dst: Duration,
        am_crl: Cr<stm32f1xx_hal::gpio::CRL, 'A'>,
        am_afio: stm32f1xx_hal::afio::Parts,
        buzz_pwm: Pwm<TIM3, Tim3NoRemap, C1, stm32f1xx_hal::gpio::gpioa::PA6<Alternate<PushPull>>>,
        wdg: IndependentWatchdog,
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
            .pclk1(24.mhz())
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

        // Set up I2C.
        let mut gpiob = c.device.GPIOB.split();
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = BlockingI2c::i2c1(
            c.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let interface = I2CDisplayInterface::new(i2c);
        let display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let pins = (
            NoSck,
            NoMiso,
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );
        let spi = Spi::spi2(c.device.SPI2, pins, ws2812::MODE, 3000.khz(), clocks);
        let mut leds = [RGB8::default(); NUM_LEDS];

        for x in 0..16 {
            for y in 0..8 {
                leds[leds_xy_to_n(x, y)] = hsv2rgb(Hsv {
                    hue: (x * 255 / 16).try_into().unwrap(),
                    sat: 255,
                    val: 3,
                });
            }
        }

        let mut ws = Ws2812::new(spi);
        ws.write(leds.iter().cloned()).unwrap();

        let mut dcb = c.core.DCB;
        let dwt = c.core.DWT;
        let systick = c.core.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        let parser = Parser::new().sentence_filter(Sentence::RMC | Sentence::RMC);
        let (prod, cons) = BB.try_split().unwrap();

        let mut eeprom = flash.eeprom(EEPROM_PARAMS);
        eeprom.init().unwrap();
        let dst = match eeprom.read(0).unwrap() {
            0 => Duration::ZERO,
            1 => Duration::HOUR,
            _ => {
                eeprom.write(0, 1).unwrap();
                Duration::HOUR
            }
        };
        defmt::info!("DST: {}", dst.whole_hours());

        let buzz_pin = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
        let mut buzz_pwm = Timer::new(c.device.TIM3, &clocks).pwm::<Tim3NoRemap, _, _, _>(
            buzz_pin,
            &mut afio.mapr,
            BUZZER_BASE_FREQ_HZ.hz(),
        );
        buzz_pwm.set_duty(Channel::C1, 2 * (buzz_pwm.get_max_duty() / 100));

        let mut am_pin = gpioa.pa0.into_dynamic(&mut gpioa.crl);
        am_pin.make_push_pull_output(&mut gpioa.crl);
        am_pin.set_high().unwrap();
        let am_crl = gpioa.crl;

        let wdg = IndependentWatchdog::new(c.device.IWDG);

        display::spawn_after(50.millis(), DispEvent::INIT).unwrap();
        main_sm::spawn_after(2.secs(), MainEvent::Timeout).unwrap();

        (
            Shared {
                datetime: None,
                gps_locked: false,
                pps_timeout: None,
                eeprom,
                env_data: None,
                am_pin,
                am_reset: true,
                btn_pin: btn,
                exti: c.device.EXTI,
            },
            Local {
                led,
                pps,
                rx,
                parser,
                display,
                ws,
                leds,
                prod,
                cons,
                dst,
                am_crl,
                am_afio: afio,
                buzz_pwm,
                wdg,
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
                    parse::spawn().unwrap();
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

    #[task(shared = [datetime, pps_timeout, gps_locked], local = [cons, parser], priority = 2)]
    fn parse(cx: parse::Context) {
        let cons = cx.local.cons;
        let parser = cx.local.parser;

        let parse::SharedResources {
            mut datetime,
            pps_timeout,
            gps_locked,
        } = cx.shared;

        let mut res1 = (gps_locked, pps_timeout);

        let rgr = cons.read().unwrap();
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
            am_sensor::spawn_after(800.millis(), AmSensorEvent::START).unwrap();
        }
    }

    #[task(shared = [gps_locked], local = [], priority = 2)]
    fn pps_timeout(cx: pps_timeout::Context) {
        let mut gps_locked = cx.shared.gps_locked;
        defmt::warn!("GPS lock lost");
        gps_locked.lock(|gps_locked| *gps_locked = false)
    }

    fn display_big_txt(d: &mut Display, str: &str) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&PROFONT_18_POINT)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(str, Point::new(18, 10), text_style, Baseline::Top)
            .draw(d)
            .unwrap();
    }

    fn display_small_txt(d: &mut Display, str: &str) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&PROFONT_9_POINT)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(str, Point::new(18, 0), text_style, Baseline::Top)
            .draw(d)
            .unwrap();
    }

    #[task(shared = [env_data], local = [display, state: DispState = DispState::INIT], priority = 1)]
    fn display(cx: display::Context, ev: DispEvent) {
        let d = cx.local.display;
        let s = cx.local.state;
        let mut env_data_r = cx.shared.env_data;

        match s {
            DispState::INIT => match ev {
                DispEvent::INIT => {
                    defmt::info!("Initializing display");
                    d.init().unwrap();
                    //d.set_brightness(Brightness::BRIGHTEST).unwrap();
                    d.clear();
                    let title_screen_raw: ImageRaw<BinaryColor> =
                        ImageRaw::new(include_bytes!("../../assets/title_screen.raw"), 128);
                    let title_screen = Image::new(&title_screen_raw, Point::new(0, 0));
                    title_screen.draw(d).unwrap();
                    d.flush().unwrap();
                    *s = DispState::READY;
                }
                _ => {}
            },
            DispState::READY => match ev {
                DispEvent::UPDATE { dt, gps } => {
                    let mut buf: String<32> = String::new();
                    let (h, m, s) = dt.time().as_hms();
                    buf.write_fmt(format_args!("{:02}:{:02}:{:02}", h, m, s))
                        .unwrap();
                    d.clear();
                    display_big_txt(d, buf.as_str());

                    if gps {
                        let gps_icon_raw: ImageRaw<BinaryColor> =
                            ImageRaw::new(include_bytes!("../../assets/sat_icon.raw"), 14);
                        let gps_icon = Image::new(&gps_icon_raw, Point::new(0, 0));
                        gps_icon.draw(d).unwrap();
                    }

                    if let Some((h, t)) = env_data_r.lock(|env_data| *env_data) {
                        buf.clear();
                        buf.write_fmt(format_args!(
                            "{}.{}% {}.{}C",
                            h / 10,
                            h % 10,
                            t / 10,
                            t % 10
                        ))
                        .unwrap();
                        display_small_txt(d, buf.as_str());
                    }

                    d.flush().unwrap();
                }
                _ => {}
            },
        }
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

    #[task(shared = [], local = [ws, leds], priority = 16, capacity = 2)]
    fn light(cx: light::Context, ev: LightEvent) {
        let ws = cx.local.ws;
        let leds = cx.local.leds;

        match ev {
            LightEvent::TICK { h, m, s } => {
                let mut buf: String<4> = String::new();
                buf.write_fmt(format_args!("{:02}{:02}", h, m)).unwrap();

                *leds = leds.map(|_| RGB8::default());
                let is_night = if h > 20 || h < 6 { true } else { false };

                if !is_night {
                    for i in 0..(s / 4) {
                        leds[leds_xy_to_n(i as usize, 0)] = RGB8::new(0, LED_LEVEL_DAY + 1, 0);
                    }

                    leds[leds_xy_to_n((s / 4) as usize, 0)] =
                        RGB8::new(0, 1 + (LED_LEVEL_DAY * (s % 4) / 3), 0);
                }

                for (x, mut d) in buf.bytes().enumerate() {
                    d -= '0' as u8;
                    leds_draw_digit(
                        leds,
                        1 + (4 * x),
                        2,
                        d as u8,
                        RGB8::new(
                            if is_night {
                                LED_LEVEL_NIGHT
                            } else {
                                LED_LEVEL_DAY
                            },
                            0,
                            0,
                        ),
                    );
                }
                leds_write(ws, leds);
            }
            LightEvent::SOLID_COLOR { color } => {
                *leds = leds.map(|_| color);
                leds_write(ws, leds);
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

    #[derive(Debug, PartialEq, Format)]
    pub enum AmSensorEvent {
        START,
        DATA { data: [u16; 5] },
        TIMEOUT,
    }

    #[task(binds = EXTI0, shared = [am_pin, am_reset],
                          local = [last: u32 = 0,
                                   i: usize = 0,
                                   data: [u16; 5] = [0; 5]], priority = 2)]
    fn am_data_isr(cx: am_data_isr::Context) {
        let pin_r = cx.shared.am_pin;
        let reset_r = cx.shared.am_reset;
        let i = cx.local.i;
        let data = cx.local.data;

        let pps_int = (pin_r, reset_r).lock(|pin, reset| {
            let intr = pin.check_interrupt();
            if intr {
                pin.clear_interrupt_pending_bit();
                if *reset {
                    *i = 0;
                    *cx.local.last = 0;
                    *reset = false;
                }
            }
            intr
        });

        if pps_int {
            *i += 1;
            let curr = monotonics::now().ticks();
            let len = (curr - *cx.local.last) / 72;

            if *i > 2 {
                if (*i % 2) == 1 {
                    if !(len >= 40 && len < 70) {
                        defmt::warn!("Wrong preamble length");
                    }
                } else {
                    let j = (*i - 4) / 2;
                    let (n, m) = ((j / 8), 7 - (j % 8));
                    match len {
                        20..=40 => {
                            data[n] &= !(1 << m);
                        }
                        60..=80 => {
                            data[n] |= 1 << m;
                        }
                        _ => defmt::warn!("Wrong bit length"),
                    }
                }
                if *i >= 83 {
                    defmt::debug!("Sensor data: {}", data);
                    *i = 0;
                    am_sensor::spawn(AmSensorEvent::DATA { data: data.clone() }).unwrap();
                }
            }
            *cx.local.last = curr;
        }
    }

    #[task(shared = [env_data, am_pin, am_reset, exti], local = [am_crl, am_afio,
                                                 state: AmSensorState = AmSensorState::START1,
                                                 handle: Option<am_sensor::SpawnHandle> = None,
                                                 count: u16 = 0],
                                                 priority = 1, capacity = 2)]
    fn am_sensor(cx: am_sensor::Context, ev: AmSensorEvent) {
        let mut pin_r = cx.shared.am_pin;
        let mut crl = cx.local.am_crl;
        let mut env_data_r = cx.shared.env_data;
        let mut afio = cx.local.am_afio;
        let mut exti = cx.shared.exti;
        let mut reset_r = cx.shared.am_reset;

        let handle = cx.local.handle;

        let s = cx.local.state;

        pin_r.lock(|pin| match s {
            AmSensorState::START1 => {
                if ev == AmSensorEvent::START {
                    pin.make_push_pull_output(&mut crl);
                    pin.set_low().unwrap();
                    am_sensor::spawn_after(20.millis(), AmSensorEvent::TIMEOUT).unwrap();
                    reset_r.lock(|reset| {
                        *reset = true;
                    });
                    *s = AmSensorState::START2;
                } else {
                    defmt::warn!("Unexpected event {}. Ignoring", ev);
                }
            }
            AmSensorState::START2 => {
                defmt::assert!(ev == AmSensorEvent::TIMEOUT);
                pin.set_high().unwrap();
                am_sensor::spawn_after(20.micros(), AmSensorEvent::TIMEOUT).unwrap();
                *s = AmSensorState::ACK1;
            }
            AmSensorState::ACK1 => {
                defmt::assert!(ev == AmSensorEvent::TIMEOUT);
                pin.make_pull_up_input(&mut crl);
                am_sensor::spawn_after(30.micros(), AmSensorEvent::TIMEOUT).unwrap();
                *s = AmSensorState::ACK2;
            }
            AmSensorState::ACK2 => {
                defmt::assert!(ev == AmSensorEvent::TIMEOUT);
                if pin.is_low().unwrap() {
                    pin.make_interrupt_source(&mut afio);

                    exti.lock(|exti| {
                        pin.enable_interrupt(exti);
                        pin.trigger_on_edge(exti, Edge::RisingFalling);
                    });
                    *handle = am_sensor::spawn_after(10.millis(), AmSensorEvent::TIMEOUT).ok();
                    *s = AmSensorState::RX;
                } else {
                    defmt::warn!("Sensor not responding");
                    env_data_r.lock(|env_data| *env_data = None);
                    *s = AmSensorState::START1;
                }
            }
            AmSensorState::RX => {
                match ev {
                    AmSensorEvent::DATA { data } => {
                        if let Some(handle) = handle.take() {
                            handle.cancel().ok();
                        }
                        let crc: u16 = data.split_last().unwrap().1.iter().sum();

                        let crc_exp: u8 = data[4] as u8;
                        let temp: u16 = (data[2] << 8) | data[3];
                        let humi: u16 = (data[0] << 8) | data[1];

                        let signed_temp = if temp & 0x8000 != 0 {
                            -((temp & 0x7FF) as i16)
                        } else {
                            temp as i16
                        };

                        defmt::info!("Temp: {},  RHS: {}", signed_temp, humi);
                        if crc as u8 != crc_exp {
                            defmt::warn!("Wrong CRC");
                        }
                        env_data_r.lock(|env_data| *env_data = Some((humi, signed_temp)));
                    }
                    AmSensorEvent::TIMEOUT => {
                        defmt::warn!("Timeout");
                    }
                    _ => {
                        defmt::panic!()
                    }
                }

                exti.lock(|exti| {
                    pin.disable_interrupt(exti);
                });
                pin.clear_interrupt_pending_bit();
                *s = AmSensorState::START1;
            }
        });
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
           priority = 2,
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

    #[rustfmt::skip]
    const _LED_DIGITS: [[u8; 5]; 10] =
                                      [[0b111,
                                        0b101,
                                        0b101,
                                        0b101,
                                        0b111],

                                       [0b010,
                                        0b110,
                                        0b010,
                                        0b010,
                                        0b010],

                                       [0b111,
                                        0b001,
                                        0b111,
                                        0b100,
                                        0b111],

                                       [0b111,
                                        0b001,
                                        0b111,
                                        0b001,
                                        0b111],
                                        
                                       [0b101,
                                        0b101,
                                        0b111,
                                        0b001,
                                        0b001],

                                       [0b111,
                                        0b100,
                                        0b111,
                                        0b001,
                                        0b111],

                                       [0b111,
                                        0b100,
                                        0b111,
                                        0b101,
                                        0b111],

                                       [0b111,
                                        0b101,
                                        0b001,
                                        0b001,
                                        0b001],

                                       [0b111,
                                        0b101,
                                        0b111,
                                        0b101,
                                        0b111],

                                       [0b111,
                                        0b101,
                                        0b111,
                                        0b001,
                                        0b111],
                                        ];

    fn leds_xy_to_n(x: usize, y: usize) -> usize {
        let _y = 7 - y;
        let _x = 15 - x;
        let n = if _x > 7 { NUM_LEDS / 2 } else { 0 };
        return n + (_y * 8) + (_x % 8);
    }

    fn leds_draw_digit(leds: &mut [RGB8; NUM_LEDS], x: usize, y: usize, d: u8, color: RGB8) {
        for (j, bits) in _LED_DIGITS[d as usize].iter().enumerate() {
            for i in 0..3 {
                if bits & (1 << (2 - i)) != 0 {
                    leds[leds_xy_to_n(i + x, j + y)] = color;
                }
            }
        }
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
        TitleScreen,
        TimeDisplay,
        Sunrise {
            ramp: Peekable<Bresenham>,
            prev_val: Option<isize>,
        },
        Alarm,
        AlarmSetting,
    }

    #[derive(Debug)]
    pub enum MainEvent {
        Timeout,
        PPSTick {
            dt: Option<PrimitiveDateTime>,
            gps_lock: bool,
        },
        ShortButtonPress,
        LongButtonPress,
    }

    fn pps_tick_handler(dt: Option<PrimitiveDateTime>, gps_lock: bool) {
        if let Some(dt) = dt {
            display::spawn(DispEvent::UPDATE {
                dt: dt,
                gps: gps_lock,
            })
            .ok();

            let (h, s) = (dt.time().hour(), dt.time().second());
            if h < 22 && h > 7 {
                match dt.time().minute() {
                    0 => {
                        if s == 0 {
                            buzzer::spawn(BuzzerEvent::HourChime).unwrap();
                        } else {
                            buzzer::spawn(BuzzerEvent::SecondTick).unwrap();
                        }
                    }
                    59 => {
                        if s > 56 {
                            buzzer::spawn(BuzzerEvent::QuarterChime).unwrap();
                        } else {
                            buzzer::spawn(BuzzerEvent::SecondTick).unwrap();
                        }
                    }
                    15 | 30 | 45 => {
                        if s == 0 {
                            buzzer::spawn(BuzzerEvent::QuarterChime).unwrap()
                        } else {
                            buzzer::spawn(BuzzerEvent::SecondTick).unwrap();
                        }
                    }
                    _ => buzzer::spawn(BuzzerEvent::SecondTick).unwrap(),
                }
            }
        } else {
            buzzer::spawn(BuzzerEvent::SecondTick).unwrap();
        }
    }

    #[task(shared = [], local = [curr_al: usize = 0, state: MainState = MainState::TitleScreen],
                        priority = 4, capacity = 4)]
    fn main_sm(cx: main_sm::Context, ev: MainEvent) {
        let state = cx.local.state;

        match state {
            MainState::TitleScreen => match ev {
                MainEvent::PPSTick { .. } => {
                    buzzer::spawn(BuzzerEvent::SecondTick).unwrap();
                }
                MainEvent::Timeout => {
                    *state = MainState::TimeDisplay;
                }
                _ => (),
            },
            MainState::TimeDisplay => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    pps_tick_handler(dt, gps_lock);
                    if let Some(dt) = dt {
                        if ![Weekday::Saturday, Weekday::Sunday].contains(&dt.date().weekday()) {
                            if dt.time() == (ALARM_TIME - SUNRISE_LEN) {
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

                        let (h, m, s) = dt.time().as_hms();
                        light::spawn(LightEvent::TICK { h, m, s }).unwrap();
                    }
                }
                MainEvent::LongButtonPress => {
                    let (h, m, s) = ALARM_TIME.as_hms();
                    light::spawn(LightEvent::TICK { h, m, s }).unwrap();

                    buzzer::spawn(BuzzerEvent::Alarm {
                        i: Some(*cx.local.curr_al),
                    })
                    .unwrap();
                    *state = MainState::AlarmSetting;
                }
                _ => (),
            },
            MainState::Sunrise { ramp, prev_val } => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    pps_tick_handler(dt, gps_lock);
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
                                let c = RGB8::new(rgb[0], rgb[1], rgb[2]);
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
                        }
                    }
                }
                _ => (),
            },
            MainState::Alarm => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    if let Some(dt) = dt {
                        display::spawn(DispEvent::UPDATE {
                            dt: dt,
                            gps: gps_lock,
                        })
                        .ok();
                    }
                }
                MainEvent::LongButtonPress | MainEvent::ShortButtonPress => {
                    buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                    *state = MainState::TimeDisplay;
                }
                _ => {}
            },
            MainState::AlarmSetting => match ev {
                MainEvent::PPSTick { dt, gps_lock } => {
                    if let Some(dt) = dt {
                        display::spawn(DispEvent::UPDATE {
                            dt: dt,
                            gps: gps_lock,
                        })
                        .unwrap()
                    }
                }
                MainEvent::LongButtonPress => {
                    *state = MainState::TimeDisplay;
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
                _ => (),
            },
        }
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
