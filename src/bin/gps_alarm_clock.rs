//#![deny(warnings)]
#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [PVD, WWDG])]
mod app {
    use bbqueue::BBBuffer;
    use bresenham::Bresenham;
    use core::fmt::{Debug, Write};
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
    use heapless::String;
    use nb::block;
    use nmea0183::{
        coords::{Hemisphere, Latitude, Longitude},
        ParseResult, Parser,
    };
    use profont::{PROFONT_18_POINT, PROFONT_9_POINT};
    use rtic::Monotonic;
    use smart_leds::{SmartLedsWrite, RGB8};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use stm32f103_rtic_playground as _;
    use stm32f1xx_hal::gpio::{
        gpioa::PA1, gpioc::PC13, Cr, Edge, ExtiPin, Input, Output, PullUp, PushPull, CRL,
    };
    use stm32f1xx_hal::{
        flash::{FlashSize, Parts, SectorSize},
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac::I2C1,
        prelude::*,
        serial::{Config, Rx2, Serial},
        spi::{NoMiso, NoSck, Spi, Spi2NoRemap},
        timer::{CountDownTimer, Timer},
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
                    stm32f1xx_hal::gpio::gpiob::PB6<
                        stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                    >,
                    stm32f1xx_hal::gpio::gpiob::PB7<
                        stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
                    >,
                ),
            >,
        >,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >;

    const FREQ: u32 = 72_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    const BUFF_SIZE: usize = 1024;
    const NUM_LEDS: usize = 64;
    const UTC_OFFSET: Duration = Duration::HOUR;
    const ALARM_TIME: Time = time!(6:00:00);
    const SUNRISE_LEN: Duration = Duration::minutes(30);

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
        SUNRISE_START,
        TICK,
    }

    pub enum LightState {
        OFF,
        SUNRISE { prev_val: Option<isize> },
    }

    #[shared]
    struct Shared {
        datetime: Option<PrimitiveDateTime>,
        gps_locked: bool,
        is_sunrise: bool,
        pps_timeout: Option<pps_timeout::SpawnHandle>,
        eeprom: EEPROM<&'static mut stm32f1xx_hal::flash::Parts>,
        env_data: Option<(u16, i16)>,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        pps: PA1<Input<PullUp>>,
        rx: Rx2,
        parser: Parser,
        display: Display,
        ws: ws2812_spi::Ws2812<
            Spi<
                stm32f1xx_hal::pac::SPI2,
                Spi2NoRemap,
                (
                    NoSck,
                    NoMiso,
                    stm32f1xx_hal::gpio::gpiob::PB15<
                        stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>,
                    >,
                ),
                u8,
            >,
        >,
        leds: [RGB8; NUM_LEDS],
        ramp: Bresenham,
        prod: bbqueue::Producer<'static, BUFF_SIZE>,
        cons: bbqueue::Consumer<'static, BUFF_SIZE>,
        dst: Duration,
        am_pin: stm32f1xx_hal::gpio::Pin<
            stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
            CRL,
            'A',
            0_u8,
        >,
        am_timer: Option<Timer<stm32f1xx_hal::pac::TIM2>>,
        am_crl: Cr<stm32f1xx_hal::gpio::CRL, 'A'>,
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

        let mut pps = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);
        pps.make_interrupt_source(&mut afio);
        pps.enable_interrupt(&mut c.device.EXTI);
        pps.trigger_on_edge(&mut c.device.EXTI, Edge::Falling);

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
        let spi = Spi::spi2(c.device.SPI2, pins, ws2812::MODE, 3.mhz(), clocks);
        let leds = [RGB8::default(); NUM_LEDS];
        let mut ws = Ws2812::new(spi);
        ws.write(leds.iter().cloned()).unwrap();

        let mut dcb = c.core.DCB;
        let dwt = c.core.DWT;
        let systick = c.core.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        let parser = Parser::new(); //sentence_filter(Sentence::RMC | Sentence::RMC);
        let (prod, cons) = BB.try_split().unwrap();

        let ramp = Bresenham::new(
            (0, 0),
            (
                SUNRISE_LEN.whole_seconds().try_into().unwrap(),
                SUNRISE_CMAP.len().try_into().unwrap(),
            ),
        );

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

        let mut am_pin = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);
        am_pin.set_high();
        let am_timer = Timer::tim2(c.device.TIM2, &clocks);
        let am_crl = gpioa.crl;

        display::spawn_after(50.millis(), DispEvent::INIT).unwrap();

        (
            Shared {
                datetime: None,
                gps_locked: false,
                is_sunrise: false,
                pps_timeout: None,
                eeprom,
                env_data: None,
            },
            Local {
                led,
                pps,
                rx,
                parser,
                display,
                ws,
                leds,
                ramp,
                prod,
                cons,
                dst,
                am_pin,
                am_timer: Some(am_timer),
                am_crl,
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
                Ok(_) => {}
                Err(_) => {}
            }
        }

        let len = rgr.len();
        rgr.release(len);
    }

    #[task(binds = EXTI1, shared = [datetime, gps_locked, is_sunrise, pps_timeout, eeprom],
                           local = [pps, dst, am_tm: u8 = 2], priority = 16)]
    fn pps_isr(cx: pps_isr::Context) {
        let pps_isr::SharedResources {
            datetime,
            gps_locked,
            is_sunrise,
            pps_timeout,
            eeprom,
        } = cx.shared;

        cx.local.pps.clear_interrupt_pending_bit();

        defmt::debug!("pps");
        let (dt, gps) = (datetime, gps_locked, is_sunrise, pps_timeout, eeprom).lock(
            |dt, gps_locked, is_sunrise, pps_timeout, eeprom| {
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
                    if !*is_sunrise {
                        if (![Weekday::Saturday, Weekday::Sunday].contains(&dt_.date().weekday()))
                            && dt_.time() == (ALARM_TIME - SUNRISE_LEN)
                        {
                            *is_sunrise = true;
                            light::spawn(LightEvent::SUNRISE_START).unwrap();
                        }
                    } else {
                        light::spawn(LightEvent::TICK).unwrap();
                    }
                    (Some(dt_), *gps_locked)
                } else {
                    (None, *gps_locked)
                }
            },
        );
        if let Some(dt) = dt {
            display::spawn(DispEvent::UPDATE { dt: dt, gps: gps }).unwrap();
        }
        if *cx.local.am_tm == 0 {
            am_sensor::spawn_after(600.millis()).unwrap();
            *cx.local.am_tm = 10;
        } else {
            *cx.local.am_tm -= 1;
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

    #[task(shared = [is_sunrise], local = [ws, leds, ramp, state: LightState = LightState::OFF], priority = 1)]
    fn light(cx: light::Context, ev: LightEvent) {
        let ws = cx.local.ws;
        let leds = cx.local.leds;
        let ramp = cx.local.ramp;
        let s = cx.local.state;
        let mut is_sunrise = cx.shared.is_sunrise;

        match s {
            LightState::OFF => match ev {
                LightEvent::SUNRISE_START => {
                    for i in 0..leds.len() {
                        leds[i] = RGB8::default();
                    }
                    ws.write(leds.iter().cloned()).unwrap();
                    *s = LightState::SUNRISE { prev_val: None };
                }
                _ => {
                    defmt::panic!();
                }
            },
            LightState::SUNRISE { prev_val } => match ev {
                LightEvent::TICK => {
                    defmt::debug!("tick");
                    match ramp.next() {
                        Some((_, y)) => {
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
                                let v: u32 = SUNRISE_CMAP[TryInto::<usize>::try_into(y).unwrap()];
                                let c = RGB8 {
                                    b: v as u8,
                                    g: (v >> 8) as u8,
                                    r: (v >> 16) as u8,
                                };
                                *leds = leds.map(|_| c);
                                ws.write(leds.iter().cloned()).unwrap();
                                *prev_val = Some(y);
                            }
                        }
                        None => {
                            *leds = leds.map(|_| RGB8::default());
                            ws.write(leds.iter().cloned()).unwrap();
                            is_sunrise.lock(|is_sunrise| *is_sunrise = false);
                            *s = LightState::OFF;
                        }
                    }
                }
                _ => {
                    defmt::panic!();
                }
            },
        }
    }

    fn wait_for_state(
        timer: &mut CountDownTimer<stm32f1xx_hal::pac::TIM2>,
        pin: &mut stm32f1xx_hal::gpio::Pin<
            stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>,
            stm32f1xx_hal::gpio::CRL,
            'A',
            0_u8,
        >,
        state: bool,
    ) -> Result<bool, nb::Error<void::Void>> {
        if pin.is_high() == state {
            return Ok(true);
        } else {
            let res = timer.wait();
            return match res {
                Ok(_) => Ok(false),
                Err(e) => Err(e),
            };
        }
    }

    #[task(shared = [env_data], local = [am_pin, am_timer, am_crl], priority = 1)]
    fn am_sensor(cx: am_sensor::Context) {
        let timer = cx.local.am_timer;
        let pin = cx.local.am_pin;
        let mut crl = cx.local.am_crl;
        let mut env_data_r = cx.shared.env_data;

        let mut temp: u16 = 0;
        let mut humi: u16 = 0;
        let mut crc: u8 = 0;

        if let Some(tm) = timer.take() {
            let mut cd = tm.start_count_down(1000.ms());
            //START
            pin.as_push_pull_output(&mut crl, |out| {
                let mut s = || -> Result<(), void::Void> {
                    out.set_low();
                    cd.start(20.ms());
                    block!(cd.wait())?;
                    out.set_high();
                    cd.start(25.us());
                    block!(cd.wait())?;
                    Ok(())
                };
                if let Err(_) = s() {
                    defmt::error!("Failed in START");
                    return;
                }
            });

            //DATA
            pin.as_pull_up_input(&mut crl, |inp| {
                let mut s = || -> Result<(u16, i16), &str> {
                    if !inp.is_low() {
                        return Err("not low before ACK");
                    }
                    cd.start(100.us());
                    let res = block!(wait_for_state(&mut cd, inp, true)).unwrap();
                    if !res {
                        return Err("error in ACK 1");
                    }

                    cd.start(100.us());
                    let res = block!(wait_for_state(&mut cd, inp, false)).unwrap();
                    if !res {
                        return Err("error in ACK 2");
                    }

                    for i in 0..40 {
                        cd.start(80.us());
                        let res = block!(wait_for_state(&mut cd, inp, true)).unwrap();
                        if !res {
                            return Err("error in DATA 1");
                        }

                        cd.start(35.us());
                        let res = block!(wait_for_state(&mut cd, inp, false)).unwrap();
                        if res == false {
                            cd.start(55.us());
                            let res = block!(wait_for_state(&mut cd, inp, false)).unwrap();
                            if res == true {
                                if i < 16 {
                                    humi |= 1 << (15 - i);
                                } else if i < 32 {
                                    temp |= 1 << (15 - (i - 16));
                                } else {
                                    crc |= 1 << (7 - (i - 32));
                                }
                            } else {
                                return Err("error in DATA 2");
                            }
                        }
                    }

                    let t = if temp & 0x8000 != 0 {
                        -((temp & 0x7FF) as i16)
                    } else {
                        temp as i16
                    };
                    defmt::debug!(
                        "Temperature: {}.{}C, Humidity: {}.{}%, CRC: {}",
                        humi / 10,
                        humi % 10,
                        t / 10,
                        t % 10,
                        crc
                    );
                    let crc_ex: u8 = ((humi) + (humi >> 8) + (temp) + (temp >> 8)) as u8;
                    if crc != crc_ex {
                        defmt::warn!("Wrong CRC. Expected: {}, got: {}", crc_ex, crc);
                    }
                    Ok((humi, t))
                };
                match s() {
                    Err(e) => {
                        defmt::error!("Failed in DATA: {}", e);
                        env_data_r.lock(|env_data| *env_data = None);
                    }
                    Ok(data) => env_data_r.lock(|env_data| *env_data = Some(data)),
                }
            });

            // Not sure why this is needed, but it's needed
            // Probably bug in HAL
            pin.as_push_pull_output(&mut crl, |out| {
                cd.start(5.us());
                block!(cd.wait()).ok();
                out.set_high();
            });

            *timer = Some(cd.stop());
        } else {
            defmt::panic!();
        }
    }

    #[idle(local = [led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local.led.set_high();
            rtic::export::wfi();
            cx.local.led.set_low();
        }
    }
}
