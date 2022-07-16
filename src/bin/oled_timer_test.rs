#![deny(warnings)]
#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [PVD, WWDG])]
mod app {
    use core::fmt::Write;
    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use embedded_graphics::{
        mono_font::MonoTextStyleBuilder,
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyleBuilder, Rectangle, RoundedRectangle},
        text::{Baseline, Text},
    };
    use heapless::String;
    use profont::PROFONT_24_POINT;
    use rtic::Monotonic;
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
    use stm32f103_rtic_playground as _;
    use stm32f1xx_hal::gpio::{gpioc::PC13, Output, PushPull};
    use stm32f1xx_hal::{
        i2c::{BlockingI2c, DutyCycle, Mode},
        pac::I2C1,
        prelude::*,
    };

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
    const FREQ_MHZ: u32 = FREQ / 1_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        disp_busy: bool,
        update_pending: bool,
        count: u32,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        display: Display,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
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
            .freeze(&mut flash.acr);

        // Set up I2C.
        let mut afio = c.device.AFIO.constrain();
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
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display.set_brightness(Brightness::BRIGHTEST).unwrap();
        display.clear();

        let mut gpioc = c.device.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        let mut dcb = c.core.DCB;
        let dwt = c.core.DWT;
        let systick = c.core.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        display::spawn().unwrap();
        update::spawn_after(1.secs()).unwrap();

        (
            Shared {
                disp_busy: false,
                update_pending: false,
                count: 0,
            },
            Local { led, display },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [count, disp_busy, update_pending], priority = 2)]
    fn update(cx: update::Context) {
        let r1 = cx.shared.disp_busy;
        let r2 = cx.shared.update_pending;
        let r3 = cx.shared.count;

        (r1, r2, r3).lock(|db, up, c| {
            if !*db {
                *db = true;
                display::spawn().ok();
            } else {
                *up = true;
            }
            *c += 1;
            if *c == 1000000 {
                *c = 0;
            }
        });
        update::spawn_after(1.millis()).unwrap();
    }

    #[task(shared = [count, disp_busy, update_pending], local = [display], priority = 1)]
    fn display(cx: display::Context) {
        let r1 = cx.shared.disp_busy;
        let r2 = cx.shared.update_pending;
        let r3 = cx.shared.count;
        let start = monotonics::now();
        let d = cx.local.display;
        let text_style = MonoTextStyleBuilder::new()
            .font(&PROFONT_24_POINT)
            .text_color(BinaryColor::On)
            .build();

        let mut buf: String<6> = String::new();
        let c = (r3,).lock(|c| *c);
        buf.write_fmt(format_args!("{:06}", c)).unwrap();

        let style = PrimitiveStyleBuilder::new()
            .stroke_width(2)
            .stroke_color(BinaryColor::On)
            .fill_color(BinaryColor::Off)
            .build();

        d.clear();

        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(2, 3), Size::new(124, 28)),
            Size::new(10, 10),
        )
        .into_styled(style)
        .draw(d)
        .ok();

        Text::with_baseline(buf.as_str(), Point::new(16, 2), text_style, Baseline::Top)
            .draw(d)
            .ok();

        d.flush().ok();
        defmt::info!(
            "drawing took: {} us",
            (monotonics::now() - start).ticks() / FREQ_MHZ
        );
        (r1, r2).lock(|db, up| {
            if *up {
                *up = false;
                display::spawn().ok();
            } else {
                *db = false;
            }
        });
    }

    #[idle(local = [led])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            _cx.local.led.set_high();
            rtic::export::wfi();
            _cx.local.led.set_low();
        }
    }
}
