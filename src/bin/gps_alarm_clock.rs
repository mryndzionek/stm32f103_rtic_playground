//#![deny(warnings)]
#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [PVD, WWDG, RTC, SPI1])]
mod app {
    use bbqueue::BBBuffer;
    use bresenham::Bresenham;
    use core::fmt::{Debug, Write};
    use core::iter::Peekable;
    use core::ops::RangeInclusive;
    use core::slice::Iter;
    use defmt::{write, Debug2Format, Format, Formatter};
    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use eeprom::{EEPROMExt, Params, EEPROM};
    use heapless::{String, Vec};
    use infrared::protocol::nec::NecCommand;
    use infrared::protocol::Nec;
    use infrared::remotecontrol::Action::*;
    use infrared::remotecontrol::{Action, Button, DeviceType, RemoteControlModel};
    use infrared::ProtocolId;
    use infrared::Receiver;
    use nmea0183::{
        coords::{Hemisphere, Latitude, Longitude},
        ParseResult, Parser, Sentence,
    };
    use oorandom::Rand32;
    use rtic::Monotonic;
    use smart_leds::RGB8;
    use stm32f103_rtic_playground as _;
    use stm32f1xx_hal::gpio::{
        gpioa::PA1, gpiob::PB0, gpioc::PC13, Alternate, Edge, ExtiPin, Floating, Input, Output,
        PullDown, PushPull,
    };
    use stm32f1xx_hal::watchdog::IndependentWatchdog;
    use stm32f1xx_hal::{
        dma::{Transfer, TxDma},
        flash::{FlashSize, Parts, SectorSize},
        pac::TIM1,
        pac::TIM3,
        prelude::*,
        serial::{Config, Rx2, Serial},
        spi::{Mode, NoMiso, NoSck, Phase, Polarity, Spi, Spi2NoRemap},
        timer::{Ch, Channel, PwmHz, Tim1NoRemap, Tim3NoRemap, Timer},
    };
    use time::macros::time;
    use time::{Date, Duration, Month, PrimitiveDateTime, Time, Weekday};

    use stm32f103_rtic_playground::*;

    #[derive(Debug, Default)]
    pub struct Mp3Remote;

    impl RemoteControlModel for Mp3Remote {
        const MODEL: &'static str = "Special for Mp3";
        const DEVTYPE: DeviceType = DeviceType::Generic;
        const PROTOCOL: ProtocolId = ProtocolId::Nec;
        const ADDRESS: u32 = 0;
        type Cmd = NecCommand;

        const BUTTONS: &'static [(u32, Action)] = &[
            (69, ChannelListPrev),
            (70, ChannelList),
            (71, ChannelListNext),
            (68, Prev),
            (64, Next),
            (67, Play_Pause),
            (7, VolumeDown),
            (21, VolumeUp),
            (9, Eq),
            (22, Zero),
            (25, Up),
            (13, Down),
            (12, One),
            (24, Two),
            (94, Three),
            (8, Four),
            (28, Five),
            (90, Six),
            (66, Seven),
            (82, Eight),
            (74, Nine),
        ];
    }
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

    struct SerialErrorWrapper(stm32f1xx_hal::serial::Error);
    impl Format for SerialErrorWrapper {
        fn format(&self, fmt: Formatter) {
            let s = match self.0 {
                stm32f1xx_hal::serial::Error::Framing => "Framing error",
                stm32f1xx_hal::serial::Error::Noise => "Noise error",
                stm32f1xx_hal::serial::Error::Overrun => "Overrun error",
                stm32f1xx_hal::serial::Error::Parity => "Parity error",
                _ => "Unknown error",
            };
            write!(fmt, "{}", s);
        }
    }

    pub struct LinearRamp {
        start: usize,
        end: usize,
        b: Peekable<Bresenham>,
    }

    impl LinearRamp {
        #[inline]
        pub fn new(start: usize, end: usize) -> LinearRamp {
            LinearRamp {
                start,
                end,
                b: Bresenham::new((0, 0), (start.try_into().unwrap(), end.try_into().unwrap()))
                    .peekable(),
            }
        }
    }

    impl PartialEq for LinearRamp {
        fn eq(&self, other: &Self) -> bool {
            (self.start == other.start) && (self.end == other.end)
        }
    }

    impl Iterator for LinearRamp {
        type Item = isize;

        #[inline]
        fn next(&mut self) -> Option<Self::Item> {
            let v;
            loop {
                match self.b.next() {
                    Some((cx, cy)) => match self.b.peek() {
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
            v
        }
    }

    pub trait ColorMap {
        fn get(&self, x: usize, y: usize) -> RGB8;
    }

    pub struct SolidColor {
        color: RGB8,
    }

    impl SolidColor {
        #[inline]
        pub fn new(color: RGB8) -> Self {
            Self { color }
        }
    }

    impl ColorMap for SolidColor {
        fn get(&self, _: usize, _: usize) -> RGB8 {
            self.color
        }
    }

    pub struct Gradient;

    impl Gradient {
        const LED_PALETTE_BRG: [RGB8; DIGIT_HEIGHT] = [
            RGB8::new(254, 1, 0),
            RGB8::new(224, 0, 31),
            RGB8::new(192, 0, 63),
            RGB8::new(160, 0, 95),
            RGB8::new(128, 0, 127),
            RGB8::new(96, 0, 159),
            RGB8::new(64, 0, 191),
            RGB8::new(32, 0, 223),
            RGB8::new(0, 0, 255),
        ];

        #[inline]
        pub fn new() -> Self {
            Self {}
        }
    }

    impl Default for Gradient {
        fn default() -> Self {
            Self::new()
        }
    }

    impl ColorMap for Gradient {
        fn get(&self, _: usize, y: usize) -> RGB8 {
            if y < Gradient::LED_PALETTE_BRG.len() {
                Gradient::LED_PALETTE_BRG[y]
            } else {
                defmt::panic!();
            }
        }
    }

    pub struct FireSim<const X: usize, const Y: usize> {
        pixels: [[u8; X]; Y],
        prev_pixels: [[u8; X]; Y],
        rng: Rand32,
        slf: usize,
        c: usize,
    }

    impl<const X: usize, const Y: usize> FireSim<X, Y> {
        const FIRE_PALETTE: [RGB8; 40] = [
            RGB8::new(11, 0, 0),
            RGB8::new(24, 0, 0),
            RGB8::new(37, 0, 0),
            RGB8::new(50, 0, 0),
            RGB8::new(66, 0, 0),
            RGB8::new(79, 0, 0),
            RGB8::new(92, 0, 0),
            RGB8::new(105, 0, 0),
            RGB8::new(121, 0, 0),
            RGB8::new(134, 0, 0),
            RGB8::new(147, 0, 0),
            RGB8::new(160, 0, 0),
            RGB8::new(176, 0, 0),
            RGB8::new(189, 0, 0),
            RGB8::new(202, 0, 0),
            RGB8::new(215, 0, 0),
            RGB8::new(231, 0, 0),
            RGB8::new(244, 0, 0),
            RGB8::new(255, 2, 0),
            RGB8::new(255, 16, 0),
            RGB8::new(255, 31, 0),
            RGB8::new(255, 44, 0),
            RGB8::new(255, 58, 0),
            RGB8::new(255, 71, 0),
            RGB8::new(255, 86, 0),
            RGB8::new(255, 100, 0),
            RGB8::new(255, 113, 0),
            RGB8::new(255, 126, 0),
            RGB8::new(255, 142, 0),
            RGB8::new(255, 155, 0),
            RGB8::new(255, 168, 0),
            RGB8::new(255, 181, 0),
            RGB8::new(255, 197, 0),
            RGB8::new(255, 210, 0),
            RGB8::new(255, 223, 0),
            RGB8::new(255, 236, 0),
            RGB8::new(255, 252, 0),
            RGB8::new(255, 255, 15),
            RGB8::new(255, 255, 34),
            RGB8::new(255, 255, 54),
        ];

        #[inline]
        fn new(seed: u64, slf: usize) -> Self {
            Self {
                pixels: [[0; X]; Y],
                prev_pixels: [[0; X]; Y],
                rng: Rand32::new(seed),
                slf,
                c: 0,
            }
        }

        fn reset(&mut self) {
            self.pixels = [[0; X]; Y];
            self.prev_pixels = [[0; X]; Y];
            self.c = 0;
        }

        fn update(&mut self) {
            if self.c == self.slf {
                self.c = 0;
                self.prev_pixels = self.pixels;
                for i in (1..Y - 1).rev() {
                    for j in 1..X - 1 {
                        let mut n = 0;

                        let decay = self.rng.rand_range(4..5) as u8;
                        if self.pixels[i - 1][j] > decay {
                            n = self.pixels[i - 1][j] - decay;
                        }
                        if self.rng.rand_range(0..200) == 0 {
                            n += 4;
                        }
                        self.pixels[i][j] = n;
                    }
                }

                for j in 1..X - 1 {
                    let range = ((Fire::FIRE_PALETTE.len() - 30) as u32)
                        ..((Fire::FIRE_PALETTE.len() - 1) as u32);
                    self.pixels[0][j] = self.rng.rand_range(range) as u8;
                }
            } else {
                self.c += 1;
            }
        }
    }

    impl<const X: usize, const Y: usize> ColorMap for FireSim<X, Y> {
        fn get(&self, x: usize, y: usize) -> RGB8 {
            let (y, x) = (Y - y - 2, x + 1);
            let (p1, p2) = (self.pixels[y][x] as i16, self.prev_pixels[y][x] as i16);
            let r = (self.c as i16) * (p1 - p2) / (self.slf as i16);
            let mut p = self.prev_pixels[y][x] as i16 + r;
            if p < 0 {
                p = 0;
            };
            let k = if p < (Fire::FIRE_PALETTE.len() as i16) {
                p as u8
            } else {
                (Fire::FIRE_PALETTE.len() - 1) as u8
            };
            Fire::FIRE_PALETTE[k as usize]
        }
    }

    const FREQ: u32 = 72_000_000;
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    type IrPin = PB0<Input<Floating>>;
    pub type IrReceiver = Receiver<
        Nec,
        IrPin,
        dwt_systick_monotonic::fugit::Instant<u32, 1, FREQ>,
        Button<Mp3Remote>,
    >;

    const DIGIT_LEN: usize = (5 * 4) + 5 + 3;
    const DIGIT_WIDTH: usize = 6;
    const DIGIT_HEIGHT: usize = 9;
    const NUM_DIGITS: usize = 6;
    const COLON_LEN: usize = 4;
    const NUM_SEGS: usize = 7;
    const BUFF_SIZE: usize = 512;
    const NUM_LEDS: usize = (DIGIT_LEN * 6) + (2 * COLON_LEN);
    const LEDS_BUF_SIZE: usize = 2 * 2048;
    const LED_FADE_MAX: u8 = 250;
    const LED_FADE_SLOPE: usize = 10;
    const LED_FADE_SPEED: u32 = 5;
    const UTC_OFFSET: Duration = Duration::HOUR;
    const NIGHT_RANGE: RangeInclusive<u8> = 7..=21;
    const SUNRISE_LEN: Duration = Duration::minutes(30);
    const ALARM_LEN: Duration = Duration::minutes(3);
    const BUZZER_BASE_FREQ_HZ: u32 = 6000;
    const DEFAULT_LED_COLOR: RGB8 = RGB8::new(255, 0, 0);
    const DEFAULT_DAY_MODE: LightMode = LightMode::SOLID_COLOR {
        color: RGB8::new(255, 0, 0),
    };

    const EEPROM_PARAMS: Params = Params {
        first_page: 126,
        flash_size: FlashSize::Sz128K,
        page_size: SectorSize::Sz1K,
        page_count: 2,
    };

    const LED_POS_TO_XY: [(u8, u8); DIGIT_LEN] = [
        (4, 4),
        (3, 4),
        (2, 4),
        (1, 4),
        (0, 4),
        (0, 3),
        (0, 2),
        (0, 1),
        (1, 0),
        (2, 0),
        (3, 0),
        (4, 0),
        (5, 0),
        (5, 1),
        (5, 2),
        (5, 3),
        (5, 4),
        (5, 5),
        (5, 6),
        (5, 7),
        (5, 8),
        (4, 8),
        (3, 8),
        (2, 8),
        (1, 8),
        (0, 7),
        (0, 6),
        (0, 5),
    ];

    const COLON_POS_TO_XY: [(u8, u8); 2 * COLON_LEN] = [
        (13, 2),
        (14, 2),
        (13, 6),
        (14, 6),
        (23, 2),
        (24, 2),
        (23, 6),
        (24, 6),
    ];

    const SYMBOL_TO_SEGS: [u8; 10] = [
        0b1111110, 0b0011000, 0b1101101, 0b0111101, 0b0011011, 0b0110111, 0b1110111, 0b0011100,
        0b1111111, 0b0111111,
    ];

    const SEG_TO_OFFSET: [(usize, usize); NUM_SEGS] =
        [(0, 4), (4, 4), (8, 4), (12, 4), (16, 5), (21, 4), (25, 3)];

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

    #[derive(Debug, PartialEq, Clone)]
    #[allow(non_camel_case_types)]
    pub enum LightMode {
        SOLID_COLOR { color: RGB8 },
        GRADIENT,
        FIRE,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum LightEvent {
        DIGITS {
            dg: Option<[char; NUM_DIGITS]>,
            col1: bool,
            col2: bool,
            mode: LightMode,
        },
        TIMEOUT,
    }

    #[derive(PartialEq)]
    #[allow(non_camel_case_types)]
    pub enum OperatingMode {
        DAY { mode: LightMode },
        SUNRISE { ramp: LinearRamp },
        ALARM,
    }

    #[derive(Debug, PartialEq, Clone, Copy)]
    pub enum LightTransition {
        InProgress { brightness: i16, slope: i16 },
        Finished { state: bool },
    }

    #[shared]
    struct Shared {
        datetime: Option<PrimitiveDateTime>,
        gps_locked: bool,
        pps_timeout: Option<pps_timeout::SpawnHandle>,
        eeprom: EEPROM<&'static mut stm32f1xx_hal::flash::Parts>,
    }

    type Leds = TxDma<
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
        stm32f1xx_hal::dma::dma1::C5,
    >;

    type Xfer = Transfer<stm32f1xx_hal::dma::R, &'static [u8; LEDS_BUF_SIZE], Leds>;
    type Fire = FireSim<{ (DIGIT_WIDTH * NUM_DIGITS) + 2 + 2 + 2 }, { DIGIT_HEIGHT + 2 }>;

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        pps: PA1<Input<PullDown>>,
        rx: Rx2,
        parser: Parser,
        ws: Option<Leds>,
        prod: bbqueue::Producer<'static, BUFF_SIZE>,
        cons: bbqueue::Consumer<'static, BUFF_SIZE>,
        dst: Duration,
        buzz_pwm:
            PwmHz<TIM3, Tim3NoRemap, Ch<0>, stm32f1xx_hal::gpio::gpioa::PA6<Alternate<PushPull>>>,
        wdg: IndependentWatchdog,
        alarm_enabled: bool,
        alarm_time: Time,
        leds: [RGB8; NUM_LEDS],
        receiver: IrReceiver,
        fire: Fire,
        gradient: Gradient,
        solid: SolidColor,
        led_pwm:
            PwmHz<TIM1, Tim1NoRemap, Ch<0>, stm32f1xx_hal::gpio::gpioa::PA8<Alternate<PushPull>>>,
        decoder: &'static mut lpc_seq_decoder_t,
        voice_rng: Rand32,
    }

    #[init(local = [flash: Option<Parts> = None,
                    BB: BBBuffer<BUFF_SIZE> = BBBuffer::new()])]
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
            .use_hse(8.MHz())
            .sysclk(FREQ.Hz())
            .pclk1(36.MHz())
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
        pps.enable_interrupt(&c.device.EXTI);
        pps.trigger_on_edge(&c.device.EXTI, Edge::Rising);

        let mut gpiob = c.device.GPIOB.split();
        let pins = (
            NoSck,
            NoMiso,
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
        );
        let mode: Mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::spi2(c.device.SPI2, pins, mode, 3800.kHz(), clocks);
        let dma = c.device.DMA1.split();
        let ws = spi.with_tx_dma(dma.5);

        let leds = [RGB8::default(); NUM_LEDS];

        let mut dcb = c.core.DCB;
        let dwt = c.core.DWT;
        let systick = c.core.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().raw());

        let parser = Parser::new().sentence_only(Sentence::RMC);
        let (prod, cons) = c.local.BB.try_split().unwrap();

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
        let mut buzz_pwm = Timer::new(c.device.TIM3, &clocks).pwm_hz::<Tim3NoRemap, _, _>(
            buzz_pin,
            &mut afio.mapr,
            BUZZER_BASE_FREQ_HZ.Hz(),
        );
        buzz_pwm.set_duty(Channel::C1, 20 * (buzz_pwm.get_max_duty() / 100));

        let led_pin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        let mut led_pwm = Timer::new(c.device.TIM1, &clocks).pwm_hz::<Tim1NoRemap, _, _>(
            led_pin,
            &mut afio.mapr,
            250.Hz(),
        );
        led_pwm.set_duty(Channel::C1, 0);
        led_pwm.enable(Channel::C1);

        let mut pin = gpiob.pb0.into_floating_input(&mut gpiob.crl);
        pin.make_interrupt_source(&mut afio);
        pin.enable_interrupt(&c.device.EXTI);
        pin.trigger_on_edge(&c.device.EXTI, Edge::RisingFalling);

        let receiver = Receiver::with_fugit(pin);

        let wdg = IndependentWatchdog::new(c.device.IWDG);
        main_sm::spawn(MainEvent::Timeout).unwrap();
        buzzer::spawn(BuzzerEvent::QuarterChime).unwrap();

        let fire = FireSim::new(666, 15);
        let gradient = Gradient::new();
        let solid = SolidColor::new(DEFAULT_LED_COLOR);

        led_pwm::spawn().unwrap();

        let decoder = unsafe { lpc_seq_decoder_new() };
        let voice_rng = Rand32::new(664);

        (
            Shared {
                datetime: None,
                gps_locked: false,
                pps_timeout: None,
                eeprom,
            },
            Local {
                led,
                pps,
                rx,
                parser,
                ws: Some(ws),
                prod,
                cons,
                dst,
                buzz_pwm,
                wdg,
                alarm_enabled,
                alarm_time,
                leds,
                receiver,
                fire,
                gradient,
                solid,
                led_pwm,
                decoder: unsafe { &mut *decoder },
                voice_rng,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = USART2, shared = [], local = [rx, prod], priority = 16)]
    fn usart2(cx: usart2::Context) {
        let rx = cx.local.rx;
        let prod = cx.local.prod;
        if rx.is_rx_not_empty() {
            match nb::block!(rx.read()) {
                Ok(b) => {
                    let mut wgr = prod.grant_exact(1).unwrap();
                    wgr[0] = b;
                    wgr.commit(1);
                    if b == b'\n' {
                        parse::spawn().ok();
                    }
                }
                Err(e) => {
                    defmt::error!("usart error!!! {}", SerialErrorWrapper(e));
                }
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
                    -Duration::HOUR
                } else {
                    SM = false;
                    Duration::ZERO
                }
            }
        } else if (dt.month() == Month::March)
            && (dt.day() + 7 > 31)
            && (dt.weekday() == Weekday::Sunday)
            && (tm == time!(2:00:00))
        {
            Duration::HOUR
        } else {
            Duration::ZERO
        }
    }

    #[task(local = [led_pwm, i: usize = 0, dir: bool = false], priority = 16, capacity = 1)]
    fn led_pwm(cx: led_pwm::Context) {
        if *cx.local.dir {
            *cx.local.i -= 1;
            if *cx.local.i == 28 {
                *cx.local.dir = false;
            }
        } else {
            *cx.local.i += 1;
            if *cx.local.i == 128 {
                *cx.local.dir = true;
            }
        }

        let s = (GAMMA8[*cx.local.i] as u32) * (cx.local.led_pwm.get_max_duty() as u32) / 255;
        cx.local
            .led_pwm
            .set_duty(Channel::C1, s.try_into().unwrap());
        led_pwm::spawn_after(50.millis()).unwrap();
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

        if let Ok(rgr) = cons.read() {
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
                            day,
                        )
                        .unwrap();
                        let time = Time::from_hms(hours, minutes, seconds as u8).unwrap();
                        let date_time = PrimitiveDateTime::new(date, time);
                        datetime.lock(|dt| match *dt {
                            Some(_dt) => {
                                if date_time != _dt {
                                    defmt::warn!(
                                        "Unexpected date_time: {} != {}",
                                        Debug2Format(&date_time),
                                        Debug2Format(&_dt)
                                    );
                                    *dt = Some(date_time);
                                }
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
                        defmt::warn!("Incomplete NMEA")
                    }
                    Err(e) => {
                        defmt::warn!("NMEA parsing error: {}", e)
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

    fn char_to_segs(c: char) -> u8 {
        match c {
            '0'..='9' => SYMBOL_TO_SEGS[(c as usize) - ('0' as usize)],
            ' ' => 0b0000000,
            '-' => 0b0000001,
            'A' => 0b1011111,
            'b' => 0b1110011,
            'c' => 0b1100001,
            'C' => 0b1100110,
            'd' => 0b1111001,
            'E' => 0b1100111,
            'F' => 0b1000111,
            'G' => 0b1110110,
            'h' => 0b1010011,
            'I' => 0b0011000,
            'J' => 0b1111000,
            'K' => 0b1011011,
            'L' => 0b1100010,
            'm' => 0b1010100,
            'n' => 0b1010001,
            'N' => 0b1011110,
            'o' => 0b1110001,
            'P' => 0b1001111,
            'q' => 0b0011111,
            'r' => 0b1000001,
            'S' => 0b0110111,
            't' => 0b1100011,
            'U' => 0b1111010,
            'w' => 0b0101010,
            'X' => 0b1011011,
            'y' => 0b0111011,
            'Z' => 0b1101101,
            '?' | _ => 0b0010111,
        }
    }

    fn update_transition(t: LightTransition) -> LightTransition {
        match t {
            LightTransition::InProgress { brightness, slope } => {
                let nv: u8 = (brightness + slope).try_into().unwrap();
                if nv == 0 || nv == LED_FADE_MAX {
                    LightTransition::Finished {
                        state: nv == LED_FADE_MAX,
                    }
                } else {
                    LightTransition::InProgress {
                        brightness: nv.try_into().unwrap(),
                        slope,
                    }
                }
            }
            LightTransition::Finished { state: _ } => t,
        }
    }

    fn transition_to_brightness(t: LightTransition) -> u8 {
        match t {
            LightTransition::InProgress {
                brightness,
                slope: _,
            } => brightness.try_into().unwrap(),
            LightTransition::Finished { state } => {
                if state {
                    LED_FADE_MAX
                } else {
                    0
                }
            }
        }
    }

    fn leds_write_byte(data: &mut [u8], mut byte: u8) {
        const PATTERNS: [u8; 4] = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];
        for d in data.iter_mut().take(4) {
            let bits = (byte & 0b1100_0000) >> 6;
            *d = PATTERNS[bits as usize];
            byte <<= 2;
        }
    }

    fn leds_write_bytes(data: &mut [u8; LEDS_BUF_SIZE], leds: &[RGB8; NUM_LEDS]) {
        for (i, rgb) in leds.iter().enumerate() {
            let j = 12 * i;
            leds_write_byte(&mut data[j..j + 4], rgb.g);
            leds_write_byte(&mut data[j + 4..j + 8], rgb.r);
            leds_write_byte(&mut data[j + 8..j + 12], rgb.b);
        }
    }

    fn leds_write(
        data: &'static mut [u8; LEDS_BUF_SIZE],
        leds: &[RGB8; NUM_LEDS],
        xs: &mut Option<Xfer>,
        ws: &mut Option<Leds>,
    ) {
        leds_write_bytes(data, leds);
        if let Some(xfer) = xs.take() {
            let (_, spi_dma) = xfer.wait();
            *xs = Some(spi_dma.write(data));
        } else if let Some(spi_dma) = ws.take() {
            *xs = Some(spi_dma.write(data));
        } else {
            defmt::panic!();
        }
    }

    #[task(local = [ws, leds, is_trans: bool = false,
                    fire,
                    gradient,
                    solid,
                    mode: LightMode = DEFAULT_DAY_MODE,
                    xfer: Option<Xfer> = None,
                    prev_dg: [char; NUM_DIGITS] = [' '; NUM_DIGITS],
                    trans_state: [[LightTransition; NUM_SEGS]; NUM_DIGITS] = [[LightTransition::Finished {state: false}; NUM_SEGS]; NUM_DIGITS],
                    handle: Option<light::SpawnHandle> = None],
                        priority = 8, capacity = 3)]
    fn light(cx: light::Context, ev: LightEvent) {
        static mut LEDS_BUF: [u8; LEDS_BUF_SIZE] = [0; LEDS_BUF_SIZE];
        let ws = cx.local.ws;
        let leds = cx.local.leds;
        let is_trans = cx.local.is_trans;
        let prev_dg = cx.local.prev_dg;
        let trans_state = cx.local.trans_state;
        let handle = cx.local.handle;
        let fire = cx.local.fire;

        match ev {
            LightEvent::DIGITS {
                dg,
                col1,
                col2,
                mode,
            } => {
                // if *is_trans {
                //     if let Some(handle) = handle.take() {
                //         handle.cancel().ok();
                //     }
                // }
                if let Some(dg) = dg {
                    for (i, (d, pd)) in dg.iter().zip((*prev_dg).iter()).enumerate() {
                        let (sd, psd) = (char_to_segs(*d), char_to_segs(*pd));
                        for j in 0..NUM_SEGS {
                            if (sd & (1 << j)) != (psd & (1 << j)) {
                                if (sd & (1 << j)) != 0 {
                                    trans_state[i][j] = LightTransition::InProgress {
                                        brightness: 0,
                                        slope: LED_FADE_SLOPE.try_into().unwrap(),
                                    };
                                } else {
                                    trans_state[i][j] = LightTransition::InProgress {
                                        brightness: LED_FADE_MAX.try_into().unwrap(),
                                        slope: -TryInto::<i16>::try_into(LED_FADE_SLOPE).unwrap(),
                                    };
                                }
                            } else {
                                trans_state[i][j] = LightTransition::Finished {
                                    state: (sd & (1 << j)) != 0,
                                }
                            }
                        }
                    }
                    *cx.local.mode = mode;
                    *prev_dg = dg;
                    defmt::info!("Transition started");
                    if !*is_trans {
                        *is_trans = true;
                        *handle =
                            light::spawn_after(LED_FADE_SPEED.millis(), LightEvent::TIMEOUT).ok();
                    }
                }

                let cmap: &dyn ColorMap = match *cx.local.mode {
                    LightMode::GRADIENT => {
                        fire.reset();
                        cx.local.gradient
                    }
                    LightMode::SOLID_COLOR { color } => {
                        fire.reset();
                        *cx.local.solid = SolidColor::new(color);
                        cx.local.solid
                    }
                    LightMode::FIRE => fire,
                };

                if col1 {
                    leds_draw_colon(leds, 0, LED_FADE_MAX, cmap);
                } else {
                    leds_draw_colon(leds, 0, 0, cmap);
                }
                if col2 {
                    leds_draw_colon(leds, 1, LED_FADE_MAX, cmap);
                } else {
                    leds_draw_colon(leds, 1, 0, cmap);
                }
                unsafe {
                    leds_write(&mut LEDS_BUF, leds, cx.local.xfer, ws);
                }
            }
            LightEvent::TIMEOUT => {
                if *cx.local.mode == LightMode::FIRE {
                    fire.update();
                }

                defmt::assert!(*is_trans);
                for d in (*trans_state).iter_mut() {
                    for s in d.iter_mut() {
                        *s = update_transition(*s);
                    }
                }

                let cmap: &dyn ColorMap = match *cx.local.mode {
                    LightMode::GRADIENT => cx.local.gradient,
                    LightMode::SOLID_COLOR { .. } => cx.local.solid,
                    LightMode::FIRE => fire,
                };

                for (x, d) in (*trans_state).iter().rev().enumerate() {
                    let segs: [u8; NUM_SEGS] = d.map(transition_to_brightness);
                    leds_draw_digit(leds, x, segs, cmap);
                }
                unsafe {
                    leds_write(&mut LEDS_BUF, leds, cx.local.xfer, ws);
                }
                *handle = light::spawn_after(LED_FADE_SPEED.millis(), LightEvent::TIMEOUT).ok();
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
        Say,
    }

    #[derive(Debug, PartialEq, Format)]
    pub enum BuzzerEvent {
        HourChime,
        QuarterChime,
        SecondTick,
        Next,
        Alarm { i: Option<usize> },
        Say { h: u8, m: u8 },
    }

    const ALARMS: &[&[(u16, u16)]] = &[
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

    fn get_seq_from_time(h: u8, m: u8) -> Option<Vec<*const lpc_seq_t, 5>> {
        let mut v: Vec<*const lpc_seq_t, 5> = Vec::new();
        unsafe {
            v.push(lpc_get_seq(lpc_seq_e_LPC_JEST_GODZINA)).ok()?;
            v.push(lpc_get_seq(if h < 21 { h.into() } else { 20 }))
                .ok()?;
            if h > 20 {
                v.push(lpc_get_seq((h - 20).into())).ok()?;
            }
            if m < 21 {
                if m == 0 {
                    v.push(lpc_get_seq(lpc_seq_e_LPC_ZERO)).ok()?;
                }
                v.push(lpc_get_seq((lpc_seq_e_LPC_ZERO + (m as u32)).into()))
                    .ok()?;
            } else {
                v.push(lpc_get_seq(
                    (lpc_seq_e_LPC_OSIEMNASCIE + ((m / 10) as u32)).into(),
                ))
                .ok()?;
                if (m % 10) > 0 {
                    v.push(lpc_get_seq((lpc_seq_e_LPC_ZERO + ((m % 10) as u32)).into()))
                        .ok()?;
                }
            }
        }
        Some(v)
    }

    #[task(local = [buzz_pwm, decoder, voice_rng,
                    y_min: fix16_t = 0,
                    y_max: fix16_t = 0,
                    state: BuzzerState = BuzzerState::Chime {is_on: false, chime: None},
                    handle: Option<buzzer::SpawnHandle> = None],
           shared = [],
           priority = 16,
           capacity = 2)]
    fn buzzer(cx: buzzer::Context, ev: BuzzerEvent) {
        const HOUR_CHIME: [u32; 3] = [100, 80, 50];
        const QUARTER_CHIME: [u32; 1] = [50];
        const SECOND_TICK: [u32; 1] = [1];

        let next_run = monotonics::now() + 90.micros();

        let buzzer = cx.local.buzz_pwm;
        let s = cx.local.state;
        let handle = cx.local.handle;
        let decoder = *cx.local.decoder as *mut lpc_seq_decoder_t;
        let rng = cx.local.voice_rng;

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
                        buzzer.set_period(BUZZER_BASE_FREQ_HZ.Hz());
                        buzzer.enable(Channel::C1);
                        buzzer::spawn(BuzzerEvent::Next).unwrap();
                    }
                }
                BuzzerEvent::Say { h, m } => {
                    if let Some(seq) = get_seq_from_time(h, m) {
                        unsafe {
                            lpc_seq_decoder_update(
                                decoder,
                                seq.as_slice().as_ptr(),
                                seq.len() as u32,
                            );
                        }

                        *s = BuzzerState::Say;
                        buzzer.set_period(32000.Hz());
                        buzzer.enable(Channel::C1);
                        if let Some(handle) = handle.take() {
                            handle.cancel().ok();
                        }
                        buzzer::spawn(BuzzerEvent::Next).unwrap();
                    }
                }
                _ => {
                    match ev {
                        BuzzerEvent::HourChime => *chime = Some(HOUR_CHIME.iter()),
                        BuzzerEvent::QuarterChime => *chime = Some(QUARTER_CHIME.iter()),
                        BuzzerEvent::SecondTick => *chime = Some(SECOND_TICK.iter()),
                        BuzzerEvent::Next => (),
                        BuzzerEvent::Alarm { .. } | BuzzerEvent::Say { .. } => {
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
                                    buzzer.set_period(BUZZER_BASE_FREQ_HZ.Hz());
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
                                    buzzer.set_period((*freq as u32).Hz());
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
            BuzzerState::Say => match ev {
                BuzzerEvent::Next => {
                    let mut y = 0;
                    let finished = unsafe {
                        lpc_seq_decoder_exec(decoder, rng.rand_u32(), &mut y as *mut fix16_t)
                    };
                    if finished {
                        defmt::error!(
                            "{} - {} {}",
                            *cx.local.y_min,
                            *cx.local.y_max,
                            buzzer.get_max_duty()
                        );
                        buzzer.disable(Channel::C1);
                        *s = BuzzerState::Chime {
                            is_on: false,
                            chime: None,
                        };
                    } else {
                        buzzer.set_duty(Channel::C1, ((y + 5000000) / 2300) as u16);
                        *handle = buzzer::spawn_at(next_run, BuzzerEvent::Next).ok();
                    }
                }
                _ => {}
            },
        }
    }

    fn dim_color(c: RGB8, b: u8) -> RGB8 {
        c.iter()
            .map(|v| {
                (((v as usize) * (b as usize)) / LED_FADE_MAX as usize)
                    .try_into()
                    .unwrap()
            })
            .collect()
    }

    fn leds_draw_digit(
        leds: &mut [RGB8; NUM_LEDS],
        x: usize,
        segs: [u8; NUM_SEGS],
        cmap: &dyn ColorMap,
    ) {
        let dx = x * DIGIT_LEN + (COLON_LEN * (x / 2));
        let ddx = NUM_DIGITS * x;
        for (i, s) in segs.iter().enumerate() {
            let (o, l) = SEG_TO_OFFSET[i];
            for (i, l) in leds.iter_mut().skip(dx + o).take(l).enumerate() {
                let (x, y) = LED_POS_TO_XY[o + i];
                let c = cmap.get(ddx + x as usize, y as usize);
                *l = dim_color(c, *s);
            }
        }
    }

    fn leds_draw_colon(leds: &mut [RGB8; NUM_LEDS], x: usize, b: u8, cmap: &dyn ColorMap) {
        let offset = 2 * DIGIT_LEN + ((2 * DIGIT_LEN) + COLON_LEN) * x;
        for (i, l) in leds.iter_mut().skip(offset).take(COLON_LEN).enumerate() {
            let (x, y) = COLON_POS_TO_XY[(x * COLON_LEN) + i];
            let c = cmap.get(x as usize, y as usize);
            *l = dim_color(c, b);
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

    #[task(binds = EXTI0, shared = [],
        local = [receiver], priority = 16)]
    fn ir_isr(cx: ir_isr::Context) {
        let now = monotonics::now();
        let receiver = cx.local.receiver;
        if receiver.pin_mut().check_interrupt() {
            if let Ok(Some(cmd)) = receiver.event_instant(now) {
                if let Some(action) = cmd.action() {
                    match action {
                        One | Two | Three | Four | Five | Six | Seven | Eight | Nine | Zero => {
                            if !cmd.is_repeat() {
                                main_sm::spawn(MainEvent::Remote { btn: action }).unwrap();
                            }
                        }
                        _ => {
                            main_sm::spawn(MainEvent::Remote { btn: action }).unwrap();
                        }
                    }
                }
            }

            receiver.pin_mut().clear_interrupt_pending_bit();
        }
    }
    pub enum MainState {
        NoSync { s: bool },
        TimeDisplay,
        AlarmSetting,
    }

    #[derive(Debug)]
    pub enum MainEvent {
        Timeout,
        PPSTick {
            dt: Option<PrimitiveDateTime>,
            gps_lock: bool,
        },
        Remote {
            btn: Action,
        },
    }

    fn pps_tick_handler(
        dt: Option<PrimitiveDateTime>,
        mode: &mut OperatingMode,
        alarm_dur: &mut Duration,
    ) {
        if let Some(dt) = dt {
            let s = dt.time().second();
            if NIGHT_RANGE.contains(&dt.time().hour()) {
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

            if *mode == OperatingMode::ALARM {
                *alarm_dur += Duration::SECOND;
                if *alarm_dur == ALARM_LEN {
                    buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                }
            }
        }
    }

    fn disp_time(time: Time, mode: &mut OperatingMode, alarm_dur: &mut Duration) {
        let (h, m, s) = time.as_hms();
        let mut buf: String<NUM_DIGITS> = String::new();
        buf.write_fmt(format_args!("{:02}{:02}{:02}", h, m, s))
            .unwrap();
        let mut dg = str_to_array(&buf);
        if dg[0] == '0' {
            dg[0] = ' ';
        }

        let lmode = match mode {
            OperatingMode::DAY { mode } => mode.clone(),
            OperatingMode::SUNRISE { ramp } => {
                if let Some(y) = ramp.next() {
                    let rgb =
                        &SUNRISE_CMAP[TryInto::<usize>::try_into(y).unwrap()].to_be_bytes()[1..4];
                    LightMode::SOLID_COLOR {
                        color: RGB8::new(rgb[0], rgb[1], rgb[2]) / 4,
                    }
                } else {
                    buzzer::spawn(BuzzerEvent::Alarm { i: Some(0) }).unwrap();
                    *mode = OperatingMode::ALARM;
                    *alarm_dur = Duration::ZERO;
                    LightMode::GRADIENT
                }
            }
            _ => LightMode::GRADIENT,
        };

        light::spawn(LightEvent::DIGITS {
            dg: Some(dg),
            col1: true,
            col2: true,
            mode: lmode,
        })
        .unwrap();
    }

    fn str_to_array<const N: usize>(str: &str) -> [char; N] {
        str.chars().collect::<Vec<_, N>>().into_array().unwrap()
    }

    #[task(shared = [eeprom], local = [
                                 curr_al: usize = 0,
                                 state: MainState = MainState::NoSync {s: true},
                                 handle: Option<main_sm::SpawnHandle> = None,
                                 mode: OperatingMode = OperatingMode::DAY {mode: DEFAULT_DAY_MODE},
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
            MainState::NoSync { s } => match ev {
                MainEvent::Timeout => {
                    if *s {
                        light::spawn(LightEvent::DIGITS {
                            dg: Some(str_to_array("noSyNC")),
                            col1: false,
                            col2: false,
                            mode: DEFAULT_DAY_MODE,
                        })
                        .unwrap();
                        *state = MainState::NoSync { s: false };
                    } else {
                        light::spawn(LightEvent::DIGITS {
                            dg: Some([' '; NUM_DIGITS]),
                            col1: false,
                            col2: false,
                            mode: DEFAULT_DAY_MODE,
                        })
                        .unwrap();
                        *state = MainState::NoSync { s: true };
                    }
                    *cx.local.handle = main_sm::spawn_after(500.millis(), MainEvent::Timeout).ok();
                }
                MainEvent::PPSTick { dt, gps_lock: _ } => {
                    if dt.is_some() {
                        if let Some(handle) = cx.local.handle.take() {
                            handle.cancel().ok();
                        }
                        *state = MainState::TimeDisplay;
                        main_sm::spawn(ev).unwrap();
                    }
                }
                _ => (),
            },
            MainState::TimeDisplay => {
                match ev {
                    MainEvent::PPSTick { dt, gps_lock: _ } => {
                        pps_tick_handler(dt, curr_mode, cx.local.alarm_dur);
                        *datetime = dt;
                        if let Some(dt) = *datetime {
                            if *cx.local.alarm_enabled
                                && ![Weekday::Saturday, Weekday::Sunday]
                                    .contains(&dt.date().weekday())
                                && dt.time() == (*alarm_time - SUNRISE_LEN)
                            {
                                defmt::info!("Starting sunrise");
                                *curr_mode = OperatingMode::SUNRISE {
                                    ramp: LinearRamp::new(
                                        SUNRISE_LEN.whole_seconds().try_into().unwrap(),
                                        SUNRISE_CMAP.len(),
                                    ),
                                };
                            }

                            disp_time(dt.time(), curr_mode, cx.local.alarm_dur);
                            *cx.local.handle =
                                main_sm::spawn_after(500.millis(), MainEvent::Timeout).ok();
                        }
                    }
                    MainEvent::Timeout => {
                        light::spawn(LightEvent::DIGITS {
                            dg: None,
                            col1: false,
                            col2: false,
                            mode: DEFAULT_DAY_MODE,
                        })
                        .unwrap();
                    }
                    MainEvent::Remote { btn } => match btn {
                        One => {
                            if let Some(handle) = cx.local.handle.take() {
                                handle.cancel().ok();
                            }
                            disp_time(*alarm_time, curr_mode, cx.local.alarm_dur);
                            buzzer::spawn(BuzzerEvent::Alarm {
                                i: Some(*cx.local.curr_al),
                            })
                            .unwrap();
                            *state = MainState::AlarmSetting;
                        }
                        Two => {
                            *cx.local.alarm_enabled ^= true;
                            if *cx.local.alarm_enabled {
                                buzzer::spawn(BuzzerEvent::HourChime).unwrap();
                            } else {
                                buzzer::spawn(BuzzerEvent::QuarterChime).unwrap();
                            }
                            let mut eeprom = cx.shared.eeprom;
                            eeprom.lock(|eeprom| {
                                eeprom.write(1, *cx.local.alarm_enabled as u16).unwrap();
                            });
                        }
                        Four => {
                            *curr_mode = OperatingMode::DAY {
                                mode: DEFAULT_DAY_MODE,
                            };
                            if let Some(dt) = *datetime {
                                disp_time(dt.time(), curr_mode, cx.local.alarm_dur);
                            }
                        }
                        Five => {
                            *curr_mode = OperatingMode::DAY {
                                mode: LightMode::FIRE,
                            };
                            if let Some(dt) = *datetime {
                                disp_time(dt.time(), curr_mode, cx.local.alarm_dur);
                            }
                        }
                        Six => {
                            *curr_mode = OperatingMode::DAY {
                                mode: LightMode::SOLID_COLOR {
                                    color: DEFAULT_LED_COLOR
                                        .iter()
                                        .map(|v| ((v as usize) * (GAMMA8[60] as usize) / 255) as u8)
                                        .collect(),
                                },
                            };
                            if let Some(dt) = *datetime {
                                disp_time(dt.time(), curr_mode, cx.local.alarm_dur);
                            }
                        }
                        Nine => {
                            if let Some(dt) = *datetime {
                                let (h, m, _) = dt.as_hms();
                                buzzer::spawn(BuzzerEvent::Say { h: h, m: m }).unwrap();
                            }
                        }
                        _ => {
                            defmt::error!("Unh");
                        }
                    },
                }
                if (matches!(*curr_mode, OperatingMode::SUNRISE { .. })
                    || matches!(*curr_mode, OperatingMode::ALARM))
                    && matches!(ev, MainEvent::Remote { .. })
                {
                    *curr_mode = OperatingMode::DAY {
                        mode: DEFAULT_DAY_MODE,
                    };
                    buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                }
            }
            MainState::AlarmSetting => match ev {
                MainEvent::PPSTick { dt: _, gps_lock: _ } => {}
                MainEvent::Remote { btn } => match btn {
                    One => {
                        let (h, m, _) = alarm_time.as_hms();
                        let hm: u16 = ((h as u16) << 8) | (m as u16);

                        let mut eeprom = cx.shared.eeprom;
                        eeprom.lock(|eeprom| {
                            eeprom.write(2, hm).unwrap();
                        });
                        buzzer::spawn(BuzzerEvent::Alarm { i: None }).unwrap();
                        *state = MainState::TimeDisplay;
                        if let Some(dt) = *datetime {
                            disp_time(dt.time(), curr_mode, cx.local.alarm_dur);
                        }
                    }
                    Next => {
                        *cx.local.curr_al += 1;
                        if *cx.local.curr_al >= ALARMS.len() {
                            *cx.local.curr_al = 0;
                        }
                        buzzer::spawn(BuzzerEvent::Alarm {
                            i: Some(*cx.local.curr_al),
                        })
                        .unwrap();
                    }
                    Prev => {
                        if *cx.local.curr_al == 0 {
                            *cx.local.curr_al = ALARMS.len() - 1;
                        } else {
                            *cx.local.curr_al -= 1;
                        }
                        buzzer::spawn(BuzzerEvent::Alarm {
                            i: Some(*cx.local.curr_al),
                        })
                        .unwrap();
                    }
                    ChannelListPrev => {
                        *alarm_time -= Duration::minutes(5);
                        disp_time(*alarm_time, curr_mode, cx.local.alarm_dur);
                    }
                    ChannelListNext => {
                        *alarm_time += Duration::minutes(5);
                        disp_time(*alarm_time, curr_mode, cx.local.alarm_dur);
                    }
                    _ => {
                        defmt::error!("Unh");
                    }
                },
                _ => (),
            },
        }
    }

    #[idle(local = [led, wdg])]
    fn idle(cx: idle::Context) -> ! {
        cx.local.wdg.start(4000.millis());

        loop {
            cx.local.led.set_high();
            rtic::export::wfi();
            cx.local.led.set_low();
            cx.local.wdg.feed();
        }
    }
}
