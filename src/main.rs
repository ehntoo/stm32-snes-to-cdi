/*
 * SNES to CD-i Controller Adapter
 *
 * Use any old SNES controller as a joypad on your CD-i console using just an STM32F103 "blue pill"
 * development board.
 *
 * This could stand to be updated to reflect newer Rust practices, having been written in 2018
 * before stabilization of async, and before a whole lot of API development for embedded crates.
 *
 * The method to achieve low-idle serial signalling is... hackish, using an interrupt to invert the
 * output of a hardware UART peripheral, but the UART in the STM32F103 does not have a negated
 * signalling mode and this avoids use of any additional hardware.
 *
 * Hook up your SNES controller as follows:
 * SNES Controller:
 * ----------------------------------------------------------------------------
 * ASCII Art borrowed from
 * https://steckschwein.de/2020/01/15/connecting-snes-controller-to-the-steckschwein/
 *
 *  /---------------------
 * | 7  6  5 | 4  3  2  1 |
 *  \---------------------
 * 
 * Pin Description
 * 1   +5V
 * 2  CLK
 * 3  LATCH
 * 4  DATA
 * 5  –
 * 6  –
 * 7  GND
 *
 * STM32 connections:
 * PB3 - Clock
 * PB4 - Data
 * PB5 - Latch
 * Ground
 * +5v to Controller
 *
 * CD-i
 * ----------------------------------------------------------------------------
 * http://www.icdia.co.uk/cdprosupport/hardware/cables/cdi_ser.htm
 * https://cdii.blogspot.com/2019/08/the-8-pin-din-on-cd-i-player-is.html
 * CD-i      STM32
 * 2/RX   -  PB6
 * 5/GND  -  Ground
 * 7/RTS  -  PB7
 * 8/+5V  -  5V supply
 */
#![no_std]
#![no_main]

extern crate panic_halt;
extern crate stm32f1;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use stm32f1::interrupt;
use stm32f1::stm32f103;

#[macro_use]
extern crate bitflags;
bitflags! {
    struct SnesState: u16 {
        const B      = 0x001;
        const Y      = 0x002;
        const Select = 0x004;
        const Start  = 0x008;
        const Up     = 0x010;
        const Down   = 0x020;
        const Left   = 0x040;
        const Right  = 0x080;
        const A      = 0x100;
        const X      = 0x200;
        const L_B    = 0x400;
        const R_B    = 0x800;
    }
}

const CONTROLLER_TYPE_ID: u16 = 0x4a;

#[entry]
fn main() -> ! {
    let cp = stm32f103::CorePeripherals::take().unwrap();
    let p = stm32f103::Peripherals::take().unwrap();
    let mut syst = cp.SYST;
    let mut nvic = cp.NVIC;
    let mut uart = p.USART1;
    let mut spi = p.SPI1;
    let gpio_a = p.GPIOA;
    let gpio_b = p.GPIOB;
    let rcc = p.RCC;
    let exti = p.EXTI;
    let afio = p.AFIO;

    let rts_is_negated = || gpio_b.idr.read().idr7().bit_is_clear();
    let pulse_strobe = || {
        // strobe latch line
        gpio_b.odr.modify(|_, w| w.odr5().bit(true));
        // erm. I don't really want to set up a timer yet.
        delay_us(12);
        gpio_b.odr.modify(|_, w| w.odr5().bit(false));
    };

    // TODO - track "last CDI data" instead
    // that way we can deal with things like custom acceleration or messages handled by this device
    // without spamming the console
    let mut last_snes_data = SnesState::all();

    // enable peripheral clocks
    rcc.apb2enr.modify(|_, w| w
                       .iopben().bit(true)
                       .iopaen().bit(true)
                       .spi1en().bit(true)
                       .afioen().bit(true)
                       .usart1en().bit(true));

    unsafe {
        afio.mapr.modify(|_, w| w
                        // disable jtag (but not swd) so we can use spi remap
                        .swj_cfg().bits(0b010)
                        // Remap (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
                        .spi1_remap().bit(true));

        gpio_b.crl.modify(|_, w| w
                          // set PB3 (clock) as 50MHz push-pull AF outputs
                          .cnf3().bits(0b10)
                          .mode3().bits(0b11)
                          // set PB6 (uart tx) as 50 mhz open-drain gp output
                          // this will require a pull-up to 5v
                          .cnf6().bits(0b01)
                          .mode6().bits(0b11)

                          // set PB4 (data) and PB7 (rts) as input
                          .cnf4().bits(0b01)
                          .mode4().bits(0b00)
                          .cnf7().bits(0b01)
                          .mode7().bits(0b00)

                          // set pb5 (latch) as 2MHz push-pull output
                          .cnf5().bits(0b00)
                          .mode5().bits(0b11));

        gpio_a.crh.modify(|_, w| w
                          // unremapped tx/rx for uart1 to trigger EXTI interrupt
                          .cnf9().bits(0b10)
                          .mode9().bits(0b11)
                          .cnf10().bits(0b01)
                          .mode10().bits(0b00));
                          
        // want USARTDIV to be 52.08 = 8mhz/16/9600
        // fraction is in 16ths, so 52.0625 is what we end up with
        uart.brr.write(|w| w
                       .div_mantissa().bits(52)
                       .div_fraction().bits(1));

        spi.cr1.modify(|_, w| w
                      // pclk/128 -> 8mhz/128 ~= 62.5kHz
                      .br().bits(0b110)
                      .dff().bit(true)
                      .mstr().bit(true)
                      .ssm().bit(true)
                      .lsbfirst().bit(true)
                      .ssi().bit(true)
                      .spe().bit(true)
                      .cpol().bit(true)
                      .cpha().bit(false));

        // set exti7 as port b interrupt
        afio.exticr2.modify(|_, w| w.exti7().bits(0b001));
        // set exti9 as port a interrupt
        // tx on port a
        afio.exticr3.modify(|_, w| w.exti10().bits(0b000));
    }

    // unmask exti10 to "receive" our outgoing serial signals for inversion
    exti.imr.modify(|_, w| w.mr10().bit(true));
    // enable rising edge irq for exti10
    exti.rtsr.modify(|_, w| w.tr10().bit(true));
    // enable falling edge irq for exti10
    exti.ftsr.modify(|_, w| w.tr10().bit(true));

    // unmask exti7 to receive interrupts for RTS inputs
    exti.imr.modify(|_, w| w.mr7().bit(true));
    // enable rising edge irq for exti7
    exti.rtsr.modify(|_, w| w.tr7().bit(true));
    // enable falling edge irq for exti7
    exti.ftsr.modify(|_, w| w.tr7().bit(true));

    // enable the uart transmit
    uart.cr1.write(|w| w
                   .ue().bit(true)
                   .te().bit(true));

    while rts_is_negated() {}
    delay_ms(150);
    send_controller_type(&mut uart);

    syst.set_clock_source(SystClkSource::Core);
    // configures the system timer to trigger a SysTick exception at 1khz
    // core clock = 8 mhz
    syst.set_reload(8_000_000 / 100);
    syst.enable_counter();
    syst.enable_interrupt();

    // enable exti7 in nvic
    nvic.enable(stm32f103::Interrupt::EXTI9_5);
    nvic.enable(stm32f103::Interrupt::EXTI15_10);

    loop {
        // did we see rts falling edge? if so, we need to shut up until it goes high
        if rts_is_negated() {
            // wait for a rising edge
            while rts_is_negated() {
                cortex_m::asm::wfi();
            }

            send_controller_type(&mut uart);
        }

        // refresh snes data
        let new_snes_data = fetch_snes_controller_state(&mut spi, pulse_strobe);

        // transmit a full data packet if the snes data changed or we're holding a direction
        // the spec requests continuous packets when the device is out of center position
        if (last_snes_data != new_snes_data)
            || new_snes_data
                .intersects(SnesState::Up | SnesState::Down | SnesState::Left | SnesState::Right)
        {
            // transmit packet here
            tx_controller_state(new_snes_data, &mut uart, rts_is_negated);
        }
        last_snes_data = new_snes_data;

        cortex_m::asm::wfi();
    }
}

fn fetch_snes_controller_state<F>(spi: &mut stm32f103::SPI1, pulse_strobe: F) -> SnesState
where
    F: Fn(),
{
    pulse_strobe();

    // write a nonsense byte to tx to trigger a transfer
    unsafe {
        spi.dr.write(|w| w.dr().bits(0x55aa));
    }
    // wait for some fresh rx data
    while spi.sr.read().rxne().bit_is_clear() {}
    // grab the data out of the DR and invert it
    SnesState::from_bits_truncate(!spi.dr.read().dr().bits())
}

fn tx_serial_byte(uart: &mut stm32f103::USART1, byte: u16) {
    unsafe {
        uart.dr.write(|w| w.dr().bits(byte | 0x80));
    }
    while uart.sr.read().txe().bit_is_clear() {}
}

fn send_controller_type(uart: &mut stm32f103::USART1) {
    tx_serial_byte(uart, CONTROLLER_TYPE_ID);
}

fn tx_controller_state<F>(
    controller_state: SnesState,
    uart: &mut stm32f103::USART1,
    rts_is_negated: F,
) where
    F: Fn() -> bool,
{
    let mut button_1 = 0;
    let mut button_2 = 0;
    if controller_state.contains(SnesState::Y) {
        button_1 = 1;
        button_2 = 1;
    } else {
        if controller_state.intersects(SnesState::B | SnesState::X) {
            button_1 = 1;
        }
        if controller_state.contains(SnesState::A) {
            button_2 = 1;
        }
    }

    let x = if controller_state.contains(SnesState::Right) {
        0x1
    } else if controller_state.contains(SnesState::Left) {
        0xff
    } else {
        0
    };

    let y = if controller_state.contains(SnesState::Up) {
        0xff
    } else if controller_state.contains(SnesState::Down) {
        0x1
    } else {
        0
    };

    tx_serial_byte(
        uart,
        0x40 | button_1 << 5 | button_2 << 4 | (y & 0xc0) >> 4 | (x & 0xc0) >> 6,
    );
    if rts_is_negated() {
        return;
    }
    tx_serial_byte(uart, x & 0x3f);
    if rts_is_negated() {
        return;
    }
    tx_serial_byte(uart, y & 0x3f);
}

fn delay_us(us: u16) {
    // this works out about right at 8mhz in release mode
    for _ in 0..(3 * us / 2) {
        cortex_m::asm::nop()
    }
}

fn delay_ms(ms: u32) {
    // this works out about right at 8mhz in release mode
    for _ in 0..(1000 * ms) {
        cortex_m::asm::nop()
    }
}

#[exception]
fn SysTick() {
    // wake up to grab the latest SNES inputs, send them to the global storage
}

interrupt!(EXTI9_5, rts_edge);
fn rts_edge() {
    unsafe {
        // TODO: this is not particularly pretty.
        (*stm32f103::EXTI::ptr()).pr.write(|w| w.pr7().bit(true));
    }
}

interrupt!(EXTI15_10, serial_edge);
fn serial_edge() {
    unsafe {
        // TODO: this is not particularly pretty.
        (*stm32f103::EXTI::ptr()).pr.write(|w| w.pr10().bit(true));

        // I'm using a dangling output pin and this irq to act as an inverter.
        // don't panic.
        let pin_state = (*stm32f103::GPIOA::ptr()).idr.read().idr10().bit();
        (*stm32f103::GPIOB::ptr()).odr.modify(|_, w| w.odr6().bit(!pin_state));
    }
}
