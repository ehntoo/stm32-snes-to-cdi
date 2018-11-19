#![no_std]
#![no_main]

extern crate panic_halt;
extern crate stm32f1;

use cortex_m_rt::{entry, exception};
use cortex_m::peripheral::syst::SystClkSource;
use stm32f1::stm32f103;
use stm32f1::interrupt;

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

#[derive(PartialEq)]
enum WakeSource {
    NoWake,
    SysTick,
    CdiRts,
}

static mut WAKEUP_SOURCE: WakeSource = WakeSource::NoWake;

#[entry]
fn main() -> ! {
    let cp = stm32f103::CorePeripherals::take().unwrap();
    let p = stm32f103::Peripherals::take().unwrap();
    let mut syst = cp.SYST;
    let mut uart = p.USART1;
    let mut spi = p.SPI1;
    let mut gpio_b = p.GPIOB;
    let rcc = p.RCC;
    let exti = p.EXTI;
    let afio = p.AFIO;
    let mut nvic = cp.NVIC;

    // TODO - track "last CDI data" instead
    // that way we can deal with things like custom acceleration or messages handled by this device
    // without spamming the console
    let mut last_snes_data = 0xff as u16;

    // enable peripheral clocks
    rcc.apb2enr.modify(|_, w| w
                       .iopben().bit(true)
                       .spi1en().bit(true)
                       .afioen().bit(true)
                       .usart1en().bit(true));

    unsafe {
        afio.mapr.modify(|_, w| w
                        // Remap (TX/PB6, RX/PB7)
                        .usart1_remap().bit(true)
                        // disable jtag (but not swd) so we can use spi remap
                        .swj_cfg().bits(0b010)
                        // Remap (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
                        .spi1_remap().bit(true));

        gpio_b.crl.modify(|_, w| w
                          // set PB3 (clock) and PB6 (uart tx) as 2MHz push-pull AF outputs
                          .cnf3().bits(0b10)
                          .mode3().bits(0b11)
                          .cnf6().bits(0b10)
                          .mode6().bits(0b11)

                          // set PB4 (data) and PB7 (rts) as input
                          .cnf4().bits(0b01)
                          .mode4().bits(0b00)
                          .cnf7().bits(0b01)
                          .mode7().bits(0b00)

                          // set pb5 (latch) as 2MHz push-pull output
                          .cnf5().bits(0b00)
                          .mode5().bits(0b11));

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
    }

    // unmask exti7
    exti.imr.modify(|_, w| w.mr7().bit(true));
    // enable rising edge irq for exti7
    exti.rtsr.modify(|_, w| w.tr7().bit(true));
    exti.ftsr.modify(|_, w| w.tr7().bit(true));

    // enable the uart transmit
    uart.cr1.write(|w| w
                   .ue().bit(true)
                   .te().bit(true));

    send_controller_type(&mut uart);

    syst.set_clock_source(SystClkSource::Core);
    // configures the system timer to trigger a SysTick exception at 1khz
    // core clock = 8 mhz
    syst.set_reload(8_000_000 / 1000);
    syst.enable_counter();
    syst.enable_interrupt();

    // enable exti7 in nvic
    nvic.enable(stm32f103::Interrupt::EXTI9_5);

    loop {
        if unsafe {WAKEUP_SOURCE == WakeSource::CdiRts} {
            // did we see rts falling edge? if so, we need to shut up until it goes high
            if gpio_b.idr.read().idr7().bit_is_clear() {
                // wait for a rising edge
                syst.disable_interrupt();
                cortex_m::asm::wfi();
            }
            send_controller_type(&mut uart);
            syst.enable_interrupt();
        } else {
            // refresh snes data
            let new_snes_data = fetch_snes_controller_state(&mut spi, &mut gpio_b);

            // transmit a full data packet iff the snes data changed
            if last_snes_data != new_snes_data {
                last_snes_data = new_snes_data;
                // transmit packet here
                tx_controller_state(last_snes_data, &mut uart);
            }
        }
        cortex_m::asm::wfi();
    }
}

// TODO - rework to take a closure for strobing?
fn fetch_snes_controller_state(spi: &mut stm32f103::SPI1, gpio: &mut stm32f103::GPIOB) -> u16 {
    // strobe latch line
    gpio.odr.modify(|_, w| w.odr5().bit(true));
    // erm. I don't really want to set up a timer yet.
    delay_us(12);
    gpio.odr.modify(|_, w| w.odr5().bit(false));

    // write a nonsense byte to tx to trigger a transfer
    unsafe { spi.dr.write(|w| w.dr().bits(0x55aa)); }
    // wait for some fresh rx data
    while spi.sr.read().rxne().bit_is_clear() { }
    // grab the data out of the DR and invert it
    (!spi.dr.read().dr().bits()) & 0xfff
}

fn nybble_to_hex(nybble: u16) -> u16 {
    if nybble > 9 {
        (nybble + 0x41 - 10) as u16
    } else {
        (nybble + 0x30) as u16
    }
}

fn tx_serial_byte(uart: &mut stm32f103::USART1, byte: u16) {
    unsafe { uart.dr.write(|w| w.dr().bits(byte)); }
    while uart.sr.read().txe().bit_is_clear() {}
}

fn send_controller_type(uart: &mut stm32f103::USART1) {
    tx_serial_byte(uart, CONTROLLER_TYPE_ID);
}

fn tx_controller_state(controller_state: u16, uart: &mut stm32f103::USART1) {
    // TODO: manipulate data for transmit
    tx_serial_byte(uart, nybble_to_hex((controller_state & 0xf00) >> 8));
    tx_serial_byte(uart, nybble_to_hex((controller_state & 0xf0) >> 4));
    tx_serial_byte(uart, nybble_to_hex((controller_state & 0xf) >> 0));
    tx_serial_byte(uart, 0x0d);
    tx_serial_byte(uart, 0x0a);
}

fn delay_us(us: u16) {
    // this works out about right at 8mhz in release mode
    for _ in 0..(3 * us / 2) {
        cortex_m::asm::nop()
    }
}

#[exception]
fn SysTick() {
    // wake up to grab the latest SNES inputs, send them to the global storage
    unsafe {WAKEUP_SOURCE = WakeSource::SysTick};
}

interrupt!(EXTI9_5, rts_edge);
fn rts_edge() {
    // the CDI is asking us to identify ourselves
    unsafe {
        WAKEUP_SOURCE = WakeSource::CdiRts;
        // TODO: this is not particularly pretty.
        (*stm32f103::EXTI::ptr()).pr.write(|w| w.pr7().bit(true)) ;
    }
}
