#![no_std]
#![no_main]

extern crate panic_halt;
extern crate stm32f1;

use cortex_m_rt::{entry, exception};
use cortex_m::peripheral::syst::SystClkSource;
use stm32f1::stm32f103;
use stm32f1::interrupt;

const CONTROLLER_TYPE_ID: u16 = 0x4a;

#[derive(PartialEq)]
enum WakeSource {
    NoWake,
    SysTick,
    CdiRts,
    _SpiRxComplete,
    _UartTxComplete,
}

static mut WAKEUP_SOURCE: WakeSource = WakeSource::NoWake;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let f1_p = stm32f103::Peripherals::take().unwrap();
    let mut syst = p.SYST;
    let mut uart = f1_p.USART1;
    let mut spi = f1_p.SPI1;
    // TODO - track "last CDI data instead"
    // that way we can deal with things like custom acceleration or messages handled by this device
    // without spamming the console
    let mut last_snes_data = 0xff as u16;

    // TODO enable uart clock
    // TODO enable spi clock
    // TODO configure pins (TX, RTS, clock, data, strobe)
    // TODO configure uart baud rate
    // TODO configure spi clock speed
    // TODO configure spi to receive-only mode (BIDIMODE=0, RXONLY=1)
    // TODO configure 16-bit spi word
    // TODO configure falling edge interrupt on RTS

    send_controller_type(&mut uart);

    syst.set_clock_source(SystClkSource::Core);
    // configures the system timer to trigger a SysTick exception at 1khz
    // core clock = 8 mhz
    syst.set_reload(8_000_000 / 1000);
    syst.enable_counter();
    syst.enable_interrupt();

    loop {
        // TODO - check if the wakeup cause was the CDI requesting ident or a systick
        if unsafe {WAKEUP_SOURCE == WakeSource::CdiRts} {
            send_controller_type(&mut uart);
        } else {
            // refresh snes data
            let new_snes_data = fetch_snes_controller_state(&mut spi);

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

fn fetch_snes_controller_state(spi: &mut stm32f103::SPI1) -> u16 {
    // TODO: strobe latch line

    // enable spi to trigger a transfer
    spi.cr1.modify(|_, w| w.spe().bit(true));
    // wait for the rx complete interrupt to fire
    cortex_m::asm::wfi();
    // disable the spi transfer before we fetch data to avoid triggering another
    spi.cr1.modify(|_, w| w.spe().bit(false));
    // grab the data out of the DR
    spi.dr.read().dr().bits()
}

fn send_controller_type(uart: &mut stm32f103::USART1) {
    unsafe { uart.dr.write(|w| w.dr().bits(CONTROLLER_TYPE_ID)); }
    while uart.sr.read().txe().bit_is_clear() {}
}

fn tx_controller_state(_controller_state: u16, uart: &mut stm32f103::USART1) {
    // TODO: manipulate data for transmit
    unsafe { uart.dr.write(|w| w.dr().bits(0x11)); }
    while uart.sr.read().txe().bit_is_clear() {}
    unsafe { uart.dr.write(|w| w.dr().bits(0x22)); }
    while uart.sr.read().txe().bit_is_clear() {}
    unsafe { uart.dr.write(|w| w.dr().bits(0x33)); }
    while uart.sr.read().txe().bit_is_clear() {}
}

#[exception]
fn SysTick() {
    // wake up to grab the latest SNES inputs, send them to the global storage
    unsafe {WAKEUP_SOURCE = WakeSource::SysTick};
}

// TODO: figure out the correct irq source for this one
interrupt!(EXTI0, rts_edge);
fn rts_edge() {
    // the CDI is asking us to identify ourselves
    unsafe {WAKEUP_SOURCE = WakeSource::CdiRts};
}
