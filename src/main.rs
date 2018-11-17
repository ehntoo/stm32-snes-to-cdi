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
    let mut gpio_b = f1_p.GPIOB;
    let rcc = f1_p.RCC;
    let afio = f1_p.AFIO;

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

    // use port b for usart1/spi1
    afio.mapr.modify(|_, w| w
                     .usart1_remap().bit(true)
                     .spi1_remap().bit(true));

    unsafe {
        gpio_b.crl.modify(|_, w| w
                          // set PB3 (clock) and PB6 (uart tx) as 2MHz push-pull AF outputs
                          .cnf3().bits(0b10)
                          .mode3().bits(0b10)
                          .cnf6().bits(0b10)
                          .mode6().bits(0b10)

                          // set pb5 (latch) as 2MHz push-pull output
                          .cnf5().bits(0b00)
                          .mode5().bits(0b10));

        // want USARTDIV to be 52.08 = 8mhz/16/9600
        // fraction is in 16ths, so 52.0625 is what we end up with
        uart.brr.write(|w| w
                       .div_mantissa().bits(52)
                       .div_fraction().bits(1));

        spi.cr1.write(|w| w
                      // pclk/128 -> 8mhz/128 ~= 62.5kHz
                      .br().bits(0b110)
                      .dff().bit(true)
                      .mstr().bit(true)
                      .ssm().bit(true)
                      .cpol().bit(true)
                      .cpha().bit(false)
                      // .rxonly().bit(true)
                      .ssi().bit(true));
    }
    spi.cr1.modify(|_, w| w
                   .spe().bit(true));

    // spi.cr2.write(|w| w.rxneie().bit(true));

    // enable the uart transmit
    uart.cr1.write(|w| w
                   .ue().bit(true)
                   // TODO - WFI during char transmission rather than polling?
                   //.tcie().bit(true)
                   .te().bit(true));

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
            let new_snes_data = fetch_snes_controller_state(&mut spi, &mut gpio_b);
            tx_serial_byte(&mut uart, new_snes_data + 0x30);

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

    // // enable spi to trigger a transfer
    // spi.cr1.modify(|_, w| w.spe().bit(true));
    // // wait for the rx complete interrupt to fire
    // // cortex_m::asm::wfi();
    // while spi.sr.read().rxne().bit_is_clear() { }
    // // disable the spi transfer before we fetch data to avoid triggering another
    // spi.cr1.modify(|_, w| w.spe().bit(false));
    // // grab the data out of the DR
    // spi.dr.read().dr().bits()

    // write a nonsense byte to tx to trigger a transfer
    unsafe { spi.dr.write(|w| w.dr().bits(0xff)); }
    // wait for some fresh rx data
    while spi.sr.read().rxne().bit_is_clear() { }
    // grab the data out of the DR
    spi.dr.read().dr().bits()
}

fn tx_serial_byte(uart: &mut stm32f103::USART1, byte: u16) {
    unsafe { uart.dr.write(|w| w.dr().bits(byte)); }
    while uart.sr.read().txe().bit_is_clear() {}
}

fn send_controller_type(uart: &mut stm32f103::USART1) {
    tx_serial_byte(uart, CONTROLLER_TYPE_ID);
}

fn tx_controller_state(_controller_state: u16, uart: &mut stm32f103::USART1) {
    // TODO: manipulate data for transmit
    tx_serial_byte(uart, 0x41);
    tx_serial_byte(uart, 0x42);
    tx_serial_byte(uart, 0x43);
}

fn delay_us(us: u16) {
    const K: u16 = 3; // this value needs to be tweaked
    for _ in 0..(K * us) {
        cortex_m::asm::nop()
    }
}

#[exception]
fn SysTick() {
    // wake up to grab the latest SNES inputs, send them to the global storage
    unsafe {WAKEUP_SOURCE = WakeSource::SysTick};
}

// interrupt!(SPI1, spi_interrupt);
// fn spi_interrupt() {
//     unsafe {WAKEUP_SOURCE = WakeSource::SpiRxComplete};
// }

// TODO: figure out the correct irq source for this one
interrupt!(EXTI0, rts_edge);
fn rts_edge() {
    // the CDI is asking us to identify ourselves
    unsafe {WAKEUP_SOURCE = WakeSource::CdiRts};
}
