# `stm32-snes-to-cdi`

> A way to use SNES controllers for your Philips CD-i

SNES clock on PB3
     data on PB4
     latch on PB5

serial tx on PB6
serial rts on PB7

brown is ground
data red
latch yellow
clock blue
5v white


mini-din
5 - green gnd
2 - rx red
7 - rts purple
8 - 5v black


hm. okay, I can move either the uart tx output or the rts pins to get them on separate EXTI interrupts, since EXTI9_5 are all the same irq
unremapped uart1 tx is pa9, remapped is pb6, so both are on that irq

rts can go anywhere I've got a 5v tolerant pin
5v tolerant pins:
PB10, PB11, most everything on the other side of the board

exti interrupts:
0, 1, 2, 3, 4, 9_5, 15_10

Let's move RTS to PA15

Nope. Let's bridge rx/tx on pa9/pa10 and we've got things set
