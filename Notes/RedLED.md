# Flashing Red LED Using TIM4 Channel 2

System Timer Clock (Maximum frequency of the system)

$f_{sys}=64MHz$

Prescaler: $0 - 2^{16}$ (Unitless value to scale the system timer clock)

Counter Period

Pulse: > pulse, output on; < pulse, output off.

$T$: Period

$f$: Frequency

$f=\frac{f_{sys}}{(Prescaler + 1)(Counter Period + 1)}$

$Prescaler = \frac{f_{sys}}{(Period + 1)\cdot f}$