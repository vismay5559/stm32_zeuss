# The robot's software, in plain language

This folder is the part of the firmware that decides what the robot does. The
rest of the tree is chip support code from the manufacturer.

If you are not a programmer, read this page first. Every header file in this
folder starts with a plain description of what that piece is for, and the words
below are the ones that come up everywhere.

## The words you need

**Board / MCU** — the small computer bolted to the robot. It has no screen and
no operating system. It runs one program, forever, from the moment it is
powered on.

**Tick** — one heartbeat of the robot. The board wakes up 1000 times a second,
and each of those wake-ups is a tick. Everything happens on this rhythm: read
the sensors, work out where the robot is, decide what the legs should do, send
a report. One tick is one millisecond, so there is not much time to waste.

**The loop** — the program never finishes. It goes round and round doing the
tick work, from power-on until power-off. "The loop stalled" means the robot
stopped thinking, which is the thing everyone is most afraid of.

**Interrupt** — a shoulder-tap from the hardware. When a sensor has data ready,
it interrupts whatever the board was doing, the board handles it in a few
microseconds, then goes back. Interrupts are why some of this code has to be
careful: two things can touch the same piece of memory at slightly different
moments.

**DMA** — a courier. Instead of the board copying data from a sensor one byte
at a time, dedicated hardware fetches the whole lot in the background and taps
the board on the shoulder when it is done. The board gets on with other work
meanwhile.

**CRC** — a checksum. A short number calculated from a message and sent with
it. If the message is damaged in transit, the number no longer matches and the
receiver throws it away. It proves the message arrived intact. It does **not**
prove the message was a good idea.

**Mask / bit** — a compact way of carrying several yes/no answers in one
number. "Bit 0 is the left toe switch" means one particular yes/no lives in one
particular slot. A "valid mask" says which readings in a batch can be trusted.

**Debounce** — ignoring flicker. A mechanical switch does not close cleanly; it
chatters for a few thousandths of a second. Debouncing means waiting until it
has settled before believing it.

**The Pi** — a larger computer riding on the robot, which decides how to walk.
It talks to the board over a USB cable. The board handles the fast, safety
critical work; the Pi handles the thinking. If the Pi goes quiet, the board has
to cope on its own.

**Estimator / filter** — the piece that works out where the robot actually is.
No single sensor knows: the accelerometer is noisy, the foot switches only say
"touching", the joint sensors only say "bent this far". The estimator combines
them into one best guess, and reports how much it trusts that guess.

**Arm / armed** — whether the robot is allowed to move. Armed means the motors
will act on commands. Idle means they will not, no matter what arrives.

## How the pieces fit together

Once per tick, in this order:

1. **Sensors are read** — the movement sensor (`imu_bno085`), the joint motors
   (`act_odrive`), the spring sensors (`enc_as5048a`), the foot switches
   (`contact`).
2. **Position is worked out** — `fusion` hands the readings to `inekf`, the
   estimator, which answers "here is where I think the robot is, and here is
   how sure I am".
3. **Safety decides** — `safety` checks the robot is healthy and that the Pi is
   still talking. Nothing reaches a motor without passing it.
4. **A report goes to the Pi** — `link_usb` sends everything the Pi needs, and
   collects whatever command the Pi sent back.
5. **The watchdog is fed** — `watchdog` is a dead-man's switch. If a tick ever
   fails to finish, it restarts the board rather than leaving the motors
   holding their last order.
