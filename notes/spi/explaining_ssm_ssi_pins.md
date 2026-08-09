# What exactly are SSM and SSI?

The **SSM (Software Slave Management)** and **SSI (Internal Slave Select)** bits are used to manage the **NSS (Chip Select)** signal, but **only internally inside the STM32**.

They **do not generate any signal on the physical NSS pin** that can be seen by another device.

## SSM = 1

Setting `SSM = 1` means that you disconnect the **physical NSS pin** from the SPI hardware inside the STM32.

From this point on, the SPI peripheral no longer cares about the actual voltage level on the physical NSS pin. Instead, it uses the **SSI bit** internally.

```text
SSM = 0
    ↓
SPI hardware uses the physical NSS pin

SSM = 1
    ↓
SPI hardware ignores the physical NSS pin
    ↓
SPI hardware uses the SSI bit instead
```

## SSI = 1

Setting `SSI = 1` tells the internal SPI logic:

> "Treat NSS as if it were HIGH."

This is important when the STM32 operates as an SPI Master, because it prevents the SPI peripheral from detecting an incorrect NSS state and triggering a **Mode Fault (`MODF`)**.

```text
SSM = 1
SSI = 1
    ↓
Internal NSS = HIGH
```

However, this is only an **internal state** inside the STM32.

## The important problem

When `SSM = 1`, the physical NSS pin is disconnected from the SPI hardware.

Therefore, changing `SSI` does **not** change the voltage level on the physical NSS pin.

For example:

```text
STM32 SPI peripheral
        │
        │ SSM = 1
        ↓
   physical NSS
        X
   disconnected
```

The STM32 SPI peripheral is perfectly happy because:

```text
SSM = 1
SSI = 1
```

but an external device, such as a **Raspberry Pi Pico**, does not know anything about the internal `SSI` bit.

The Pico only sees the **actual voltage on its CS pin**.

So if the Pico expects:

```text
CS = HIGH → idle
CS = LOW  → slave selected
```

then the STM32 still needs to generate that physical `HIGH → LOW` transition somehow.

### In other words

```text
SSI = 1
    ↓
Internal STM32 state only
    ↓
Does NOT change the physical NSS pin
    ↓
Does NOT select the external slave
```

If you want to control the slave's CS pin while using `SSM = 1`, you normally use a **separate GPIO**:

```text
STM32                         Raspberry Pi Pico
------                        ------------------
SCK  ───────────────────────► SCK
MOSI ───────────────────────► MOSI
MISO ◄─────────────────────── MISO
GPIO ───────────────────────► CS
GND  ──────────────────────── GND
```

The GPIO then controls the actual chip-select signal:

```text
GPIO = HIGH → Pico not selected
GPIO = LOW  → Pico selected
```

So the key distinction is:

> **SSM/SSI control the internal NSS logic of the STM32 SPI peripheral. They do not automatically control the physical CS/NSS signal of an external SPI device.**
