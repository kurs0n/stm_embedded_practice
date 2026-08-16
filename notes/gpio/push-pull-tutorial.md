https://www.youtube.com/watch?v=IjKDKGqCm_4

# Push-pull, open-drain i dlaczego I²C potrzebuje open-drain

Najłatwiej zrozumieć **push-pull** i **open-drain**, zaczynając od tego, co fizycznie robi wyjście cyfrowe.

## 1. Co oznacza „wyjście” GPIO?

Załóżmy, że mamy mikrokontroler i pin:

```text
              VCC = 3.3 V
                │
             [ tranzystor ]
                │
GPIO ───────────┤
                │
             [ tranzystor ]
                │
               GND
```

W uproszczeniu wyjście cyfrowe może zrobić dwie rzeczy:

* podłączyć linię do **VCC** → mamy logiczne `1`
* podłączyć linię do **GND** → mamy logiczne `0`

Kluczowe pytanie brzmi:

> **Czy pin może aktywnie wymusić zarówno `1`, jak i `0`?**

To właśnie różni **push-pull** od **open-drain**.

---

# 2. Push-pull

Push-pull ma **dwa tranzystory**:

```text
             VCC
              │
          ┌───┴───┐
          │   P   │  ← tranzystor "górny"
          └───┬───┘
              │
              ├──────── OUT
              │
          ┌───┴───┐
          │   N   │  ← tranzystor "dolny"
          └───┬───┘
              │
             GND
```

Działa to mniej więcej tak:

### Chcemy wysłać `1`

Górny tranzystor zostaje włączony:

```text
VCC
 │
[ON]
 │
 ├──── OUT = 3.3 V
 │
[OFF]
 │
GND
```

Czyli pin **aktywnie podciąga** linię do VCC.

### Chcemy wysłać `0`

Dolny tranzystor zostaje włączony:

```text
VCC
 │
[OFF]
 │
 ├──── OUT = 0 V
 │
[ON]
 │
GND
```

Czyli pin **aktywnie ściąga** linię do GND.

Dlatego:

> **Push-pull może aktywnie wymusić zarówno `1`, jak i `0`.**

---

# 3. Co to jest open-drain?

W open-drain nie mamy tranzystora, który aktywnie podciąga wyjście do VCC.

Mamy zasadniczo tylko dolny tranzystor:

```text
OUT
 │
 │
[ tranzystor ]
 │
GND
```

Może on zrobić:

### `0`

Włączamy tranzystor:

```text
OUT
 │
[ON]
 │
GND
```

→ `OUT = 0`

Ale co, jeśli chcemy `1`?

Wyłączamy tranzystor:

```text
OUT
 │
[OFF]

GND
```

I teraz ważna rzecz:

**wyjście nie jest połączone ani z VCC, ani z GND.**

Jest jakby „odłączone”.

To nazywamy:

> **High impedance / Hi-Z**

Czyli pin sam z siebie **nie robi logicznej jedynki**.

---

# 4. Skąd bierze się `1` w open-drain?

Potrzebujemy **rezystora podciągającego (pull-up)**:

```text
             VCC
              │
             [R]
              │
              ├──────── OUT
              │
           [tranzystor]
              │
             GND
```

Teraz:

## Tranzystor OFF

```text
             VCC
              │
             [R]
              │
              ├──────── OUT
              │
           [OFF]
              │
             GND
```

Tranzystor nie przewodzi.

Rezystor podciąga więc linię:

```text
OUT ≈ VCC
```

→ logiczne `1`.

## Tranzystor ON

```text
             VCC
              │
             [R]
              │
              ├──────── OUT
              │
           [ON]
              │
             GND
```

Teraz tranzystor ma znacznie mniejszą rezystancję niż rezystor pull-up, więc linia zostaje ściągnięta do GND:

```text
OUT ≈ 0 V
```

→ logiczne `0`.

Czyli open-drain można zapamiętać tak:

| Stan tranzystora | Stan linii               |
| ---------------- | ------------------------ |
| OFF              | `1` dzięki pull-up       |
| ON               | `0` dzięki tranzystorowi |

---

# 5. Najważniejsza różnica

## Push-pull

```text
          VCC
           │
        [HIGH]
           │
           ├──── OUT
           │
        [LOW]
           │
          GND
```

Mikrokontroler mówi:

> „Ja sam ustawię linię na `1` albo `0`.”

## Open-drain

```text
          VCC
           │
          [R]  ← pull-up
           │
           ├──── OUT
           │
        [LOW]
           │
          GND
```

Mikrokontroler mówi:

> „Potrafię tylko ściągnąć linię do zera. Jeśli tego nie robię, pozwalam rezystorowi zrobić jedynkę.”

I właśnie ta różnica jest **kluczowa dla I²C**.

---

# 6. Dlaczego I²C potrzebuje open-drain?

Bo na jednej magistrali I²C **wiele urządzeń jest podłączonych do tych samych przewodów**.

Mamy dwie linie:

* **SDA** — dane
* **SCL** — zegar

Przykładowo:

```text
                VCC
                 │
                [R]
                 │
SDA ─────────────┼─────────────┬────────────
                 │             │
               MCU           EEPROM
                 │             │
               GND           GND


                VCC
                 │
                [R]
                 │
SCL ─────────────┼─────────────┬────────────
                 │             │
               MCU           SENSOR
                 │             │
               GND           GND
```

I tutaj pojawia się problem z **push-pull**.

---

# 7. Co by się stało przy push-pull?

Załóżmy, że mamy dwa urządzenia:

```text
Device A ───── SDA ───── Device B
```

A mówi:

```text
SDA = 1
```

czyli aktywnie podłącza:

```text
SDA → VCC
```

B mówi:

```text
SDA = 0
```

czyli aktywnie podłącza:

```text
SDA → GND
```

Dostajemy:

```text
VCC
 │
[A: HIGH]
 │
 ├──── SDA
 │
[B: LOW]
 │
GND
```

Czyli mamy praktycznie:

```text
VCC ───── zwarcie ───── GND
```

To jest **konflikt na magistrali**.

Płynie duży prąd, napięcie jest nieokreślone, a w skrajnym przypadku można uszkodzić układy.

---

# 8. Open-drain rozwiązuje ten problem

W open-drain urządzenie **nigdy aktywnie nie wystawia `1`**.

Może tylko powiedzieć:

> „Ściągam linię do zera”

albo:

> „Puszczam linię”

Jeśli **nikt nie ściąga**, pull-up robi:

```text
SDA = 1
```

Jeśli **ktokolwiek ściąga**, mamy:

```text
SDA = 0
```

Czyli:

```text
SDA = 1  → jeśli WSZYSCY puszczają
SDA = 0  → jeśli KTOKOLWIEK ściąga
```

To jest bardzo ważna właściwość I²C.

---

# 9. Przykład z dwoma urządzeniami

Załóżmy:

```text
              VCC
               │
              [R]
               │
               ├──────── SDA
               │
          ┌────┴────┐
          │         │
        MCU A      MCU B
          │         │
       open-drain open-drain
```

## Oba puszczają linię

```text
A = OFF
B = OFF
```

Pull-up robi:

```text
VCC
 │
[R]
 │
SDA = 1
```

Czyli:

```text
A: "nie ciągnę"
B: "nie ciągnę"

→ SDA = 1
```

## A ściąga do zera

```text
A = ON
B = OFF
```

Mamy:

```text
VCC
 │
[R]
 │
SDA
 │
[A ON]
 │
GND
```

→ `SDA = 0`

## B ściąga do zera

```text
A = OFF
B = ON
```

→ `SDA = 0`

## Oba ściągają

```text
A = ON
B = ON
```

→ nadal `SDA = 0`

I **nie ma konfliktu**.

---

# 10. Dlaczego to jest szczególnie dobre dla I²C?

Ponieważ I²C jest magistralą, na której **wiele urządzeń może wpływać na stan linii**.

Open-drain pozwala zrobić coś bardzo interesującego:

```text
KAŻDY może wymusić 0
NIKT nie może aktywnie wymusić 1
```

W efekcie:

```text
SDA = 1  → jeśli WSZYSCY puszczają
SDA = 0  → jeśli KTOKOLWIEK ściąga
```

Można to traktować jak zachowanie podobne do **wired-AND**:

```text
0 AND 0 = 0
0 AND 1 = 0
1 AND 0 = 0
1 AND 1 = 1
```

---

# 11. Arbitraż w I²C

Załóżmy, że dwóch masterów jednocześnie próbuje nadawać:

```text
Master A: 1010...
Master B: 1001...
```

W pewnym momencie:

```text
Master A chce: 0
Master B chce: 1
```

Dzięki open-drain:

```text
Master A → ściąga SDA do 0
Master B → puszcza SDA
```

Więc fizycznie:

```text
SDA = 0
```

Master B **odczytuje linię** i widzi:

```text
"Chciałem wysłać 1,
ale na magistrali jest 0."
```

W ten sposób może wykryć, że **przegrał arbitraż**.

To jest jedna z bardzo ważnych przyczyn, dla których konstrukcja I²C opiera się na open-drain/open-collector.

---

# 12. Dlaczego potrzebny jest rezystor?

Bez rezystora, kiedy wszystkie tranzystory są OFF:

```text
VCC
 │
 │
SDA ─────────── ????
```

SDA byłoby w stanie **Hi-Z**.

Nie mielibyśmy stabilnego `1`.

Pull-up robi więc:

```text
             VCC
              │
             [R]
              │
              ├──────── SDA
              │
        ┌─────┴─────┐
        │            │
      Device A    Device B
        │            │
       GND          GND
```

Rezystor mówi:

> „Jeżeli nikt nie ściąga linii, podciągnę ją do VCC.”

Ale ponieważ jest rezystorem, urządzenie może nadal bezpiecznie zrobić:

```text
SDA → GND
```

i płynie tylko ograniczony prąd:

```text
VCC
 │
[R]
 │
SDA
 │
[transistor ON]
 │
GND
```

---

# 13. Najważniejsze do zapamiętania

### Push-pull

> **Mogę aktywnie wystawić `0` albo `1`.**

### Open-drain

> **Mogę aktywnie wystawić tylko `0`; `1` powstaje dzięki pull-upowi.**

### I²C

> **Ponieważ wiele urządzeń współdzieli tę samą linię, każde może bezpiecznie ściągnąć ją do `0`, ale żadne nie próbuje aktywnie wystawić `1`.**

Możesz więc myśleć o SDA/SCL w ten sposób:

```text
          VCC
           │
          [R]
           │
           ├─────────── SDA
           │
       ┌───┴───┐
       │       │
      A↓      B↓
     pull    pull
      down    down
```

**Wszyscy puszczają → `1`**

**Ktokolwiek ciągnie → `0`**

I to jest cała idea **open-drain w I²C**.
