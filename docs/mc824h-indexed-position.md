# MC824H indexed encoder position

This branch adds support for the indexed DMP position format observed and validated on a Nice MC824H / MCA1R10 controller with a single motor connected to M2.

## Observed wire format

A normal unindexed read of DMP register `04/11` can return an unusable or previously selected channel. The MC824H accepts an indexed GET:

```text
04 11 99 00 01 01
```

where the final `01` is the DMP index.

The response payload is three bytes:

```text
01 PP PP
```

- byte 0: echoed index
- bytes 1..2: 16-bit big-endian encoder position

Examples captured from the controller:

```text
01 0B 12 -> 0x0B12 = 2834  (fully open)
01 0A F5 -> 0x0AF5 = 2805  (closing)
01 00 77 -> 0x0077 = 119   (near closed)
01 00 00 -> 0x0000 = 0     (closed during travel)
01 00 05 -> 0x0005 = 5     (settled closed)
```

Index `02` returned zero on the tested one-motor installation, while index `01` tracked the active motor continuously.

## Position limits on the tested MC824H

Indexed `04/18` is supported and returned:

```text
01 0B 10 -> 2832
```

Indexed `04/19` returned `0xFD` (unsupported). The component therefore learns the closed endpoint from the real `04/11` value when the controller reports `Closed`. The open endpoint is also refreshed from `04/11` when the controller reports `Opened`; on the tested gate it settled at `2834`, slightly above the `04/18` value of `2832`.

## Implementation

When product detection reports `MC824H`, the component:

1. selects DMP position index `0x01`;
2. reads `04/11`, `04/18`, and `04/19` with that index;
3. decodes three-byte position responses as `[index][MSB][LSB]`;
4. prefers fresh encoder values over time-based estimation;
5. learns endpoint values from real `04/11` readings at `Opened` / `Closed` states;
6. does not apply the generic `04/D1` limit-switch bit mapping, because that mapping is not valid for the tested MC824H;
7. keeps the cover UI in an active movement state until the controller reports `Opened` / `Closed`, even if the encoder reaches its endpoint slightly earlier.

The percentage is calculated as:

```text
(current - closed) / (open - closed)
```

using the best available open/closed endpoint values.

## Validation

Tested on ESPHome 2026.9.0 with MC824H firmware `CD14e`.

A full close/open cycle confirmed:

- indexed `04/11` follows the encoder continuously in both directions;
- the time-based estimator no longer overwrites fresh encoder values;
- while closing, the cover remains at 1% until `Closed`, then publishes 0% / IDLE;
- while opening, the cover remains in OPENING until `Opened`, then publishes 100% / IDLE;
- endpoint learning stabilizes around `closed = 5` and `open = 2834` on the tested installation.

## Expected log output

During discovery:

```text
Detected MC824H - using indexed 16-bit position (index 0x01)
```

During movement with verbose logging:

```text
Indexed position register 0x11: index=1 raw=2805 (0x0AF5)
Indexed position register 0x11: index=1 raw=2772 (0x0AD4)
...
```

At the endpoints:

```text
CMD status: Closed
MC824H learned closed encoder endpoint: 5

CMD status: Opened
MC824H learned open encoder endpoint: 2834
```

This behavior is based on read-only BusT4 captures and a complete real-gate validation cycle. No position register writes are required.