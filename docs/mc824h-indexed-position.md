# MC824H indexed encoder position

This branch adds support for the indexed DMP position format observed on a Nice MC824H / MCA1R10 controller.

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
01 00 06 -> 0x0006 = 6     (settled closed)
```

Index `02` returned zero on the tested one-motor installation, while index `01` tracked the active motor continuously.

## Implementation

When product detection reports `MC824H`, the component:

1. selects DMP position index `0x01`;
2. reads `04/11`, `04/18`, and `04/19` with that index;
3. decodes three-byte position responses as `[index][MSB][LSB]`;
4. prefers fresh encoder values over time-based estimation;
5. learns open/closed encoder endpoints from real `04/11` readings at endpoint states as a fallback;
6. does not apply the generic `04/D1` limit-switch bit mapping, because that mapping has not been validated for MC824H.

The percentage remains based on:

```text
(current - closed) / (open - closed)
```

using `04/18` / `04/19` where available and endpoint observations as fallback.

## Expected log output

During discovery:

```text
Detected MC824H - using indexed 16-bit position (index 0x01)
```

During movement with verbose logging:

```text
Indexed position register 0x11: index=1 raw=2834 (0x0B12)
Indexed position register 0x11: index=1 raw=2805 (0x0AF5)
...
```

## Test checklist

- Start fully closed and confirm Encoder Raw is near 0.
- Open fully and confirm Encoder Raw rises continuously to roughly the learned/open endpoint.
- Close fully and confirm Encoder Raw falls continuously back near 0.
- Confirm Gate Position moves smoothly between 0 and 100 percent.
- Confirm time-based estimation does not overwrite fresh encoder updates.
- Check `Position max` and `Position close (min)` logs to see whether indexed `04/18` / `04/19` are provided by the controller.

This behavior is based on read-only BusT4 captures from the tested MC824H. No position register writes are required.
