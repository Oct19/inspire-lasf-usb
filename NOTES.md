# LASF motor control note

Page: `py -3.13 python/bench_ui.py` → http://127.0.0.1:8766/ . HTML is read from disk on each GET. A change to `python/bench_ui.py` or `python/lasf16.py` needs a restart. Kill the LISTENING pid on 8766 first. Do not flash parameters, change id or baud, or point the tracker at COM4. Python is `py -3.13`. Update this note when the control rules change.

## Motor

Two LASF16 rods, ids 1 and 2, one CH340 board, COM4, 921600 8N1, supply 8.0 V. Close the vendor host program before opening the port. Counts 0–2000, larger means more extension. Nominal 2000 counts is 16 mm. The status frame has position, target, current, temperature, and force. It has no speed. Measured speed is the change in counts divided by the sample interval. Positive speed extends.

Frame: `55 AA | len | id | cmd | payload | checksum`. Status is `04 22`. Enable `04 04` after estop `04 23`. The power stage comes off once the measured position is within 2 counts of the aim that was sent. Leaving a speed command set after that keeps the stage on, and the rod hunts and steps back. Do not cut on the status target while a speed command is in flight: that target can still be the old count, and the cutoff then stops the move before it starts. The move to the curve start has to finish. A full stroke needs more than 3 s. The next command arms the rod again. A hard stop can read past 0 or 2000.

Speed command is one register write, address `0x28`, unsigned steps/s plus the aim position. Direction is the aim, not the sign of the speed word. The log stores a signed target speed. Mode register `0x25`: 0 position, 2 speed. A speed word without the aim in the same frame is ignored. Sending the same speed and aim again restarts the move and cuts the pace. Do not send `0xF2` for a slider release. That plan sprints, stops short, and creeps.

Cruise saturates near 1620 steps/s on the slower rod. Faster commands do not go faster. Cap commands at 1620.

How to move:

- Slider: while the pointer is down, the current counts are the destination and the rod starts toward them. Release sends the final counts. Skip a repeat if the aim moved by fewer than 20 counts. Do not label this mode in the UI.
- Default play follows a point 0.3 s ahead on the curve. The position is what the rod chases.
- Smooth is a switch. One direction is one frame: one speed and one destination. The default sine wave is two destinations per cycle, out and back. The speed is faster than distance/time so the drive's own speed-up and slow-down still arrive on time. One bench move fixed that allowance: 350 counts commanded at 800 steps/s took 0.58 s, about 5600 counts/s² if the two ramps are symmetric. No other speed frame is sent until the direction changes. The Hz box does not apply to this plan. Cut power when the planned time ends.
- Stop is the red square. It estops both rods, drops the rest of the plan, and disconnects the tracker. The time cursor and the position bars freeze. The triangle is pause bars while a move is running. The next Play waits for that move to leave, then clears cancel.
- Pause keeps the rest of the plan, estops the rods, and disconnects the tracker. Resume opens the tracker the same way Play does, then waits up to 2 s for one marker. A 3 s limit used to abort INIT and say there was no tracker while the Vega was already on the link. Ignore continues without a marker.
- A new speed frame is a new move. The rod does not keep its current pace and blend into the new speed. In a 10 Hz run, when the command stepped from 581 to 962 the measured speed fell from 444 to 343 before it climbed. It does not have to reach zero to be a restart. Sending the same speed and the same aim again also restarts the move and cuts the pace.

Before Play, wait up to 3 s for the tracker and for both rods to be within 30 counts of the curve start, then wait until both positions have been still for 0.5 s. That approach already marks the move as running so the idle cutoff does not cut it. The curve must start on that same move. Treating the flag as a second move leaves the rods sitting at the start and every later click waiting. If something is still unmet, list only those items. Each item can be ignored. The first cycle sounds different if the rod is still moving when the curve starts.

## NDI

Polaris Vega on Ethernet, Combined API ASCII, TCP 8765. The bench finds whichever link-local address answers. Do not freeze an old address. Do not open COM4 for the tracker. `INIT` then `TSTART` are required. A stray read is `TX 1800`, falling back to `TX 1000`. Coordinates in the reply are hundredths of a millimetre.

A tool line and the stray line are separated by a newline. Example: `0101` then `011+006108-100574-2241140100` is about (61.08, −1005.74, −2241.14) mm. In the CSV, store that reply with the newline turned into a space so Excel keeps one row. The checker turns the first space back into a newline before parsing.

One marker is the track. If the tracker never produced a point, do not rewrite the file and do not warn. If some rows had a marker, fill a gap from the neighbors, and if a row has several markers keep the one closest to the single-marker track. A row that was not exactly one marker, when the tracker was present, turns the record control yellow with “marker not found”.

The page must keep showing X Y Z during a recording. The readiness check is “exactly one point on screen”, the same points the page draws. A counter that stays at 0 while X Y Z are visible is a false “no tracker” dialog.

Log motors and the tracker on one clock, 55 Hz. `time_s` starts at 0. NDI’s practical ceiling is about 60 Hz. Run the marker repair after the file closes, not inside the sample loop.

## Experiment UI

One row, left to right: plan menu (curve, testA, testB), play/pause triangle, red stop square, Smooth, Cycles, Duration, Note and its box on that same line. The triangle is pause bars while a move is running. Stop is always the red square. testA and testB are empty until a plan is filled in `TEST_PLANS`. There is no Hz box and no Recording only button.

Two columns, aligned with the pair cards. Each column is position, then speed. The rod’s own curve is black with markers. The other rod is a light gray line with no markers and cannot be selected. Time runs from the left edge to the right edge.

Position is one cubic through the markers. The drawn position is cut at 0 and 2000, and those values are the top and bottom of the box. Do not rebuild the pieces on either side of a marker. That gives the two sides different slopes and puts a step in the speed plot. A marker on 0 or 2000 is still just a point on the cubic.

Speed is the slope of that cubic before the position cut. The speed box stays at ±1620. Do not grow the scale to the peak. A demand past 1620 runs off that fixed scale. That is how the page shows the curve cannot be run as drawn.

Default: 2 cycles, 4 s. Pair 1 starts and ends at 0. Pair 2 starts and ends at 2000. Five markers, one per quarter of the period. With more than one cycle the path repeats forward only, the end position is locked to the start, and the end marker is not a separate knob.

During Play, a vertical line marks the current time on all four plots. Measured position and measured speed are a red line on the same plots. Measured speed for that line is the change in position over about 0.1 s. A one-sample difference flies off the chart. Clear the red line when a new cycle starts. When the planned cycles end, stop the rods and leave the last red line up. Time after the last cycle is not another cycle.

`reaction_force` and the tip-sensor columns stay empty until those devices are wired. Do not invent a parser.
