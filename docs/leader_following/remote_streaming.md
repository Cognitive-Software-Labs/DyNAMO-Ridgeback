# Measuring remote browser delivery

A simulator's render rate, ROS camera-message rate and browser-delivered desktop
updates are different measurements. A 60 Hz browser animation callback does not
prove that RDP delivered 60 new images.

The optional probe under `tools/diagnostics/remote_stream_probe/` shows a small
frame-number marker on the remote workstation. Its client bookmarklet reads that
marker from the visible Guacamole canvases, counts distinct valid frame IDs and
downloads a local JSON result. It makes no network requests and requests no screen
capture permission. It does not inspect credentials or other desktop pixels.

## Run the probe

On the remote workstation, from the repository root:

```bash
python3 tools/diagnostics/remote_stream_probe/server.py \
  --seconds 120 --rate 60 \
  --output artifacts/leader-following/rdp-probe/server-draws.csv
```

Use a new output path for each attempt. The marker window stays above the
simulator. Close it to stop early. The CSV counts **issued marker draws** after
Tk flushes its drawing commands; it does not prove that the compositor or RDP
presented every draw.

On the computer where the browser displays `mole.rz.tu-clausthal.de`:

1. Create a temporary browser bookmark. Set its URL to the complete contents of
   `tools/diagnostics/remote_stream_probe/bookmarklet.txt` (starts `javascript:`).
2. Keep the remote-desktop tab foreground and invoke that bookmark there.
3. Click the centre of the cyan square in the marker window. Keep the marker
   fully visible and avoid resizing, zooming or scrolling during measurement.
4. After 30 seconds, the probe downloads `dynamo-rdp-delivery.json`. Escape stops
   it early. Invoking the bookmark again also stops it. Remove the temporary
   bookmark afterwards if desired.

The browser probe is readable in `client.js`. It checks the cyan/magenta boundary
cells and inverted bit rows before accepting an ID. Invalid partial or compressed
updates are excluded and counted. Test the same marker directly on the workstation
when possible to distinguish decoding limitations from transport losses.

## Interpret the result

Compare the server CSV draw count over the client measurement interval with
`delivered_updates_per_second`. The timestamps use separate clocks: compare rates
and overlapping frame IDs, not absolute timestamps or one-way latency.

Compare simulator viewport render events separately over the same scene/window
and host load. The marker measures desktop delivery capacity, not how many whole
simulator frames arrived. It can update independently of the simulator. A browser
sampling at 60 Hz cannot establish a received rate above 60, and low decoding
rates alone do not prove a network bottleneck. Compositing, clipping, canvas
security restrictions, browser refresh and the probe itself can affect results.

Keep raw JSON/CSV under `artifacts/`. No received-frame result exists until the
client browser actually runs the probe. Server-only access cannot establish it.
