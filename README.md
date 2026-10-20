# sensors-web

What a web page can read from a device **without ever showing a permission prompt**.

[**Try the demo →**](https://idle-intelligence.github.io/sensors-web/)

`web/index.html` is the demo: one row per thing a page can read, each with its availability and
a live value, and unsupported rows collected in a last section. Feature detection only, no
user-agent sniffing. A Start button begins the live readers and asks for the motion permission
where the platform requires a tap. The rows are:

- **Motion**: the Generic Sensor classes (accelerometer, linear acceleration, gravity, gyroscope,
  magnetometer, absolute and relative orientation, ambient light), the device orientation,
  compass and motion events, and Compute Pressure.
- **Location**: `watchPosition` with high accuracy, on its own permission (Start or the Allow
  location button): latitude, longitude, accuracy, altitude, altitude accuracy, heading, speed,
  fix age and update rate.
- **Network**: online state and every Network Information field (type, effective type, downlink,
  downlink max, round-trip time, save data).
- **Power and load**: every Battery Status field, JS heap, logical cores, device memory, storage
  estimate.
- **Display**: screen, window, pixel ratio, colour depth, visual viewport, screen orientation.
- **Device**: WebGPU adapter info and key limits, WebGL renderer, secure context and
  cross-origin isolation, client hints, languages, time zone.
- **Input and media**: touch points, pointer pressure and tilt, gamepads, camera, microphone and
  speaker counts, speech voices.
- **Preferences**: the `prefers-*` media queries plus pointer, hover, colour gamut, dynamic range,
  display mode and scripting.
- **Page and permissions**: visibility, focus, and the state of every permission the browser can
  query.
- **Other APIs**: presence only for Bluetooth, USB, Serial, HID, NFC, wake lock, vibration, idle
  detection, MIDI, speech recognition, screen capture and the like.

The web exposes no satellite counts, no Wi-Fi networks or signal strengths, no cell information,
no barometer and no temperature; those rows do not exist for that reason.

`web/probe.html` is a dependency-free page that probes every sensor and device-info API it
can reach, shows the values live, and POSTs them back to a dev server so readings can be
captured from a phone. `src/main.rs` is that dev server: std-only Rust, no crates.

## Run

```sh
cargo run --release              # serves web/ on :8787, pretty-prints POSTed reports
cloudflared tunnel --url http://localhost:8787
```

Open the printed `trycloudflare.com` URL on the phone. HTTPS matters: most of this is
gated on a secure context, not on a permission.

`web/` is self-contained and works as a static GitHub Pages deploy; without the server the
page just shows values instead of posting them.

## Findings

Measured 2026-07-25 on: macOS 26.3.1 / Chrome 149 (M2), Android 15 / Chrome 150 (Adreno 642L), Android 15 / Firefox 153 (Adreno 650).

### The headline

Motion sensors are **base authorisation**. No prompt, no gesture, no opt-in on Android.
`navigator.permissions.query()` returns `granted` outright for `accelerometer`,
`gyroscope` and `magnetometer` - the user is never asked and never notified. Only
geolocation, camera, microphone and notifications sit behind `prompt`.

The compass comes with it. `Magnetometer` has no constructor in any browser tested, but
`deviceorientationabsolute` hands over a true magnetometer-derived heading for free:
absolute alpha read 134.9° on Chrome and 299–334° on Firefox while relative alpha sat near
90° on both.

### Live sensor data, no prompt

| | Chrome Android | Firefox Android | Chrome desktop |
|---|---|---|---|
| Generic Sensor API | streams | **absent entirely** | ctors exist, `NotReadableError` |
| `Accelerometer` | `x 0.2 y 7.9 z 6.1` | no ctor | no hardware |
| `Gyroscope` | `x -0.005 y -0.028 z 0.023` | no ctor | no hardware |
| `GravitySensor` | `x 0.1 y 8.0 z 5.7` | no ctor | no hardware |
| `AbsoluteOrientationSensor` | quaternion | no ctor | no hardware |
| `devicemotion` | 16 ms interval | 100 ms interval | fires, all-null |
| `deviceorientation` | real | real | null |
| `deviceorientationabsolute` (compass) | real | real | null |
| `permissions.query({name:'accelerometer'})` | `granted` | `TypeError` | `granted` |

`Magnetometer` and `AmbientLightSensor` are flag-gated off everywhere - a hard no rather
than a permission question. **There is no consent path to either**: flag-gated in Chromium
since ~2017, spec never past W3C Working Draft, no origin trial, Firefox/WebKit negative.
You cannot prompt your way in. WebXR lighting estimation (illuminance + spherical
harmonics) is the only legal route to ambient light, at the cost of an AR session.

Also absent from the web platform entirely, with no path: **barometer** (no spec, no
implementation - this is why a web PressureNet is impossible), thermometer, battery
temperature, proximity, raw GNSS, cell/Wi-Fi scan info.

Worth probing next, not yet covered by this page: **Compute Pressure API** (Chromium, no
prompt) exposes CPU pressure states - a thermal-throttling proxy, as is timing a fixed
WASM workload.

**WebAssembly adds no sensors.** A WebAssembly module has no hardware access of its own. It reaches the device only through JavaScript imports of the same Web APIs this page already calls, and browsers have no WASI-style device access. So moving the code into WASM does not expose any sensor or reading that the JavaScript probe cannot see. It can add timing precision and shared-memory threads, which help with side channels such as the thermal proxy above, but that is better timing of the same data, not new data.

**iOS scopes the headline.** Safari gates all motion/orientation events behind
`DeviceMotionEvent.requestPermission()` with a user gesture, and has no Generic Sensor
API. "No prompt" is an Android claim.

**Duty cycle governs everything.** Sensors suspend when the tab is backgrounded or the
screen locks. A web page is a foreground, minutes-scale instrument - which rules out any
always-on or background-network concept regardless of the physics.

Chrome quantizes: accelerometer and devicemotion values arrive rounded to 0.1 m/s².
Gyroscope is not rounded the same way (0.005 rad/s resolution). Firefox reports
unquantized accelerometer values (`8.323`, `4.413`) but at a slower 100 ms cadence.

**Portability:** the DOM events (`devicemotion` / `deviceorientation` /
`deviceorientationabsolute`) are the only path that works on both engines. Anything
shipped must treat Generic Sensors as a Chrome fast-path over that baseline, not as the
primary API.

### Free device fingerprint, no prompt

| | Chrome Android | Firefox Android |
|---|---|---|
| GPU string | `ANGLE (Qualcomm, Adreno (TM) 642L, OpenGL ES 3.2)` | `Adreno (TM) 650, or similar` |
| cores / RAM | 8 / 8 GB | 8 / withheld |
| battery | `93%, not charging` | unsupported |
| network | `4g, downlink 1.7Mb, rtt 200ms` | unsupported |
| media device counts | 1 cam, 1 mic, 1 out (labels hidden) | - |
| storage quota | 10 GiB | 10 GiB |
| JS heap limit | 3.14 GB | unsupported |

UA-CH defeats UA reduction: the Chrome UA string claims "Android 10; K", while
`getHighEntropyValues()` returns the real `Android 15.0.0` and the device model,
with no prompt. Firefox withholds Battery, NetworkInformation, UA-CH, `deviceMemory`,
`performance.memory`, `vibrate` and WebGPU, and fuzzes its GPU string with "or similar".

### Open question

On Firefox Android the first two 1 s samples carried no motion keys; they appeared in the
same sample where `touch points: 1` first showed up. Firefox may gate motion events on
user interaction. Not confirmed - needs a clean reload with no touch.
