# Claptrap Cardputer Adv Controller

An unofficial handheld controller for the **CL4P-TP Claptrap-in-a-Box robot** from *Borderlands: The Handsome Collection*, built for the [M5Stack Cardputer Adv](https://docs.m5stack.com/en/core/Cardputer-Adv).

The Cardputer scans for the robot's Wi-Fi, drives it with the printed arrow keys, displays its camera feed, and sends microphone audio while **G0** is held. Movement, video, and audio have been exercised on hardware; microphone audio quality is still being tuned.

## Build and upload

You need a Cardputer Adv, the CL4P-TP robot, and Arduino IDE. The sketch builds with these tested versions:

- M5Stack ESP32 board package **3.3.10**
- `M5Cardputer` **1.1.1**, `M5Unified` **0.2.28**, and `M5GFX` **0.2.35** (plus the libraries requested by Arduino's Library Manager)

1. Open [ClaptrapCardputer.ino](ClaptrapCardputer/ClaptrapCardputer.ino) in Arduino IDE.
2. Select the **M5Cardputer** board and the Cardputer Adv's serial port, then upload.
3. Turn on the robot. The Cardputer scans for nearby `CL4P-TP...` Wi-Fi networks. Use the printed **Up/Down arrows** to choose one and press **Enter** to connect.
4. Wait for **Robot: online** before driving. Start with speed preset **1** to check the direction of movement.

If the upload cannot connect to the ESP32-S3, follow [M5Stack's download mode instructions](https://docs.m5stack.com/en/arduino/m5cardputer/program): turn the Cardputer off, hold **G0**, connect a USB-C data cable, then release G0.

## Controls

| Key | Action |
| --- | --- |
| **Arrow keys** | Hold Up/Down to drive forward/backward and Left/Right to steer. Release to send zero movement. |
| **1–5** | Select speed 6, 12, 18, 24, or 31 out of 31. The default is 12. |
| **[ / ]** | Decrease or increase speed by one step. |
| **G0** | Hold to send the Cardputer microphone to the robot; release to stop. |
| **- / =** | Lower or raise microphone output volume by 10%. Default: 40%; range: 10–100%. |
| **V** | Switch between the status screen and camera view. |
| **R** | Scan again for robot Wi-Fi networks. |
| **Enter** | Connect to the selected Wi-Fi network. |

The printed arrow keys occupy the `;`, `.`, `,`, and `/` key positions. Speed and microphone volume changes take effect immediately and appear on screen.

## Robot power modes

The robot's **2 / On** setting supports the camera and audio. Use **1 / Run** to enable its motor. The [robot manual](https://fccid.io/m/236e138fda5134ed42783f05948c5547a5333b328ad25364203c9d2a4fb97770.pdf) notes that Run mode can shut off unless the robot is upright on a solid surface. If the camera works in 2 but disappears in 1, check that the robot remains powered and its Wi-Fi network is still visible.

Keep the robot in a clear area and its physical power switch within reach. Releasing the arrows sends zero movement, but that packet may not reach the robot if Wi-Fi or the Cardputer loses power.

## Troubleshooting

- **Movement stops:** movement pauses if robot status is missing for two seconds. Once it returns, release the arrows and press them again.
- **Speaker feedback:** press **-** to lower the microphone output level, or move the Cardputer farther from the robot. **Vol** shows the selected level; **Spd** shows movement speed.
- **Microphone is silent:** on the status screen, `p` is the captured peak, `r` is the sample range, and `tx` counts sent audio packets. If `r` stays at zero while speaking, the microphone is returning constant samples.
- **Video is slow or fails:** the camera view shows measured frames per second. Frames larger than 48 KiB are dropped with **Video frame too large**.
- **Robot Wi-Fi is missing:** press **R** to scan again. The robot IP is fixed at `192.168.10.123`, but the network name can vary. Only open `CL4P-TP...` networks are supported.

The firmware does not receive robot audio through the Cardputer speaker. Microphone audio is streamed while G0 is held; it is not saved.

## Development

Packet encoding, camera parsing, and voice filtering have tests that run on a computer:

```sh
c++ -std=c++11 -Wall -Wextra -Werror tests/protocol_test.cpp -o /tmp/claptrap-protocol-test
/tmp/claptrap-protocol-test
c++ -std=c++11 -Wall -Wextra -Werror tests/camera_frame_test.cpp -o /tmp/claptrap-camera-test
/tmp/claptrap-camera-test
c++ -std=c++11 -Wall -Wextra -Werror tests/voice_filter_test.cpp -o /tmp/claptrap-voice-filter-test
/tmp/claptrap-voice-filter-test
```

These tests do not replace a hardware check with the robot.

## Support

<a href="https://ko-fi.com/zuiko9"><img src="https://storage.ko-fi.com/cdn/brandasset/v2/support_me_on_kofi_blue.png" alt="Support me on Ko-fi" width="220"></a>

## License

MIT. See [LICENSE](LICENSE).
