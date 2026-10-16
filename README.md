# OnePlus 13R / Ace 5 Mainline

## Support

| Feature              | Description             | State |
|:---------------------|:------------------------|:-----:|
| Internal storage     |                         | ✅    |
| Side buttons         |                         | ✅    |
| Mute slider          | AK09970 hall + userspace daemon | ✅    |
| Proximity sensor     | SSC tcs3720 via FastRPC/libssc | ✅    |
| Light sensor         | TCS3720 via vendor wise_rgb + giulia-brightnessd | ✅ |
| Accelerometer        | SSC icm456xx via FastRPC/libssc | ✅    |
| Gyroscope            | SSC icm456xx via libssc | ✅    |
| Magnetometer         | SSC mmc56x3x via FastRPC/libssc | ✅    |
| Fingerprint          |                         | ❌    |
| NFC                  | nxp-nci                 | ✅    |
| Thermals             |                         | ✅    |
| Battery              |                         | ✅    |
| USB host             |                         | ✅    |
| USB device           |                         | ✅    |
| USB power delivery   |                         | ✅    |
| Charging             |                         | ✅    |
| WLAN                 | MAC from serial (systemd .link) | ✅    |
| CPU                  |                         | ✅    |
| Gunyah               |                         | ✅    |
| Touchscreen          | Off-tree                | ✅    |
| Bluetooth            | Uses controller default MAC | ✅    |
| GPS                  |                         | ❌    |
| Speakers             | Off-tree                | ✅    |
| Microphones          | WCD9395 analog mics: bottom(main)+top(sec) stereo, headset (UCM) | ✅    |
| GPU                  |                         | ✅    |
| Camera               | Ultrawide IMX355 via camss (RAW capture); main/macro no driver | ⚠️ |
| Flash                | pm8350c flash-led       | ✅    |
| Calls                | VoLTE: PDC profile active; needs AP IMS stack (OpenIMSd) | ❌ |
| SMS                  | Receive works; send fails (QMI WMS 56) | ⚠️ |
| Mobile Data          | IPA sm8650 consumer endpoints 17/22/24 | ✅ |
| Display              | Off-tree                | ✅    |
| Haptics              | driver + userspace event daemon | ✅    |