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
| Fingerprint          | `uff_jv` TA loads and INIT succeeds; sensor instance blocked by single-slot QSEEComCompat app-loader (needs multi-region container) | ❌ |
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
| GPS                  | QMI LOC master shim -> NMEA socket -> Geoclue/KDE; satellite fix works (accuracy metadata still 0) | ⚠️   |
| Speakers             | Off-tree                | ✅    |
| Microphones          | WCD9395 analog mics: bottom(main)+top(sec) stereo, headset (UCM) | ✅    |
| GPU                  |                         | ✅    |
| Camera               | Ultrawide IMX355 + main IMX906 (RAW 4096x3072 via CSIPHY1/CSID0/VFE0); front IMX480 driver probes but no CSI frames; 2MP macro OV02B10 DT added (CSIPHY3/CSID2/VFE2) | ⚠️ |
| Flash                | pm8350c flash-led       | ✅    |
| Calls                | VoLTE/IMS registration + PDN work; in-call audio still missing (AGM voice session) | ⚠️ |
| SMS                  | Send/receive over IMS works (DCM/OpenIMSd; self-SMS end-to-end tested) | ✅ |
| Mobile Data          | IPA sm8650 consumer endpoints 17/22/24 | ✅ |
| Display              | Off-tree                | ✅    |
| Haptics              | driver + userspace event daemon | ✅    |
