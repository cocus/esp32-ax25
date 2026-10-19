# esp32-afsk-modem

A 1200 baud AFSK (Bell 202) modem for the ESP32 that uses the chip's own ADC and DAC, no
codec needed. It speaks KISS over TCP, so a Linux box on the same network can use it as an
AX.25 interface and run IP over the radio. The idea is a small mesh for TCP traffic, a bit
like Meshtastic but on plain packet radio.

What's in here:

- the modem itself: demodulator, modulator, HDLC, carrier detect and KISS-style channel access
- a KISS TCP server on port 8001
- a debug console on port 2323 and an audio tap on port 2324
- a small web page with an ADC scope, status and recent frames
- a PC test that runs the same modem code against simulated audio or captures from the board

## Hardware

| Signal  | Pin    | Notes |
|---------|--------|-------|
| RX audio | GPIO36 (ADC1_CH0) | Bias to about 1.6 V (two 10k resistors) and AC couple the audio in through ~1 uF. |
| TX audio | GPIO25 (DAC1) | Idles at mid scale (~1.65 V). AC couple it and divide it down for a mic input. An RC low pass around 5 kHz smooths the DAC steps. |
| PTT     | GPIO33 | Active low. |

For a quick test without a radio, just put a wire between GPIO25 and GPIO36. The DAC already
sits at mid scale, so no parts are needed, and every frame the board sends comes back in.

## Building

This is a PlatformIO project on ESP-IDF 6.1 (platform `espressif32@8.0.0`).

1. Copy `src/secrets.example.h` to `src/secrets.h` and put your WiFi network in it.
2. Over USB the first time: `pio run -e esp32dev -t upload`, then `pio run -e esp32dev -t uploadfs`
   for the web page.
3. After that everything goes over WiFi: `pio run -e esp32dev_ota -t upload` for the firmware
   (the board reboots) and `pio run -e esp32dev_ota -t uploadfs` for the web files (no reboot).
   The board's address is in `platformio.ini` (`upload_flags`), change it to yours.

Both go to the same `POST /update` URL. The board tells them apart by the data: firmware
starts with the ESP image magic byte, a filesystem image is exactly the size of the `www`
partition. Anything else gets refused. To be explicit, add `?target=app` or `?target=www`.

`src/main.c` has the pin numbers, ports and a test beacon (an AX.25 UI frame every 30 s, set
`BEACON_INTERVAL_MS` to 0 to turn it off). Use your own callsign before putting it on the air.

## Using it

### Web page

`http://<board>/` shows the raw ADC as a scope, the levels and counters, and the last frames.
There's also a box to send some text as a frame.

The page lives in `data/` (plain HTML, JS and CSS) and is served from the `www` SPIFFS
partition, so changing it only needs an `uploadfs`. The data comes from `/api/scope`
(raw ADC codes), `/api/status` (JSON) and `POST /api/send`, in `src/web_api.c`.

### Debug console

`nc <board> 2323` (or telnet), then `help`. Frames and TX events always show up; the rest
you turn on:

    stats                  sample rate, ADC range and noise, tone levels, carrier detect,
                           channel parameters, KISS counters, drops, heap, WiFi signal
    frames [n]             last frames received, including bad CRC ones that had a signal
    level on|off           once a second: ADC range and rms, tone levels, flags/s, carrier
    bad on|off             show frames that fail the CRC as they come in
    log on|off             copy the ESP log into the console
    send <text>            send the text as one frame
    tx <tnc2>              send an AX.25 UI frame, e.g. tx N0CALL>TEST:hello
    tone mark|space|alt [ms]   key up with a steady tone, for setting deviation
    tap 48k|12k [secs]     choose what the audio port sends, see below

Some things worth knowing when reading it:

- The sample rate should say about 48000. If it doesn't, nothing else will work.
- The ADC range shouldn't touch 0 or 4095. If it does you'll see `CLIPPING`.
- Flags per second sit around 5 on noise and jump to about 150 during a preamble. If they
  jump but nothing decodes, the signal is getting there and something else is off (levels,
  twist, clipping).
- Mark and space levels far apart (more than 6 to 10 dB) usually means pre or de-emphasis.
- Bad CRC frames show up on plain noise too. The `frames` list only keeps the ones that had
  a real signal.

### Audio tap

`nc <board> 2324 > capture.wav` streams a WAV of what the demodulator hears, until you
disconnect. Live streaming needs 96 kB/s at 48 kHz or 24 kB/s at 12 kHz, which a weak WiFi
link won't keep up with. In that case type `tap 12k 3` in the console first: the board then
records 3 seconds into RAM and sends them afterwards, at whatever speed the link manages.
At 12 kHz about 3 s fits in memory, at 48 kHz less than 1 s.

### Linux and KISS

The board is a KISS TNC on TCP port 8001, the same convention Dire Wolf uses. With the
kernel AX.25 modules and `ax25-tools` installed:

    # add this to /etc/ax25/axports:
    # radio  N0CALL-1  1200  256  2  esp32 afsk modem
    socat pty,link=/tmp/kiss,raw,echo=0 tcp:<board>:8001 &
    sudo kissattach $(readlink /tmp/kiss) radio 44.0.0.1      # creates ax0
    sudo ip link set ax0 mtu 256
    sudo kissparms -p radio -t 300 -r 63 -s 100    # TX delay ms, persistence, slot ms
    axlisten -a                                    # watch the channel

Before sending, the board waits until it hears no carrier, then goes with a chance of
(P+1)/256 per slot. With the usual P=63 and 100 ms slots that adds about 300 ms on average.
On a link with just two nodes, `kissparms -r 255` gets rid of it. Frames that pile up while
waiting go out together and share one TX delay.

At 1200 baud a 256 byte frame is about 2 s on the air. TCP works, but keep the MTU small and
expect long round trips.

## Testing on the PC

`test/test_modem.c` builds the modem's portable parts (demodulator, modulator, AX.25) and
runs them through simulated audio: clock error, twist, noise, DC offset and 12-bit
quantisation. Build it with clang or gcc (gcc also wants `-lm`):

    clang -O2 -Icomponents/afsk_modem/include test/test_modem.c \
        components/afsk_modem/src/afsk_demod.c components/afsk_modem/src/afsk_mod.c \
        components/afsk_modem/src/ax25.c -o test/test_modem

    test/test_modem                                 # all the scenarios, PASS or FAILED
    test/test_modem --wav capture.wav --trace t.csv # decode a capture from the board
    test/test_modem --make-wav sim.wav              # make a capture in the board's format
    python test/plot_trace.py t.csv 12.3 0.3        # plot 0.3 s starting at 12.3 s

The replay prints every frame it finds, good or bad, and the trace has the filtered audio,
tone levels, demod output, bit clock, bit sample points and carrier detect, one row per
12 kHz sample.
