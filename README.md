# fpaper


TODO 
how to set partition sheme what is huge app required for
what is/does psram OPIPsram what is it required for
what is/does Flash size 8MB(64Mb) how to use this with web flasher


install arduino cli
```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/home/codespace/.local/bin sh
```

allow lib form git links
```
export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
```


init arduino sketch
```
arduino-cli sketch new "test"
```

update core index
```
arduino-cli core update-index  --additional-urls "https://espressif.github.io/arduino-esp32/package_esp32_index.json"
```

find core paltform
```
arduino-cli board search esp32
```

install core platform 
```
arduino-cli core install esp32:esp32
```

find board fqbn
```
arduino-cli board search ESP32S3 Dev Module
```

compile with
```
arduino-cli compile -v --fqbn esp32:esp32:esp32s3 --build-path ./firmware
```

'merged.bin' at adress 0x0 with https://espressif.github.io/esptool-js/ for web programming
