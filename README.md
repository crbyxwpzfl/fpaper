#### https://github.com/javl/image2cpp - the dithering is so cool javl



# fpaper


TODO - in index.html make images open with constant width

TODO - with custom partition merged.bin is broken you manually have to falsh main bin to app0 offset

this is the current custom partition no spiffs!!

```
| Type | Sub |  Offset  |   Size   |       Label      | Size (MB) |
| ---- | --- | -------- | -------- | ---------------- | --------- |
|  01  | 02  | 0x009000 | 0x200000 | nvs              | 2.0 MB    |
|  01  | 00  | 0x209000 | 0x002000 | otadata          | 0.008 MB  |
|  00  | 10  | 0x210000 | 0x200000 | app0             | 2.0 MB    |
|  00  | 11  | 0x410000 | 0x200000 | app1             | 2.0 MB    |
|  01  | 03  | 0x610000 | 0x010000 | coredump         | 0.0625 MB |

-- total ~6.07/8 MB (~1.93 MB free)
```

previously we had minispiffs
```
Label       Offset     Size (hex)   Size (bytes)    Size (KB)   Size (MB)
nvs         0x009000   0x005000     20,480          20.0        0.0195
otadata     0x00E000   0x002000     8,192           8.0	        0.0078
app0        0x010000   0x1E0000     1,966,080       1,920.0     1.875
app1        0x1F0000   0x1E0000     1,966,080       1,920.0     1.875
spiffs      0x3D0000   0x020000     131,072         128.0       0.125
coredump    0x3F0000   0x010000     65,536          64.0        0.0625

Region          Start       End         Size (bytes)    Size (MB)
Bootloader      0x0000      0x8000      32,768          0.03125
Partition Table 0x8000      0x9000      4,096           0.0039
All partitions  0x9000      0x400000    4,142,848       ~3.95
Unused          0x400000    0x800000    4,194,304       4.0
```

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

check board
```
arduino-cli board details -b esp32:esp32:esp32s3:PartitionScheme=custom,PSRAM=opi,FlashMode=qio,FlashSize=8M
```

compile with
```
arduino-cli compile -v --fqbn esp32:esp32:esp32s3:PartitionScheme=custom,PSRAM=opi,FlashMode=qio,FlashSize=8M --build-path ./firmware
```

'merged.bin' at adress 0x0 with https://espressif.github.io/esptool-js/ for web programming
