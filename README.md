#### https://github.com/javl/image2cpp - the dithering is so cool javl



# fpaper


TODO - in index.html make images open with constant width

this is the current custom partition no spiffs!! also always set app0 at 0x10000 otherwise merged.bin is broken
```
| Name     | Type  | SubType  | Offset   | Size     | Size (MB)   |
|----------|-------|----------|----------|----------|-------------|
| otadata  | data  | ota      | 0x9000   | 0x2000   | 0.00781 MB  |
| app0     | app   | ota_0    | 0x10000  | 0x200000 | 2.00000 MB  |
| app1     | app   | ota_1    | 0x210000 | 0x200000 | 2.00000 MB  |
| coredump | data  | coredump | 0x410000 | 0x10000  | 0.06250 MB  |
| nvs      | data  | nvs      | 0x420000 | 0x200000 | 2.00000 MB  |

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

/home/codespace/.arduino15/packages/esp32/hardware/esp32/3.2.0/tools/partitions


arduino-cli compile -v --fqbn esp32:esp32:esp32s3:PartitionScheme=custom,CustomPartitions=partitions,PSRAM=opi,FlashMode=qio,FlashSize=8M --build-path ./firmware


cp /workspaces/fpaper/test/partitions.csv /home/codespace/.arduino15/packages/esp32/hardware/esp32/3.2.0/tools/partitions/custom.csv


'merged.bin' at adress 0x0 with https://espressif.github.io/esptool-js/ for web programming





/home/codespace/.arduino15/packages/esp32/tools/esptool_py/4.9.dev3/esptool --chip esp32s3 partition_table --partition-table-file partitions.csv




esptool.py --chip esp32s3 merge_bin -o merged.bin --flash_mode qio --flash_freq 80m --flash_size 8MB \
  0x0 firmware/test.ino.bootloader.bin \
  0x8000 firmware/test.ino.partitions.bin \
  0x9000 nvs_data.bin \
  0x210000 firmware/test.ino.bin




/home/codespace/.arduino15/packages/esp32/tools/esptool_py/4.9.dev3/esptool --chip esp32s3 merge_bin -o merged-manual.bin --flash_mode qio --flash_freq 80m --flash_size 8MB \
  0x0 firmware/test.ino.bootloader.bin \
  0x8000 firmware/test.ino.partitions.bin \
  0x210000 firmware/test.ino.bin