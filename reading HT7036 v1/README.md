
## Description
Perangkat Monitoring Power 3 fase kompetibel protokol Modbus RTU


## Authors

- [@aldidwiputra9](https://github.com/aldidwiputra9)


## Features

- Backlight LCD
- Setting parameter calibration dapat menggunakan Button key atau Modbus 
- Pembacaan sensor Power 3 fase
- Pembacaan Nilai Energy Sensor Non-Volatile 

## Power Meter Configution
#### **1. FOR USER**
konfigurasi **_for user_** ditujukan untuk user karena skema kalibrasi nya hanya sebatas oprasi matematika saja tidak kalibrasi langsung ke IC HT7036, gain (perkalian) dan offset (pertambahan). untuk melakukan kalibrasi perlu menentukan channel phase mana yang akan digunakan sebagai referensi, setelah itu tentukan nilai kalibrasi dan pack ke dataframe modbus rtu sesuai dengan register dan function code yang tuju

**_example:_**
Kalibrasi _tegangan pada gain parameter-nya di channel phase A_ senilai 2x maka dataframe modbusnya adalah

* slave address: ```0x01```
* function code: ```0x06```
* register address: ```0x2007```
* value (```2*1000 = 2000```): ```0x07D0```

| slave addr | function code | register addr | value        | CRC          |
| ---------- | ------------- | ------------- | ------------ | ------------ |
| ```0x01``` | ```0x06```    | ```0x2007```  | ```0x07D0``` | ```0x3067``` |

#### 1.A List Calibration Voltage for User 

| Address (HEX) | Address (DEC) | Description               | Encode | Decode      |
| ------------- | ------------- | ------------------------- | ------ | ---------- |
| 2001          | 8193          | Offset Voltage A STM32    | ```value*1000``` | ```value/1000``` |
| 2002          | 8194          | Offset Voltage B STM32    | ```value*1000``` | ```value/1000``` |
| 2003          | 8195          | Offset Voltage C STM32    | ```value*1000``` | ```value/1000``` |
| 2007          | 8199          | Gain Voltage A STM32      | ```value*1000``` | ```value/1000``` |
| 2008          | 8200          | Gain Voltage B STM32      | ```value*1000``` | ```value/1000``` |
| 2009          | 8201          | Gain Voltage C STM32      | ```value*1000``` | ```value/1000``` |


#### 1.B List Calibration Current For User

| Address (HEX) | Address (DEC) | Description                     | Encode | Decode      |
| ------------- | ------------- | ------------------------------- | ------ | ---------- |
| 200A          | 8202          | Gain Current A STM32            | ```value*1``` | ```value/1``` |
| 200B          | 8203          | Gain Current B STM32            | ```value*1``` | ```value/1``` |
| 200C          | 8204          | Gain Current C STM32            | ```value*1``` | ```value/1``` |
| 2004          | 8196          | Offset Current A STM32          | ```value*1000``` | ```value/1000``` |
| 2005          | 8197          | Offset Current B STM32          | ```value*1000``` | ```value/1000``` |
| 2006          | 8198          | Offset Current C STM32          | ```value*1000``` | ```value/1000``` |

#### **2. FOR SUPER USER**
sedangkan pada konfigurasi **_for super user_** ditujukan untuk developer karena konfigurasi yang dilakukan langsung ke IC HT7036, serta skema kalibrasi yang berbeda dengan **_for user_**. untuk melakukan **_kalibrasi offset(zeroing)_** perlu menentukan channel phase referensi nilai data yang akan dikirim (```valueCommand=1``` >> auto zeroing, ```valueCommand!=1``` >> manual zeroing(unit data)), sedangkan untuk **_kalibrasi gain(multiplication)_** hanya perlu memasukkan data aktual pembacaan(dengan encode-nya yaitu ```value*100```)

**_example:_** 
* _kalibrasi offset_, kalibrasi tegangan offset menggunakan auto zeroing(```value=1```) dengan channel phase referensi nya yaitu A, maka dataframe sebagai berikut

| slave addr | function code | register addr | value        | CRC          |
| ---------- | ------------- | ------------- | ------------ | ------------ |
| ```0x01``` | ```0x06```    | ```0x1001```  | ```0x0001``` | ```0x1D0A``` |

* _kalibrasi gain_, kalibrasi tegangan gain yang memiliki data aktual 220.3(```value=220.3*100=22030[0x560E]```) dengan channel phase referensi nya yaitu A, maka dataframe sebagai berikut

| slave addr | function code | register addr | value        | CRC          |
| ---------- | ------------- | ------------- | ------------ | ------------ |
| ```0x01``` | ```0x06```    | ```0x1007```  | ```0x560E``` | ```0x82AF``` |

#### 2.A List Calibration Voltage For Super User

| Address (HEX) | Address (DEC) | Description                     | Encode | Decode      |
| ------------- | ------------- | ------------------------------- | ------ | ---------- |
| 1001          | 4097          | Offset Voltage A HT7036         | ```value*1``` | ```value*1``` |
| 1002          | 4098          | Offset Voltage B HT7036         | ```value*1``` | ```value*1``` |
| 1003          | 4099          | Offset Voltage C HT7036         | ```value*1``` | ```value*1``` |
| 1007          | 4103          | Gain Voltage A HT7036           | ```value*100``` | ```value/100``` |
| 1008          | 4104          | Gain Voltage B HT7036           | ```value*100``` | ```value/100``` |
| 1009          | 4105          | Gain Voltage C HT7036           | ```value*100``` | ```value/100``` |

#### C. List Calibration Current For Super User

| Address (HEX) | Address (DEC) | Description                     | Encode | Decode      |
| ------------- | ------------- | ------------------------------- | ------ | ---------- |
| 1004          | 4100          | Offset Current A HT7036         | ```value*1``` | ```value*1``` |
| 1005          | 4101          | Offset Current B HT7036         | ```value*1``` | ```value*1``` |
| 1006          | 4102          | Offset Current C HT7036         | ```value*1``` | ```value*1``` |
| 100A          | 4106          | Gain Current A HT7036           | ```value*100``` | ```value/100``` |
| 100B          | 4107          | Gain Current B HT7036           | ```value*100``` | ```value/100``` |
| 100C          | 4108          | Gain Current C HT7036           | ```value*100``` | ```value/100``` |

## Structure Menu in LCD
    Startup Device
    │
    ├── Monitoring Mode                      
    │   ├── Current Sensor (A)              
    │   ├── Voltage Sensor (V)
    │   ├── Voltage Sensor Dif (V)
    │   ├── Power Active (kW)
    │   ├── Power Readctive (Kvar)
    │   └── Power Factor 
    └── Setting Mode  
        ├── Multiplication Trafo
        │   └── Setting Value (0-9999)                           
        ├── Slave Address
        │   └── Setting Value (0-254)                             
        ├── Energy Reset 
        │   ├── no 
        │   └── yes                        
        └── Confirmation Setting
            ├── save 
            ├── cancel  
            └── back
    
## Mapping Register Power Meter

| Address (HEX) | Address (DEC) | Description                     | Size   | State      |
| ------------- | ------------- | ------------------------------- | ------ | ---------- |
| BD3 - BD4     | 3027 - 3028   | Voltage RMS A                   | 4 byte | Read       |
| BD5 - BD6     | 3029 -3030    | Voltage RMS B                   | 4 byte | Read       |
| BD7 - BD8     | 3031 - 3032   | Voltage RMS C                   | 4 byte | Read       |
| BDB - BDC     | 3035 - 3036   | Voltage RMS Combine             | 4 byte | Read       |
| BB7 - BB8     | 2999 - 3000   | Current RMS A                   | 4 byte | Read       |
| BB9 - BBA     | 3001 - 3002   | Current RMS B                   | 4 byte | Read       |
| BBB - BBC     | 3003 - 3004   | Current RMS C                   | 4 byte | Read       |
| BC1 BC2       | 3009 - 3005   | Current RMS Combine             | 4 byte | Read       |
| BED - BEE     | 3053 - 3054   | Active Power A                  | 4 byte | Read       |
| BEF - BF0     | 3055 - 3056   | Active Power B                  | 4 byte | Read       |
| BF1 - BF2     | 3057 - 3058   | Active Power C                  | 4 byte | Read       |
| BF3 - BF4     | 3059 - 3060   | Active Power Total              | 4 byte | Read       |
| BF5 - BF6     | 3061 - 3062   | Reactive Power A                | 4 byte | Read       |
| BF7 - BF8     | 3063 - 3064   | Reactive Power B                | 4 byte | Read       |
| BF9 - BFA     | 3065 - 3066   | Reactive Power C                | 4 byte | Read       |
| BFB - BFC     | 3067 - 3068   | Reactive Power Total            | 4 byte | Read       |
| BFD - BFE     | 3069 - 3070   | Apparent Power A                | 4 byte | Read       |
| BFF - C00     | 3071  - 3072  | Apparent Power B                | 4 byte | Read       |
| C01 - C02     | 3073 - 3074   | Apparent Power C                | 4 byte | Read       |
| C03 - C04     | 3075 - 3076   | Apparent Power Total            | 4 byte | Read       |
| C05 - C06     | 3077 - 3078   | Power Factor A                  | 4 byte | Read       |
| C07 - C08     | 3079 - 3080   | Power Factor B                  | 4 byte | Read       |
| C09 - C0A     | 3081 - 3082   | Power Factor C                  | 4 byte | Read       |
| C0B - C0C     | 3083 - 3084   | Power Factor Combine            | 4 byte | Read       |
| DBD - DC0     | 3517 - 3520   | Active Energy A                 | 8 Byte | Read/Write |
| DC1 - DC4     | 3521 - 3524   | Active Energy B                 | 8 Byte | Read/Write |
| DC5 - DC8     | 3525 - 3528   | Active Energy C                 | 8 Byte | Read/Write |
| C83 - C86     | 3203 - 3206   | Active Energy Total             | 8 Byte | Read       |
| DC9 - DCC     | 3529 - 3532   | Reactive Energy A               | 8 Byte | Read/Write |
| DCD - DD0     | 3533 - 3536   | Reactive Energy B               | 8 Byte | Read/Write |
| DD1 - DD4     | 3537 - 3540   | Reactive Energy C               | 8 Byte | Read/Write |
| C93 - C96     | 3219 - 3222   | Reactive Energy Total           | 8 Byte | Read       |
| BCB - BCC     | 3019 - 3020   | Voltage RMS AB                  | 4 byte | Read       |
| BCD - BCE     | 3021 - 3022   | Voltage RMS BC                  | 4 byte | Read       |
| BCF - BD0     | 3023 - 3024   | Voltage RMS CA                  | 4 byte | Read       |
| 5351 - 5352   | 21329 - 21330 | Total Harmonic Distortion A     | 4 byte | Read       |
| 5353 - 5354   | 21331 - 21332 | Total Harmonic Distortion B     | 4 byte | Read       |
| 5355          | 21333 - 21334 | Total Harmonic Distortion C     | 4 byte | Read       |
| 1000          | 4096          | Slave ID Modbus                 | 2 Byte | Read/Write |
| 1001          | 4097          | Offset Voltage A HT7036         | 2 Byte | Read/Write |
| 1002          | 4098          | Offset Voltage B HT7036         | 2 Byte | Read/Write |
| 1003          | 4099          | Offset Voltage C HT7036         | 2 Byte | Read/Write |
| 1004          | 4100          | Offset Current A HT7036         | 2 Byte | Read/Write |
| 1005          | 4101          | Offset Current B HT7036         | 2 Byte | Read/Write |
| 1006          | 4102          | Offset Current C HT7036         | 2 Byte | Read/Write |
| 1007          | 4103          | Gain Voltage A HT7036           | 2 Byte | Read/Write |
| 1008          | 4104          | Gain Voltage B HT7036           | 2 Byte | Read/Write |
| 1009          | 4105          | Gain Voltage C HT7036           | 2 Byte | Read/Write |
| 100A          | 4106          | Gain Current A HT7036           | 2 Byte | Read/Write |
| 100B          | 4107          | Gain Current B HT7036           | 2 Byte | Read/Write |
| 100C          | 4108          | Gain Current C HT7036           | 2 Byte | Read/Write |
| 2001          | 8193          | Offset Voltage A STM32          | 2 Byte | Read/Write |
| 2002          | 8194          | Offset Voltage B STM32          | 2 Byte | Read/Write |
| 2003          | 8195          | Offset Voltage C STM32          | 2 Byte | Read/Write |
| 2004          | 8196          | Offset Current A STM32          | 2 Byte | Read/Write |
| 2005          | 8197          | Offset Current B STM32          | 2 Byte | Read/Write |
| 2006          | 8198          | Offset Current C STM32          | 2 Byte | Read/Write |
| 2007          | 8199          | Gain Voltage A STM32            | 2 Byte | Read/Write |
| 2008          | 8200          | Gain Voltage B STM32            | 2 Byte | Read/Write |
| 2009          | 8201          | Gain Voltage C STM32            | 2 Byte | Read/Write |
| 200A          | 8202          | Gain Current A STM32            | 2 Byte | Read/Write |
| 200B          | 8203          | Gain Current B STM32            | 2 Byte | Read/Write |
| 200C          | 8204          | Gain Current C STM32            | 2 Byte | Read/Write |
| 3001          | 12289         | Offset Voltage STM32[Raw Byte]  | 2 Byte | Read       |
| 3002          | 12290         | Offset Current STM32 [Raw Byte] | 2 Byte | Read       |
| 3003          | 12291         | Gain Voltage STM32 [Raw Byte]   | 2 Byte | Read       |
| 3004          | 12292         | Gain Current STM32 [Raw Byte]   | 2 Byte | Read       |

## Pinout STM32F030C8T6

| PIN | PIN NAME              | FUNCTION              | DESCRIPTION       |
| --- | --------------------- | --------------------- | ----------------- |
| 1   | VDD                   | SUPPLY                | 3.3V              |
| 2   | PC13                  | \-                    | \-                |
| 3   | PC14-OSC32_IN (PC14)  | OSC32_IN              | OSILATOR_IN       |
| 4   | PC15-OSC32_OUT (PC15) | OSC32_OUT             | OSILATOR_OUT      |
| 5   | PF0-OSC_IN (PF0)      | \-                    | \-                |
| 6   | PF1-OSC_OUT (PF1)     | \-                    | \-                |
| 7   | NRST                  | RESET                 | RESET             |
| 8   | VSSA                  | Analog ground         | GND               |
| 9   | VDDA                  | Analog power supply   | 3.3V              |
| 10  | PA0                   | \-                    | \-                |
| 11  | PA1                   | OUT                   | EN_MODBUS(RS485)  |
| 12  | PA2                   | TX_UART               | RX_MODBUS (RS485) |
| 13  | PA3                   | RX_UART               | TX_MODBUS (RS485) |
| 14  | PA4                   | DIGITAL_OUT           | CS1_HT1622        |
| 15  | PA5                   | SPI1_SCK              | SCK_HT1622        |
| 16  | PA6                   | \-                    | \-                |
| 17  | PA7                   | SPI1_MOSI             | MOSI_HT1622       |
| 18  | PB0                   | DIGITAL_OUT           | CS2_HT1622        |
| 19  | PB1                   | \-                    | \-                |
| 20  | PB2                   | \-                    | \-                |
| 21  | PB10                  | I2C1_SCL              | SCL_EEPROOM       |
| 22  | PB11                  | I2C1_SDA              | SDA_EEPROOM       |
| 23  | VSS                   | GROUND                | GND               |
| 24  | VDD                   | SUPPLY                | 3.3V              |
| 25  | PB12                  | \-                    | \-                |
| 26  | PB13                  | SPI2_SCK              | SCK_HT7036        |
| 27  | PB14                  | SPI2_MISO             | DOUT_HT7036       |
| 28  | PB15                  | SPI2_MOSI             | DIN_HT7036        |
| 29  | PA8                   | DIGITAL_OUT           | CS_HT7036         |
| 30  | PA9                   | \-                    | \-                |
| 31  | PA10                  | \-                    | \-                |
| 32  | PA11                  | \-                    | \-                |
| 33  | PA12                  | \-                    | \-                |
| 34  | PA13 (SWDIO)          | SWDIO                 | PROGRAM UPLOAD    |
| 35  | PF6                   | \-                    | \-                |
| 36  | PF7                   | \-                    | \-                |
| 37  | PA14                  | SWCLK                 | PROGRAM UPLOAD    |
| 38  | PA15                  | DIGITAL_IN            | KEY_4             |
| 39  | PB3                   | DIGITAL_IN            | KEY_3             |
| 40  | PB4                   | DIGITAL_IN            | KEY_2             |
| 41  | PB5                   | DIGITAL_OUT           | BACKLIGHT_EN      |
| 42  | PB6                   | DIGITAL_IN            | KEY_1             |
| 43  | PB7                   | \-                    | \-                |
| 44  | BOOT0                 | Boot memory selection | R 10K TO GND      |
| 45  | PB8                   | \-                    | \-                |
| 46  | PB9                   | \-                    | \-                |
| 47  | VSS                   | GROUND                | ground            |
| 48  | VDD                   | SUPPLY                | 3.3V              |