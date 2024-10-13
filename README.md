# BME280 sensor API
## Summary
This project adds ESP32-IDF I2C driver support for the BME280 API.

This is a fork of https://github.com/boschsensortec/BME280_SensorAPI.

## Installation
To use the library in your ESP32-IDF project, perform the following steps:

1. Create a directory called `components` in your main project directory.
2. Change into the `components` directory.
3. Run `git clone https://github.com/jalius/bme280_esp32.git` to bring in the latest copy of this library.

If your project itself is a git repository, you should consider using `git submodule add` instead of cloning.

For more info about components in the ESP32-IDF build system, [read the docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html).
## Details
### Sensor overview

BME280 is a combined digital humidity, pressure and temperature sensor based on proven sensing principles.
Its small dimensions and its low power consumption allow the implementation in battery driven devices such as
handsets, GPS modules or watches.

### Target Application
- Context awareness, e.g.skin detection, room change detection.
- Fitness monitoring / well-being
- Home automation control
- Internet of things
- GPS enhancement(e.g.time-to-first-fix improvement, dead reckoning, slope detection)
- Indoor navigation(change of floor detection, elevation detection)
- Outdoor navigation, leisure and sports applications
- Weather forecast
- Vertical velocity indication(rise/sink speed)

### Feature
- Pressure
- Temperature
- Humidity

### Important links

- [BME280 product page](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
- [BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [BME280 shuttle board flyer](https://www.bosch-sensortec.com/media/boschsensortec/downloads/shuttle_board_flyer/application_board_3_1/bst-bme280-sf000.pdf)
- [Community support page](https://community.bosch-sensortec.com)# bme280_esp32
