# LIS2DUXS12
Arduino library to support the LIS2DUXS12 3D accelerometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LIS2DUXS12Sensor Accelero(&dev_i2c);
    Accelero.begin();
    Accelero.Enable_X();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LIS2DUXS12Sensor Accelero(&dev_spi, CS_PIN);  
    Accelero.begin();
    Accelero.Enable_X();

An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):  
    LIS2DUXS12Sensor Accelero(&I3C, LIS2DUXS12_I3C_ADD_H);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(Accelero.getStaticAddress(), LIS2DUXS12_DYNAMIC_ADDRESS);
    Accelero.begin(LIS2DUXS12_DYNAMIC_ADDRESS);
    I3C.setClock(12500000);
    Accelero.Enable_X();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):  

    LIS2DUXS12Sensor Accelero(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LIS2DUXS12_I3C_PID_H in discovered devices
    Accelero.begin(dynAddr);
    I3C.setClock(12500000);
    Accelero.Enable_X();

The access to the sensor values is done as explained below:  

  Read accelerometer.  

    int32_t accelerometer[3];
    Accelero.Get_X_Axes(accelerometer);  

## Examples

* LIS2DUXS12_DataLog_Terminal_I2C: This application shows how to get data from LIS2DUXS12 accelerometer and print them on terminal over I2C.

* LIS2DUXS12_Datalog_Terminal_I3C: This application shows how to use LIS2DUXS12 accelerometer over I3C using SETDASA.

* LIS2DUXS12_Datalog_Terminal_I3C_ENTDAA: This application shows how to discover and use LIS2DUXS12 dynamic address over I3C.

* LIS2DUXS12_6D_Orientation_I2C: This application shows how to use LIS2DUXS12 accelerometer to find out the 6D orientation and display data on a hyperterminal.

* LIS2DUXS12_Double_Tap_Detection_I2C: This application shows how to detect the double tap event using the LIS2DUXS12 accelerometer.

* LIS2DUXS12_FIFO_Polling_I2C: This application shows how to get accelerometer data from FIFO in pooling mode and print them on terminal.

* LIS2DUXS12_FIFO_Interrupt_I2C: This application shows how to get accelerometer data from FIFO using interrupt and print them on terminal.

* LIS2DUXS12_Free_Fall_Detection_I2C: This application shows how to detect the free fall event using the LIS2DUXS12 accelerometer.

* LIS2DUXS12_MLC_I2C: This application shows how to detect the activity using the LIS2DUXS12 Machine Learning Core.

* LIS2DUXS12_Pedometer_I2C: This application shows how to use LIS2DUXS12 accelerometer to count steps.

* LIS2DUXS12_Qvar_Polling_I2C: This application shows how to use LIS2DUXS12 Qvar features in polling mode.

* LIS2DUXS12_Single_Tap_Detection_I2C: This application shows how to detect the single tap event using the LIS2DUXS12 accelerometer.

* LIS2DUXS12_Tilt_Detection_I2C: This application shows how to detect the tilt event using the LIS2DUXS12 accelerometer.

* LIS2DUXS12_Triple_Tap_Detection_I2C: This application shows how to detect the triple tap event using the LIS2DUXS12 accelerometer.

* LIS2DUXS12_Wake_Up_Detection_I2C: This application shows how to detect the wake-up event using the LIS2DUXS12 accelerometer.

## Documentation

You can find the source files at  
https://github.com/stm32duino/LIS2DUXS12

The LIS2DUXS12 datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/accelerometers/LIS2DUXS12.html
