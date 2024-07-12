# HX711 non-blocking SPI (STM32G431KB)
 An STM32 HAL example of reading the Avia Semiconductor HX711 24-bit ADC for load cells / weight scales / tensometric beams.

# Gain selection
![AVIA Semiconductor HX711 24-Bit Analog-to-Digital Converter (ADC) for Weigh Scales: Fig.2 Data output, input and gain selection timing and control](/Assets/Images/hx711_gain_selection.JPG)
In the example MOSI is used as CLK to produce the required number of cycles/pulses for HX711. It enables straightforward use of DMA just by calling HAL_SPI_TransmitReceive_DMA().

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

Before:

![screenshot of the imported CubeIDE project](/Assets/Images/before.JPG)

After:

![screenshot after generating the code and building the project](/Assets/Images/after.JPG)

# Sources and inspirations
* MOSI as CLK for HX711: My original idea, i.e. not copied from other sources. It doesn't mean that I'm the first one to do it this way - probably I'm not. I just didn't came across such a solution in other projects known to me.
* OLED: [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) (MIT license)

# Call for action
Build your own [home laboratory/workshop](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), and many other professional hobbyists sharing their awesome projects and tutorials! Shoutout/kudos to all of them!

:warning: WARNING: Control engineering - try this at home!

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
