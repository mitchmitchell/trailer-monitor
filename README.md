# TrailerMonitor

An IoT monitoring system based on the Particle Electron and Particle Asset Tracker kit.

The hardware is based on the Particle Electron microprocessor cellular board (now superseded by the next generation Particle boron).  The board has an STM32 microprocessor and a cellular radio to access the network. https://www.particle.io/cellular/

The GPS unit is the Particle Asset Tracker which includes a GPS unit, antenna, battery, carrier board, and waterproof box.
https://www.particle.io/products/hardware/asset-tracker/

I added an Adafruit DHT22 temperature and humidity sensor to track how extreme the inside environment gets during the peak of summer and winter.

The software is based on several example and library modules available on the particle web site.  I've placed it into a GitHub repository that is publicly accessible in case anyone wants to duplicate the build. https://github.com/mitchmitchell/trailer-monitor

The device monitors A/C power to the trailer to let me know if the GCFI the trailer is plugged into cuts out so I can turn off the battery cut off switch until power is restored.  The asset tracker board provides GPS and an accelerometer that will let me know if the trailer is jostled.  Its sensitive enough to detect someone entering the trailer or hitching it up.  The temperature sensor warns me if I need to start thinking of winterizing the trailer.

Actual notification is handled by a Raspberry Pi running node-red using nodes which connect to the Particle cloud to receive data.  No ports or anything need be opened on the network where the Raspberry Pi resides.   The node-red program is very specific to my network so you might want to build your own.  It connects to Twilio to send SMS message alerts and to our Google Home to announce issues audibly.

The monitor has saved our batteries on several occasions already paying for itself so we've been quite happy with it and it definitely helps our peace of mind.

## A Particle project

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for NewTrailerMonitor.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h` and `.cpp` files for your library there. All contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
