# AutomationShield Library and Examples

## About AutomationShield

Arduino library for the AutomationShield Arduino expansion boards for control engineering education. You can read more about the AutomationShield project in our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki).

## Installing the library

### Arduino IDE

Download the library from the `master` branch of the [Git repository](https://github.com/gergelytakacs/AutomationShield/archive/master.zip). Open the Arduino IDE, click on the *Sketch* menu, and find the *Include Library* option. This opens another sub-menu from which you shold select *Add .ZIP Library...* that will open the file browser. Locate the renamed library .zip file and click *Open*.

## Library usage

The harware library for a particular board is initialized by the board name followed by `begin` for example to initialize the OptoShield call:
```
OptoShield.begin();
```
Please consult our [Wiki page](https://github.com/gergelytakacs/AutomationShield/wiki) for details.

## Examples

The library contains examples in the `\examples\` directory that will also show up in the Examples collection of the Arduino IDE. You can access this by clicking *File*, *Examples* and look for the *AutomationShield* library and its examples.

Examples showcase the use of the AutomationShield library in Arduino sketches.

Lorem ipsum
