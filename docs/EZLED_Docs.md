# 3256 EZ-LED Framework:
Documentation on how to use our EZ-LED Framework
## Quickstart
* Plug in your AddressableLED into 5V, GND, PWM port
* Create a EZLED object in RobotContainer
* Create LED patterns using our LED pattern bases. Use the LED patterns under our patterns folder as examples.
* Attach your custom LED patterns to sections using the LEDSetSectionPattern command.
* Have fun!!
### patternBases
* LED Pattern
* Animated Pattern
* Blinking Pattern
* Time Function Pattern
### LED Section:
* Holds the LEDPattern that will be displayed
* Renders a pattern by scaling it to it's length
### LED Subsystem:
* A collection of disjoint sections
* Each section displays a user controlled pattern
### Note:
* pixels are NOT LEDs
* patterns are NOT LEDPatterns
* Most classes and methods have java docs
