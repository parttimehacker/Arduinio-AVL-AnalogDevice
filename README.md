# Arduino class to provide basic analog sensor methods

Design Goals: This library is designed to be...

* Create a high level analog sensor class
* Hide some of the details of analog sensor implementation
* Create simple count to millivolt to engineering unit conversion
* Engineering units are obtain from a simple linear curve fit

NOTE - Reading error can be produced if multiple ADC reads are 
performed on multiple pins. The solution is to read one sample, 
throw it away and then take multiple samples on the same pin to 
get an average reading.

MIT License.