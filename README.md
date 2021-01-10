# [Complex Numbers & Quaternions](https://github.com/halsw/Quaternions) for Arduino and Teensy
A library with template classes for quaternions and complex numbers

The library caters for mixed basic operations and mixed functions with support for scalars, complex numbers, quaternions and limited support for vectors. The functions are overloaded versions of the [MathFixed](https://github.com/halsw/MathFixed) library so it can be used with double, float or any fixed point number type.

**CAUTION** Vectors and Matrices are accessed directly (not through class methods) so that either raw arrays or any matrix library can be used as long as it stores elements in array format with no additional attributes. 

## Example file
The included .ino file provides a limited example of the full capabilities and uses fixed point numbers with 7 integer bits and 8 fractional. You may change the type of numbers used by changing the statement *#define TFixed*

To compile the .ino file you must first install the [MathFixed](https://github.com/halsw/MathFixed),  **FixedPoints** and **BasicLinearAlgebra** libraries

 