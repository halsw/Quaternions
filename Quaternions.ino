/*
 * This file is part of the quaternions library
 * Usage: Provide an example use of the library
 * 
 * Version 1.0.0
 * Developed by Evan https://github.com/halsw
 *
 * Dependencies: MathFixed library (https://github.com/halsw/MathFixed)
 *               FixedPoints library (https://github.com/Pharap/FixedPointsArduino)
 *               BasicLinearAlgebra library (https://github.com/tomstewart89/BasicLinearAlgebra)
 *               
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "MathFixed.h"
#include "Quaternions.h"
#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

//Define the type used for calculations
#define TFixed SQ7x8

#define PERIOD_MS 1500

Complex<TFixed> c={1,1}; 
Quaternion<TFixed> r={1,1,1,1};
ArrayMatrix<4,4,TFixed> mr;
ArrayMatrix<3,3,TFixed> rot;
BLA::Matrix<2,2,Array<2,2,Complex<TFixed>>> cr;
ArrayMatrix<3,1,TFixed> v={1,0,0};

void setup() {
  Serial.begin(115200);
  fxibits<TFixed>(7); //Define integer bits for the MathFixed library
  fxfbits<TFixed>(8); //Define fractional bits for the MathFixed library
  randomSeed(analogRead(0)); //assuming A0 is not connected
}

void loop() {
  Complex<TFixed> ca, cb;
  Quaternion<TFixed> a, b;
  ca=fxrandom(c);
  cb=fxrandom(c);
  Serial.print("Complex number basic operations ie multiplication "); Serial.print(ca);
  Serial.print(" x "); Serial.print(cb);
  Serial.print(" = "); Serial.println(ca*cb);

  Serial.print("\nComplex number functions ie pow "); Serial.print(ca);
  Serial.print(" ^ "); Serial.print(cb);
  Serial.print(" = "); Serial.println(fxpow(ca, cb));
 
  a=fxrandom(r);
  b=fxrandom(r);
  Serial.print("\nQuaternion basic operations ie division "); Serial.print(a);
  Serial.print(" / "); Serial.print(b);
  Serial.print(" = "); Serial.println(a/b);

  Serial.print("\nQuaternion functions ie log("); Serial.print(a);
  Serial.print(") = "); Serial.println(fxlog(a));

  Serial.print("\nMixed scalar/complex numbers/quaternion basic operations ie "); Serial.print(a);
  Serial.print(" / "); Serial.print(cb);
  Serial.print(" + 2 = "); Serial.println(a/cb+2.0);

  Serial.print("\nMixed scalar/complex numbers/quaternion functions ie pow "); Serial.print(ca);
  Serial.print(" ^ "); Serial.print(b);
  Serial.print(" = "); Serial.println(fxpow(ca, b));

  Serial.print("\nVector and quaternion basic operations ie "); Sprint3dVector( vectorPtr(&v,TFixed) ,TFixed);
  Serial.print(" * "); Serial.print(a);
  Serial.print(" = "); Serial.println( vectorPtr(&v,TFixed) * a );

  Serial.print("\nQuaternion r roll:");
  Serial.print(RAD_TO_DEG*(double)r.unit().roll());
  Serial.print("° pitch:");
  Serial.print(RAD_TO_DEG*(double)r.unit().pitch());
  Serial.print("° yaw:");
  Serial.print(RAD_TO_DEG*(double)r.unit().yaw());
  Serial.println("°");
  
  Serial.print("\nVector v=");
  Sprint3dVector( vectorPtr(&v,TFixed) ,TFixed);
  Serial.print(" rotation by quternion r=");
  Serial.print(r); 
  Serial.print(" results to ");
  Sprintln3dVector( r.rotate( vectorPtr(&v,TFixed) ),TFixed );

  Serial.print("\nQuaternion r="); Serial.print(r); 
  Serial.println(" in matrix form is:");
  Sprintln4x4Matrix(r.to4x4Matrix( vectorPtr(&mr,TFixed) ),TFixed);
  
  Serial.print("Quaternion r="); Serial.print(r); 
  Serial.println(" in complex matrix form is:");
  r.to2x2ComplexMatrix( vectorPtr(&cr,TFixed) );
  Serial.print("|");Serial.print(cr(0,0));Serial.print(" ");Serial.print(cr(0,1));Serial.println("|");
  Serial.print("|");Serial.print(cr(1,0));Serial.print(" ");Serial.print(cr(1,1));Serial.println("|");
  
  Serial.print("\nThe rotation matrix or quaternion r=");Serial.print(r); Serial.println(" is:");
  Sprintln3x3Matrix( r.unit().to3x3RotationMatrix( vectorPtr(&rot,TFixed) ) ,TFixed);
  delay(PERIOD_MS);
}
