/*
 * This file is part of the quaternions library
 * Usage: Provide an example use of the library
 * 
 * Version 1.1.1
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

//Print an T[N] *X array to D device with I integer digits space and F fractional digits  
#define printVector(D,X,N,T,I,F,E) {\
  T *p = (T*)(X);\
  int i,c,n;\
  D.print("|");\
  for (i = 0; i < (N) - 1; i++) {\
    c =0;\
    if ( p[i] >= static_cast<T>(0.0) ) c=D.print(" ");\  
    c+=D.print((double)p[i],F);\
    D.print(",");\
    for (int n=0; n<(I)+(F)+2-c; n++) D.print(" ");\
  }\  
  if ( p[i] >= static_cast<T>(0.0) ) D.print(" ");\  
  D.print((double)p[i]);\
  D.print("|");\
  D.print(E);\
}

//Print an T[N,N] *X array to D device with I integer digits space and F fractional digits  
#define printMatrix(D,X,N,T,I,F,E) {\
  for (int j = 0; j < (N) ; j++) {\
    printVector(D,(T*)(X) + j * (N),N,T,I,F,"")\
    D.println();\
  }\
  D.print(E);\
}    

//Some fast printing of arrays and compatible objects to Serial definitions
#define Sprint3dVector(X,T) printVector(Serial,X,3,T,5,2,"")
#define Sprint3x3Matrix(X,T) printMatrix(Serial,X,3,T,5,2,"")
#define Sprint4x4Matrix(X,T) printMatrix(Serial,X,4,T,5,2,"")
#define Sprintln3dVector(X,T) printVector(Serial,X,3,T,5,2,"\n")
#define Sprintln3x3Matrix(X,T) printMatrix(Serial,X,3,T,5,2,"\n")
#define Sprintln4x4Matrix(X,T) printMatrix(Serial,X,4,T,5,2,"\n")

Complex<TFixed> c={1,1}; 
Quaternion<TFixed> r={1,1,1,1};
ArrayMatrix<4,4,TFixed> mr;
ArrayMatrix<3,3,TFixed> rot;
BLA::Matrix<2,2,Array<2,2,Complex<TFixed>>> cr;
ArrayMatrix<3,1,TFixed> v={1,0,0};

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0)); //assuming A0 is not connected
}

void loop() {
  Complex<TFixed> ca, cb;
  Quaternion<TFixed> a, b;
  ca=fxrandom(c);
  cb=fxrandom(c);
  Serial<< "Complex number basic operations ie multiplication " << c << " * " << cb << " = " << ca*cb;
  Serial << "\nComplex number functions ie pow " << ca << " ^ " << cb << " = " << fxpow(ca, cb);
 
  a=fxrandom(r);
  b=fxrandom(r);
  Serial << "\nQuaternion basic operations ie division " << a << " / " << b << " = " << a/b;
  Serial << "\nQuaternion functions ie log(" << a << ") = "<<fxlog(a);

  Serial << "\nMixed scalar/complex numbers/quaternion basic operations ie " << a << " / " << cb << " + 2 = " << a/cb+2.0;
  Serial << "\nMixed scalar/complex numbers/quaternion functions ie pow " << ca << " ^ " << b << " = " << fxpow(ca, b);

  Serial << "\nVector and quaternion basic operations ie ";
  Sprint3dVector( qarray(&v,TFixed) ,TFixed);
  Serial << " * " << a << " = " << qarray(&v,TFixed) * a ;

  Serial << "\nQuaternion SLERP: slerp( "<< a<< ", " << b << ", 1/2 ) = " << fxslerp(a,b,(TFixed)0.5);
  
  Serial << "\nQuaternion r roll:" << (float)(RAD_TO_DEG*r.unit().roll())\
    << "° pitch:" << (float)(RAD_TO_DEG*r.unit().pitch())\
    << "° yaw:" << (float)(RAD_TO_DEG*r.unit().yaw()) << "°\n";
  
  Serial << "\nVector v=";
  Sprint3dVector( qarray(&v,TFixed) ,TFixed);
  Serial << " rotation by quaternion r=" << r << " results to ";
  Sprintln3dVector( r.rotate( qarray(&v,TFixed) ),TFixed );

  Serial << "\nQuaternion a=" << a << " in matrix form is:\n";
  Sprintln4x4Matrix(a.to4x4Matrix( qarray(&mr,TFixed) ),TFixed);
  
  Serial << "Quaternion a=" << a << " in complex matrix form is:\n";
  a.to2x2ComplexMatrix( qarray(&cr,TFixed) );
  Serial << "|" << cr(0,0) << " " << cr(0,1) << "|\n"\
         << "|" << cr(1,0) << " " << cr(1,1) << "|\n";
  
  Serial << "\nThe rotation matrix for quaternion a=" << a << " is:";
  Sprintln3x3Matrix( a.unit().to3x3RotationMatrix( qarray(&rot,TFixed) ) ,TFixed);

  Serial << "\nQuaternion . Matrix " << b.vector() << " * R = " << b.mulmatrix( qarray(&rot,TFixed), 3 ).vector();

  
delay(PERIOD_MS);
}
