/*
 * This file is part of the quaternions library
 * Usage: A template library for the implementation of
 *        complex numbers and quaternions for arduino/teensy
 *        It is an extension of the MathFixed.h library and
 *        provides basic mixed operations and functions for 
 *        scalars, vectors, complex numbers and quaternions
 * Version 1.1.1
 * Dependencies: MathFixed library (https://github.com/halsw/MathFixed)
 * 
 * Developed by Evan https://github.com/halsw
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
 * 
 * Types:
 *   QtrnCoord: enumeration for the constructors of complex numbers & quaternions
 *   
 * Classes:
 *   Complex<T>: complex number
 *   Quaternion<T>: printable quaternion
 *   
 * Functions: (overloaded MathFixed.h functions for complex numbers and quaternions)
 *   fxsize() gets number of real numbers needed for representation
 *   fxnan() gets the representation of NaN
 *   fxisnan() tests if argument is not a Complex/Quaternion
 *   fxisinf() tests if argument is infinity, but here just a copy of fxisnan()
 *   fxabs() the norm of a Complex/Quaternion
 *   fxequ() almost equal comparison (set by fxalmost tollerance)
 *   fxsign() the sign (unit vector of spatial/imaginary quaternion coefficients)
 *   fxcopysign() copies the sign of second argument to the first one
 *   fxmax() the greater norm of two Complex/Quaternions
 *   fxmin() the lesser norm of two Complex/Quaternions
 *   fxsq() the square of a Complex/Quaternion
 *   fxsqrt() the square root
 *   fxcbrt() the cubic root
 *   fxrandom() get a random Complex/Quaternion between two limits
 *   fxsin() the sine
 *   fxcos() the cosine
 *   fxtan() the tangent
 *   fxcot() the cotangent
 *   fxatan() the inverse tangent
 *   fxasin() the inverse sine
 *   fxacos() the inverse cosine
 *   fxexp() the natural exponential
 *   fxlog() the natural logarithm
 *   fxpow() raise a Complex/Quaternion to given Complex/Quaternion exponent
 *   fxsinh() the hyperbolic sine
 *   fxsinh() the hyperbolic cosine
 *   fxtanh() the hyperbolic tangent
 *   fxmean() a running calculation of quaternion mean value
 *   fxslerp() quaternion interpolation using SLERP
 *   fxmeanslerp() a running calculation of quaternion mean using SLERP
 *   fxconj() conugate of a Complex/Quaternion
 *   
 * Defines:
 *   Complex(): NaN Complex
 *   Quaternion(): NaN Quaternion 
 *   qarray: Macro to cast containers as arrays (use with care)
 *   QXr0: 0.0 constant templated
 *   QXr1: 1.0 constant templated
 *   QXr2: 2.0 constant templated
 *   CX_0: 0.0 complex constant templated
 *   CX_1: 1.0 complex constant templated
 *   CX_I: i imaginary constant templated
 *   QX_0: 0.0 Quaternion constant templated
 *   QX_1: 1.0 Quaternion constant templated (t axis unit)
 *   QX_I: i quaternionic constant templated (x axis unit)
 *   QX_J: j quaternionic constant templated (y axis unit)
 *   QX_K: k quaternionic constant templated  (z axis unit)
 */

#include "MathFixed.h"

#ifndef QUATERNIONS_H
#define QUATERNIONS_H

typedef enum {R=1, I=2, RI=3, J=4, RJ=5, JI=6, RJI=7, K=8, RK=9, IK=10, RIK=11, JK=12, RJK=13, JIK=14, RJIK=15} Qdim; 

//Helper to cast arrays/objects containing T type elements
#define qarray(X,T) reinterpret_cast<T*>(X)

enum QtrnCoord {
    cCartesian = 0,
    cPolar = 1,
    cSemiPolar = 2,
    cVector = 3,
    cEuler = 4,
    cSpherical = 5,
    cCylindrical = 6,
    cCylinderSpherical = 7,    
    cRotationMatrix = 8    
};

template <class T> class Complex;
template <class T> class Quaternion;

#define QXr0 static_cast<T>(0.0)
#define QXr1 static_cast<T>(1.0)
#define QXr2 static_cast<T>(2.0)

template <class T> const Complex<T> CX_0 = Complex<T>(QXr0);
template <class T> const Complex<T> CX_1 = Complex<T>(QXr1);
template <class T> const Complex<T> CX_I = Complex<T>(QXr0, QXr1);

template <class T> const Quaternion<T> QX_0 = Quaternion<T>(QXr0);
template <class T> const Quaternion<T> QX_1 = Quaternion<T>(QXr1);
template <class T> const Quaternion<T> QX_I = Quaternion<T>(QXr0, QXr1);
template <class T> const Quaternion<T> QX_J = Quaternion<T>(QXr0, QXr0, QXr1);
template <class T> const Quaternion<T> QX_K = Quaternion<T>(QXr0, QXr0, QXr0, QXr1);
 
template <class T>
class Complex {
  protected:
    T r;
    T i;
  public:
    template <typename NUM> constexpr Complex(const NUM re, const NUM im): r((T)re), i((T)im) {};
    Complex(): i(0){ fxnan(r); }
    Complex(T re, T im = 0.0, QtrnCoord type = cCartesian ): r(re), i(im) {
      if ( type == cPolar ) {
        r = re*fxcos(im); 
        i = re*fxsin(im);
      }   
    }  
    Complex(T *a, QtrnCoord type = cCartesian): Complex(a[0],a[1],type) {}
    inline T a() const {return r;}
    inline T b() const {return i;}
    virtual T normSq() const {return this->isnan() ? fxm::info<T>.nan.val : r*r+i*i;}
    T norm() const {T d=normSq(); return fxisnan(d) ? d : fxsqrt(d);}
    inline T theta() const {return fxatan2(i,r); }
    inline Complex real() const {return Complex<T>(r, QXr0);}
    inline Complex imaginary() const {return Complex<T>(QXr0, i);}
    virtual bool isnan() const {return fxisnan(r) || fxisnan(i);}
    inline  iszero() const {return r == QXr0 && i == QXr0; }
    inline bool isreal() const {return !fxisnan(r)  && i == QXr0;}
    inline bool isimaginary() const {return !fxisnan(this->i)  && this->r == QXr0;}
    virtual bool iscomplex() const {return !this->isnan();}
    virtual bool isquaternion() const {return false;}
    T distance(const Complex<T> &x) {return (*this - x).norm();}
    inline bool isnear(const Complex<T> &x, T d = static_cast<T>(0.1)) {return distance(x) <= d;}
    inline bool isunit(const Complex<T> &x, T d = static_cast<T>(0.1)) {return norm() - QXr1 <= d;}
    Complex conjugate() const {return this->isnan() ? Complex() : Complex(r,-i);}
    Complex inverse() const {return conjugate() / normSq();}
    Complex unit() const {T d=r(); return fxisnan(d) ? Complex() : Complex(r/d,i/d);}
    T& operator()(Qdim d) {if (d & R) return &r; return &i;}
    Complex  operator()(T v, Qdim d) {if (d & R) r=v; if (d & I) i=v; return *this;}
    explicit operator T*() const {return &r;}
    operator Quaternion<T>() const {return Quaternion<T>(r, i, QXr0, QXr0);}  
    Complex& operator = (const T x) {  r = x; i = (T)0.0; return *this; }
    Complex& operator = (const Complex<T> &x) { if (this != &x) { r = x.r; i = x.i; } return *this; }
    Complex& operator = (const Quaternion<T> &x) { if (!x.iscomplex()) return Complex(); if (this != &x) { r = x.r; i = x.i; } return *this; }
    inline bool operator == (const Quaternion<T> &x) { return iscomplex() && x.iscomplex() && r == x.r && i == x.i;}
    inline bool operator != (const Quaternion<T> &x) { return !iscomplex() || !x.iscomplex() || r != x.r || i != x.i;}
    inline bool operator > (const Quaternion<T> &x) { return iscomplex() && x.iscomplex() && norm() > x.norm();}
    inline bool operator < (const Quaternion<T> &x) { return iscomplex() && x.iscomplex() && norm() < x.norm();}
    inline bool operator >= (const Quaternion<T> &x) { return iscomplex() && x.iscomplex() && norm() >= x.norm();}
    inline bool operator <= (const Quaternion<T> &x) { return iscomplex() && x.iscomplex() && norm() <= x.norm();}
    inline bool operator == (const Complex<T> &x) { return iscomplex() && x.iscomplex() && this->r == x.r && this->i == x.i;}
    inline bool operator != (const Complex<T> &x) { return !iscomplex() || !x.iscomplex() || this->r != x.r || this->i != x.i;}
    inline bool operator > (const Complex<T> &x) { return iscomplex() && x.iscomplex() && norm() > x.norm();}
    inline bool operator < (const Complex<T> &x) { return iscomplex() && x.iscomplex() && norm() < x.norm();}
    inline bool operator >= (const Complex<T> &x) { return iscomplex() && x.iscomplex() && norm() >= x.norm();}
    inline bool operator <= (const Complex<T> &x) { return iscomplex() && x.iscomplex() && norm() <= x.norm();}
    inline bool operator == (const T &x) { return isreal() && !fxisnan(x) && this->r == x;}
    inline bool operator != (const T &x) { return !isreal() || fxisnan(x) || this->r != x;}
    inline bool operator > (const T &x) { return isreal() && !fxisnan(x) && this->r > x;}
    inline bool operator < (const T &x) { return isreal() && !fxisnan(x) && this->r < x;}
    inline bool operator >= (const T &x) { return isreal() && !fxisnan(x) && this->r >= x.norm();}
    inline bool operator <= (const T &x) { return isreal() && !fxisnan(x) && this->r <= x.norm();}    operator Complex<T>() const {return iscomplex() ? Complex<T>(this->r,this->i) : Complex();}
    Complex operator+() const {return *this;}
    Complex operator-() const { return Complex<T>(-r, -i);}
    Complex operator+(const T &x) { return Complex<T>(r+x, i);}
    Complex operator-(const T &x) { return Complex<T>(r-x, i);}
    Complex operator*(const T &x) { return Complex<T>(r*x, i*x);}
    Complex operator/(const T &x) { return Complex<T>(r/x, i/x);}
    Complex operator+=(T x) {r+=x; return *this;}
    Complex operator-=(T x) {r-=x; return *this;}
    Complex operator*=(T x) {r*=x; i*=x; return *this;}
    Complex operator/=(T x) {r/=x; i/=x; return *this;}
    Complex operator+(const Complex &x) { if ( isnan() || x.isnan() ) return Complex(); return Complex<T>(r+x.r, i+x.i);}
    Complex operator-(const Complex &x) { if ( isnan() || x.isnan() ) return Complex(); return Complex<T>(r-x.r, i-x.i);}
    Complex operator*(const Complex &x) { if ( isnan() || x.isnan() ) return Complex(); return Complex<T>(r+x.r-i*x.i, r*x.i+i*x.r);}
    Complex operator/(const Complex &x) { return *this * x.inverse();}
    Complex operator+=(const Complex<T> &x) {return *this = *this + x;}
    Complex operator-=(const Complex<T> &x) {return *this = *this - x;}
    Complex operator*=(const Complex<T> &x) { return *this = *this * x; }
    Complex operator/=(const Complex<T> &x) { return *this = *this / x; }
};

template <class T> Complex<T> operator+(const T &x, const Complex<T> &y) { return Complex<T>(x+y.a(), y.b());}
template <class T> Complex<T> operator-(const T &x, const Complex<T> &y) { return Complex<T>(x-y.a(),-y.b());}
template <class T> Complex<T> operator*(const T &x, const Complex<T> &y) { return Complex<T>(x*y.a(), x*y.b());}
template <class T> Complex<T> operator/(const T &x, const Complex<T> &y) { return y.inverse()*x;}

template <class T>
class Quaternion : public Complex<T> {
  protected:
    T j;
    T k;
  public:
    template <typename NUM> constexpr Quaternion(const NUM a, const NUM b, const NUM c, const NUM d): Complex<T>(a, b), j((T)c), k((T)d) {};
    Quaternion(): Complex<T>::Complex(), j(0), k(0) {}
    Quaternion(T a, T b = 0.0, T c = 0.0, T d = 0.0, QtrnCoord type = cCartesian): Complex<T>(a,b,cCartesian), j(c), k(d) {
      switch (type) {
        case cPolar: {fromPolar(a,b,c,d); break;}
        case cSemiPolar: {fromSemiPolar(a,b,c,d); break;}
        case cVector: {k=j; j=this->i; this->i=this->r; this->r=QXr0; break;}
        case cEuler: {fromEuler(a,b,c); break;}
        case cSpherical: {fromSpherical(a,b,c,d); break;}
        case cCylindrical: {fromCylindrical(a,b,c,d); break;}
        case cCylinderSpherical: {fromCylinderSpherical(a,b,c,d); break;}    
      }   
    } 
    Quaternion(T *a, QtrnCoord type = cVector) : Quaternion(a[0],a[1],a[2],a[3],type) {
      if ( type == cRotationMatrix) fromRotationMatrix(a);
    }
    Quaternion(const Complex<T>& a): Complex<T>(a.a(),a.b()), j(QXr0), k(QXr0) {}
    Quaternion(const Complex<T>& a, const Complex<T>& b): Complex<T>(a.a(),a.b()), j(b.a()), k(b.b()) {}
    Quaternion(const T a[]): Complex<T>(a[0],a[1]), j(a[2]), k(a[3]) {}
    inline T c() const {return j;}
    inline T d() const {return k;}
    virtual T normSq() const {return this->isnan()?fxm::info<T>.nan.val:Complex<T>::normSq() + j*j + k*k;}
    T theta() const {return iscomplex()?fxatan2(this->i,this->r):fxm::info<T>.nan.val; } //This is complex number theta, for quaternion use argument()
    T vecnormSq() const {return this->isnan() ? fxm::info<T>.nan.val : this->i*this->i + j*j + k*k;}
    T vecnorm() const {T d=vecnormSq(); return fxisnan(d) ? d : fxsqrt(d);}
    Quaternion sign() const {T d=vecnorm();return Quaternion<T>(QXr0, this->i/d, j/d, k/d);}
    T argument() const {return isquaternion()?fxatan2(vecnorm(),this->r):fxm::info<T>.nan.val; }
    T* to3dVector(void* arr = NULL) { //a suitable 3d vector object may also be passed as argument here ie. BLA::Matrix<3,1,Array<3,1,T>>
      T* v = arr;
      if (!v) v = malloc(3*sizeof(T));
      if (!v) return(NULL);
      v[0] = this->i;   
      v[1] = j;   
      v[2] = k;   
      return v;   
    }
    T* to3x3RotationMatrix(void* arr = NULL) { //a suitable 3x3 matrix object may also be passed as argument here ie. BLA::Matrix<3,3,Array<3,3,T>>
      T* m = arr;
      if (!m) m = malloc(9*sizeof(T));
      if (!m) return(NULL);
      T r2 = this->r*this->r, i2 = this->r*this->i, j2 = j*j, k2 = k*k;
      T ri = QXr2*this->r*this->i, ij = QXr2*this->i*j, ik = QXr2*this->i*k;
      T rj = QXr2*this->r*j, jk = QXr2*j*k, rk = QXr2*this->r*k;
      m[0] = r2 + i2 - j2 - k2; m[1] = ij - rk;           m[2] = rj + ik;   
      m[3] = ij + rk;           m[4] = r2 - i2 + j2 - k2; m[5] = jk - ri;   
      m[6] = ik - rj;           m[7] = ri + jk;           m[8] = r2 - i2 - j2 + k2;   
      return m;   
    }
    T* to4x4Matrix(void* arr = NULL) { //a suitable 4x4 matrix object may also be passed as argument here ie. BLA::Matrix<4,4,Array<4,4,T>>
      T* m = arr;
      if (!m) m = malloc(16*sizeof(T));
      if (!m) return(NULL);
      m[0] = this->r; m[1] = this->i; m[2] = j; m[3] = k;   
      m[4] = -this->i; m[5] = this->r; m[6] = -k; m[7] = j;   
      m[8] = -j; m[9] = k; m[10] = this->r; m[11] = -this->i;   
      m[12] = -k; m[13] = -j; m[14] = this->i; m[15] = -this->r;
      return m;   
    }
    Complex<T>* to2x2ComplexMatrix(void* arr = NULL) { //a suitable 2x2(x2) matrix object may also be passed as argument here ie. BLA::Matrix<2,2,Array<2,2,Complex<T>>>
      Complex<T>* m = arr;
      if (!m) m = malloc(4*sizeof(Complex<T>));
      if (!m) return(NULL);
      m[0] = Complex<T>(this->r, this->i);
      m[1] = Complex<T>(j,       k);   
      m[2] = Complex<T>(-k,      k);
      m[3] = Complex<T>(this->r, -this->i);   
      return m;   
    }
    T* to3x3SkewSymmetric(void* arr = NULL) { //a suitable 3x3 matrix object may also be passed as argument here ie. BLA::Matrix<3,3,Array<3,3,T>>
      T* m = arr;
      if (!m) m = malloc(9*sizeof(T));
      if (!m) return(NULL);
      m[0] = QXr0;  m[1] =-k;      m[2] = j;   
      m[3] = k;     m[4] = QXr0;   m[5] = -this->i;   
      m[6] =-j;     m[7] = this->i;m[8] = QXr0;   
      return m;   
    }
    void fromPolar(T r0, T theta0, T r1, T theta1) {
      this->r = r0*fxcos(theta0);
      this->i = r0*fxsin(theta0);
      j = r1*fxcos(theta1);
      k = r1*fxsin(theta1);
    }
    void fromSemiPolar(T r, T phi, T theta0, T theta1) {
      T cp = r*fxcos(phi);
      T sp = r*fxsin(phi);
      this->r = cp*fxcos(theta0);
      this->i = cp*fxsin(theta0);;
      j = sp*fxcos(theta1);
      k = sp*fxsin(theta1);
    }
    void fromEuler(T roll, T pitch, T yaw) {
      T s0 = fxsin(roll/QXr2);
      T c0 = fxcos(roll/QXr2);
      T s1 = fxsin(pitch/QXr2);
      T c1 = fxcos(pitch/QXr2);
      T s2 = fxsin(yaw/QXr2);
      T c2 = fxcos(yaw/QXr2);
      this->r = c0*c1*c2+s0*s1*s2;
      this->i = s0*c1*c2-c0*c1*s2;
      j = c0*s1*c2+s0*c1*s2;
      k = c0*c1*s2-s0*s1*c2;
    }
    void fromSpherical(T r, T phi, T theta0, T theta1) {
      T c1 = r*fxcos(theta1);
      T c0 = c1*fxcos(theta0);
      this->r = c0*fxcos(phi);
      this->i = c0*fxsin(phi);
      j = c1*fxsin(theta0);
      k = r*fxsin(theta1);
    }
    void fromCylindrical(T r, T phi, T z0, T z1) {
      this->r = r*fxcos(phi);
      this->i = r*fxsin(phi);
      j = z0;
      k = z1;
    }
    void fromCylinderSpherical(T t, T r, T phi, T theta) {
      T ct = r*fxcos(theta);
      this->r = t;
      this->i = ct*fxcos(phi);
      j = ct*fxsin(phi);
      k = r*fxsin(theta);
    }
    void fromRotationMatrix(void *arr) { //a suitable 3x3 matrix object may also be passed as argument here ie. BLA::Matrix<3,3,Array<3,3,T>>
      T *m = arr, d=m[0]+m[4]+m[8];
      if (d > QXr0) {
         d = fxsqrt(d + QXr1) / QXr2;
         this->r = static_cast<T>(0.25) * d ;
         this->i = (m[7] - m[5]) / d;
         j = (m[2] - m[6]) / d;
         k = (m[3] - m[1]) / d;
      } else if ( m[0] > fxmax(m[4],m[8]) ) {
         d = QXr2 * fxsqrt(1.0 + m[0] - m[4] - m[8]);
         this->r = (m[7] - m[5]) / d;
         this->i = static_cast<T>(0.25) * d;
         j = (m[1] + m[3]) / d;
         k = (m[2] + m[6]) / d;
      } else if ( m[4] > m[8] ) {
         d = QXr2 * fxsqrt(1.0 + m[4] - m[0] - m[8]);
         this->r = (m[2] - m[6]) / d;
         this->i = (m[1] + m[3]) / d;
         j = static_cast<T>(0.25) * d;
         k = (m[5] + m[7]) / d;
      } else {
         d = QXr2 * fxsqrt(1.0 + m[8] - m[0] - m[4]);
         this->r = (m[3] - m[1]) / d;
         this->i = (m[2] + m[6]) / d;
         j = (m[5] + m[7]) / d;
         k = static_cast<T>(0.25) * d;
      }
    }
    inline Complex<T> ca() const {return Complex<T>(this->r,this->i);}
    inline Complex<T> cb() const {return Complex<T>(j,k);}
    inline Quaternion real() const {return Quaternion<T>(this->r, QXr0 , QXr0, QXr0);}
    inline Quaternion imaginary() const {return Quaternion<T>(QXr0 , this->i, QXr0, QXr0);}
    inline Quaternion vector() const {return Quaternion<T>(QXr0, this->i, j, k);}
    inline Quaternion conjugate() const {return Quaternion<T>(this->r,-this->i,-j,-k);}
    Quaternion inverse() const {return conjugate() / normSq();}
    Quaternion unit() const {T d=norm(); return fxisnan(d) ? Quaternion() : Quaternion(this->r/d, this->i/d, j/d, k/d);}
    Quaternion reciprocal() const {T d=normSq(); return fxisnan(d) ? Quaternion() : Quaternion(this->r/d, -this->i/d, -j/d, -k/d);}
    virtual bool isnan() const {return Complex<T>::isnan() || fxisnan(j) || fxisnan(k);}
    inline bool iszero() const {return isreal() && this->r == QXr0;}
    inline bool isreal() const {return iscomplex() && !fxisnan(this->r) && this->i == QXr0;}
    virtual bool iscomplex() const {return !fxisnan(this->r) && !fxisnan(this->i) && j == QXr0 && k == QXr0;}
    inline bool isimaginary() const {return iscomplex() && !fxisnan(this->i) && this->r == QXr0;}
    inline bool isvector() const {return !this->isnan() && this->r == QXr0;}
    virtual bool isquaternion() const {return !isnan();}
    T dot(const Quaternion<T> &x) {return this->r*x.r + this->i*x.i + j*x.j + k*x.k;}
    Quaternion<T> cross(const Quaternion<T> &x) {
      T ni, nj, nk; 
      if(!isvector() || !x.isvector()) return Quaternion();
      ni = j*x.k - x.j*k;
      nj = x.i*k - this->i*x.k;
      nk = this->i*x.j - x.i*j;
      return Quaternion<T>(QXr0, ni, nj, nk);
    }    
    Quaternion<T> mulmatrix(T m[], int rows=1 ) {//Quaternion is supposed to be a vertical vector[3], and matrix of size[3,rows]
      Quaternion<T>  r = QX_0<T>;
      if (rows<1) return Quaternion();
      for (int n=0; n<rows; n++)
        r += *this * (m+n*3);
      return r;  
    }
    Quaternion multransposed(T m[], int columns=1 ) {//Quaternion is supposed to be a vertical vector[3], and matrix of size[columns,3]  
      Quaternion<T>  r = QX_0<T>;
      if (columns<1) return Quaternion();
      for (int n=0; n<columns; n++) {
        r.i += this->i * m[n];
        r.j += j*m[n+columns];
        r.k += k*m[n+(columns<<1)];
      }  
      return r;  
    }
    T distance(const Quaternion<T> &x) {return (*this - x).norm();}
    inline bool isnear(const Quaternion<T> &x, T d = static_cast<T>(0.1)) {return distance(x) <= d;}
    inline bool isunit(const Quaternion<T> &x, T d = static_cast<T>(0.1)) {return norm() - QXr1 <= d;}
    T* rotate(T* vec) { Quaternion<T> v=Quaternion<T>(vec), u = unit(); v = u * v * u.conjugate(); vec[0] = v.i; vec[1] = v.j; vec[2] = v.k; return(vec);} 
    T roll() { return isnan() ? fxm::info<T>.nan.val: fxatan2( QXr2*(this->r*this->i + j*k), QXr1-QXr2*(this->i*this->i+j*j) );}
    T pitch() { if ( isnan() ) return fxm::info<T>.nan.val;
      T p = QXr2*(this->r*j - this->i*k);
      if (fxabs(p) >= QXr1) return p>QXr0 ? HALF_PI : -HALF_PI; 
      return fxasin(p);}
    T yaw() { return isnan() ? fxm::info<T>.nan.val: fxatan2( QXr2*(this->r*k + this->i*j), QXr1-QXr2*(j*j+k*k) );}
    T& operator ()(Qdim d) {if (d & R) return &this->r; if (d & I) return &this->i; if (d & J) &j; return &k;}
    Quaternion operator ()(T v, Qdim d) {if (d & R) this->r=v; if (d & I) this->i=v; if (d & J) j=v; if (d & K) k=v; return *this;}
    operator Complex<T>() const {return iscomplex() ? Complex<T>(this->r,this->i) : Complex<T>();}
    Quaternion& operator = (const T x) {  this->r = x; this->i = j = k = (T)0.0; return *this; }
    Quaternion& operator = (const Quaternion<T> &x) { if (this != &x) { this->r = x.r; this->i = x.i; j = x.j; k = x.k;} return *this; }
    Quaternion& operator = (const Complex<T> &x) { if (this != &x) { this->r = x.r; this->i = x.i; j = QXr0; k = QXr0; } return *this; }
    inline bool operator == (const Quaternion<T> &x) const { return isquaternion() && x.isquaternion() && this->r == x.r && this->i == x.i && j == x.j && k == x.k;}
    inline bool operator != (const Quaternion<T> &x) const { return !isquaternion() || !x.isquaternion() || this->r != x.r || this->i != x.i || j != x.j || k != x.k;}
    inline bool operator > (const Quaternion<T> &x) const { return isquaternion() && x.isquaternion() && norm() > x.norm();}
    inline bool operator < (const Quaternion<T> &x) { return isquaternion() && x.isquaternion() && norm() < x.norm();}
    inline bool operator >= (const Quaternion<T> &x) const { return isquaternion() && x.isquaternion() && norm() >= x.norm();}
    inline bool operator <= (const Quaternion<T> &x) const { return isquaternion() && x.isquaternion() && norm() <= x.norm();}
    inline bool operator == (const Complex<T> &x) const { return iscomplex() && x.iscomplex() && this->r == x.r && this->i == x.i;}
    inline bool operator != (const Complex<T> &x) const { return !iscomplex() || !x.iscomplex() || this->r != x.r || this->i != x.i;}
    inline bool operator > (const Complex<T> &x) const { return iscomplex() && x.iscomplex() && norm() > x.norm();}
    inline bool operator < (const Complex<T> &x) const { return iscomplex() && x.iscomplex() && norm() < x.norm();}
    inline bool operator >= (const Complex<T> &x) const { return iscomplex() && x.iscomplex() && norm() >= x.norm();}
    inline bool operator <= (const Complex<T> &x) const { return iscomplex() && x.iscomplex() && norm() <= x.norm();}
    inline bool operator == (const T &x) const { return isreal() && !fxisnan(x) && this->r == x;}
    inline bool operator != (const T &x) const { return !isreal() || fxisnan(x) || this->r != x;}
    inline bool operator > (const T &x) const { return isreal() && !fxisnan(x) && this->r > x;}
    inline bool operator < (const T &x) const { return isreal() && !fxisnan(x) && this->r < x;}
    inline bool operator >= (const T &x) const { return isreal() && !fxisnan(x) && this->r >= x.norm();}
    inline bool operator <= (const T &x) const { return isreal() && !fxisnan(x) && this->r <= x.norm();}
    Quaternion operator+() const {return *this;}
    Quaternion operator-() const {return Quaternion<T>(-this->r, -this->i, -j, -k);}
    Quaternion operator+(const T &x) {return Quaternion<T>(this->r+x, this->i, j, k);}
    Quaternion operator-(const T &x) {return Quaternion<T>(this->r-x, this->i, j, k);}
    Quaternion operator*(const T &x) {return Quaternion<T>(this->r*x, this->i*x, j*x, k*x);}
    Quaternion operator/(const T &x) {return Quaternion<T>(this->r/x, this->i/x, j/x, k/x);}
    Quaternion operator+=(T &x) {this->r+=x; return *this;}
    Quaternion operator-=(T &x) {this->r-=x; return *this;}
    Quaternion operator*=(T &x) {this->r*=x; this->i*=x; j*=x; k*=x; return *this;}
    Quaternion operator/=(T &x) {this->r/=x; this->i/=x; j/=x; k/=x; return *this;}
    Quaternion operator+=(const Complex<T> &x) {this->r+=x.r; this->i+=x.i; return *this;}
    Quaternion operator-=(const Complex<T> &x) {this->r-=x.r; this->i-=x.i; return *this;}
    Quaternion operator*=(const Complex<T> &x) { return *this = ca() * x; }
    Quaternion operator/=(const Complex<T> &x) { return *this = ca() / x; }
    Quaternion operator+(const Quaternion<T> &x) {this->r+=x.r; this->i+=x.i; j+=x.j; k+=x.k; return *this;}
    Quaternion operator-(const Quaternion<T> &x) {this->r-=x.r; this->i-=x.i; j-=x.j; k-=x.k; return *this;}
    Quaternion operator*(const Quaternion<T> &x) {
      T nr, ni, nj, nk;
      if ( isnan() || x.isnan() ) return Quaternion();
      nr = this->r * x.r - this->i * x.i - j * x.j - k * x.k;
      ni = this->r * x.i + this->i * x.r + j * x.k - k * x.j;
      nj = this->r * x.j - this->i * x.k + j * x.r + k * x.i;
      nk = this->r * x.k + this->i * x.j - j * x.i + k * x.r;
      return Quaternion<T>(nr, ni, nj, nk);
    }
    Quaternion operator/(const Quaternion<T> &x) { if ( isnan() || x.isnan() ) return Quaternion(); return *this * x.inverse();}
    Quaternion operator+=(const Quaternion<T> &x) { return *this = *this + x;}
    Quaternion operator-=(const Quaternion<T> &x) { return *this = *this - x;}
    Quaternion operator*=(const Quaternion<T> &x) { return *this = *this * x;}
    Quaternion operator/=(const Quaternion<T> &x) { return *this = *this / x;}
    Quaternion operator+(const T x[]) { return Quaternion<T>(this->r, this->i+x[0], j+x[1], k+x[2]); }
    Quaternion operator-(const T x[]) { return Quaternion<T>(this->r, this->i-x[0], j-x[1], k-x[2]); }
    Quaternion operator*(const T x[]) { return *this * Quaternion<T>(0.0,x[0],x[1],x[2]); }
    Quaternion operator/(const T x[]) { return *this / Quaternion<T>(0.0,x[0],x[1],x[2]); }
    Quaternion operator+=(const T x[]) { return *this = *this + x;}
    Quaternion operator-=(const T x[]) { return *this = *this - x;}
    Quaternion operator*=(const T x[]) { return *this = *this * x;}
    Quaternion operator/=(const T x[]) { return *this = *this / x;}
  };

template <class T> Quaternion<T> operator+(const T &x, const Quaternion<T> &y) { return Quaternion<T>(x+y.a(), y.b(), y.c(), y.d());}
template <class T> Quaternion<T> operator-(const T &x, const Quaternion<T> &y) { return Quaternion<T>(x-y.a(), -y.b(), -y.c(), -y.d());}
template <class T> Quaternion<T> operator*(const T &x, const Quaternion<T> &y) { return Quaternion<T>(x*y.a(), x*y.b(), x*y.c(), x*y.d());}
template <class T> Quaternion<T> operator/(const T &x, const Quaternion<T> &y) { return y.inverse()*x;}
template <class T> Quaternion<T> operator+(const Complex<T> &x, const Quaternion<T> &y) { return Quaternion<T>(x.a()+y.a(), x.b()+y.b(), y.c(), y.d());}
template <class T> Quaternion<T> operator-(const Complex<T> &x, const Quaternion<T> &y) { return Quaternion<T>(x.a()-y.a(), x.b()-y.b(), -y.c(), -y.d());}
template <class T> Quaternion<T> operator*(const Complex<T> &x, const Quaternion<T> &y) { return Quaternion<T>(x.a()*y.a()-x.b()*y.b(), x.a()*y.b()+x.b()*y.a(), x.a()*y.c()-x.b()*y.d(), x.a()*y.d()+x.b()*y.c());}
template <class T> Quaternion<T> operator/(const Complex<T> &x, const Quaternion<T> &y) { return x * y.inverse();}
template <class T> Quaternion<T> operator+(const T x[], const Quaternion<T> &y) { return Quaternion<T>(y.a(), x[0]+y.b(), x[1]+y.c(), x[2]+y.d());}
template <class T> Quaternion<T> operator-(const T x[], const Quaternion<T> &y) { return Quaternion<T>(-y.a(), x[0]-y.b(), x[1]-y.c(), x[2]-y.d());}
template <class T> Quaternion<T> operator*(const T x[], const Quaternion<T> &y) { return y * Quaternion<T>(x);}
template <class T> Quaternion<T> operator/(const T x[], const Quaternion<T> &y) { return Quaternion<T>(x) * y.inverse();}

template <class T>
Stream& operator << (Stream &p, const Complex<T> &x) {
  double v;
  size_t n = p.write('(');
  
  if ( x.isnan() )
    n+=p.print(F("NaN"));
  else if ( x.iszero() )
    n+= p.print(F("0.0"));
  else {
    if ( !x.isimaginary()  ) {
      v = (double)x.a();
      n+=p.print(v,fxPrecision);
    }  
    if ( !x.isreal() ) {
      v = (double)x.b();
      if ( v < 0.0 ) {
        n+=p.write('-');
        v = -v;
      } else
        n+=p.write('+');
      n+=p.write('i');
      n+=p.print(v,fxPrecision);
    }
  }      
  n+=p.write(')');
  if (fxWidth) while (n++<fxWidth) p.write(' ');
  return p;
};

template <class T>
Stream& operator << (Stream &p, const Quaternion<T> &x) {
  double v;
  bool nz=false;
  char c[6] = "i\0j\0k\0";
  size_t n = p.write('[');
  if ( x.isnan() )
    n += p.print(F("NaN"));
  else if ( x.iszero() )
    n += p.print(F("0.0"));
  else for (uint8_t i=0; i<3; i++) {
    switch (i) {
      case 1: {v=(double)x.b(); break;}
      case 2: {v=(double)x.c(); break;}
      case 3: {v=(double)x.d(); break;}
      default: {v=(double)x.a(); break;}
    }
    if ( v != 0.0 ) {
      if (v<0.0) {
        n += p.write('-');
        v = -v;
      } else   
        if (nz) n += p.write('+');
      if (i) n += p.print(c+((i-1)<<1));
      nz = true;
      n += p.print(v,fxPrecision);
    }
  }    
  n += p.print(']');
  if (fxWidth) while (n++<fxWidth) p.print(' ');
  return p;
}

template <class T> constexpr size_t fxsize(Complex<T> x) { return 2;}

template <class T> 
inline Complex<T>& fxnan( Complex<T>& x ) {return x = fxm::info<T>.nan.val;}

template <class T>
inline bool fxisnan(const Complex<T>& x) {
  return !x.iscomplex();
}

template <class T>
inline bool fxisinf(const Complex<T>& x) {
  return !x.iscomplex();
}

template <class T>
inline T fxabs(const Complex<T>& x) {
  return x.norm();
}

template <class T>
inline bool fxequ(const Complex<T>& x) {
  if ( fxisnan(x)) return false;
    return x.norm() <= fxalmost<T>();
  }

template <class T>
inline bool fxequ(const Complex<T>& x, const Complex<T>& y) {
  if ( fxisnan(x) || fxisnan(y) ) return false;
    return fxabs(x-y) <= fxalmost<T>();
  }

template <class T>
  Complex<T> fxsign(const Complex<T> &x) {
    if (x.isnan()) return Complex<T>();
    if (fxequ(x)) return CX_1<T>;
    return x / x.norm();
  }

template <class T>
  Complex<T> fxcopysign(const Complex<T>& x, const Complex<T>& y) {
    if (x.isnan() || y.isnan()) return Complex<T>();
    return x.norm()*fxsign(y);
  }

template <class T>
Complex<T> fxmax(const Complex<T>& x, const Complex<T>& y) {
  if ( !x.iscomplex() ) return x;
  if ( !y.iscomplex() ) return y;
  return x.norm()< y.norm() ? y : x;
}

template <class T>
T fxmax(T x, const Complex<T>& y) {
  if ( !y.iscomplex() || fxisnan(x) ) return fxm::info<T>.nan.val;
  T ay = y.norm();
  return x < ay ? ay : x;
}

template <class T>
inline T fxmax(const Complex<T>& x, T y) {
  return fxmax(y, x);
}

template <class T>
Complex<T> fxmin(const Complex<T>& x, const Complex<T>& y) {
  if ( !x.iscomplex() ) return x;
  if ( !y.iscomplex() ) return y;
  return x.norm() > y.norm() ? y : x;
}

template <class T>
T fxmin(T x, const Complex<T>& y) {
  if ( !y.iscomplex() || fxisnan(x) ) return fxm::info<T>.nan.val;
  T ay = y.norm();
  return x > ay ? ay : x;
}

template <class T>
inline T fxmin(const Complex<T>& x, T y) {
  return fxmin(y, x);
}

template <class T>
Complex<T> fxsq(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  return x*x;
}

template <class T>
Complex<T> fxsqrt(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  if ( x.a() == 0 ) return Complex<T>(QXr0,  x.b()<QXr0 ? -HALF_PI/QXr2: HALF_PI/QXr2);
  T cs = x.b() / x.norm();
  T sn = fxsqrt( (QXr1 - cs) / QXr2);
  cs = fxsqrt( (QXr1 + cs) / QXr2);
  if (x.a() < 0) sn = -sn; 
  if (x.b() < 0) cs = -cs; 
  return fxsqrt(x.norm())*Complex<T>(cs, sn);
}

template <class T>
Complex<T> fxcbrt(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  if ( x.a() == 0 ) return Complex<T>(QXr0,  x.b()<QXr0 ? -HALF_PI/3.0: HALF_PI/3.0);
  return Complex<T>(fxcbrt(x.norm()), x.theta() / 3.0, cPolar);
}

template <class T>
Complex<T> fxrandom(const Complex<T>& x, Complex<T> y = CX_0<T>) {
  if ( !x.iscomplex() || !y.iscomplex() ) return Complex<T>();
  return Complex<T>(fxrandom(x.a(),y.a()) , fxrandom(x.b(),y.b()));
}
  
template <class T>
Complex<T> fxexp(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  if ( x.a() == 0 ) return Complex<T>(QXr0,  x.b()<QXr0 ? -fxexp(x.b()): fxexp(x.b()) );
  if ( x.b() == 0 ) return Complex<T>(fxexp(x.a()), QXr0 );
  return Complex<T>(fxcos(x.b()), fxsin(x.b())) * fxexp(x.a());
}

template <class T>
Complex<T> fxlog(const Complex<T>& x) {
  T r = x.norm();
  if ( !x.iscomplex() || r == QXr0 ) return Complex<T>();
  return Complex<T>(fxlog(r),x.theta());
}

template <class T>
Complex<T> fxpow(const Complex<T>& x, const Complex<T>& y) {
  if ( !x.iscomplex() || !y.iscomplex() ) return Complex<T>();
  return fxexp(fxlog(x) * y);
}

template <class T>
Complex<T> fxpow(T x, const Complex<T>& y) {
  if ( fxisnan(x) || !y.iscomplex() ) return Complex<T>();
  return fxexp(fxlog(x) * y);
}

template <class T>
Complex<T> fxpow(const Complex<T>& x, T y) {
  if ( !x.iscomplex() || fxisnan(y) ) return Complex<T>();
  return fxexp(fxlog(x) * y);
}

template <class T>
Complex<T> fxsin(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  return Complex<T>(fxsin(x.a())*fxcosh(x.b()), fxcos(x.a())*fxsinh(x.b()));
}

template <class T>
Complex<T> fxcos(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  return Complex<T>(fxcos(x.a())*fxcosh(x.b()), -fxsin(x.a())*fxsinh(x.b()));
}

template <class T>
Complex<T> fxtan(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  T ca = fxcos(x.a());
  T sa = fxsin(x.a());
  T cb = fxcosh(x.b());
  T sb = fxsinh(x.a());
  return Complex<T>(sa*cb,ca*sb) / Complex<T>(ca*cb,-sa*sb);
}

template <class T>
Complex<T> fxsinh(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  return (fxexp(x) - fxexp(-x)) / QXr2;
}

template <class T>
Complex<T> fxcosh(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  return (fxexp(x) - fxexp(-x)) / QXr2;
}

template <class T>
Complex<T> fxtanh(const Complex<T>& x) {
  if ( !x.iscomplex() ) return Complex<T>();
  x=fxexp(QXr2*x);
  return (x - QXr1)/(x + QXr1);
}

template <class T>
inline Complex<T> fxconj(const Complex<T>&  x) { return x.conjugate(); }

template <class T> constexpr size_t fxsize(Quaternion<T> x) { return 4;}

template <class T> 
inline Quaternion<T>& fxnan( Quaternion<T>& x ) {return x = fxm::info<T>.nan.val;}

template <class T>
inline bool fxisnan(const Quaternion<T>& x) {
  return !x.isquaternion();
}

template <class T>
inline bool fxisinf(const Quaternion<T>& x) {
  return !x.isquaternion();
}

template <class T>
inline T fxabs(const Quaternion<T>& x) {
  return x.norm();
}

template <class T>
inline bool fxequ(const Quaternion<T>& x) {
  if ( fxisnan(x) ) return false;
    return x.norm() <= fxalmost<T>();
  }

template <class T>
inline bool fxequ(const Quaternion<T>& x, const Quaternion<T> y) {
  if ( fxisnan(x) || fxisnan(y) ) return false;
    return fxabs(x-y) <= fxalmost<T>();
  }

template <class T>
  Quaternion<T> fxsign(const Quaternion<T>& x) {
    if (x.isnan()) return Quaternion<T>();
    if (fxequ(x)) return QX_1<T>;
    return x.sign();
  }

template <class T>
  Quaternion<T> fxcopysign(const Quaternion<T>& x, const Quaternion<T>& y) {
    if (x.isnan() || y.isnan()) return Quaternion<T>();
    return x.vecnorm()*fxsign(y);
  }

template <class T>
Quaternion<T> fxmax(const Quaternion<T>& x, const Quaternion<T>& y) {
  if ( !x.isquaternion() ) return x;
  if ( !y.isquaternion() ) return y;
  return x.norm()< y.norm() ? y : x;
}

template <class T>
Complex<T> fxmax(const Quaternion<T>& x, const Complex<T>& y) {
  if ( !x.isquaternion() ) return x;
  if ( !y.iscomplex() ) return y;
  return x.norm()< y.norm() ? y : x;
}

template <class T>
T fxmax(T x, const Quaternion<T>& y) {
  if ( !y.isquaternion() || fxisnan(x) ) return fxm::info<T>.nan.val;
  T ay = y.norm();
  return x < ay ? ay : x;
}

template <class T> inline Complex<T> fxmax(const Complex<T>& x, const Quaternion<T>& y) { return fxmax(y, x); }

template <class T> inline T fxmax(const Quaternion<T>& x, T y) { return fxmax(y, x); }

template <class T>
Quaternion<T> fxmin(const Quaternion<T>& x, const Quaternion<T>& y) {
  if ( !x.isquaternion() ) return x;
  if ( !y.isquaternion() ) return y;
  return x.norm() > y.norm() ? y : x;
}

template <class T>
Complex<T> fxmin(const Quaternion<T>& x, const Complex<T>& y) {
  if ( !x.isquaternion() ) return x;
  if ( !y.iscomplex() ) return y;
  return x.norm() > y.norm() ? y : x;
}

template <class T>
T fxmin(T x, const Quaternion<T>& y) {
  if ( !y.isquaternion() || fxisnan(x) ) return fxm::info<T>.nan.val;
  T ay = y.norm();
  return x > ay ? ay : x;
}

template <class T> inline Complex<T> fxmin(const Complex<T>& x, const Quaternion<T>& y) { return fxmax(y, x); }

template <class T> inline T fxmin(const Quaternion<T>& x, T y) { return fxmax(y, x); }


template <class T>
Quaternion<T> fxsq(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  return x*x;
}

template <class T>
Quaternion<T> fxsqrt(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x == QX_0<T> ) return QX_0<T>;
  T cs = x.a() / x.norm();
  T sn = fxsqrt( (QXr1 - cs*cs) / QXr2);
  cs = fxsqrt( (QXr1 + cs*cs) / QXr2);
  return fxsqrt(x.norm())*(cs + sn*x.sign());
}

template <class T>
Quaternion<T> fxcbrt(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x == QX_0<T> ) return QX_0<T>;
  T a = x.argument()/3.0;
  return fxcbrt(x.norm())*(fxcos(a) + fxsin(a)*x.sign());
}

template <class T>
Quaternion<T> fxrandom(const Quaternion<T>& x, Quaternion<T> y = QX_0<T>) {
  if ( !x.isquaternion() || !y.isquaternion() ) return Quaternion<T>();
  return Quaternion<T>(fxrandom(x.a(),y.a()), fxrandom(x.b(),y.b()), fxrandom(x.c(),y.c()), fxrandom(x.d(),y.d()));
}
  
template <class T>
Quaternion<T> fxexp(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x == QX_0<T> ) return QX_1<T>;
  T cs = x.a() / x.norm();
  T sn = fxsqrt( QXr1 - cs*cs );
  return ( cs + sn*x.sign()) * fxexp(x.a());
}

template <class T>
Quaternion<T> fxlog(const Quaternion<T>& x) {
  T r = x.norm();
  if ( !x.isquaternion() || r == QXr0 ) return Quaternion<T>();
  return fxlog(r) + x.sign()*x.argument();
}

template <class T>
Quaternion<T> fxpow(const Quaternion<T>& x, const Quaternion<T>& y) {
  if ( !x.isquaternion() || !y.isquaternion() ) return Quaternion<T>();
  return fxexp(fxlog(x) * y);
}

template <class T>
Quaternion<T> fxpow(const Complex<T>& x, const Quaternion<T>& y) {
  if ( !x.iscomplex() || !y.isquaternion() ) return Quaternion<T>();
  return fxexp( fxlog(Quaternion<T>(x.a(),x.b(),QXr0,QXr0)) * y );
}

template <class T>
Quaternion<T> fxpow( const Quaternion<T>& x, const Complex<T>& y) {
  if ( !x.isquaternion() || !y.iscomplex() ) return Quaternion<T>();
  return fxexp( fxlog(x)  * Quaternion<T>(y.a(),y.b(),QXr0,QXr0) );
}

template <class T>
Quaternion<T> fxpow(T x, const Quaternion<T>& y) {
  if ( fxisnan(x) || !y.isquaternion() ) return Quaternion<T>();
  return fxexp( y * fxlog(x) );
}

template <class T>
Quaternion<T> fxpow( const Quaternion<T>& x, T y) {
  if ( !x.isquaternion() || fxisnan(y) ) return Quaternion<T>();
  return fxexp( fxlog(x) * y );
}

template <class T>
Quaternion<T> fxsin(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x.isreal() ) return Quaternion<T>(fxsin(x.a()));
  return x.sign()/QXr2*(fxexp(x*x.sign()) - fxexp(-x*x.sign()));
}

template <class T>
Quaternion<T> fxcos(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x.isreal() ) return Quaternion<T>(fxcos(x.a()));
  return x.sign()/QXr2*(fxexp(x*x.sign()) + fxexp(-x*x.sign()));
}

template <class T>
Quaternion<T> fxtan(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  if ( x.isreal() ) return Quaternion<T>(fxtan(x.a()));
  Quaternion<T> u = fxexp(x*x.sign());
  Quaternion<T> v = fxexp(-x*x.sign());
  return (u - v) / (u + v);
}

template <class T>
T fxsinh(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  return (fxexp(x) - fxexp(-x)) / QXr2;
}

template <class T>
T fxcosh(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  return (fxexp(x) - fxexp(-x)) / QXr2;
}

template <class T>
T fxtanh(const Quaternion<T>& x) {
  if ( !x.isquaternion() ) return Quaternion<T>();
  x=fxexp(QXr2*x);
  return (x - QXr1)/(x + QXr1);
}

template <class T>
inline Quaternion<T> fxconj(const Quaternion<T>&  x) { return x.conjugate(); }

template <class T>
Quaternion<T> fxmean(Quaternion<T> q=QX_0<T>) {// mean value online calculation of quarernions
  static Quaternion<T> mean=QX_0<T>;
  T mnorm=0;
  int total =0;
  if ( fxisnan(q) ) return;
  if (q == QX_0<T>) {
    q = mean;
    mean = QX_0<T>;
    mnorm =0.0;
    total =0;
    return q;
  }
  if (!total) {
    mnorm = q.norm();
    mean = q.unit();
    total =1;
    return q;
  }
  if (mean.dot(q)<0) q = -q;
  mnorm += (q.norm() - mnorm) / total;
  mean += (q.unit() - mean) / total++;
  return mnorm*mean;
}

template <class T>
Quaternion<T> fxslerp(Quaternion<T> q0, Quaternion<T> q1, T t) {
   T d = q0.dot(q1);
   T s = fxsqrt(1.0 - d*d);
   T phi = fxacos(fxabs(d));
   d = t*phi;    
   return ( fxsin(phi-d)*q0 + fxsin(d)*q1 )/s;
}

template <class T>
Quaternion<T> fxmeanslerp(Quaternion<T> q=QX_0<T>) {
  static Quaternion<T> mean=QX_0<T>;
  int total =0;
  if (q == QX_0<T>) {
    q = mean;
    mean = QX_0<T>;
    total =0;
    return q;
  }
  if (!total) {
    mean = q;
    total =1;
    return q;
  }
  total++;
  mean=slerp(mean, q , QXr1 / total);
  return mean;
}


#endif
