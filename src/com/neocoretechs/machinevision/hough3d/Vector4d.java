//#include "Vector4d.h"
//#include "Matrix4d.h"
//#include <cmath>
package com.neocoretechs.machinevision.hough3d;

public class Vector4d {

   double x;
   double y;
   double z;
   double w;
 
   public Vector4d() { x=y=z=0.0f; w = 1; }
   public Vector4d(double _x, double _y, double _z) {
     x = _x;
     y = _y;
     z = _z;
     w = 1;
   }
   public Vector4d(double _x, double _y, double _z, double _w) {
     x = _x;
     y = _y;
     z = _z;
     w = _w;
  }

  public Vector4d(Vector4d v) {
    x = v.x;
    y = v.y;
    z = v.z;
    w = v.w;
  }

  Vector4d(double[] v) {
   x = v[0];
   y = v[1];
   z = v[2];
   w = v[3];
}

Vector4d set(Vector4d v) {
   x = v.x;
   y = v.y;
   z = v.z;
   w = v.w;
  return this;
}

Vector4d add(Vector4d v) {
   x += v.x;
   y += v.y;
   z += v.z;
   return this;
}

Vector4d subtract(Vector4d v) {
   x -= v.x;
   y -= v.y;
   z -= v.z;
   return this;
}

Vector4d add(double val) {
   x += val;
   y += val;
   z += val;
   return this;
}

Vector4d subtract(double val) {
   x -= val;
   y -= val;
   z -= val;
   return this;
}

Vector4d multiply(double val) {
   x *= val;
   y *= val;
   z *= val;
   return this;
}

Vector4d multiply(Vector4d v) {
   x *= v.x;
   y *= v.y;
   z *= v.z;
   return this;
}


Vector4d divide(double val) {
   x /= val;
   y /= val;
   z /= val;
   return this;
}

Vector4d divide(Vector4d v) {
   x /= v.x;
   y /= v.y;
   z /= v.z;
   return this;
}


double and(Vector4d v) { // scalar
   return x*v.x + y*v.y + z*v.z;
}

Vector4d multiply(Matrix4d m) {
   Vector4d vec = new Vector4d();
   vec.x = x * m.data[0] + y * m.data[4] + z * m.data[8] + w * m.data[12];
   vec.y = x * m.data[1] + y * m.data[5] + z * m.data[9] + w * m.data[13];
   vec.z = x * m.data[2] + y * m.data[6] + z * m.data[10] + w * m.data[14];
   vec.w = x * m.data[3] + y * m.data[7] + z * m.data[11] + w * m.data[15];
   return vec;
}


Vector4d multiplyVectorial(Vector4d v) { // vectorial
   return new Vector4d(y * v.z - z * v.y,
                   z * v.x - x * v.z,
                   x * v.y - y * v.x,
                   1);
}


static Vector4d multiply(double s, Vector4d v ){
   return new Vector4d(v.x*s, v.y*s, v.z*s, 1);
}

boolean equals(Vector4d v) {
   return (boolean)(x == v.x && y == v.y && z == v.z && w == v.w);
}

boolean notEquals(Vector4d v){
   return (boolean)(x != v.x || y != v.y || z != v.z || w != v.w);
}

double getLength() {
   return Math.sqrt(x*x + y*y + z*z);
}

Vector4d /*Vector4d::operator - ()*/ negated(){
   return new Vector4d(-x, -y, -z, -w);
}
void negate() {
   x = -x; y = -y; z = -z; w = -w;
}

void Normalize() {
   double length = getLength();
   x /= length;
   y /= length;
   z /= length;
}

Vector4d Normalized() {
   return divide(getLength());
}

double Distance(Vector4d v) {
   return Math.sqrt(Math.pow(x-v.x,2)+Math.pow(y-v.y,2)+Math.pow(z-v.z,2));
}

Vector4d Harmonized(){
   this.w = 1.0;
   return divide(z);
}

void Harmonize()  {
   //*this /= data[3];
   //this->w = 1.0;
   this.w = 1.0;
   divide(z);
}
double get(int index) {
	switch(index) {
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	case 3:
		return w;
	default:
		return x;
	}
	//if ( index>=0 && index<=3 ) return data.get(index); return data.get(0); }
}
}

