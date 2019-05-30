package com.neocoretechs.machinevision.hough3d;
/**
 * 4D matrix
 * @author jg
 *
 */
public class Matrix4d {
    double data[] = new double[16];
	final static double Z[] = {0.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 0.0f};
	final static double I[] = {1.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 1.0f};
void swap(double[] arr, int f1, int f2) {
   double k = arr[f1];
   arr[f1] = arr[f2];
   arr[f2] = k;
}

public Matrix4d() {
   zero();
}

public Matrix4d(Matrix4d m) {
   System.arraycopy(m, 0, data, 0, 16);
}

public Matrix4d(double f) {
   for(int i=0; i<16; i++)
      data[i] = f;
}
public Matrix4d(double[] f) {
	   System.arraycopy(f, 0, data, 0, 16);
}
Matrix4d set(Matrix4d m) {
   System.arraycopy(m, 0, data, 0, 16);
   return this;
}

double get(int index1, int index2) {
   return data[index1*4+index2];
}

Matrix4d add(Matrix4d m) {
   for (int i=0; i<16; i++)
      data[i] += m.data[i];
   return this;
}


Matrix4d subtract(Matrix4d m) {
   for(int i=0; i<16; i++)
      data[i] -= m.data[i];
   return this;
}

Matrix4d multiply(Matrix4d m) {
   Matrix4d t = multiply2(m);
   set(t); // set this to computed values
   return t;
}

Matrix4d add(double val) {
   for(int i=0; i<16; i++)
      data[i] += val;
   return this;
}

Matrix4d subtract(double val) {
   for(int i=0; i<16; i++)
      data[i] -= val;
   return this;
}

Matrix4d multiply(double val) {
   for(int i=0; i<16; i++)
      data[i] *= val;
   return this;
}

Matrix4d divide(double val) {
   for(int i=0; i<16; i++)
      data[i] /= val;
   return this;
}

Matrix4d add2(Matrix4d m){
   Matrix4d t = new Matrix4d(this);
   return t.add(m);
}

Matrix4d subtract2(Matrix4d m) {
   Matrix4d t = new Matrix4d(this);
   return t.subtract(m);
}


Matrix4d multiply2(Matrix4d m) {
   Matrix4d t = Zeros();
   int i, j, k;
   for (i=0; i<4; i++)
      for (j=0; j<4; j++)
         for (k=0; k<4; k++)
            t.data[i*4+j] += data[i*4+k] * m.data[k*4+j];
   return t;
}

Matrix4d add2(double val) {
   Matrix4d t = new Matrix4d(this);
   return t.add(val);
}

Matrix4d subtract2(double val) {
   Matrix4d t = new Matrix4d(this);
   return t.subtract(val);
}

Matrix4d multiply2(double val) {
   Matrix4d t = new Matrix4d(this);
   return t.multiply(val);
}

Matrix4d divide2(double val){
   Matrix4d t = new Matrix4d(this);
   return t.divide(val);
}

public boolean equals(Matrix4d m) {
   for(int i=0; i<16; i++)
      if (data[i] != m.data[i]) return false;
   return true;
}

boolean notEquals(Matrix4d m){
   for(int i=0; i<16; i++)
      if (data[i] != m.data[i]) return true;
   return false;
}

public static Matrix4d Identity() {
   return new Matrix4d(I);
}

static Matrix4d Zeros() {
   return new Matrix4d(Z);
}

public void zero() {
   System.arraycopy(Z, 0, data, 0, 16);
}

void Invert() {
   double det = Det();
   double detij;
   int i, j;
   Matrix4d mInverse = Zeros();
   det = 1.0f / det;
   for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
         detij = DetIJ(data, j, i);
         mInverse.data[i*4+j] = (((i+j) & 0x1) != 0) ? (-detij * det) : (detij *det);
      }
   }
   set(mInverse);
}

void Transpose() {
   swap(data,1,4);
   swap(data,2,8);
   swap(data,3,12);
   swap(data,6,9);
   swap(data,7,13);
   swap(data,11,14);
}

double DetIJ(double[] m, int i, int j) {
   int x, y, ii, jj;
   double ret;
   double[][] mat = new double[3][3];
   x = 0;
   for (ii = 0; ii < 4; ii++) {
      if (ii == i) continue;
      y = 0;
      for (jj = 0; jj < 4; jj++)    {
         if (jj == j) continue;
         mat[x][y] = m[ii*4+jj];
         y++;
      }
      x++;
   }
   ret =  mat[0][0]*(mat[1][1]*mat[2][2]-mat[2][1]*mat[1][2]);
   ret -= mat[0][1]*(mat[1][0]*mat[2][2]-mat[2][0]*mat[1][2]);
   ret += mat[0][2]*(mat[1][0]*mat[2][1]-mat[2][0]*mat[1][1]);
   return ret;
}

double Det() {
	double det = 0.0f;
	int i;
	for (i = 0; i < 4; i++) {
		det += (((i & 0x1) != 0) ? (-data[i] * DetIJ(data, 0, i)) : (data[i] * DetIJ(data,0,i)));
	}
	return det;
}


static Matrix4d Translation(double x, double y, double z) {
   Matrix4d t = Identity();
   t.data[12] = x;
   t.data[13] = y;
   t.data[14] = z;
   return t;
}

static Matrix4d Translation(Vector4d v) {
   Matrix4d t = Identity();
   t.data[3] = v.x;
   t.data[7] = v.y;
   t.data[11] = v.z;
   return t;
}



static Matrix4d Scaling(double x, double y, double z) {
   Matrix4d t = Identity();
   t.data[0] = x;
   t.data[5] = y;
   t.data[10] = z;
   return t;
}


Matrix4d Rotation(double angle, double x, double y, double z) {
   double vecLength, sinSave, cosSave, oneMinusCos;
   double xx, yy, zz, xy, yz, zx, xs, ys, zs;
   Matrix4d t = Zeros();
   // Scale vector
   vecLength = Math.sqrt(x*x + y*y + z*z);
   // Rotation matrix is normalized
   x /= vecLength;
   y /= vecLength;
   z /= vecLength;
   sinSave = Math.sin(angle);
   cosSave = Math.cos(angle);
   oneMinusCos = 1.0f - cosSave;
   xx = x * x;
   yy = y * y;
   zz = z * z;
   xy = x * y;
   yz = y * z;
   zx = z * x;
   xs = x * sinSave;
   ys = y * sinSave;
   zs = z * sinSave; 
   t.data[0] = (oneMinusCos * xx) + cosSave;  t.data[1] = (oneMinusCos * xy) + zs;        t.data[2] = (oneMinusCos * zx) - ys;         t.data[3] = 0.0f;
   t.data[4] = (oneMinusCos * xy) - zs;       t.data[5] = (oneMinusCos * yy) + cosSave;   t.data[6] = (oneMinusCos * yz) + xs;         t.data[7] = 0.0f;
   t.data[8] = (oneMinusCos * zx) + ys;       t.data[9] = (oneMinusCos * yz) - xs;        t.data[10] = (oneMinusCos * zz) + cosSave;   t.data[11] = 0.0f;
   t.data[12] = 0.0f;                         t.data[13] = 0.0f;                          t.data[14] = 0.0f;                           t.data[15] = 1.0f;
   return t;
}

Matrix4d RotationQuaternion(double angle, double x, double y, double z) {
   double q0, q1, q2, q3, q12, q22, q32;
   Vector4d q;   
   double cosHalfAngle = Math.cos(angle/2.0);
   double sinHalfAngle = Math.sin(angle/2.0);
   Vector4d u = new Vector4d(x,y,z);
   u.Normalize();
   q0 = cosHalfAngle;
   q = u.multiply(sinHalfAngle);
   q1 = q.get(0);
   q2 = q.get(1);
   q3 = q.get(2);
   q12 = q1*q1;
   q22 = q2*q2;
   q32 = q3*q3;
   Matrix4d t = new Matrix4d();
   t.data[0] =  1.0 - 2.0 * (q22 + q32); t.data[1]  = 2.0 * (q1*q2 - q0*q3);   t.data[2]  = 2.0 * (q1*q3 + q0*q2);   t.data[3]  = 0.0;
   t.data[4] =  2.0 * (q1*q2 + q0*q3);   t.data[5]  = 1.0 - 2.0 * (q12 + q32); t.data[6]  = 2.0 * (q2*q3 - q0*q1);   t.data[7]  = 0.0;
   t.data[8] =  2.0 * (q1*q3 - q0*q2);   t.data[9]  = 2.0 * (q2*q3 + q0*q1);   t.data[10] = 1.0 - 2.0 * (q12 + q22); t.data[11] = 0.0;
   t.data[12] = 0.0;                     t.data[13] = 0.0;                     t.data[14] = 0.0;                     t.data[15] = 1.0; 
   return t;
}

Matrix4d Rotation(double angle, Vector4d v) {
   return Rotation(angle,v.x,v.y,v.z);
}
Matrix4d RotationQuaternion(double angle, Vector4d v) {
   return RotationQuaternion(angle,v.x,v.y,v.z);
}

static Matrix4d RotationX(double angle) {
   Matrix4d t = Zeros();
   double sina = Math.sin(angle);
   double cosa = Math.cos(angle);
   t.data[0] = 1.0f;     t.data[1] = 0.0f;      t.data[2] = 0.0f;      t.data[3] = 0.0f;
   t.data[4] = 0.0f;     t.data[5] = cosa;      t.data[6] = -sina;     t.data[7] = 0.0f;
   t.data[8] = 0.0f;     t.data[9] = sina;      t.data[10] = cosa;     t.data[11] = 0.0f;
   t.data[12] = 0.0f;    t.data[13] = 0.0f;     t.data[14] = 0.0f;     t.data[15] = 1.0f;
   return t;
}

static Matrix4d RotationY(double angle) {
   Matrix4d t = Zeros();
   double sina = Math.sin(angle);
   double cosa = Math.cos(angle);
   t.data[0] = cosa;     t.data[1] = 0.0f;     t.data[2] = sina;     t.data[3] = 0.0f;
   t.data[4] = 0.0;      t.data[5] = 1.0f;     t.data[6] = 0.0f;     t.data[7] = 0.0f;
   t.data[8] = -sina;    t.data[9] = 0.0f;     t.data[10] = cosa;    t.data[11] = 0.0f;
   t.data[12] = 0.0f;    t.data[13] = 0.0f;    t.data[14] = 0.0f;    t.data[15] = 1.0f;
   return t;
}

static Matrix4d RotationZ(double angle) {
   Matrix4d t = Zeros();
   double sina = Math.sin(angle);
   double cosa = Math.cos(angle);
   t.data[0] = cosa;    t.data[1] = -sina;   t.data[2] = 0.0f;    t.data[3] = 0.0f;
   t.data[4] = sina;    t.data[5] = cosa;    t.data[6] = 0.0f;    t.data[7] = 0.0f;
   t.data[8] = 0.0f;    t.data[9] = 0.0f;    t.data[10] = 1.0f;   t.data[11] = 0.0f;
   t.data[12] = 0.0f;   t.data[13] = 0.0f;   t.data[14] = 0.0f;   t.data[15] = 1.0f;
   return t;
}

Vector4d multiply(Vector4d rhs) {

   return new Vector4d(
      data[0] * rhs.x + data[1] * rhs.y + data[2] * rhs.z + data[3] * rhs.w,
      data[4] * rhs.x + data[5] * rhs.y + data[6] * rhs.z + data[7] * rhs.w,
      data[8] * rhs.x + data[9] * rhs.y + data[10] * rhs.z + data[11] * rhs.w,
      data[12] * rhs.x + data[13] * rhs.y + data[14] * rhs.z + data[15] * rhs.w
   );
}

static Matrix4d Perspective(double r) {
	Matrix4d t = Identity();
	t.data[10] = 0.0f;
	t.data[14] = -1.0f / r;
	return t;
}
}
