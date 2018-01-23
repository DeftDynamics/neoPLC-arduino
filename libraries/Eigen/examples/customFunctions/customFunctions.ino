#include <Eigen.h>           // Calls main Eigen matrix class library 
//#include <Eigen/Core>        // Matrix and Array classes, basic linear algebra (including triangular and selfadjoint products), array manipulation
//#include <Eigen/Geometry>    // Transform, Translation, Scaling, Rotation2D and 3D rotations (Quaternion, AngleAxis)
//#include <Eigen/LU>          // Inverse, determinant, LU decompositions with solver (FullPivLU, PartialPivLU)

using namespace Eigen;          // Eigen related statement; simplifies syntax for declaration of matrices
typedef Eigen::MatrixXf matrix; // arbitrary size matrix, just to simplify the name
typedef Eigen::VectorXf vector; // arbitrary size vector, just to simplify the name
void printMatrix(const matrix& X, int tol=2); // print the array
void printVector(const vector& x, int tol=2); // print the vector

// quick reference here: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title0

matrix A(4,4);
matrix Ad(4,4);

vector a(3);
vector b(3);
vector c(3);

void setup() {
  
  delay(2000); // wait for serial monitor to warm up
  Serial.println("*** Demonstrating custom functions using matrix math on neoPLC ***\n");
  
// Custom function to solve the matrix exponential. Accepts and returns MatrixXf, although 
// the input must be square for the result to be meaningful.
  A << 1,2,3,1,
       5,2,1,1,
       3,-1,1,2,
       3,4,5,2;
  Ad = expm(A,1e-7);
  Serial.println("Ad = expm(A) = ");
  printMatrix(Ad);

// vector cross product using a custom function
  a << 1,2,3;
  b << 3,2,1;
  c = cross(a,b);
  Serial.print("c = a x b = ");
  printVector(c,4);

  Serial.println("*** Test complete ***\n\n");

}

void loop() {
  // do nothing
}

// Simple function to print a matrix to the serial port
void printMatrix(const matrix& X, int tol)
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), tol);
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

// Simple function to print a vector to the serial port
void printVector(const vector& x, int tol)
{
   int j, n;
   n = x.size();
   for (j=0; j<n; j++)
   {
       Serial.print(x(j), tol);
       Serial.print(", ");
   }
   Serial.println('\n');
}

// Matrix exponential computed using Taylor Series
// example of building a complicated function which returns a matrix (no pointers needed when called!)
matrix expm(const matrix& A, float tol) {
  int nrow = A.rows();
  matrix Err(nrow,nrow);
  matrix E(nrow,nrow);
  matrix e(nrow,nrow);

  int n = 100;
  E.setIdentity();
  for (int j=1; j<=n;j++){
    e = A;
    float scale = 1.0;
    for (int k=1; k<=(j-1); k++){
      e = e*A;
      scale = scale*(k+1);
    }
    e = e/scale;
    Err = E;
    E = E+e;
    Err = E-Err;
    Err = Err.array().abs();
    if (Err.maxCoeff()<tol){
        //Serial.printf("matrix exponential converged to within %1.2e in %d iterations\n",tol,j);
        break;
    } else if (j==n){
        //Serial.printf("matrix exponential did not converge in %d iterations\n",j);
    }
  }
  return E;
}

vector cross(const vector& a, const vector& b){
   int n = a.size();
   vector c(n);
   c.setZero();
   c(0) = a(1)*b(2) - a(2)*b(1);
   c(1) = a(2)*b(0) - a(0)*b(2);
   c(2) = a(0)*b(1) - a(1)*b(0); 
   return c;
}

