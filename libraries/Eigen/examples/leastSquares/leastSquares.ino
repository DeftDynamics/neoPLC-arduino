#include <Eigen.h>           // Calls main Eigen matrix class library 
#include <Eigen/LU>          // Inverse, determinant, LU decompositions with solver (FullPivLU, PartialPivLU)
#include <Eigen/SVD>         // SVD decompositions with least-squares solver (JacobiSVD, BDCSVD)

using namespace Eigen;          // Eigen related statement; simplifies syntax for declaration of matrices
typedef Eigen::MatrixXf matrix; // arbitrary size matrix, just to simplify the name
typedef Eigen::VectorXf vector; // arbitrary size vector, just to simplify the name
void printMatrix(const matrix& X, int tol=2); // print the array
void printVector(const vector& x, int tol=2); // print the vector

// quick reference here: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title0

matrix A(8,3);
vector b(8);
vector x(3);

int tic = 0;

void setup() {
  
  delay(2000); // wait for serial monitor to warm up
  Serial.println("*** Least Squares solutions on neoPLC using Eigen library ***\n");

// SVD Least Squares Solution (uses the Eigen/SVD library)
  
  A <<     1.124015,   0.599901,   0.045109,
           0.072363,  -0.742180,   0.831148,
          -0.455651,  -0.190699,  -0.462148,
           0.233395,  -1.043766,   0.769582,
          -2.473212,   0.183244,  -0.671758,
          -0.045038,  -0.313578,  -0.572786,
           1.424725,   0.079417,  -0.876544,
           1.052354,   0.502563,   0.557932;
  b << 1,0,0,0,0,0,0,0;
  tic = micros();
  
  // fastest technique if we're only using SVD for least squares. Need this class created:
  JacobiSVD<matrix> svd(A, ComputeThinU | ComputeThinV);
  x = svd.solve(b);         // then solve the SVD. (note that this library is quite large)
  int toc1 = micros()-tic;
  Serial.print("given A*x = b, a least squares solution for x (same result as x=(A^-1)*b) computed using SVD is:\n");
  printVector(x,3);

  
// Pseudo-inverse Least Squares Solution (uses the Eigen/Core library)

  Serial.print("compare this to Moore-Penrose pseudo-inverse solution (i.e. x=(A'*(A*A')^-1)*b) is:\n");
  tic = micros();
  x = ((A.transpose()*A).inverse()*A.transpose())*b;
  int toc2 = micros()-tic;
  printVector(x,3);  
  Serial.printf("Expected solution is \n0.081;0.247;0.053\n\n");
  Serial.printf("SVD solved in %d us, vs Pseudo-inverse in %d us.\n",toc1,toc2);

  Serial.println("\n*** Test complete ***\n");

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


