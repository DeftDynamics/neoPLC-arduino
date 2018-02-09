#include <Eigen.h>           // Calls main Eigen matrix class library 
//#include <Eigen/Core>        // Matrix and Array classes, basic linear algebra (including triangular and selfadjoint products), array manipulation
#include <Eigen/Geometry>    // Transform, Translation, Scaling, Rotation2D and 3D rotations (Quaternion, AngleAxis)
#include <Eigen/LU>          // Inverse, determinant, LU decompositions with solver (FullPivLU, PartialPivLU)
//#include <Eigen/Cholesky>    // LLT and LDLT Cholesky factorization with solver
//#include <Eigen/Householder> // Householder transformations; this module is used by several linear algebra modules
//#include <Eigen/SVD>         // SVD decompositions with least-squares solver (JacobiSVD, BDCSVD)
//#include <Eigen/QR>          // QR decomposition with solver (HouseholderQR, ColPivHouseholderQR, FullPivHouseholderQR)
//#include <Eigen/Eigenvalues> // Eigenvalue, eigenvector decompositions (EigenSolver, SelfAdjointEigenSolver, ComplexEigenSolver)
//#include <Eigen/Sparse>      // Sparse matrix storage and related basic linear algebra (SparseMatrix, SparseVector)
//#include <Eigen/Dense>       // Includes all of: Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header files  
//#include <Eigen/Eigen>       // Includes Dense and Sparse header files (i.e. the whole Eigen library) 

using namespace Eigen;          // Eigen related statement; simplifies syntax for declaration of matrices
typedef Eigen::MatrixXf matrix; // arbitrary size matrix, just to simplify the name
typedef Eigen::VectorXf vector; // arbitrary size vector, just to simplify the name
typedef Eigen::Vector3f vector3; // fixed size 3-vector, just to simplify the name
void printMatrix(const matrix& X, int tol=2); // print the array
void printVector(const vector& x, int tol=2); // print the vector

// quick reference here: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#title0

matrix A(4,4);
matrix B(4,4);
matrix C(4,4);
matrix D(4,4);
matrix E(4,2);
matrix G(4,2);
matrix H(4,4);
matrix I(4,4);
matrix J(8,4);
matrix K(8,4);
matrix L(3,3);
matrix M(3,1);
matrix N(8,3);

vector3 a(3);
vector3 b(3);
vector c(3);
float d;
vector e(3);
float f;
vector g(8);
vector x(3);

int tic = 0;

void setup() {
  
  delay(2000); // wait for serial monitor to warm up
  Serial.println("*** Demonstrating matrix math on neoPLC, enabled by the Eigen library ***\n");

// 1) Construction 

  // easiest way to create a known matrix is using the 'comma initialization' process
  A << 1,2,3,1,
       5,2,1,1,
       3,-1,1,2,
       3,4,5,2;
  Serial.println("create A = ");
  printMatrix(A);

  // there are many special matrices that can be constructed with commands:
  I.setIdentity();
  Serial.println("create I = ");
  printMatrix(I);
  
  // we can insert or retrieve any element using parenthesis notation:
  E.setZero();
  E(0,0) = 4;
  E(2,1) = 6;
  Serial.println("create E = ");
  printMatrix(E);
  Serial.printf("E(2,1) = %2.2f\n\n",E(2,1));
  
  // We can make a matrix of smaller sub matrices
  J << A,
       I;
  Serial.println("J = [A,I]'");
  printMatrix(J);
  
// 2) Matrix Properties

  float aMax = A.maxCoeff();
  float aMin = A.minCoeff();
  Serial.printf("The maximum value in A is %2.2f\n",aMax);
  Serial.printf("The minimum value in A is %2.2f\n\n",aMin);

// 3) Basic Math

  // matrix addition
  C = A+I;
  Serial.println("C = A+I = ");
  printMatrix(C);

  // matrix subtraction
  D = A-I;
  Serial.println("D = A-I = ");
  printMatrix(D);

  // matrix multiplication
  G = A*E;
  Serial.println("G = A*E = ");
  printMatrix(G);

  // matrix*scalar multiplication
  K = 3*I;
  Serial.println("K = 3*I = ");
  printMatrix(K);

  // matrix inversion
  B = A.inverse();
  Serial.println("B = A^-1 = ");
  printMatrix(B);

// 5) vectors 

  // easiest way to create a known vector is using the 'comma initialization' process
  a << 1,2,3;
  b << 3,2,1;
  Serial.print("create a = ");
  printVector(a,4);
  Serial.print("create b = ");
  printVector(b,4);

  // cross product (uses the Eigen/Geometry library, requires fixed size vectors)
  M = a.cross(b);
  Serial.print("M = a x b = ");
  printMatrix(M.transpose(),4);

  // vector dot product
  d = a.dot(b);
  Serial.printf("d = a . b = %2.4f\n\n",d);
  
  // vector*scalar multiplication
  e = 4*a;
  Serial.print("e = 4*a = ");
  printVector(e,4);
  
  // vector norm
  f = a.norm();
  Serial.printf("f = norm(a) = %2.4f\n\n",f);

  Serial.println("\n*** Matrix math demo complete ***\n\n");

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


