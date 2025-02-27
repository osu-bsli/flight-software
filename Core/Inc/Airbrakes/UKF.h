/**
 * UKF.h
 * Unscented Kalman Filter declarations and constants
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ==================== Constants ==================== */

#define SIZE 18
#define ROCKET_MASS 20
#define grav 9.80665
#define AccC 0.01
#define GyroC 0.001
#define GPSC 0.05

/* ==================== Function Prototypes ==================== */

/* -------------------- UKF Core -------------------- */

/* Process model function: modifies 'sigma' in place */
void processModel(double *sigma, double t, double *DeployAngle, double *sensorVector);

/* Compute mean of sigma points */
void computeMean(double *sigma, double *Weights, double *stateMean);

/* Predict covariance from sigma points and add ProcessNoise */
void PredictCovariance(double *sigma, double *stateMean, double *PredictedCovariance, double *weights, double *ProcessNoise);

/* Measurement function: transforms each sigma point into measurement space */
void MeasurementFunction(double *Measurement, double *sigma);

/* Cross-covariance between state sigma points and measurement sigma points */
void CrossCovariance(double *sigma, double *measurementMatrix, double *stateMean, double *measurementMean, double *crossCovariance, double *weights);

/* Update the state vector using the Kalman gain and measurement residual */
void updateState(double *X0, double *K, double *Zpred, double *Zinit, double *stateVector);

/* -------------------- Cholesky / Covariance Helpers -------------------- */

/* Cholesky Decomposition (in-place) of an n x n matrix A (lower-triangular) */
void choleskyDecomp(double *A, int n);

/* Scale entire matrix A by scalar (A *= scalar) */
void scaleMatrix(double *A, int n, double scalar);

/* Invert a lower-triangular matrix L, result in L_inv */
void invertLowerTriangular(double *L, double *L_inv, int n);

/* Multiply L_inv^T * L_inv and store result in A_inv (n x n) */
void multiplyTranspose(double *L_inv, double *A_inv, int n);

/* Invert an n x n positive-definite matrix A using Cholesky */
void invertMatrixCholesky(double *A, double *A_inv, int n);

/* Rank-1 update/downdate multiple columns */
void cholUpdateMulti(double *Sx, double *U, int n, int m, int sign);

/* Rank-1 update/downdate single vector x */
void cholUpdate(double *S, double *x, int n, int sign);

/* -------------------- Linear Algebra Helpers -------------------- */

/* Dot product (BLAS style) of length N with increments incX, incY */
double dotProduct(const int N, const double *X, const int incX, const double *Y, const int incY);

/* Matrix multiply: C = A * B */
void matrixMultiply(double *A, double *B, double *C, int rowsA, int colsA, int colsB);

/* Matrix transpose for a x a square matrices */
void matrixTranspose(double *A, double *A_T, int a);

/* -------------------- Misc Helpers -------------------- */

/* Gaussian random: returns ~N(0,1) */
double randn(void);

/* Example drag function: given angle & velocity, returns some drag value */
double drag(double theta, double velocity);

/* Example density model (rho) */
double rho(double height);

/* Example surface area function (angleOfDeployment) */
double surfaceA(double angleOfDeployment);

/* ==================== Deployment Angle Optimization ==================== */

/* Predicts an optimal deployment angle using an iterative approach */
double PredictDeploymentAngle(double *stateVector, double previousAngle);

/* Cost function used by PredictDeploymentAngle() */
double cost(double deploymentAngle, double *stateVector);

/* Numerical gradient for cost function */
double computeGrad(double deploymentAngle, double *stateVector, double epsilon);

/* Example apogee prediction for cost function */
double PredictApogee(double *stateVector, double deploymentAngle);

#ifdef __cplusplus
}
#endif
