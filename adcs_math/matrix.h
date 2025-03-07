/**@file matrix.h
 *
 * @brief Interface for matrix utility functions.
 *
 * @author Jacob Tkeio (jacobtkeio@gmail.com) 7/13/2023
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "vector.h"


typedef struct Matrix3x3 {
	double x1;
	double x2;
	double x3;
	double y1;
	double y2;
	double y3;
	double z1;
	double z2;
	double z3;
} mat3; //Use "mat3" as the type.


/**@brief Load a matrix with the given values.
 *
 * @param [xyz][123] The value to put in the [xyz] row and [123] column.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_set(
	double x1, double x2, double x3,
	double y1, double y2, double y3,
	double z1, double z2, double z3,
	mat3 *output
);


/**@brief Load a matrix's columns with the given vectors.
 *
 * @param first The vector to load into the first column.
 * @param second The vector to load into the second column.
 * @param third The vector to load into the third column.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_set_from_vec(
	vec3 first,
	vec3 second,
	vec3 third,
	mat3 *output
);


/**@brief Perform matrix transposition.
 *
 * @param matrix The matrix to transpose.
 * @param output The mat3* that will hold the transposed matrix.
 *
 * @return Void.
 */
void mat_transpose(mat3 matrix, mat3 *output);


/**@brief Add each element of a matrix by each element of another.
 *
 * @param left  One matrix being added.
 * @param right The other matrix being added.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_add(mat3 left, mat3 right, mat3 *output);


/**@brief Subtract each element of a matrix by each element of another.
 *
 * @param left  The matrix being subtracted from.
 * @param right The matrix being subtracted.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_sub(mat3 left, mat3 right, mat3 *output);


/**@brief Multiply each element of a matrix by a scalar value.
 *
 * @param scalar The scalar by which to multiply.
 * @param matrix The matrix being multiplied.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_scalar(double scalar, mat3 matrix, mat3 *output);


/**@brief Multiply two matrices.
 *
 * Note that matrix multiplication is NONCOMMUTATIVE!
 *
 * @param left The matrix being multiplied that is on the left.
 * @param right The matrix being multiplied that is on the right.
 * @param output The mat3* that will hold the result.
 *
 * @return Void.
 */
void mat_mult(mat3 left, mat3 right, mat3 *output);


/**@brief Multiply a matrix with a vector.
 *
 * @param left The matrix on the left side of the multiplication.
 * @param right The vector on the right side of the multiplication.
 * @param output The vec3* that will hold the result.
 *
 * @return Void.
 */
void mat_vec_mult(mat3 left, vec3 right, vec3 *output);


/**@breif Calculate the determinant of a matrix.
 *
 * @param matrix The matrix for which to calculate the determinant.
 *
 * @return The determinant of the matrix.
 */
double mat_det(mat3 matrix);


/**@breif Calculate the inverse of a matrix.
 *
 * @param matrix The matrix for which to calculate the inverse.
 * @param output The mat3* that will hold the result.
 *
 * @return int Error code:  -1 if det(matrix) == 0
 *                          0 if success.
 */
int mat_inverse(mat3 matrix, mat3 *output);


#endif//MATRIX_H





