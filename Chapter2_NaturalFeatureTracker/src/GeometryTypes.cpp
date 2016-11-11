/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/


////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

//
//Matrix44 Matrix44::getInvertedRT() const
//{
//	Matrix44 t = identity();
//
//	for (int col = 0; col < 3; col++)
//	{
//		for (int row = 0; row < 3; row++)
//		{
//			// Transpose rotation component (inversion)
//			t.mat[row][col] = mat[col][row];
//		}
//
//		// Inverse translation component
//		t.mat[3][col] = -mat[3][col];
//	}
//	return t;
//}

Transformation::Transformation()
	: m_rotation(Matrix33::eye())
	, m_translation(Vector3::all(0))
{

}

Transformation::Transformation(const Matrix33& r, const Vector3& t)
	: m_rotation(r)
	, m_translation(t)
{

}

Matrix33& Transformation::r()
{
	return m_rotation;
}

Vector3&  Transformation::t()
{
	return  m_translation;
}

const Matrix33& Transformation::r() const
{
	return m_rotation;
}

const Vector3&  Transformation::t() const
{
	return  m_translation;
}

Matrix44 Transformation::getMat44() const
{
	Matrix44 res = Matrix44::eye();

	for (int col = 0; col < 3; col++)
	{
		for (int row = 0; row < 3; row++)
		{
			// Copy rotation component
			res(row, col) = m_rotation(row, col);
		}

		// Copy translation component
		res(3, col) = m_translation(col);
	}

	return res;
}

Transformation Transformation::getInverted() const
{
	return Transformation(m_rotation.t(), -m_translation);
}