#pragma once

#include <opencv2/opencv.hpp>
#include <array>

using Matrix44 = cv::Matx44f;
using Matrix33 = cv::Matx33f;
using Vector3 = cv::Vec3f;
using Vector4 = cv::Vec4f;


struct Transformation
{
	Transformation();
	Transformation(const Matrix33& r, const Vector3& t);

	Matrix33& r();
	Vector3&  t();

	const Matrix33& r() const;
	const Vector3&  t() const;

	Matrix44 getMat44() const;

	Transformation getInverted() const;
private:
	Matrix33 m_rotation;
	Vector3  m_translation;
};

struct Quadrangle
{
	std::array<cv::Point2f,4> corners;

	cv::Point2f& tl() { return corners[0]; }
	cv::Point2f& tr() { return corners[1]; }
	cv::Point2f& bl() { return corners[2]; }
	cv::Point2f& br() { return corners[3]; }
};