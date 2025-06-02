#pragma once
#include <cmath>
#include <array>
#include <cstring>

#include "FVector4.hpp"

namespace nanite
{
	using FLOAT = float;

	struct FMatrix4x4
	{
		FLOAT m[4][4]; // row-major

		FMatrix4x4(); // 기본 생성자
		FMatrix4x4(const FLOAT* data); // 16개 float로 초기화
		FMatrix4x4(const FMatrix4x4& other);

		static FMatrix4x4 Identity();
		static FMatrix4x4 Zero();

		inline FMatrix4x4 Inverse() const;
		inline FMatrix4x4 Transposed() const;
		inline float Determinant() const;

		FMatrix4x4& operator=(const FMatrix4x4& other);
		inline FLOAT& operator[](size_t idx);
		inline const FLOAT& operator[](size_t idx) const;
		// 행렬 * 벡터 연산
		FVector4 MultiplyPoint(const FVector4& v) const;

		// 행렬 곱
		FMatrix4x4 operator*(const FMatrix4x4& rhs) const;

		FLOAT* operator[](int row) { return m[row]; }
		const FLOAT* operator[](int row) const { return m[row]; }
	};

	// --- 구현 ---

	inline FMatrix4x4::FMatrix4x4()
	{
		std::memset(m, 0, sizeof(m));
	}

	inline FMatrix4x4::FMatrix4x4(const FLOAT* data)
	{
		std::memcpy(m, data, sizeof(m));
	}

	inline FMatrix4x4::FMatrix4x4(const FMatrix4x4& other)
	{
		std::memcpy(m, other.m, sizeof(m));
	}

	inline FMatrix4x4& FMatrix4x4::operator=(const FMatrix4x4& other)
	{
		if (this != &other)
			std::memcpy(m, other.m, sizeof(m));
		return *this;
	}

	inline FLOAT& FMatrix4x4::operator[](size_t idx)
	{
		return *(reinterpret_cast<FLOAT*>(this) + idx);
	}

	inline const FLOAT& FMatrix4x4::operator[](size_t idx) const
	{
		return *(reinterpret_cast<const FLOAT*>(this) + idx);
	}

	inline FMatrix4x4 FMatrix4x4::Identity()
	{
		FMatrix4x4 mat;
		for (int i = 0; i < 4; ++i)
			mat.m[i][i] = 1.0f;
		return mat;
	}

	inline FMatrix4x4 FMatrix4x4::Zero()
	{
		return FMatrix4x4();
	}

	inline FMatrix4x4 FMatrix4x4::Inverse() const
	{
		FMatrix4x4 inv;
		float det = Determinant();

		if (fabs(det) < 1e-6f)
		{
			return FMatrix4x4(); // 역행렬 없음
		}

		float invDet = 1.0f / det;

#define M(i,j) (*this)[i][j]

		inv[0][0] = (M(1, 1) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) - M(1, 2) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) + M(1, 3) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1))) * invDet;
		inv[0][1] = -(M(0, 1) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) - M(0, 2) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) + M(0, 3) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1))) * invDet;
		inv[0][2] = (M(0, 1) * (M(1, 2) * M(3, 3) - M(1, 3) * M(3, 2)) - M(0, 2) * (M(1, 1) * M(3, 3) - M(1, 3) * M(3, 1)) + M(0, 3) * (M(1, 1) * M(3, 2) - M(1, 2) * M(3, 1))) * invDet;
		inv[0][3] = -(M(0, 1) * (M(1, 2) * M(2, 3) - M(1, 3) * M(2, 2)) - M(0, 2) * (M(1, 1) * M(2, 3) - M(1, 3) * M(2, 1)) + M(0, 3) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1))) * invDet;

		inv[1][0] = -(M(1, 0) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) - M(1, 2) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) + M(1, 3) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0))) * invDet;
		inv[1][1] = (M(0, 0) * (M(2, 2) * M(3, 3) - M(2, 3) * M(3, 2)) - M(0, 2) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) + M(0, 3) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0))) * invDet;
		inv[1][2] = -(M(0, 0) * (M(1, 2) * M(3, 3) - M(1, 3) * M(3, 2)) - M(0, 2) * (M(1, 0) * M(3, 3) - M(1, 3) * M(3, 0)) + M(0, 3) * (M(1, 0) * M(3, 2) - M(1, 2) * M(3, 0))) * invDet;
		inv[1][3] = (M(0, 0) * (M(1, 2) * M(2, 3) - M(1, 3) * M(2, 2)) - M(0, 2) * (M(1, 0) * M(2, 3) - M(1, 3) * M(2, 0)) + M(0, 3) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0))) * invDet;

		inv[2][0] = (M(1, 0) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) - M(1, 1) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) + M(1, 3) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))) * invDet;
		inv[2][1] = -(M(0, 0) * (M(2, 1) * M(3, 3) - M(2, 3) * M(3, 1)) - M(0, 1) * (M(2, 0) * M(3, 3) - M(2, 3) * M(3, 0)) + M(0, 3) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))) * invDet;
		inv[2][2] = (M(0, 0) * (M(1, 1) * M(3, 3) - M(1, 3) * M(3, 1)) - M(0, 1) * (M(1, 0) * M(3, 3) - M(1, 3) * M(3, 0)) + M(0, 3) * (M(1, 0) * M(3, 1) - M(1, 1) * M(3, 0))) * invDet;
		inv[2][3] = -(M(0, 0) * (M(1, 1) * M(2, 3) - M(1, 3) * M(2, 1)) - M(0, 1) * (M(1, 0) * M(2, 3) - M(1, 3) * M(2, 0)) + M(0, 3) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0))) * invDet;

		inv[3][0] = -(M(1, 0) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) - M(1, 1) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)) + M(1, 2) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))) * invDet;
		inv[3][1] = (M(0, 0) * (M(2, 1) * M(3, 2) - M(2, 2) * M(3, 1)) - M(0, 1) * (M(2, 0) * M(3, 2) - M(2, 2) * M(3, 0)) + M(0, 2) * (M(2, 0) * M(3, 1) - M(2, 1) * M(3, 0))) * invDet;
		inv[3][2] = -(M(0, 0) * (M(1, 1) * M(3, 2) - M(1, 2) * M(3, 1)) - M(0, 1) * (M(1, 0) * M(3, 2) - M(1, 2) * M(3, 0)) + M(0, 2) * (M(1, 0) * M(3, 1) - M(1, 1) * M(3, 0))) * invDet;
		inv[3][3] = (M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) - M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) + M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0))) * invDet;

#undef M

		return inv;
	}

	inline FMatrix4x4 FMatrix4x4::Transposed() const
	{
		FMatrix4x4 result;
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				result.m[i][j] = m[j][i];
		return result;
	}

	inline float FMatrix4x4::Determinant() const
	{
		const float a00 = (*this)[0][0], a01 = (*this)[0][1], a02 = (*this)[0][2], a03 = (*this)[0][3];
		const float a10 = (*this)[1][0], a11 = (*this)[1][1], a12 = (*this)[1][2], a13 = (*this)[1][3];
		const float a20 = (*this)[2][0], a21 = (*this)[2][1], a22 = (*this)[2][2], a23 = (*this)[2][3];
		const float a30 = (*this)[3][0], a31 = (*this)[3][1], a32 = (*this)[3][2], a33 = (*this)[3][3];

		return
			a00 * (
				a11 * (a22 * a33 - a23 * a32) -
				a12 * (a21 * a33 - a23 * a31) +
				a13 * (a21 * a32 - a22 * a31)
				) -
			a01 * (
				a10 * (a22 * a33 - a23 * a32) -
				a12 * (a20 * a33 - a23 * a30) +
				a13 * (a20 * a32 - a22 * a30)
				) +
			a02 * (
				a10 * (a21 * a33 - a23 * a31) -
				a11 * (a20 * a33 - a23 * a30) +
				a13 * (a20 * a31 - a21 * a30)
				) -
			a03 * (
				a10 * (a21 * a32 - a22 * a31) -
				a11 * (a20 * a32 - a22 * a30) +
				a12 * (a20 * a31 - a21 * a30)
				);
	}

	inline FMatrix4x4 FMatrix4x4::operator*(const FMatrix4x4& rhs) const
	{
		FMatrix4x4 result;
		for (int row = 0; row < 4; ++row)
		{
			for (int col = 0; col < 4; ++col)
			{
				result.m[row][col] = 0.f;
				for (int k = 0; k < 4; ++k)
					result.m[row][col] += m[row][k] * rhs.m[k][col];
			}
		}
		return result;
	}

	inline FVector4 FMatrix4x4::MultiplyPoint(const FVector4& v) const
	{
		FLOAT x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w;
		FLOAT y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w;
		FLOAT z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w;
		FLOAT w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w;
		return FVector4{ x, y, z, w };
	}
}
