#pragma once
#include <cmath>
#include <cstring>

#include "FVector3.hpp"

namespace nanite
{
	using FLOAT = float;

	struct FMatrix3x3
	{
		FLOAT m[3][3]; // row-major

		FMatrix3x3();                      // 기본 생성자 (0으로 초기화)
		FMatrix3x3(const FLOAT* data);     // 배열 초기화
		FMatrix3x3(const FMatrix3x3& other);

		static FMatrix3x3 Identity();
		static FMatrix3x3 Zero();

		inline FMatrix3x3 Inverse() const;
		inline FMatrix3x3 Transposed() const;
		inline float Determinant() const;

		FMatrix3x3& operator=(const FMatrix3x3& other);
		inline FLOAT& operator[](size_t idx);
		inline const FLOAT& operator[](size_t idx) const;

		FMatrix3x3 operator*(const FMatrix3x3& rhs) const;
		FVector3 MultiplyVector(const FVector3& v) const;

		FLOAT* operator[](int row) { return m[row]; }
		const FLOAT* operator[](int row) const { return m[row]; }
	};

	// --- 구현 ---

	inline FMatrix3x3::FMatrix3x3()
	{
		std::memset(m, 0, sizeof(m));
	}

	inline FMatrix3x3::FMatrix3x3(const FLOAT* data)
	{
		std::memcpy(m, data, sizeof(m));
	}

	inline FMatrix3x3::FMatrix3x3(const FMatrix3x3& other)
	{
		std::memcpy(m, other.m, sizeof(m));
	}

	inline FMatrix3x3& FMatrix3x3::operator=(const FMatrix3x3& other)
	{
		if (this != &other)
			std::memcpy(m, other.m, sizeof(m));
		return *this;
	}

	inline FLOAT& FMatrix3x3::operator[](size_t idx)
	{
		return *(reinterpret_cast<FLOAT*>(this) + idx);
	}

	inline const FLOAT& FMatrix3x3::operator[](size_t idx) const
	{
		return *(reinterpret_cast<const FLOAT*>(this) + idx);
	}

	inline FMatrix3x3 FMatrix3x3::Identity()
	{
		FMatrix3x3 mat;
		for (int i = 0; i < 3; ++i)
			mat.m[i][i] = 1.0f;
		return mat;
	}

	inline FMatrix3x3 FMatrix3x3::Zero()
	{
		return FMatrix3x3();
	}

 

	inline FMatrix3x3 FMatrix3x3::operator*(const FMatrix3x3& rhs) const
	{
		FMatrix3x3 result;
		for (int row = 0; row < 3; ++row)
		{
			for (int col = 0; col < 3; ++col)
			{
				result.m[row][col] = 0.f;
				for (int k = 0; k < 3; ++k)
					result.m[row][col] += m[row][k] * rhs.m[k][col];
			}
		}
		return result;
	}

	inline FVector3 FMatrix3x3::MultiplyVector(const FVector3& v) const
	{
		return FVector3{
			m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
		};
	}
}
