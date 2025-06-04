#pragma once
#include <cmath>
#include <cstring>
#include <cassert>

#include "FVector3.hpp"

namespace nanite
{
	using FLOAT = float;

	struct FMatrix3x3
	{
		FLOAT m[3][3]; // row-major

		FMatrix3x3();
		FMatrix3x3(const FLOAT* data);
		FMatrix3x3(std::initializer_list<FLOAT> list);
		FMatrix3x3(const FMatrix3x3& other);

		FMatrix3x3& operator=(const FMatrix3x3& other);
		inline FLOAT& operator[](size_t idx);
		inline const FLOAT& operator[](size_t idx) const;
		inline FLOAT* operator[](int row) { return m[row]; }
		inline const FLOAT* operator[](int row) const { return m[row]; }

		static FMatrix3x3 Identity();
		static FMatrix3x3 Zero();

		inline FMatrix3x3 Inverse() const;
		inline FMatrix3x3 Transposed() const;
		inline float Determinant() const;

		inline FMatrix3x3 operator+=(const FMatrix3x3& other);
		inline FMatrix3x3 operator-=(const FMatrix3x3& other);
		inline FMatrix3x3 operator*=(FLOAT v);
		inline FMatrix3x3 operator/=(FLOAT v);
	};

	inline FMatrix3x3 operator-(const FMatrix3x3& mat);
	inline FMatrix3x3 operator+(const FMatrix3x3& lhs, const FMatrix3x3& rhs);
	inline FMatrix3x3 operator-(const FMatrix3x3& lhs, const FMatrix3x3& rhs);
	inline FMatrix3x3 operator*(const FMatrix3x3& lhs, const FMatrix3x3& rhs);
	inline FMatrix3x3 operator*(FLOAT v, const FMatrix3x3& mat);
	inline FMatrix3x3 operator*(const FMatrix3x3& mat, FLOAT v);
	inline FVector3 operator*(const FMatrix3x3& mat, const FVector3 vec);
	inline FMatrix3x3 operator/(const FMatrix3x3& mat, FLOAT v);
	inline bool operator==(const FMatrix3x3& lhs, const FMatrix3x3& rhs);
	inline bool operator!=(const FMatrix3x3& lhs, const FMatrix3x3& rhs);

	// --- Implementation ---

	inline FMatrix3x3::FMatrix3x3()
	{
		std::memset(m, 0, sizeof(m));
	}

	inline FMatrix3x3::FMatrix3x3(const FLOAT* data)
	{
		std::memcpy(m, data, sizeof(m));
	}

	inline FMatrix3x3::FMatrix3x3(std::initializer_list<FLOAT> list)
	{
		assert(list.size() == 9);
		std::copy(list.begin(), list.end(), &m[0][0]);
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

	inline FMatrix3x3 FMatrix3x3::Inverse() const
	{
		FMatrix3x3 inv;

		float det = Determinant();
		if (fabs(det) < 1e-6f)
		{
			return FMatrix3x3();
		}

		float invDet = 1.0f / det;

		inv[0][0] = ((*this)[1][1] * (*this)[2][2] - (*this)[1][2] * (*this)[2][1]) * invDet;
		inv[0][1] = ((*this)[0][2] * (*this)[2][1] - (*this)[0][1] * (*this)[2][2]) * invDet;
		inv[0][2] = ((*this)[0][1] * (*this)[1][2] - (*this)[0][2] * (*this)[1][1]) * invDet;

		inv[1][0] = ((*this)[1][2] * (*this)[2][0] - (*this)[1][0] * (*this)[2][2]) * invDet;
		inv[1][1] = ((*this)[0][0] * (*this)[2][2] - (*this)[0][2] * (*this)[2][0]) * invDet;
		inv[1][2] = ((*this)[0][2] * (*this)[1][0] - (*this)[0][0] * (*this)[1][2]) * invDet;

		inv[2][0] = ((*this)[1][0] * (*this)[2][1] - (*this)[1][1] * (*this)[2][0]) * invDet;
		inv[2][1] = ((*this)[0][1] * (*this)[2][0] - (*this)[0][0] * (*this)[2][1]) * invDet;
		inv[2][2] = ((*this)[0][0] * (*this)[1][1] - (*this)[0][1] * (*this)[1][0]) * invDet;

		return inv;
	}

	inline FMatrix3x3 FMatrix3x3::Transposed() const
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				result.m[i][j] = m[j][i];
		return result;
	}

	inline float FMatrix3x3::Determinant() const
	{
		return
			(*this)[0][0] * ((*this)[1][1] * (*this)[2][2] - (*this)[1][2] * (*this)[2][1]) -
			(*this)[0][1] * ((*this)[1][0] * (*this)[2][2] - (*this)[1][2] * (*this)[2][0]) +
			(*this)[0][2] * ((*this)[1][0] * (*this)[2][1] - (*this)[1][1] * (*this)[2][0]);
	}

	inline FMatrix3x3 FMatrix3x3::operator+=(const FMatrix3x3& other)
	{
		*this = *this + other;
	}

	inline FMatrix3x3 FMatrix3x3::operator-=(const FMatrix3x3& other)
	{
		*this = *this - other;
	}

	inline FMatrix3x3 FMatrix3x3::operator*=(FLOAT v)
	{
		*this = *this * v;
	}

	inline FMatrix3x3 FMatrix3x3::operator/=(FLOAT v)
	{
		*this = *this / v;
	}

	inline FMatrix3x3 operator-(const FMatrix3x3& mat)
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				result.m[i][j] = -mat.m[i][j];
			}
		}
		return result;
	}

	inline FMatrix3x3 operator+(const FMatrix3x3& lhs, const FMatrix3x3& rhs)
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				result.m[i][j] = lhs.m[i][j] + rhs.m[i][j];
			}
		}
		return result;
	}

	inline FMatrix3x3 operator-(const FMatrix3x3& lhs, const FMatrix3x3& rhs)
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				result.m[i][j] = lhs.m[i][j] - rhs.m[i][j];
			}
		}
		return result;
	}

	inline FMatrix3x3 operator*(const FMatrix3x3& lhs, const FMatrix3x3& rhs)
	{
		FMatrix3x3 result;
		for (int row = 0; row < 3; ++row)
		{
			for (int col = 0; col < 3; ++col)
			{
				result.m[row][col] = 0.f;
				for (int k = 0; k < 3; ++k)
				{
					result.m[row][col] += lhs.m[row][k] * rhs.m[k][col];
				}
			}
		}
		return result;
	}

	inline FMatrix3x3 operator*(FLOAT v, const FMatrix3x3& mat) 
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				result.m[i][j] = v * mat.m[i][j];
			}
		}
		return result;
	}

	inline FMatrix3x3 operator*(const FMatrix3x3& mat, FLOAT v)
	{
		return v * mat;
	}

	inline FVector3 operator*(const FMatrix3x3& mat, const FVector3 vec) 
	{
		return FVector3(
			mat[0][0] * vec.x + mat[0][1] * vec.y + mat[0][2] * vec.z,
			mat[1][0] * vec.x + mat[1][1] * vec.y + mat[1][2] * vec.z,
			mat[2][0] * vec.x + mat[2][1] * vec.y + mat[2][2] * vec.z
		);
	}

	inline FMatrix3x3 operator/(const FMatrix3x3& mat, FLOAT v)
	{
		FMatrix3x3 result;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				result.m[i][j] = mat.m[i][j] / v;
			}
		}
		return result;
	}

	inline bool operator==(const FMatrix3x3& lhs, const FMatrix3x3& rhs)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (lhs.m[i][j] != rhs.m[i][j])
				{
					return false;
				}
			}
		}
		return true;
	}

	inline bool operator!=(const FMatrix3x3& lhs, const FMatrix3x3& rhs)
	{
		return !(lhs == rhs);
	}
}
