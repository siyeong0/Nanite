#pragma once
#include <cmath>
#include <limits>

namespace nanite
{
	using FLOAT = float;

	struct FVector4
	{
		FLOAT x, y, z, w;

		FVector4() = default;
		~FVector4() = default;

		FVector4(FLOAT x, FLOAT y, FLOAT z, FLOAT w);
		FVector4(const FVector4& other);
		FVector4& operator=(const FVector4& other);
		inline FLOAT& operator[](size_t idx);
		inline const FLOAT& operator[](size_t idx) const;

		static inline FVector4 Zero() { return FVector4{ 0.f, 0.f, 0.f, 0.f }; }
		static inline FVector4 One() { return FVector4{ 1.f, 1.f, 1.f, 1.f }; }

		FLOAT Dot(const FVector4& other) const;
		FLOAT Magnitude() const;
		FLOAT SqrMagnitude() const;
		FLOAT Length() const;

		FVector4 Norm() const;
		void Normalize();

		FVector4 operator+=(const FVector4& other);
		FVector4 operator-=(const FVector4& other);
		FVector4 operator*=(FLOAT v);
		FVector4 operator/=(FLOAT v);
	};

	// --- 연산자 오버로딩 (비멤버) ---

	inline FVector4 operator-(const FVector4& vec)
	{
		return FVector4{ -vec.x, -vec.y, -vec.z, -vec.w };
	}

	inline FVector4 operator+(const FVector4& lhs, const FVector4& rhs)
	{
		return FVector4{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w };
	}

	inline FVector4 operator-(const FVector4& lhs, const FVector4& rhs)
	{
		return FVector4{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w };
	}

	inline FVector4 operator*(FLOAT v, const FVector4& vec)
	{
		return FVector4{ v * vec.x, v * vec.y, v * vec.z, v * vec.w };
	}

	inline FVector4 operator*(const FVector4& vec, FLOAT v)
	{
		return FVector4{ v * vec.x, v * vec.y, v * vec.z, v * vec.w };
	}

	inline FVector4 operator/(const FVector4& vec, FLOAT v)
	{
		return FVector4{ vec.x / v, vec.y / v, vec.z / v, vec.w / v };
	}

	inline bool operator==(const FVector4& lhs, const FVector4& rhs)
	{
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
	}

	inline bool operator!=(const FVector4& lhs, const FVector4& rhs)
	{
		return !(lhs == rhs);
	}

	// --- 구현 ---

	inline FVector4::FVector4(FLOAT x, FLOAT y, FLOAT z, FLOAT w)
		: x(x), y(y), z(z), w(w) {
	}

	inline FVector4::FVector4(const FVector4& other)
		: x(other.x), y(other.y), z(other.z), w(other.w) {
	}

	inline FVector4& FVector4::operator=(const FVector4& other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
		w = other.w;
		return *this;
	}

	inline FLOAT& FVector4::operator[](size_t idx)
	{
		return *(reinterpret_cast<FLOAT*>(this) + idx);
	}

	inline const FLOAT& FVector4::operator[](size_t idx) const
	{
		return *(reinterpret_cast<const FLOAT*>(this) + idx);
	}

	inline FLOAT FVector4::Dot(const FVector4& other) const
	{
		return x * other.x + y * other.y + z * other.z + w * other.w;
	}

	inline FLOAT FVector4::Magnitude() const
	{
		return Dot(*this);
	}

	inline FLOAT FVector4::SqrMagnitude() const
	{
		return std::sqrtf(Magnitude());
	}

	inline FLOAT FVector4::Length() const
	{
		return SqrMagnitude();
	}

	inline FVector4 FVector4::Norm() const
	{
		FLOAT l = Length();
		if (l == 0)
			return FVector4{ 0.f, 0.f, 0.f, 0.f };
		else
			return *this / l;
	}

	inline void FVector4::Normalize()
	{
		FLOAT l = Length();
		if (l != 0)
			*this /= l;
	}

	inline FVector4 FVector4::operator+=(const FVector4& other)
	{
		*this = *this + other;
		return *this;
	}

	inline FVector4 FVector4::operator-=(const FVector4& other)
	{
		*this = *this - other;
		return *this;
	}

	inline FVector4 FVector4::operator*=(FLOAT v)
	{
		*this = *this * v;
		return *this;
	}

	inline FVector4 FVector4::operator/=(FLOAT v)
	{
		*this = *this / v;
		return *this;
	}
}
