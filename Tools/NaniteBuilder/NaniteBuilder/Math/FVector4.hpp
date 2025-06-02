#pragma once
#include <cmath>
#include <limits>

namespace nanite
{
	using FLOAT = float;

	struct FVector4
	{
		FLOAT x;
		FLOAT y;
		FLOAT z;
		FLOAT w;

		FVector4() = default;
		~FVector4() = default;

		FVector4(FLOAT x, FLOAT y, FLOAT z, FLOAT w);
		FVector4(const FVector4& other);
		FVector4& operator=(const FVector4& other);
		inline FLOAT& operator[](size_t idx);
		inline const FLOAT& operator[](size_t idx) const;

		static inline FVector4 Zero() { return FVector4{ 0.f, 0.f, 0.f, 0.f }; }
		static inline FVector4 One() { return FVector4{ 1.f, 1.f, 1.f, 1.f }; }
		static inline FVector4 FMaxValue() { constexpr float v = std::numeric_limits<float>::max(); return FVector4{ v, v, v, v }; }
		static inline FVector4 FMinValue() { constexpr float v = std::numeric_limits<float>::lowest(); return FVector4{ v, v, v, v }; }

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

		static inline FVector4 Abs(const FVector4& vec);
		static inline FVector4 Min(const FVector4& a, const FVector4& b);
		static inline FVector4 Max(const FVector4& a, const FVector4& b);
		static inline FVector4 Clamp(const FVector4& value, const FVector4& min, const FVector4& max);
		static inline FVector4 Lerp(const FVector4& a, const FVector4& b, FLOAT t);
		static inline FVector4 SmoothStep(const FVector4& a, const FVector4& b, FLOAT t);
	};

	FVector4 operator-(const FVector4& vec);
	FVector4 operator+(const FVector4& lhs, const FVector4& rhs);
	FVector4 operator-(const FVector4& lhs, const FVector4& rhs);
	FVector4 operator*(const FVector4& lhs, const FVector4& rhs);
	FVector4 operator*(FLOAT v, const FVector4& vec);
	FVector4 operator*(const FVector4& vec, FLOAT v);
	FVector4 operator/(const FVector4& vec, FLOAT v);
	bool operator==(const FVector4& lhs, const FVector4& rhs);
	bool operator!=(const FVector4& lhs, const FVector4& rhs);


	// --- Implementation ---

	inline FVector4::FVector4(FLOAT x, FLOAT y, FLOAT z, FLOAT w)
		: x(x), y(y), z(z), w(w) {
	}

	inline FVector4::FVector4(const FVector4& other)
		: x(other.x), y(other.y), z(other.z), w(other.w) {
	}

	inline FVector4& FVector4::operator=(const FVector4& other)
	{
		x = other.x; y = other.y; z = other.z; w = other.w;
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
		return (l == 0.f) ? FVector4{ 0.f, 0.f, 0.f, 0.f } : *this / l;
	}

	inline void FVector4::Normalize()
	{
		FLOAT l = Length();
		if (l != 0.f) *this /= l;
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

	inline FVector4 FVector4::Abs(const FVector4& vec)
	{
		return FVector4{ std::fabs(vec.x), std::fabs(vec.y), std::fabs(vec.z), std::fabs(vec.w) };
	}

	inline FVector4 FVector4::Min(const FVector4& a, const FVector4& b)
	{
		return FVector4{ std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z), std::fmin(a.w, b.w) };
	}

	inline FVector4 FVector4::Max(const FVector4& a, const FVector4& b)
	{
		return FVector4{ std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z), std::fmax(a.w, b.w) };
	}

	inline FVector4 FVector4::Clamp(const FVector4& value, const FVector4& min, const FVector4& max)
	{
		return FVector4{
			std::fmax(min.x, std::fmin(value.x, max.x)),
			std::fmax(min.y, std::fmin(value.y, max.y)),
			std::fmax(min.z, std::fmin(value.z, max.z)),
			std::fmax(min.w, std::fmin(value.w, max.w))
		};
	}

	inline FVector4 FVector4::Lerp(const FVector4& a, const FVector4& b, FLOAT t)
	{
		t = std::fmax(0.f, std::fmin(t, 1.f));
		return FVector4{
			a.x + (b.x - a.x) * t,
			a.y + (b.y - a.y) * t,
			a.z + (b.z - a.z) * t,
			a.w + (b.w - a.w) * t
		};
	}

	inline FVector4 FVector4::SmoothStep(const FVector4& a, const FVector4& b, FLOAT t)
	{
		t = std::fmax(0.f, std::fmin(t, 1.f));
		FLOAT t2 = t * t;
		return FVector4{
			a.x + (b.x - a.x) * t2 * (3.f - 2.f * t),
			a.y + (b.y - a.y) * t2 * (3.f - 2.f * t),
			a.z + (b.z - a.z) * t2 * (3.f - 2.f * t),
			a.w + (b.w - a.w) * t2 * (3.f - 2.f * t)
		};
	}

	// --- Operators (Non member functions) ---

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

	inline FVector4 operator*(const FVector4& lhs, const FVector4& rhs)
	{
		return FVector4{ lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w };
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
		return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z) && (lhs.w == rhs.w);
	}

	inline bool operator!=(const FVector4& lhs, const FVector4& rhs)
	{
		return !(lhs == rhs);
	}
}
