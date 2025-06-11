#pragma once
#include "Geometry.h"
#include "Path.h"
#include "PRIME_ARRAY.h"

namespace nanite
{
	namespace utils
	{
		inline FVector3 HSVtoRGB(float h, float s, float v)
		{
			float c = v * s;
			float x = c * (1.f - abs(fmod(h * 6.f, 2.f) - 1.f));
			float m = v - c;

			FVector3 rgb;

			if (h < 1.0 / 6.0)
				rgb = FVector3(c, x, 0);
			else if (h < 2.0 / 6.0)
				rgb = FVector3(x, c, 0);
			else if (h < 3.0 / 6.0)
				rgb = FVector3(0, c, x);
			else if (h < 4.0 / 6.0)
				rgb = FVector3(0, x, c);
			else if (h < 5.0 / 6.0)
				rgb = FVector3(x, 0, c);
			else
				rgb = FVector3(c, 0, x);

			return rgb + FVector3{ m, m, m };
		}

		inline bool IsPrime(size_t num)
		{
			if (num < 2) return false;
			size_t limit = static_cast<size_t>(std::sqrt(num));
			for (size_t p : PRIME_ARRAY) 
			{
				if (p > limit) break;
				if (num % p == 0) return false;
			}
			return true;
		}

		inline size_t NextPrime(size_t n)
		{
			auto begin = std::begin(PRIME_ARRAY);
			auto end = std::end(PRIME_ARRAY);
			auto it = std::upper_bound(begin, end, n);
			if (it != end) 
			{
				return *it;
			}
			else 
			{
				return 0;
			}
		}
	}
}