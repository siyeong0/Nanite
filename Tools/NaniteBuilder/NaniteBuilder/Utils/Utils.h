#pragma once

#include "../Math/FVector3.hpp"

namespace nanite
{
	FVector3 HSVtoRGB(float h, float s, float v)
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
}