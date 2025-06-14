#pragma once
#include <string>
#include <random>
#include <sstream>
#include <iomanip>

#include "UUID.h"

namespace nanite
{
	namespace utils
	{
		class UUIDGnerator
		{
		public:
			UUIDGnerator()
				: mGen(std::random_device{}())
				, mDist(0, static_cast<int>(ALPHANUM.size()) - 1)
			{
				
			}
			UUIDGnerator(const UUIDGnerator&) = delete;
			UUIDGnerator& operator=(const UUIDGnerator&) = delete;
			UUIDGnerator(UUIDGnerator&&) = delete;
			~UUIDGnerator() = default;

			UUID operator()()
			{

				std::string result;
				result.reserve(UUID::LENGTH);
				for (size_t i = 0; i < UUID::LENGTH; ++i)
				{
					result += ALPHANUM[mDist(mGen)];
				}

				return UUID(result);
			}

		public:
			inline static const std::string ALPHANUM = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

		private:
			std::mt19937 mGen;
			std::uniform_int_distribution<> mDist;
		};
	}
}