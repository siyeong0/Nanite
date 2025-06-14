#pragma once
#include <string>
#include <cassert>

namespace nanite
{
	namespace utils
	{
		static const std::string INVALID_UUID_STRING = "00000000-0000-0000-0000-000000000000";

		class UUID
		{
		public:
			UUID() : mData(INVALID_UUID_STRING) { assert(mData.length() == LENGTH); }
			UUID(const std::string& data) : mData(data) { assert(mData.length() == LENGTH); }
			~UUID() {}
			UUID(const UUID& other) : mData(other.mData) {}
			UUID(UUID&& other) noexcept : mData(std::move(other.mData)) {}
			UUID& operator=(const UUID& other) { if (this != &other) mData = other.mData; return *this; }

			bool operator==(const UUID& other) const { return mData == other.mData; }
			const char& operator[](size_t index) const { return mData[index]; }

			bool IsValid() const { return mData != INVALID_UUID_STRING; }

			friend std::ostream& operator<<(std::ostream& os, const UUID& uuid);

		public:
			static const int LENGTH = 36; // Standard UUID length
		private:
			std::string mData;
		};

		inline std::ostream& operator<<(std::ostream& os, const UUID& uuid)
		{
			os << uuid.mData;
			return os;
		}

		const UUID INVALID_UUID(INVALID_UUID_STRING);
	}
}