#pragma once
#include <string>

namespace nanite
{
	namespace utils
	{
		inline std::string ExtractDirectory(const std::string& path)
		{
			size_t lastSlash = path.find_last_of("/\\");
			if (lastSlash != std::string::npos)
			{
				return path.substr(lastSlash + 1);
			}
			return "";
		}

		inline std::string ExtractFileName(const std::string& path)
		{
			size_t lastSlash = path.find_last_of("/\\");
			size_t lastDot = path.find_last_of('.');
			if (lastSlash != std::string::npos && lastDot != std::string::npos)
			{
				return path.substr(lastSlash + 1, lastDot - lastSlash - 1);
			}
			else if (lastSlash != std::string::npos)
			{
				return path.substr(lastSlash + 1);
			}
			else if (lastDot != std::string::npos)
			{
				return path.substr(0, lastDot);
			}
			return "";
		}

		inline std::string ExtractExtension(const std::string& path)
		{
			size_t lastDot = path.find_last_of('.');
			if (lastDot != std::string::npos)
			{
				return path.substr(lastDot + 1);
			}
			return "";
		}
	}
}