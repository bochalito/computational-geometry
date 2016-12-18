#ifdef _WIN32
#	ifdef VVRScene_EXPORTS
#		define VVRScene_API __declspec(dllexport)
#	else
#		define VVRScene_API  __declspec(dllimport)
#	endif
#else
#	define VVRScene_API
#endif // WIN32
