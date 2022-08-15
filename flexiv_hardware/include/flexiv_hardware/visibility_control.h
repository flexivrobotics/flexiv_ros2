#ifndef FLEXIV_HARDWARE__VISIBILITY_CONTROL_H_
#define FLEXIV_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define FLEXIV_HARDWARE_EXPORT __attribute__((dllexport))
#define FLEXIV_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define FLEXIV_HARDWARE_EXPORT __declspec(dllexport)
#define FLEXIV_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef FLEXIV_HARDWARE_BUILDING_DLL
#define FLEXIV_HARDWARE_PUBLIC FLEXIV_HARDWARE_EXPORT
#else
#define FLEXIV_HARDWARE_PUBLIC FLEXIV_HARDWARE_IMPORT
#endif
#define FLEXIV_HARDWARE_PUBLIC_TYPE FLEXIV_HARDWARE_PUBLIC
#define FLEXIV_HARDWARE_LOCAL
#else
#define FLEXIV_HARDWARE_EXPORT __attribute__((visibility("default")))
#define FLEXIV_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define FLEXIV_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define FLEXIV_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define FLEXIV_HARDWARE_PUBLIC
#define FLEXIV_HARDWARE_LOCAL
#endif
#define FLEXIV_HARDWARE_PUBLIC_TYPE
#endif

#endif  // FLEXIV_HARDWARE__VISIBILITY_CONTROL_H_
