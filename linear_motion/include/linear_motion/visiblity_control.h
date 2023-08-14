#ifndef SLIDER_HARDWARE__VISIBLITY_CONTROL_H_
#define SLIDER_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SLIDER_HARDWARE_EXPORT __attribute__((dllexport))
#define SLIDER_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define SLIDER_HARDWARE_EXPORT __declspec(dllexport)
#define SLIDER_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef SLIDER_HARDWARE_BUILDING_DLL
#define SLIDER_HARDWARE_PUBLIC SLIDER_HARDWARE_EXPORT
#else
#define SLIDER_HARDWARE_PUBLIC SLIDER_HARDWARE_IMPORT
#endif
#define SLIDER_HARDWARE_PUBLIC_TYPE SLIDER_HARDWARE_PUBLIC
#define SLIDER_HARDWARE_LOCAL
#else
#define SLIDER_HARDWARE_EXPORT __attribute__((visibility("default")))
#define SLIDER_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define SLIDER_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define SLIDER_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define SLIDER_HARDWARE_PUBLIC
#define SLIDER_HARDWARE_LOCAL
#endif
#define SLIDER_HARDWARE_PUBLIC_TYPE
#endif

#endif  // SLIDER_HARDWARE__VISIBLITY_CONTROL_H_
