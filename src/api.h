#pragma once

// NOLINTBEGIN(readability-identifier-naming)

#if defined _WIN32 || defined __CYGWIN__
#define KinovaLearning_DLLIMPORT __declspec(dllimport)
#define KinovaLearning_DLLEXPORT __declspec(dllexport)
#define KinovaLearning_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#if __GNUC__ >= 4
#define KinovaLearning_DLLIMPORT __attribute__((visibility("default")))
#define KinovaLearning_DLLEXPORT __attribute__((visibility("default")))
#define KinovaLearning_DLLLOCAL __attribute__((visibility("hidden")))
#else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#define KinovaLearning_DLLIMPORT
#define KinovaLearning_DLLEXPORT
#define KinovaLearning_DLLLOCAL
#endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef KinovaLearning_STATIC
// If one is using the library statically, get rid of
// extra information.
#define KinovaLearning_DLLAPI
#define KinovaLearning_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#ifdef KinovaLearning_EXPORTS
#define KinovaLearning_DLLAPI KinovaLearning_DLLEXPORT
#else
#define KinovaLearning_DLLAPI KinovaLearning_DLLIMPORT
#endif // KinovaLearning_EXPORTS
#define KinovaLearning_LOCAL KinovaLearning_DLLLOCAL
#endif // KinovaLearning_STATIC

// NOLINTEND(readability-identifier-naming)
