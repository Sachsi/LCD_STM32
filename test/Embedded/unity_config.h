/* Unity Configuration
 * As of May 11th, 2016 at ThrowTheSwitch/Unity commit 837c529
 * Update: December 29th, 2016
 * See Also: Unity/docs/UnityConfigurationGuide.pdf
 *
 * Unity is designed to run on almost anything that is targeted by a C compiler.
 * It would be awesome if this could be done with zero configuration. While
 * there are some targets that come close to this dream, it is sadly not
 * universal. It is likely that you are going to need at least a couple of the
 * configuration options described in this document.
 *
 * All of Unity's configuration options are `#defines`. Most of these are simple
 * definitions. A couple are macros with arguments. They live inside the
 * unity_internals.h header file. We don't necessarily recommend opening that
 * file unless you really need to. That file is proof that a cross-platform
 * library is challenging to build. From a more positive perspective, it is also
 * proof that a great deal of complexity can be centralized primarily to one
 * place in order to provide a more consistent and simple experience elsewhere.
 *
 * Using These Options
 * It doesn't matter if you're using a target-specific compiler and a simulator
 * or a native compiler. In either case, you've got a couple choices for
 * configuring these options:
 *
 *  1. Because these options are specified via C defines, you can pass most of
 *     these options to your compiler through command line compiler flags. Even
 *     if you're using an embedded target that forces you to use their
 *     overbearing IDE for all configuration, there will be a place somewhere in
 *     your project to configure defines for your compiler.
 *  2. You can create a custom `unity_config.h` configuration file (present in
 *     your toolchain's search paths). In this file, you will list definitions
 *     and macros specific to your target. All you must do is define
 *     `UNITY_INCLUDE_CONFIG_H` and Unity will rely on `unity_config.h` for any
 *     further definitions it may need.
 */

#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifndef NULL
#ifndef __cplusplus
#define NULL (void*)0
#else
#define NULL 0
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef G0

extern void unityOutputStart();
extern void unityOutputChar(char);
extern void unityOutputFlush();
extern void unityOutputComplete();

#define UNITY_OUTPUT_START()    unityOutputStart()
#define UNITY_OUTPUT_CHAR(c)    unityOutputChar(c)
#define UNITY_OUTPUT_FLUSH()    unityOutputFlush()
#define UNITY_OUTPUT_COMPLETE() unityOutputComplete()
#endif //G0

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* UNITY_CONFIG_H */