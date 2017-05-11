/*************************************************************/
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/* Contributing Programmer(s):                               */
/* Revision History:                                         */
/*************************************************************/
#ifndef _H_setup
#define _H_setup

#define GENERIC 0
#define UNIX_V  0
#define UNIX_7  0
#define MAC_MCW 0
#define IBM_MCW 0
#define IBM_MSC 0
#define IBM_TBC 0
#define IBM_GCC 1

#define IBM_ZTC 0
#define IBM_ICB 0
#define IBM_SC  0
#define MAC_SC6 0
#define MAC_SC7 0
#define MAC_SC8 0
#define MAC_MPW 0
#define VAX_VMS 0
#define MAC_XCD 0

#if IBM_ZTC || IBM_MSC || IBM_TBC || IBM_ICB || IBM_SC || IBM_MCW
#define IBM 1
#else
#define IBM 0
#endif

#if MAC_SC6 || MAC_SC7 || MAC_SC8
#define MAC_SC 1
#else
#define MAC_SC 0
#endif

#if MAC_SC || MAC_MPW || MAC_MCW || MAC_XCD
#define MAC 1
#else
#define MAC 0
#endif


#if CLIPS_MAJOR >= 6

#if CLIPS_MINOR < 23
#error "Cannot build using CLIPS version less than 6.23"
#endif /* CLIPS_MINOR >= 23 */

#define VOID     void
#define VOID_ARG void
#define STD_SIZE size_t

#if CLIPS_MINOR < 24
#define BOOLEAN int
#else
#define intBool int
#endif /* CLIPS_MINOR < 24 */

#define globle

#define ALLOW_ENVIRONMENT_GLOBALS 1
#define BASIC_IO 1
#define BLOAD 0
#define BLOAD_AND_BSAVE 1
#define BLOAD_ONLY 0
#define BLOAD_INSTANCES 1
#define BLOCK_MEMORY 0
#define BSAVE_INSTANCES 1
#define CONFLICT_RESOLUTION_STRATEGIES 1
#define CONSTRUCT_COMPILER 0
#define DEBUGGING_FUNCTIONS 1
#define DEFFACTS_CONSTRUCT 1
#define DEFFUNCTION_CONSTRUCT 1
#define DEFGENERIC_CONSTRUCT 1
#define DEFGLOBAL_CONSTRUCT 1
#define DEFINSTANCES_CONSTRUCT 1
#define DEFMODULE_CONSTRUCT 1
#define DEFRULE_CONSTRUCT 1
#define DEFTEMPLATE_CONSTRUCT 1
#define EMACS_EDITOR 0
#define ENVIRONMENT_API_ONLY 0
#define EX_MATH 1
#define EXT_IO 1
#define FACT_SET_QUERIES 1
#define HELP_FUNCTIONS 0
#define INSTANCE_SET_QUERIES 1
#define MULTIFIELD_FUNCTIONS 1
#define OBJECT_SYSTEM 1
#define PROFILING_FUNCTIONS 1
#define RUN_TIME 0
#define STRING_FUNCTIONS 1
#define TEXTPRO_FUNCTIONS 1
#define WINDOW_INTERFACE 1

#define DEVELOPER 0

#if CLIPS_MINOR < 24
#define AUXILIARY_MESSAGE_HANDLERS 1
#define DYNAMIC_SALIENCE 1
#define IMPERATIVE_MESSAGE_HANDLERS 1
#define IMPERATIVE_METHODS 1
#define INCREMENTAL_RESET 1
#define INSTANCE_PATTERN_MATCHING 1
#define LOGICAL_DEPENDENCIES  1
#define SHORT_LINK_NAMES 0
#endif  /* CLIPS_MINOR < 24 */

#else   /* CLIPS_MAJOR >= 6 */
#error "Cannot build using CLIPS version less than 6.23"
#endif

#include "envrnmnt.h"

#define Bogus(x)
#define PrintCLIPS(x,y) EnvPrintRouter(GetCurrentEnvironment(),x,y)
#define GetcCLIPS(x,y) EnvGetcRouter(GetCurrentEnvironment(),x)
#define UngetcCLIPS(x,y) EnvUngetcRouter(GetCurrentEnvironment(),x,y)
#define ExitCLIPS(x) EnvExitRouter(GetCurrentEnvironment(),x)
#define CLIPSSystemError(x,y) SystemError(x,y)
#define CLIPSFunctionCall(x,y,z) FunctionCall(x,y,z)
#define InitializeCLIPS() InitializeEnvironment()
#define WCLIPS WPROMPT
#define CLIPSTrueSymbol SymbolData(GetCurrentEnvironment())->TrueSymbol
#define CLIPSFalseSymbol SymbolData(GetCurrentEnvironment())->FalseSymbol
#define EnvCLIPSTrueSymbol(theEnv) SymbolData(theEnv)->TrueSymbol
#define EnvCLIPSFalseSymbol(theEnv) SymbolData(theEnv)->FalseSymbol
#define CLIPS_FALSE 0
#define CLIPS_TRUE 1
#if BLOCK_MEMORY
#define INITBLOCKSIZE 32000
#define BLOCKSIZE 32000
#endif

#include "usrsetup.h"

#endif  /* _H_setup */
