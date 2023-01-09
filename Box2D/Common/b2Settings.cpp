/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Common/b2Settings.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

b2Version b2_version = {2, 4, 0};

#define LIQUIDFUN_VERSION_MAJOR 1
#define LIQUIDFUN_VERSION_MINOR 1
#define LIQUIDFUN_VERSION_REVISION 0
#define LIQUIDFUN_STRING_EXPAND(X) #X
#define LIQUIDFUN_STRING(X) LIQUIDFUN_STRING_EXPAND(X)

const b2Version b2_liquidFunVersion = {
	LIQUIDFUN_VERSION_MAJOR, LIQUIDFUN_VERSION_MINOR,
	LIQUIDFUN_VERSION_REVISION,
};

const char *b2_liquidFunVersionString =
"LiquidFun "
LIQUIDFUN_STRING(LIQUIDFUN_VERSION_MAJOR) "."
LIQUIDFUN_STRING(LIQUIDFUN_VERSION_MINOR) "."
LIQUIDFUN_STRING(LIQUIDFUN_VERSION_REVISION);

// Memory allocators. Modify these to use your own allocator.
void* b2Alloc_Default(int32 size)
{
	return malloc(size);
}

void b2Free_Default(void* mem)
{
	free(mem);
}

// You can modify this to use your logging facility.
void b2Log_Default(const char* string, va_list args)
{
	vprintf(string, args);
}

FILE* b2_dumpFile = nullptr;

void b2OpenDump(const char* fileName)
{
	b2Assert(b2_dumpFile == nullptr);
	b2_dumpFile = fopen(fileName, "w");
}

void b2Dump(const char* string, ...)
{
	if (b2_dumpFile == nullptr)
	{
		return;
	}

	va_list args;
	va_start(args, string);
	vfprintf(b2_dumpFile, string, args);
	va_end(args);
}

void b2CloseDump()
{
	fclose(b2_dumpFile);
	b2_dumpFile = nullptr;
}

//CORONASDK/CORONALABS NOTE:
//THIS CLASS ("Validator") CAN CAUSE LINK ERRORS IF EXPOSED (For example,
//with Apple's OpenEars framework). IT'S ALSO NOT BEING USED ANYWHERE,
//SO WE'RE WRAPPING IT IN AN ANONYMOUS NAMESPACE TO KEEP IT PRIVATE TO
//THIS FILE!!!
// namespace
// {
// 	class Validator
// 	{
// 	public:
// 		Validator()
// 		{
// 			b2Assert(sizeof(uint64)==8);
// 			b2Assert(sizeof(int64)==8);
// 		}
// 	} validate;
// }

float b2Settings::linearSlop = 0.005f * b2_lengthUnitsPerMeter;
float b2Settings::velocityThreshold = b2_velocityThreshold;
float b2Settings::timeToSleep = b2_timeToSleep;
int32 b2Settings::maxSubSteps = b2_maxSubSteps;
float b2Settings::linearSleepTolerance = b2_linearSleepTolerance;
float b2Settings::angularSleepTolerance = b2_angularSleepTolerance;
float b2Settings::linearSleepToleranceSq = b2_linearSleepTolerance * b2_linearSleepTolerance;
float b2Settings::angularSleepToleranceSq = b2_angularSleepTolerance * b2_angularSleepTolerance;
