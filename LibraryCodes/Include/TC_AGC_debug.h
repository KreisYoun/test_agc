
#ifndef TCAGC_DEBUG_CONFIG_H
#define TCAGC_DEBUG_CONFIG_H


#ifdef ARM_SIM
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



// [1] Debug Parameters
extern int		giFrameCounter;
extern char	gcFileName[256];


// [2] Debug Enable
#define TCAGC_DBG_ENABLE		(0)		// Debug Enable
#define TCAGC_DBG_FRAME_COUNT	(10000)


// [3] Functions
static void TCAGC_dbg_file_naming(char* pcSourceName, char* pcRefName, char* pcResultName)
{
	char*	pc_temp;
	strcpy(pcResultName,pcSourceName);
	pc_temp = pcResultName + strlen(pcResultName);
	strcpy(pc_temp,pcRefName);
}

#endif		// ARM SIM
#endif		// TCAGC_DEBUG_CONFIG_H
