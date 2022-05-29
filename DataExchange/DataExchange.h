// ==============================================================================
// 
// Data exchange format for UT-SIM framework (www.ut-sim.ca)
// 
// Developed by Dr. Xu Huang (xu.huang@mail.utoronto.ca)
//              @ University of Toronto
// Version: 2.0.1 (Messageheader->version == 2)
// Last updated on Dec. 12, 2021
// Change log:
// - function name changes: updatesubtype -> updatedatatype
//                              getformat -> printheader
//                              terminate -> close
//
//==============================================================================

/*#include <stdint.h>*/
#include <winsock2.h>

// define data types
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;


#ifndef dll_h
#define dll_h

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#ifdef EXPORT_FCNS
#define FUNCSDLL_API __declspec(dllexport)
#else
#define FUNCSDLL_API __declspec(dllimport)
#endif
#else
#define FUNCSDLL_API
#endif

    
	FUNCSDLL_API void updatemessageheader (uint8_t version, uint8_t command, uint8_t testtype, 
        					   uint8_t subtype, uint8_t precision, uint16_t numdofs);
            
	FUNCSDLL_API int setupconnection(int port, SOCKET* ClientSocket, int flag, char* machineInetAddr, int protocol); // either client or server
    FUNCSDLL_API int initialization (SOCKET ClientSocket, int flag, int protocol); // 
    FUNCSDLL_API uint8_t command(SOCKET ClientSocket, int flag, int protocol);
    FUNCSDLL_API int senddata (SOCKET ClientSocket, double* sdata, int len, int protocol);
	FUNCSDLL_API int recvdata (SOCKET ClientSocket, double* response, int len, int protocol);
	FUNCSDLL_API int close(SOCKET* ClientSocket);
        
	FUNCSDLL_API uint16_t getnumdof (void);
    FUNCSDLL_API uint16_t getnumstep (void);
    FUNCSDLL_API void updatecommand (uint8_t command);
    FUNCSDLL_API void updatenumstep (uint16_t step);
    FUNCSDLL_API uint8_t getsubtype (void);
    FUNCSDLL_API void updatedatatype (int disp, int vel, int accel, int force, int stiff, int mass, int temp);
	FUNCSDLL_API void updatenumdofs (uint16_t ndfs);
    FUNCSDLL_API int indicator (void);
	FUNCSDLL_API void printheader (void);

#ifdef  __cplusplus
}
#endif

#endif