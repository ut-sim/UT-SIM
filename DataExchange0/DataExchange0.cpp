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
#define EXPORT_FCNS
#include "DataExchange0.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>
#include <windows.h>
#include <inaddr.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#define MAX_UDP_DATAGRAM 9126

using namespace std;

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")


union MA {
	struct sockaddr    addr;
    struct sockaddr_in addr_in;
} ;

union MA myaddr;                                  // Server address struct
union MA otheraddr;                               // Client address struct
int AddLength;                                    // address length


//communication data format
// 
/* version 2.0.1
//Command 
#define RemoteTest_setTrialResponse  3            // send trial displacement
#define RemoteTest_setMass           4            // receive mass (no use anymore)
#define RemoteTest_getForce         10            // receive restoring force
#define RemoteTest_getInitialStiff  12            // receive initial stiffness (no use anymore)
#define RemoteTest_DIE              99            // terminate analysis

//substructure types
#define OpenSees                     1            
#define Zeus_NL                      2
#define Abaqus                       3
#define VecTor2                      4   
#define Cyrus                        5
#define SFrame                       6
#define NICON                        7
#define VecTor4                      8

//test type
#define Ramp_hold                    1
#define Continuous                   2
#define Real_time                    3
#define Software_only                4            // component-level
#define Software_only2               5            // system-level 

//Communication precision
#define Single_precision             1
#define Double_precision             2            // default

*/

struct messageheader {
	uint8_t Version;
	uint8_t Command;
    uint8_t Test_type;
	uint8_t Sub_type;
	uint8_t Precision;
	struct Data_type {
		uint8_t disp:1;
		uint8_t vel:1;
		uint8_t accel:1;
		uint8_t force:1;
		uint8_t stiff:1;
		uint8_t mass:1;                            // add mass indicator
		uint8_t temper:1;
		uint8_t resvd:1;
	}datatype;
	uint16_t Num_DOFs;
	uint16_t Step_num;
	uint16_t Reserved;
	uint32_t Time_stamp;
} ;
	
struct messageheader *MessageHeader;

//variables for communication
char* recvbuf;
char* sendbuf;
double* rData;
double* sData;

// convert message from host to net
void 
htonDataheader (struct messageheader* h, char buffer[])
{
    uint16_t snum;
	uint16_t numdofs;
	uint16_t re;
	uint32_t timest;

	snum = htons(h->Step_num);
	numdofs = htons(h->Num_DOFs);
	re = htons(h->Reserved);
	timest = htonl(h->Time_stamp);
	
    memcpy(buffer+0, &(h->Version), 1);
	memcpy(buffer+1, &(h->Command), 1);
	memcpy(buffer+2, &(h->Test_type), 1);
	memcpy(buffer+3, &(h->Sub_type), 1);
    memcpy(buffer+4, &(h->Precision), 1);
	memcpy(buffer+5, &(h->datatype), 1);
	memcpy(buffer+6, &numdofs, 2);
	memcpy(buffer+8, &snum, 2);
	memcpy(buffer+10, &re, 2);
    memcpy(buffer+12, &timest, 4);

	// debugging
	//printf("Version in Send() is =%d\n", h->Version);
	//printf("Command in Send() is =%d\n", h->Command);
	//printf("Test_type in Send() is =%d\n", h->Test_type);
	//printf("Sub_type in Send() is =%d\n", h->Sub_type);
	//printf("Precision in Send() is =%d\n", h->Precision);
	//printf("Num_DOFs in Send() is =%d\n", h->Num_DOFs);
}


// convert message from net to host
void
ntohDataheader(char buffer[], struct messageheader *h) 
{
    uint16_t snum = 0;
	uint16_t numdofs = 0;
	uint16_t re = 0;
	uint32_t timest = 0;
	
	memcpy(&(h->Version), buffer+0, 1);
	memcpy(&(h->Command), buffer+1,  1);
	memcpy(&(h->Test_type), buffer+2,  1);
	memcpy(&(h->Sub_type), buffer+3,  1);
    memcpy(&(h->Precision), buffer+4, 1);
	memcpy(&(h->datatype), buffer+5, 1);
	memcpy(&(numdofs), buffer+6, 2);
	memcpy(&(snum), buffer+8, 2);
	memcpy(&(re), buffer+10, 2);
    memcpy(&(timest), buffer+12, 4);

    h->Step_num = ntohs(snum);
	h->Num_DOFs = ntohs(numdofs);
	h->Reserved = ntohs(re);
	h->Time_stamp = ntohl(timest);

	// debugging
	//printf("Version in Recv() is =%d\n", h->Version);
	//printf("Command in Recv() is =%d\n", h->Command);
	//printf("Test_type in Recv() is =%d\n", h->Test_type);
	//printf("Sub_type in Recv() is =%d\n", h->Sub_type);
	//printf("Precision in Recv() is =%d\n", h->Precision);
	//printf("Num_DOFs in Recv() is =%d\n", h->Num_DOFs);

}

// function to calculate data size appended to the data exchange format
int
indicator (void)
{
	int nd = 0;
	if (MessageHeader->datatype.disp == 1) 
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.vel == 1) 
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.accel == 1)
		nd = nd + MessageHeader->Num_DOFs;
	if (MessageHeader->datatype.force == 1)
		nd = nd + MessageHeader->Num_DOFs;
	
	if (MessageHeader->datatype.stiff == 1) {
			nd = nd + MessageHeader->Num_DOFs * MessageHeader->Num_DOFs;
	}
	
	if (MessageHeader->datatype.mass == 1)
		nd = nd + MessageHeader->Num_DOFs * MessageHeader->Num_DOFs;
	
	if (MessageHeader->datatype.temper == 1) {	
		nd = nd + MessageHeader->Num_DOFs;
	} 
	if (MessageHeader->datatype.resvd) {
		// do nothing
	}

	return nd;
}

void
	printheader (void)
{
	printf("Version      =%d\n", MessageHeader->Version);
	printf("Command      =%d\n", MessageHeader->Command);
	printf("Test_type    =%d\n", MessageHeader->Test_type);
	printf("Sub_type     =%d\n", MessageHeader->Sub_type);
	printf("Precision    =%d\n", MessageHeader->Precision);
	printf("Num_DOFs     =%d\n", MessageHeader->Num_DOFs);

}


// TCP send method
int
TCP_Send(char* sbuf, int dsize, double *sd, SOCKET sockfd)
{
	
	if (MessageHeader->Precision == 1) {// single precision
		
		float* sdata = new float [dsize];
		if (sd !=0) {
			for (int i=0; i<dsize; i++) {
				sdata[i] = (float) sd[i];
			}
		}

		int nwrite;
        char* Msg = sbuf;
	    int nleft = dsize*sizeof(float)+16;

	    htonDataheader(MessageHeader, sbuf);
		if (nleft-16 > 0) {
        memcpy (sbuf+16, sdata, nleft-16);
		}

	    while (nleft > 0) {
			nwrite = send(sockfd,Msg,nleft,0);	
			if (nwrite < 0) {
				closesocket(sockfd);
			    WSACleanup();
			    return -1;
			}
			else if (nwrite == 0) {}
			else {
			nleft -= nwrite;
			Msg += nwrite;
			}
		}

		delete[] sdata;
		
	} else if (MessageHeader->Precision == 2) {//double precision

		int nwrite;
        char* Msg = sbuf;
	    int nleft = dsize*sizeof(double)+16;

	    htonDataheader(MessageHeader, sbuf);

		if (nleft-16 > 0) {
        memcpy (sbuf+16, sd, nleft-16);
		}

	    while (nleft > 0) {
			nwrite = send(sockfd,Msg,nleft,0);
			if (nwrite < 0) {
				closesocket(sockfd);
			    WSACleanup();
			    return -1;
			}
			else if (nwrite == 0) {}
			else {
			nleft -= nwrite;
			Msg += nwrite;
			}
		}

	} else {
		printf("The defined precision %d in TCP_Send() is not accepted\n", MessageHeader->Precision);
	}

	


	return 0;
}


// TCP receive method
int 
TCP_Recv(char* rbuf, int dsize, double *rd , SOCKET sockfd)
{
	
	if (MessageHeader->Precision == 1) {//single precision

	int nread = 0;
	char* Msg = rbuf;
	
	int nleft = dsize*sizeof(float)+16;

	while (nleft > 0) {
		nread = recv(sockfd,Msg,nleft,0);
		if (nread < 0) {
			closesocket(sockfd);
			WSACleanup();
			return -1;
		}
		else if (nread == 0) {}
		else {
			nleft -= nread;
			Msg += nread;
		}
	}

	ntohDataheader(rbuf, MessageHeader);
    
	float* rdata = new float [dsize];

	memcpy (rdata, rbuf+16, dsize*sizeof(float));	
	for (int i=0; i<dsize; i++) {
		rd[i] = (double) rdata[i];
	}

	delete[] rdata;

	} else if (MessageHeader->Precision == 2) {//double precision
		

		int nread = 0;
	    char* Msg = rbuf;
	    
	    int nleft = dsize*sizeof(double)+16;
	    
	    while (nleft > 0) {
	    	nread = recv(sockfd,Msg,nleft,0);
			if (nread < 0) {
			    closesocket(sockfd);
			    WSACleanup();
			    return -1;
			}
			else if (nread == 0) {}
			else {
				nleft -= nread;
			    Msg += nread;
			}
	    }
	    

	    ntohDataheader(rbuf, MessageHeader);
        
	    memcpy (rd, rbuf+16, dsize*sizeof(double));	
	    
	} else {
		printf("The defined precision %d in TCP_Recv() is not accepted\n", MessageHeader->Precision);
	}

	return 0;
}


// UDP send method
int
UDP_Send(char* sbuf, int dsize, double *sd, SOCKET sockfd)
{
	
	if (MessageHeader->Precision == 1) {// single precision
		
		float* sdata = new float [dsize];
		if (sd !=0) {
			for (int i=0; i<dsize; i++) {
				sdata[i] = (float) sd[i];
			}
		}

        char* Msg = sbuf;
	    int nleft = dsize*sizeof(float)+16;

	    htonDataheader(MessageHeader, sbuf);
		if (nleft-16 > 0) {
        memcpy (sbuf+16, sdata, nleft-16);
		}

	    while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				sendto(sockfd, Msg, nleft, 0, &otheraddr.addr, AddLength);
			    nleft =0;
			}
			else {
				sendto(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}
		

		delete[] sdata;
		
	} else if (MessageHeader->Precision == 2) {//double precision

        char* Msg = sbuf;
	    int nleft = dsize*sizeof(double)+16;

	    htonDataheader(MessageHeader, sbuf);

		if (nleft-16 > 0) {
        memcpy (sbuf+16, sd, nleft-16);
		}

	    while (nleft > 0) {
			if (nleft <= MAX_UDP_DATAGRAM) {
				sendto(sockfd, Msg, nleft, 0, &otheraddr.addr, AddLength);
			    nleft =0;
			}
			else {
				sendto(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, AddLength);
				Msg += MAX_UDP_DATAGRAM;
				nleft -= MAX_UDP_DATAGRAM;
			}
		}

	}

	return 0;
}


// UDP receive method
int 
UDP_Recv(char* rbuf, int dsize, double *rd , SOCKET sockfd)
{
	
	if (MessageHeader->Precision == 1) {//single precision

	int nread = 0;
	char* Msg = rbuf;
	
	int nleft = dsize*sizeof(float)+16;

	while (nleft > 0) {
		if (nleft <= MAX_UDP_DATAGRAM) {
			recvfrom(sockfd, Msg, nleft, 0, &otheraddr.addr, &AddLength);
		    nleft =0;
		}
		else {
			recvfrom(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, &AddLength);
			Msg += MAX_UDP_DATAGRAM;
			nleft -= MAX_UDP_DATAGRAM;
		}
	}

	ntohDataheader(rbuf, MessageHeader);
    
	float* rdata = new float [dsize];

	memcpy (rdata, rbuf+16, dsize*sizeof(float));	
	for (int i=0; i<dsize; i++) {
		rd[i] = (double) rdata[i];
	}

	delete[] rdata;

	} else if (MessageHeader->Precision == 2) {//double precision
		int nread = 0;
	    char* Msg = rbuf;
	    
	    int nleft = dsize*sizeof(double)+16;
	    
	    while (nleft > 0) {
	    	if (nleft <= MAX_UDP_DATAGRAM) {
	    		recvfrom(sockfd, Msg, nleft, 0, &otheraddr.addr, &AddLength);
	    	    nleft =0;
	    	}
	    	else {
	    		recvfrom(sockfd, Msg, MAX_UDP_DATAGRAM, 0, &otheraddr.addr, &AddLength);
	    		Msg += MAX_UDP_DATAGRAM;
	    		nleft -= MAX_UDP_DATAGRAM;
	    	}
	    }
	    
	    ntohDataheader(rbuf, MessageHeader);
        
	    memcpy (rd, rbuf+16, dsize*sizeof(double));	
	    
	}

	return 0;
}


// Function to set up connection 
int setupconnection(int port, SOCKET* ClientSocket, int flag, char* machineInetAddr, int protocol)
{
	// Inputs:
	// port: port number 
	// ClienSocket: socket ID
	// flag: 
	//		 1 - Integration module
	//       2 - Substructure module
	// machineInetAddr: IP address of the machine
	// protocol:
	//       1 - TCP/IP
	//       2 - UDP 

	// variables for network communication 
	WSADATA wsaData;
    SOCKET ListenSocket = INVALID_SOCKET;			// listen socket
                    
    int iResult;
	
	if (flag == 1) {                                // Client (integration module)
		// set remote address
	    memset((char *) &otheraddr, 0, sizeof(otheraddr));
	    otheraddr.addr_in.sin_family = AF_INET;
	    otheraddr.addr_in.sin_port = htons(port);
	    otheraddr.addr_in.sin_addr.S_un.S_addr = inet_addr(machineInetAddr);
	    
	    // set local address
	    memset((char *) &myaddr, 0, sizeof(myaddr));
	    myaddr.addr_in.sin_family = AF_INET;
	    myaddr.addr_in.sin_port = htons(0);
	    myaddr.addr_in.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	    
		printf("before WSAStartup: %d\n", *ClientSocket);

	    // Initialize Winsock
	    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	    if (iResult != 0) {
	    	printf("WSAStartup failed with error: %d\n", iResult);
	    	return -1;
	    }
	    
	    // open a socket
		if (protocol == 1) {
		printf("before socket: %d\n", *ClientSocket);
			if ((*ClientSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				fprintf(stderr, "setupconnection() - could not open socket\n");
	    	    WSACleanup();
	    	    return -2;
			}
		} else {
			if ((*ClientSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				fprintf(stderr, "setupconnection() - could not open socket\n");
	    	    WSACleanup();
	    	    return -3;
			}
		}
	   
		// bind local address to it
				printf("before bind: %d\n", *ClientSocket);

		if (bind(*ClientSocket, &myaddr.addr, sizeof(myaddr.addr)) < 0) {
	    	fprintf(stderr,"setupconnection() - could not bind local address\n");
	    	closesocket(*ClientSocket);
	    	WSACleanup();
	    	return -4;
		}
	       

		if (protocol == 1) {
					printf("before ClientSocket: %d\n", *ClientSocket);

			// try to connect to socket with remote address.
			if (connect(*ClientSocket, &otheraddr.addr, sizeof(otheraddr.addr))< 0) {
				fprintf(stderr, "setupconnection() - could not connect\n");
				return *ClientSocket;
			}
				    return 0;

		} else {
            
			AddLength = sizeof(myaddr.addr);
			// send a message to address
		    char data = 'a';
		    sendto(*ClientSocket, &data, 1, 0, &otheraddr.addr, AddLength);
		    
            // receive a message from other
            recvfrom(*ClientSocket, &data, 1, 0, &otheraddr.addr, &AddLength);    
		    if (data != 'b') {
				fprintf(stderr, "setupconnection() - could not connect\n");
				return -1;
			}
		}

	    return 0;

	} else if (flag == 2) {                         // server (substructure module)
		// Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
           if (iResult != 0) {
               printf("WSAStartup failed with error: %d\n", iResult);
               return 1;
           }
	       
	       // set up my_Addr.addr_in with address given by port and internet address of
	       // machine on which the process that uses this routine is running.
           
	       memset((char *) &myaddr, 0, sizeof(myaddr));
	       myaddr.addr_in.sin_family = AF_INET;
	       myaddr.addr_in.sin_port = htons(port);
	       myaddr.addr_in.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	       
	       // setup connection
	       // wait for other process to contact me & set up connection	
	       // open a socket	
		   if (protocol == 1) {
			   if ((ListenSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {	
	       	   fprintf(stderr, "setupconnection() - could not open socket\n");
	       	   return -1;	
			   }
		   } else {
			   if ((ListenSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {	
	       	   fprintf(stderr, "setupconnection() - could not open socket\n");
	       	   return -1;	
			   }
		   }
	       
	       // bind local address to it
	       if (bind(ListenSocket, &myaddr.addr, sizeof(myaddr.addr)) < 0) {
	       	fprintf(stderr, "setupconnection() - could not bind local address\n");
	          return -1;
	       }    

	       AddLength = sizeof(myaddr. addr);
	       
		   if (protocol == 1) {
			   listen(ListenSocket, 1);    
	           fprintf(stderr, "setupconnection() - waiting for connection\n");
	           *ClientSocket = accept(ListenSocket, &otheraddr.addr, &AddLength);
	           
	           if (*ClientSocket < 0) {
				   fprintf(stderr, "setupconnection() - could not accept connection\n");
	           	   return -1;
	           } else {
	           	   fprintf(stderr, "setupconnection() - accept connection\n");
				                  // No longer need server socket
               closesocket(ListenSocket);
                return 0;
	           }
	           
		   } else {
			   *ClientSocket = ListenSocket;
			   // handshake by receiving a character from client
		       char data;
		       recvfrom(*ClientSocket, &data, 1, 0, &otheraddr.addr, &AddLength); 
               if (data == 'a') {    
		       	fprintf(stderr, "setupconnection() - accept connection\n");
					           // then send a message back
		       data = 'b';
               sendto(*ClientSocket, &data, 1, 0, &otheraddr.addr, AddLength);        
			   return 0;
		       } 
			   else 
				   return -1;
		       

		   }
	
    }		
	 
}

// Function to define and intialize the data exchange format
void updatemessageheader (uint8_t version, uint8_t command, uint8_t testtype, uint8_t subtype, uint8_t precision, uint16_t numdofs)
{

	// create and update *MessageHeader 
	if (MessageHeader == NULL)
		MessageHeader = (struct messageheader*) malloc (sizeof(struct messageheader));
			
	MessageHeader->Version = version;
	MessageHeader->Command = command;
	MessageHeader->Test_type = testtype;
	MessageHeader->Sub_type = subtype;
	MessageHeader->Precision = precision;

	MessageHeader->datatype.disp = 0;   
	MessageHeader->datatype.vel = 0;
	MessageHeader->datatype.accel = 0;
	MessageHeader->datatype.force = 0;
	MessageHeader->datatype.stiff = 0;
	MessageHeader->datatype.mass = 0;
	MessageHeader->datatype.temper = 0;
	MessageHeader->datatype.resvd = 0;

	MessageHeader->Num_DOFs = numdofs;
	MessageHeader->Step_num = 1;   
	MessageHeader->Reserved = 0;
	MessageHeader->Time_stamp = 0;

}

// Function to define command value in exchange format
void updatecommand (uint8_t command)
{
	MessageHeader->Command = command;
	
}

// Function to update the step number in the message header
void updatenumstep (uint16_t step)
{
	MessageHeader->Step_num = step;
}

// Function to update the number of dofs in the message header
void updatenumdofs (uint16_t ndfs)
{
	MessageHeader->Num_DOFs = ndfs;
}

// Function to return the substructure type in the message header
uint8_t getsubtype (void)
{
	return MessageHeader->Sub_type;
}

// Function to update data type in the message header
void updatedatatype (int disp, int vel, int accel, int force, int stiff, int mass, int temp)
{
	if (disp == 1)
		MessageHeader->datatype.disp = 1;
	else
		MessageHeader->datatype.disp = 0;

	if (vel == 1)
		MessageHeader->datatype.vel = 1;
	else
		MessageHeader->datatype.vel = 0;
	
	if (accel == 1)
		MessageHeader->datatype.accel = 1;
	else
		MessageHeader->datatype.accel = 0;

	if (force == 1)
		MessageHeader->datatype.force = 1;
	else
		MessageHeader->datatype.force = 0;

	if (stiff == 1)
		MessageHeader->datatype.stiff = 1;
	else
		MessageHeader->datatype.stiff = 0;
	
	if (mass == 1)
		MessageHeader->datatype.mass = 1;
	else
		MessageHeader->datatype.mass = 0;

	if (temp == 1)
		MessageHeader->datatype.temper = 1;
	else
		MessageHeader->datatype.temper =  0;

}
		
// Function to sync the data exchange format on both sides
int initialization (SOCKET ClientSocket, int flag, int protocol)
{
	int iResult;
	if (flag == 1) {
		sendbuf = new char[16];
		sData = NULL;
		if (protocol == 1) 
			iResult = TCP_Send(sendbuf, 0, sData, ClientSocket);
		else
			iResult = UDP_Send(sendbuf, 0, sData, ClientSocket);

		//cleanup temporary memory
		sData = NULL;
		delete[] sendbuf;
		sendbuf = NULL;

	} else if (flag == 2) {
	    recvbuf = new char[16];
	    rData = NULL;
		if (protocol == 1)
			iResult = TCP_Recv (recvbuf, 0, rData, ClientSocket);
		else
			iResult = UDP_Recv (recvbuf, 0, rData, ClientSocket);

		//cleanup temporary memory
		rData = NULL;
		delete[] recvbuf;
		recvbuf = NULL;
		
	} 

	if (iResult < 0) {		
		fprintf(stderr, "Initialization() - failed\n");
		return -1;
	}

	return 0;
}

// Function to send command
uint8_t command(SOCKET ClientSocket, int flag, int protocol)
{

	int iResult;
	if (flag ==1) {
		sendbuf = new char [16];
		sData = NULL;
		if (protocol == 1)
			iResult = TCP_Send (sendbuf, 0, sData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Send (sendbuf, 0, sData, ClientSocket);
		else
			fprintf(stderr, "command() - wrong protocol\n");

		// cleanup temporary memory
		delete[] sendbuf;
		sendbuf =NULL;
		sData = NULL;

	} 
	else if (flag ==2 ) {
	    recvbuf = new char [16];
	    rData = NULL;	

		if (protocol == 1)
			iResult = TCP_Recv (recvbuf, 0, rData, ClientSocket);
		else if (protocol == 2)
			iResult = UDP_Recv (recvbuf, 0, rData, ClientSocket);
		else
			fprintf(stderr, "command() - wrong protocol\n");

		// cleanup temporary memory	
		delete[] recvbuf;
		recvbuf =NULL;
		rData = NULL;
	}

	if (iResult < 0) {
		fprintf(stderr, "RecvData() - failed\n");
		return 1;
	}

	return MessageHeader->Command;
	

}

// Function to receive data
int recvdata (SOCKET ClientSocket, double* response, int len, int protocol)
{
	int iResult;
	//nd = indicator (MessageHeader);
	rData = new double [len];
	if (MessageHeader->Precision == 1) {
		 recvbuf = new char[16+len*sizeof(float)];
		 if (protocol == 1)
			 iResult = TCP_Recv(recvbuf, len, rData, ClientSocket);
		 else if (protocol == 2)
			 iResult = UDP_Recv(recvbuf, len, rData, ClientSocket);
		 else
			 fprintf(stderr, "RecvData() - wrong protocol\n");

	} else if (MessageHeader->Precision == 2) {
		 recvbuf = new char[16+len*sizeof(double)];
		 if (protocol == 1)
			 iResult = TCP_Recv(recvbuf, len, rData, ClientSocket);
		 else if (protocol == 2)
			 iResult = UDP_Recv(recvbuf, len, rData, ClientSocket);
		 else
			 fprintf(stderr, "RecvData() - wrong protocol\n");
	} else {
		fprintf(stderr, "RecvData() - wrong precision\n");
	
	}
	
	
	if (iResult < 0) {
		fprintf(stderr, "RecvData() - failed\n");
		return -1;
	}

	for (int i = 0; i<len; i++) {
		response[i] = rData[i];
	}

	// cleanup temporary memory
	delete[] recvbuf;
	recvbuf = NULL;
	delete[] rData;
	rData =NULL;

	return 0;
}

// Function to get number of DOFs in the message header
uint16_t getnumdof (void)
{
	return MessageHeader->Num_DOFs;

}

// Function to get step number in the message header
uint16_t getnumstep (void)
{
	return MessageHeader->Step_num;
}

// Function to send data
int senddata (SOCKET ClientSocket, double* sdata, int len, int protocol)
{
	int iSendResult;
	//nd = indicator(MessageHeader);

	if (sData == NULL)
		sData = new double [len];

	for (int i = 0; i<len; i++) {
		sData[i] = sdata[i];
	}

	// send resisting force to Client
	if (MessageHeader->Precision == 1) {
		sendbuf = new char[16+len*sizeof(float)];
		if (protocol == 1)
			iSendResult = TCP_Send(sendbuf, len, sData, ClientSocket);
		else if (protocol == 2)
			iSendResult = UDP_Send(sendbuf, len, sData, ClientSocket);
		else
			 fprintf(stderr, "SendData() - wrong protocol\n");
	} else if (MessageHeader->Precision == 2) {
		 sendbuf = new char[16+len*sizeof(double)];
		 if (protocol == 1)
			 iSendResult = TCP_Send(sendbuf, len, sData, ClientSocket);
		 else if (protocol == 2)
			 iSendResult = UDP_Send(sendbuf, len, sData, ClientSocket);
		 else
			 fprintf(stderr, "SendData() - wrong protocol\n");
	} else {
		fprintf(stderr, "SendData() - wrong precision\n");
	}

	if (iSendResult < 0) {
		fprintf(stderr, "SendData() - failed\n");
		return -1;
	}

	// cleanup temporary memory
	delete[] sendbuf;			 
	sendbuf = NULL;
	delete[] sData;
	sData = NULL;

	return 0;
}


// Function to shut down connection and close socket
int close(SOCKET* ClientSocket)
{

    // Shutdown the network connection 
	int iResult = shutdown(*ClientSocket, SD_SEND);
	 
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(*ClientSocket);
        WSACleanup();
        return -1;
	}
	else {
    	closesocket(*ClientSocket);
    	WSACleanup();
    }

	delete sData;
//	delete MessageHeader;

	return 0;
}

		
