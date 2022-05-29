** ****************************************************************** **
**    UT-SIM - University of Toronto Simulation Framework             **
																		
**                                                                    **
** Developed by:                                                      **
**   Xu Huang (xu.huang@mail.utoronto.ca)                             **
**   Oh-Sung Kwon (os.kwon@utoronto.ca)                               **
** Date: 2021-12-12                                                   **
** Revision: 2                                                        **
** Description:                                                       **
**             this version allows multiple interface elements and    **
**             works for vector2 and vector4                          **
** ****************************************************************** **


** main issues in this element are:
** option to define the IP address
		

      subroutine uel(rhs,amatrx,svars,energy,
     *               ndofel,nrhs,nsvars,props,nprops,
     *               coords,mcrd,nnode,u,du,v,a,
     *               jtype,time,dtime,kstep,kinc,jelem,
     *               params,ndload,jdltyp,adlmag,predef,npredf,
     *               lflags,mlvarx,ddlmag,mdload,pnewdt,
     *               jprops,njprop,period)
      
      include 'aba_param.inc'
	  
c========================================================================================================================================    
c   define function pointers pointing to the target functions in DataExchange.dll    
c========================================================================================================================================    

c Function to setup connection between S-FRAME and VecTor2
      INTERFACE
      INTEGER*4 FUNCTION connection(port, socket, flag, addr
     *             , protocol) BIND(C, NAME='setupconnection')
    
        INTEGER*4,value :: port
        INTEGER*4 socket
        INTEGER*4,value :: flag
        CHARACTER addr
        INTEGER*4,value :: protocol
        
      END FUNCTION connection
      END INTERFACE

c Function to create and initalize data exchange format defined in S-FRAME
      INTERFACE
      SUBROUTINE updates(version,command,testtype,subt,pre
     *                   ,ndof) BIND(C, NAME='updatemessageheader')

	    INTEGER*1,value :: version
        INTEGER*1,value :: command
        INTEGER*1,value :: testtype
        INTEGER*1,value :: subt
        INTEGER*1,value :: pre
        INTEGER*2,value :: ndof      
	
      END SUBROUTINE updates
      END INTERFACE

c Function to send data exchange format to VecTor2 for initialization    
      INTERFACE    
      INTEGER*4 FUNCTION initiate(socket, flag,
     *    	  protocol) BIND(C, NAME='initialization')
	 
	    INTEGER*4,value :: socket
	    INTEGER*4,value :: flag
	    INTEGER*4,value :: protocol
	    
      END FUNCTION initiate
      END INTERFACE

c Function to send command
      INTERFACE    
      INTEGER*1 FUNCTION command(socket, flag,
     *	  protocol) BIND(C, NAME='command')
	    
	    INTEGER*4,value :: socket
	    INTEGER*4,value :: flag
	    INTEGER*4,value :: protocol
	    
      END FUNCTION command
      END INTERFACE

c Function to send data
      INTERFACE
      INTEGER*4 FUNCTION senddata(socket, sdata, lens,
     *	  protocol) BIND(C, NAME='senddata')	
	
        INTEGER*4,value :: socket
	    REAL*8 sdata(lens)
	    INTEGER*4,value :: lens
	    INTEGER*4,value :: protocol
	
      END FUNCTION senddata
      END INTERFACE
    
c Function to receive data    
      INTERFACE
      INTEGER*4 FUNCTION recvdata(socket, response, lens,
     *	  protocol) BIND(C, NAME='recvdata')
	 
        INTEGER*4,value :: socket
	    REAL*8 response(lens)
	    INTEGER*4,value :: lens
	    INTEGER*4,value :: protocol
	    
      END FUNCTION recvdata
      END INTERFACE

c Function to return number of dofs of interface nodes    
       INTERFACE 
      INTEGER*2 FUNCTION getdofs() BIND(C, NAME='getnumdof')
      END FUNCTION getdofs
      END INTERFACE

c Function to return current number of steps 
      INTERFACE 
      INTEGER*2 FUNCTION getsteps() BIND(C, NAME='getnumstep')
      END FUNCTION getsteps
      END INTERFACE	
 
c Function to terminate communicaiton connection
      INTERFACE
	  INTEGER*4 FUNCTION terminate(socket) BIND(C, NAME='close')
	
        INTEGER*4 socket
	
      END FUNCTION terminate
      END INTERFACE
 
c Function to update command defined in data exchange format    
      INTERFACE
      SUBROUTINE upCommand(command) BIND(C, NAME='updatecommand')
    
	    INTEGER*1,value :: command
	
      END SUBROUTINE upCommand
      END INTERFACE	

c Function to update command defined in data exchange format    
      INTERFACE
	  SUBROUTINE upstep(step) BIND(C, NAME='updatenumstep')
		INTEGER*2,value :: step
	  END SUBROUTINE upstep
	  END INTERFACE
	  
c Function to return substructure type defined in data exchange format    
      INTERFACE
      INTEGER*1 FUNCTION getsubtype() BIND(C, NAME='getsubtype')
      END FUNCTION getsubtype
      END INTERFACE
    
c Function to update test type defined in data exchange format    
      INTERFACE
      SUBROUTINE updatatype(disp, vel, accel, force, stiff, mass,
     *	  temp) BIND(C, NAME='updatedatatype')
   
        INTEGER*4,value :: disp
        INTEGER*4,value :: vel
        INTEGER*4,value :: accel
        INTEGER*4,value :: force
        INTEGER*4,value :: stiff
        INTEGER*4,value :: mass
        INTEGER*4,value :: temp
      
      END SUBROUTINE updatatype
      END INTERFACE
    
      INTERFACE
      INTEGER*2 FUNCTION indicator() BIND(C, NAME='indicator')
      END FUNCTION indicator
      END INTERFACE    
    
c========================================================================================================================================    
c  end of loading dll !  
c========================================================================================================================================    

	  
c     operational code keys
      parameter (jNormalTimeIncr  = 1,
     *           jCurrentStiff    = 2,
     *           jCurrentMass     = 4,
     *           jCurrentResidual = 5,
     *           jInitialAccel    = 6)
      
c     flag indices
      parameter (iProcedure = 1,
     *           iOpCode    = 3,
     *           iStep      = 4)
      
c     procedure flags
      parameter (jStaticAutoIncr      =  1,
     *           jStaticDirectIncr    =  2,
     *           jDynImpHalfStepRes   = 11,
     *           jDynImpFixedTimeIncr = 12)
      
c     communication flag
      parameter (
     *           protocol_TCP  = 1,
     *			 protocol_UDP  = 2)	
	  
c     time indices
      parameter (iStepTime  = 1,
     *           iTotalTime = 2,
     *           nTime      = 2)
      
c     energy array indices
      parameter (iElPd     = 1,
     *           iElCd     = 2,
     *           iElIe     = 3,
     *           iElTs     = 4,
     *           iElDd     = 5,
     *           iElBv     = 6,
     *           iElDe     = 7,
     *           iElHe     = 8,
     *           iElKe     = 9,
     *           iElTh     = 10,
     *           iElDmd    = 11,
     *           iElDc     = 12,
     *           nElEnergy = 12)
      
c     parameter statement for communication
      parameter (RemoteTest_setTrialResponse   =  3,
     *           RemoteTest_getResponse        = 10,
     *           RemoteTest_getInitialStiff    = 12,
     *           RemoteTest_terminate          = 99)	 
	  
c     Define communication precision
      parameter (Single_Precision    = 1,
     *           Double_Precision    = 2)

c     Define communication protocol
      parameter (TCP_IP  = 1,
     *           UDP     = 2)	 
	  
c     Define test type
      parameter (Ramp_hold   = 1,	  
     *           Continuous  = 2,
     *           Real_time   = 3,
     *           Software    = 4)

c     Define substructure type
      parameter (OpenSees    = 1,
     *           Zeus_NL     = 2,
     *           ABAQUS      = 3,
     *           VecTor2     = 4,
     *           Cyrus       = 5,
     *           SFrame      = 6,
     *           NICON       = 7,
     *           VecTor4     = 8)							
	 

	  
	  
      dimension rhs(mlvarx,*),amatrx(ndofel,ndofel),
     *     svars(nsvars),energy(8),props(*),coords(mcrd,nnode),
     *     u(ndofel),du(mlvarx,*),v(ndofel),a(ndofel),time(2),
     *     params(*),jdltyp(mdload,*),adlmag(mdload,*),
     *     ddlmag(mdload,*),predef(2,npredf,nnode),lflags(*),
     *     jprops(*)

c     props (1) = port 
c     props (2) = 1-TCP/IP,
c                 2-UDP;
c     props (3) = Substructure type
c     props (4) = simulation type
c     props (5) = precision type
c     props (6) = 1 (read input stiffness from input)
c               = 0 (obtain stiffness from communication)												 
			
c     local variables
      integer*4 iresult
      integer*4 dataSize
      parameter (dataSize = 256)
      
      integer*4 numSockIDs
      parameter (numSockIDs = 80)
	  
	  integer*4 MaxNumDOFs               
	  parameter (MaxNumDOFs = 1000)       ! must be hardcoded by the user
									     ! this parameter cannot be less than 
										 ! maximum number of interface dofs among the connected substructure 
										 ! e.g. 
										 ! For two connected VT4 substructures, A and B,
										 ! assume the number of interface dofs of A is a
										 ! and the number of interface dofs of B is b
										 ! then MaxNumDOFs must be no less than max(a,b)
      
c     variables for data exchange format
      integer*1 version 
	  data version /1/
      integer*1 commands
      integer*1 Ttype 
      integer*1 Stype(numSockIDs)  
      integer*1 ComPre
      integer*2 NumDOFs 
      integer*1 Fstiff(numSockIDs)
	  
      integer*4 iflag
      integer*4 port
	  integer*4 protocol

      integer*4 socketIDs(numSockIDs)
      integer*4 socketID
      integer*1 cmd
	  integer*4 lens
	  integer*4 stiff_flags1(numSockIDs)        ! for multiple substructure modules  (02282019)
	  integer*4 stiff_flag1
	  integer*4 stiff_flags2(numSockIDs)        ! for multiple substructure modules  (02282019)
	  integer*4 stiff_flag2
	  integer*2 tmp_step

      integer*4 dof
	  parameter (dof = 6)
	  integer*4 dofs
      parameter (dofs = 36)	  
      
      real*8, DIMENSION(:), ALLOCATABLE :: sData
      real*8, DIMENSION(:), ALLOCATABLE :: rData
	  
      real*8    timePast
	  ! change for multiple substructures (02-28-2019)
	  real*8, save, DIMENSION(:,:,:), ALLOCATABLE :: stiffness       ! static stiffness ! for multiple substructure modules  (02282019)
	  real*8, save, DIMENSION(:,:), ALLOCATABLE :: stiffness2      ! dynamic stiffness  ! for multiple substructure modules  (02282019) not yet implemented
	  !real*8, save, DIMENSION(:,:)stiffness 
	  
	  ! array to store past state variables (06-28-2019)
	  ! this array is to overcome the limitation on the number of nsvars in ABAQUS
	  real*8, save, DIMENSION(:), ALLOCATABLE :: Past_disp
	  
	  
	  real*8, DIMENSION(5) :: b_cvet
	  real*8, DIMENSION(5) :: b_cvef
	  
	  real*8, DIMENSION(5,5) :: b_cmat

	  
	  
      save socketIDs
      save timePast
c     save stiff_matrix1
c	  save stiff_matrix2
      save stiff_flags1
	  save stiff_flags2
	  save Stype
	  save protocol
c	  save Past_disp
      save tmp_step
      save Fstiff
	  
	  
      data socketIDs /numSockIDs*0/
      data timePast /0.0/
	  data iflag /1/
      data stiff_flags1 /numSockIDs*0/
      data stiff_flags2 /numSockIDs*0/
      data Stype /numSockIDs*0/      
	  data protocol /0/
c	  data Past_disp
      data tmp_step/1/
      data Fstiff/numSockIDs*0/
	  
	
c      data stiff_matrix1/24937.60,	0.00,	-43640800.00,	-24937.60,	0.00,
c     *	  -43640800.00,	0.00,	3428570.00,	0.00,	0.00,	-3428570.00,
c     *    0.00,	-43640800.00,	0.00,	101829000000.00,	43640800.00,	0.00,
c     *	 50914300000.00,	-24937.60,	0.00,	43640800.00,	24937.60,	0.00,
c     *	 43640800.00,	0.00,	-3428570.00,	0.00,	0.00,	3428570.00,	0.00,
c     *	 -43640800.00,	0.00,	50914300000.00,	43640800.00,	0.00,	
c     *	 101829000000.00/
c	  data stiff_matrix2/dofs*0.0/
      
	  	  b_cmat = reshape((/1000, 1000, 1000, 1000000, 1000000, 
     *	                    1000, 1000, 1000, 1000000, 1000000, 
     *				 1000, 1000, 1000, 1000000, 1000000, 
     * 			        1000000, 1000000, 1000000, 1000000000, 1000000000, 
     *                  1000000, 1000000, 1000000, 1000000000, 1000000000/), (/5,5/))
	  
	  
	  data b_cvet /1,1,1,1000,1000/
	  
	  
	  data b_cvef /1000,1000,1000,1000000,1000000/
	  
c     extract socketID
c     (jtype = user-defined integer value n in element type Un)
      if (jtype .le. numSockIDs) then
         socketID = socketIDs(jtype)
		 stiff_flag1 = stiff_flags1(jtype)
		 stiff_flag2 = stiff_flags2(jtype)
		 
      else
         write(*,*) 'ERROR - Only ',numSockIDs,' genericClient_imp ',
     *              'elements supported: consider increasing ',
     *              'numSockIDs parameter in genericClient_imp.for'
         call xit
      endif
      
c     setup connection with SubSub element
      if (socketID .eq. 0) then
         
!c        check number of state variables first     !(06-28-2019)
!         if (nsvars .lt. 2*ndofel) then
!            write(*,*) 'ERROR - ',2*ndofel,' state variables are ',
!     *                 'required for genericClient element U',jtype
!            call xit
!         endif
         
c        now setup the connection
         port = props(1)
		 protocol = props(2)
c note '127.0.0.1' should be changed later

		write(*,*) 'connection'
         iresult = connection(port, socketID, iflag, 
     *                    '127.0.0.1', protocol)
		 		 
         if (iresult .ne. 0) then
            write(*,*) 'ERROR - failed to setup connection'
            call xit
         endif
		write(*,*) 'end of connection'
         socketIDs(jtype) = socketID

         commands = 0
		 Stype(jtype)   = props(3)
		 Ttype   = props(4)
		 ComPre  = props(5)
         Fstiff(jtype)  = props(6)
		 NumDOFs = ndofel

         call updates(version, commands, Ttype, Stype(jtype), 
     *                                  ComPre, NumDOFs)
         
         if (initiate(socketID, iflag, protocol) .ne. 0) THEN
            write(*,*) 'ERROR - initialization failed'
            call xit
	     endif
	  
	  endif
      
	  
	  
c     zero rhs vector and A matrix
      do i = 1, ndofel
         do j = 1, nrhs
            rhs(i,j) = 0.0
         enddo
      enddo
	  
	  if (.not.allocated(stiffness)) then
		allocate (stiffness(MaxNumDOFs,MaxNumDOFs,numSockIDs))
		stiffness = 0
	  endif
	  
	  if (.not.allocated(Past_disp)) then
		allocate (Past_disp(2*MaxNumDOFs*numSockIDs))
		Past_disp = 0
	  endif 
      
c     normal incrementation
      if (lflags(iOpCode) .eq. jNormalTimeIncr) then
c        static analysis
         if (lflags(iProcedure) .eq. jStaticAutoIncr .or.
     *       lflags(iProcedure) .eq. jStaticDirectIncr) then
            if (lflags(iStep) .ne. 0) then
               write(*,*) 'ERROR - Linear perturbation not supported ',
     *                    'in IntSub element U',jtype
c               call closeconnection(socketID, stat)
               iresult = terminate(socketID) 
               call xit
            else
			  
			   write(*,*) 'static analysis'
c              commit state

c				abaqus-vt2
				  if (Stype(jtype) .eq. 4) then 
					!write(*,*) 'abaqus-vt2 stiffness'
				    if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 0) then 
				     !write(*,*) 'receive stiffness'
				     call upCommand(RemoteTest_getInitialStiff)
				     call updatatype(0, 0, 0, 0, 1, 0, 0)
				     
				     cmd = command(socketID, iflag, protocol)
				     !write(*,*) 'protocol=', protocol
			         if (cmd .ne. RemoteTest_getInitialStiff) then
			         write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			         endif
				     
				     lens = ndofel*ndofel ! indicator()
				     !write(*,*) lens
				     allocate(rData(lens))
				     
				     if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			         write(*,*) 'ERROR - Failed to receive stiffness matrix'
			         endif
                    
				     
					 !if (.not.allocated(stiffness)) then
				     !    allocate (stiffness(ndofel,ndofel))
				     !endif
				     
                     k = 1
                     do i = 1, ndofel
                        do j = 1, ndofel
                           amatrx(i,j) = rData(k)*1000.
					  	 stiffness(i,j,jtype) = rData(k)*1000.
                           k = k + 1
                        enddo
                     enddo
				     
				     deallocate (rData)
				     
				     stiff_flag1 = 1
				     !write(*,*) 'end tangent stiffness'
				    stiff_flag2 = 1
				    endif 
				!write(*,*) 'done with abaqus-vt2 stiffness' 
				  endif
				
c	            end abaqus-vt2			
				
c				abaqus-vt4
				!write(*,*) 'abaqus-vt4: stiffness'
				  if (Stype(jtype) .eq. 8) then 
				  !write(*,*) 'abaqus-vt4: stiffness'
				  write(*,*) 'stiff_flags1', stiff_flags1(jtype) 
				    if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 0) then 
				     !write(*,*) 'receive stiffness'
				     call upCommand(RemoteTest_getInitialStiff)
				     call updatatype(0, 0, 0, 0, 1, 0, 0)
				     
				     cmd = command(socketID, iflag, protocol)
				     !write(*,*) 'protocol=', protocol
			         if (cmd .ne. RemoteTest_getInitialStiff) then
			         write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			         endif
				     
				     lens = ndofel*ndofel ! indicator()
				     !write(*,*) lens
				     allocate(rData(lens))
				     
				     if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			         write(*,*) 'ERROR - Failed to receive stiffness matrix'
			         endif
                    
				     !if (.not.allocated(stiffness)) then
				     !    allocate (stiffness(ndofel,ndofel))
				     !endif
				     
					 write(*,*) 'Recv initial stiffness'
                     k = 1
                     do i = 1, ndofel
                        do j = 1, ndofel
                           amatrx(i,j) = rData(k)
						   stiffness(i,j,jtype) = rData(k)
						   !write(*,*) rData(k)
                           k = k + 1
                        enddo
                     enddo
				     
				     deallocate (rData)
				     
					! unit conversion 
					
					 
					 do i = 1, nnode 
						do j  = 1, nnode
							do ii = 1,5
								do jj = 1,5
									amatrx(ii+5*(i-1),jj+5*(j-1)) = b_cmat(ii,jj)* 
     *                                  amatrx(ii+5*(i-1),jj+5*(j-1)) 
									stiffness(ii+5*(i-1),jj+5*(j-1),jtype) = b_cmat(ii,jj)*
     *                                  stiffness(ii+5*(i-1),jj+5*(j-1),jtype)
								!write(*,*) 'i=', ii+5*(i-1), ' j=', jj+5*(j-1)
								!write(*,*) stiffness(ii+5*(i-1),jj+5*(j-1))
								enddo
							enddo
						enddo
					 enddo
					 
				     stiff_flag1 = 1
					 stiff_flags1(jtype)=stiff_flag1
					 
				     !write(*,*) 'end tangent stiffness'
				    stiff_flag2 = 1
					stiff_flags2(jtype)=stiff_flag2
				    endif 
				!write(*,*) 'done with abaqus-vt4 stiffness'
				  endif
				 
c	            end abaqus-vt4		
				
				

               if (time(iTotalTime) .gt. timePast) then
			   timePast = time(iTotalTime)
			   call upstep(tmp_step)
			   
			   tmp_step = tmp_step + 1
			   
               endif
               
c              send trial response to substructure module
               call upCommand(RemoteTest_setTrialResponse)
			   call updatatype(1,0,0,0,0,0,0)
			   
			   cmd = command(socketID, iflag, protocol)
			   !write(*,*) 'command:', socketID, iflag, protocol
			   if (cmd .ne. RemoteTest_setTrialResponse) then
			      write(*,*) 'ERROR - Failed to send RemoteTest_setTrialResponse'
			   endif
			   
			   lens = ndofel !indicator()
			   !write(*,*) 'disp lens=', lens 
			   ALLOCATE (sData(lens))
			   
			   
			   !do i = 1, ndofel
               !   if (Stype(jtype) .eq. VECTOR4) then
				!		sData(i) = 
				!  else 
				!  sData(i) = u(i)
				!  endif
               !enddo
			   if (Stype(jtype) .eq. VecTor4) then
			   write(*,*) 'vector4 send disp'
				do i = 1, nnode
					do ii = 1,5
						sData(ii+5*(i-1)) = b_cvet(ii) * u(ii+5*(i-1))
						write(*,*) sData(ii+5*(i-1))
					enddo	
				enddo
			   else 
				do i = 1, ndofel
					sData(i) = u(i)
					write(*,*) sData(i)
				enddo
			   endif
			   
			   if (senddata(socketID,sData,lens, protocol) .ne. 0) then
				   write(*,*) 'ERROR - Failed to send trial response'
			   endif
			   
			   DEALLOCATE (sData)
               
c              get measured displacement and resisting forces
               call upCommand(RemoteTest_getResponse)
			   if (Stype(jtype) .eq. VecTor2 .or. Stype(jtype) .eq. VECTOR4) then
			   !write(*,*) 'VecTor programs' 
			   call updatatype(0, 0, 0, 1, 0, 0, 0)
			   lens = ndofel
			   else
			   !write(*,*) 'Stype(jtype)<>4' 
			   
			   call updatatype(1, 0, 0, 1, 0, 0, 0)
			   lens = 2*ndofel
			   endif
			   
			   cmd = command(socketID, iflag, protocol)
			   
			   if (cmd .ne. RemoteTest_getResponse) then
			      write(*,*) 'ERROR - Failed to send RemoteTest_getResponse'
			   endif
			   
			   !lens = ndofel !indicator()
			   !write(*,*) 'force lens=', lens
			   
			   allocate (rData(lens))
			   
			   if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			       write(*,*) 'ERROR - Failed to receive measured forces'
			   endif
               
               write(*,*) 'Recv forces'
			   if (Stype(jtype) .eq. VecTor4) then 
					do i = 1, nnode
						do ii = 1,5
							rhs(ii+5*(i-1),1) = -b_cvef(ii) * rData(ii+5*(i-1))
							write(*,*) rData(ii+5*(i-1))
						enddo
					enddo
				
			   else 
					do i = 1, ndofel
						if (Stype(jtype) .eq. Vector2) then
							rhs(i,1) = -rData(i)*1000.
							
						else 
							rhs(i,1) = -rData(ndofel+i)
							write(*,*) rhs(i,1)
						endif
					enddo
			   
			   endif
			   
			   
               
			   deallocate (rData)
			   
c              get tangent stiffness matrix
               if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 0) then
			   write(*,*) 'tangent stiffness'
			       call upCommand(RemoteTest_getInitialStiff)
				   call updatatype(0, 0, 0, 0, 1, 0, 0)
				   
				   cmd = command(socketID, iflag, protocol)
				   
			       if (cmd .ne. RemoteTest_getInitialStiff) then
			       write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			       endif
				   
				   lens = ndofel*ndofel !indicator()
				   allocate(rData(lens))
				   
				   if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			       write(*,*) 'ERROR - Failed to receive stiffness matrix'
			       endif

				   !if (.not.allocated(stiffness)) then
				   !    allocate (stiffness(ndofel,ndofel))
				   !endif
				   
                   k = 1
                   do i = 1, ndofel
                      do j = 1, ndofel
                         amatrx(i,j) = rData(k)
						 stiffness(i,j,jtype) = rData(k)
                         k = k + 1
                      enddo
                   enddo
				   
				   deallocate (rData)
				   
				   stiff_flag1 = 1
				   stiff_flags1(jtype)=stiff_flag1
				   write(*,*) 'end tangent stiffness'
				   
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag1 .eq. 0) then
			           !write(*,*) 'read from input file'
c      open (unit=99, file='C:\Users\Xu\Desktop\Abaqus\try\test5_implicit
c     *\Kinit.txt', status='old', action='read')
c                allocate (stiff_matrix1(ndofel,ndofel))
				!if (.not.allocated(stiffness)) then
				!    allocate(stiffness(ndofel,ndofel))
				!endif
				k = 1
				do i = 1, ndofel
					do j = 1, ndofel
						stiffness(i,j,jtype) = props(6+k)
						!write(*,*) 6+k, stiffness(i,j)
						k=k+1
					enddo
				enddo
				
				
				
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      amatrx(i,j) = stiffness(i,j,jtype)
					  
					  k=k+1  
                   enddo
                enddo				
			   
			   stiff_flag1 = 1
			   stiff_flags1(jtype)=stiff_flag1
			   
c			   close(unit=99)
			   
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag1 .eq. 1) then
                k = 1
				
				if (.not.allocated(stiffness)) then
				    write(*,*) 'ERROR - Fail to save stiffness matrix'
					call xit
				endif
				
                do i = 1, ndofel
                   do j = 1, ndofel
				      amatrx(i,j) = stiffness(i,j,jtype)
					  k=k+1  
                   enddo
                enddo		
	
			   else if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 1) then

			       if (.not. allocated(stiffness)) then
				      write(*,*) 'ERROR - Fail to save stiffness matrix'
					  call xit
				   endif
				   
                   k = 1
                   do i = 1, ndofel
                      do j = 1, ndofel
                         amatrx(i,j) = stiffness(i,j,jtype)
                         k = k + 1
                      enddo
                   enddo
				   
			   endif
			   
			   
            endif
         
c        dynamic analysis
         else if (lflags(iProcedure) .eq. jDynImpHalfStepRes .or.
     *            lflags(iProcedure) .eq. jDynImpFixedTimeIncr) then
	                 
			write(*,*) 'dynamic analysis'

            alpha = params(1)
            beta  = params(2)
            gamma = params(3)
            			
            dadu = 1.0/(beta*dtime**2)
            dvdu = gamma/(beta*dtime)
            
c           commit state


c				abaqus-vt2
				  if (Stype(jtype) .eq. 4) then 
	   if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 0 .and. stiff_flag2 .eq. 0) then 
				     write(*,*) 'receive stiffness'
				     call upCommand(RemoteTest_getInitialStiff)
				     call updatatype(0, 0, 0, 0, 1, 0, 0)
				     
				     cmd = command(socketID, iflag, protocol)
				     write(*,*) 'protocol=', protocol
			         if (cmd .ne. RemoteTest_getInitialStiff) then
			         write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			         endif
				     
				     lens = ndofel*ndofel ! indicator()
				     write(*,*) lens
				     allocate(rData(lens))
				     
				     if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			         write(*,*) 'ERROR - Failed to receive stiffness matrix'
			         endif
                    
				     if (.not. allocated(stiffness2)) then
				    allocate(stiffness2(ndofel,ndofel))
				endif
				
				
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = (1.0+alpha)*rData(k)*1000.
					  stiffness2(i,j) = (1.0+alpha)*rData(k)*1000.
                      k = k + 1
                   enddo
                enddo
    
				stiff_flag2 = 1
					 stiff_flags2(jtype)=stiff_flag2
				     write(*,*) 'end tangent stiffness'
				     
					 deallocate(rData)
					 
				    endif 
				
				  endif
				
c	            end abaqus-vt2	



c            write(*,*) 'timepast', timepast
            if (time(iTotalTime) .gt. timePast) then
               timePast = time(iTotalTime)
			   call upstep(tmp_step)
			   tmp_step = tmp_step + 1
            endif
            
c           send trial response to experimental element
            call upCommand(RemoteTest_setTrialResponse)
			call updatatype(1,0,0,0,0,0,0)
			
			cmd = command(socketID, iflag,protocol)
			
			if (cmd .ne. RemoteTest_setTrialResponse) then
			   write(*,*) 'ERROR - Failed to send RemoteTest_setTrialResponse'
			endif
			
			lens = ndofel
			ALLOCATE (sData(lens))
			
			
			do i = 1, ndofel
               sData(i) = u(i)
            enddo

			if (senddata(socketID,sData,lens,protocol) .ne. 0) then
			   write(*,*) 'ERROR - Failed to send trial response'
			endif
			
			DEALLOCATE (sData)
			
c           get measured resisting forces
            call upCommand(RemoteTest_getResponse)
			if (Stype(jtype) .eq. VecTor2) then
			call updatatype(0, 0, 0, 1, 0, 0, 0)
			lens = ndofel
			else
			call updatatype(1, 0, 0, 1, 0, 0, 0)
			lens = ndofel*2
			endif
			
			cmd = command(socketID, iflag, protocol)
			
			if (cmd .ne. RemoteTest_getResponse) then
			   write(*,*) 'ERROR - Failed to send RemoteTest_getResponse'
			endif
			
			!lens = ndofel
			allocate (rData(lens))
			
			if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			    write(*,*) 'ERROR - Failed to receive measured forces'
			endif
            
! avoid the use of svars (06-28-2019)
!            do i = 1, ndofel
!			   if (Stype(jtype) .eq. VECTOR2) then
!               rhs(i,1) = (-(1.0+alpha)*rData(i) + alpha*svars(i))*1000.
!               svars(ndofel+i) = svars(i)
!               svars(i) = rData(i)
!			   else 
!               rhs(i,1) = -(1.0+alpha)*rData(ndofel+i) + alpha*svars(i)
!               svars(ndofel+i) = svars(i)
!               svars(i) = rData(i)			   
!			   endif
!            enddo

! (06-28-2019)
            do i = 1, ndofel
			   if (Stype(jtype) .eq. Vector2) then
               rhs(i,1) = (-(1.0+alpha)*rData(i) + alpha*Past_disp(i))*1000.
               Past_disp(ndofel+i) = Past_disp(i)
			   Past_disp(i) = rData(i)
			   else 
               rhs(i,1) = -(1.0+alpha)*rData(ndofel+i) + alpha*Past_disp(i)
               Past_disp(ndofel+i) = Past_disp(i)
			   Past_disp(i) = rData(i)			   
			   endif
            enddo

            
			deallocate (rData)

c           get tangent stiffness matrix
            if (Fstiff(jtype) .eq. 0 .and. stiff_flag2 .eq. 0) then
			    call upCommand(RemoteTest_getInitialStiff)
			    call updatatype(0, 0, 0, 0, 1, 0, 0)
			 
			    cmd = command(socketID, iflag, protocol)
			 
			    if (cmd .ne. RemoteTest_getInitialStiff) then
			    write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			    endif
			 
			    lens = ndofel*ndofel
			    allocate(rData(lens))
			 
			    if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			    write(*,*) 'ERROR - Failed to receive stiffness matrix'
			    endif

				if (.not. allocated(stiffness2)) then
				    allocate(stiffness2(ndofel,ndofel))
				endif
				
				
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = (1.0+alpha)*rData(k)
					  stiffness2(i,j) = (1.0+alpha)*rData(k)
                      k = k + 1
                   enddo
                enddo
    
				stiff_flag2 = 1
				stiff_flags2(jtype)=stiff_flag2
			    deallocate (rData)
			 
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag2 .eq. 0) then
c			    write(*,*) 'read from input file1'
				
				if (.not. allocated(stiffness2)) then
				   allocate (stiffness2(ndofel,ndofel))
				endif
				
				if (.not. allocated(stiffness) .and. stiff_flag1 .eq. 1) then
				   write(*,*) 'ERROR - fail to save stiffness matrix - 1'
				   call xit
				else
				   !allocate (stiffness(ndofel,ndofel))
c      open (unit=99, file='C:\Users\Xu\Desktop\Abaqus\try\test5_implicit
c     *\Kinit.txt', status='old', action='read')
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      stiffness(i,j,jtype) = props(6+k)
                      k = k + 1
                   enddo
                enddo	
                 
c   				do i = 1, ndofel
c	    			read (99, *), stiffness(i,:,jtype)
c		    		enddo		

c                    close(unit=99)					
				endif
						
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      amatrx(i,j) = (1.0+alpha)*stiffness(i,j,jtype)
					  stiffness2(i,j) = (1.0+alpha)*stiffness(i,j,jtype)
                      k = k + 1
                   enddo
                enddo				
			   
			   stiff_flag2 = 1
				stiff_flags2(jtype)=stiff_flag2
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag2 .eq. 1) then 
			    write(*,*) 'read from input file2'
                
				if (.not. allocated(stiffness2)) then
				    write(*,*) 'ERROR - fail to save stiffness matrix - 2'
					call xit
				endif
				
                do i = 1, ndofel
                   do j = 1, ndofel			
						write(*,*) stiffness(i,j,jtype)
                   enddo
                enddo	
				
				k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      amatrx(i,j) = (1.0+alpha)*stiffness2(i,j)
                      k = k + 1
                   enddo
                enddo				   
               

			   else if (Fstiff(jtype) .eq. 0 .and. stiff_flag2 .eq. 1) then 
c                     do noting		
				write(*,*) 'do nothing'
                if (.not. allocated(stiffness2)) then
				    write(*,*) 'ERROR - fail to save stiffness matrix'
				endif
			
            	  k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = stiffness2(i,j)
                      k = k + 1
                   enddo
                enddo

				endif

            endif
      
c     stiffness matrix
      else if (lflags(iOpCode) .eq. jCurrentStiff) then
c        get tangent stiffness matrix
         write(*,*) 'stiffness matrix'

               if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 0) then
			       call upCommand(RemoteTest_getInitialStiff)
				   call updatatype(0, 0, 0, 0, 1, 0, 0)
				   
				   cmd = command(socketID, iflag, protocol)
				   
			       if (cmd .ne. RemoteTest_getInitialStiff) then
			       write(*,*) 'ERROR - Failed to send RemoteTest_getInitialStiff'
			       endif
				   
				   lens = ndofel*ndofel
				   allocate(rData(lens))
				   
				   if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			       write(*,*) 'ERROR - Failed to receive stiffness matrix'
			       endif
                   !if (.not. allocated(stiffness)) then
				   !    allocate(stiffness(ndofel,ndofel))
				   !endif
				   
                   k = 1
                   do i = 1, ndofel
                      do j = 1, ndofel
                         amatrx(i,j) = rData(k)
						 stiffness(i,j,jtype) = rData(k)
                         k = k + 1
                      enddo
                   enddo
				   
				   
				   stiff_flag1 = 1
				   stiff_flags1(jtype)=stiff_flag1
				   
				   deallocate (rData)
				   
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag1 .eq. 0) then
c			   write(*,*) 'read from input file'

c      open (unit=99, file='C:\Users\Xu\Desktop\Abaqus\try\test5_implicit
c     *\Kinit.txt', status='old', action='read')
c                allocate (stiff_matrix1(ndofel,ndofel))
				!if (.not. allocated(stiffness)) then
				!		allocate (stiffness(ndofel,ndofel))
	            !endif
				k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      stiffness(i,j,jtype) = props(6+k)
                      k = k + 1
				   enddo
                enddo	
				
c				do i = 1, ndofel
c				read (99, *), stiffness(i,:,jtype)
c				enddo
				
				k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
				      amatrx(i,j) = stiffness(i,j,jtype)
                      k = k + 1
				   enddo
                enddo				

				stiff_flag1 = 1
				stiff_flags1(jtype)=stiff_flag1
c			   close(unit=99)
			   
			   else if (Fstiff(jtype) .eq. 1 .and. stiff_flag1 .eq. 1) then
               if (.not. allocated(stiffness)) then
			       write(*,*) 'ERROR - fail to save stiffness matrix'
				   call xit
			   endif
			   
c			   k = 1
c                do i = 1, ndofel
c                   do j = 1, ndofel
c				      amatrx(i,j) = stiffness(k)
c                      k = k + 1
c				   enddo
c                enddo	               
			   
				
			   else if (Fstiff(jtype) .eq. 0 .and. stiff_flag1 .eq. 1) then
c                    do noting 
                   if (.not. allocated(stiffness)) then
				       write(*,*) 'ERROR - fail to save stiffness matrix'
					   call xit
				   endif
				   
                   k = 1
                   do i = 1, ndofel
                      do j = 1, ndofel
                         amatrx(i,j) = stiffness(i,j,jtype)
                         k = k + 1
                      enddo
                   enddo

				   endif
      
c     mass matrix
      else if (lflags(iOpCode) .eq. jCurrentMass) then
c        get mass matrix

			   write(*,*) 'Warnings - Mass matrix is not supported ',
     *                    'in IntSub element',jtype
c               call closeconnection(socketID, stat)
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = 0.0
                      if (i .eq. j .and. amatrx(i,j) .le. 0.0) then
                         amatrx(i,j) = 1E-12
                      endif
                      k = k + 1
                   enddo
                enddo
			         
c     residual calculation
      else if (lflags(iOpCode) .eq. jCurrentResidual) then
         alpha = params(1)
                  write(*,*) 'residual calculation'

c        send trial response to experimental element
            call upCommand(RemoteTest_setTrialResponse)
	        call updatatype(1,0,0,0,0,0,0)
			
		    cmd = command(socketID, iflag,protocol)
			
			if (cmd .ne. RemoteTest_setTrialResponse) then
			   write(*,*) 'ERROR - Failed to send RemoteTest_setTrialResponse'
		    endif
			
		    lens = ndofel
		    ALLOCATE (sData(lens))
			
			do i = 1, ndofel
               sData(i) = u(i)
            enddo
			
			if (senddata(socketID,sData,lens, protocol) .ne. 0) then
			   write(*,*) 'ERROR - Failed to send trial response'
		    endif
			
		    DEALLOCATE (sData)
			
			
c           get measured resisting forces
            call upCommand(RemoteTest_getResponse)
			call updatatype(0, 0, 0, 1, 0, 0, 0)
			
			cmd = command(socketID, iflag, protocol)
			
			if (cmd .ne. RemoteTest_getResponse) then
			   write(*,*) 'ERROR - Failed to send RemoteTest_getResponse'
			endif
			
			lens = ndofel
			allocate (rData(lens))
			
			if (recvdata(socketID, rData, lens, protocol) .ne. 0) then
			    write(*,*) 'ERROR - Failed to receive measured forces'
			endif
     
! to avoid the use of svars	 (06-28-2019)
!            do i = 1, ndofel
!               rhs(i,1) = (-(1.0+alpha)*rData(i)
!     *                 + 0.5*alpha*(svars(i)+svars(ndofel+i)))
!            enddo
 
! (06-28-2019)
            do i = 1, ndofel
               rhs(i,1) = (-(1.0+alpha)*rData(i)
     *                 + 0.5*alpha*(Past_disp(i)+Past_disp(ndofel+i)))
            enddo			
			
		    deallocate (rData)

      
c     initial acceleration calculation
      else if (lflags(iOpCode) .eq. jInitialAccel) then

                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = 0.0
                      if (i .eq. j .and. amatrx(i,j) .le. 0.0) then
                         amatrx(i,j) = 1E-12
                      endif
                      k = k + 1
                   enddo
                enddo

      endif
      
      return
      end
