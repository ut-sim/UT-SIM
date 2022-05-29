** ****************************************************************** **
**    UT-SIM - University of Toronto Simulation Framework             **
																		
**                                                                    **
** Developed by:                                                      **
**   Xu Huang (xu.huang@mail.utoronto.ca)                             **
**   Oh-Sung Kwon (os.kwon@utoronto.ca)                               **
** Date: 2021-12-12                                                   **
** Revision: 2                                                        **
** Description:                                                       **
**             this version includes both component- and system-level **
**             decomposition                                          **
** ****************************************************************** **

** main issues in this element are:
** Ttype should be defiend in the inp file



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

c Function to setup connection between Integration/coordinate and ABAQUS
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
     *           Software    = 4,
     *           Software2   = 5)

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
c     props (3) = ficticious stiffness 

			
c     local variables
      integer*4 iresult
      integer*4 dataSize
      parameter (dataSize = 256)
      
      integer*4 numSockIDs
      parameter (numSockIDs = 32)
      
c     variables for data exchange format
      integer*1 version 
	  data version /1/
      integer*1 commands
      integer*1 Ttype  
      integer*1 Stype  
      integer*1 ComPre 
      integer*2 NumDOFs 
	  
      integer*4 iflag
      integer*4 port
	  integer*4 protocol
	  integer*4 flag_dyna
	  integer*4 flag_iteration1
	  integer*4 flag_iteration2
      integer*4 socketIDs(numSockIDs)
      integer*4 socketID
      integer*1 cmd
	  integer*4 lens
	  integer*4 stiff_flag1
	  integer*4 stiff_flag2
      integer*4 dof
	  parameter (dof = 6)
	  integer*4 dofs
      parameter (dofs = 36)	  
      
      real*8, save, DIMENSION(:), ALLOCATABLE :: sData
      real*8, save, DIMENSION(:), ALLOCATABLE :: rData
	  
      real*8    timePast
	  
	  real*8 pr(ndofel)
	  
	  real*8, save, DIMENSION(:,:), ALLOCATABLE :: stiffness
	  real*8, save, DIMENSION(:,:), ALLOCATABLE :: stiffness2
	  
	  integer*4 massFlag
	  real*8, save, DIMENSION(:), ALLOCATABLE :: Mm
	  
	  
      save socketIDs
      save timePast
      save stiff_flag1
	  save stiff_flag2
	  save Ttype                         ! modify later
	  save flag_dyna
	  save flag_iteration1
	  save flag_iteration2
	  save massFlag
      data socketIDs /numSockIDs*0/
      data timePast /0.0/
	  data iflag /2/
      data stiff_flag1 /0/
      data stiff_flag2 /0/
      data flag_dyna /0/
	  data flag_iteration1 /0/             ! flag for iterations
	  data flag_iteration2 /0/   
	  data massFlag /0/
	  port = props(1)
	  protocol = props(2)
	  
	  
c     extract socketID
c     (jtype = user-defined integer value n in element type Un)
      if (jtype .le. numSockIDs) then
         socketID = socketIDs(jtype)
      else
         write(*,*) 'ERROR - Only ',numSockIDs,' genericClient_imp ',
     *              'elements supported: consider increasing ',
     *              'numSockIDs parameter in genericClient_imp.for'
         call xit
      endif
      
c     setup connection with SimAppElemServer
      if (socketID .eq. 0) then
         write(*,*) 'Waiting for connection'
c        now setup the connection
         if (nsvars .lt. 2*(ndofel)) then
            write(*,*) 'ERROR - ',2*ndofel,' state variables are ',
     *                 'required for adapter element U',jtype
            call xit
         endif
         
         iresult = connection(port, socketID, iflag, 
     *                    '127.0.0.1', protocol)
		 		 
         if (iresult .ne. 0) then
            write(*,*) 'ERROR - failed to setup connection'
            call xit
         endif
		 
         socketIDs(jtype) = socketID

         commands = 0
		 Stype   = 0
c   ! modify later!!!!!		 Ttype = 5 for dynamic integration
		 Ttype   = 0
		 ComPre  = 2
		 NumDOFs = ndofel

         call updates(version, commands, Ttype, Stype, 
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
      
c     normal incrementation
      if (lflags(iOpCode) .eq. jNormalTimeIncr) then
c        static analysis
         if (lflags(iProcedure) .eq. jStaticAutoIncr .or.
     *       lflags(iProcedure) .eq. jStaticDirectIncr) then
            if (lflags(iStep) .ne. 0) then
               write(*,*) 'ERROR - Linear perturbation not supported ',
     *                    'in genericClient element U',jtype
c               call closeconnection(socketID, stat)
               iresult = terminate(socketID) 
               call xit
            else
				    
					if (Ttype .eq. 5) then
					! this is static analysis for dynamic substructure modules
					!write(*,*) 'static analysis'
					!write(*,*) KSTEP
					
						if (time(iTotalTime) .ge. timePast) then
							! recv equivalent interface force  
						
							cmd = command(socketID, iflag, protocol)
							write(*,*) 'RecvCMD', cmd
							SELECT CASE(cmd)
								CASE (3)
								
								lens = ndofel
								if (.not. allocated(rData)) then
									allocate(rData(lens))
								endif 
								
								if (recvdata(socketID,rData,lens,protocol) .ne. 0) then
									write(*,*) 'ERROR - failed to recv trial response'
								endif
								write(*,*) 'RecvData', lens
								
								CASE (10)
								
								lens = ndofel
								
								if (.not. allocated(sData)) then
									allocate(sData(lens))
								endif 
								
								do i = 1, lens
									sData(i) = u(i)   
								enddo
								
								if (senddata(socketID,sData,lens,protocol) .ne. 0) then
									write(*,*) 'ERROR - failed to send restoring forces'
								endif					  
								write(*,*) 'SendData', lens
								
								CASE (99)
									iresult = terminate(socketID)
									call xit
								
								CASE DEFAULT
								
								write(*,*) 'ERROR - incorrect command received ', cmd
								call xit
								
							END SELECT
	   
							timePast = time(iTotalTime)
		  
						endif
						
c       	       get measured resisting forces
					
						do i = 1, ndofel
							pr(i) = rData(i)
							rhs(i,1) = pr(i)
						enddo
						
c	    	    		   assign daq values for feedback
						do i = 1, ndofel
						
							svars(i) = u(i)
							!svars(ndofel+i) = -pr(i)
						enddo			   
						
						!write(*,*) 'svars', svars
														
c						   set initial stiffness matrix 
						if (props(3) .ne. 1) then
							
							write(*,*) 'ERROR - initial interface stiffness is not defined' 
							call xit
							
						else if (stiff_flag1 .eq. 0) then
							write(*,*) 'read data from the input file'
							
							if (.not.allocated(stiffness)) then
								allocate(stiffness(ndofel,ndofel))
							endif
							
							k = 1
							
							do i = 1, ndofel
								do j = 1, ndofel
									stiffness(i,j) = props(3+k)
									!write(*,*) 'k=', k
									k=k+1
								enddo
							enddo
							
							
							k = 1
							do i = 1, ndofel
								do j = 1, ndofel
									amatrx(i,j) = stiffness(i,j)
									WRITE(*,*) AMATRX(I,J)
									k=k+1  
								enddo
							enddo				
			   
							stiff_flag1 = 1
							
						else if (stiff_flag1 .eq. 1) then
							k = 1
				
							if (.not.allocated(stiffness)) then
								write(*,*) 'ERROR - Fail to save stiffness matrix'
								call xit
							endif
							
							do i = 1, ndofel
								do j = 1, ndofel
									amatrx(i,j) = stiffness(i,j)
									WRITE(*,*) AMATRX(I,J)
									k=k+1  
								enddo
							enddo		
							
						endif
						
						
					else 
					! This is static analysis for pseudo-dynamic substructure modules
						if (time(iTotalTime) .gt. timePast) then
							
c       	    		       recv trial response to experimental element
			
							cmd = command(socketID, iflag, protocol)
								
							SELECT CASE(cmd)
								CASE (3)
								
								lens = indicator()
								if (.not. allocated(rData)) then
									allocate(rData(lens))
								endif 
								
								if (recvdata(socketID,rData,lens,protocol) .ne. 0) then
									write(*,*) 'ERROR - failed to recv trial response'
								endif
													
								CASE (10)
								
								lens = indicator()
								write(*,*) 'lens = ', lens
								if (.not. allocated(sData)) then
									allocate(sData(lens))
								endif 
								
								do i = 1, lens
									sData(i) = svars(i)
								enddo
								
								if (senddata(socketID,sData,lens,protocol) .ne. 0) then
									write(*,*) 'ERROR - failed to send restoring forces'
								endif					  
								
								CASE (99)
									iresult = terminate(socketID)
									call xit
								
								CASE DEFAULT
								
								write(*,*) 'ERROR - incorrect command received ', cmd
								call xit
								
							END SELECT
							
							timePast = time(iTotalTime)
							
						endif
					
c       	       get measured resisting forces
					
						do i = 1, ndofel
							pr(i) = props(3)*(u(i) - rData(i))
							rhs(i,1) = -pr(i)
						enddo
						
c	    	    		   assign daq values for feedback
						do i = 1, ndofel
						
							svars(i) = u(i)
							svars(ndofel+i) = -pr(i)
						enddo			   
						
						!write(*,*) 'svars', svars
														
c						   set initial stiffness matrix 
						
						do i = 1, ndofel
							amatrx(i,i) = props(3)
						enddo
						
						
					endif
				
				endif
         else if (lflags(iProcedure) .eq. jDynImpHalfStepRes .or.
     *            lflags(iProcedure) .eq. jDynImpFixedTimeIncr) then
	                
			if (Ttype .eq. 5) then
				if (flag_dyna .eq. 0) then ! fake dtime (very large value)
					if (.not. allocated(rData)) then
						allocate(rData(ndofel))
						do i = 1, ndofel
							rData(i) = 0.
						enddo
					endif
					
					flag_dyna = 1
c					set initial stiffness matrix 
							
					k = 1		
					do i = 1, ndofel
						do j = 1, ndofel
							amatrx(i,j) = 0 !1.D0/(PARAMS(2)*0.01**2)*Mm((i-1)*ndofel+j)
							k=k+1 
							!write(*,*) amatrx(i,j)
						enddo
					enddo	
				
				else if (time(iTotalTime) .gt. timePast ) then
						! recv equivalent interface force  
					
						cmd = command(socketID, iflag, protocol)
						write(*,*) 'RecvCMD:', cmd
						
						!if (flag_iteration1 .eq. 0) then
						!	flag_iteration1 = 1
						!else if (flag_iteration1 .eq. 1) then	
						!	flag_iteration1 = 2
						!		
						!do i = 1, ndofel
						!	svars(ndofel+i) = svars(i)
						!	svars(i) = rData(i)	
						!enddo	
						!
					    !
						!endif
						
						SELECT CASE(cmd)
							CASE (3)
														!lens = indicator()
							lens = ndofel
							if (.not. allocated(rData)) then
								allocate(rData(lens))
							endif 
							
							write(*,*) 'RecvData: ', lens
							
							if (recvdata(socketID,rData,lens,protocol) .ne. 0) then
								write(*,*) 'ERROR - failed to recv trial response'
							endif
							
							!rhs(i,1) = (1.0+alpha)*rData(i) + alpha*svars(i)
							
							!do i = 1, ndofel
							!	svars(ndofel+i) = svars(i)
							!	svars(i) = rData(i)	
							!enddo				
							
							 CASE (10)
							 
							 lens = ndofel
							 
							 if (.not. allocated(sData)) then
							 	allocate(sData(lens))
							 endif 
							 
							 do i = 1, lens
							 	sData(i) = svars(i)   
							 enddo
							 
							 write(*,*) 'SendData: ', lens
							 
							 if (senddata(socketID,sData,lens,protocol) .ne. 0) then
							 	write(*,*) 'ERROR - failed to send restoring forces'
							 endif					  
							 
							 do i = 1, ndofel
							 	svars(ndofel+i) = svars(i)
							 	svars(i) = rData(i)
								!write(*,*) sData(i)
							 enddo
							 
							cmd = command(socketID, iflag, protocol)
							write(*,*) 'RecvCMD:', cmd
							 
							lens = ndofel
							if (.not. allocated(rData)) then
								allocate(rData(lens))
							endif 
							
							write(*,*) 'RecvData: ', lens
							
							if (recvdata(socketID,rData,lens,protocol) .ne. 0) then
								write(*,*) 'ERROR - failed to recv trial response'
							endif 
							 
							 
							
							CASE (99)
								iresult = terminate(socketID)
								call xit
							
							CASE DEFAULT
							
							write(*,*) 'ERROR - incorrect command received ', cmd
							call xit
							
						END SELECT
						
						timePast = time(iTotalTime)
						!if (flag_iteration1 .eq. 1) then
						!	flag_iteration2 = 1
						!endif
						
c					set initial stiffness matrix 
							
					k = 1		
					do i = 1, ndofel
						do j = 1, ndofel
							amatrx(i,j) = 1.D0/(PARAMS(2)*DTIME**2)*Mm((i-1)*ndofel+j)
							k=k+1 
							!write(*,*) 1.D0/(PARAMS(2)*DTIME**2)
							!write(*,*) amatrx(i,j)
						enddo
					enddo	
						
						
						
						
				endif
				
					
				
				
					
c       	       get measured resisting forces
				
					do i = 1, ndofel
						
						rhs(i,1) = -Mm((i-1)*ndofel+i)*A(i) + (1.0+alpha)*rData(i) - alpha*svars(i)
						!svars(ndofel+i) = svars(i)
						svars(i) = A(i)	
						
					enddo
					
					!write(*,*) 'svars', svars
													
	
						
			
               write(*,*) 'ERROR - dynamic analysis not supported ',
     *                    'in adapter element U',jtype
													
               iresult = terminate(socketID) 
               call xit

			endif 

      endif 
c     stiffness matrix
      else if (lflags(iOpCode) .eq. jCurrentStiff) then
c        get tangent stiffness matrix
            if (Ttype .eq. 5) then 
				if (props(3) .ne. 1) then
							
					write(*,*) 'ERROR - initial interface stiffness is not defined' 
					call xit
					
				else if (stiff_flag1 .eq. 0) then
					write(*,*) 'read data from the input file'
					k = 1
					do i = 1, ndofel
						do j = 1, ndofel
							stiffness(i,j) = 0. !props(3+k)
							k=k+1
						enddo
					enddo
					
					
					k = 1
					do i = 1, ndofel
						do j = 1, ndofel
							amatrx(i,j) = 0.
							k=k+1  
						enddo
					enddo				
			   
					stiff_flag1 = 1
				

				else if (stiff_flag1 .eq. 1) then
					k = 1
				
					if (.not.allocated(stiffness)) then
						write(*,*) 'ERROR - Fail to save stiffness matrix'
						call xit
					endif
					
					do i = 1, ndofel
						do j = 1, ndofel
							amatrx(i,j) = 0.
							k=k+1  
						enddo
					enddo		
					
				endif
			   
			else 
			   
			   do i = 1, ndofel
			      amatrx(i,i) = props(3)
			   enddo
			
			endif
			
			
c     mass matrix
      else if (lflags(iOpCode) .eq. jCurrentMass) then
c        get mass matrix for frequency analysis only

c			   write(*,*) 'Warnings - Mass matrix is not supported ',
c     *                    'in the substructure element',jtype
c               call closeconnection(socketID, stat)
				
				if (Ttype .eq. 5) then
				! system-level analysis
				if (massFlag .eq. 0) then
					cmd = command(socketID, iflag, protocol)
					write(*,*) 'RecvCMD:', cmd
					
					lens = ndofel*ndofel;
					if (.not. allocated(Mm)) then
						allocate(Mm(lens))
					endif 
							
					write(*,*) 'RecvData: ', lens
							
					if (recvdata(socketID,Mm,lens,protocol) .ne. 0) then
						write(*,*) 'ERROR - failed to recv trial response'
					endif
						
					massFlag = 1
				
				endif 
                k = 1
                do i = 1, ndofel
                   do j = 1, ndofel
                      amatrx(i,j) = 0.0
                      !if (i .eq. j .and. amatrx(i,j) .le. 0.0) then
                      !   amatrx(i,j) = 1.E-8
                      !endif
					  !write(*,*) amatrx(i,j)
                      amatrx(i,j) = Mm((i-1)*ndofel+j)
					  !write(*,*) amatrx(i,j)
					  !write(*,*) ndofel
					  k = k + 1
                   enddo
                enddo
				
				else
					! component-level analysis
					do i = 1, ndofel
								   
						amatrx(i,i) = 0.0
							
					enddo
				
				endif
				
			         
c     residual calculation
      else if (lflags(iOpCode) .eq. jCurrentResidual) then
         alpha = params(1)
			! this is only used for half step residual calculation, which is needed only for auotmatic time incrementation.
			! can be ingored in this implementation
			! in other words, alpha = 0.
			!write(*,*) 'residual force calculation'
			if (Ttype .eq. 5) then
				
				write(*,*) 'ERROR - residual calculation not supported ',
     *                    'in SubSub element U',jtype
				iresult = terminate(socketID) 
               call xit
			
			else 
		 
               write(*,*) 'ERROR - residual calculation not supported ',
     *                    'in SubSub element U',jtype
               iresult = terminate(socketID) 
               call xit
			   
			endif    
			   
			   
c     initial acceleration calculation
      else if (lflags(iOpCode) .eq. jInitialAccel) then

         do i = 1, ndofel
            amatrx(i,i) = 0.0
            rhs(i,1) = 0.0
         enddo

      endif
      
      return
      end