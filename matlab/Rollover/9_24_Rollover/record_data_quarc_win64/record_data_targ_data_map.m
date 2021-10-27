  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 4;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (record_data_P)
    ;%
      section.nData     = 15;
      section.data(15)  = dumData; %prealloc
      
	  ;% record_data_P.HILInitialize_OOStart
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_P.HILInitialize_OOEnter
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% record_data_P.HILInitialize_OOTerminate
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% record_data_P.HILInitialize_OOExit
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% record_data_P.HILInitialize_AIHigh
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% record_data_P.HILInitialize_AILow
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% record_data_P.HILInitialize_AOHigh
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% record_data_P.HILInitialize_AOLow
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% record_data_P.HILInitialize_AOInitial
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% record_data_P.HILInitialize_AOFinal
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 9;
	
	  ;% record_data_P.HILInitialize_AOWatchdog
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 10;
	
	  ;% record_data_P.HILInitialize_EIFrequency
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 11;
	
	  ;% record_data_P.HILInitialize_POFrequency
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 12;
	
	  ;% record_data_P.HILInitialize_POInitial
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 13;
	
	  ;% record_data_P.HILInitialize_POFinal
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 14;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% record_data_P.HILInitialize_CKChannels
	  section.data(1).logicalSrcIdx = 15;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_P.HILInitialize_CKModes
	  section.data(2).logicalSrcIdx = 16;
	  section.data(2).dtTransOffset = 2;
	
	  ;% record_data_P.HILInitialize_DOWatchdog
	  section.data(3).logicalSrcIdx = 17;
	  section.data(3).dtTransOffset = 4;
	
	  ;% record_data_P.HILInitialize_EIInitial
	  section.data(4).logicalSrcIdx = 18;
	  section.data(4).dtTransOffset = 5;
	
	  ;% record_data_P.HILInitialize_POModes
	  section.data(5).logicalSrcIdx = 19;
	  section.data(5).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% record_data_P.HILInitialize_AIChannels
	  section.data(1).logicalSrcIdx = 20;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_P.HILInitialize_AOChannels
	  section.data(2).logicalSrcIdx = 21;
	  section.data(2).dtTransOffset = 4;
	
	  ;% record_data_P.HILInitialize_EIChannels
	  section.data(3).logicalSrcIdx = 22;
	  section.data(3).dtTransOffset = 8;
	
	  ;% record_data_P.HILInitialize_EIQuadrature
	  section.data(4).logicalSrcIdx = 23;
	  section.data(4).dtTransOffset = 12;
	
	  ;% record_data_P.HILReadAnalog_Channels
	  section.data(5).logicalSrcIdx = 24;
	  section.data(5).dtTransOffset = 13;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 36;
      section.data(36)  = dumData; %prealloc
      
	  ;% record_data_P.HILInitialize_Active
	  section.data(1).logicalSrcIdx = 25;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_P.HILInitialize_CKPStart
	  section.data(2).logicalSrcIdx = 26;
	  section.data(2).dtTransOffset = 1;
	
	  ;% record_data_P.HILInitialize_CKPEnter
	  section.data(3).logicalSrcIdx = 27;
	  section.data(3).dtTransOffset = 2;
	
	  ;% record_data_P.HILInitialize_CKStart
	  section.data(4).logicalSrcIdx = 28;
	  section.data(4).dtTransOffset = 3;
	
	  ;% record_data_P.HILInitialize_CKEnter
	  section.data(5).logicalSrcIdx = 29;
	  section.data(5).dtTransOffset = 4;
	
	  ;% record_data_P.HILInitialize_AIPStart
	  section.data(6).logicalSrcIdx = 30;
	  section.data(6).dtTransOffset = 5;
	
	  ;% record_data_P.HILInitialize_AIPEnter
	  section.data(7).logicalSrcIdx = 31;
	  section.data(7).dtTransOffset = 6;
	
	  ;% record_data_P.HILInitialize_AOPStart
	  section.data(8).logicalSrcIdx = 32;
	  section.data(8).dtTransOffset = 7;
	
	  ;% record_data_P.HILInitialize_AOPEnter
	  section.data(9).logicalSrcIdx = 33;
	  section.data(9).dtTransOffset = 8;
	
	  ;% record_data_P.HILInitialize_AOStart
	  section.data(10).logicalSrcIdx = 34;
	  section.data(10).dtTransOffset = 9;
	
	  ;% record_data_P.HILInitialize_AOEnter
	  section.data(11).logicalSrcIdx = 35;
	  section.data(11).dtTransOffset = 10;
	
	  ;% record_data_P.HILInitialize_AOTerminate
	  section.data(12).logicalSrcIdx = 36;
	  section.data(12).dtTransOffset = 11;
	
	  ;% record_data_P.HILInitialize_AOExit
	  section.data(13).logicalSrcIdx = 37;
	  section.data(13).dtTransOffset = 12;
	
	  ;% record_data_P.HILInitialize_AOReset
	  section.data(14).logicalSrcIdx = 38;
	  section.data(14).dtTransOffset = 13;
	
	  ;% record_data_P.HILInitialize_DOPStart
	  section.data(15).logicalSrcIdx = 39;
	  section.data(15).dtTransOffset = 14;
	
	  ;% record_data_P.HILInitialize_DOPEnter
	  section.data(16).logicalSrcIdx = 40;
	  section.data(16).dtTransOffset = 15;
	
	  ;% record_data_P.HILInitialize_DOStart
	  section.data(17).logicalSrcIdx = 41;
	  section.data(17).dtTransOffset = 16;
	
	  ;% record_data_P.HILInitialize_DOEnter
	  section.data(18).logicalSrcIdx = 42;
	  section.data(18).dtTransOffset = 17;
	
	  ;% record_data_P.HILInitialize_DOTerminate
	  section.data(19).logicalSrcIdx = 43;
	  section.data(19).dtTransOffset = 18;
	
	  ;% record_data_P.HILInitialize_DOExit
	  section.data(20).logicalSrcIdx = 44;
	  section.data(20).dtTransOffset = 19;
	
	  ;% record_data_P.HILInitialize_DOReset
	  section.data(21).logicalSrcIdx = 45;
	  section.data(21).dtTransOffset = 20;
	
	  ;% record_data_P.HILInitialize_EIPStart
	  section.data(22).logicalSrcIdx = 46;
	  section.data(22).dtTransOffset = 21;
	
	  ;% record_data_P.HILInitialize_EIPEnter
	  section.data(23).logicalSrcIdx = 47;
	  section.data(23).dtTransOffset = 22;
	
	  ;% record_data_P.HILInitialize_EIStart
	  section.data(24).logicalSrcIdx = 48;
	  section.data(24).dtTransOffset = 23;
	
	  ;% record_data_P.HILInitialize_EIEnter
	  section.data(25).logicalSrcIdx = 49;
	  section.data(25).dtTransOffset = 24;
	
	  ;% record_data_P.HILInitialize_POPStart
	  section.data(26).logicalSrcIdx = 50;
	  section.data(26).dtTransOffset = 25;
	
	  ;% record_data_P.HILInitialize_POPEnter
	  section.data(27).logicalSrcIdx = 51;
	  section.data(27).dtTransOffset = 26;
	
	  ;% record_data_P.HILInitialize_POStart
	  section.data(28).logicalSrcIdx = 52;
	  section.data(28).dtTransOffset = 27;
	
	  ;% record_data_P.HILInitialize_POEnter
	  section.data(29).logicalSrcIdx = 53;
	  section.data(29).dtTransOffset = 28;
	
	  ;% record_data_P.HILInitialize_POTerminate
	  section.data(30).logicalSrcIdx = 54;
	  section.data(30).dtTransOffset = 29;
	
	  ;% record_data_P.HILInitialize_POExit
	  section.data(31).logicalSrcIdx = 55;
	  section.data(31).dtTransOffset = 30;
	
	  ;% record_data_P.HILInitialize_POReset
	  section.data(32).logicalSrcIdx = 56;
	  section.data(32).dtTransOffset = 31;
	
	  ;% record_data_P.HILInitialize_OOReset
	  section.data(33).logicalSrcIdx = 57;
	  section.data(33).dtTransOffset = 32;
	
	  ;% record_data_P.HILInitialize_DOInitial
	  section.data(34).logicalSrcIdx = 58;
	  section.data(34).dtTransOffset = 33;
	
	  ;% record_data_P.HILInitialize_DOFinal
	  section.data(35).logicalSrcIdx = 59;
	  section.data(35).dtTransOffset = 34;
	
	  ;% record_data_P.HILReadAnalog_Active
	  section.data(36).logicalSrcIdx = 60;
	  section.data(36).dtTransOffset = 35;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 0;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (record_data_B)
    ;%
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 5;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (record_data_DWork)
    ;%
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% record_data_DWork.HILInitialize_AIMinimums
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_DWork.HILInitialize_AIMaximums
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 4;
	
	  ;% record_data_DWork.HILInitialize_AOMinimums
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 8;
	
	  ;% record_data_DWork.HILInitialize_AOMaximums
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 12;
	
	  ;% record_data_DWork.HILInitialize_AOVoltages
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 16;
	
	  ;% record_data_DWork.HILInitialize_FilterFrequency
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 20;
	
	  ;% record_data_DWork.HILReadAnalog_Buffer
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 24;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% record_data_DWork.HILInitialize_Card
	  section.data(1).logicalSrcIdx = 7;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% record_data_DWork.HILReadAnalog_PWORK
	  section.data(1).logicalSrcIdx = 8;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_DWork.ToFile_PWORK.FilePtr
	  section.data(2).logicalSrcIdx = 9;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% record_data_DWork.HILInitialize_QuadratureModes
	  section.data(1).logicalSrcIdx = 10;
	  section.data(1).dtTransOffset = 0;
	
	  ;% record_data_DWork.HILInitialize_InitialEICounts
	  section.data(2).logicalSrcIdx = 11;
	  section.data(2).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% record_data_DWork.ToFile_IWORK.Count
	  section.data(1).logicalSrcIdx = 12;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 3125857871;
  targMap.checksum1 = 3337065614;
  targMap.checksum2 = 42826559;
  targMap.checksum3 = 768167036;

