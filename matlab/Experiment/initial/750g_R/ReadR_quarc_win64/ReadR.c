/*
 * ReadR.c
 *
 * Code generation for model "ReadR.mdl".
 *
 * Model version              : 1.27
 * Simulink Coder version : 8.1 (R2011b) 08-Jul-2011
 * C source code generated on : Thu Sep 10 15:13:14 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "ReadR.h"
#include "ReadR_private.h"
#include "ReadR_dt.h"

/* Block signals (auto storage) */
BlockIO_ReadR ReadR_B;

/* Block states (auto storage) */
D_Work_ReadR ReadR_DWork;

/* Real-time model */
RT_MODEL_ReadR ReadR_M_;
RT_MODEL_ReadR *const ReadR_M = &ReadR_M_;

/*
 * Writes out MAT-file header.  Returns success or failure.
 * Returns:
 *      0 - success
 *      1 - failure
 */
int_T rt_WriteMat4FileHeader(FILE *fp, int32_T m, int32_T n, const char *name)
{
  typedef enum { ELITTLE_ENDIAN, EBIG_ENDIAN } ByteOrder;

  int16_T one = 1;
  ByteOrder byteOrder = (*((int8_T *)&one)==1) ? ELITTLE_ENDIAN : EBIG_ENDIAN;
  int32_T type = (byteOrder == ELITTLE_ENDIAN) ? 0: 1000;
  int32_T imagf = 0;
  int32_T name_len = strlen(name) + 1;
  if ((fwrite(&type, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&m, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&n, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&imagf, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&name_len, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(name, sizeof(char), name_len, fp) == 0)) {
    return(1);
  } else {
    return(0);
  }
}                                      /* end rt_WriteMat4FileHeader */

/* Model output function */
void ReadR_output(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_HILReadAnalog1;

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog' */

  /* S-Function Block: ReadR/HIL Read Analog (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(ReadR_DWork.HILInitialize_Card,
      &ReadR_P.HILReadAnalog_Channels, 1, &ReadR_DWork.HILReadAnalog_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadR_M, _rt_error_message);
    }

    ReadR_B.HILReadAnalog = ReadR_DWork.HILReadAnalog_Buffer;
  }

  /* ToFile: '<Root>/To File' */
  if (!(++ReadR_DWork.ToFile_IWORK.Decimation % 1) &&
      (ReadR_DWork.ToFile_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) ReadR_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      ReadR_DWork.ToFile_IWORK.Decimation = 0;
      u[0] = ReadR_M->Timing.t[0];
      u[1] = ReadR_B.HILReadAnalog;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(ReadR_M, "Error writing to MAT-file ch0.mat");
        return;
      }

      if (((++ReadR_DWork.ToFile_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file ch0.mat.\n");
      }
    }
  }

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog1' */

  /* S-Function Block: ReadR/HIL Read Analog1 (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(ReadR_DWork.HILInitialize_Card,
      &ReadR_P.HILReadAnalog1_Channels, 1, &ReadR_DWork.HILReadAnalog1_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadR_M, _rt_error_message);
    }

    rtb_HILReadAnalog1 = ReadR_DWork.HILReadAnalog1_Buffer;
  }

  /* ToFile: '<Root>/To File1' */
  if (!(++ReadR_DWork.ToFile1_IWORK.Decimation % 1) &&
      (ReadR_DWork.ToFile1_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) ReadR_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      ReadR_DWork.ToFile1_IWORK.Decimation = 0;
      u[0] = ReadR_M->Timing.t[0];
      u[1] = rtb_HILReadAnalog1;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(ReadR_M, "Error writing to MAT-file ch1.mat");
        return;
      }

      if (((++ReadR_DWork.ToFile1_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file ch1.mat.\n");
      }
    }
  }

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model update function */
void ReadR_update(int_T tid)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++ReadR_M->Timing.clockTick0)) {
    ++ReadR_M->Timing.clockTickH0;
  }

  ReadR_M->Timing.t[0] = ReadR_M->Timing.clockTick0 * ReadR_M->Timing.stepSize0
    + ReadR_M->Timing.clockTickH0 * ReadR_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void ReadR_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)ReadR_M, 0,
                sizeof(RT_MODEL_ReadR));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = ReadR_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    ReadR_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    ReadR_M->Timing.sampleTimes = (&ReadR_M->Timing.sampleTimesArray[0]);
    ReadR_M->Timing.offsetTimes = (&ReadR_M->Timing.offsetTimesArray[0]);

    /* task periods */
    ReadR_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    ReadR_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(ReadR_M, &ReadR_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = ReadR_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    ReadR_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(ReadR_M, 2.0);
  ReadR_M->Timing.stepSize0 = 0.001;

  /* external mode info */
  ReadR_M->Sizes.checksums[0] = (1057398693U);
  ReadR_M->Sizes.checksums[1] = (473297492U);
  ReadR_M->Sizes.checksums[2] = (2237844436U);
  ReadR_M->Sizes.checksums[3] = (3641120578U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    ReadR_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(ReadR_M->extModeInfo,
      &ReadR_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(ReadR_M->extModeInfo, ReadR_M->Sizes.checksums);
    rteiSetTPtr(ReadR_M->extModeInfo, rtmGetTPtr(ReadR_M));
  }

  ReadR_M->solverInfoPtr = (&ReadR_M->solverInfo);
  ReadR_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&ReadR_M->solverInfo, 0.001);
  rtsiSetSolverMode(&ReadR_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  ReadR_M->ModelData.blockIO = ((void *) &ReadR_B);
  (void) memset(((void *) &ReadR_B), 0,
                sizeof(BlockIO_ReadR));

  /* parameters */
  ReadR_M->ModelData.defaultParam = ((real_T *)&ReadR_P);

  /* states (dwork) */
  ReadR_M->Work.dwork = ((void *) &ReadR_DWork);
  (void) memset((void *)&ReadR_DWork, 0,
                sizeof(D_Work_ReadR));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    ReadR_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 15;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.B = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.P = &rtPTransTable;
  }
}

/* Model terminate function */
void ReadR_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: ReadR/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    hil_task_stop_all(ReadR_DWork.HILInitialize_Card);
    hil_monitor_stop_all(ReadR_DWork.HILInitialize_Card);
    is_switching = false;
    if ((ReadR_P.HILInitialize_AOTerminate && !is_switching) ||
        (ReadR_P.HILInitialize_AOExit && is_switching)) {
      ReadR_DWork.HILInitialize_AOVoltages[0] = ReadR_P.HILInitialize_AOFinal;
      ReadR_DWork.HILInitialize_AOVoltages[1] = ReadR_P.HILInitialize_AOFinal;
      ReadR_DWork.HILInitialize_AOVoltages[2] = ReadR_P.HILInitialize_AOFinal;
      ReadR_DWork.HILInitialize_AOVoltages[3] = ReadR_P.HILInitialize_AOFinal;
      result = hil_write_analog(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_AOChannels, 4U,
        &ReadR_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
      }
    }

    hil_task_delete_all(ReadR_DWork.HILInitialize_Card);
    hil_monitor_delete_all(ReadR_DWork.HILInitialize_Card);
    hil_close(ReadR_DWork.HILInitialize_Card);
    ReadR_DWork.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) ReadR_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "ch0.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadR_M, "Error closing MAT-file ch0.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(ReadR_M, "Error reopening MAT-file ch0.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2, ReadR_DWork.ToFile_IWORK.Count, "ch0"))
      {
        rtmSetErrorStatus(ReadR_M,
                          "Error writing header for ch0 to MAT-file ch0.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadR_M, "Error closing MAT-file ch0.mat");
        return;
      }

      ReadR_DWork.ToFile_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (FILE *) ReadR_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "ch1.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadR_M, "Error closing MAT-file ch1.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(ReadR_M, "Error reopening MAT-file ch1.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2, ReadR_DWork.ToFile1_IWORK.Count, "ch1"))
      {
        rtmSetErrorStatus(ReadR_M,
                          "Error writing header for ch1 to MAT-file ch1.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadR_M, "Error closing MAT-file ch1.mat");
        return;
      }

      ReadR_DWork.ToFile1_PWORK.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  ReadR_output(tid);
}

void MdlUpdate(int_T tid)
{
  ReadR_update(tid);
}

void MdlInitializeSizes(void)
{
  ReadR_M->Sizes.numContStates = (0);  /* Number of continuous states */
  ReadR_M->Sizes.numY = (0);           /* Number of model outputs */
  ReadR_M->Sizes.numU = (0);           /* Number of model inputs */
  ReadR_M->Sizes.sysDirFeedThru = (0); /* The model is not direct feedthrough */
  ReadR_M->Sizes.numSampTimes = (1);   /* Number of sample times */
  ReadR_M->Sizes.numBlocks = (6);      /* Number of blocks */
  ReadR_M->Sizes.numBlockIO = (1);     /* Number of block outputs */
  ReadR_M->Sizes.numBlockPrms = (74);  /* Sum of parameter "widths" */
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: ReadR/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q4", "0", &ReadR_DWork.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadR_M, _rt_error_message);
      return;
    }

    is_switching = false;
    if ((ReadR_P.HILInitialize_CKPStart && !is_switching) ||
        (ReadR_P.HILInitialize_CKPEnter && is_switching)) {
      result = hil_set_clock_mode(ReadR_DWork.HILInitialize_Card, (t_clock *)
        ReadR_P.HILInitialize_CKChannels, 2U, (t_clock_mode *)
        ReadR_P.HILInitialize_CKModes);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    result = hil_watchdog_clear(ReadR_DWork.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadR_M, _rt_error_message);
      return;
    }

    if ((ReadR_P.HILInitialize_AIPStart && !is_switching) ||
        (ReadR_P.HILInitialize_AIPEnter && is_switching)) {
      ReadR_DWork.HILInitialize_AIMinimums[0] = ReadR_P.HILInitialize_AILow;
      ReadR_DWork.HILInitialize_AIMinimums[1] = ReadR_P.HILInitialize_AILow;
      ReadR_DWork.HILInitialize_AIMinimums[2] = ReadR_P.HILInitialize_AILow;
      ReadR_DWork.HILInitialize_AIMinimums[3] = ReadR_P.HILInitialize_AILow;
      ReadR_DWork.HILInitialize_AIMaximums[0] = ReadR_P.HILInitialize_AIHigh;
      ReadR_DWork.HILInitialize_AIMaximums[1] = ReadR_P.HILInitialize_AIHigh;
      ReadR_DWork.HILInitialize_AIMaximums[2] = ReadR_P.HILInitialize_AIHigh;
      ReadR_DWork.HILInitialize_AIMaximums[3] = ReadR_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_AIChannels, 4U,
        &ReadR_DWork.HILInitialize_AIMinimums[0],
        &ReadR_DWork.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    if ((ReadR_P.HILInitialize_AOPStart && !is_switching) ||
        (ReadR_P.HILInitialize_AOPEnter && is_switching)) {
      ReadR_DWork.HILInitialize_AOMinimums[0] = ReadR_P.HILInitialize_AOLow;
      ReadR_DWork.HILInitialize_AOMinimums[1] = ReadR_P.HILInitialize_AOLow;
      ReadR_DWork.HILInitialize_AOMinimums[2] = ReadR_P.HILInitialize_AOLow;
      ReadR_DWork.HILInitialize_AOMinimums[3] = ReadR_P.HILInitialize_AOLow;
      ReadR_DWork.HILInitialize_AOMaximums[0] = ReadR_P.HILInitialize_AOHigh;
      ReadR_DWork.HILInitialize_AOMaximums[1] = ReadR_P.HILInitialize_AOHigh;
      ReadR_DWork.HILInitialize_AOMaximums[2] = ReadR_P.HILInitialize_AOHigh;
      ReadR_DWork.HILInitialize_AOMaximums[3] = ReadR_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_AOChannels, 4U,
        &ReadR_DWork.HILInitialize_AOMinimums[0],
        &ReadR_DWork.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    if ((ReadR_P.HILInitialize_AOStart && !is_switching) ||
        (ReadR_P.HILInitialize_AOEnter && is_switching)) {
      ReadR_DWork.HILInitialize_AOVoltages[0] = ReadR_P.HILInitialize_AOInitial;
      ReadR_DWork.HILInitialize_AOVoltages[1] = ReadR_P.HILInitialize_AOInitial;
      ReadR_DWork.HILInitialize_AOVoltages[2] = ReadR_P.HILInitialize_AOInitial;
      ReadR_DWork.HILInitialize_AOVoltages[3] = ReadR_P.HILInitialize_AOInitial;
      result = hil_write_analog(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_AOChannels, 4U,
        &ReadR_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    if (ReadR_P.HILInitialize_AOReset) {
      ReadR_DWork.HILInitialize_AOVoltages[0] = ReadR_P.HILInitialize_AOWatchdog;
      ReadR_DWork.HILInitialize_AOVoltages[1] = ReadR_P.HILInitialize_AOWatchdog;
      ReadR_DWork.HILInitialize_AOVoltages[2] = ReadR_P.HILInitialize_AOWatchdog;
      ReadR_DWork.HILInitialize_AOVoltages[3] = ReadR_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (ReadR_DWork.HILInitialize_Card, ReadR_P.HILInitialize_AOChannels, 4U,
         &ReadR_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    if ((ReadR_P.HILInitialize_EIPStart && !is_switching) ||
        (ReadR_P.HILInitialize_EIPEnter && is_switching)) {
      ReadR_DWork.HILInitialize_QuadratureModes[0] =
        ReadR_P.HILInitialize_EIQuadrature;
      ReadR_DWork.HILInitialize_QuadratureModes[1] =
        ReadR_P.HILInitialize_EIQuadrature;
      ReadR_DWork.HILInitialize_QuadratureModes[2] =
        ReadR_P.HILInitialize_EIQuadrature;
      ReadR_DWork.HILInitialize_QuadratureModes[3] =
        ReadR_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_EIChannels, 4U, (t_encoder_quadrature_mode *)
        &ReadR_DWork.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }

      ReadR_DWork.HILInitialize_FilterFrequency[0] =
        ReadR_P.HILInitialize_EIFrequency;
      ReadR_DWork.HILInitialize_FilterFrequency[1] =
        ReadR_P.HILInitialize_EIFrequency;
      ReadR_DWork.HILInitialize_FilterFrequency[2] =
        ReadR_P.HILInitialize_EIFrequency;
      ReadR_DWork.HILInitialize_FilterFrequency[3] =
        ReadR_P.HILInitialize_EIFrequency;
      result = hil_set_encoder_filter_frequency(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_EIChannels, 4U,
        &ReadR_DWork.HILInitialize_FilterFrequency[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }

    if ((ReadR_P.HILInitialize_EIStart && !is_switching) ||
        (ReadR_P.HILInitialize_EIEnter && is_switching)) {
      ReadR_DWork.HILInitialize_InitialEICounts[0] =
        ReadR_P.HILInitialize_EIInitial;
      ReadR_DWork.HILInitialize_InitialEICounts[1] =
        ReadR_P.HILInitialize_EIInitial;
      ReadR_DWork.HILInitialize_InitialEICounts[2] =
        ReadR_P.HILInitialize_EIInitial;
      ReadR_DWork.HILInitialize_InitialEICounts[3] =
        ReadR_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(ReadR_DWork.HILInitialize_Card,
        ReadR_P.HILInitialize_EIChannels, 4U,
        &ReadR_DWork.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadR_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for ToFile: '<Root>/To File' */
  {
    char fileName[509] = "ch0.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(ReadR_M, "Error creating .mat file ch0.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch0")) {
      rtmSetErrorStatus(ReadR_M, "Error writing mat file header to file ch0.mat");
      return;
    }

    ReadR_DWork.ToFile_IWORK.Count = 0;
    ReadR_DWork.ToFile_IWORK.Decimation = -1;
    ReadR_DWork.ToFile_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/To File1' */
  {
    char fileName[509] = "ch1.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(ReadR_M, "Error creating .mat file ch1.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch1")) {
      rtmSetErrorStatus(ReadR_M, "Error writing mat file header to file ch1.mat");
      return;
    }

    ReadR_DWork.ToFile1_IWORK.Count = 0;
    ReadR_DWork.ToFile1_IWORK.Decimation = -1;
    ReadR_DWork.ToFile1_PWORK.FilePtr = fp;
  }

  MdlInitialize();
}

void MdlTerminate(void)
{
  ReadR_terminate();
}

RT_MODEL_ReadR *ReadR(void)
{
  ReadR_initialize(1);
  return ReadR_M;
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
