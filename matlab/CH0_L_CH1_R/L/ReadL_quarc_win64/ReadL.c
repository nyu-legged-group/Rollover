/*
 * ReadL.c
 *
 * Code generation for model "ReadL.mdl".
 *
 * Model version              : 1.26
 * Simulink Coder version : 8.1 (R2011b) 08-Jul-2011
 * C source code generated on : Thu Sep 10 15:29:09 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "ReadL.h"
#include "ReadL_private.h"
#include "ReadL_dt.h"

/* Block states (auto storage) */
D_Work_ReadL ReadL_DWork;

/* Real-time model */
RT_MODEL_ReadL ReadL_M_;
RT_MODEL_ReadL *const ReadL_M = &ReadL_M_;

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
void ReadL_output(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_HILReadAnalog1;

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog' */

  /* S-Function Block: ReadL/HIL Read Analog (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(ReadL_DWork.HILInitialize_Card,
      &ReadL_P.HILReadAnalog_Channels, 1, &ReadL_DWork.HILReadAnalog_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadL_M, _rt_error_message);
    }

    rtb_HILReadAnalog1 = ReadL_DWork.HILReadAnalog_Buffer;
  }

  /* ToFile: '<Root>/To File' */
  if (!(++ReadL_DWork.ToFile_IWORK.Decimation % 1) &&
      (ReadL_DWork.ToFile_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) ReadL_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      ReadL_DWork.ToFile_IWORK.Decimation = 0;
      u[0] = ReadL_M->Timing.t[0];
      u[1] = rtb_HILReadAnalog1;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(ReadL_M, "Error writing to MAT-file ch0.mat");
        return;
      }

      if (((++ReadL_DWork.ToFile_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file ch0.mat.\n");
      }
    }
  }

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog1' */

  /* S-Function Block: ReadL/HIL Read Analog1 (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(ReadL_DWork.HILInitialize_Card,
      &ReadL_P.HILReadAnalog1_Channels, 1, &ReadL_DWork.HILReadAnalog1_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadL_M, _rt_error_message);
    }

    rtb_HILReadAnalog1 = ReadL_DWork.HILReadAnalog1_Buffer;
  }

  /* ToFile: '<Root>/To File1' */
  if (!(++ReadL_DWork.ToFile1_IWORK.Decimation % 1) &&
      (ReadL_DWork.ToFile1_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) ReadL_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      ReadL_DWork.ToFile1_IWORK.Decimation = 0;
      u[0] = ReadL_M->Timing.t[0];
      u[1] = rtb_HILReadAnalog1;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(ReadL_M, "Error writing to MAT-file ch1.mat");
        return;
      }

      if (((++ReadL_DWork.ToFile1_IWORK.Count)*2)+1 >= 100000000) {
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
void ReadL_update(int_T tid)
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
  if (!(++ReadL_M->Timing.clockTick0)) {
    ++ReadL_M->Timing.clockTickH0;
  }

  ReadL_M->Timing.t[0] = ReadL_M->Timing.clockTick0 * ReadL_M->Timing.stepSize0
    + ReadL_M->Timing.clockTickH0 * ReadL_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void ReadL_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)ReadL_M, 0,
                sizeof(RT_MODEL_ReadL));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = ReadL_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    ReadL_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    ReadL_M->Timing.sampleTimes = (&ReadL_M->Timing.sampleTimesArray[0]);
    ReadL_M->Timing.offsetTimes = (&ReadL_M->Timing.offsetTimesArray[0]);

    /* task periods */
    ReadL_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    ReadL_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(ReadL_M, &ReadL_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = ReadL_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    ReadL_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(ReadL_M, 2.0);
  ReadL_M->Timing.stepSize0 = 0.001;

  /* external mode info */
  ReadL_M->Sizes.checksums[0] = (3028295419U);
  ReadL_M->Sizes.checksums[1] = (135995796U);
  ReadL_M->Sizes.checksums[2] = (1993463673U);
  ReadL_M->Sizes.checksums[3] = (389706212U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    ReadL_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(ReadL_M->extModeInfo,
      &ReadL_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(ReadL_M->extModeInfo, ReadL_M->Sizes.checksums);
    rteiSetTPtr(ReadL_M->extModeInfo, rtmGetTPtr(ReadL_M));
  }

  ReadL_M->solverInfoPtr = (&ReadL_M->solverInfo);
  ReadL_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&ReadL_M->solverInfo, 0.001);
  rtsiSetSolverMode(&ReadL_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* parameters */
  ReadL_M->ModelData.defaultParam = ((real_T *)&ReadL_P);

  /* states (dwork) */
  ReadL_M->Work.dwork = ((void *) &ReadL_DWork);
  (void) memset((void *)&ReadL_DWork, 0,
                sizeof(D_Work_ReadL));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    ReadL_M->SpecialInfo.mappingInfo = (&dtInfo);
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
void ReadL_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: ReadL/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    hil_task_stop_all(ReadL_DWork.HILInitialize_Card);
    hil_monitor_stop_all(ReadL_DWork.HILInitialize_Card);
    is_switching = false;
    if ((ReadL_P.HILInitialize_AOTerminate && !is_switching) ||
        (ReadL_P.HILInitialize_AOExit && is_switching)) {
      ReadL_DWork.HILInitialize_AOVoltages[0] = ReadL_P.HILInitialize_AOFinal;
      ReadL_DWork.HILInitialize_AOVoltages[1] = ReadL_P.HILInitialize_AOFinal;
      ReadL_DWork.HILInitialize_AOVoltages[2] = ReadL_P.HILInitialize_AOFinal;
      ReadL_DWork.HILInitialize_AOVoltages[3] = ReadL_P.HILInitialize_AOFinal;
      result = hil_write_analog(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_AOChannels, 4U,
        &ReadL_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
      }
    }

    hil_task_delete_all(ReadL_DWork.HILInitialize_Card);
    hil_monitor_delete_all(ReadL_DWork.HILInitialize_Card);
    hil_close(ReadL_DWork.HILInitialize_Card);
    ReadL_DWork.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) ReadL_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "ch0.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadL_M, "Error closing MAT-file ch0.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(ReadL_M, "Error reopening MAT-file ch0.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2, ReadL_DWork.ToFile_IWORK.Count, "ch0"))
      {
        rtmSetErrorStatus(ReadL_M,
                          "Error writing header for ch0 to MAT-file ch0.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadL_M, "Error closing MAT-file ch0.mat");
        return;
      }

      ReadL_DWork.ToFile_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (FILE *) ReadL_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "ch1.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadL_M, "Error closing MAT-file ch1.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(ReadL_M, "Error reopening MAT-file ch1.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2, ReadL_DWork.ToFile1_IWORK.Count, "ch1"))
      {
        rtmSetErrorStatus(ReadL_M,
                          "Error writing header for ch1 to MAT-file ch1.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(ReadL_M, "Error closing MAT-file ch1.mat");
        return;
      }

      ReadL_DWork.ToFile1_PWORK.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  ReadL_output(tid);
}

void MdlUpdate(int_T tid)
{
  ReadL_update(tid);
}

void MdlInitializeSizes(void)
{
  ReadL_M->Sizes.numContStates = (0);  /* Number of continuous states */
  ReadL_M->Sizes.numY = (0);           /* Number of model outputs */
  ReadL_M->Sizes.numU = (0);           /* Number of model inputs */
  ReadL_M->Sizes.sysDirFeedThru = (0); /* The model is not direct feedthrough */
  ReadL_M->Sizes.numSampTimes = (1);   /* Number of sample times */
  ReadL_M->Sizes.numBlocks = (5);      /* Number of blocks */
  ReadL_M->Sizes.numBlockIO = (0);     /* Number of block outputs */
  ReadL_M->Sizes.numBlockPrms = (74);  /* Sum of parameter "widths" */
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

  /* S-Function Block: ReadL/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q4", "0", &ReadL_DWork.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadL_M, _rt_error_message);
      return;
    }

    is_switching = false;
    if ((ReadL_P.HILInitialize_CKPStart && !is_switching) ||
        (ReadL_P.HILInitialize_CKPEnter && is_switching)) {
      result = hil_set_clock_mode(ReadL_DWork.HILInitialize_Card, (t_clock *)
        ReadL_P.HILInitialize_CKChannels, 2U, (t_clock_mode *)
        ReadL_P.HILInitialize_CKModes);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    result = hil_watchdog_clear(ReadL_DWork.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(ReadL_M, _rt_error_message);
      return;
    }

    if ((ReadL_P.HILInitialize_AIPStart && !is_switching) ||
        (ReadL_P.HILInitialize_AIPEnter && is_switching)) {
      ReadL_DWork.HILInitialize_AIMinimums[0] = ReadL_P.HILInitialize_AILow;
      ReadL_DWork.HILInitialize_AIMinimums[1] = ReadL_P.HILInitialize_AILow;
      ReadL_DWork.HILInitialize_AIMinimums[2] = ReadL_P.HILInitialize_AILow;
      ReadL_DWork.HILInitialize_AIMinimums[3] = ReadL_P.HILInitialize_AILow;
      ReadL_DWork.HILInitialize_AIMaximums[0] = ReadL_P.HILInitialize_AIHigh;
      ReadL_DWork.HILInitialize_AIMaximums[1] = ReadL_P.HILInitialize_AIHigh;
      ReadL_DWork.HILInitialize_AIMaximums[2] = ReadL_P.HILInitialize_AIHigh;
      ReadL_DWork.HILInitialize_AIMaximums[3] = ReadL_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_AIChannels, 4U,
        &ReadL_DWork.HILInitialize_AIMinimums[0],
        &ReadL_DWork.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    if ((ReadL_P.HILInitialize_AOPStart && !is_switching) ||
        (ReadL_P.HILInitialize_AOPEnter && is_switching)) {
      ReadL_DWork.HILInitialize_AOMinimums[0] = ReadL_P.HILInitialize_AOLow;
      ReadL_DWork.HILInitialize_AOMinimums[1] = ReadL_P.HILInitialize_AOLow;
      ReadL_DWork.HILInitialize_AOMinimums[2] = ReadL_P.HILInitialize_AOLow;
      ReadL_DWork.HILInitialize_AOMinimums[3] = ReadL_P.HILInitialize_AOLow;
      ReadL_DWork.HILInitialize_AOMaximums[0] = ReadL_P.HILInitialize_AOHigh;
      ReadL_DWork.HILInitialize_AOMaximums[1] = ReadL_P.HILInitialize_AOHigh;
      ReadL_DWork.HILInitialize_AOMaximums[2] = ReadL_P.HILInitialize_AOHigh;
      ReadL_DWork.HILInitialize_AOMaximums[3] = ReadL_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_AOChannels, 4U,
        &ReadL_DWork.HILInitialize_AOMinimums[0],
        &ReadL_DWork.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    if ((ReadL_P.HILInitialize_AOStart && !is_switching) ||
        (ReadL_P.HILInitialize_AOEnter && is_switching)) {
      ReadL_DWork.HILInitialize_AOVoltages[0] = ReadL_P.HILInitialize_AOInitial;
      ReadL_DWork.HILInitialize_AOVoltages[1] = ReadL_P.HILInitialize_AOInitial;
      ReadL_DWork.HILInitialize_AOVoltages[2] = ReadL_P.HILInitialize_AOInitial;
      ReadL_DWork.HILInitialize_AOVoltages[3] = ReadL_P.HILInitialize_AOInitial;
      result = hil_write_analog(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_AOChannels, 4U,
        &ReadL_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    if (ReadL_P.HILInitialize_AOReset) {
      ReadL_DWork.HILInitialize_AOVoltages[0] = ReadL_P.HILInitialize_AOWatchdog;
      ReadL_DWork.HILInitialize_AOVoltages[1] = ReadL_P.HILInitialize_AOWatchdog;
      ReadL_DWork.HILInitialize_AOVoltages[2] = ReadL_P.HILInitialize_AOWatchdog;
      ReadL_DWork.HILInitialize_AOVoltages[3] = ReadL_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (ReadL_DWork.HILInitialize_Card, ReadL_P.HILInitialize_AOChannels, 4U,
         &ReadL_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    if ((ReadL_P.HILInitialize_EIPStart && !is_switching) ||
        (ReadL_P.HILInitialize_EIPEnter && is_switching)) {
      ReadL_DWork.HILInitialize_QuadratureModes[0] =
        ReadL_P.HILInitialize_EIQuadrature;
      ReadL_DWork.HILInitialize_QuadratureModes[1] =
        ReadL_P.HILInitialize_EIQuadrature;
      ReadL_DWork.HILInitialize_QuadratureModes[2] =
        ReadL_P.HILInitialize_EIQuadrature;
      ReadL_DWork.HILInitialize_QuadratureModes[3] =
        ReadL_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_EIChannels, 4U, (t_encoder_quadrature_mode *)
        &ReadL_DWork.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }

      ReadL_DWork.HILInitialize_FilterFrequency[0] =
        ReadL_P.HILInitialize_EIFrequency;
      ReadL_DWork.HILInitialize_FilterFrequency[1] =
        ReadL_P.HILInitialize_EIFrequency;
      ReadL_DWork.HILInitialize_FilterFrequency[2] =
        ReadL_P.HILInitialize_EIFrequency;
      ReadL_DWork.HILInitialize_FilterFrequency[3] =
        ReadL_P.HILInitialize_EIFrequency;
      result = hil_set_encoder_filter_frequency(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_EIChannels, 4U,
        &ReadL_DWork.HILInitialize_FilterFrequency[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }

    if ((ReadL_P.HILInitialize_EIStart && !is_switching) ||
        (ReadL_P.HILInitialize_EIEnter && is_switching)) {
      ReadL_DWork.HILInitialize_InitialEICounts[0] =
        ReadL_P.HILInitialize_EIInitial;
      ReadL_DWork.HILInitialize_InitialEICounts[1] =
        ReadL_P.HILInitialize_EIInitial;
      ReadL_DWork.HILInitialize_InitialEICounts[2] =
        ReadL_P.HILInitialize_EIInitial;
      ReadL_DWork.HILInitialize_InitialEICounts[3] =
        ReadL_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(ReadL_DWork.HILInitialize_Card,
        ReadL_P.HILInitialize_EIChannels, 4U,
        &ReadL_DWork.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(ReadL_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for ToFile: '<Root>/To File' */
  {
    char fileName[509] = "ch0.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(ReadL_M, "Error creating .mat file ch0.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch0")) {
      rtmSetErrorStatus(ReadL_M, "Error writing mat file header to file ch0.mat");
      return;
    }

    ReadL_DWork.ToFile_IWORK.Count = 0;
    ReadL_DWork.ToFile_IWORK.Decimation = -1;
    ReadL_DWork.ToFile_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/To File1' */
  {
    char fileName[509] = "ch1.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(ReadL_M, "Error creating .mat file ch1.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch1")) {
      rtmSetErrorStatus(ReadL_M, "Error writing mat file header to file ch1.mat");
      return;
    }

    ReadL_DWork.ToFile1_IWORK.Count = 0;
    ReadL_DWork.ToFile1_IWORK.Decimation = -1;
    ReadL_DWork.ToFile1_PWORK.FilePtr = fp;
  }

  MdlInitialize();
}

void MdlTerminate(void)
{
  ReadL_terminate();
}

RT_MODEL_ReadL *ReadL(void)
{
  ReadL_initialize(1);
  return ReadL_M;
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
