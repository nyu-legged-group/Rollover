/*
 * Read_data_optimal.c
 *
 * Code generation for model "Read_data_optimal.mdl".
 *
 * Model version              : 1.50
 * Simulink Coder version : 8.1 (R2011b) 08-Jul-2011
 * C source code generated on : Fri Sep 18 17:27:55 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "Read_data_optimal.h"
#include "Read_data_optimal_private.h"
#include "Read_data_optimal_dt.h"

/* Block signals (auto storage) */
BlockIO_Read_data_optimal Read_data_optimal_B;

/* Block states (auto storage) */
D_Work_Read_data_optimal Read_data_optimal_DWork;

/* Real-time model */
RT_MODEL_Read_data_optimal Read_data_optimal_M_;
RT_MODEL_Read_data_optimal *const Read_data_optimal_M = &Read_data_optimal_M_;

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
void Read_data_optimal_output(int_T tid)
{
  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog' */

  /* S-Function Block: Read_data_optimal/HIL Read Analog (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(Read_data_optimal_DWork.HILInitialize_Card,
      &Read_data_optimal_P.HILReadAnalog_Channels, 1,
      &Read_data_optimal_DWork.HILReadAnalog_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
    }

    Read_data_optimal_B.HILReadAnalog =
      Read_data_optimal_DWork.HILReadAnalog_Buffer;
  }

  /* ToFile: '<Root>/To File' */
  if (!(++Read_data_optimal_DWork.ToFile_IWORK.Decimation % 1) &&
      (Read_data_optimal_DWork.ToFile_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) Read_data_optimal_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      Read_data_optimal_DWork.ToFile_IWORK.Decimation = 0;
      u[0] = Read_data_optimal_M->Timing.t[0];
      u[1] = Read_data_optimal_B.HILReadAnalog;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error writing to MAT-file 9_17_Rollover003_ch3.mat");
        return;
      }

      if (((++Read_data_optimal_DWork.ToFile_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file 9_17_Rollover003_ch3.mat.\n");
      }
    }
  }

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog1' */

  /* S-Function Block: Read_data_optimal/HIL Read Analog1 (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(Read_data_optimal_DWork.HILInitialize_Card,
      &Read_data_optimal_P.HILReadAnalog1_Channels, 1,
      &Read_data_optimal_DWork.HILReadAnalog1_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
    }

    Read_data_optimal_B.HILReadAnalog1 =
      Read_data_optimal_DWork.HILReadAnalog1_Buffer;
  }

  /* ToFile: '<Root>/To File1' */
  if (!(++Read_data_optimal_DWork.ToFile1_IWORK.Decimation % 1) &&
      (Read_data_optimal_DWork.ToFile1_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) Read_data_optimal_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      Read_data_optimal_DWork.ToFile1_IWORK.Decimation = 0;
      u[0] = Read_data_optimal_M->Timing.t[0];
      u[1] = Read_data_optimal_B.HILReadAnalog1;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error writing to MAT-file 9_17_Rollover003_ch2.mat");
        return;
      }

      if (((++Read_data_optimal_DWork.ToFile1_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file 9_17_Rollover003_ch2.mat.\n");
      }
    }
  }

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model update function */
void Read_data_optimal_update(int_T tid)
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
  if (!(++Read_data_optimal_M->Timing.clockTick0)) {
    ++Read_data_optimal_M->Timing.clockTickH0;
  }

  Read_data_optimal_M->Timing.t[0] = Read_data_optimal_M->Timing.clockTick0 *
    Read_data_optimal_M->Timing.stepSize0 +
    Read_data_optimal_M->Timing.clockTickH0 *
    Read_data_optimal_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void Read_data_optimal_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Read_data_optimal_M, 0,
                sizeof(RT_MODEL_Read_data_optimal));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = Read_data_optimal_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    Read_data_optimal_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    Read_data_optimal_M->Timing.sampleTimes =
      (&Read_data_optimal_M->Timing.sampleTimesArray[0]);
    Read_data_optimal_M->Timing.offsetTimes =
      (&Read_data_optimal_M->Timing.offsetTimesArray[0]);

    /* task periods */
    Read_data_optimal_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    Read_data_optimal_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(Read_data_optimal_M, &Read_data_optimal_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = Read_data_optimal_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    Read_data_optimal_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(Read_data_optimal_M, 20.0);
  Read_data_optimal_M->Timing.stepSize0 = 0.001;

  /* external mode info */
  Read_data_optimal_M->Sizes.checksums[0] = (3120410891U);
  Read_data_optimal_M->Sizes.checksums[1] = (2793539452U);
  Read_data_optimal_M->Sizes.checksums[2] = (3076720371U);
  Read_data_optimal_M->Sizes.checksums[3] = (1898348650U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    Read_data_optimal_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Read_data_optimal_M->extModeInfo,
      &Read_data_optimal_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Read_data_optimal_M->extModeInfo,
                        Read_data_optimal_M->Sizes.checksums);
    rteiSetTPtr(Read_data_optimal_M->extModeInfo, rtmGetTPtr(Read_data_optimal_M));
  }

  Read_data_optimal_M->solverInfoPtr = (&Read_data_optimal_M->solverInfo);
  Read_data_optimal_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&Read_data_optimal_M->solverInfo, 0.001);
  rtsiSetSolverMode(&Read_data_optimal_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  Read_data_optimal_M->ModelData.blockIO = ((void *) &Read_data_optimal_B);
  (void) memset(((void *) &Read_data_optimal_B), 0,
                sizeof(BlockIO_Read_data_optimal));

  /* parameters */
  Read_data_optimal_M->ModelData.defaultParam = ((real_T *)&Read_data_optimal_P);

  /* states (dwork) */
  Read_data_optimal_M->Work.dwork = ((void *) &Read_data_optimal_DWork);
  (void) memset((void *)&Read_data_optimal_DWork, 0,
                sizeof(D_Work_Read_data_optimal));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    Read_data_optimal_M->SpecialInfo.mappingInfo = (&dtInfo);
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
void Read_data_optimal_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Read_data_optimal/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    hil_task_stop_all(Read_data_optimal_DWork.HILInitialize_Card);
    hil_monitor_stop_all(Read_data_optimal_DWork.HILInitialize_Card);
    is_switching = false;
    if ((Read_data_optimal_P.HILInitialize_AOTerminate && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_AOExit && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_AOVoltages[0] =
        Read_data_optimal_P.HILInitialize_AOFinal;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[1] =
        Read_data_optimal_P.HILInitialize_AOFinal;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[2] =
        Read_data_optimal_P.HILInitialize_AOFinal;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[3] =
        Read_data_optimal_P.HILInitialize_AOFinal;
      result = hil_write_analog(Read_data_optimal_DWork.HILInitialize_Card,
        Read_data_optimal_P.HILInitialize_AOChannels, 4U,
        &Read_data_optimal_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
      }
    }

    hil_task_delete_all(Read_data_optimal_DWork.HILInitialize_Card);
    hil_monitor_delete_all(Read_data_optimal_DWork.HILInitialize_Card);
    hil_close(Read_data_optimal_DWork.HILInitialize_Card);
    Read_data_optimal_DWork.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) Read_data_optimal_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "9_17_Rollover003_ch3.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error closing MAT-file 9_17_Rollover003_ch3.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error reopening MAT-file 9_17_Rollover003_ch3.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2,
           Read_data_optimal_DWork.ToFile_IWORK.Count, "ch0")) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error writing header for ch0 to MAT-file 9_17_Rollover003_ch3.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error closing MAT-file 9_17_Rollover003_ch3.mat");
        return;
      }

      Read_data_optimal_DWork.ToFile_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (FILE *) Read_data_optimal_DWork.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "9_17_Rollover003_ch2.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error closing MAT-file 9_17_Rollover003_ch2.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error reopening MAT-file 9_17_Rollover003_ch2.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2,
           Read_data_optimal_DWork.ToFile1_IWORK.Count, "ch1")) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error writing header for ch1 to MAT-file 9_17_Rollover003_ch2.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Read_data_optimal_M,
                          "Error closing MAT-file 9_17_Rollover003_ch2.mat");
        return;
      }

      Read_data_optimal_DWork.ToFile1_PWORK.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  Read_data_optimal_output(tid);
}

void MdlUpdate(int_T tid)
{
  Read_data_optimal_update(tid);
}

void MdlInitializeSizes(void)
{
  Read_data_optimal_M->Sizes.numContStates = (0);/* Number of continuous states */
  Read_data_optimal_M->Sizes.numY = (0);/* Number of model outputs */
  Read_data_optimal_M->Sizes.numU = (0);/* Number of model inputs */
  Read_data_optimal_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  Read_data_optimal_M->Sizes.numSampTimes = (1);/* Number of sample times */
  Read_data_optimal_M->Sizes.numBlocks = (7);/* Number of blocks */
  Read_data_optimal_M->Sizes.numBlockIO = (2);/* Number of block outputs */
  Read_data_optimal_M->Sizes.numBlockPrms = (74);/* Sum of parameter "widths" */
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

  /* S-Function Block: Read_data_optimal/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q4", "0", &Read_data_optimal_DWork.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
      return;
    }

    is_switching = false;
    if ((Read_data_optimal_P.HILInitialize_CKPStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_CKPEnter && is_switching)) {
      result = hil_set_clock_mode(Read_data_optimal_DWork.HILInitialize_Card,
        (t_clock *) Read_data_optimal_P.HILInitialize_CKChannels, 2U,
        (t_clock_mode *) Read_data_optimal_P.HILInitialize_CKModes);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    result = hil_watchdog_clear(Read_data_optimal_DWork.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
      return;
    }

    if ((Read_data_optimal_P.HILInitialize_AIPStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_AIPEnter && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_AIMinimums[0] =
        Read_data_optimal_P.HILInitialize_AILow;
      Read_data_optimal_DWork.HILInitialize_AIMinimums[1] =
        Read_data_optimal_P.HILInitialize_AILow;
      Read_data_optimal_DWork.HILInitialize_AIMinimums[2] =
        Read_data_optimal_P.HILInitialize_AILow;
      Read_data_optimal_DWork.HILInitialize_AIMinimums[3] =
        Read_data_optimal_P.HILInitialize_AILow;
      Read_data_optimal_DWork.HILInitialize_AIMaximums[0] =
        Read_data_optimal_P.HILInitialize_AIHigh;
      Read_data_optimal_DWork.HILInitialize_AIMaximums[1] =
        Read_data_optimal_P.HILInitialize_AIHigh;
      Read_data_optimal_DWork.HILInitialize_AIMaximums[2] =
        Read_data_optimal_P.HILInitialize_AIHigh;
      Read_data_optimal_DWork.HILInitialize_AIMaximums[3] =
        Read_data_optimal_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges
        (Read_data_optimal_DWork.HILInitialize_Card,
         Read_data_optimal_P.HILInitialize_AIChannels, 4U,
         &Read_data_optimal_DWork.HILInitialize_AIMinimums[0],
         &Read_data_optimal_DWork.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    if ((Read_data_optimal_P.HILInitialize_AOPStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_AOPEnter && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_AOMinimums[0] =
        Read_data_optimal_P.HILInitialize_AOLow;
      Read_data_optimal_DWork.HILInitialize_AOMinimums[1] =
        Read_data_optimal_P.HILInitialize_AOLow;
      Read_data_optimal_DWork.HILInitialize_AOMinimums[2] =
        Read_data_optimal_P.HILInitialize_AOLow;
      Read_data_optimal_DWork.HILInitialize_AOMinimums[3] =
        Read_data_optimal_P.HILInitialize_AOLow;
      Read_data_optimal_DWork.HILInitialize_AOMaximums[0] =
        Read_data_optimal_P.HILInitialize_AOHigh;
      Read_data_optimal_DWork.HILInitialize_AOMaximums[1] =
        Read_data_optimal_P.HILInitialize_AOHigh;
      Read_data_optimal_DWork.HILInitialize_AOMaximums[2] =
        Read_data_optimal_P.HILInitialize_AOHigh;
      Read_data_optimal_DWork.HILInitialize_AOMaximums[3] =
        Read_data_optimal_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges
        (Read_data_optimal_DWork.HILInitialize_Card,
         Read_data_optimal_P.HILInitialize_AOChannels, 4U,
         &Read_data_optimal_DWork.HILInitialize_AOMinimums[0],
         &Read_data_optimal_DWork.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    if ((Read_data_optimal_P.HILInitialize_AOStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_AOEnter && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_AOVoltages[0] =
        Read_data_optimal_P.HILInitialize_AOInitial;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[1] =
        Read_data_optimal_P.HILInitialize_AOInitial;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[2] =
        Read_data_optimal_P.HILInitialize_AOInitial;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[3] =
        Read_data_optimal_P.HILInitialize_AOInitial;
      result = hil_write_analog(Read_data_optimal_DWork.HILInitialize_Card,
        Read_data_optimal_P.HILInitialize_AOChannels, 4U,
        &Read_data_optimal_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    if (Read_data_optimal_P.HILInitialize_AOReset) {
      Read_data_optimal_DWork.HILInitialize_AOVoltages[0] =
        Read_data_optimal_P.HILInitialize_AOWatchdog;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[1] =
        Read_data_optimal_P.HILInitialize_AOWatchdog;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[2] =
        Read_data_optimal_P.HILInitialize_AOWatchdog;
      Read_data_optimal_DWork.HILInitialize_AOVoltages[3] =
        Read_data_optimal_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (Read_data_optimal_DWork.HILInitialize_Card,
         Read_data_optimal_P.HILInitialize_AOChannels, 4U,
         &Read_data_optimal_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    if ((Read_data_optimal_P.HILInitialize_EIPStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_EIPEnter && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_QuadratureModes[0] =
        Read_data_optimal_P.HILInitialize_EIQuadrature;
      Read_data_optimal_DWork.HILInitialize_QuadratureModes[1] =
        Read_data_optimal_P.HILInitialize_EIQuadrature;
      Read_data_optimal_DWork.HILInitialize_QuadratureModes[2] =
        Read_data_optimal_P.HILInitialize_EIQuadrature;
      Read_data_optimal_DWork.HILInitialize_QuadratureModes[3] =
        Read_data_optimal_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (Read_data_optimal_DWork.HILInitialize_Card,
         Read_data_optimal_P.HILInitialize_EIChannels, 4U,
         (t_encoder_quadrature_mode *)
         &Read_data_optimal_DWork.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }

      Read_data_optimal_DWork.HILInitialize_FilterFrequency[0] =
        Read_data_optimal_P.HILInitialize_EIFrequency;
      Read_data_optimal_DWork.HILInitialize_FilterFrequency[1] =
        Read_data_optimal_P.HILInitialize_EIFrequency;
      Read_data_optimal_DWork.HILInitialize_FilterFrequency[2] =
        Read_data_optimal_P.HILInitialize_EIFrequency;
      Read_data_optimal_DWork.HILInitialize_FilterFrequency[3] =
        Read_data_optimal_P.HILInitialize_EIFrequency;
      result = hil_set_encoder_filter_frequency
        (Read_data_optimal_DWork.HILInitialize_Card,
         Read_data_optimal_P.HILInitialize_EIChannels, 4U,
         &Read_data_optimal_DWork.HILInitialize_FilterFrequency[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }

    if ((Read_data_optimal_P.HILInitialize_EIStart && !is_switching) ||
        (Read_data_optimal_P.HILInitialize_EIEnter && is_switching)) {
      Read_data_optimal_DWork.HILInitialize_InitialEICounts[0] =
        Read_data_optimal_P.HILInitialize_EIInitial;
      Read_data_optimal_DWork.HILInitialize_InitialEICounts[1] =
        Read_data_optimal_P.HILInitialize_EIInitial;
      Read_data_optimal_DWork.HILInitialize_InitialEICounts[2] =
        Read_data_optimal_P.HILInitialize_EIInitial;
      Read_data_optimal_DWork.HILInitialize_InitialEICounts[3] =
        Read_data_optimal_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(Read_data_optimal_DWork.HILInitialize_Card,
        Read_data_optimal_P.HILInitialize_EIChannels, 4U,
        &Read_data_optimal_DWork.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_data_optimal_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for ToFile: '<Root>/To File' */
  {
    char fileName[509] = "9_17_Rollover003_ch3.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(Read_data_optimal_M,
                        "Error creating .mat file 9_17_Rollover003_ch3.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch0")) {
      rtmSetErrorStatus(Read_data_optimal_M,
                        "Error writing mat file header to file 9_17_Rollover003_ch3.mat");
      return;
    }

    Read_data_optimal_DWork.ToFile_IWORK.Count = 0;
    Read_data_optimal_DWork.ToFile_IWORK.Decimation = -1;
    Read_data_optimal_DWork.ToFile_PWORK.FilePtr = fp;
  }

  /* Start for ToFile: '<Root>/To File1' */
  {
    char fileName[509] = "9_17_Rollover003_ch2.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(Read_data_optimal_M,
                        "Error creating .mat file 9_17_Rollover003_ch2.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch1")) {
      rtmSetErrorStatus(Read_data_optimal_M,
                        "Error writing mat file header to file 9_17_Rollover003_ch2.mat");
      return;
    }

    Read_data_optimal_DWork.ToFile1_IWORK.Count = 0;
    Read_data_optimal_DWork.ToFile1_IWORK.Decimation = -1;
    Read_data_optimal_DWork.ToFile1_PWORK.FilePtr = fp;
  }

  MdlInitialize();
}

void MdlTerminate(void)
{
  Read_data_optimal_terminate();
}

RT_MODEL_Read_data_optimal *Read_data_optimal(void)
{
  Read_data_optimal_initialize(1);
  return Read_data_optimal_M;
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
