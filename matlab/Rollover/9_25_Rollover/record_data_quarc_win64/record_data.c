/*
 * record_data.c
 *
 * Code generation for model "record_data.mdl".
 *
 * Model version              : 1.62
 * Simulink Coder version : 8.1 (R2011b) 08-Jul-2011
 * C source code generated on : Fri Sep 25 21:35:18 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "record_data.h"
#include "record_data_private.h"
#include "record_data_dt.h"

/* Block states (auto storage) */
D_Work_record_data record_data_DWork;

/* Real-time model */
RT_MODEL_record_data record_data_M_;
RT_MODEL_record_data *const record_data_M = &record_data_M_;

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
void record_data_output(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_HILReadAnalog;

  /* S-Function (hil_read_analog_block): '<Root>/HIL Read Analog' */

  /* S-Function Block: record_data/HIL Read Analog (hil_read_analog_block) */
  {
    t_error result = hil_read_analog(record_data_DWork.HILInitialize_Card,
      &record_data_P.HILReadAnalog_Channels, 1,
      &record_data_DWork.HILReadAnalog_Buffer);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(record_data_M, _rt_error_message);
    }

    rtb_HILReadAnalog = record_data_DWork.HILReadAnalog_Buffer;
  }

  /* ToFile: '<Root>/To File' */
  if (!(++record_data_DWork.ToFile_IWORK.Decimation % 1) &&
      (record_data_DWork.ToFile_IWORK.Count*2)+1 < 100000000 ) {
    FILE *fp = (FILE *) record_data_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      real_T u[2];
      record_data_DWork.ToFile_IWORK.Decimation = 0;
      u[0] = record_data_M->Timing.t[0];
      u[1] = rtb_HILReadAnalog;
      if (fwrite(u, sizeof(real_T), 2, fp) != 2) {
        rtmSetErrorStatus(record_data_M,
                          "Error writing to MAT-file 9_25_Rollover001_right_sensor.mat");
        return;
      }

      if (((++record_data_DWork.ToFile_IWORK.Count)*2)+1 >= 100000000) {
        (void)fprintf(stdout,
                      "*** The ToFile block will stop logging data before\n"
                      "    the simulation has ended, because it has reached\n"
                      "    the maximum number of elements (100000000)\n"
                      "    allowed in MAT-file 9_25_Rollover001_right_sensor.mat.\n");
      }
    }
  }

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model update function */
void record_data_update(int_T tid)
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
  if (!(++record_data_M->Timing.clockTick0)) {
    ++record_data_M->Timing.clockTickH0;
  }

  record_data_M->Timing.t[0] = record_data_M->Timing.clockTick0 *
    record_data_M->Timing.stepSize0 + record_data_M->Timing.clockTickH0 *
    record_data_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void record_data_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)record_data_M, 0,
                sizeof(RT_MODEL_record_data));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = record_data_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    record_data_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    record_data_M->Timing.sampleTimes = (&record_data_M->
      Timing.sampleTimesArray[0]);
    record_data_M->Timing.offsetTimes = (&record_data_M->
      Timing.offsetTimesArray[0]);

    /* task periods */
    record_data_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    record_data_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(record_data_M, &record_data_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = record_data_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    record_data_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(record_data_M, 20.0);
  record_data_M->Timing.stepSize0 = 0.001;

  /* external mode info */
  record_data_M->Sizes.checksums[0] = (909592701U);
  record_data_M->Sizes.checksums[1] = (3005770592U);
  record_data_M->Sizes.checksums[2] = (3053699554U);
  record_data_M->Sizes.checksums[3] = (2103597544U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    record_data_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(record_data_M->extModeInfo,
      &record_data_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(record_data_M->extModeInfo,
                        record_data_M->Sizes.checksums);
    rteiSetTPtr(record_data_M->extModeInfo, rtmGetTPtr(record_data_M));
  }

  record_data_M->solverInfoPtr = (&record_data_M->solverInfo);
  record_data_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&record_data_M->solverInfo, 0.001);
  rtsiSetSolverMode(&record_data_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* parameters */
  record_data_M->ModelData.defaultParam = ((real_T *)&record_data_P);

  /* states (dwork) */
  record_data_M->Work.dwork = ((void *) &record_data_DWork);
  (void) memset((void *)&record_data_DWork, 0,
                sizeof(D_Work_record_data));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    record_data_M->SpecialInfo.mappingInfo = (&dtInfo);
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
void record_data_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: record_data/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    hil_task_stop_all(record_data_DWork.HILInitialize_Card);
    hil_monitor_stop_all(record_data_DWork.HILInitialize_Card);
    is_switching = false;
    if ((record_data_P.HILInitialize_AOTerminate && !is_switching) ||
        (record_data_P.HILInitialize_AOExit && is_switching)) {
      record_data_DWork.HILInitialize_AOVoltages[0] =
        record_data_P.HILInitialize_AOFinal;
      record_data_DWork.HILInitialize_AOVoltages[1] =
        record_data_P.HILInitialize_AOFinal;
      record_data_DWork.HILInitialize_AOVoltages[2] =
        record_data_P.HILInitialize_AOFinal;
      record_data_DWork.HILInitialize_AOVoltages[3] =
        record_data_P.HILInitialize_AOFinal;
      result = hil_write_analog(record_data_DWork.HILInitialize_Card,
        record_data_P.HILInitialize_AOChannels, 4U,
        &record_data_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
      }
    }

    hil_task_delete_all(record_data_DWork.HILInitialize_Card);
    hil_monitor_delete_all(record_data_DWork.HILInitialize_Card);
    hil_close(record_data_DWork.HILInitialize_Card);
    record_data_DWork.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) record_data_DWork.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "9_25_Rollover001_right_sensor.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(record_data_M,
                          "Error closing MAT-file 9_25_Rollover001_right_sensor.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(record_data_M,
                          "Error reopening MAT-file 9_25_Rollover001_right_sensor.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 2, record_data_DWork.ToFile_IWORK.Count,
           "ch0")) {
        rtmSetErrorStatus(record_data_M,
                          "Error writing header for ch0 to MAT-file 9_25_Rollover001_right_sensor.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(record_data_M,
                          "Error closing MAT-file 9_25_Rollover001_right_sensor.mat");
        return;
      }

      record_data_DWork.ToFile_PWORK.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  record_data_output(tid);
}

void MdlUpdate(int_T tid)
{
  record_data_update(tid);
}

void MdlInitializeSizes(void)
{
  record_data_M->Sizes.numContStates = (0);/* Number of continuous states */
  record_data_M->Sizes.numY = (0);     /* Number of model outputs */
  record_data_M->Sizes.numU = (0);     /* Number of model inputs */
  record_data_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  record_data_M->Sizes.numSampTimes = (1);/* Number of sample times */
  record_data_M->Sizes.numBlocks = (3);/* Number of blocks */
  record_data_M->Sizes.numBlockIO = (0);/* Number of block outputs */
  record_data_M->Sizes.numBlockPrms = (72);/* Sum of parameter "widths" */
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

  /* S-Function Block: record_data/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q4", "0", &record_data_DWork.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(record_data_M, _rt_error_message);
      return;
    }

    is_switching = false;
    if ((record_data_P.HILInitialize_CKPStart && !is_switching) ||
        (record_data_P.HILInitialize_CKPEnter && is_switching)) {
      result = hil_set_clock_mode(record_data_DWork.HILInitialize_Card, (t_clock
        *) record_data_P.HILInitialize_CKChannels, 2U, (t_clock_mode *)
        record_data_P.HILInitialize_CKModes);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    result = hil_watchdog_clear(record_data_DWork.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(record_data_M, _rt_error_message);
      return;
    }

    if ((record_data_P.HILInitialize_AIPStart && !is_switching) ||
        (record_data_P.HILInitialize_AIPEnter && is_switching)) {
      record_data_DWork.HILInitialize_AIMinimums[0] =
        record_data_P.HILInitialize_AILow;
      record_data_DWork.HILInitialize_AIMinimums[1] =
        record_data_P.HILInitialize_AILow;
      record_data_DWork.HILInitialize_AIMinimums[2] =
        record_data_P.HILInitialize_AILow;
      record_data_DWork.HILInitialize_AIMinimums[3] =
        record_data_P.HILInitialize_AILow;
      record_data_DWork.HILInitialize_AIMaximums[0] =
        record_data_P.HILInitialize_AIHigh;
      record_data_DWork.HILInitialize_AIMaximums[1] =
        record_data_P.HILInitialize_AIHigh;
      record_data_DWork.HILInitialize_AIMaximums[2] =
        record_data_P.HILInitialize_AIHigh;
      record_data_DWork.HILInitialize_AIMaximums[3] =
        record_data_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges(record_data_DWork.HILInitialize_Card,
        record_data_P.HILInitialize_AIChannels, 4U,
        &record_data_DWork.HILInitialize_AIMinimums[0],
        &record_data_DWork.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    if ((record_data_P.HILInitialize_AOPStart && !is_switching) ||
        (record_data_P.HILInitialize_AOPEnter && is_switching)) {
      record_data_DWork.HILInitialize_AOMinimums[0] =
        record_data_P.HILInitialize_AOLow;
      record_data_DWork.HILInitialize_AOMinimums[1] =
        record_data_P.HILInitialize_AOLow;
      record_data_DWork.HILInitialize_AOMinimums[2] =
        record_data_P.HILInitialize_AOLow;
      record_data_DWork.HILInitialize_AOMinimums[3] =
        record_data_P.HILInitialize_AOLow;
      record_data_DWork.HILInitialize_AOMaximums[0] =
        record_data_P.HILInitialize_AOHigh;
      record_data_DWork.HILInitialize_AOMaximums[1] =
        record_data_P.HILInitialize_AOHigh;
      record_data_DWork.HILInitialize_AOMaximums[2] =
        record_data_P.HILInitialize_AOHigh;
      record_data_DWork.HILInitialize_AOMaximums[3] =
        record_data_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges(record_data_DWork.HILInitialize_Card,
        record_data_P.HILInitialize_AOChannels, 4U,
        &record_data_DWork.HILInitialize_AOMinimums[0],
        &record_data_DWork.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    if ((record_data_P.HILInitialize_AOStart && !is_switching) ||
        (record_data_P.HILInitialize_AOEnter && is_switching)) {
      record_data_DWork.HILInitialize_AOVoltages[0] =
        record_data_P.HILInitialize_AOInitial;
      record_data_DWork.HILInitialize_AOVoltages[1] =
        record_data_P.HILInitialize_AOInitial;
      record_data_DWork.HILInitialize_AOVoltages[2] =
        record_data_P.HILInitialize_AOInitial;
      record_data_DWork.HILInitialize_AOVoltages[3] =
        record_data_P.HILInitialize_AOInitial;
      result = hil_write_analog(record_data_DWork.HILInitialize_Card,
        record_data_P.HILInitialize_AOChannels, 4U,
        &record_data_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    if (record_data_P.HILInitialize_AOReset) {
      record_data_DWork.HILInitialize_AOVoltages[0] =
        record_data_P.HILInitialize_AOWatchdog;
      record_data_DWork.HILInitialize_AOVoltages[1] =
        record_data_P.HILInitialize_AOWatchdog;
      record_data_DWork.HILInitialize_AOVoltages[2] =
        record_data_P.HILInitialize_AOWatchdog;
      record_data_DWork.HILInitialize_AOVoltages[3] =
        record_data_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (record_data_DWork.HILInitialize_Card,
         record_data_P.HILInitialize_AOChannels, 4U,
         &record_data_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    if ((record_data_P.HILInitialize_EIPStart && !is_switching) ||
        (record_data_P.HILInitialize_EIPEnter && is_switching)) {
      record_data_DWork.HILInitialize_QuadratureModes[0] =
        record_data_P.HILInitialize_EIQuadrature;
      record_data_DWork.HILInitialize_QuadratureModes[1] =
        record_data_P.HILInitialize_EIQuadrature;
      record_data_DWork.HILInitialize_QuadratureModes[2] =
        record_data_P.HILInitialize_EIQuadrature;
      record_data_DWork.HILInitialize_QuadratureModes[3] =
        record_data_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (record_data_DWork.HILInitialize_Card,
         record_data_P.HILInitialize_EIChannels, 4U, (t_encoder_quadrature_mode *)
         &record_data_DWork.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }

      record_data_DWork.HILInitialize_FilterFrequency[0] =
        record_data_P.HILInitialize_EIFrequency;
      record_data_DWork.HILInitialize_FilterFrequency[1] =
        record_data_P.HILInitialize_EIFrequency;
      record_data_DWork.HILInitialize_FilterFrequency[2] =
        record_data_P.HILInitialize_EIFrequency;
      record_data_DWork.HILInitialize_FilterFrequency[3] =
        record_data_P.HILInitialize_EIFrequency;
      result = hil_set_encoder_filter_frequency
        (record_data_DWork.HILInitialize_Card,
         record_data_P.HILInitialize_EIChannels, 4U,
         &record_data_DWork.HILInitialize_FilterFrequency[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }

    if ((record_data_P.HILInitialize_EIStart && !is_switching) ||
        (record_data_P.HILInitialize_EIEnter && is_switching)) {
      record_data_DWork.HILInitialize_InitialEICounts[0] =
        record_data_P.HILInitialize_EIInitial;
      record_data_DWork.HILInitialize_InitialEICounts[1] =
        record_data_P.HILInitialize_EIInitial;
      record_data_DWork.HILInitialize_InitialEICounts[2] =
        record_data_P.HILInitialize_EIInitial;
      record_data_DWork.HILInitialize_InitialEICounts[3] =
        record_data_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(record_data_DWork.HILInitialize_Card,
        record_data_P.HILInitialize_EIChannels, 4U,
        &record_data_DWork.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(record_data_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for ToFile: '<Root>/To File' */
  {
    char fileName[509] = "9_25_Rollover001_right_sensor.mat";
    FILE *fp = (NULL);
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(record_data_M,
                        "Error creating .mat file 9_25_Rollover001_right_sensor.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp,2,0,"ch0")) {
      rtmSetErrorStatus(record_data_M,
                        "Error writing mat file header to file 9_25_Rollover001_right_sensor.mat");
      return;
    }

    record_data_DWork.ToFile_IWORK.Count = 0;
    record_data_DWork.ToFile_IWORK.Decimation = -1;
    record_data_DWork.ToFile_PWORK.FilePtr = fp;
  }

  MdlInitialize();
}

void MdlTerminate(void)
{
  record_data_terminate();
}

RT_MODEL_record_data *record_data(void)
{
  record_data_initialize(1);
  return record_data_M;
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
