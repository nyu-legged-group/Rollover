/*
 * Read.c
 *
 * Code generation for model "Read.mdl".
 *
 * Model version              : 1.17
 * Simulink Coder version : 8.1 (R2011b) 08-Jul-2011
 * C source code generated on : Wed Sep 09 17:40:07 2020
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include "Read.h"
#include "Read_private.h"
#include "Read_dt.h"

/* Block signals (auto storage) */
BlockIO_Read Read_B;

/* Block states (auto storage) */
D_Work_Read Read_DWork;

/* Real-time model */
RT_MODEL_Read Read_M_;
RT_MODEL_Read *const Read_M = &Read_M_;

/* Model output function */
void Read_output(int_T tid)
{
  /* S-Function (hil_read_analog_buffer_block): '<Root>/HIL Read Analog Buffer' */

  /* S-Function Block: Read/HIL Read Analog Buffer (hil_read_analog_buffer_block) */
  {
    t_error result = hil_read_analog_buffer(Read_DWork.HILInitialize_Card,
      (t_clock) Read_P.HILReadAnalogBuffer_Clock,
      Read_P.HILReadAnalogBuffer_Frequency,
      2000, &Read_P.HILReadAnalogBuffer_Channels, 1,
      &Read_DWork.HILReadAnalogBuffer_Buffer[0]);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_M, _rt_error_message);
    }

    {
      int_T i1;
      real_T *dw_Buffer = &Read_DWork.HILReadAnalogBuffer_Buffer[0];
      real_T *y0 = Read_B.HILReadAnalogBuffer;
      for (i1=0; i1 < 2000; i1++) {
        y0[i1] = dw_Buffer[i1 * 1 + 0];
      }
    }
  }

  /* S-Function (hil_read_analog_buffer_block): '<Root>/HIL Read Analog Buffer1' */

  /* S-Function Block: Read/HIL Read Analog Buffer1 (hil_read_analog_buffer_block) */
  {
    t_error result = hil_read_analog_buffer(Read_DWork.HILInitialize_Card,
      (t_clock) Read_P.HILReadAnalogBuffer1_Clock,
      Read_P.HILReadAnalogBuffer1_Frequency,
      2000, &Read_P.HILReadAnalogBuffer1_Channels, 1,
      &Read_DWork.HILReadAnalogBuffer1_Buffer[0]);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_M, _rt_error_message);
    }

    {
      int_T i1;
      real_T *dw_Buffer = &Read_DWork.HILReadAnalogBuffer1_Buffer[0];
      real_T *y0 = Read_B.HILReadAnalogBuffer1;
      for (i1=0; i1 < 2000; i1++) {
        y0[i1] = dw_Buffer[i1 * 1 + 0];
      }
    }
  }

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model update function */
void Read_update(int_T tid)
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
  if (!(++Read_M->Timing.clockTick0)) {
    ++Read_M->Timing.clockTickH0;
  }

  Read_M->Timing.t[0] = Read_M->Timing.clockTick0 * Read_M->Timing.stepSize0 +
    Read_M->Timing.clockTickH0 * Read_M->Timing.stepSize0 * 4294967296.0;

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void Read_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Read_M, 0,
                sizeof(RT_MODEL_Read));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = Read_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    Read_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    Read_M->Timing.sampleTimes = (&Read_M->Timing.sampleTimesArray[0]);
    Read_M->Timing.offsetTimes = (&Read_M->Timing.offsetTimesArray[0]);

    /* task periods */
    Read_M->Timing.sampleTimes[0] = (0.001);

    /* task offsets */
    Read_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(Read_M, &Read_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = Read_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    Read_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(Read_M, -1);
  Read_M->Timing.stepSize0 = 0.001;

  /* external mode info */
  Read_M->Sizes.checksums[0] = (2209353128U);
  Read_M->Sizes.checksums[1] = (1879282233U);
  Read_M->Sizes.checksums[2] = (4155993715U);
  Read_M->Sizes.checksums[3] = (349446549U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    Read_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Read_M->extModeInfo,
      &Read_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Read_M->extModeInfo, Read_M->Sizes.checksums);
    rteiSetTPtr(Read_M->extModeInfo, rtmGetTPtr(Read_M));
  }

  Read_M->solverInfoPtr = (&Read_M->solverInfo);
  Read_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&Read_M->solverInfo, 0.001);
  rtsiSetSolverMode(&Read_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  Read_M->ModelData.blockIO = ((void *) &Read_B);
  (void) memset(((void *) &Read_B), 0,
                sizeof(BlockIO_Read));

  /* parameters */
  Read_M->ModelData.defaultParam = ((real_T *)&Read_P);

  /* states (dwork) */
  Read_M->Work.dwork = ((void *) &Read_DWork);
  (void) memset((void *)&Read_DWork, 0,
                sizeof(D_Work_Read));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    Read_M->SpecialInfo.mappingInfo = (&dtInfo);
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
void Read_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Read/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    hil_task_stop_all(Read_DWork.HILInitialize_Card);
    hil_monitor_stop_all(Read_DWork.HILInitialize_Card);
    is_switching = false;
    if ((Read_P.HILInitialize_AOTerminate && !is_switching) ||
        (Read_P.HILInitialize_AOExit && is_switching)) {
      Read_DWork.HILInitialize_AOVoltages[0] = Read_P.HILInitialize_AOFinal;
      Read_DWork.HILInitialize_AOVoltages[1] = Read_P.HILInitialize_AOFinal;
      Read_DWork.HILInitialize_AOVoltages[2] = Read_P.HILInitialize_AOFinal;
      Read_DWork.HILInitialize_AOVoltages[3] = Read_P.HILInitialize_AOFinal;
      result = hil_write_analog(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_AOChannels, 4U,
        &Read_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
      }
    }

    hil_task_delete_all(Read_DWork.HILInitialize_Card);
    hil_monitor_delete_all(Read_DWork.HILInitialize_Card);
    hil_close(Read_DWork.HILInitialize_Card);
    Read_DWork.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  Read_output(tid);
}

void MdlUpdate(int_T tid)
{
  Read_update(tid);
}

void MdlInitializeSizes(void)
{
  Read_M->Sizes.numContStates = (0);   /* Number of continuous states */
  Read_M->Sizes.numY = (0);            /* Number of model outputs */
  Read_M->Sizes.numU = (0);            /* Number of model inputs */
  Read_M->Sizes.sysDirFeedThru = (0);  /* The model is not direct feedthrough */
  Read_M->Sizes.numSampTimes = (1);    /* Number of sample times */
  Read_M->Sizes.numBlocks = (5);       /* Number of blocks */
  Read_M->Sizes.numBlockIO = (2);      /* Number of block outputs */
  Read_M->Sizes.numBlockPrms = (78);   /* Sum of parameter "widths" */
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

  /* S-Function Block: Read/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("q4", "0", &Read_DWork.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_M, _rt_error_message);
      return;
    }

    is_switching = false;
    if ((Read_P.HILInitialize_CKPStart && !is_switching) ||
        (Read_P.HILInitialize_CKPEnter && is_switching)) {
      result = hil_set_clock_mode(Read_DWork.HILInitialize_Card, (t_clock *)
        Read_P.HILInitialize_CKChannels, 2U, (t_clock_mode *)
        Read_P.HILInitialize_CKModes);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    result = hil_watchdog_clear(Read_DWork.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Read_M, _rt_error_message);
      return;
    }

    if ((Read_P.HILInitialize_AIPStart && !is_switching) ||
        (Read_P.HILInitialize_AIPEnter && is_switching)) {
      Read_DWork.HILInitialize_AIMinimums[0] = Read_P.HILInitialize_AILow;
      Read_DWork.HILInitialize_AIMinimums[1] = Read_P.HILInitialize_AILow;
      Read_DWork.HILInitialize_AIMinimums[2] = Read_P.HILInitialize_AILow;
      Read_DWork.HILInitialize_AIMinimums[3] = Read_P.HILInitialize_AILow;
      Read_DWork.HILInitialize_AIMaximums[0] = Read_P.HILInitialize_AIHigh;
      Read_DWork.HILInitialize_AIMaximums[1] = Read_P.HILInitialize_AIHigh;
      Read_DWork.HILInitialize_AIMaximums[2] = Read_P.HILInitialize_AIHigh;
      Read_DWork.HILInitialize_AIMaximums[3] = Read_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_AIChannels, 4U,
        &Read_DWork.HILInitialize_AIMinimums[0],
        &Read_DWork.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    if ((Read_P.HILInitialize_AOPStart && !is_switching) ||
        (Read_P.HILInitialize_AOPEnter && is_switching)) {
      Read_DWork.HILInitialize_AOMinimums[0] = Read_P.HILInitialize_AOLow;
      Read_DWork.HILInitialize_AOMinimums[1] = Read_P.HILInitialize_AOLow;
      Read_DWork.HILInitialize_AOMinimums[2] = Read_P.HILInitialize_AOLow;
      Read_DWork.HILInitialize_AOMinimums[3] = Read_P.HILInitialize_AOLow;
      Read_DWork.HILInitialize_AOMaximums[0] = Read_P.HILInitialize_AOHigh;
      Read_DWork.HILInitialize_AOMaximums[1] = Read_P.HILInitialize_AOHigh;
      Read_DWork.HILInitialize_AOMaximums[2] = Read_P.HILInitialize_AOHigh;
      Read_DWork.HILInitialize_AOMaximums[3] = Read_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_AOChannels, 4U,
        &Read_DWork.HILInitialize_AOMinimums[0],
        &Read_DWork.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    if ((Read_P.HILInitialize_AOStart && !is_switching) ||
        (Read_P.HILInitialize_AOEnter && is_switching)) {
      Read_DWork.HILInitialize_AOVoltages[0] = Read_P.HILInitialize_AOInitial;
      Read_DWork.HILInitialize_AOVoltages[1] = Read_P.HILInitialize_AOInitial;
      Read_DWork.HILInitialize_AOVoltages[2] = Read_P.HILInitialize_AOInitial;
      Read_DWork.HILInitialize_AOVoltages[3] = Read_P.HILInitialize_AOInitial;
      result = hil_write_analog(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_AOChannels, 4U,
        &Read_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    if (Read_P.HILInitialize_AOReset) {
      Read_DWork.HILInitialize_AOVoltages[0] = Read_P.HILInitialize_AOWatchdog;
      Read_DWork.HILInitialize_AOVoltages[1] = Read_P.HILInitialize_AOWatchdog;
      Read_DWork.HILInitialize_AOVoltages[2] = Read_P.HILInitialize_AOWatchdog;
      Read_DWork.HILInitialize_AOVoltages[3] = Read_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (Read_DWork.HILInitialize_Card, Read_P.HILInitialize_AOChannels, 4U,
         &Read_DWork.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    if ((Read_P.HILInitialize_EIPStart && !is_switching) ||
        (Read_P.HILInitialize_EIPEnter && is_switching)) {
      Read_DWork.HILInitialize_QuadratureModes[0] =
        Read_P.HILInitialize_EIQuadrature;
      Read_DWork.HILInitialize_QuadratureModes[1] =
        Read_P.HILInitialize_EIQuadrature;
      Read_DWork.HILInitialize_QuadratureModes[2] =
        Read_P.HILInitialize_EIQuadrature;
      Read_DWork.HILInitialize_QuadratureModes[3] =
        Read_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_EIChannels, 4U, (t_encoder_quadrature_mode *)
        &Read_DWork.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }

      Read_DWork.HILInitialize_FilterFrequency[0] =
        Read_P.HILInitialize_EIFrequency;
      Read_DWork.HILInitialize_FilterFrequency[1] =
        Read_P.HILInitialize_EIFrequency;
      Read_DWork.HILInitialize_FilterFrequency[2] =
        Read_P.HILInitialize_EIFrequency;
      Read_DWork.HILInitialize_FilterFrequency[3] =
        Read_P.HILInitialize_EIFrequency;
      result = hil_set_encoder_filter_frequency(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_EIChannels, 4U,
        &Read_DWork.HILInitialize_FilterFrequency[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }

    if ((Read_P.HILInitialize_EIStart && !is_switching) ||
        (Read_P.HILInitialize_EIEnter && is_switching)) {
      Read_DWork.HILInitialize_InitialEICounts[0] =
        Read_P.HILInitialize_EIInitial;
      Read_DWork.HILInitialize_InitialEICounts[1] =
        Read_P.HILInitialize_EIInitial;
      Read_DWork.HILInitialize_InitialEICounts[2] =
        Read_P.HILInitialize_EIInitial;
      Read_DWork.HILInitialize_InitialEICounts[3] =
        Read_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(Read_DWork.HILInitialize_Card,
        Read_P.HILInitialize_EIChannels, 4U,
        &Read_DWork.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Read_M, _rt_error_message);
        return;
      }
    }
  }

  MdlInitialize();
}

void MdlTerminate(void)
{
  Read_terminate();
}

RT_MODEL_Read *Read(void)
{
  Read_initialize(1);
  return Read_M;
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
