/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_STUDENT_ID,
    IDX_CHAR_VAL_STUDENT_ID,
    IDX_CHAR_CFG_STUDENT_ID,
    HRS_IDX_NB,
};
