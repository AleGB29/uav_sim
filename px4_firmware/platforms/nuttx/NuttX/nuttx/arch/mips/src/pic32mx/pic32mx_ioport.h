/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_ioport.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IOPORT_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IOPORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
/* Offsets relative to PIC32MX_IOPORTn_K1BASE */

#  define PIC32MX_IOPORT_ANSEL_OFFSET     0x0000 /* Analog select register */
#  define PIC32MX_IOPORT_ANSELCLR_OFFSET  0x0004 /* Analog select clear register */
#  define PIC32MX_IOPORT_ANSELSET_OFFSET  0x0008 /* Analog select set register */
#  define PIC32MX_IOPORT_ANSELINV_OFFSET  0x000c /* Analog select invert register */
#  define PIC32MX_IOPORT_TRIS_OFFSET      0x0010 /* Tri-state register */
#  define PIC32MX_IOPORT_TRISCLR_OFFSET   0x0014 /* Tri-state clear register */
#  define PIC32MX_IOPORT_TRISSET_OFFSET   0x0018 /* Tri-state set register */
#  define PIC32MX_IOPORT_TRISINV_OFFSET   0x001c /* Tri-state invert register */
#  define PIC32MX_IOPORT_PORT_OFFSET      0x0020 /* Port register */
#  define PIC32MX_IOPORT_PORTCLR_OFFSET   0x0024 /* Port clear register */
#  define PIC32MX_IOPORT_PORTSET_OFFSET   0x0028 /* Port set register */
#  define PIC32MX_IOPORT_PORTINV_OFFSET   0x002c /* Port invert register */
#  define PIC32MX_IOPORT_LAT_OFFSET       0x0030 /* Port data latch register */
#  define PIC32MX_IOPORT_LATCLR_OFFSET    0x0034 /* Port data latch clear register */
#  define PIC32MX_IOPORT_LATSET_OFFSET    0x0038 /* Port data latch set register */
#  define PIC32MX_IOPORT_LATINV_OFFSET    0x003c /* Port data latch invert register */
#  define PIC32MX_IOPORT_ODC_OFFSET       0x0040 /* Open drain control register */
#  define PIC32MX_IOPORT_ODCCLR_OFFSET    0x0044 /* Open drain control clear register */
#  define PIC32MX_IOPORT_ODCSET_OFFSET    0x0048 /* Open drain control set register */
#  define PIC32MX_IOPORT_ODCINV_OFFSET    0x004c /* Open drain control invert register */

#  define PIC32MX_IOPORT_CNPU_OFFSET      0x0050 /* Change Notification Pull-up register */
#  define PIC32MX_IOPORT_CNPUCLR_OFFSET   0x0054 /* Change Notification Pull-up clear register */
#  define PIC32MX_IOPORT_CNPUSET_OFFSET   0x0058 /* Change Notification Pull-up set register */
#  define PIC32MX_IOPORT_CNPUINV_OFFSET   0x005c /* Change Notification Pull-up invert register */
#  define PIC32MX_IOPORT_CNPD_OFFSET      0x0060 /* Change Notification Pull-down register */
#  define PIC32MX_IOPORT_CNPDCLR_OFFSET   0x0064 /* Change Notification Pull-down clear register */
#  define PIC32MX_IOPORT_CNPDSET_OFFSET   0x0068 /* Change Notification Pull-down set register */
#  define PIC32MX_IOPORT_CNPDINV_OFFSET   0x006c /* Change Notification Pull-down invert register */
#  define PIC32MX_IOPORT_CNCON_OFFSET     0x0070 /* Change Notification Control register */
#  define PIC32MX_IOPORT_CNCONCLR_OFFSET  0x0074 /* Change Notification Control clear register */
#  define PIC32MX_IOPORT_CNCONSET_OFFSET  0x0078 /* Change Notification Control set register */
#  define PIC32MX_IOPORT_CNCONINV_OFFSET  0x007c /* Change Notification Control invert register */
#  define PIC32MX_IOPORT_CNEN_OFFSET      0x0080 /* Change Notification Interrupt Enable register */
#  define PIC32MX_IOPORT_CNENCLR_OFFSET   0x0084 /* Change Notification Interrupt Enable clear register */
#  define PIC32MX_IOPORT_CNENSET_OFFSET   0x0088 /* Change Notification Interrupt Enable set register */
#  define PIC32MX_IOPORT_CNENINV_OFFSET   0x008c /* Change Notification Interrupt Enable *invert register */

#else
/* Offsets relative to PIC32MX_IOPORTn_K1BASE */

#  define PIC32MX_IOPORT_TRIS_OFFSET     0x0000 /* Tri-state register */
#  define PIC32MX_IOPORT_TRISCLR_OFFSET  0x0004 /* Tri-state clear register */
#  define PIC32MX_IOPORT_TRISSET_OFFSET  0x0008 /* Tri-state set register */
#  define PIC32MX_IOPORT_TRISINV_OFFSET  0x000c /* Tri-state invert register */
#  define PIC32MX_IOPORT_PORT_OFFSET     0x0010 /* Port register */
#  define PIC32MX_IOPORT_PORTCLR_OFFSET  0x0014 /* Port clear register */
#  define PIC32MX_IOPORT_PORTSET_OFFSET  0x0018 /* Port set register */
#  define PIC32MX_IOPORT_PORTINV_OFFSET  0x001c /* Port invert register */
#  define PIC32MX_IOPORT_LAT_OFFSET      0x0020 /* Port data latch register */
#  define PIC32MX_IOPORT_LATCLR_OFFSET   0x0024 /* Port data latch clear register */
#  define PIC32MX_IOPORT_LATSET_OFFSET   0x0028 /* Port data latch set register */
#  define PIC32MX_IOPORT_LATINV_OFFSET   0x002c /* Port data latch invert register */
#  define PIC32MX_IOPORT_ODC_OFFSET      0x0030 /* Open drain control register */
#  define PIC32MX_IOPORT_ODCCLR_OFFSET   0x0034 /* Open drain control clear register */
#  define PIC32MX_IOPORT_ODCSET_OFFSET   0x0038 /* Open drain control set register */
#  define PIC32MX_IOPORT_ODCINV_OFFSET   0x003c /* Open drain control invert register */

/* Offsets relative to PIC32MX_IOPORTCN_K1BASE */

#  define PIC32MX_IOPORT_CNCON_OFFSET    0x0000 /* Interrupt-on-change control register */
#  define PIC32MX_IOPORT_CNCONCLR_OFFSET 0x0004 /* Interrupt-on-change control clear register */
#  define PIC32MX_IOPORT_CNCONSET_OFFSET 0x0008 /* Interrupt-on-change control set register */
#  define PIC32MX_IOPORT_CNCONINV_OFFSET 0x000c /* Interrupt-on-change control invert register */
#  define PIC32MX_IOPORT_CNEN_OFFSET     0x0010 /* Input change notification interrupt enable */
#  define PIC32MX_IOPORT_CNENCLR_OFFSET  0x0014 /* Input change notification interrupt enable clear */
#  define PIC32MX_IOPORT_CNENSET_OFFSET  0x0018 /* Input change notification interrupt enable set */
#  define PIC32MX_IOPORT_CNENINV_OFFSET  0x001c /* Input change notification interrupt enable invert */
#  define PIC32MX_IOPORT_CNPUE_OFFSET    0x0020 /* Input change notification pull-up enable */
#  define PIC32MX_IOPORT_CNPUECLR_OFFSET 0x0024 /* Input change notification pull-up enable clear */
#  define PIC32MX_IOPORT_CNPUESET_OFFSET 0x0028 /* Input change notification pull-up enable set */
#  define PIC32MX_IOPORT_CNPUEINV_OFFSET 0x002c /* Input change notification pull-up enable invert */
#endif

/* Register Addresses *******************************************************/

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORT_ANSEL(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ANSEL_OFFSET)
#  define PIC32MX_IOPORT_ANSELCLR(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ANSELCLR_OFFSET)
#  define PIC32MX_IOPORT_ANSELSET(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ANSELSET_OFFSET)
#  define PIC32MX_IOPORT_ANSELINV(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ANSELINV_OFFSET)
#endif

#define PIC32MX_IOPORT_TRIS(n)         (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORT_TRISCLR(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORT_TRISSET(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORT_TRISINV(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORT_PORT(n)         (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORT_PORTCLR(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORT_PORTSET(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORT_PORTINV(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORT_LAT(n)          (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORT_LATCLR(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORT_LATSET(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORT_LATINV(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORT_ODC(n)          (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORT_ODCCLR(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORT_ODCSET(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORT_ODCINV(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_ODCINV_OFFSET)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORT_CNPU(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPU_OFFSET)
#  define PIC32MX_IOPORT_CNPUCLR(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPUCLR_OFFSET)
#  define PIC32MX_IOPORT_CNPUSET(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPUSET_OFFSET)
#  define PIC32MX_IOPORT_CNPUINV(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPUINV_OFFSET)
#  define PIC32MX_IOPORT_CNPD(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPD_OFFSET)
#  define PIC32MX_IOPORT_CNPDCLR(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPDCLR_OFFSET)
#  define PIC32MX_IOPORT_CNPDSET(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPDSET_OFFSET)
#  define PIC32MX_IOPORT_CNPDINV(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNPDINV_OFFSET)
#  define PIC32MX_IOPORT_CNCON(n)      (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNCON_OFFSET)
#  define PIC32MX_IOPORT_CNCONCLR(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNCONCLR_OFFSET)
#  define PIC32MX_IOPORT_CNCONSET(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNCONSET_OFFSET)
#  define PIC32MX_IOPORT_CNCONINV(n)   (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNCONINV_OFFSET)
#  define PIC32MX_IOPORT_CNEN(n)       (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNEN_OFFSET)
#  define PIC32MX_IOPORT_CNENCLR(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNENCLR_OFFSET)
#  define PIC32MX_IOPORT_CNENSET(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNENSET_OFFSET)
#  define PIC32MX_IOPORT_CNENINV(n)    (PIC32MX_IOPORT_K1BASE(n)+PIC32MX_IOPORT_CNENINV_OFFSET)
#endif

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTA_ANSEL        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ANSEL_OFFSET)
#  define PIC32MX_IOPORTA_ANSELCLR     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ANSELCLR_OFFSET)
#  define PIC32MX_IOPORTA_ANSELSET     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ANSELSET_OFFSET)
#  define PIC32MX_IOPORTA_ANSELINV     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ANSELINV_OFFSET)
#endif

#define PIC32MX_IOPORTA_TRIS           (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTA_TRISCLR        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTA_TRISSET        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTA_TRISINV        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTA_PORT           (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTA_PORTCLR        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTA_PORTSET        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTA_PORTINV        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTA_LAT            (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTA_LATCLR         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTA_LATSET         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTA_LATINV         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTA_ODC            (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTA_ODCCLR         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTA_ODCSET         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTA_ODCINV         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTA_CNPU         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPU_OFFSET)
#  define PIC32MX_IOPORTA_CNPUCLR      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPUCLR_OFFSET)
#  define PIC32MX_IOPORTA_CNPUSET      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPUSET_OFFSET)
#  define PIC32MX_IOPORTA_CNPUINV      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPUINV_OFFSET)
#  define PIC32MX_IOPORTA_CNPD         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPD_OFFSET)
#  define PIC32MX_IOPORTA_CNPDCLR      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPDCLR_OFFSET)
#  define PIC32MX_IOPORTA_CNPDSET      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPDSET_OFFSET)
#  define PIC32MX_IOPORTA_CNPDINV      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNPDINV_OFFSET)
#  define PIC32MX_IOPORTA_CNCON        (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNCON_OFFSET)
#  define PIC32MX_IOPORTA_CNCONCLR     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNCONCLR_OFFSET)
#  define PIC32MX_IOPORTA_CNCONSET     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNCONSET_OFFSET)
#  define PIC32MX_IOPORTA_CNCONINV     (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNCONINV_OFFSET)
#  define PIC32MX_IOPORTA_CNEN         (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNEN_OFFSET)
#  define PIC32MX_IOPORTA_CNENCLR      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNENCLR_OFFSET)
#  define PIC32MX_IOPORTA_CNENSET      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNENSET_OFFSET)
#  define PIC32MX_IOPORTA_CNENINV      (PIC32MX_IOPORTA_K1BASE+PIC32MX_IOPORT_CNENINV_OFFSET)
#endif

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTB_ANSEL        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ANSEL_OFFSET)
#  define PIC32MX_IOPORTB_ANSELCLR     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ANSELCLR_OFFSET)
#  define PIC32MX_IOPORTB_ANSELSET     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ANSELSET_OFFSET)
#  define PIC32MX_IOPORTB_ANSELINV     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ANSELINV_OFFSET)
#endif

#define PIC32MX_IOPORTB_TRIS           (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTB_TRISCLR        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTB_TRISSET        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTB_TRISINV        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTB_PORT           (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTB_PORTCLR        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTB_PORTSET        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTB_PORTINV        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTB_LAT            (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTB_LATCLR         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTB_LATSET         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTB_LATINV         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTB_ODC            (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTB_ODCCLR         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTB_ODCSET         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTB_ODCINV         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTB_CNPU         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPU_OFFSET)
#  define PIC32MX_IOPORTB_CNPUCLR      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPUCLR_OFFSET)
#  define PIC32MX_IOPORTB_CNPUSET      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPUSET_OFFSET)
#  define PIC32MX_IOPORTB_CNPUINV      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPUINV_OFFSET)
#  define PIC32MX_IOPORTB_CNPD         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPD_OFFSET)
#  define PIC32MX_IOPORTB_CNPDCLR      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPDCLR_OFFSET)
#  define PIC32MX_IOPORTB_CNPDSET      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPDSET_OFFSET)
#  define PIC32MX_IOPORTB_CNPDINV      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNPDINV_OFFSET)
#  define PIC32MX_IOPORTB_CNCON        (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNCON_OFFSET)
#  define PIC32MX_IOPORTB_CNCONCLR     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNCONCLR_OFFSET)
#  define PIC32MX_IOPORTB_CNCONSET     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNCONSET_OFFSET)
#  define PIC32MX_IOPORTB_CNCONINV     (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNCONINV_OFFSET)
#  define PIC32MX_IOPORTB_CNEN         (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNEN_OFFSET)
#  define PIC32MX_IOPORTB_CNENCLR      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNENCLR_OFFSET)
#  define PIC32MX_IOPORTB_CNENSET      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNENSET_OFFSET)
#  define PIC32MX_IOPORTB_CNENINV      (PIC32MX_IOPORTB_K1BASE+PIC32MX_IOPORT_CNENINV_OFFSET)
#endif

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTC_ANSEL        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ANSEL_OFFSET)
#  define PIC32MX_IOPORTC_ANSELCLR     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ANSELCLR_OFFSET)
#  define PIC32MX_IOPORTC_ANSELSET     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ANSELSET_OFFSET)
#  define PIC32MX_IOPORTC_ANSELINV     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ANSELINV_OFFSET)
#endif

#define PIC32MX_IOPORTC_TRIS           (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTC_TRISCLR        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTC_TRISSET        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTC_TRISINV        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTC_PORT           (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTC_PORTCLR        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTC_PORTSET        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTC_PORTINV        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTC_LAT            (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTC_LATCLR         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTC_LATSET         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTC_LATINV         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTC_ODC            (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTC_ODCCLR         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTC_ODCSET         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTC_ODCINV         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORTC_CNPU         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPU_OFFSET)
#  define PIC32MX_IOPORTC_CNPUCLR      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPUCLR_OFFSET)
#  define PIC32MX_IOPORTC_CNPUSET      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPUSET_OFFSET)
#  define PIC32MX_IOPORTC_CNPUINV      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPUINV_OFFSET)
#  define PIC32MX_IOPORTC_CNPD         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPD_OFFSET)
#  define PIC32MX_IOPORTC_CNPDCLR      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPDCLR_OFFSET)
#  define PIC32MX_IOPORTC_CNPDSET      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPDSET_OFFSET)
#  define PIC32MX_IOPORTC_CNPDINV      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNPDINV_OFFSET)
#  define PIC32MX_IOPORTC_CNCON        (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNCON_OFFSET)
#  define PIC32MX_IOPORTC_CNCONCLR     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNCONCLR_OFFSET)
#  define PIC32MX_IOPORTC_CNCONSET     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNCONSET_OFFSET)
#  define PIC32MX_IOPORTC_CNCONINV     (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNCONINV_OFFSET)
#  define PIC32MX_IOPORTC_CNEN         (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNEN_OFFSET)
#  define PIC32MX_IOPORTC_CNENCLR      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNENCLR_OFFSET)
#  define PIC32MX_IOPORTC_CNENSET      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNENSET_OFFSET)
#  define PIC32MX_IOPORTC_CNENINV      (PIC32MX_IOPORTC_K1BASE+PIC32MX_IOPORT_CNENINV_OFFSET)
#endif

#define PIC32MX_IOPORTD_TRIS           (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTD_TRISCLR        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTD_TRISSET        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTD_TRISINV        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTD_PORT           (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTD_PORTCLR        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTD_PORTSET        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTD_PORTINV        (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTD_LAT            (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTD_LATCLR         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTD_LATSET         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTD_LATINV         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTD_ODC            (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTD_ODCCLR         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTD_ODCSET         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTD_ODCINV         (PIC32MX_IOPORTD_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#define PIC32MX_IOPORTE_TRIS           (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTE_TRISCLR        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTE_TRISSET        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTE_TRISINV        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTE_PORT           (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTE_PORTCLR        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTE_PORTSET        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTE_PORTINV        (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTE_LAT            (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTE_LATCLR         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTE_LATSET         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTE_LATINV         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTE_ODC            (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTE_ODCCLR         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTE_ODCSET         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTE_ODCINV         (PIC32MX_IOPORTE_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#define PIC32MX_IOPORTF_TRIS           (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTF_TRISCLR        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTF_TRISSET        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTF_TRISINV        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTF_PORT           (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTF_PORTCLR        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTF_PORTSET        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTF_PORTINV        (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTF_LAT            (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTF_LATCLR         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTF_LATSET         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTF_LATINV         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTF_ODC            (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTF_ODCCLR         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTF_ODCSET         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTF_ODCINV         (PIC32MX_IOPORTF_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#define PIC32MX_IOPORTG_TRIS           (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_TRIS_OFFSET)
#define PIC32MX_IOPORTG_TRISCLR        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_TRISCLR_OFFSET)
#define PIC32MX_IOPORTG_TRISSET        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_TRISSET_OFFSET)
#define PIC32MX_IOPORTG_TRISINV        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_TRISINV_OFFSET)
#define PIC32MX_IOPORTG_PORT           (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_PORT_OFFSET)
#define PIC32MX_IOPORTG_PORTCLR        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_PORTCLR_OFFSET)
#define PIC32MX_IOPORTG_PORTSET        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_PORTSET_OFFSET)
#define PIC32MX_IOPORTG_PORTINV        (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_PORTINV_OFFSET)
#define PIC32MX_IOPORTG_LAT            (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_LAT_OFFSET)
#define PIC32MX_IOPORTG_LATCLR         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_LATCLR_OFFSET)
#define PIC32MX_IOPORTG_LATSET         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_LATSET_OFFSET)
#define PIC32MX_IOPORTG_LATINV         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_LATINV_OFFSET)
#define PIC32MX_IOPORTG_ODC            (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_ODC_OFFSET)
#define PIC32MX_IOPORTG_ODCCLR         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_ODCCLR_OFFSET)
#define PIC32MX_IOPORTG_ODCSET         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_ODCSET_OFFSET)
#define PIC32MX_IOPORTG_ODCINV         (PIC32MX_IOPORTG_K1BASE+PIC32MX_IOPORT_ODCINV_OFFSET)

#if !defined(CHIP_PIC32MX1) && !defined(CHIP_PIC32MX2)
#  define PIC32MX_IOPORT_CNCON         (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNCON_OFFSET)
#  define PIC32MX_IOPORT_CNCONCLR      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNCONCLR_OFFSET)
#  define PIC32MX_IOPORT_CNCONSET      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNCONSET_OFFSET)
#  define PIC32MX_IOPORT_CNCONINV      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNCONINV_OFFSET)
#  define PIC32MX_IOPORT_CNEN          (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNEN_OFFSET)
#  define PIC32MX_IOPORT_CNENCLR       (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNENCLR_OFFSET)
#  define PIC32MX_IOPORT_CNENSET       (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNENSET_OFFSET)
#  define PIC32MX_IOPORT_CNENINV       (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNENINV_OFFSET)
#  define PIC32MX_IOPORT_CNPUE         (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNPUE_OFFSET)
#  define PIC32MX_IOPORT_CNPUECLR      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNPUECLR_OFFSET)
#  define PIC32MX_IOPORT_CNPUESET      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNPUESET_OFFSET)
#  define PIC32MX_IOPORT_CNPUEINV      (PIC32MX_IOPORTCN_K1BASE+PIC32MX_IOPORT_CNPUEINV_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

/* Analog select register */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
#  define IOPORT_ANSEL(n)              (1 << (n)) /* Bits 0-15: Analog select */
#endif

/* Tri-state register */

#define IOPORT_TRIS(n)                 (1 << (n)) /* Bits 0-15: 1: Input 0: Output */

/* Port register */

#define IOPORT_PORT(n)                 (1 << (n)) /* Bits 0-15: Pin value */

/* Port data latch register */

#define IOPORT_LAT(n)                  (1 << (n)) /* Bits 0-15: Port latch value */

/* Open drain control register */

#define IOPORT_ODC(n)                  (1 << (n)) /* Bits 0-15: 1: OD output enabled, 0: Disabled */

#if defined(CHIP_PIC32MX1) || defined(CHIP_PIC32MX2)
/* Change Notification Pull-up register */

#  define IOPORT_CNPU(n)               (1 << (n)) /* Bits 0:15: 1=Pull-up enabled */

/* Change Notification Pull-down register */

#  define IOPORT_CNPD(n)               (1 << (n)) /* Bits 0:15: 1=Pull-down enabled */

/* Change Notification Control register */

#  define IOPORT_CNCON_SIDL            (1 << 13) /* Bit 13: Stop in idle mode */
#  define IOPORT_CNCON_ON              (1 << 15) /* Bit 15: Change notice module enable */

/* Change Notification Interrupt Enable register */

#  define IOPORT_CNEN(n)               (1 << (n)) /* Bits 0:15: 1=Interrupt enabled */

#else
  /* Interrupt-on-change control register */

#  define IOPORT_CNCON_SIDL            (1 << 13) /* Bit 13: Stop in idle mode */
#  define IOPORT_CNCON_FRZ             (1 << 14) /* Bit 14: Freeze in debug exception mode */
#  define IOPORT_CNCON_ON              (1 << 15) /* Bit 15: Change notice module enable */

  /* Input change notification interrupt enable */

#  define IOPORT_CNEN(n)               (1 << (n)) /* Bits 0-18/21: Port pin input change notice enabled */

  /* Input change notification pull-up enable */

#  define IOPORT_CNPUE(n)              (1 << (n)) /* Bits 0-18/21: Port pin pull-up enabled */

#  if defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4)
#    define IOPORT_CN_ALL              0x0007ffff /* Bits 0-18 */
#    define IOPORT_NUMCN               19
#  elif defined(CHIP_PIC32MX5) || defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
#    define IOPORT_CN_ALL              0x003fffff /* Bits 0-21 */
#    define IOPORT_NUMCN               22
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_IOPORT_H */
