MEMORY
{
   /* Flash sectors */
   CMBANK0_RESETISR : origin = 0x00200000, length = 0x00000008 /* Boot to Flash Entry Point */
   CMBANK0_SECTOR0  : origin = 0x00200008, length = 0x00003FF7
   CMBANK0_SECTOR1  : origin = 0x00204000, length = 0x00004000
   CMBANK0_SECTOR2  : origin = 0x00208000, length = 0x00004000
   CMBANK0_SECTOR3  : origin = 0x0020C000, length = 0x00004000
   CMBANK0_SECTOR4  : origin = 0x00210000, length = 0x00010000
   CMBANK0_SECTOR5  : origin = 0x00220000, length = 0x00010000
   CMBANK0_SECTOR6  : origin = 0x00230000, length = 0x00010000
   CMBANK0_SECTOR7  : origin = 0x00240000, length = 0x00010000
   CMBANK0_SECTOR8  : origin = 0x00250000, length = 0x00010000
   CMBANK0_SECTOR9  : origin = 0x00260000, length = 0x00010000
   CMBANK0_SECTOR10 : origin = 0x00270000, length = 0x00004000
   CMBANK0_SECTOR11 : origin = 0x00274000, length = 0x00004000
   CMBANK0_SECTOR12 : origin = 0x00278000, length = 0x00004000
   CMBANK0_SECTOR13 : origin = 0x0027C000, length = 0x00004000

   C1RAM            : origin = 0x1FFFC000, length = 0x00001FFF
   C0RAM            : origin = 0x1FFFE000, length = 0x00001FFF

   BOOT_RSVD        : origin = 0x20000000, length = 0x00000800 /* Part of S0, BOOT rom will use this for stack */
   SRAM             : origin = 0x20000800, length = 0x0000F7FF
   E0RAM            : origin = 0x20010000, length = 0x00003FFF

   CPU1TOCMMSGRAM0  : origin = 0x20080000, length = 0x00000800
   CPU1TOCMMSGRAM1  : origin = 0x20080800, length = 0x00000800
   CMTOCPU1MSGRAM0  : origin = 0x20082000, length = 0x00000800
   CMTOCPU1MSGRAM1  : origin = 0x20082800, length = 0x00000800
   CPU2TOCMMSGRAM0  : origin = 0x20084000, length = 0x00000800
   CPU2TOCMMSGRAM1  : origin = 0x20084800, length = 0x00000800
   CMTOCPU2MSGRAM0  : origin = 0x20086000, length = 0x00000800
   CMTOCPU2MSGRAM1  : origin = 0x20086800, length = 0x00000800
}

SECTIONS
{
  .resetisr        : > CMBANK0_RESETISR,FILL=0
   .vftable         : > CMBANK0_SECTOR3,ALIGN(8) /* Application placed vector table in Flash, this shall not be collided with any other flash sectors*/
  .vtable          : > E0RAM             /* Application placed vector table in RAM*/


   .text            : >> CMBANK0_SECTOR2,ALIGN(8)
   .cinit           : > CMBANK0_SECTOR1,ALIGN(8)
   .pinit           : >> CMBANK0_SECTOR0 | CMBANK0_SECTOR1,ALIGN(8)
   .switch          : >> CMBANK0_SECTOR0 | CMBANK0_SECTOR1,ALIGN(8)
   .sysmem          : > E0RAM
   .binit 			: > CMBANK0_SECTOR1

   .stack           : > C0RAM
   .ebss            : > C0RAM
   .econst          : >> CMBANK0_SECTOR0 | CMBANK0_SECTOR1 ,ALIGN(8)
   .esysmem         : > C0RAM
   .data            : > E0RAM
   .bss             : > E0RAM
   .const           : {} LOAD = CMBANK0_SECTOR0 | CMBANK0_SECTOR1 |CMBANK0_SECTOR2 ,ALIGN(8)
   							RUN = E0RAM,
   							LOAD_START(constLoadStart),
   							LOAD_SIZE(constLoadSize),
   							LOAD_END(constLoadEnd),
   							RUN_START(constRunStart),
   							RUN_SIZE(constRunSize),
   							RUN_END(constRunEnd)

    MSGRAM_CM_TO_CPU1 : > CMTOCPU1MSGRAM0, type=NOINIT
    MSGRAM_CM_TO_CPU2 : > CMTOCPU2MSGRAM0, type=NOINIT
    MSGRAM_CPU1_TO_CM : > CPU1TOCMMSGRAM0, type=NOINIT
    MSGRAM_CPU2_TO_CM : > CPU2TOCMMSGRAM0, type=NOINIT

    .TI.ramfunc : {} LOAD = CMBANK0_SECTOR0 | CMBANK0_SECTOR1 | CMBANK0_SECTOR2 ,
                           RUN = E0RAM,
                           LOAD_START(RamfuncsLoadStart),
                           LOAD_SIZE(RamfuncsLoadSize),
                           LOAD_END(RamfuncsLoadEnd),
                           RUN_START(RamfuncsRunStart),
                           RUN_SIZE(RamfuncsRunSize),
                           RUN_END(RamfuncsRunEnd),
                           ALIGN(8)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
