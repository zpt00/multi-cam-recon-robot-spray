! Cell Digital Inputs (Memory-Mapped)							
! Cell Digital Inputs (Model A I/O - 5 or 10 Slot Rack)
!2	1	15	1	1	1	15	1	Clear Ques	PAVRSHIO	clr_ques_di
! Cell Digital Inputs (PLC I/O)
PAVRSHIO  strt_sty_di                             9
PAVRSHIO  prd_abort_di                            10
PAVRSHIO  tShioCmos[1].cancel_di                  12
PAVRSHIO  tShioCmos[1].continue_di                13
PAVRSHIO  clr_ques_di                             15
PAVRSHIO  tShioInit[1].iDataBeg                   17
PAVRSHIO  tShioInit[1].iDataEnd                   26
PAVRSHIO  tShioInit[1].iColInitBeg                27
PAVRSHIO  tShioInit[1].iColInitEnd                31
PAVRSHIO  tShioInit[1].read_sty_di                32
! Cell Digital Inputs (PDI IO)
! Cell Digital Inputs (Robot I/O mapped to RDI[1])
! Cell Group Inputs (Memory-Mapped)
! Cell Group Inputs (Model A I/O - 5 or 10 Slot Rack)
! Cell Group Inputs (PLC I/O)
PAVRSHIO  sty_code_gi                             1
PAVRSHIO  colr_code_gi                            2
! Cell Group Inputs (PDI IO)
! Cell Digital Outputs (Memory-Mapped)
! Cell Digital Outputs (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Outputs (PLC I/O)
PAVRSHIO  tShioCmos[1].prd_mode_do                9
PAVRSHIO  tShioCmos[1].job_actv_do                10
PAVRSHIO  tShioCmos[1].sty_ready_do               12
PAVRSHIO  rob_home_do                             13
PAVRSHIO  maj_flt_do                              14
PAVRSHIO  sh_runing_do                            15
PAVRSHIO  tShioCmos[1].wait_plc_do                19
PAVRSHIO  tShioCmos[1].mot_nrec_do                20
! Cell Digital Outputs (PDI IO)
! uncomment this line (and comment out the followning line) if you don't want to
!13	2	17	48	1	2	17	1	Gun 1 Pilot	PAVRGUNS	tApplEq[1].ApplNum[1].gun_out_num
! Cell Digital Outputs (PDI IO but mapped to RDO[1])
! Cell Group Outputs (Memory-Mapped)
! Cell Group Outputs (Model A I/O - 5 Slot Rack)
! Cell Group Outputs (Model A I/O - 10 Slot Rack)
! Cell Group Outputs (PLC I/O)
! Cell Digital Inputs (Model A I/O - 5 or 10 Slot Rack)
! Process Digital Inputs (Memory-Mapped)
! Process Digital Inputs (Model A I/O - 5 Slot Rack)
! Process Digital Inputs (Model A I/O - 10 Slot Rack)
! Process Analog Inputs (Memory-Mapped)
! Process Analog Inputs (Model A I/O - 5 Slot Rack)
! Process Analog Inputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Memory-Mapped)
! Process Digital Outputs (Model A I/O - 5 Slot Rack)
! Process Digital Outputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Ethernet IP but mapped to RDO[1])
! Process Digital Outputs (Robot I/O mapped to RDO[1])
! Process Analog Outputs (Memory-Mapped)
! Process Analog Outputs (Model A I/O - 5 Slot Rack)
! Process Analog Outputs (Model A I/O - 10 Slot Rack)
! Cell Digital Inputs (Memory-Mapped)
! Note: Just always assign reserved I/O ports for any Cell I/O Type for unsed I/
! Cell Digital Outputs (Memory-Mapped)
! Note: Just always assign reserved I/O ports for any Cell I/O Type for unsed I/
*WARN*[PAVROPTN]byOptn[1].byOp[110] = 0
user data is 1 at line 1315 column 7
! PaintTool Standard I/O Variables (Any Type)
!Set default color name once
PAVRSYSC  sys_colors[1].color_desc                default
!The following must be set for Standard I/O
!force_use = 48
PAVROPTN  byOptn[1].byOp[48]                      1
!rob_main_set = 84
PAVROPTN  byOptn[1].byOp[84]                      1
!job_ent_tp = 3
PAVROPTN  byOptn[1].byOp[3]                       1
! PaintTool Standard I/O Variables (Any Type)
*SYSTEM*  $TG_MX_EVUS                             200
! Cell Digital Outputs Function Instruction (Memory-Mapped)
! Cell Digital Outputs Function Instruction (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Outputs Function Instruction (PLC I/O)
PAVRFUNC  func_array[1].dout                      25
PAVRFUNC  func_array[2].dout                      26
PAVRFUNC  func_array[3].dout                      27
PAVRFUNC  func_array[4].dout                      28
PAVRFUNC  func_array[5].dout                      29
PAVRFUNC  func_array[6].dout                      30
PAVRFUNC  func_array[7].dout                      31
PAVRFUNC  func_array[8].dout                      32
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 1434 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 1449 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 1458 column 4
! Process Analog Outputs Applicator Parameter Option (Memory-Mapped)										
PAVRPARM  tParmEq[1].con_prm_desc[1].app_otpt_num 1
PAVRPARM  tParmEq[1].con_prm_desc[2].app_otpt_num 2
PAVRPARM  tParmEq[1].con_prm_desc[3].app_otpt_num 3
PAVRPARM  tParmEq[1].con_prm_desc[4].app_otpt_num 4
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 5 Slot Rack 
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 5 Slot Rack 
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 10 Slot Rack
! Process Analog Outputs Applicator Parameter Option (Ethernet IP)
! Applicator Parameter Option Variables (Any Type)
! Applicator Parameter Option Variables (Any Type)
PAVRPARM  estat_type                              3
PAVRPARM  tParmEq[1].con_prm_desc[1].app_con_type 3
PAVRPARM  tParmEq[1].con_prm_desc[1].cal_date     0
PAVRPARM  tParmEq[1].con_prm_desc[1].cal_source   5
PAVRPARM  tParmEq[1].con_prm_desc[1].cal_status   3
PAVRPARM  tParmEq[1].con_prm_desc[1].cal_units    cc/min
PAVRPARM  tParmEq[1].con_prm_desc[1].app_con_max  1500
PAVRPARM  tParmEq[1].con_prm_desc[1].app_con_min  0
PAVRPARM  tParmEq[1].con_prm_desc[1].app_cal_size 10
PAVRPARM  tParmEq[1].con_prm_desc[1].app_min_setp TRUE
PAVRPARM  tParmEq[1].con_prm_desc[1].app_frst_bit 1
PAVRPARM  tParmEq[1].con_prm_desc[1].app_otpt_bts 12
PAVRPARM  tParmEq[1].con_prm_desc[1].app_max_out  8000
PAVRPARM  tParmEq[1].con_prm_desc[1].app_min_out  1600
PAVRPARM  tParmEq[1].con_prm_desc[1].app_strob_ln 0
PAVRPARM  tParmEq[1].con_prm_desc[1].app_strb_out 0
PAVRPARM  tParmEq[1].con_prm_desc[1].app_con_nam  FF
PAVRPARM  tParmEq[1].con_prm_desc[1].app_con_titl Fluid Flow
PAVRPARM  tParmEq[1].con_prm_desc[1].chg_date     0
PAVRPARM  tParmEq[1].con_prm_des3[1].app_cntr_typ 1
PAVRPARM  tParmEq[1].con_prm_des3[1].app_cntr_st  0
PAVRPARM  tParmEq[1].con_prm_des3[1].app_echo_scl 1
! 
PAVRPARM  tParmEq[1].con_prm_desc[2].app_con_type 3
PAVRPARM  tParmEq[1].con_prm_desc[2].cal_date     0
PAVRPARM  tParmEq[1].con_prm_desc[2].cal_source   0
PAVRPARM  tParmEq[1].con_prm_desc[2].cal_status   3
PAVRPARM  tParmEq[1].con_prm_desc[2].app_con_max  150
PAVRPARM  tParmEq[1].con_prm_desc[2].app_con_min  0
PAVRPARM  tParmEq[1].con_prm_desc[2].app_cal_size 10
PAVRPARM  tParmEq[1].con_prm_desc[2].app_min_setp TRUE
PAVRPARM  tParmEq[1].con_prm_desc[2].app_frst_bit 2
PAVRPARM  tParmEq[1].con_prm_desc[2].app_otpt_bts 12
PAVRPARM  tParmEq[1].con_prm_desc[2].app_max_out  8000
PAVRPARM  tParmEq[1].con_prm_desc[2].app_min_out  1600
PAVRPARM  tParmEq[1].con_prm_desc[2].app_strob_ln 0
PAVRPARM  tParmEq[1].con_prm_desc[2].app_strb_out 0
PAVRPARM  tParmEq[1].con_prm_desc[2].chg_date     0
PAVRPARM  tParmEq[1].con_prm_des3[2].app_cntr_typ 1
PAVRPARM  tParmEq[1].con_prm_des3[2].app_cntr_st  0
PAVRPARM  tParmEq[1].con_prm_des3[2].app_echo_scl 10
!
PAVRPARM  tParmEq[1].con_prm_desc[3].app_con_type 3
PAVRPARM  tParmEq[1].con_prm_desc[3].cal_date     0
PAVRPARM  tParmEq[1].con_prm_desc[3].cal_source   0
PAVRPARM  tParmEq[1].con_prm_desc[3].cal_status   3
PAVRPARM  tParmEq[1].con_prm_desc[3].cal_units    psi
PAVRPARM  tParmEq[1].con_prm_desc[3].app_con_max  150
PAVRPARM  tParmEq[1].con_prm_desc[3].app_con_min  0
PAVRPARM  tParmEq[1].con_prm_desc[3].app_cal_size 10
PAVRPARM  tParmEq[1].con_prm_desc[3].app_min_setp TRUE
PAVRPARM  tParmEq[1].con_prm_desc[3].app_frst_bit 1
PAVRPARM  tParmEq[1].con_prm_desc[3].app_otpt_bts 12
PAVRPARM  tParmEq[1].con_prm_desc[3].app_max_out  8000
PAVRPARM  tParmEq[1].con_prm_desc[3].app_min_out  1600
PAVRPARM  tParmEq[1].con_prm_desc[3].app_strob_ln 0
PAVRPARM  tParmEq[1].con_prm_desc[3].app_strb_out 0
PAVRPARM  tParmEq[1].con_prm_desc[3].chg_date     0
PAVRPARM  tParmEq[1].con_prm_des3[3].app_cntr_typ 1
PAVRPARM  tParmEq[1].con_prm_des3[3].app_cntr_st  0
PAVRPARM  tParmEq[1].con_prm_des3[3].app_echo_scl 10
!
PAVRPARM  tParmEq[1].con_prm_desc[4].app_con_type 3
PAVRPARM  tParmEq[1].con_prm_desc[4].cal_date     0
PAVRPARM  tParmEq[1].con_prm_desc[4].cal_source   0
PAVRPARM  tParmEq[1].con_prm_desc[4].cal_status   3
PAVRPARM  tParmEq[1].con_prm_desc[4].cal_units    Kv
PAVRPARM  tParmEq[1].con_prm_desc[4].app_con_max  100
PAVRPARM  tParmEq[1].con_prm_desc[4].app_con_min  0
PAVRPARM  tParmEq[1].con_prm_desc[4].app_cal_size 10
PAVRPARM  tParmEq[1].con_prm_desc[4].app_min_setp TRUE
PAVRPARM  tParmEq[1].con_prm_desc[4].app_frst_bit 2
PAVRPARM  tParmEq[1].con_prm_desc[4].app_otpt_bts 12
PAVRPARM  tParmEq[1].con_prm_desc[4].app_max_out  8000
PAVRPARM  tParmEq[1].con_prm_desc[4].app_min_out  1600
PAVRPARM  tParmEq[1].con_prm_desc[4].app_strob_ln 0
PAVRPARM  tParmEq[1].con_prm_desc[4].app_strb_out 0
PAVRPARM  tParmEq[1].con_prm_desc[4].app_con_nam  ES
PAVRPARM  tParmEq[1].con_prm_desc[4].app_con_titl Estats
PAVRPARM  tParmEq[1].con_prm_desc[4].chg_date     0
PAVRPARM  tParmEq[1].con_prm_des3[4].app_cntr_typ 1
PAVRPARM  tParmEq[1].con_prm_des3[4].app_cntr_st  0
PAVRPARM  tParmEq[1].con_prm_des3[4].app_echo_scl 10
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[10
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[211.1
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[322.2
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[433.3
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[544.4
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[655.6
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[766.7
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[877.8
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[988.9
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[1100
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[11600
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[22311
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[33022
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[43733
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[54444
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[65156
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[75867
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[86578
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[97289
PAVRPARM  tParmEq[1].con_prm_desc[4].control_cal[18000
PAPARMDB  num_con_parm                            4
PAPARMDB  ins_to_par[1,1]                         1
PAPARMDB  ins_to_par[1,2]                         2
PAPARMDB  ins_to_par[1,3]                         3
PAPARMDB  ins_to_par[2,1]                         4
PAPARMDB  scrn_to_par[1,1]                        1
PAPARMDB  scrn_to_par[1,2]                        2
PAPARMDB  scrn_to_par[1,3]                        3
PAPARMDB  scrn_to_par[1,4]                        0
PAPARMDB  scrn_to_par[2,1]                        4
PAPARMDB  scrn_to_par[2,2]                        0
PAPARMDB  scrn_to_par[2,3]                        0
PAPARMDB  scrn_to_par[2,4]                        0
! Applicator Parameter Option Variables (Gun)
PAVRPARM  tParmEq[1].con_prm_desc[2].cal_units    psi
PAVRPARM  tParmEq[1].con_prm_desc[2].app_con_nam  AA
PAVRPARM  tParmEq[1].con_prm_desc[2].app_con_titl Atom. Air
PAVRPARM  tParmEq[1].con_prm_desc[3].app_con_nam  FA
PAVRPARM  tParmEq[1].con_prm_desc[3].app_con_titl Fan Air
! Applicator Parameter Option Variables (Bell-306)
! Applicator Parameter Option Variables (Bell-607)
! Applicator Parameter Option Variables (Ethernet IP)
!
! Cell Digital Outputs Standard Color Change Option (Memory-Mapped)
! Cell Digital Outputs Standard Color Change Option (Model A I/O - 5 or 10 Slot 
! Cell Digital Outputs Standard Color Change Option (PLC I/O)
! Cell Digital Outputs Standard Color Change Option (PDI I/O)
! uncomment this line (and comment out the followning line) if you don't want to
!13	2	17	48	1	2	17	1	Gun 1 Pilot	PAVRCCIO	tCCIOEqCmos[1].cc_dout_do[1]
! Cell Digital Outputs Standard Color Change Option (PDI I/O but mapped to RDO[1
! Cell Digital Outputs Standard Color Change Option (Robot I/O mapped to RDO[1])
! Process Digital Outputs Standard Color Change Option (Memory-Mapped)
! Reserved for 2nd CC Control Valve group output (maximum 15 bits)
! Reserved for IPC mix valves and catalyst valves
! Process Digital Outputs Standard Color Change Option (Model A I/O - 5 Slot Rac
! Process Digital Outputs Standard Color Change Option (Model A I/O - 5 Slot Rac
! Process Digital Outputs Standard Color Change Option (Model A I/O - 10 Slot Ra
! Process Digital Outputs Standard Color Change Option (Ethernet IP)
! Process Group Outputs Standard Color Change Option (Memory-Mapped)
PAVRCCIO  tCCIOEqCmos[1].cc_grp1_go               1
PAVRCCIO  tCCIOEqCmos[1].csb_color_go             2
! Process Group Outputs Standard Color Change Option (Model A I/O - 5 Slot Rack 
! Process Group Outputs Standard Color Change Option (Model A I/O - 5 Slot Rack 
! Process Group Outputs Standard Color Change Option (Model A I/O - 10 Slot Rack
! Process Robot Outputs Standard Color Change Option (Ethernet IP but mapped to 
! Process Group Outputs Standard Color Change Option (Robot I/O mapped to RDO[1]
! Standard Color Change Option Variables (Any Type)
PAVRCCEX  tCCEXCmos.sap_unknown                   FALSE
PAVRCCEX  tCCEXCmos.cln_unknown                   FALSE
PAVRCCEX  tCCEXCmos.cc_cyc_nam[1]                 Push Out Cycle
PAVRCCEX  tCCEXCmos.cc_cyc_nam[2]                 Clean Out Cycle
PAVRCCEX  tCCEXCmos.cc_cyc_nam[3]                 Cap Clean Cycle
PAVRCCEX  tCCEXCmos.cc_cyc_nam[4]                 Fill Cycle
PAVRCCEX  tCCEXCmos.cc_cyc_nam[5]                 Refill Cycle
PAVRCCEX  tCCEXCmos.cc_cyc_nam[6]                 Full Push to Dump
PAVRCCEX  tCCEXCmos.cc_cyc_nam[7]                 Spare 1
PAVRCCEX  tCCEXCmos.cc_cyc_nam[8]                 Super Purge
PAVRCCEX  tCCEXCmos.push_cyc_no                   1
PAVRCCEX  tCCEXCmos.clean_cyc_no                  2
PAVRCCEX  tCCEXCmos.cap_cyc_no                    3
PAVRCCEX  tCCEXCmos.fill_cyc_no                   4
PAVRCCEX  tCCEXCmos.refil_cyc_no                  5
PAVRCCEX  tCCEXCmos.purge_cyc_no                  6
PAVRCCEX  tCCEXCmos.trig_cyc_no                   0
PAVRCCEX  tCCEXCmos.super_cyc_no                  8
PAVRCCIO  tCCIOEqCmos[1].cc_dout_do[4]            1025
PAVRCCEX  tCCEXCmos.dvalve_nam[1]                 pTRIG
PAVRCCEX  tCCEXCmos.dvalve_nam[2]                 RESERVED
PAVRCCEX  tCCEXCmos.dvalve_nam[3]                 RESERVED
PAVRCCEX  tCCEXCmos.dvalve_nam[4]                 pCC1-24
PAVRCCEX  tCCEXCmos.action_nam[1]                 Exit App Clnr
PAVRCCEX  tCCEXCmos.action_nam[2]                 Goto App Clnr
PAVRCCEX  tCCEXCmos.action_nam[3]                 Goto Purge
PAVRCCEX  tCCEXCmos.action_nam[4]                 Goto Home
PAVRCCEX  tCCEXCmos.event_nam[1]                  Above Cleaner
PAVRCCEX  tCCEXCmos.event_nam[2]                  In Cleaner
PAVRCCEX  tCCEXCmos.event_nam[3]                  At Purge
PAVRCCEX  tCCEXCmos.event_nam[4]                  At Home
PAVRCCEX  tCCEXCmos.stand_cc_cyc[1]               2
PAVRCCEX  tCCEXCmos.stand_cc_cyc[2]               4
PAVRCCEX  tCCEXCmos.stand_cc_cyc[3]               0
PAVRCCEX  tCCEXCmos.stand_cc_cyc[4]               0
PAVRCCEX  tCCEXCmos.stand_cc_cyc[5]               0
PAVRCCEX  tCCEXCmos.full_cc_cyc[1]                6
PAVRCCEX  tCCEXCmos.full_cc_cyc[2]                2
PAVRCCEX  tCCEXCmos.full_cc_cyc[3]                4
PAVRCCEX  tCCEXCmos.full_cc_cyc[4]                0
PAVRCCEX  tCCEXCmos.full_cc_cyc[5]                0
PAVRCCEX  tCCEXCmos.clean_to_0[1]                 6
PAVRCCEX  tCCEXCmos.clean_to_0[2]                 2
PAVRCCEX  tCCEXCmos.clean_to_0[3]                 0
PAVRCCEX  tCCEXCmos.clean_to_0[4]                 0
PAVRCCEX  tCCEXCmos.clean_to_0[5]                 0
PAVRCCEX  tCCEXCmos.cc_gun_no                     1
PAVRCCEX  tCCEXCmos.clr_ena_no                    4
PAVRCCEX  tCCEXCmos.prg_solv_no                   4
PAVRCCEX  tCCEXCmos.prg_air_no                    5
PAVRCCEX  tCCEXCmos.gun_dump_no                   7
! Standard Color Change Option Variables (Gun)
! wet_mask is now a 31 bit word!
PAVRCCEX  tCCEXCmos.wet_mask                      0
PAVRCCEX  tCCEXCmos.gvalve_nam[1]                 pPE
PAVRCCEX  tCCEXCmos.gvalve_nam[2]                 pIW
PAVRCCEX  tCCEXCmos.gvalve_nam[3]                 pBW
PAVRCCEX  tCCEXCmos.gvalve_nam[4]                 pSOL
PAVRCCEX  tCCEXCmos.gvalve_nam[5]                 pAIR
PAVRCCEX  tCCEXCmos.gvalve_nam[6]                 pCC
PAVRCCEX  tCCEXCmos.gvalve_nam[7]                 pDUMP
PAVRCCEX  tCCEXCmos.gvalve_nam[8]                 pFLUSH
PAVRCCEX  tCCEXCmos.gvalve_nam[9]                 ACA
PAVRCCEX  tCCEXCmos.gvalve_nam[10]                pDUMP2
PAVRCCEX  tCCEXCmos.gvalve_nam[11]                ACVA
PAVRCCEX  tCCEXCmos.gvalve_nam[12]                pACS
PAVRCCEX  tCCEXCmos.gvalve_nam[13]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[14]                p2T
PAVRCCEX  tCCEXCmos.gvalve_nam[15]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[16]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[17]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[18]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[19]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[20]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[21]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[22]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[23]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[24]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[25]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[26]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[27]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[28]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[29]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[30]                RESERVED
PAVRCCEX  tCCEXCmos.gvalve_nam[31]                RESERVED
! Standard Color Change Option Variables (Bell-306)
! Standard Color Change Option Variables (Bell-607)
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2276 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2289 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2300 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2700 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2713 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2724 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2825 column 4
! Cell Digital Inputs Tracking Option (Memory-Mapped)										
! Cell Digital Inputs Tracking Option (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Inputs Tracking Option (PLC I/O)
PAVRSHIO  tShioInit[1].prt_detct_di               9
! Cell Digital Inputs (PDI IO)
! Cell Digital Inputs (Robot I/O mapped to RDI[1])
! Tracking Option Variables (Any Type)
PAVRLNTR  sim_conv_spd                            30
PAVRLNTR  gunoff_opt                              FALSE
PAVRLNTR  conv_opt                                FALSE
PAVRLNTR  estat_opt                               FALSE
!job_strt_sg = 4
PAVROPTN  byOptn[1].byOp[4]                       1
! Tracking Option Variables (Gun)
! Tracking Option Variables (Bell-306)
! Tracking Option Variables (Bell-607)
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 2932 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 3128 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[67] = 0
user data is 1 at line 3143 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 3192 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[29] = 0
user data is 1 at line 3214 column 4
PAVRPADG  dmon_set_sch                            TRUE
PAVRPADG  flowoptn_err                            TRUE
PAVRPADG  alarm_range[1].min                      0
PAVRPADG  alarm_range[1].max                      0
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 3233 column 7
*WARN*[PAVROPTN]byOptn[1].byOp[10] = 0
user data is 1 at line 3240 column 7
PAVRPADG  paus_w_conv                             FALSE
*WARN*[PAVROPTN]byOptn[1].byOp[111] = 1
user data is 0 at line 3253 column 7
! Change default port types
! De-assign P4 from KCL
*SYSTEM*  $RS232_CFG[4].$COMMENT                  No Use
*SYSTEM*  $RS232_CFG[4].$DEVICEUSE                1
*SYSTEM*  $RS232_CFG[4].$FLOWCONTROL              274
! De-assign P2 from Floppy
*SYSTEM*  $RS232_CFG[2].$COMMENT                  No Use
*SYSTEM*  $RS232_CFG[2].$DEVICEUSE                1
*SYSTEM*  $RS232_CFG[2].$FLOWCONTROL              274
*SYSTEM*  $RS232_CFG[2].$AUXTASK                  0
! Assign P4 to Floppy
*SYSTEM*  $RS232_CFG[4].$COMMENT                  PS100/200 Disk
*SYSTEM*  $RS232_CFG[4].$DEVICEUSE                4
*SYSTEM*  $RS232_CFG[4].$FLOWCONTROL              19
*SYSTEM*  $RS232_CFG[4].$AUXTASK                  75
! Assign P2 to Conssole
*SYSTEM*  $RS232_CFG[2].$COMMENT                  Maintenance Cons
*SYSTEM*  $RS232_CFG[2].$DEVICEUSE                3
*SYSTEM*  $RS232_CFG[2].$FLOWCONTROL              275
! Set Option Already processed Flag
PAVROPTN  byOptn[1].byOp[112]                     1
! Cell Digital Outputs (Memory-Mapped)
! Cell Digital Outputs (PDI IO but mapped to RDO[1])
PAVRGUNS  tApplEq[1].ApplNum[1].gun_out_num       17
*WARN*[PAVROPTN]byOptn[1].byOp[33] = 0
user data is 1 at line 3325 column 4
Data File run-time =  0 (sec)
