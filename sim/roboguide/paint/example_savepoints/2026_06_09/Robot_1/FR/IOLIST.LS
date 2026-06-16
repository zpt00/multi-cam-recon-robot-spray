! Cell Digital Inputs (Memory-Mapped)							
! Cell Digital Inputs (Model A I/O - 5 or 10 Slot Rack)
!2	1	15	1	1	1	15	1	Clear Ques	PAVRSHIO	clr_ques_di
! Cell Digital Inputs (PLC I/O)
 1   9 67  1  1   9 1	Start Job
 1  10 67  1  1  10 1	Abort Prod.
 1  11 67  1  1  11 1	
 1  12 67  1  1  12 1	Cancel Job
 1  13 67  1  1  13 1	Continue Job
 1  14 67  1  1  14 1	
 1  15 67  1  1  15 1	Clear Ques
 1  16 67  1  1  16 1	
 1  17 67  1  1  17 1	Job Data B0
 1  18 67  1  1  18 1	Job Data B1
 1  19 67  1  1  19 1	Job Data B2
 1  20 67  1  1  20 1	Job Data B3
 1  21 67  1  1  21 1	Job Data B4
 1  22 67  1  1  22 1	Job Data B5
 1  23 67  1  1  23 1	Job Data B6
 1  24 67  1  1  24 1	Job Data B7
 1  25 67  1  1  25 1	Job Data B8
 1  26 67  1  1  26 1	Job Data B9
 1  27 67  1  1  27 1	ColorData B0
 1  28 67  1  1  28 1	ColorData B1
 1  29 67  1  1  29 1	ColorData B2
 1  30 67  1  1  30 1	ColorData B3
 1  31 67  1  1  31 1	ColorData B4
 1  32 67  1  1  32 1	Read Strobe
! Cell Digital Inputs (PDI IO)
! Cell Digital Inputs (Robot I/O mapped to RDI[1])
! Cell Group Inputs (Memory-Mapped)
! Cell Group Inputs (Model A I/O - 5 or 10 Slot Rack)
! Cell Group Inputs (PLC I/O)
 18   1 67  1  1  17 10	Job Data
 18   2 67  1  1  27 5	Color Data
! Cell Group Inputs (PDI IO)
! Cell Digital Outputs (Memory-Mapped)
! Cell Digital Outputs (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Outputs (PLC I/O)
 2   9 67  1  2   9 1	Prod. Mode
 2  10 67  1  2  10 1	Job Active
 2  11 67  1  2  11 1	
 2  12 67  1  2  12 1	Job Ready
 2  13 67  1  2  13 1	At HOME
 2  14 67  1  2  14 1	Major Fault
 2  15 67  1  2  15 1	Heartbeat
 2  16 67  1  2  16 1	
 2  17 67  1  2  17 1	
 2  18 67  1  2  18 1	
 2  19 67  1  2  19 1	Wait For PLC
 2  20 67  1  2  20 1	No Continue
 2  21 67  1  2  21 1	
 2  22 67  1  2  22 1	
 2  23 67  1  2  23 1	
 2  24 67  1  2  24 1	
 2  25 67  1  2  25 1	
 2  26 67  1  2  26 1	
 2  27 67  1  2  27 1	
 2  28 67  1  2  28 1	
 2  29 67  1  2  29 1	
 2  30 67  1  2  30 1	
 2  31 67  1  2  31 1	
 2  32 67  1  2  32 1	
! Cell Digital Outputs (PDI IO)
! uncomment this line (and comment out the followning line) if you don't want to
!13	2	17	48	1	2	17	1	Gun 1 Pilot	PAVRGUNS	tApplEq[1].ApplNum[1].gun_out_num
! Cell Digital Outputs (PDI IO but mapped to RDO[1])
! Cell Group Outputs (Memory-Mapped)
! Cell Group Outputs (Model A I/O - 5 Slot Rack)
! Cell Group Outputs (Model A I/O - 10 Slot Rack)
! Cell Group Outputs (PLC I/O)
*WARN*[PAVROPTN]byOptn[1].byOp[110] = 0
user data is 1 at line 931 column 7
! Cell Digital Inputs (Model A I/O - 5 or 10 Slot Rack)
! Process Digital Inputs (Memory-Mapped)
 1 129  0  0 28 129 1	
 1 130  0  0 28 130 1	
 1 131  0  0 28 131 1	
 1 132  0  0 28 132 1	
 1 133  0  0 28 133 1	
 1 134  0  0 28 134 1	
 1 135  0  0 28 135 1	
 1 136  0  0 28 136 1	
 1 137  0  0 28 137 1	
 1 138  0  0 28 138 1	
 1 139  0  0 28 139 1	
 1 140  0  0 28 140 1	
 1 141  0  0 28 141 1	
 1 142  0  0 28 142 1	
 1 143  0  0 28 143 1	
 1 144  0  0 28 144 1	
 1 145  0  0 28 145 1	
 1 146  0  0 28 146 1	
 1 147  0  0 28 147 1	
 1 148  0  0 28 148 1	
 1 149  0  0 28 149 1	
 1 150  0  0 28 150 1	
 1 151  0  0 28 151 1	
 1 152  0  0 28 152 1	
 1 153  0  0 28 153 1	
 1 154  0  0 28 154 1	
 1 155  0  0 28 155 1	
 1 156  0  0 28 156 1	
 1 157  0  0 28 157 1	
 1 158  0  0 28 158 1	
 1 159  0  0 28 159 1	
 1 160  0  0 28 160 1	
 1 161  0  0 28 161 1	
 1 162  0  0 28 162 1	
 1 163  0  0 28 163 1	
 1 164  0  0 28 164 1	
 1 165  0  0 28 165 1	
 1 166  0  0 28 166 1	
 1 167  0  0 28 167 1	
 1 168  0  0 28 168 1	
 1 169  0  0 28 169 1	
 1 170  0  0 28 170 1	
 1 171  0  0 28 171 1	
 1 172  0  0 28 172 1	
 1 173  0  0 28 173 1	
 1 174  0  0 28 174 1	
 1 175  0  0 28 175 1	
 1 176  0  0 28 176 1	
 1 177  0  0 28 177 1	
 1 178  0  0 28 178 1	
 1 179  0  0 28 179 1	
 1 180  0  0 28 180 1	
 1 181  0  0 28 181 1	
 1 182  0  0 28 182 1	
 1 183  0  0 28 183 1	
 1 184  0  0 28 184 1	
 1 185  0  0 28 185 1	
 1 186  0  0 28 186 1	
 1 187  0  0 28 187 1	
 1 188  0  0 28 188 1	
 1 189  0  0 28 189 1	
 1 190  0  0 28 190 1	
 1 191  0  0 28 191 1	
 1 192  0  0 28 192 1	
 1 193  0  0 28 193 1	
 1 194  0  0 28 194 1	
 1 195  0  0 28 195 1	
 1 196  0  0 28 196 1	
 1 197  0  0 28 197 1	
 1 198  0  0 28 198 1	
 1 199  0  0 28 199 1	
 1 200  0  0 28 200 1	
 1 201  0  0 28 201 1	
 1 202  0  0 28 202 1	
 1 203  0  0 28 203 1	
 1 204  0  0 28 204 1	
 1 205  0  0 28 205 1	
 1 206  0  0 28 206 1	
 1 207  0  0 28 207 1	
 1 208  0  0 28 208 1	
 1 209  0  0 28 209 1	
 1 210  0  0 28 210 1	
 1 211  0  0 28 211 1	
 1 212  0  0 28 212 1	
 1 213  0  0 28 213 1	
 1 214  0  0 28 214 1	
 1 215  0  0 28 215 1	
 1 216  0  0 28 216 1	
 1 217  0  0 28 217 1	
 1 218  0  0 28 218 1	
 1 219  0  0 28 219 1	
 1 220  0  0 28 220 1	
 1 221  0  0 28 221 1	
 1 222  0  0 28 222 1	
 1 223  0  0 28 223 1	
 1 224  0  0 28 224 1	
 1 225  0  0 28 225 1	
 1 226  0  0 28 226 1	
 1 227  0  0 28 227 1	
 1 228  0  0 28 228 1	
 1 229  0  0 28 229 1	
 1 230  0  0 28 230 1	
 1 231  0  0 28 231 1	
 1 232  0  0 28 232 1	
 1 233  0  0 28 233 1	
 1 234  0  0 28 234 1	
 1 235  0  0 28 235 1	
 1 236  0  0 28 236 1	
 1 237  0  0 28 237 1	
 1 238  0  0 28 238 1	
 1 239  0  0 28 239 1	
 1 240  0  0 28 240 1	
 1 241  0  0 28 241 1	
 1 242  0  0 28 242 1	
 1 243  0  0 28 243 1	
 1 244  0  0 28 244 1	
 1 245  0  0 28 245 1	
 1 246  0  0 28 246 1	
 1 247  0  0 28 247 1	
 1 248  0  0 28 248 1	
 1 249  0  0 28 249 1	
 1 250  0  0 28 250 1	
 1 251  0  0 28 251 1	
 1 252  0  0 28 252 1	
 1 253  0  0 28 253 1	
 1 254  0  0 28 254 1	
 1 255  0  0 28 255 1	
 1 256  0  0 28 256 1	
! Process Digital Inputs (Model A I/O - 5 Slot Rack)
! Process Digital Inputs (Model A I/O - 10 Slot Rack)
! Process Analog Inputs (Memory-Mapped)
 3   1  0  0 29  51 1	
 3   2  0  0 29  52 1	
 3   3  0  0 29  53 1	
 3   4  0  0 29  54 1	
 3   5  0  0 29  55 1	
 3   6  0  0 29  56 1	
 3   7  0  0 29  57 1	
 3   8  0  0 29  58 1	
 3   9  0  0 29  59 1	
 3  10  0  0 29  60 1	
 3  11  0  0 29  61 1	
 3  12  0  0 29  62 1	
 3  13  0  0 29  63 1	
 3  14  0  0 29  64 1	
 3  15  0  0 29  65 1	
 3  16  0  0 29  66 1	
 3  17  0  0 29  67 1	
 3  18  0  0 29  68 1	
 3  19  0  0 29  69 1	
 3  20  0  0 29  70 1	
 3  21  0  0 29  71 1	
 3  22  0  0 29  72 1	
 3  23  0  0 29  73 1	
 3  24  0  0 29  74 1	
 3  25  0  0 29  75 1	
! Process Analog Inputs (Model A I/O - 5 Slot Rack)
! Process Analog Inputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Memory-Mapped)
 2 129  0  0 28 385 1	
 2 130  0  0 28 386 1	
 2 131  0  0 28 387 1	
 2 132  0  0 28 388 1	
 2 133  0  0 28 389 1	
 2 134  0  0 28 390 1	
 2 135  0  0 28 391 1	
 2 136  0  0 28 392 1	
 2 137  0  0 28 393 1	
 2 138  0  0 28 394 1	
 2 139  0  0 28 395 1	
 2 140  0  0 28 396 1	
 2 141  0  0 28 397 1	
 2 142  0  0 28 398 1	
 2 143  0  0 28 399 1	
 2 144  0  0 28 400 1	
 2 145  0  0 28 401 1	
 2 146  0  0 28 402 1	
 2 147  0  0 28 403 1	
 2 148  0  0 28 404 1	
 2 149  0  0 28 405 1	
 2 150  0  0 28 406 1	
 2 151  0  0 28 407 1	
 2 152  0  0 28 408 1	
 2 153  0  0 28 409 1	
 2 154  0  0 28 410 1	
 2 155  0  0 28 411 1	
 2 156  0  0 28 412 1	
 2 157  0  0 28 413 1	
 2 158  0  0 28 414 1	
 2 159  0  0 28 415 1	
 2 160  0  0 28 416 1	
 2 161  0  0 28 417 1	
 2 162  0  0 28 418 1	
 2 163  0  0 28 419 1	
 2 164  0  0 28 420 1	
 2 165  0  0 28 421 1	
 2 166  0  0 28 422 1	
 2 167  0  0 28 423 1	
 2 168  0  0 28 424 1	
 2 169  0  0 28 425 1	
 2 170  0  0 28 426 1	
 2 171  0  0 28 427 1	
 2 172  0  0 28 428 1	
 2 173  0  0 28 429 1	
 2 174  0  0 28 430 1	
 2 175  0  0 28 431 1	
 2 176  0  0 28 432 1	
 2 177  0  0 28 433 1	
 2 178  0  0 28 434 1	
 2 179  0  0 28 435 1	
 2 180  0  0 28 436 1	
 2 181  0  0 28 437 1	
 2 182  0  0 28 438 1	
 2 183  0  0 28 439 1	
 2 184  0  0 28 440 1	
 2 185  0  0 28 441 1	
 2 186  0  0 28 442 1	
 2 187  0  0 28 443 1	
 2 188  0  0 28 444 1	
 2 189  0  0 28 445 1	
 2 190  0  0 28 446 1	
 2 191  0  0 28 447 1	
 2 192  0  0 28 448 1	
 2 193  0  0 28 449 1	
 2 194  0  0 28 450 1	
 2 195  0  0 28 451 1	
 2 196  0  0 28 452 1	
 2 197  0  0 28 453 1	
 2 198  0  0 28 454 1	
 2 199  0  0 28 455 1	
 2 200  0  0 28 456 1	
 2 201  0  0 28 457 1	
 2 202  0  0 28 458 1	
 2 203  0  0 28 459 1	
 2 204  0  0 28 460 1	
 2 205  0  0 28 461 1	
 2 206  0  0 28 462 1	
 2 207  0  0 28 463 1	
 2 208  0  0 28 464 1	
 2 209  0  0 28 465 1	
 2 210  0  0 28 466 1	
 2 211  0  0 28 467 1	
 2 212  0  0 28 468 1	
 2 213  0  0 28 469 1	
 2 214  0  0 28 470 1	
 2 215  0  0 28 471 1	
 2 216  0  0 28 472 1	
 2 217  0  0 28 473 1	
 2 218  0  0 28 474 1	
 2 219  0  0 28 475 1	
 2 220  0  0 28 476 1	
 2 221  0  0 28 477 1	
 2 222  0  0 28 478 1	
 2 223  0  0 28 479 1	
 2 224  0  0 28 480 1	
 2 225  0  0 28 481 1	
 2 226  0  0 28 482 1	
 2 227  0  0 28 483 1	
 2 228  0  0 28 484 1	
 2 229  0  0 28 485 1	
 2 230  0  0 28 486 1	
 2 231  0  0 28 487 1	
 2 232  0  0 28 488 1	
 2 233  0  0 28 489 1	
 2 234  0  0 28 490 1	
 2 235  0  0 28 491 1	
 2 236  0  0 28 492 1	
 2 237  0  0 28 493 1	
 2 238  0  0 28 494 1	
 2 239  0  0 28 495 1	
 2 240  0  0 28 496 1	
 2 241  0  0 28 497 1	
 2 242  0  0 28 498 1	
 2 243  0  0 28 499 1	
 2 244  0  0 28 500 1	
 2 245  0  0 28 501 1	
 2 246  0  0 28 502 1	
 2 247  0  0 28 503 1	
 2 248  0  0 28 504 1	
 2 249  0  0 28 505 1	
 2 250  0  0 28 506 1	
 2 251  0  0 28 507 1	
 2 252  0  0 28 508 1	
 2 253  0  0 28 509 1	
 2 254  0  0 28 510 1	
 2 255  0  0 28 511 1	
 2 256  0  0 28 512 1	
! Process Digital Outputs (Model A I/O - 5 Slot Rack)
! Process Digital Outputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Model A I/O - 10 Slot Rack)
! Process Digital Outputs (Ethernet IP but mapped to RDO[1])
! Process Digital Outputs (Robot I/O mapped to RDO[1])
! Process Analog Outputs (Memory-Mapped)
 4   1  0  0 29  76 1	
 4   2  0  0 29  77 1	
 4   3  0  0 29  78 1	
 4   4  0  0 29  79 1	
 4   5  0  0 29  80 1	
 4   6  0  0 29  81 1	
 4   7  0  0 29  82 1	
 4   8  0  0 29  83 1	
 4   9  0  0 29  84 1	
 4  10  0  0 29  85 1	
 4  11  0  0 29  86 1	
 4  12  0  0 29  87 1	
 4  13  0  0 29  88 1	
 4  14  0  0 29  89 1	
 4  15  0  0 29  90 1	
 4  16  0  0 29  91 1	
 4  17  0  0 29  92 1	
 4  18  0  0 29  93 1	
 4  19  0  0 29  94 1	
 4  20  0  0 29  95 1	
 4  21  0  0 29  96 1	
 4  22  0  0 29  97 1	
 4  23  0  0 29  98 1	
 4  24  0  0 29  99 1	
 4  25  0  0 29 100 1	
! Process Analog Outputs (Model A I/O - 5 Slot Rack)
! Process Analog Outputs (Model A I/O - 10 Slot Rack)
! Cell Digital Inputs (Memory-Mapped)
! Note: Just always assign reserved I/O ports for any Cell I/O Type for unsed I/
 1 1025  0  0 28 2049 1	
 1 1026  0  0 28 2050 1	
 1 1027  0  0 28 2051 1	
 1 1028  0  0 28 2052 1	
 1 1029  0  0 28 2053 1	
 1 1030  0  0 28 2054 1	
 1 1031  0  0 28 2055 1	
 1 1032  0  0 28 2056 1	
! Cell Digital Outputs (Memory-Mapped)
! Note: Just always assign reserved I/O ports for any Cell I/O Type for unsed I/
 2 1025  0  0 28 2057 1	
 2 1026  0  0 28 2058 1	
 2 1027  0  0 28 2059 1	
 2 1028  0  0 28 2060 1	
 2 1029  0  0 28 2061 1	
 2 1030  0  0 28 2062 1	
 2 1031  0  0 28 2063 1	
 2 1032  0  0 28 2064 1	
! Cell Digital Outputs Function Instruction (Memory-Mapped)
! Cell Digital Outputs Function Instruction (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Outputs Function Instruction (PLC I/O)
 2  25 67  1  2  25 1	UserFunc B0
 2  26 67  1  2  26 1	UserFunc B1
 2  27 67  1  2  27 1	UserFunc B2
 2  28 67  1  2  28 1	UserFunc B3
 2  29 67  1  2  29 1	UserFunc B4
 2  30 67  1  2  30 1	UserFunc B5
 2  31 67  1  2  31 1	UserFunc B6
 2  32 67  1  2  32 1	UserFunc B7
*WARN*[PAVROPTN]byOptn[1].byOp[23] = 0
user data is 1 at line 1382 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 1427 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[16] = 0
user data is 1 at line 1472 column 4
! Process Analog Outputs Applicator Parameter Option (Memory-Mapped)										
 4   1  0  0 29  76 1	Fluid Flow
 4   2  0  0 29  77 1	Atom. Air
 4   3  0  0 29  78 1	Fan Air
 4   4  0  0 29  79 1	Estats
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 5 Slot Rack 
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 5 Slot Rack 
! Process Analog Outputs Applicator Parameter Option (Model A I/O - 10 Slot Rack
! Process Analog Outputs Applicator Parameter Option (Ethernet IP)
! Cell Digital Outputs Standard Color Change Option (Memory-Mapped)
! Cell Digital Outputs Standard Color Change Option (Model A I/O - 5 or 10 Slot 
! Cell Digital Outputs Standard Color Change Option (PLC I/O)
! Cell Digital Outputs Standard Color Change Option (PDI I/O)
! uncomment this line (and comment out the followning line) if you don't want to
!13	2	17	48	1	2	17	1	Gun 1 Pilot	PAVRCCIO	tCCIOEqCmos[1].cc_dout_do[1]
! Cell Digital Outputs Standard Color Change Option (PDI I/O but mapped to RDO[1
! Cell Digital Outputs Standard Color Change Option (Robot I/O mapped to RDO[1])
! Process Digital Outputs Standard Color Change Option (Memory-Mapped)
 2 129  0  0 28 385 1	pPE
 2 130  0  0 28 386 1	pIW
 2 131  0  0 28 387 1	pBW
 2 132  0  0 28 388 1	pSOL
 2 133  0  0 28 389 1	pAIR
 2 134  0  0 28 390 1	pCC
 2 135  0  0 28 391 1	pDUMP
 2 136  0  0 28 392 1	pFLUSH
 2 137  0  0 28 393 1	ACA
 2 138  0  0 28 394 1	pDUMP2
 2 139  0  0 28 395 1	ACVA
 2 140  0  0 28 396 1	pACS
 2 141  0  0 28 397 1	Reserved
 2 142  0  0 28 398 1	p2T
 2 143  0  0 28 399 1	Reserved
 2 144  0  0 28 400 1	Gun 1 Pilot
! Reserved for 2nd CC Control Valve group output (maximum 15 bits)
 2 145  0  0 28 401 1	Reserved
 2 146  0  0 28 402 1	Reserved
 2 147  0  0 28 403 1	Reserved
 2 148  0  0 28 404 1	Reserved
 2 149  0  0 28 405 1	Reserved
 2 150  0  0 28 406 1	Reserved
 2 151  0  0 28 407 1	Reserved
 2 152  0  0 28 408 1	Reserved
 2 153  0  0 28 409 1	Reserved
 2 154  0  0 28 410 1	Reserved
 2 155  0  0 28 411 1	Reserved
 2 156  0  0 28 412 1	Reserved
 2 157  0  0 28 413 1	Reserved
 2 158  0  0 28 414 1	Reserved
 2 159  0  0 28 415 1	Reserved
! Reserved for IPC mix valves and catalyst valves
 2 160  0  0 28 416 1	Reserved
 2 161  0  0 28 417 1	Reserved
 2 162  0  0 28 418 1	Reserved
 2 163  0  0 28 419 1	Reserved
 2 164  0  0 28 420 1	Reserved
 2 165  0  0 28 421 1	Reserved
 2 166  0  0 28 422 1	Reserved
 2 167  0  0 28 423 1	Reserved
 2 168  0  0 28 424 1	Color Vlv 1
 2 169  0  0 28 425 1	Color Vlv 2
 2 170  0  0 28 426 1	Color Vlv 3
 2 171  0  0 28 427 1	Color Vlv 4
 2 172  0  0 28 428 1	Color Vlv 5
 2 173  0  0 28 429 1	Color Vlv 6
 2 174  0  0 28 430 1	Color Vlv 7
 2 175  0  0 28 431 1	Color Vlv 8
! Process Digital Outputs Standard Color Change Option (Model A I/O - 5 Slot Rac
! Process Digital Outputs Standard Color Change Option (Model A I/O - 5 Slot Rac
! Process Digital Outputs Standard Color Change Option (Model A I/O - 10 Slot Ra
! Process Digital Outputs Standard Color Change Option (Ethernet IP)
! Process Group Outputs Standard Color Change Option (Memory-Mapped)
 19   1  0  0 29  26 1	CC Cntr Valv
 19   2  0  0 29  27 1	Color valves
! Process Group Outputs Standard Color Change Option (Model A I/O - 5 Slot Rack 
! Process Group Outputs Standard Color Change Option (Model A I/O - 5 Slot Rack 
! Process Group Outputs Standard Color Change Option (Model A I/O - 10 Slot Rack
! Process Robot Outputs Standard Color Change Option (Ethernet IP but mapped to 
! Process Group Outputs Standard Color Change Option (Robot I/O mapped to RDO[1]
*WARN*[PAVRDBCV]qty_clr_val = 4
user data is 16 at line 2016 column 7
*WARN*[PAVRDBCV]qty_clr_val = 4
user data is 24 at line 2045 column 7
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2196 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2212 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2228 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[40] = 0
user data is 1 at line 2243 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2339 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2355 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2370 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2624 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[41] = 0
user data is 1 at line 2653 column 4
! Cell Digital Inputs Tracking Option (Memory-Mapped)										
! Cell Digital Inputs Tracking Option (Model A I/O - 5 or 10 Slot Rack)
! Cell Digital Inputs Tracking Option (PLC I/O)
 1   9 67  1  1   9 1	Part Detect
! Cell Digital Inputs (PDI IO)
! Cell Digital Inputs (Robot I/O mapped to RDI[1])
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 2870 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 2881 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 3151 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[65] = 0
user data is 1 at line 3181 column 4
*WARN*[PAVROPTN]byOptn[1].byOp[29] = 0
user data is 1 at line 3202 column 4
! Cell Digital Outputs (Memory-Mapped)
! Cell Digital Outputs (PDI IO but mapped to RDO[1])
 2  17  0  0  9   1 1	Gun 1 Pilot
*WARN*[PAVROPTN]byOptn[1].byOp[33] = 0
user data is 1 at line 3315 column 4
Data File run-time =  0 (sec)
