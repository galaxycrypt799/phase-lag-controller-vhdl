----------------------------------------------------------------------------------
-- Company: FPGA Course - Final Project
-- Engineer: Student
--
-- Create Date:    June 2026
-- Design Name:    FPGA based Phase Lag Controller - Control Subsystem
-- Module Name:    microctrl - Behavioral
-- Project Name:   FPGA based Phase Lag Controller
-- Target Devices: Spartan 3E / Any FPGA
-- Tool versions:  ISE Design Suite 14.7 / Vivado
--
-- Description:
--   CONTROL SUBSYSTEM (MICROPROGRAMMED SEQUENCER) for Phase Lag Controller
--
--   This module implements a microprogrammed control unit that sequences
--   the datapath through the Phase Lag control algorithm.
--
--   ARCHITECTURE:
--   - CSAR (Control Store Address Register): Points to current microinstruction
--   - CS (Control Store): ROM-like array of 26-bit microinstructions
--   - Condition evaluator: Checks status flags for branching
--
--   MICROINSTRUCTION FORMAT (26 bits):
--
--   OPERATIONAL MODE (bit 25 = '0'):
--   ┌──────┬────────┬────────┬────────┬────────┬───┬───┬───┬───┬───┬───┬────────┬───┬───┐
--   │Mode=0│ alu_op │ fld_a  │ fld_b  │ fld_c  │reg_bank │RIN│B1 │A1 │A0 │OUT│selr_in │D1 │D0 │
--   │ [25] │[24:23]│[22:19] │[18:15] │[14:11] │[10] [9] [8] [7] [6] [5]│ [4:2]  │[1]│[0]│
--   └──────┴────────┴────────┴────────┴────────┴───┴───┴───┴───┴───┴───┴────────┴───┴───┘
--
--   BRANCH MODE (bit 25 = '1'):
--   ┌──────┬───────────┬──────────┬──────────────────────────────────────┐
--   │Mode=1│ branch_flag │ branch_level │          branch_address [21:0]        │
--   │ [25] │ [24:23]   │  [22]    │                                       │
--   └──────┴───────────┴──────────┴──────────────────────────────────────┘
--
--   CONDITION CODES (bits 24-23):
--     "00" = start signal
--     "01" = zero flag
--     "10" = negative flag
--     "11" = carry flag
--
--   PHASE LAG ALGORITHM IMPLEMENTED:
--     u(k) = (B1*u(k-1) + A0*e(k) + A1*e(k-1)) >> 14
--
--   Register map:
--     R1 = e(k), R6 = e(k-1), R7 = u(k-1)
--     R2 = accumulator / u(k)
--
--   REGISTER MAP (for reference):
--     R0 = Constant 0
--     R1 = Current error e(k)
--     R2 = Accumulator (intermediate and final result)
--     R3 = B1 coefficient product path
--     R4 = A1 coefficient product path
--     R5 = Unused
--     R6 = Previous error e(k-1) [PERSISTENT]
--     R7 = Previous output u(k-1) [PERSISTENT]
--
-- Dependencies: IEEE standard libraries
--
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--   The microprogram is 17 states (0-16) implementing the Phase Lag algorithm.
--   Each complete iteration takes approximately 17 clock cycles.
--   With a 50MHz clock, one iteration = 17 * 20ns = 340ns.
--   The effective control loop rate is ~1/(340ns + plant settling time).
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;

ENTITY microctrl IS
    GENERIC (
        rom_depth : NATURAL := 17   -- Number of microinstructions
    );
    PORT (
        -- Input status flags
        start     : IN  STD_LOGIC;   -- Start new computation
        zero      : IN  STD_LOGIC;   -- ALU zero flag
        neg       : IN  STD_LOGIC;   -- ALU negative flag
        cy        : IN  STD_LOGIC;   -- ALU carry flag

        -- Output control signals to datapath
        fld_a     : OUT UNSIGNED(3 DOWNTO 0);  -- Source register A
        fld_b     : OUT UNSIGNED(3 DOWNTO 0);  -- Source register B
        fld_c     : OUT UNSIGNED(3 DOWNTO 0);  -- Destination register
        alu_op    : OUT UNSIGNED(1 DOWNTO 0);  -- ALU operation
        ldr_in    : OUT STD_LOGIC;   -- Load error input
        ldr_b1    : OUT STD_LOGIC;   -- Load B1 coefficient
        ldr_a1    : OUT STD_LOGIC;   -- Load A1 coefficient
        ldr_a0    : OUT STD_LOGIC;   -- Load A0 coefficient
        ldr_out   : OUT STD_LOGIC;   -- Load output register
        selr_in   : OUT UNSIGNED(2 DOWNTO 0);  -- Write-back source select
        ldrf      : OUT STD_LOGIC;   -- Register file write enable
        ctrl_done : OUT STD_LOGIC := '0';   -- Computation complete

        -- System clock
        clk       : IN  STD_LOGIC;

        -- Control connection
        control_con : OUT STD_LOGIC
    );
END microctrl;

ARCHITECTURE behav_microprogr OF microctrl IS

    ---------------------------------------------------------------------------
    -- CONTROL STORE ADDRESS REGISTER (CSAR)
    -- Points to the current microinstruction being executed
    ---------------------------------------------------------------------------
    SIGNAL rom_addr   : NATURAL RANGE 0 TO rom_depth-1 := 0;

    -- Current microinstruction (decoded from control store)
    SIGNAL rom_word : UNSIGNED(25 DOWNTO 0) := (OTHERS => '0');

    ---------------------------------------------------------------------------
    -- MICROINSTRUCTION FIELD ALIASES (for readability)
    ---------------------------------------------------------------------------
    ALIAS word_kind      : STD_LOGIC              IS rom_word(25);
    ALIAS branch_flag : UNSIGNED(1 DOWNTO 0)   IS rom_word(24 DOWNTO 23);
    ALIAS branch_level  : STD_LOGIC              IS rom_word(22);

BEGIN

    ---------------------------------------------------------------------------
    -- CSAR UPDATE PROCESS (Synchronous)
    --
    -- On each rising edge of the clock:
    --   - If word_kind='0' (operational): increment CSAR (next instruction)
    --   - If word_kind='1' (branch): evaluate branch_flag and branch or proceed
    --
    -- Branch logic:
    --   IF (selected_condition == branch_level) THEN
    --       CSAR = branch_address (take the branch)
    --   ELSE
    --       CSAR = CSAR + 1 (fall through)
    --   END IF;
    ---------------------------------------------------------------------------
    PROCESS (clk)
        VARIABLE branch_address : UNSIGNED(21 DOWNTO 0);
    BEGIN
        IF (clk'EVENT AND clk = '1') THEN

            IF (rom_addr = rom_depth-1) THEN
                -- Last ROM entry is a safety return to the wait-start loop_state.
                -- This prevents fall-through to address 17 when a status flag
                -- has an unexpected value at the end of a transaction.
                rom_addr <= 1;

            ELSIF (word_kind = '0') THEN
                -- Operational word_kind: simply advance to next instruction
                rom_addr <= rom_addr + 1;
            ELSE
                -- Branch word_kind: evaluate branch_flag
                CASE branch_flag IS
                    -----------------------------------------------------------
                    -- Condition "00": Check START signal
                    -----------------------------------------------------------
                    WHEN "00" =>
                        IF (start = branch_level) THEN
                            branch_address := rom_word(21 DOWNTO 0);
                            rom_addr  <= CONV_INTEGER(branch_address);
                        ELSE
                            rom_addr <= rom_addr + 1;
                        END IF;

                    -----------------------------------------------------------
                    -- Condition "01": Check ZERO flag
                    -----------------------------------------------------------
                    WHEN "01" =>
                        IF (zero = branch_level) THEN
                            branch_address := rom_word(21 DOWNTO 0);
                            rom_addr  <= CONV_INTEGER(branch_address);
                        ELSE
                            rom_addr <= rom_addr + 1;
                        END IF;

                    -----------------------------------------------------------
                    -- Condition "10": Check NEGATIVE flag
                    -----------------------------------------------------------
                    WHEN "10" =>
                        IF (neg = branch_level) THEN
                            branch_address := rom_word(21 DOWNTO 0);
                            rom_addr  <= CONV_INTEGER(branch_address);
                        ELSE
                            rom_addr <= rom_addr + 1;
                        END IF;

                    -----------------------------------------------------------
                    -- Condition "11": Check CARRY flag
                    -----------------------------------------------------------
                    WHEN "11" =>
                        IF (cy = branch_level) THEN
                            branch_address := rom_word(21 DOWNTO 0);
                            rom_addr  <= CONV_INTEGER(branch_address);
                        ELSE
                            rom_addr <= rom_addr + 1;
                        END IF;

                    WHEN OTHERS => NULL;
                END CASE;
            END IF;

        END IF;
    END PROCESS;

    ---------------------------------------------------------------------------
    -- CONTROL STORE (Microprogram ROM)
    --
    -- This is the heart of the Phase Lag controller. Each 26-bit entry
    -- encodes one micro-operation. The sequencer walks through these
    -- instructions to compute:
    --
    --   u(k) = (B1*u(k-1) + A0*e(k) + A1*e(k-1)) >> 14
    --
    -- Instruction format (operational, bit 25='0'):
    --   [25]=0, [24:23]=alu_op, [22:19]=fld_a, [18:15]=fld_b,
    --   [14:11]=fld_c, [10]=ldrf, [9]=ldr_in, [8]=ldr_b1, [7]=ldr_a1,
    --   [6]=ldr_a0, [5]=ldr_out, [4:2]=selr_in, [1]=done_hi, [0]=done_lo
    --
    -- Instruction format (branch, bit 25='1'):
    --   [25]=1, [24:23]=branch_flag, [22]=branch_level, [21:0]=branch_address
    ---------------------------------------------------------------------------
    PROCESS (rom_addr)
        TYPE rom_array_t IS ARRAY (0 TO rom_depth-1) OF UNSIGNED(25 DOWNTO 0);
        VARIABLE rom_image : rom_array_t := (

            -------------------------------------------------------------------
            -- STEP 0: Initialize - Clear R0 (R0 = 0)
            --
            -- Operation: R0 <- ALU(0+0) = 0
            -- This ensures R0 holds the constant zero for future operations.
            -- selr_in=000 selects ALU output as write-back source.
            -------------------------------------------------------------------
            0 => "00000000000000010000000001",
            --    ││││││││││││││││││││││││││
            --    ││││└┴┴┴┤└┴┴┴┤└┴┴┴┤││││││└┴┴┘│└┘
            --    ││││ fld_a fld_b fld_c│││││selr_in │
            --    ││└┴┘=0000 =0000 =0000││││││ =000  │
            --    ││ alu_op=ADD         │││││└ldr_out └─ done(1,0)=(0,1)=clear
            --    │└────────────────────┘│││└─ldkp
            --    └─ word_kind=0 (operational)││└──ldki
            --                           │└───ldkd
            --                           └────ldr_in
            --                            ldrf=1 (write reg_bank)

            -------------------------------------------------------------------
            -- STEP 1: Wait for START signal
            --
            -- Branch branch_flag: if start=0, jump to addr 1 (self-loop)
            --                   if start=1, fall through to step 2
            -- This implements polling: stay here until external start pulse.
            -------------------------------------------------------------------
            1 => "10000000000000000000000001",
            --    ││││└──────────────────────────┘
            --    │││└─ branch_level=0                └── branch_address=1
            --    │└┴── branch_flag=00 (check START)
            --    └─── word_kind=1 (branch)

            -------------------------------------------------------------------
            -- STEP 2: Load all inputs
            --
            -- R1 <- 0 (initialize)
            -- Load external inputs: mic_in -> err_latch, kd(b1), ki(a1), kp(a0)
            -- All load signals active: ldr_in=1, ldr_b1=1, ldr_a1=1, ldr_a0=1
            -- selr_in=000 (ALU), alu_op=ADD, fld_a=0, fld_b=0 -> ALU=0 -> R1=0
            -------------------------------------------------------------------
            2 => "00000000000000111111000001",
            --    ││││││││││││││││││││││││││
            --    ││││=0 =0 =0 =1 1 1 1 1 0 =0 │
            --    ││││f_a f_b f_c R K K K K O R  done=(0,1)
            --    ││││         l F D I P U I
            --    ││││         d   d d d T N
            --    ││││         r         _
            --    ││││         f         o
            --    ││││                   u
            --    ││││                   t
            --    ││││
            --    ││└┴─ alu_op=00 (ADD)
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 3: R1 <- current error e(k)
            --
            -- Load the error value (held in err_latch) into register R1.
            -- selr_in=001 selects err_latch (the external mic_in error input).
            -- R1 now holds e(k) for this iteration.
            -------------------------------------------------------------------
            3 => "00000000000000110000000101",
            --    ││││=0 =0 =0 =1 1 0 0 0 0 0 =1 │
            --    ││││f_a f_b f_c l   l l l l s    done
            --    ││││              d   d d d d e   =(0,1)
            --    ││││              r   k k k k l
            --    ││││              f   d i p o r
            --    ││││                      u _
            --    ││││                      t i
            --    ││││                        n
            --    ││└┴─ alu_op=00
            --    └─── word_kind=0
            --    selr_in=001 (RIN)

            -------------------------------------------------------------------
            -- STEP 4: R2 <- a0 coefficient (kp input)
            --
            -- Load the a0 coefficient from kp holding register into R2.
            -- selr_in=100 selects a0 coefficient.
            -- R2 will be the accumulator for computing u(k).
            -------------------------------------------------------------------
            4 => "00000000000001010000010001",
            --    ││││=0 =0 =0 =2 1 0 0 0 0 0 =4 │
            --    ││││f_a f_b f_c l   l l l l s    done
            --    ││││              d   d d d d e   =(0,1)
            --    ││││              r   k k k k l
            --    ││││              f   d i p o r
            --    ││││                      u _
            --    ││││                      t i
            --    ││││                        n
            --    ││││
            --    ││││ selr_in=100 (A0)
            --    ││││ fld_c=0010 (R2)
            --    ││└┴─ alu_op=00
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 5: R2 <- R2 * R1  =>  a0 * e(k)
            --
            -- Multiply a0 (R2) by current error e(k) (R1).
            -- ALU performs signed multiplication; datapath rescales by shifting right 14 bits for Q14.
            -- fld_a=2 (R2=a0), fld_b=1 (R1=e(k)), fld_c=2 (R2=result)
            -- alu_op=10 (MUL)
            -------------------------------------------------------------------
            5 => "01000100001001010000000001",
            --    ││││=2 =1 =2 1 0 0 0 0 0 =0 │
            --    ││││R2 R1 R2 l   l l l l s    done
            --    ││││          d   d d d d e   =(0,1)
            --    ││││          r   k k k k l
            --    ││││          f   d i p o r
            --    ││││                  u _
            --    ││││                  t i
            --    ││││                    n
            --    ││└┴─ alu_op=10 (MUL)
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 6: R4 <- a1 coefficient (ki input)
            --
            -- Load a1 from ki holding register into R4.
            -- selr_in=011 selects a1 coefficient.
            -------------------------------------------------------------------
            6 => "00000000000010010000001101",
            --    ││││=0 =0 =4 1 0 0 0 0 0 =3 │
            --    ││││f_a f_b R4 l   l l l l KI   done
            --    ││││          d   d d d d     =(0,1)
            --    ││││          r   k k k k
            --    ││││          f   d i p o
            --    ││││                  u
            --    ││││                  t
            --    ││└┴─ alu_op=00
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 7: R4 <- R4 * R6  =>  a1 * e(k-1)
            --
            -- Multiply a1 (R4) by previous error e(k-1) (R6).
            -- fld_a=4 (R4=a1), fld_b=6 (R6=e_old), fld_c=4 (R4=result)
            -- alu_op=10 (MUL)
            -------------------------------------------------------------------
            7 => "01001000110010010000000001",
            --    ││││=4 =6 =4 1 0 0 0 0 0 =0 │
            --    ││││R4 R6 R4 l   l l l l s    done
            --    ││││          d   d d d d e   =(0,1)
            --    ││││          r   k k k k l
            --    ││││          f   d i p o r
            --    ││││                  u _
            --    ││││                  t i
            --    ││││                    n
            --    ││└┴─ alu_op=10 (MUL)
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 8: R2 <- R2 + R4  =>  a0*e(k) + a1*e(k-1)
            --
            -- Add a0*e(k) [in R2] and a1*e(k-1) [in R4].
            -- This is the first partial sum of the controller output.
            -- fld_a=2 (R2), fld_b=4 (R4), fld_c=2 (R2=result)
            -- alu_op=00 (ADD)
            -------------------------------------------------------------------
            8 => "00000100100001010000000001",
            --    ││││=2 =4 =2 1 0 0 0 0 0 =0 │
            --    ││││R2 R4 R2 l   l l l l s    done
            --    ││││          d   d d d d e   =(0,1)
            --    ││││          r   k k k k l
            --    ││││          f   d i p o r
            --    ││││                  u _
            --    ││││                  t i
            --    ││││                    n
            --    ││└┴─ alu_op=00 (ADD)
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 9: R3 <- b1 coefficient (kd input)
            --
            -- Load b1 from kd holding register into R3.
            -- selr_in=010 selects b1 coefficient.
            -------------------------------------------------------------------
            9 => "00000000000001110000001001",
            --    ││││=0 =0 =3 1 0 0 0 0 0 =2 │
            --    ││││f_a f_b R3 l   l l l l KD   done
            --    ││││          d   d d d d      =(0,1)
            --    ││││          r   k k k k
            --    ││││          f   d i p o
            --    ││││                  u
            --    ││││                  t
            --    ││└┴─ alu_op=00
            --    └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 10: R3 <- R3 * R7  =>  b1 * u(k-1)
            --
            -- Multiply b1 (R3) by previous output u(k-1) (R7).
            -- fld_a=3 (R3=b1), fld_b=7 (R7=u_old), fld_c=3 (R3=result)
            -- alu_op=10 (MUL)
            -------------------------------------------------------------------
            10 => "01000110111001110000000001",
            --     ││││=3 =7 =3 1 0 0 0 0 0 =0 │
            --     ││││R3 R7 R3 l   l l l l s    done
            --     ││││          d   d d d d e   =(0,1)
            --     ││││          r   k k k k l
            --     ││││          f   d i p o r
            --     ││││                  u _
            --     ││││                  t i
            --     ││││                    n
            --     ││└┴─ alu_op=10 (MUL)
            --     └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 11: R2 <- R2 + R3  =>  u(k) COMPLETE!
            --
            -- Add b1*u(k-1) [in R3] to the partial sum [in R2].
            -- R2 now holds: u(k) = a0*e(k) + a1*e(k-1) + b1*u(k-1)
            -- This is the final control output for this iteration!
            -- fld_a=2 (R2), fld_b=3 (R3), fld_c=2 (R2=result)
            -- alu_op=00 (ADD)
            -------------------------------------------------------------------
            11 => "00000100011001010000000001",
            --     ││││=2 =3 =2 1 0 0 0 0 0 =0 │
            --     ││││R2 R3 R2 l   l l l l s    done
            --     ││││          d   d d d d e   =(0,1)
            --     ││││          r   k k k k l
            --     ││││          f   d i p o r
            --     ││││                  u _
            --     ││││                  t i
            --     ││││                    n
            --     ││└┴─ alu_op=00 (ADD)
            --     └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 12: R6 <- R1  =>  Save e(k) as e(k-1) for next iteration
            --
            -- Copy current error to previous error register.
            -- Uses ALU ADD with R0(0) to pass through: R1 + 0 = R1 -> R6
            -- fld_a=1 (R1=e(k)), fld_b=0 (R0=0), fld_c=6 (R6=e_old)
            -- alu_op=00 (ADD)
            -------------------------------------------------------------------
            12 => "00000010000011010000000001",
            --     ││││=1 =0 =6 1 0 0 0 0 0 =0 │
            --     ││││R1 R0 R6 l   l l l l s    done
            --     ││││          d   d d d d e   =(0,1)
            --     ││││          r   k k k k l
            --     ││││          f   d i p o r
            --     ││││                  u _
            --     ││││                  t i
            --     ││││                    n
            --     ││└┴─ alu_op=00 (ADD)
            --     └─── word_kind=0
            --     selr_in=000 (ALU)

            -------------------------------------------------------------------
            -- STEP 13: R7 <- R2  =>  Save u(k) as u(k-1) for next iteration
            --
            -- Copy current output to previous output register.
            -- Uses ALU ADD with R0(0) to pass through: R2 + 0 = u(k) -> R7
            -- fld_a=2 (R2=u(k)), fld_b=0 (R0=0), fld_c=7 (R7=u_old)
            -- alu_op=00 (ADD)
            -------------------------------------------------------------------
            13 => "00000100000011110000000001",
            --     ││││=2 =0 =7 1 0 0 0 0 0 =0 │
            --     ││││R2 R0 R7 l   l l l l s    done
            --     ││││          d   d d d d e   =(0,1)
            --     ││││          r   k k k k l
            --     ││││          f   d i p o r
            --     ││││                  u _
            --     ││││                  t i
            --     ││││                    n
            --     ││└┴─ alu_op=00 (ADD)
            --     └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 14: Output result - z_out <- u(k) and pulse DONE
            --
            -- Drive the computed u(k) to the output port and signal completion.
            -- ldr_out=1 captures ALU output (R2+R0=u(k)) to z_out register.
            -- fld_a=2 (R2=u(k)), fld_b=0 (R0=0)
            -- alu_op=00 (ADD), selr_in=000 (ALU), ldrf=0 (don't write reg_bank)
            -- done bits: (1,0) = done pulse (auto-clears after 20ns)
            -------------------------------------------------------------------
            14 => "00000100000000000000100010",
            --     ││││=2 =0 =0 0 0 0 0 0 1 =0 1 0
            --     ││││R2 R0 -- l   l l l l o s    done
            --     ││││          d   d d d d u r   =(1,0)=pulse
            --     ││││          r   k k k k t _
            --     ││││          f   d i p o   i
            --     ││││                        n
            --     ││└┴─ alu_op=00 (ADD)
            --     └─── word_kind=0

            -------------------------------------------------------------------
            -- STEP 15: Unconditional branch back to STEP 1 (wait for next start)
            --
            -- Condition=11 (carry), branch_level=0.
            -- Since the ADD in step 14 (R2+R0) won't produce a carry (no overflow),
            -- cy=0 always matches branch_level=0, so the branch is always taken.
            -- Branch address = 1 (wait for next start signal).
            -------------------------------------------------------------------
            15 => "11100000000000000000000001",
            --     ││││└──────────────────────────┘
            --     │││└─ branch_level=0                └── branch_address=1
            --     │└┴── branch_flag=11 (check CARRY)
            --     └─── word_kind=1 (branch)

            -------------------------------------------------------------------
            -- STEP 16: Safety fallback (should never be reached)
            --
            -- If somehow we fall through step 15, force branch back to step 1.
            -------------------------------------------------------------------
            16 => "11100000000000000000000001"
            --     Same as step 15: unconditional branch to 1
        );
    BEGIN
        -- Read current microinstruction from control store
        rom_word <= rom_image(rom_addr);

        -- Output control_con (word_kind bit) to datapath
        control_con <= rom_word(25);

        -----------------------------------------------------------------------
        -- DECODE AND OUTPUT CONTROL SIGNALS
        --
        -- Depending on the word_kind bit:
        --   word_kind='0' (operational): Drive all datapath control signals
        --   word_kind='1' (branch):      Disable all datapath writes
        -----------------------------------------------------------------------
        CASE rom_word(25) IS
            ---------------------------------------------------------------
            -- OPERATIONAL MODE: Decode and drive control signals
            ---------------------------------------------------------------
            WHEN '0' =>
                -- ALU control
                alu_op  <= rom_word(24 DOWNTO 23);

                -- Register file addresses
                fld_a   <= rom_word(22 DOWNTO 19);
                fld_b   <= rom_word(18 DOWNTO 15);
                fld_c   <= rom_word(14 DOWNTO 11);

                -- Register file write enable
                ldrf    <= rom_word(10);

                -- Input capture signals
                ldr_in  <= rom_word(9);
                ldr_b1  <= rom_word(8);
                ldr_a1  <= rom_word(7);
                ldr_a0  <= rom_word(6);

                -- Output capture
                ldr_out <= rom_word(5);

                -- Write-back source select
                selr_in <= rom_word(4 DOWNTO 2);

            ---------------------------------------------------------------
            -- BRANCH MODE: Disable all datapath writes
            --
            -- When branching, no datapath operations should occur.
            -- All write enables are forced low.
            ---------------------------------------------------------------
            WHEN '1' =>
                ldrf    <= '0';
                ldr_out <= '0';
                ldr_in  <= '0';
                ldr_b1  <= '0';
                ldr_a1  <= '0';
                ldr_a0  <= '0';

            WHEN OTHERS => NULL;
        END CASE;

    END PROCESS;

    ---------------------------------------------------------------------------
    -- Clock-synchronized done (step 14 = output u(k))
    ---------------------------------------------------------------------------
    PROCESS (clk)
    BEGIN
        IF rising_edge(clk) THEN
            ctrl_done <= '0';
            IF rom_addr = 14 AND rom_word(25) = '0' THEN
                ctrl_done <= '1';
            END IF;
        END IF;
    END PROCESS;

END behav_microprogr;
