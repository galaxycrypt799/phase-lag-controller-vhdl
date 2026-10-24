----------------------------------------------------------------------------------
-- Package: phase_lag_pkg
-- Phase Lag Controller for plant G(s) = 100 / (s^2 + 10s + 100)
-- Controller C(s) = 100 * (s + 1) / (100s + 1)
-- Discretization: Tustin, Ts = 0.01 s
-- Fixed-point: 24-bit signed datapath with Q14 scaling (execute with >> 14)
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;

PACKAGE phase_lag_pkg IS

    -- Data path widths
    CONSTANT WORD_BITS  : NATURAL := 24;
    CONSTANT ACC_BITS   : NATURAL := 48;
    CONSTANT Q_FRAC_BITS  : NATURAL := 14;
    CONSTANT Q_SCALE_VALUE : INTEGER := 16384;  -- 2^14

    -- Fixed-point step amplitudes (Q14 data scale)
    CONSTANT STEP_HALF_Q14 : INTEGER := 8192;   -- 0.5
    CONSTANT STEP_ONE_Q14 : INTEGER := 16384; -- 1.0

    -- Microprogram size
    CONSTANT CTRL_STORE_SIZE : NATURAL := 17;

    ---------------------------------------------------------------------------
    -- Phase Lag controller coefficients (float -> A = a * 2^14)
    -- u(k) = (B1*u(k-1) + A0*e(k) + A1*e(k-1)) >> 14
    ---------------------------------------------------------------------------
    CONSTANT CTRL_B1_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(16382, WORD_BITS);
    CONSTANT CTRL_A0_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(16465, WORD_BITS);
    CONSTANT CTRL_A1_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(-16301, WORD_BITS);

    ---------------------------------------------------------------------------
    -- Second-order plant coefficients (Tustin, Ts = 0.01 s)
    -- y(k) = (P1*y1 + P2*y2 + B0*u + B1*u1 + B2*u2) >> 14
    ---------------------------------------------------------------------------
    CONSTANT MODEL_P1_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(31056, WORD_BITS);
    CONSTANT MODEL_P2_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(-14827, WORD_BITS);
    CONSTANT MODEL_B0_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(39, WORD_BITS);
    CONSTANT MODEL_B1_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(78, WORD_BITS);
    CONSTANT MODEL_B2_Q14 : SIGNED(WORD_BITS - 1 DOWNTO 0) :=
        CONV_SIGNED(39, WORD_BITS);

    -- ALU operation codes
    CONSTANT ALUOP_ADD : UNSIGNED(1 DOWNTO 0) := "00";
    CONSTANT ALUOP_SUB : UNSIGNED(1 DOWNTO 0) := "01";
    CONSTANT ALUOP_MUL : UNSIGNED(1 DOWNTO 0) := "10";
    CONSTANT ALUOP_INC : UNSIGNED(1 DOWNTO 0) := "11";

    -- Register write-back mux
    CONSTANT WB_FROM_ALU : UNSIGNED(2 DOWNTO 0) := "000";
    CONSTANT WB_FROM_ERR : UNSIGNED(2 DOWNTO 0) := "001";
    CONSTANT WB_FROM_B1  : UNSIGNED(2 DOWNTO 0) := "010";
    CONSTANT WB_FROM_A1  : UNSIGNED(2 DOWNTO 0) := "011";
    CONSTANT WB_FROM_A0  : UNSIGNED(2 DOWNTO 0) := "100";

    FUNCTION q14_shift_down(fixed_point_value : SIGNED(ACC_BITS - 1 DOWNTO 0)) RETURN SIGNED;

    COMPONENT microdata IS
        PORT (
            err_in, coeff_b1, coeff_a1, coeff_a0 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            fld_a, fld_b, fld_c                    : IN  UNSIGNED(3 DOWNTO 0);
            alu_op                                 : IN  UNSIGNED(1 DOWNTO 0);
            ldr_in, ldr_b1, ldr_a1, ldr_a0, ldr_out : IN  STD_LOGIC;
            selr_in                                : IN  UNSIGNED(2 DOWNTO 0);
            ldrf                                   : IN  STD_LOGIC;
            zero, neg, cy                          : OUT STD_LOGIC;
            u_out                                  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            clk                                    : IN  STD_LOGIC;
            control_con                            : IN  STD_LOGIC
        );
    END COMPONENT;

    COMPONENT microctrl IS
        GENERIC (rom_depth : NATURAL := CTRL_STORE_SIZE);
        PORT (
            start, zero, neg, cy                   : IN  STD_LOGIC;
            fld_a, fld_b, fld_c                     : OUT UNSIGNED(3 DOWNTO 0);
            alu_op                                  : OUT UNSIGNED(1 DOWNTO 0);
            ldr_in, ldr_b1, ldr_a1, ldr_a0, ldr_out : OUT STD_LOGIC;
            selr_in                                 : OUT UNSIGNED(2 DOWNTO 0);
            ldrf, ctrl_done                         : OUT STD_LOGIC;
            clk                                     : IN  STD_LOGIC;
            control_con                             : OUT STD_LOGIC
        );
    END COMPONENT;

    COMPONENT micro IS
        PORT (
            start                                  : IN  STD_LOGIC;
            err_in                                 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            coeff_b1, coeff_a1, coeff_a0           : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_out                                  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            ctrl_done                              : OUT STD_LOGIC;
            clk                                    : IN  STD_LOGIC
        );
    END COMPONENT;

    COMPONENT plant IS
        PORT (
            rst, sample_en                         : IN  STD_LOGIC;
            u_now                                  : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            clk                                    : IN  STD_LOGIC;
            y_out                                  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            plant_done                             : OUT STD_LOGIC;
            y_old1, y_old2                         : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_old1, u_old2                         : OUT SIGNED(WORD_BITS - 1 DOWNTO 0)
        );
    END COMPONENT;

    COMPONENT phase_lag_top IS
        GENERIC (
            CLK_FREQ_HZ     : NATURAL := 50000000;
            SAMPLE_PERIOD_S : REAL    := 0.01;
            SIM_SHORT_DIV   : BOOLEAN := TRUE;
            G_COEFF_A0      : INTEGER := 16465;
            G_COEFF_A1      : INTEGER := -16301;
            G_COEFF_B1      : INTEGER := 16382
        );
        PORT (
            clk       : IN  STD_LOGIC;
            rst       : IN  STD_LOGIC;
            ref_in    : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            start     : IN  STD_LOGIC;
            y_out     : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_out     : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_now   : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_old   : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_old     : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            ctrl_done : OUT STD_LOGIC;
            plant_done: OUT STD_LOGIC;
            sample_tick : OUT STD_LOGIC
        );
    END COMPONENT;

END phase_lag_pkg;

PACKAGE BODY phase_lag_pkg IS

    FUNCTION q14_shift_down(fixed_point_value : SIGNED(ACC_BITS - 1 DOWNTO 0)) RETURN SIGNED IS
        VARIABLE shifted_result : SIGNED(ACC_BITS - 1 DOWNTO 0) := (OTHERS => '0');
        VARIABLE sign_extension_start : NATURAL := ACC_BITS - Q_FRAC_BITS - 1;
    BEGIN
        shifted_result(sign_extension_start DOWNTO 0) := fixed_point_value(ACC_BITS - 1 DOWNTO Q_FRAC_BITS);
        IF fixed_point_value(ACC_BITS - 1) = '1' THEN
            shifted_result(ACC_BITS - 1 DOWNTO sign_extension_start + 1) := (OTHERS => '1');
        END IF;
        RETURN shifted_result;
    END FUNCTION;

END phase_lag_pkg;
