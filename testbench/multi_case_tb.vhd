----------------------------------------------------------------------------------
-- Testbench: multi_case_tb
-- Runs four Phase Lag controller coefficient cases in parallel.
--
-- C0 original: a0=16465, a1=-16301, b1=16382
-- C1 weak:     a0=8233,  a1=-8151,  b1=16382
-- C3 strong:   a0=24698, a1=-24452, b1=16382
-- C6 smooth:   a0=8233,  a1=-8151,  b1=16383
--
-- The main design is not rewritten. phase_lag_top keeps default coefficients,
-- and this testbench overrides them through generics for comparison only.
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY multi_case_tb IS
END multi_case_tb;

ARCHITECTURE behavioral OF multi_case_tb IS

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
            clk         : IN  STD_LOGIC;
            rst         : IN  STD_LOGIC;
            ref_in      : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            start       : IN  STD_LOGIC;
            y_out       : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_out       : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_now     : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_old     : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_old       : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            ctrl_done   : OUT STD_LOGIC;
            plant_done  : OUT STD_LOGIC;
            sample_tick : OUT STD_LOGIC
        );
    END COMPONENT;

    CONSTANT TB_CLK_PERIOD : TIME := 20 ns;

    TYPE word_array_t IS ARRAY (0 TO 3) OF SIGNED(WORD_BITS - 1 DOWNTO 0);
    TYPE bit_array_t   IS ARRAY (0 TO 3) OF STD_LOGIC;

    SIGNAL tb_clk        : STD_LOGIC := '0';
    SIGNAL tb_rst        : STD_LOGIC := '1';
    SIGNAL tb_start      : STD_LOGIC := '0';
    SIGNAL tb_finished      : BOOLEAN := FALSE;
    SIGNAL tb_sample_index  : NATURAL := 0;

    SIGNAL case_ref         : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_a0          : word_array_t := (
        0 => CONV_SIGNED(16465, WORD_BITS),
        1 => CONV_SIGNED(8233, WORD_BITS),
        2 => CONV_SIGNED(24698, WORD_BITS),
        3 => CONV_SIGNED(8233, WORD_BITS)
    );
    SIGNAL case_a1          : word_array_t := (
        0 => CONV_SIGNED(-16301, WORD_BITS),
        1 => CONV_SIGNED(-8151, WORD_BITS),
        2 => CONV_SIGNED(-24452, WORD_BITS),
        3 => CONV_SIGNED(-8151, WORD_BITS)
    );
    SIGNAL case_b1          : word_array_t := (
        0 => CONV_SIGNED(16382, WORD_BITS),
        1 => CONV_SIGNED(16382, WORD_BITS),
        2 => CONV_SIGNED(16382, WORD_BITS),
        3 => CONV_SIGNED(16383, WORD_BITS)
    );
    SIGNAL case_y           : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_u           : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_error         : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_error_prev     : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_u_prev       : word_array_t := (OTHERS => (OTHERS => '0'));
    SIGNAL case_ctrl_done   : bit_array_t := (OTHERS => '0');
    SIGNAL case_model_done  : bit_array_t := (OTHERS => '0');
    SIGNAL case_sample_tick      : bit_array_t := (OTHERS => '0');

    FUNCTION to_q14_real(x : SIGNED(WORD_BITS - 1 DOWNTO 0)) RETURN REAL IS
    BEGIN
        RETURN REAL(CONV_INTEGER(x)) / REAL(Q_SCALE_VALUE);
    END FUNCTION;

    PROCEDURE emit_case_report(
        CONSTANT case_name : IN STRING;
        SIGNAL   ref_s     : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   a0_s      : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   a1_s      : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   b1_s      : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   y_s       : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   u_s       : IN SIGNED(WORD_BITS - 1 DOWNTO 0);
        SIGNAL   err_s     : IN SIGNED(WORD_BITS - 1 DOWNTO 0)
    ) IS
    BEGIN
        REPORT case_name &
               " | ref=" & INTEGER'IMAGE(CONV_INTEGER(ref_s)) &
               " (" & REAL'IMAGE(to_q14_real(ref_s)) & ")" &
               " a0=" & INTEGER'IMAGE(CONV_INTEGER(a0_s)) &
               " a1=" & INTEGER'IMAGE(CONV_INTEGER(a1_s)) &
               " b1=" & INTEGER'IMAGE(CONV_INTEGER(b1_s)) &
               " y=" & INTEGER'IMAGE(CONV_INTEGER(y_s)) &
               " (" & REAL'IMAGE(to_q14_real(y_s)) & ")" &
               " u=" & INTEGER'IMAGE(CONV_INTEGER(u_s)) &
               " (" & REAL'IMAGE(to_q14_real(u_s)) & ")" &
               " e=" & INTEGER'IMAGE(CONV_INTEGER(err_s)) &
               " (" & REAL'IMAGE(to_q14_real(err_s)) & ")";
    END PROCEDURE;

BEGIN

    tb_clk <= NOT tb_clk AFTER TB_CLK_PERIOD / 2 WHEN NOT tb_finished ELSE '0';

    dut_C0_original: phase_lag_top
        GENERIC MAP (
            CLK_FREQ_HZ     => 50000000,
            SAMPLE_PERIOD_S => 0.01,
            SIM_SHORT_DIV   => TRUE,
            G_COEFF_A0      => 16465,
            G_COEFF_A1      => -16301,
            G_COEFF_B1      => 16382
        )
        PORT MAP (
            clk         => tb_clk,
            rst         => tb_rst,
            ref_in      => case_ref(0),
            start       => tb_start,
            y_out       => case_y(0),
            u_out       => case_u(0),
            err_now     => case_error(0),
            err_old     => case_error_prev(0),
            u_old       => case_u_prev(0),
            ctrl_done   => case_ctrl_done(0),
            plant_done  => case_model_done(0),
            sample_tick => case_sample_tick(0)
        );

    dut_C1_weak: phase_lag_top
        GENERIC MAP (
            CLK_FREQ_HZ     => 50000000,
            SAMPLE_PERIOD_S => 0.01,
            SIM_SHORT_DIV   => TRUE,
            G_COEFF_A0      => 8233,
            G_COEFF_A1      => -8151,
            G_COEFF_B1      => 16382
        )
        PORT MAP (
            clk         => tb_clk,
            rst         => tb_rst,
            ref_in      => case_ref(1),
            start       => tb_start,
            y_out       => case_y(1),
            u_out       => case_u(1),
            err_now     => case_error(1),
            err_old     => case_error_prev(1),
            u_old       => case_u_prev(1),
            ctrl_done   => case_ctrl_done(1),
            plant_done  => case_model_done(1),
            sample_tick => case_sample_tick(1)
        );

    dut_C3_strong: phase_lag_top
        GENERIC MAP (
            CLK_FREQ_HZ     => 50000000,
            SAMPLE_PERIOD_S => 0.01,
            SIM_SHORT_DIV   => TRUE,
            G_COEFF_A0      => 24698,
            G_COEFF_A1      => -24452,
            G_COEFF_B1      => 16382
        )
        PORT MAP (
            clk         => tb_clk,
            rst         => tb_rst,
            ref_in      => case_ref(2),
            start       => tb_start,
            y_out       => case_y(2),
            u_out       => case_u(2),
            err_now     => case_error(2),
            err_old     => case_error_prev(2),
            u_old       => case_u_prev(2),
            ctrl_done   => case_ctrl_done(2),
            plant_done  => case_model_done(2),
            sample_tick => case_sample_tick(2)
        );

    dut_C6_smooth: phase_lag_top
        GENERIC MAP (
            CLK_FREQ_HZ     => 50000000,
            SAMPLE_PERIOD_S => 0.01,
            SIM_SHORT_DIV   => TRUE,
            G_COEFF_A0      => 8233,
            G_COEFF_A1      => -8151,
            G_COEFF_B1      => 16383
        )
        PORT MAP (
            clk         => tb_clk,
            rst         => tb_rst,
            ref_in      => case_ref(3),
            start       => tb_start,
            y_out       => case_y(3),
            u_out       => case_u(3),
            err_now     => case_error(3),
            err_old     => case_error_prev(3),
            u_old       => case_u_prev(3),
            ctrl_done   => case_ctrl_done(3),
            plant_done  => case_model_done(3),
            sample_tick => case_sample_tick(3)
        );

    stimulus: PROCESS
    BEGIN
        tb_rst   <= '1';
        tb_start <= '0';
        case_ref    <= (OTHERS => (OTHERS => '0'));
        WAIT FOR 200 ns;

        tb_rst <= '0';
        REPORT "Reset released for multi-case testbench";
        WAIT FOR 300 ns;

        tb_start <= '1';
        WAIT FOR 50 us;

        case_ref <= (OTHERS => CONV_SIGNED(STEP_HALF_Q14, WORD_BITS));
        REPORT "Step input applied to all cases: ref = 8192 (0.5 in Q14)";

        WAIT FOR 10 ms;

        REPORT "========== FINAL MULTI-CASE RESULT ==========";
        emit_case_report("C0 original", case_ref(0), case_a0(0), case_a1(0), case_b1(0), case_y(0), case_u(0), case_error(0));
        emit_case_report("C1 weak    ", case_ref(1), case_a0(1), case_a1(1), case_b1(1), case_y(1), case_u(1), case_error(1));
        emit_case_report("C3 strong  ", case_ref(2), case_a0(2), case_a1(2), case_b1(2), case_y(2), case_u(2), case_error(2));
        emit_case_report("C6 smooth  ", case_ref(3), case_a0(3), case_a1(3), case_b1(3), case_y(3), case_u(3), case_error(3));

        tb_finished <= TRUE;
        WAIT;
    END PROCESS;

    sample_monitor: PROCESS (tb_clk)
    BEGIN
        IF rising_edge(tb_clk) THEN
            IF case_sample_tick(0) = '1' AND tb_rst = '0' THEN
                tb_sample_index <= tb_sample_index + 1;

                IF tb_sample_index MOD 100 = 0 THEN
                    REPORT "========== Sample " & INTEGER'IMAGE(tb_sample_index) & " ==========";
                    emit_case_report("C0 original", case_ref(0), case_a0(0), case_a1(0), case_b1(0), case_y(0), case_u(0), case_error(0));
                    emit_case_report("C1 weak    ", case_ref(1), case_a0(1), case_a1(1), case_b1(1), case_y(1), case_u(1), case_error(1));
                    emit_case_report("C3 strong  ", case_ref(2), case_a0(2), case_a1(2), case_b1(2), case_y(2), case_u(2), case_error(2));
                    emit_case_report("C6 smooth  ", case_ref(3), case_a0(3), case_a1(3), case_b1(3), case_y(3), case_u(3), case_error(3));
                END IF;
            END IF;
        END IF;
    END PROCESS;

END behavioral;
