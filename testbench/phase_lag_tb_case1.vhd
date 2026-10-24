----------------------------------------------------------------------------------
-- Testbench: phase_lag_tb_case1
-- Verifies reset, controller step, plant impulse, closed-loop step response
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY phase_lag_tb_case1 IS
END phase_lag_tb_case1;

ARCHITECTURE behavioral OF phase_lag_tb_case1 IS

    COMPONENT phase_lag_top IS
        GENERIC (
            CLK_FREQ_HZ     : NATURAL;
            SAMPLE_PERIOD_S : REAL;
            SIM_SHORT_DIV   : BOOLEAN;
            G_COEFF_A0      : INTEGER := 8233;
            G_COEFF_A1      : INTEGER := -8151;
            G_COEFF_B1      : INTEGER := 16382
        );
        PORT (
            clk        : IN  STD_LOGIC;
            rst        : IN  STD_LOGIC;
            ref_in     : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            start      : IN  STD_LOGIC;
            y_out      : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_out      : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_now    : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            err_old    : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_old      : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            ctrl_done  : OUT STD_LOGIC;
            plant_done : OUT STD_LOGIC;
            sample_tick: OUT STD_LOGIC
        );
    END COMPONENT;

    COMPONENT micro IS
        PORT (
            start                        : IN  STD_LOGIC;
            err_in                       : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            coeff_b1, coeff_a1, coeff_a0 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_out                        : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            ctrl_done                    : OUT STD_LOGIC;
            clk                          : IN  STD_LOGIC
        );
    END COMPONENT;

    COMPONENT plant IS
        PORT (
            rst, sample_en : IN  STD_LOGIC;
            u_now          : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            clk            : IN  STD_LOGIC;
            y_out          : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            plant_done     : OUT STD_LOGIC;
            y_old1, y_old2 : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            u_old1, u_old2 : OUT SIGNED(WORD_BITS - 1 DOWNTO 0)
        );
    END COMPONENT;

    CONSTANT TB_CLK_PERIOD : TIME := 20 ns;
    CONSTANT TB_A0_Q14 : INTEGER := 8233;
    CONSTANT TB_A1_Q14 : INTEGER := -8151;
    CONSTANT TB_B1_Q14 : INTEGER := 16382;

    SIGNAL tb_clk        : STD_LOGIC := '0';
    SIGNAL tb_rst        : STD_LOGIC := '1';
    SIGNAL tb_ref        : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_a0         : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(TB_A0_Q14, WORD_BITS);
    SIGNAL tb_a1         : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(TB_A1_Q14, WORD_BITS);
    SIGNAL tb_b1         : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(TB_B1_Q14, WORD_BITS);
    SIGNAL tb_start      : STD_LOGIC := '0';
    SIGNAL tb_y          : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL tb_u          : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL tb_error        : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL tb_error_prev    : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL tb_u_prev      : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL tb_ctrl_done  : STD_LOGIC;
    SIGNAL tb_model_done : STD_LOGIC;
    SIGNAL tb_sample     : STD_LOGIC;
    SIGNAL tb_finished      : BOOLEAN := FALSE;
    SIGNAL tb_sample_index  : NATURAL := 0;

    SIGNAL unit_clk, unit_start, unit_done : STD_LOGIC := '0';
    SIGNAL unit_error                  : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(STEP_ONE_Q14, WORD_BITS);
    SIGNAL unit_control                    : SIGNED(WORD_BITS - 1 DOWNTO 0);

    SIGNAL model_clk, model_rst, model_enable, model_done : STD_LOGIC := '0';
    SIGNAL model_input                    : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(STEP_ONE_Q14, WORD_BITS);
    SIGNAL model_output                    : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL model_y_z1, model_y_z2, model_u_z1, model_u_z2 : SIGNED(WORD_BITS - 1 DOWNTO 0);

BEGIN

    DUT: phase_lag_top
        GENERIC MAP (
            CLK_FREQ_HZ     => 50000000,
            SAMPLE_PERIOD_S => 0.01,
            SIM_SHORT_DIV   => TRUE,
            G_COEFF_A0      => TB_A0_Q14,
            G_COEFF_A1      => TB_A1_Q14,
            G_COEFF_B1      => TB_B1_Q14
        )
        PORT MAP (
            clk         => tb_clk,
            rst         => tb_rst,
            ref_in      => tb_ref,
            start       => tb_start,
            y_out       => tb_y,
            u_out       => tb_u,
            err_now     => tb_error,
            err_old     => tb_error_prev,
            u_old       => tb_u_prev,
            ctrl_done   => tb_ctrl_done,
            plant_done  => tb_model_done,
            sample_tick => tb_sample
        );

    u_ctrl_test: micro
        PORT MAP (
            start     => unit_start,
            err_in    => unit_error,
            coeff_b1  => tb_b1,
            coeff_a1  => tb_a1,
            coeff_a0  => tb_a0,
            u_out     => unit_control,
            ctrl_done => unit_done,
            clk       => unit_clk
        );

    u_plant_test: plant
        PORT MAP (
            rst       => model_rst,
            sample_en => model_enable,
            u_now     => model_input,
            clk       => model_clk,
            y_out     => model_output,
            plant_done=> model_done,
            y_old1    => model_y_z1,
            y_old2    => model_y_z2,
            u_old1    => model_u_z1,
            u_old2    => model_u_z2
        );

    tb_clk <= NOT tb_clk AFTER TB_CLK_PERIOD / 2 WHEN NOT tb_finished ELSE '0';
    unit_clk  <= NOT unit_clk  AFTER 10 ns;
    model_clk  <= NOT model_clk  AFTER 10 ns;

    stimulus: PROCESS
    BEGIN
        tb_rst   <= '1';
        tb_start <= '0';
        tb_ref   <= (OTHERS => '0');
        WAIT FOR 200 ns;
        tb_rst <= '0';
        REPORT "Reset released";
        REPORT "Controller coefficients: a0=" & INTEGER'IMAGE(CONV_INTEGER(tb_a0)) &
               " a1=" & INTEGER'IMAGE(CONV_INTEGER(tb_a1)) &
               " b1=" & INTEGER'IMAGE(CONV_INTEGER(tb_b1));

        WAIT FOR 300 ns;
        ASSERT tb_y = 0 REPORT "Reset check failed: y_out should be 0" SEVERITY ERROR;
        ASSERT tb_u = 0 REPORT "Reset check failed: u_out should be 0" SEVERITY ERROR;

        tb_start <= '1';
        WAIT FOR 50 us;
        tb_ref <= CONV_SIGNED(STEP_HALF_Q14, WORD_BITS);
        REPORT "Step input applied: ref_in = 8192 (0.5 in Q14)";

        WAIT FOR 10 ms;
        tb_finished <= TRUE;
        WAIT;
    END PROCESS;

    sample_monitor: PROCESS (tb_clk)
    BEGIN
        IF rising_edge(tb_clk) THEN
            IF tb_sample = '1' AND tb_rst = '0' THEN
                tb_sample_index <= tb_sample_index + 1;
                IF tb_sample_index MOD 20 = 0 THEN
                    REPORT "Sample " & INTEGER'IMAGE(tb_sample_index) &
                           " | ref=" & INTEGER'IMAGE(CONV_INTEGER(tb_ref)) &
                           " a0=" & INTEGER'IMAGE(CONV_INTEGER(tb_a0)) &
                           " a1=" & INTEGER'IMAGE(CONV_INTEGER(tb_a1)) &
                           " b1=" & INTEGER'IMAGE(CONV_INTEGER(tb_b1)) &
                           " y=" & INTEGER'IMAGE(CONV_INTEGER(tb_y)) &
                           " u=" & INTEGER'IMAGE(CONV_INTEGER(tb_u)) &
                           " e=" & INTEGER'IMAGE(CONV_INTEGER(tb_error));
                END IF;
            END IF;
        END IF;
    END PROCESS;

    ctrl_unit_test: PROCESS
    BEGIN
        WAIT FOR 500 ns;
        unit_start <= '1';
        WAIT FOR 40 ns;
        unit_start <= '0';

        WAIT UNTIL unit_done = '1' FOR 10 us;
        WAIT FOR 40 ns;

        IF unit_control >= CONV_SIGNED(8228, WORD_BITS) AND unit_control <= CONV_SIGNED(8238, WORD_BITS) THEN
            REPORT "Controller unit test PASS: u=" & INTEGER'IMAGE(CONV_INTEGER(unit_control));
        ELSE
            REPORT "Controller unit test FAIL: expected ~8233, got " &
                   INTEGER'IMAGE(CONV_INTEGER(unit_control)) SEVERITY WARNING;
        END IF;
        WAIT;
    END PROCESS;

    plant_unit_test: PROCESS
    BEGIN
        WAIT FOR 500 ns;
        model_rst <= '1';
        WAIT FOR 40 ns;
        model_rst <= '0';
        WAIT FOR 60 ns;
        model_enable <= '1';
        WAIT FOR 20 ns;
        model_enable <= '0';

        WAIT UNTIL model_done = '1' FOR 1 us;
        WAIT FOR 40 ns;

        IF model_output >= CONV_SIGNED(37, WORD_BITS) AND model_output <= CONV_SIGNED(41, WORD_BITS) THEN
            REPORT "plant unit test PASS: y=" & INTEGER'IMAGE(CONV_INTEGER(model_output));
        ELSE
            REPORT "plant unit test FAIL: expected ~39, got " &
                   INTEGER'IMAGE(CONV_INTEGER(model_output)) SEVERITY WARNING;
        END IF;
        WAIT;
    END PROCESS;

END behavioral;
