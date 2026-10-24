----------------------------------------------------------------------------------
-- Module: phase_lag_top
-- Closed-loop Phase Lag controller + second-order plant
-- e(k) = ref_in - y(k), computed at top level
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY phase_lag_top IS
    GENERIC (
        CLK_FREQ_HZ     : NATURAL := 50000000;
        SAMPLE_PERIOD_S : REAL    := 0.01;
        SIM_SHORT_DIV   : BOOLEAN := TRUE;
        G_COEFF_A0      : INTEGER := 16465;
        G_COEFF_A1      : INTEGER := -16301;
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
END phase_lag_top;

ARCHITECTURE structural OF phase_lag_top IS

    FUNCTION derive_sample_ticks RETURN NATURAL IS
        VARIABLE sample_tick_count : REAL;
    BEGIN
        IF SIM_SHORT_DIV THEN
            RETURN 500;
        END IF;
        sample_tick_count := REAL(CLK_FREQ_HZ) * SAMPLE_PERIOD_S;
        IF sample_tick_count < 2.0 THEN
            RETURN 2;
        END IF;
        RETURN NATURAL(sample_tick_count);
    END FUNCTION;

    CONSTANT SAMPLE_TICKS : NATURAL := derive_sample_ticks;

    TYPE loop_state_t IS (ST_IDLE, ST_CTRL, ST_ARM_PLANT, ST_WAIT_PLANT);
    SIGNAL loop_state           : loop_state_t := ST_IDLE;

    SIGNAL sample_counter        : NATURAL RANGE 0 TO SAMPLE_TICKS := 0;
    SIGNAL sample_tick_i        : STD_LOGIC := '0';

    SIGNAL plant_y           : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL control_u           : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL loop_error           : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL prev_error          : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL prev_control          : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');

    SIGNAL controller_start      : STD_LOGIC := '0';
    SIGNAL controller_u          : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL controller_done_i     : STD_LOGIC;
    SIGNAL controller_done_seen : STD_LOGIC := '0';
    SIGNAL model_step        : STD_LOGIC := '0';
    SIGNAL model_done_i    : STD_LOGIC;

    SIGNAL y_hist1_mon, y_hist2_mon : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL u_hist1_mon, u_hist2_mon : SIGNED(WORD_BITS - 1 DOWNTO 0);

BEGIN

    y_out       <= plant_y;
    u_out       <= control_u;
    err_now     <= loop_error;
    err_old     <= prev_error;
    u_old       <= prev_control;
    ctrl_done   <= controller_done_i;
    plant_done  <= model_done_i;
    sample_tick <= sample_tick_i;

    u_controller: micro
        PORT MAP (
            start     => controller_start,
            err_in    => loop_error,
            coeff_b1  => CONV_SIGNED(G_COEFF_B1, WORD_BITS),
            coeff_a1  => CONV_SIGNED(G_COEFF_A1, WORD_BITS),
            coeff_a0  => CONV_SIGNED(G_COEFF_A0, WORD_BITS),
            u_out     => controller_u,
            ctrl_done => controller_done_i,
            clk       => clk
        );

    u_plant: plant
        PORT MAP (
            rst       => rst,
            sample_en => model_step,
            u_now     => control_u,
            clk       => clk,
            y_out     => plant_y,
            plant_done=> model_done_i,
            y_old1    => y_hist1_mon,
            y_old2    => y_hist2_mon,
            u_old1    => u_hist1_mon,
            u_old2    => u_hist2_mon
        );

    -- Sample tick + closed-loop sequencer (single process avoids tick/FSM race)
    PROCESS (clk)
        VARIABLE sample_due : BOOLEAN := FALSE;
    BEGIN
        IF rising_edge(clk) THEN
            controller_start <= '0';
            model_step   <= '0';
            sample_tick_i   <= '0';
            sample_due := FALSE;

            IF controller_done_i = '1' THEN
                controller_done_seen <= '1';
            END IF;

            IF rst = '1' THEN
                sample_counter        <= 0;
                loop_state           <= ST_IDLE;
                loop_error           <= (OTHERS => '0');
                prev_error          <= (OTHERS => '0');
                control_u           <= (OTHERS => '0');
                prev_control          <= (OTHERS => '0');
                controller_done_seen <= '0';
            ELSE
                IF sample_counter = SAMPLE_TICKS - 1 THEN
                    sample_counter   <= 0;
                    sample_tick_i   <= '1';
                    sample_due := TRUE;
                ELSE
                    sample_counter <= sample_counter + 1;
                END IF;

                CASE loop_state IS
                    WHEN ST_IDLE =>
                        IF start = '1' AND sample_due THEN
                            loop_error      <= ref_in - plant_y;
                            controller_start <= '1';
                            loop_state      <= ST_CTRL;
                        END IF;

                    WHEN ST_CTRL =>
                        IF controller_done_seen = '1' THEN
                            controller_done_seen <= '0';
                            prev_control          <= control_u;
                            control_u           <= controller_u;
                            prev_error          <= loop_error;
                            loop_state           <= ST_ARM_PLANT;
                        END IF;

                    WHEN ST_ARM_PLANT =>
                        model_step <= '1';
                        loop_state    <= ST_WAIT_PLANT;

                    WHEN ST_WAIT_PLANT =>
                        IF model_done_i = '1' THEN
                            loop_state <= ST_IDLE;
                        END IF;

                    WHEN OTHERS =>
                        loop_state <= ST_IDLE;
                END CASE;
            END IF;
        END IF;
    END PROCESS;

END structural;
