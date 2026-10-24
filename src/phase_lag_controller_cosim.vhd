----------------------------------------------------------------------------------
-- Module: phase_lag_controller_cosim
-- Controller-only top for Simulink HDL co-simulation (24-bit datapath)
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY phase_lag_controller_cosim IS
    PORT (
        ref_in   : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        feed     : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        coeff_a0 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        coeff_a1 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        coeff_b1 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        clk      : IN  STD_LOGIC;
        clk_op   : IN  STD_LOGIC;
        u_out    : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        err_out  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        ref_mon  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        feed_mon : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        ctrl_done: OUT STD_LOGIC
    );
END phase_lag_controller_cosim;

ARCHITECTURE structural OF phase_lag_controller_cosim IS

    SIGNAL op_clk_seen    : STD_LOGIC := '0';
    SIGNAL op_clk_prev    : STD_LOGIC := '0';
    SIGNAL launch_pulse  : STD_LOGIC := '0';
    SIGNAL error_wire : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL control_wire     : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL done_wire  : STD_LOGIC;

BEGIN

    PROCESS (clk)
    BEGIN
        IF rising_edge(clk) THEN
            op_clk_seen   <= clk_op;
            op_clk_prev   <= op_clk_seen;
            launch_pulse <= op_clk_seen AND (NOT op_clk_prev);
        END IF;
    END PROCESS;

    error_wire <= ref_in - feed;

    u_controller: micro
        PORT MAP (
            start     => launch_pulse,
            err_in    => error_wire,
            coeff_b1  => coeff_b1,
            coeff_a1  => coeff_a1,
            coeff_a0  => coeff_a0,
            u_out     => control_wire,
            ctrl_done => done_wire,
            clk       => clk
        );

    u_out     <= control_wire;
    err_out   <= error_wire;
    ref_mon   <= ref_in;
    feed_mon  <= feed;
    ctrl_done <= done_wire;

END structural;
