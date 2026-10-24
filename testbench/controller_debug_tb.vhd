LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY controller_debug_tb IS
END controller_debug_tb;

ARCHITECTURE sim OF controller_debug_tb IS
    SIGNAL dbg_ref      : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(16384, WORD_BITS);
    SIGNAL dbg_feed     : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(0, WORD_BITS);
    SIGNAL dbg_a0       : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(3277, WORD_BITS);
    SIGNAL dbg_a1       : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(-328, WORD_BITS);
    SIGNAL dbg_b1       : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(32440, WORD_BITS);
    SIGNAL dbg_clk      : STD_LOGIC := '0';
    SIGNAL dbg_clk_op   : STD_LOGIC := '0';
    SIGNAL dbg_u        : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL dbg_err      : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL dbg_ref_mon  : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL dbg_feed_mon : SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL dbg_done     : STD_LOGIC;
BEGIN
    dut: ENTITY work.phase_lag_controller_cosim
        PORT MAP (
            ref_in    => dbg_ref,
            feed      => dbg_feed,
            coeff_a0  => dbg_a0,
            coeff_a1  => dbg_a1,
            coeff_b1  => dbg_b1,
            clk       => dbg_clk,
            clk_op    => dbg_clk_op,
            u_out     => dbg_u,
            err_out   => dbg_err,
            ref_mon   => dbg_ref_mon,
            feed_mon  => dbg_feed_mon,
            ctrl_done => dbg_done
        );

    dbg_clk <= NOT dbg_clk AFTER 10 ns;

    PROCESS
    BEGIN
        WAIT FOR 100 ns;
        dbg_clk_op <= '1';
        WAIT FOR 40 ns;
        dbg_clk_op <= '0';
        WAIT FOR 800 ns;
        REPORT "case feed=0: ref_mon=" & INTEGER'IMAGE(CONV_INTEGER(dbg_ref_mon)) &
               " feed_mon=" & INTEGER'IMAGE(CONV_INTEGER(dbg_feed_mon)) &
               " err_out=" & INTEGER'IMAGE(CONV_INTEGER(dbg_err)) &
               " u_out=" & INTEGER'IMAGE(CONV_INTEGER(dbg_u));

        dbg_feed <= CONV_SIGNED(16384, WORD_BITS);
        WAIT FOR 100 ns;
        dbg_clk_op <= '1';
        WAIT FOR 40 ns;
        dbg_clk_op <= '0';
        WAIT FOR 800 ns;
        REPORT "case feed=16384: ref_mon=" & INTEGER'IMAGE(CONV_INTEGER(dbg_ref_mon)) &
               " feed_mon=" & INTEGER'IMAGE(CONV_INTEGER(dbg_feed_mon)) &
               " err_out=" & INTEGER'IMAGE(CONV_INTEGER(dbg_err)) &
               " u_out=" & INTEGER'IMAGE(CONV_INTEGER(dbg_u));

        WAIT;
    END PROCESS;
END sim;
