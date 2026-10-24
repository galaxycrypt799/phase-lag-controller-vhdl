----------------------------------------------------------------------------------
-- Module: micro
-- CPU top: microctrl + microdata for Phase Lag controller
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY micro IS
    PORT (
        start                                  : IN  STD_LOGIC;
        err_in                                 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        coeff_b1, coeff_a1, coeff_a0           : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        u_out                                  : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        ctrl_done                              : OUT STD_LOGIC;
        clk                                    : IN  STD_LOGIC
    );
END micro;

ARCHITECTURE structural OF micro IS

    SIGNAL fld_a, fld_b, fld_c   : UNSIGNED(3 DOWNTO 0);
    SIGNAL alu_op                : UNSIGNED(1 DOWNTO 0);
    SIGNAL zero, neg, cy         : STD_LOGIC;
    SIGNAL ldr_in, ldr_out, ldrf : STD_LOGIC;
    SIGNAL selr_in               : UNSIGNED(2 DOWNTO 0);
    SIGNAL ldr_b1, ldr_a1, ldr_a0 : STD_LOGIC;
    SIGNAL control_con           : STD_LOGIC;

BEGIN

    u1: microdata
        PORT MAP (
            err_in      => err_in,
            coeff_b1    => coeff_b1,
            coeff_a1    => coeff_a1,
            coeff_a0    => coeff_a0,
            fld_a       => fld_a,
            fld_b       => fld_b,
            fld_c       => fld_c,
            alu_op      => alu_op,
            ldr_in      => ldr_in,
            ldr_b1      => ldr_b1,
            ldr_a1      => ldr_a1,
            ldr_a0      => ldr_a0,
            ldr_out     => ldr_out,
            selr_in     => selr_in,
            ldrf        => ldrf,
            zero        => zero,
            neg         => neg,
            cy          => cy,
            u_out       => u_out,
            clk         => clk,
            control_con => control_con
        );

    u2: microctrl
        GENERIC MAP (rom_depth => CTRL_STORE_SIZE)
        PORT MAP (
            start       => start,
            zero        => zero,
            neg         => neg,
            cy          => cy,
            fld_a       => fld_a,
            fld_b       => fld_b,
            fld_c       => fld_c,
            alu_op      => alu_op,
            ldr_in      => ldr_in,
            ldr_b1      => ldr_b1,
            ldr_a1      => ldr_a1,
            ldr_a0      => ldr_a0,
            ldr_out     => ldr_out,
            selr_in     => selr_in,
            ldrf        => ldrf,
            ctrl_done   => ctrl_done,
            clk         => clk,
            control_con => control_con
        );

END structural;
