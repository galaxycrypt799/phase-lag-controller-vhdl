----------------------------------------------------------------------------------
-- Module: plant
-- Second-order plant G(s) = 100 / (s^2 + 10s + 100), Tustin @ Ts = 0.01 s
-- y(k) = (31056*y1 - 14827*y2 + 39*u + 78*u1 + 39*u2) >> 14
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY plant IS
    PORT (
        rst, sample_en : IN  STD_LOGIC;
        u_now          : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
        clk            : IN  STD_LOGIC;
        y_out          : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        plant_done     : OUT STD_LOGIC;
        y_old1, y_old2 : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
        u_old1, u_old2 : OUT SIGNED(WORD_BITS - 1 DOWNTO 0)
    );
END plant;

ARCHITECTURE behavioral OF plant IS

    FUNCTION clip_to_word(accumulator_value : SIGNED(ACC_BITS - 1 DOWNTO 0)) RETURN SIGNED IS
        CONSTANT MAX24 : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(8388607, WORD_BITS);
        CONSTANT MIN24 : SIGNED(WORD_BITS - 1 DOWNTO 0) := CONV_SIGNED(-8388608, WORD_BITS);
    BEGIN
        IF accumulator_value > CONV_SIGNED(CONV_INTEGER(MAX24), ACC_BITS) THEN
            RETURN MAX24;
        ELSIF accumulator_value < CONV_SIGNED(CONV_INTEGER(MIN24), ACC_BITS) THEN
            RETURN MIN24;
        ELSE
            RETURN accumulator_value(WORD_BITS - 1 DOWNTO 0);
        END IF;
    END FUNCTION;

    SIGNAL y_z1, y_z2 : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL u_z1, u_z2 : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL plant_y          : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL done_pulse       : STD_LOGIC := '0';

BEGIN

    y_out      <= plant_y;
    y_old1     <= y_z1;
    y_old2     <= y_z2;
    u_old1     <= u_z1;
    u_old2     <= u_z2;
    plant_done <= done_pulse;

    PROCESS (clk)
        VARIABLE mac_accum : SIGNED(ACC_BITS - 1 DOWNTO 0);
    BEGIN
        IF rising_edge(clk) THEN
            done_pulse <= '0';

            IF rst = '1' THEN
                plant_y  <= (OTHERS => '0');
                y_z1 <= (OTHERS => '0');
                y_z2 <= (OTHERS => '0');
                u_z1 <= (OTHERS => '0');
                u_z2 <= (OTHERS => '0');
            ELSIF sample_en = '1' THEN
                mac_accum := MODEL_P1_Q14 * y_z1;
                mac_accum := mac_accum + MODEL_P2_Q14 * y_z2;
                mac_accum := mac_accum + MODEL_B0_Q14 * u_now;
                mac_accum := mac_accum + MODEL_B1_Q14 * u_z1;
                mac_accum := mac_accum + MODEL_B2_Q14 * u_z2;
                mac_accum := q14_shift_down(mac_accum);

                plant_y  <= clip_to_word(mac_accum);
                y_z2 <= y_z1;
                y_z1 <= clip_to_word(mac_accum);
                u_z2 <= u_z1;
                u_z1 <= u_now;
                done_pulse <= '1';
            END IF;
        END IF;
    END PROCESS;

END behavioral;
