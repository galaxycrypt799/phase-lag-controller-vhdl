----------------------------------------------------------------------------------
-- Module: microdata
-- Data subsystem for Phase Lag controller (24-bit Q14 datapath)
-- u(k) = (B1*u(k-1) + A0*e(k) + A1*e(k-1)) >> 14
----------------------------------------------------------------------------------

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_SIGNED.ALL;
USE WORK.phase_lag_pkg.ALL;

ENTITY microdata IS
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
END microdata;

ARCHITECTURE behavioral OF microdata IS

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

    TYPE reg_bank_t IS ARRAY (0 TO 7) OF SIGNED(WORD_BITS - 1 DOWNTO 0);
    SIGNAL reg_bank : reg_bank_t := (OTHERS => (OTHERS => '0'));

    SIGNAL err_latch  : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL b1_latch  : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL a0_latch  : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL a1_latch  : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');

    FUNCTION register_index(register_select : UNSIGNED(3 DOWNTO 0)) RETURN NATURAL IS
    BEGIN
        IF register_select(3) = '1' THEN
            RETURN 0;
        END IF;
        RETURN CONV_INTEGER(register_select(2 DOWNTO 0));
    END FUNCTION;

BEGIN

    PROCESS (clk)
        VARIABLE operand_a, operand_b, writeback_value      : SIGNED(WORD_BITS - 1 DOWNTO 0);
        VARIABLE alu_output      : SIGNED(WORD_BITS - 1 DOWNTO 0);
        VARIABLE extended_sum  : SIGNED(WORD_BITS DOWNTO 0);
        VARIABLE scaled_product  : SIGNED(ACC_BITS - 1 DOWNTO 0);
        VARIABLE zero_flag, negative_flag, carry_flag : STD_LOGIC;
        CONSTANT ZERO_WORD : SIGNED(WORD_BITS - 1 DOWNTO 0) := (OTHERS => '0');

        PROCEDURE alu (
            zero_flag, negative_flag, carry_flag : OUT STD_LOGIC;
            alu_output                           : OUT SIGNED(WORD_BITS - 1 DOWNTO 0);
            operand_a, operand_b                 : IN  SIGNED(WORD_BITS - 1 DOWNTO 0);
            operation_code                       : IN  UNSIGNED(1 DOWNTO 0)
        ) IS
        BEGIN
            CASE operation_code IS
                WHEN "00" =>
                    extended_sum := (operand_a(WORD_BITS - 1) & operand_a) + (operand_b(WORD_BITS - 1) & operand_b);
                    alu_output := clip_to_word(CONV_SIGNED(CONV_INTEGER(extended_sum), ACC_BITS));
                    carry_flag := extended_sum(WORD_BITS);
                    zero_flag := '0';
                    IF alu_output = ZERO_WORD THEN zero_flag := '1'; END IF;
                    negative_flag := alu_output(WORD_BITS - 1);

                WHEN "01" =>
                    extended_sum := (operand_a(WORD_BITS - 1) & operand_a) - (operand_b(WORD_BITS - 1) & operand_b);
                    alu_output := clip_to_word(CONV_SIGNED(CONV_INTEGER(extended_sum), ACC_BITS));
                    carry_flag := '0';
                    zero_flag := '0';
                    IF alu_output = ZERO_WORD THEN zero_flag := '1'; END IF;
                    negative_flag := alu_output(WORD_BITS - 1);

                WHEN "10" =>
                    scaled_product := operand_a * operand_b;
                    scaled_product := q14_shift_down(scaled_product);
                    alu_output := clip_to_word(scaled_product);
                    carry_flag := '0';
                    zero_flag := '0';
                    IF alu_output = ZERO_WORD THEN zero_flag := '1'; END IF;
                    negative_flag := alu_output(WORD_BITS - 1);

                WHEN "11" =>
                    extended_sum := (operand_a(WORD_BITS - 1) & operand_a) + CONV_SIGNED(1, WORD_BITS + 1);
                    alu_output := clip_to_word(CONV_SIGNED(CONV_INTEGER(extended_sum), ACC_BITS));
                    carry_flag := '0';
                    zero_flag := '0';
                    IF alu_output = ZERO_WORD THEN zero_flag := '1'; END IF;
                    negative_flag := alu_output(WORD_BITS - 1);

                WHEN OTHERS =>
                    alu_output := (OTHERS => '0');
                    zero_flag := '0';
                    negative_flag := '0';
                    carry_flag := '0';
            END CASE;
        END PROCEDURE;

    BEGIN
        operand_a := reg_bank(register_index(fld_a));
        operand_b := reg_bank(register_index(fld_b));

        IF control_con = '0' THEN
            alu(zero_flag, negative_flag, carry_flag, alu_output, operand_a, operand_b, alu_op);
            zero <= zero_flag;
            neg  <= negative_flag;
            cy   <= carry_flag;
        END IF;

        CASE selr_in IS
            WHEN "000" => writeback_value := alu_output;
            WHEN "001" => writeback_value := err_latch;
            WHEN "010" => writeback_value := b1_latch;
            WHEN "011" => writeback_value := a1_latch;
            WHEN "100" => writeback_value := a0_latch;
            WHEN OTHERS => writeback_value := (OTHERS => '0');
        END CASE;

        IF rising_edge(clk) THEN
            IF ldr_in = '1' THEN err_latch <= err_in; END IF;
            IF ldr_b1 = '1' THEN b1_latch <= coeff_b1; END IF;
            IF ldr_a1 = '1' THEN a1_latch <= coeff_a1; END IF;
            IF ldr_a0 = '1' THEN a0_latch <= coeff_a0; END IF;
            IF ldr_out = '1' THEN u_out <= alu_output; END IF;
            IF ldrf = '1' THEN reg_bank(register_index(fld_c)) <= writeback_value; END IF;
        END IF;
    END PROCESS;

END behavioral;
