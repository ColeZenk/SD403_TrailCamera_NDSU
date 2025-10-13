library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity top is
    Port (
        clk     : in  STD_LOGIC;         -- 27MHz or 24MHz clock input
        rst_n   : in  STD_LOGIC;         -- Active-low reset
        led     : out STD_LOGIC_VECTOR(7 downto 0) -- 8 onboard LEDs
    );
end top;

architecture Behavioral of top is
    signal counter : unsigned(23 downto 0) := (others => '0');
begin

    process(clk, rst_n)
    begin
        if rst_n = '0' then
            counter <= (others => '0');
        elsif rising_edge(clk) then
            counter <= counter + 1;
        end if;
    end process;

    -- Simple LED demo: rotate pattern every ~0.5s
    led <= std_logic_vector(counter(22 downto 15));

end Behavioral;