-- #################################################################################################
-- # << wb_spi_bridge - Transparent Wishbone-to-SPI Bridge - Simple Testbench >>                   #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # wb_spi_bridge - https://github.com/stnolting/wb_spi_bridge                (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wb_spi_bridge_tb is
end wb_spi_bridge_tb;

architecture wb_spi_bridge_tb_rtl of wb_spi_bridge_tb is

  -- generators --
  signal clk_gen  : std_ulogic := '0';
  signal rstn_gen : std_ulogic := '0';

  -- Component: Wishbone to SPI Bridge --
  component wb_spi_bridge
    generic (
      SPI_CLK_DIV  : positive; -- clock divider: f(spi_clk_o) = f(clk_i) / SPI_CLK_DIV, min 2
      SPI_ABYTES   : positive; -- number of address bytes in SPI protocol, 1..4
      SPI_CPHA     : std_ulogic; -- clock phase
      SPI_CPOL     : std_ulogic; -- idle polarity
      WB_ADDR_BASE : std_ulogic_vector(31 downto 0); -- module base address, size-aligned
      WB_ADDR_SIZE : positive  -- module address space in bytes
    );
    port (
      -- wishbone interface --
      wb_clk_i   : in  std_ulogic; -- clock
      wb_rstn_i  : in  std_ulogic; -- reset, async, low-active
      wb_srstn_i : in  std_ulogic; -- reset, SYNC, low-active
      wb_adr_i   : in  std_ulogic_vector(31 downto 0); -- address
      wb_dat_i   : in  std_ulogic_vector(31 downto 0); -- read data
      wb_dat_o   : out std_ulogic_vector(31 downto 0); -- write data
      wb_we_i    : in  std_ulogic; -- read/write
      wb_sel_i   : in  std_ulogic_vector(03 downto 0); -- byte enable
      wb_stb_i   : in  std_ulogic; -- strobe
      wb_cyc_i   : in  std_ulogic; -- valid cycle
      wb_ack_o   : out std_ulogic; -- transfer acknowledge
      wb_err_o   : out std_ulogic; -- transfer error
      -- SPI interface --
      spi_csn_o  : out std_ulogic; -- chip-select, low-active
      spi_clk_o  : out std_ulogic; -- serial clock
      spi_data_i : in  std_ulogic; -- device data output
      spi_data_o : out std_ulogic  -- controller data output
    );
  end component;

  -- Wishbone bus --
  type wishbone_t is record
    addr  : std_ulogic_vector(31 downto 0);
    wdata : std_ulogic_vector(31 downto 0);
    rdata : std_ulogic_vector(31 downto 0);
    we    : std_ulogic;
    sel   : std_ulogic_vector(03 downto 0);
    stb   : std_ulogic;
    cyc   : std_ulogic;
    ack   : std_ulogic;
    err   : std_ulogic;
  end record;
  signal wb : wishbone_t;

begin

  -- Generators -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen  <= not clk_gen after 5 ns;
  rstn_gen <= '0', '1' after 60 ns;


  -- DUT ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_spi_bridge_inst: wb_spi_bridge
  generic map (
    SPI_CLK_DIV  => 4,
    SPI_ABYTES   => 2,
    SPI_CPHA     => '0',
    SPI_CPOL     => '0', -- clock mode 0,0
    WB_ADDR_BASE => x"f0000000",
    WB_ADDR_SIZE => 64*1024
  )
  port map (
    -- wishbone interface --
    wb_clk_i   => clk_gen,
    wb_rstn_i  => rstn_gen,
    wb_srstn_i => '1',
    wb_adr_i   => wb.addr,
    wb_dat_i   => wb.wdata,
    wb_dat_o   => wb.rdata,
    wb_we_i    => wb.we,
    wb_sel_i   => wb.sel,
    wb_stb_i   => wb.stb,
    wb_cyc_i   => wb.cyc,
    wb_ack_o   => wb.ack,
    wb_err_o   => wb.err,
    -- SPI interface --
    spi_csn_o  => open,
    spi_clk_o  => open,
    spi_data_i => '1',
    spi_data_o => open
  );


  -- Stimulus -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  stimulus: process
  begin
    wb.addr  <= x"00000000";
    wb.wdata <= x"00000000";
    wb.we    <= '0';
    wb.sel   <= "0000";
    wb.stb   <= '0';
    wb.cyc   <= '0';
    wait for 100 ns;

    -- module not accessed --
    wb.addr  <= x"80000000";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait for 100 ns;

    -- write word --
    wb.addr  <= x"f0000000";
    wb.wdata <= x"CAFE1234";
    wb.we    <= '1';
    wb.sel   <= "1111";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write half-word 0 --
    wb.addr  <= x"f0000004";
    wb.wdata <= x"0000ABBA";
    wb.we    <= '1';
    wb.sel   <= "0011";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write half-word 1 --
    wb.addr  <= x"f0000006";
    wb.wdata <= x"00001337";
    wb.we    <= '1';
    wb.sel   <= "0011";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write byte 0 --
    wb.addr  <= x"f0000008";
    wb.wdata <= x"000000AA";
    wb.we    <= '1';
    wb.sel   <= "0001";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write byte 1 --
    wb.addr  <= x"f0000009";
    wb.wdata <= x"0000BB00";
    wb.we    <= '1';
    wb.sel   <= "0010";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write byte 2 --
    wb.addr  <= x"f000000a";
    wb.wdata <= x"00CC0000";
    wb.we    <= '1';
    wb.sel   <= "0100";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- write byte 3 --
    wb.addr  <= x"f000000b";
    wb.wdata <= x"DD000000";
    wb.we    <= '1';
    wb.sel   <= "1000";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- read word --
    wb.addr  <= x"f0000000";
    wb.wdata <= x"00000000";
    wb.we    <= '0';
    wb.sel   <= "0000";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.ack = '1');
    wait for 10 ns;

    -- invalid data size --
    wb.addr  <= x"f0000000";
    wb.wdata <= x"00000000";
    wb.we    <= '1';
    wb.sel   <= "1010";
    wb.stb   <= '1';
    wb.cyc   <= '1';
    wait for 10 ns;
    wb.stb   <= '0';
    wait until (wb.err = '1');
    wait for 10 ns;

    wait;
  end process stimulus;


end wb_spi_bridge_tb_rtl;
