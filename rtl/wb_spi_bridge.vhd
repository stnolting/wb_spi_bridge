-- #################################################################################################
-- # << wb_spi_bridge - Transparent Wishbone-to-SPI Bridge >>                                      #
-- # ********************************************************************************************* #
-- # This bridge allows to transparently map a SPI memory into the address space of a processor.   #
-- # The bridge acts as read/write memory hiding all SPI transactions - at the cost of additional  #
-- # latency. The processor can access instructions and data located in the SPI memory allowing    #
-- # execute in place (XIP).                                                                       #
-- #                                                                                               #
-- # Check out the documentation at https://github.com/stnolting/wb_spi_bridge                     #
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

entity wb_spi_bridge is
  generic (
    SPI_CLK_DIV  : positive;   -- clock divider: f(spi_clk_o) = f(clk_i) / 2*SPI_CLK_DIV, min 2
    SPI_ABYTES   : positive;   -- number of address bytes in SPI protocol, 1..4
    SPI_CPHA     : std_ulogic; -- clock phase
    SPI_CPOL     : std_ulogic; -- idle polarity
    WB_ADDR_BASE : std_ulogic_vector(31 downto 0); -- module base address, size-aligned
    WB_ADDR_SIZE : positive    -- module address space in bytes
  );
  port (
    -- wishbone host interface --
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
    -- SPI device interface --
    spi_csn_o  : out std_ulogic; -- chip-select, low-active
    spi_clk_o  : out std_ulogic; -- serial clock
    spi_data_i : in  std_ulogic; -- device data output
    spi_data_o : out std_ulogic  -- controller data output
  );
end wb_spi_bridge;

architecture wb_spi_bridge_rtl of wb_spi_bridge is

  -- Function: Test if input number is a power of two --
  function is_power_of_two_f(input : natural) return boolean is
  begin
    if (input = 1) then -- 2^0
      return true;
    elsif ((input / 2) /= 0) and ((input mod 2) = 0) then
      return true;
    else
      return false;
    end if;
  end function is_power_of_two_f;

  -- internal constants --
  constant addr_mask_c : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(WB_ADDR_SIZE-1, 32));
  constant all_zero_c  : std_ulogic_vector(31 downto 0) := (others => '0');

  -- access check --
  type bus_access_t is record
    match : std_ulogic; -- address match
    req   : std_ulogic; -- valid request
    size  : std_ulogic_vector(1 downto 0); -- data size
    wdata : std_ulogic_vector(31 downto 0);
    addr  : std_ulogic_vector((SPI_ABYTES*8)-1 downto 0);
    valid : std_ulogic; -- valid data size
  end record;
  signal bus_access : bus_access_t;

  -- bus access arbiter --
  type ctrl_state_t is (S_RESET, S_IDLE, S_READ, S_WRITE, S_BUSY, S_ERROR);
  type ctrl_t is record
    state : ctrl_state_t;
    addr  : std_ulogic_vector((SPI_ABYTES*8)-1 downto 0);
    wdata : std_ulogic_vector(31 downto 0);
    size  : std_ulogic_vector(01 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- LINK interface --
  type link_if_t is record
    srstn : std_ulogic; -- sync reset, low-active
    start : std_ulogic; -- trigger new transmission
    rw    : std_ulogic; -- 0 = read, 1 = write
    size  : std_ulogic_vector(01 downto 0); -- data size: 00 = byte, 01 = half, 10 = word; relevant for writes only
    busy  : std_ulogic; -- transmission in progress when set
    addr  : std_ulogic_vector((SPI_ABYTES*8)-1 downto 0); -- access address (byte-address!)
    wdata : std_ulogic_vector(31 downto 0); -- write data
    rdata : std_ulogic_vector(31 downto 0); -- read data
  end record;
  signal link_if : link_if_t;

  -- Component: SPI LINK --
  component wb_spi_bridge_link
    generic (
      SPI_CLK_DIV : positive;
      SPI_ABITS   : positive;
      SPI_CPHA    : std_ulogic;
      SPI_CPOL    : std_ulogic
    );
    port (
      -- global control --
      clk_i      : in  std_ulogic;
      rstn_i     : in  std_ulogic;
      -- operation control --
      op_start_i : in  std_ulogic;
      op_rw_i    : in  std_ulogic;
      op_size_i  : in  std_ulogic_vector(01 downto 0);
      op_busy_o  : out std_ulogic;
      op_addr_i  : in  std_ulogic_vector(SPI_ABITS-1 downto 0);
      op_wdata_i : in  std_ulogic_vector(31 downto 0);
      op_rdata_o : out std_ulogic_vector(31 downto 0);
      -- SPI interface --
      spi_csn_o  : out std_ulogic;
      spi_clk_o  : out std_ulogic;
      spi_data_i : in  std_ulogic;
      spi_data_o : out std_ulogic
    );
  end component;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((SPI_ABYTES < 1) or (SPI_ABYTES > 4)) report "wb_spi_bridge config ERROR: Number of address bytes <SPI_ABYTES> has to be 1, 2, 3 or 4." severity error;
  assert not (is_power_of_two_f(WB_ADDR_SIZE) = false) report "wb_spi_bridge config ERROR: Module address space <WB_ADDR_SIZE> has to be a power of two." severity error;
  assert not ((WB_ADDR_BASE and addr_mask_c) /= all_zero_c) report "wb_spi_bridge config ERROR: Module base address <WB_ADDR_BASE> has to be aligned to it's address space <WB_ADDR_SIZE>." severity error;


  -- Access Check ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access.match <= '1' when ((wb_adr_i and (not addr_mask_c)) = (WB_ADDR_BASE and (not addr_mask_c))) else '0';
  bus_access.req   <= bus_access.match and wb_cyc_i and wb_stb_i;

  -- data quantity --
  quantity_check: process(wb_we_i, wb_sel_i, wb_dat_i, wb_adr_i)
  begin
    -- defaults --
    bus_access.wdata <= wb_dat_i;
    bus_access.addr  <= wb_adr_i((SPI_ABYTES*8)-1 downto 0);

    -- alignment --
    if (wb_we_i = '0') then -- read
      bus_access.size  <= "10"; -- full word
      bus_access.valid <= '1';
      bus_access.addr(1 downto 0) <= "00";
    else -- write
      case wb_sel_i is
        when "1111" => -- full word
          bus_access.size  <= "10";
          bus_access.valid <= '1';
        when "0011" => -- half-word 0
          bus_access.size  <= "01";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 16) <= wb_dat_i(15 downto 0);
          bus_access.addr(1 downto 0) <= "10";
        when "1100" => -- half-word 1
          bus_access.size  <= "01";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 16) <= wb_dat_i(31 downto 16);
          bus_access.addr(1 downto 0) <= "00";
        when "0001" => -- single byte 0
          bus_access.size  <= "00";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 24) <= wb_dat_i(7 downto 0);
          bus_access.addr(1 downto 0) <= "11";
        when "0010" => -- single byte 1
          bus_access.size  <= "00";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 24) <= wb_dat_i(15 downto 8);
          bus_access.addr(1 downto 0) <= "10";
        when "0100" => -- single byte 2
          bus_access.size  <= "00";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 24) <= wb_dat_i(23 downto 16);
          bus_access.addr(1 downto 0) <= "01";
        when "1000" => -- single byte 3
          bus_access.size  <= "00";
          bus_access.valid <= '1';
          bus_access.wdata(31 downto 24) <= wb_dat_i(31 downto 24);
          bus_access.addr(1 downto 0) <= "00";
        when others => -- invalid
          bus_access.size  <= "--";
          bus_access.valid <= '0';
      end case;
    end if;
  end process quantity_check;


  -- Bus Access Arbiter ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control_unit: process(wb_rstn_i, wb_clk_i)
  begin
    if (wb_rstn_i = '0') then
      ctrl.state <= S_RESET;
      ctrl.addr  <= (others => '-');
      ctrl.wdata <= (others => '-');
      ctrl.size  <= (others => '-');
      wb_ack_o   <= '0';
      wb_err_o   <= '0';
      wb_dat_o   <= (others => '0');
    elsif rising_edge(wb_clk_i) then
      if (wb_srstn_i = '0') then -- sync reset
        ctrl.state <= S_RESET;
        ctrl.addr  <= (others => '-');
        ctrl.wdata <= (others => '-');
        ctrl.size  <= (others => '-');
        wb_ack_o   <= '0';
        wb_err_o   <= '0';
        wb_dat_o   <= (others => '0');
      else
        -- defaults --
        wb_ack_o <= '0';
        wb_err_o <= '0';
        wb_dat_o <= (others => '0');

        -- fsm --
        case ctrl.state is

          when S_RESET => -- reset sub units
          -- ------------------------------------------------------------
            ctrl.state <= S_IDLE;

          when S_IDLE => -- wait for new bus request
          -- ------------------------------------------------------------
            if (bus_access.req = '1') then
              ctrl.addr  <= bus_access.addr;
              ctrl.size  <= bus_access.size;
              ctrl.wdata <= bus_access.wdata;
              if (bus_access.valid = '1') then -- valid data quantity?
                if (wb_we_i = '1') then -- write
                  ctrl.state <= S_WRITE;
                else -- read
                  ctrl.state <= S_READ;
                end if;
              else
                ctrl.state <= S_ERROR;
              end if;
            end if;

          when S_READ => -- trigger read operation
          -- ------------------------------------------------------------
            ctrl.state <= S_BUSY;

          when S_WRITE => -- trigger read operation
          -- ------------------------------------------------------------
            ctrl.state <= S_BUSY;

          when S_BUSY => -- wait for LINK to complete operation
          -- ------------------------------------------------------------
            if (link_if.busy = '0') then
              if (wb_cyc_i = '1') then -- bus access still active?
                wb_dat_o <= link_if.rdata;
                wb_ack_o <= '1';
              end if;
              ctrl.state <= S_IDLE;
            end if;

          when S_ERROR => -- error!
          -- ------------------------------------------------------------
            if (wb_cyc_i = '1') then -- bus access still active?
              wb_err_o <= '1';
            end if;
            ctrl.state <= S_IDLE;

          when others => -- undefined
          -- ------------------------------------------------------------
            ctrl.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process control_unit;


  -- Link Interface -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- trigger new operations --
  link_if.start <= '1' when (ctrl.state = S_READ) or (ctrl.state = S_WRITE) else '0';

  -- data & address & quantity & read/write --
  link_if.wdata <= ctrl.wdata;
  link_if.addr  <= ctrl.addr;
  link_if.size  <= ctrl.size;
  link_if.rw    <= '1' when (ctrl.state = S_WRITE) else '0';

  -- link SYNC reset --
  link_if.srstn <= '0' when (ctrl.state = S_RESET) else '1';


  -- SPI Link -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_spi_bridge_link_inst: wb_spi_bridge_link
  generic map (
    SPI_CLK_DIV => SPI_CLK_DIV,
    SPI_ABITS   => SPI_ABYTES*8,
    SPI_CPHA    => SPI_CPHA,
    SPI_CPOL    => SPI_CPOL
  )
  port map (
    -- global control --
    clk_i      => wb_clk_i,
    rstn_i     => link_if.srstn,
    -- operation control --
    op_start_i => link_if.start,
    op_rw_i    => link_if.rw,
    op_size_i  => link_if.size,
    op_busy_o  => link_if.busy,
    op_addr_i  => link_if.addr,
    op_wdata_i => link_if.wdata,
    op_rdata_o => link_if.rdata,
    -- SPI interface --
    spi_csn_o  => spi_csn_o,
    spi_clk_o  => spi_clk_o,
    spi_data_i => spi_data_i,
    spi_data_o => spi_data_o
  );


end wb_spi_bridge_rtl;



-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << wb_spi_bridge - Link Layer (Memory Interface Abstraction) >>                               #
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

entity wb_spi_bridge_link is
  generic (
    SPI_CLK_DIV : positive;   -- clock divider: f(spi_clk_o) = f(clk_i) / 2*SPI_CLK_DIV, min 2
    SPI_ABITS   : positive;   -- SPI address bits (1..32)
    SPI_CPHA    : std_ulogic; -- clock phase
    SPI_CPOL    : std_ulogic  -- idle polarity
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- clock
    rstn_i     : in  std_ulogic; -- reset, SYNC, low-active
    -- operation control --
    op_start_i : in  std_ulogic; -- trigger new transmission
    op_rw_i    : in  std_ulogic; -- 0 = read, 1 = write
    op_size_i  : in  std_ulogic_vector(01 downto 0); -- data size: 00 = byte, 01 = half, 10 = word; relevant for writes only
    op_busy_o  : out std_ulogic; -- transmission in progress when set
    op_addr_i  : in  std_ulogic_vector(SPI_ABITS-1 downto 0); -- access address (byte-address!)
    op_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    op_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    -- SPI interface --
    spi_csn_o  : out std_ulogic;
    spi_clk_o  : out std_ulogic;
    spi_data_i : in  std_ulogic;
    spi_data_o : out std_ulogic
  );
end wb_spi_bridge_link;

architecture wb_spi_bridge_link_rtl of wb_spi_bridge_link is

  -- spi memory opcodes -----------------------------------------------------------
  constant cmd_write_c : std_ulogic_vector(7 downto 0) := x"02"; -- write data
  constant cmd_read_c  : std_ulogic_vector(7 downto 0) := x"03"; -- read data
  constant cmd_wren_c  : std_ulogic_vector(7 downto 0) := x"06"; -- write enable
  -- ------------------------------------------------------------------------------

  -- internal constants --
  constant max_bits_c : positive := 8 + SPI_ABITS + 32; -- max number of bits per transmission

  -- link arbiter --
  type ctrl_state_t is (S_IDLE, S_INIT_WR_0, S_INIT_WR_1, S_START, S_BUSY);
  type ctrl_t is record
    state : ctrl_state_t;
    size  : std_ulogic_vector(01 downto 0);
    rw    : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- PHY interface --
  type spi_phy_t is record
    start : std_ulogic; -- trigger new transmission
    busy  : std_ulogic; -- transmission in progress when set
    nbits : std_ulogic_vector(6 downto 0); -- actual number of bits to send
    wdata : std_ulogic_vector(max_bits_c-1 downto 0); -- write data
    rdata : std_ulogic_vector(max_bits_c-1 downto 0); -- read data
  end record;
  signal spi_phy : spi_phy_t;

  -- Component: SPI PHY --
  component wb_spi_bridge_phy
    generic (
      CLK_DIV  : positive;
      MAX_BITS : positive;
      CPHA     : std_ulogic;
      CPOL     : std_ulogic
    );
    port (
      -- global control --
      clk_i      : in  std_ulogic;
      rstn_i     : in  std_ulogic;
      -- operation control --
      op_start_i : in  std_ulogic;
      op_busy_o  : out std_ulogic;
      op_nbits_i : in  std_ulogic_vector(6 downto 0);
      op_wdata_i : in  std_ulogic_vector(MAX_BITS-1 downto 0);
      op_rdata_o : out std_ulogic_vector(MAX_BITS-1 downto 0);
      -- SPI interface --
      spi_csn_o  : out std_ulogic;
      spi_clk_o  : out std_ulogic;
      spi_data_i : in  std_ulogic;
      spi_data_o : out std_ulogic
    );
  end component;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (max_bits_c > 72) report "wb_spi_bridge config ERROR: Total number of SPI bits has to be <= 72." severity error;


  -- Link Arbiter ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0') then
        ctrl.rw    <= '-';
        ctrl.size  <= (others => '-');
        ctrl.state <= S_IDLE;
      else
        case ctrl.state is

          when S_IDLE => -- wait for new access request
          -- ------------------------------------------------------------
            if (op_start_i = '1') then
              if (op_rw_i = '0') then -- read
                ctrl.rw    <= '0';
                ctrl.size  <= "10"; -- full 32-bit word
                ctrl.state <= S_START;
              else -- write
                ctrl.rw    <= '1';
                ctrl.size  <= op_size_i;
                ctrl.state <= S_INIT_WR_0;
              end if;
            end if;

          when S_INIT_WR_0 => -- set write-enable latch
          -- ------------------------------------------------------------
            if (spi_phy.busy = '1') then -- wait for PHY to start
              ctrl.state <= S_INIT_WR_1;
            end if;

          when S_INIT_WR_1 => -- wait for PHY to complete operation
          -- ------------------------------------------------------------
            if (spi_phy.busy = '0') then
              ctrl.state <= S_START;
            end if;

          when S_START => -- start actual data transmission
          -- ------------------------------------------------------------
            if (spi_phy.busy = '1') then -- wait for PHY to start
              ctrl.state <= S_BUSY;
            end if;

          when S_BUSY => -- wait for PHY to complete operation
          -- ------------------------------------------------------------
            if (spi_phy.busy = '0') then
              ctrl.state <= S_IDLE;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            ctrl.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process control_unit;

  -- pending operation --
  op_busy_o <= '0' when (ctrl.state = S_IDLE) else '1';


  -- PHY Interface --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- trigger new transmission --
  spi_phy.start <= '1' when (ctrl.state = S_INIT_WR_0) or (ctrl.state = S_START) else '0';

  -- write data --
  phy_write_data: process(op_addr_i, op_wdata_i, ctrl)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    -- defaults --
    spi_phy.wdata <= (others => '0');
    spi_phy.wdata(spi_phy.wdata'left-8 downto (spi_phy.wdata'left-8)-(SPI_ABITS-1)) <= op_addr_i; -- address
    spi_phy.wdata(spi_phy.wdata'left-(8+SPI_ABITS) downto 0) <= op_wdata_i; -- data word

    -- operation / data quantity --
    if (ctrl.state = S_INIT_WR_0) or (ctrl.state = S_INIT_WR_1) then -- set write-enable latch
      spi_phy.nbits <= std_ulogic_vector(to_unsigned(8, 7));
      spi_phy.wdata(spi_phy.wdata'left downto spi_phy.wdata'left-7) <= cmd_wren_c;
    else -- data transmission
      tmp_v := ctrl.rw & ctrl.size;
      case tmp_v is
        when "100" => -- write byte
          spi_phy.nbits <= std_ulogic_vector(to_unsigned(8+SPI_ABITS+8, 7));
          spi_phy.wdata(spi_phy.wdata'left downto spi_phy.wdata'left-7) <= cmd_write_c;
        when "101" => -- write half-word
          spi_phy.nbits <= std_ulogic_vector(to_unsigned(8+SPI_ABITS+16, 7));
          spi_phy.wdata(spi_phy.wdata'left downto spi_phy.wdata'left-7) <= cmd_write_c;
        when "110" | "111" => -- write word
          spi_phy.nbits <= std_ulogic_vector(to_unsigned(8+SPI_ABITS+32, 7));
          spi_phy.wdata(spi_phy.wdata'left downto spi_phy.wdata'left-7) <= cmd_write_c;
        when others => -- read (always full word)
          spi_phy.nbits <= std_ulogic_vector(to_unsigned(8+SPI_ABITS+32, 7));
          spi_phy.wdata(spi_phy.wdata'left downto spi_phy.wdata'left-7) <= cmd_read_c;
      end case;
    end if;
  end process phy_write_data;

  -- read data --
  op_rdata_o <= spi_phy.rdata(31 downto 0);


  -- SPI PHY --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_spi_bridge_phy_inst: wb_spi_bridge_phy
  generic map (
    CLK_DIV  => SPI_CLK_DIV,
    MAX_BITS => max_bits_c,
    CPHA     => SPI_CPHA,
    CPOL     => SPI_CPOL
  )
  port map (
    -- global control --
    clk_i      => clk_i,
    rstn_i     => rstn_i,
    -- operation control --
    op_start_i => spi_phy.start,
    op_busy_o  => spi_phy.busy,
    op_nbits_i => spi_phy.nbits,
    op_wdata_i => spi_phy.wdata,
    op_rdata_o => spi_phy.rdata,
    -- SPI interface --
    spi_csn_o  => spi_csn_o,
    spi_clk_o  => spi_clk_o,
    spi_data_i => spi_data_i,
    spi_data_o => spi_data_o
  );


end wb_spi_bridge_link_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << wb_spi_bridge - Physical Layer (SPI Protocol Engine) >>                                    #
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

entity wb_spi_bridge_phy is
  generic (
    CLK_DIV  : positive;   -- clock divider: f(spi_clk_o) = f(clk_i) / 2*SPI_CLK_DIV, min 2
    MAX_BITS : positive;   -- maximum number of bits per transmission, max 128
    CPHA     : std_ulogic; -- clock phase
    CPOL     : std_ulogic  -- idle polarity
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- clock
    rstn_i     : in  std_ulogic; -- reset, SYNC, low-active
    -- operation control --
    op_start_i : in  std_ulogic; -- trigger new transmission
    op_busy_o  : out std_ulogic; -- transmission in progress when set
    op_nbits_i : in  std_ulogic_vector(6 downto 0); -- actual number of bits to transmit
    op_wdata_i : in  std_ulogic_vector(MAX_BITS-1 downto 0); -- write data
    op_rdata_o : out std_ulogic_vector(MAX_BITS-1 downto 0); -- read data
    -- SPI interface --
    spi_csn_o  : out std_ulogic;
    spi_clk_o  : out std_ulogic;
    spi_data_i : in  std_ulogic;
    spi_data_o : out std_ulogic
  );
end wb_spi_bridge_phy;

architecture wb_spi_bridge_phy_rtl of wb_spi_bridge_phy is

  -- clock generator --
  signal gen_clk_div : integer range 0 to CLK_DIV-1;
  signal gen_spi_clk : std_ulogic;

  -- controller --
  type ctrl_state_t is (S_IDLE, S_START, S_RTX_A, S_RTX_B, S_DONE, S_PAUSE);
  type ctrl_t is record
    state  : ctrl_state_t;
    sreg   : std_ulogic_vector(MAX_BITS-1 downto 0);
    bitcnt : std_ulogic_vector(6 downto 0);
    sync   : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (CLK_DIV < 2) report "wb_spi_bridge config ERROR: SPI clock divider <SPI_CLK_DIV> has to be >= 2." severity error;


  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0')then
        gen_clk_div <= 0;
        gen_spi_clk <= '0';
      else
        if (gen_clk_div = CLK_DIV-1) then
          gen_clk_div <= 0;
          gen_spi_clk <= '1';
        else
          gen_clk_div <= gen_clk_div + 1;
          gen_spi_clk <= '0';
        end if;
      end if;
    end if;
  end process clock_gen;


  -- Serial Interface Control Unit ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0') then
        spi_clk_o   <= CPOL;
        spi_csn_o   <= '1';
        ctrl.state  <= S_IDLE;
        ctrl.sreg   <= (others => '-');
        ctrl.bitcnt <= (others => '-');
        ctrl.sync   <= '0';
      else
        -- defaults --
        spi_clk_o <= CPOL;
        ctrl.sync <= spi_data_i;

        -- fsm --
        case ctrl.state is

          when S_IDLE => -- wait for new transmission trigger
          -- ------------------------------------------------------------
            spi_csn_o   <= '1';
            ctrl.bitcnt <= op_nbits_i;
            if (op_start_i = '1') then
              spi_csn_o  <= '0';
              ctrl.sreg  <= op_wdata_i;
              ctrl.state <= S_START;
            end if;

          when S_START => -- sync start of transmission
          -- ------------------------------------------------------------
            spi_csn_o <= '0';
            if (gen_spi_clk = '1') then
              ctrl.state <= S_RTX_A;
            end if;

          when S_RTX_A => -- first half of bit transmission
          -- ------------------------------------------------------------
            spi_csn_o <= '0';
            spi_clk_o <= CPHA xor CPOL;
            if (gen_spi_clk = '1') then
              ctrl.bitcnt <= std_ulogic_vector(unsigned(ctrl.bitcnt) - 1);
              ctrl.state  <= S_RTX_B;
            end if;

          when S_RTX_B => -- second half of bit transmission
          -- ------------------------------------------------------------
            spi_csn_o <= '0';
            spi_clk_o <= CPHA xnor CPOL;
            if (gen_spi_clk = '1') then
              ctrl.sreg <= ctrl.sreg(ctrl.sreg'left-1 downto 0) & ctrl.sync;
              if (ctrl.bitcnt = "0000000") then -- all bits transferred?
                ctrl.state <= S_DONE; -- transmission done
              else
                ctrl.state <= S_RTX_A; -- next bit
              end if;
            end if;

          when S_DONE => -- transmission done
          -- ------------------------------------------------------------
            spi_csn_o <= '0';
            if (gen_spi_clk = '1') then
              ctrl.state <= S_PAUSE;
            end if;

          when S_PAUSE => -- inter-symbol delay
          -- ------------------------------------------------------------
            spi_csn_o <= '1';
            if (gen_spi_clk = '1') then
              ctrl.state <= S_IDLE;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            spi_csn_o  <= '1';
            ctrl.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process control_unit;

  -- serial unit busy --
  op_busy_o <= '0' when (ctrl.state = S_IDLE) else '1';

  -- serial data output --
  spi_data_o <= ctrl.sreg(ctrl.sreg'left);

  -- read data --
  op_rdata_o <= ctrl.sreg;


end wb_spi_bridge_phy_rtl;
