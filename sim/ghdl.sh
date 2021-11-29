#!/usr/bin/env bash

set -e

# Analise sources
ghdl -a ../rtl/wb_spi_bridge.vhd
ghdl -a wb_spi_bridge_tb.vhd

# Elaborate top entity
ghdl -e wb_spi_bridge_tb

# Run simulation
ghdl -e wb_spi_bridge_tb
ghdl -r wb_spi_bridge_tb --stop-time=1ms --vcd=wb_spi_bridge.vcd
