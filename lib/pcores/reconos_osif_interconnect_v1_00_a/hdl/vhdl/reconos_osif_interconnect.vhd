--                                                        ____  _____
--                            ________  _________  ____  / __ \/ ___/
--                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
--                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
--                         /_/   \___/\___/\____/_/ /_/\____//____/
-- 
-- ======================================================================
--
--   title:        IP-Core - OSIF Interconnect
--
--   project:      ReconOS
--   author:       Christoph Rüthing, University of Paderborn
--   description:  The interconnect allows to implement a direct fifo
--                 connection to interface with resources implemented
--                 in hardware. It routes the request based on the
--                 implementation mode of resources to either the
--                 processor interface or the according hardware
--                 implementation.
--
-- ======================================================================

<<reconos_preproc>>

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_defs.all;

library reconos_fifo_sync_v1_00_a;
use reconos_fifo_sync_v1_00_a.all;

library reconos_osif_interconnect_v1_00_a;
use reconos_osif_interconnect_v1_00_a.all;

entity reconos_osif_interconnect is
	--
	-- Generic definitions
	--
	--   C_NUM_HWTS - number of hardware threads
	--
	--   C_OSIF_DATA_WIDTH - width of the memif
	--
	generic (
		C_NUM_HWTS : integer := 1;

		C_OSIF_DATA_WIDTH : integer := 32
	);

	--
	-- Port defintions
	--
	--   OSIF_Hw2Ic_<<Id>>_/OSIF_Ic2Hw_<<Id>>_ - fifo signal inputs
	--
	--   OSIF_Sw2Ic_/OSIF_Ic2Sw_ - fifo signal inputs for software
	--
	--   OSIF_Res2Ic_<<Id>>_/OSIF_Ic2Res_<<Id>>_ - fifo signal inputs for resources
	--   
	--   SYS_Clk - system clock
	--   SYS_Rst - system reset
	--
	port (
		<<generate for SLOTS>>
		OSIF_Hw2Ic_<<Id>>_Data  : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Hw2Ic_<<Id>>_Full  : out std_logic;
		OSIF_Hw2Ic_<<Id>>_WE    : in  std_logic;
		<<end generate>>

		<<generate for SLOTS>>
		OSIF_Ic2Hw_<<Id>>_Data  : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Ic2Hw_<<Id>>_Empty : out std_logic;
		OSIF_Ic2Hw_<<Id>>_RE    : in std_logic;
		<<end generate>>

		<<generate for RESOURCES(Mode == "hw")>>
		OSIF_Res2Ic_<<Id>>_Data  : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Res2Ic_<<Id>>_Full  : out std_logic;
		OSIF_Res2Ic_<<Id>>_WE    : in  std_logic;
		<<end generate>>

		<<generate for RESOURCES(Mode == "hw")>>
		OSIF_Ic2Res_<<Id>>_Data  : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Ic2Res_<<Id>>_Empty : out std_logic;
		OSIF_Ic2Res_<<Id>>_RE    : in  std_logic;
		<<end generate>>

		OSIF_Sw2Ic_Data     : in  std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Sw2Ic_Full     : out std_logic;
		OSIF_Sw2Ic_WE       : in  std_logic;

		OSIF_Ic2Sw_Data     : out std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
		OSIF_Ic2Sw_Empty    : out std_logic;
		OSIF_Ic2Sw_RE       : in  std_logic;
		OSIF_Ic2Sw_Has_Data : out std_logic;

		SYS_Clk : in std_logic;
		SYS_Rst : in std_logic
	);
end entity reconos_osif_interconnect;


architecture imp of reconos_osif_interconnect is
	--
	-- Internal interconnect signals
	--
	--   hw<<SlotId>>2res<<ResId>>_/res<<ResId>>2hw<<SlotId>>_ - interconnect from slots to resources
	--   sw2hw<<SlotId>>_/hw<<SlotId>>2sw_                     - interconnect from slots to software
	--   sw2res<<ResId>>_/res<<ResId>>2sw_                     - interconnect for software to resources
	--
	--   dt2sw_out_empty - internal signal to support has data for software
	--
	<<generate for SLOTS>>
	<<=generate for IcRes=>>
	signal hw<<SlotId>>2res<<ResId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal hw<<SlotId>>2res<<ResId>>_empty : std_logic;
	signal hw<<SlotId>>2res<<ResId>>_re    : std_logic;

	signal res<<ResId>>2hw<<SlotId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal res<<ResId>>2hw<<SlotId>>_empty : std_logic;
	signal res<<ResId>>2hw<<SlotId>>_re    : std_logic;
	<<=end generate=>>

	signal sw2hw<<SlotId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal sw2hw<<SlotId>>_empty : std_logic;
	signal sw2hw<<SlotId>>_re    : std_logic;

	signal hw<<SlotId>>2sw_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal hw<<SlotId>>2sw_empty : std_logic;
	signal hw<<SlotId>>2sw_re    : std_logic;
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	signal sw2res<<ResId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal sw2res<<ResId>>_empty : std_logic;
	signal sw2res<<ResId>>_re    : std_logic;

	signal res<<ResId>>2sw_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal res<<ResId>>2sw_empty : std_logic;
	signal res<<ResId>>2sw_re    : std_logic;
	<<end generate>>

	--
	-- Internal signals for fifos
	--
	--   fifoin_slot<<SlotId>>_ - fifo signals for slot
	--   fifoin_res<<ResId>>_   - fifo signals for resource
	--   fifoin_sw_             - fifo signals for software
	--
	<<generate for SLOTS>>
	signal fifoin_slot<<SlotId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal fifoin_slot<<SlotId>>_empty : std_logic;
	signal fifoin_slot<<SlotId>>_re    : std_logic;
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	signal fifoin_res<<ResId>>_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal fifoin_res<<ResId>>_empty : std_logic;
	signal fifoin_res<<ResId>>_re    : std_logic;
	<<end generate>>

	signal fifoin_sw_data  : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal fifoin_sw_empty : std_logic;
	signal fifoin_sw_re    : std_logic;

	--
	-- Internal signal to add registers for multiplexed outputs
	--
	--   ic2hw_  - fifo signals for slot
	--   ic2res_ - fifo signals for resources
	--   ic2sw_  - fifo signals for software
	--
	<<generate for SLOTS>>
	signal ic2hw_<<SlotId>>_data    : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal ic2hw_<<SlotId>>_empty   : std_logic;
	signal ic2hw_<<SlotId>>_empty_n : std_logic;
	signal ic2hw_<<SlotId>>_re      : std_logic;
	signal ic2hw_<<SlotId>>_re_n    : std_logic;
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	signal ic2res_<<ResId>>_data    : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal ic2res_<<ResId>>_empty   : std_logic;
	signal ic2res_<<ResId>>_empty_n : std_logic;
	signal ic2res_<<ResId>>_re      : std_logic;
	signal ic2res_<<ResId>>_re_n    : std_logic;
	<<end generate>>

	signal ic2sw_data    : std_logic_vector(C_OSIF_DATA_WIDTH - 1 downto 0);
	signal ic2sw_empty   : std_logic;
	signal ic2sw_empty_n : std_logic;
	signal ic2sw_re      : std_logic;
	signal ic2sw_re_n    : std_logic;
begin

	-- == Added registers for output signals ==============================

	<<generate for SLOTS>>
	ic2hw_<<SlotId>>_empty_n <= not ic2hw_<<SlotId>>_empty;
	ic2hw_<<SlotId>>_re      <= not ic2hw_<<SlotId>>_re_n;
	fifoout_slot<<SlotId>> : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => 1
		)
		port map (
			FIFO_S_Data  => OSIF_Ic2Hw_<<Id>>_Data,
			FIFO_S_Empty => OSIF_Ic2Hw_<<Id>>_Empty,
			FIFO_S_RE    => OSIF_Ic2Hw_<<Id>>_RE,

			FIFO_M_Data => ic2hw_<<SlotId>>_data,
			FIFO_M_Full => ic2hw_<<SlotId>>_re_n,
			FIFO_M_WE   => ic2hw_<<SlotId>>_empty_n,

			FIFO_Clk => SYS_Clk,
			FIFO_Rst => SYS_Rst
		);
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	ic2res_<<ResId>>_empty_n <= not ic2res_<<ResId>>_empty;
	ic2res_<<ResId>>_re      <= not ic2res_<<ResId>>_re_n;
	fifoout_res<<ResId>> : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => 1
		)
		port map (
			FIFO_S_Data  => OSIF_Ic2Res_<<Id>>_Data,
			FIFO_S_Empty => OSIF_Ic2Res_<<Id>>_Empty,
			FIFO_S_RE    => OSIF_Ic2Res_<<Id>>_RE,

			FIFO_M_Data => ic2res_<<ResId>>_data,
			FIFO_M_Full => ic2res_<<ResId>>_re_n,
			FIFO_M_WE   => ic2res_<<ResId>>_empty_n,

			FIFO_Clk => SYS_Clk,
			FIFO_Rst => SYS_Rst
		);
	<<end generate>>

	ic2sw_empty_n <= not ic2sw_empty;
	ic2sw_re      <= not ic2sw_re_n;
	fifoout_sw : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => 1
		)
		port map (
			FIFO_S_Data     => OSIF_Ic2Sw_Data,
			FIFO_S_Empty    => OSIF_Ic2Sw_Empty,
			FIFO_S_RE       => OSIF_Ic2Sw_RE,

			FIFO_M_Data => ic2sw_data,
			FIFO_M_Full => ic2sw_re_n,
			FIFO_M_WE   => ic2sw_empty_n,

			FIFO_Clk      => SYS_Clk,
			FIFO_Rst      => SYS_Rst,
			FIFO_Has_Data => OSIF_Ic2Sw_Has_Data
		);


	-- == Fifo instantiation ==============================================

	<<generate for SLOTS>>
	fifoin_slot<<SlotId>> : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => C_OSIF_ADDR_WIDTH
		)
		port map (
			FIFO_S_Data  => fifoin_slot<<SlotId>>_data,
			FIFO_S_Empty => fifoin_slot<<SlotId>>_empty,
			FIFO_S_RE    => fifoin_slot<<SlotId>>_re,

			FIFO_M_Data => OSIF_Hw2Ic_<<SlotId>>_Data,
			FIFO_M_Full => OSIF_Hw2Ic_<<SlotId>>_Full,
			FIFO_M_WE   => OSIF_Hw2Ic_<<SlotId>>_WE,

			FIFO_Clk => SYS_Clk,
			FIFO_Rst => SYS_Rst
		);
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	fifoin_res<<ResId>> : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => C_OSIF_ADDR_WIDTH
		)
		port map (
			FIFO_S_Data  => fifoin_res<<ResId>>_data,
			FIFO_S_Empty => fifoin_res<<ResId>>_empty,
			FIFO_S_RE    => fifoin_res<<ResId>>_re,

			FIFO_M_Data => OSIF_Res2Ic_<<ResId>>_Data,
			FIFO_M_Full => OSIF_Res2Ic_<<ResId>>_Full,
			FIFO_M_WE   => OSIF_Res2Ic_<<ResId>>_WE,

			FIFO_Clk => SYS_Clk,
			FIFO_Rst => SYS_Rst
		);
	<<end generate>>

	fifoin_sw : entity reconos_fifo_sync_v1_00_a.reconos_fifo_sync
		generic map (
			C_FIFO_DATA_WIDTH => C_OSIF_DATA_WIDTH,
			C_FIFO_ADDR_WIDTH => C_OSIF_ADDR_WIDTH
		)
		port map (
			FIFO_S_Data  => fifoin_sw_data,
			FIFO_S_Empty => fifoin_sw_empty,
			FIFO_S_RE    => fifoin_sw_re,

			FIFO_M_Data => OSIF_Sw2Ic_Data,
			FIFO_M_Full => OSIF_Sw2Ic_Full,
			FIFO_M_WE   => OSIF_Sw2Ic_WE,

			FIFO_Clk => SYS_Clk,
			FIFO_Rst => SYS_Rst
		);


	-- == Interconnect instantiation ======================================

	icin_sw : entity reconos_osif_interconnect_v1_00_a.router_icin_sw
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_In_Data  => fifoin_sw_data,
			FIFO_In_Empty => fifoin_sw_empty,
			FIFO_In_RE    => fifoin_sw_re,

			<<generate for RESOURCES(Mode == "hw")>>
			FIFO_Res<<ResId>>_Data  => sw2res<<ResId>>_data,
			FIFO_Res<<ResId>>_Empty => sw2res<<ResId>>_empty,
			FIFO_Res<<ResId>>_RE    => sw2res<<ResId>>_re,
			<<end generate>>
			<<generate for SLOTS>>
			FIFO_Hw<<SlotId>>_Data  => sw2hw<<SlotId>>_data,
			FIFO_Hw<<SlotId>>_Empty => sw2hw<<SlotId>>_empty,
			FIFO_Hw<<SlotId>>_RE    => sw2hw<<SlotId>>_re,
			<<end generate>>

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);

	icout_sw : entity reconos_osif_interconnect_v1_00_a.arbiter_icout_sw
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_Out_Data     => ic2sw_data,
			FIFO_Out_Empty    => ic2sw_empty,
			FIFO_Out_RE       => ic2sw_re,

			<<generate for RESOURCES(Mode == "hw")>>
			FIFO_Res<<ResId>>_Data  => res<<ResId>>2sw_data,
			FIFO_Res<<ResId>>_Empty => res<<ResId>>2sw_empty,
			FIFO_Res<<ResId>>_RE    => res<<ResId>>2sw_re,
			<<end generate>>
			<<generate for SLOTS>>
			FIFO_Hw<<SlotId>>_Data  => hw<<SlotId>>2sw_data,
			FIFO_Hw<<SlotId>>_Empty => hw<<SlotId>>2sw_empty,
			FIFO_Hw<<SlotId>>_RE    => hw<<SlotId>>2sw_re,
			<<end generate>>

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);

	<<generate for SLOTS>>
	icin_slot<<Id>> : entity reconos_osif_interconnect_v1_00_a.router_icin_slot<<Id>>
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_In_Data  => fifoin_slot<<SlotId>>_data,
			FIFO_In_Empty => fifoin_slot<<SlotId>>_empty,
			FIFO_In_RE    => fifoin_slot<<SlotId>>_re,

			<<=generate for IcRes=>>
			FIFO_Res<<ResId>>_Data  => hw<<SlotId>>2res<<ResId>>_data,
			FIFO_Res<<ResId>>_Empty => hw<<SlotId>>2res<<ResId>>_empty,
			FIFO_Res<<ResId>>_RE    => hw<<SlotId>>2res<<ResId>>_re,
			<<=end generate=>>
			FIFO_Sw_Data            => hw<<SlotId>>2sw_data,
			FIFO_Sw_Empty           => hw<<SlotId>>2sw_empty,
			FIFO_Sw_RE              => hw<<SlotId>>2sw_re,

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);

	icout_slot<<Id>> : entity reconos_osif_interconnect_v1_00_a.arbiter_icout_slot<<Id>>
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_Out_Data  => ic2hw_<<SlotId>>_data,
			FIFO_Out_Empty => ic2hw_<<SlotId>>_empty,
			FIFO_Out_RE    => ic2hw_<<SlotId>>_re,

			<<=generate for IcRes=>>
			FIFO_Res<<ResId>>_Data  => res<<ResId>>2hw<<SlotId>>_data,
			FIFO_Res<<ResId>>_Empty => res<<ResId>>2hw<<SlotId>>_empty,
			FIFO_Res<<ResId>>_RE    => res<<ResId>>2hw<<SlotId>>_re,
			<<=end generate=>>
			FIFO_Sw_Data            => sw2hw<<SlotId>>_data,
			FIFO_Sw_Empty           => sw2hw<<SlotId>>_empty,
			FIFO_Sw_RE              => sw2hw<<SlotId>>_re,

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);
	<<end generate>>

	<<generate for RESOURCES(Mode == "hw")>>
	icin_res<<Id>> : entity reconos_osif_interconnect_v1_00_a.router_icin_res<<Id>>
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_In_Data  => fifoin_res<<ResId>>_data,
			FIFO_In_Empty => fifoin_res<<ResId>>_empty,
			FIFO_In_RE    => fifoin_res<<ResId>>_re,

			<<=generate for IcSlots=>>
			FIFO_Hw<<SlotId>>_Data  => res<<ResId>>2hw<<SlotId>>_data,
			FIFO_Hw<<SlotId>>_Empty => res<<ResId>>2hw<<SlotId>>_empty,
			FIFO_Hw<<SlotId>>_RE    => res<<ResId>>2hw<<SlotId>>_re,
			<<=end generate=>>
			FIFO_Sw_Data            => res<<ResId>>2sw_data,
			FIFO_Sw_Empty           => res<<ResId>>2sw_empty,
			FIFO_Sw_RE              => res<<ResId>>2sw_re,

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);

	icout_res<<Id>> : entity reconos_osif_interconnect_v1_00_a.arbiter_icout_res<<Id>>
		generic map (
			C_OSIF_DATA_WIDTH => C_OSIF_DATA_WIDTH
		)
		port map (
			FIFO_Out_Data  => ic2res_<<ResId>>_data,
			FIFO_Out_Empty => ic2res_<<ResId>>_empty,
			FIFO_Out_RE    => ic2res_<<ResId>>_re,

			<<=generate for IcSlots=>>
			FIFO_Hw<<SlotId>>_Data  => hw<<SlotId>>2res<<ResId>>_data,
			FIFO_Hw<<SlotId>>_Empty => hw<<SlotId>>2res<<ResId>>_empty,
			FIFO_Hw<<SlotId>>_RE    => hw<<SlotId>>2res<<ResId>>_re,
			<<=end generate=>>
			FIFO_Sw_Data            => sw2res<<ResId>>_data,
			FIFO_Sw_Empty           => sw2res<<ResId>>_empty,
			FIFO_Sw_RE              => sw2res<<ResId>>_re,

			SYS_Clk => SYS_Clk,
			SYS_Rst => SYS_Rst
		);
	<<end generate>>

end architecture imp;
