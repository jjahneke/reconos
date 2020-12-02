library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library reconos_v3_01_a;
use reconos_v3_01_a.reconos_pkg.all;

use work.reconos_thread_pkg.all;

entity rt_bram_test is
	port (
		-- OSIF FIFO ports
		OSIF_Sw2Hw_Data    : in  std_logic_vector(63 downto 0);
		OSIF_Sw2Hw_Empty   : in  std_logic;
		OSIF_Sw2Hw_RE      : out std_logic;

		OSIF_Hw2Sw_Data    : out std_logic_vector(63 downto 0);
		OSIF_Hw2Sw_Full    : in  std_logic;
		OSIF_Hw2Sw_WE      : out std_logic;

		-- MEMIF FIFO ports
		MEMIF64_Hwt2Mem_Data    : out std_logic_vector(63 downto 0);
		MEMIF64_Hwt2Mem_Full    : in  std_logic;
		MEMIF64_Hwt2Mem_WE      : out std_logic;

		MEMIF64_Mem2Hwt_Data    : in  std_logic_vector(63 downto 0);
		MEMIF64_Mem2Hwt_Empty   : in  std_logic;
		MEMIF64_Mem2Hwt_RE      : out std_logic;

		HWT_Clk    : in  std_logic;
		HWT_Rst    : in  std_logic;
		HWT_Signal : in  std_logic;
		
		DEBUG : out std_logic_vector(70 downto 0)
	);
end entity rt_bram_test;

architecture implementation of rt_bram_test is

	-- Declare port attributes for the Vivado IP Packager
	ATTRIBUTE X_INTERFACE_INFO : STRING;
	ATTRIBUTE X_INTERFACE_PARAMETER : STRING;

	ATTRIBUTE X_INTERFACE_INFO of HWT_Clk: SIGNAL is "xilinx.com:signal:clock:1.0 HWT_Clk CLK";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Clk: SIGNAL is "ASSOCIATED_RESET HWT_Rst, ASSOCIATED_BUSIF OSIF_Sw2Hw:OSIF_Hw2Sw:MEMIF64_Hwt2Mem:MEMIF64_Mem2Hwt";

	ATTRIBUTE X_INTERFACE_INFO of HWT_Rst: SIGNAL is "xilinx.com:signal:reset:1.0 HWT_Rst RST";
	ATTRIBUTE X_INTERFACE_PARAMETER of HWT_Rst: SIGNAL is "POLARITY ACTIVE_HIGH";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_Empty:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Sw2Hw_RE:       SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 OSIF_Sw2Hw FIFO64_S_RE";

	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Data:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_Full:     SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of OSIF_Hw2Sw_WE:       SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 OSIF_Hw2Sw FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_Full:  SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_Full";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Hwt2Mem_WE:    SIGNAL is "cs.upb.de:reconos:FIFO64_M:1.0 MEMIF64_Hwt2Mem FIFO64_M_WE";

	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Data:  SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_Data";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_Empty: SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_Empty";
	ATTRIBUTE X_INTERFACE_INFO of MEMIF64_Mem2Hwt_RE:    SIGNAL is "cs.upb.de:reconos:FIFO64_S:1.0 MEMIF64_Mem2Hwt FIFO64_S_RE";

	type STATE_TYPE is (
					STATE_INIT,
					STATE_GET_ADDR,STATE_GET_LEN,STATE_GET_ITERATIONS,
					STATE_READ,STATE_WRITE,
					STATE_READ_DONE,STATE_WRITE_DONE,
					STATE_ACK_READ,STATE_ACK_WRITE,
					STATE_THREAD_EXIT);
	
	-- The sorting application reads 'C_LOCAL_RAM_SIZE' 64-bit words into the local RAM,
	-- from a given address (send in a message box), sorts them and writes them back into main memory.

	-- IMPORTANT: define size of local RAM here!!!! 
	constant C_LOCAL_RAM_SIZE          : integer := 49152;
	constant C_LOCAL_RAM_ADDRESS_WIDTH : integer := integer(ceil(log2(real(C_LOCAL_RAM_SIZE))));
	constant C_LOCAL_RAM_SIZE_IN_BYTES : integer := 8*C_LOCAL_RAM_SIZE;

	type LOCAL_MEMORY_T is array (0 to C_LOCAL_RAM_SIZE-1) of std_logic_vector(63 downto 0);

	signal addr     : std_logic_vector(63 downto 0);
	signal len      : std_logic_vector(63 downto 0);

	signal state    : STATE_TYPE;
	signal i_osif   : i_osif_t;
	signal o_osif   : o_osif_t;
	signal i_memif  : i_memif_t;
	signal o_memif  : o_memif_t;
	signal i_ram    : i_ram_t;
	signal o_ram    : o_ram_t;

	signal o_RAMAddr_sorter : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMData_sorter : std_logic_vector(0 to 63);
	signal o_RAMWE_sorter   : std_logic;
	signal i_RAMData_sorter : std_logic_vector(0 to 63);

	signal o_RAMAddr_reconos   : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1);
	signal o_RAMAddr_reconos_2 : std_logic_vector(0 to 63);
	signal o_RAMData_reconos   : std_logic_vector(0 to 63);
	signal o_RAMWE_reconos     : std_logic;
	signal i_RAMData_reconos   : std_logic_vector(0 to 63);
	
	constant o_RAMAddr_max : std_logic_vector(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) := (others=>'1');

	shared variable local_ram : LOCAL_MEMORY_T;

	signal ignore   : std_logic_vector(63 downto 0);

	signal counter_read : std_logic_vector(63 downto 0);
	signal counter_write : std_logic_vector(63 downto 0);

	signal iterations      : std_logic_vector(63 downto 0);
	signal iterations_log2 : std_logic_vector(63 downto 0);

	signal counter_read_sum : std_logic_vector(63 downto 0);
	signal counter_write_sum : std_logic_vector(63 downto 0);

	signal counter_read_avg : std_logic_vector(63 downto 0);
	signal counter_write_avg : std_logic_vector(63 downto 0);

	signal vector_1 : std_logic_vector(63 downto 0);

begin
	DEBUG(0) <= '1' when state = STATE_INIT else '0';
	DEBUG(1) <= '1' when state = STATE_GET_ADDR else '0';
	DEBUG(2) <= '1' when state = STATE_READ else '0';
	DEBUG(4) <= '1' when state = STATE_WRITE else '0';
	DEBUG(6) <= '1' when state = STATE_THREAD_EXIT else '0';
	DEBUG(70 downto 7) <= addr;
	--DEBUG(39) <= OSIF_Sw2Hw_Empty;
	--DEBUG(71 downto 40) <= MEMIF64_Mem2Hwt_Data(31 downto 0);   --ToDo
	--DEBUG(72) <= MEMIF64_Mem2Hwt_Empty;
	--DEBUG(104 downto 73) <= o_memif.hwt2mem_data(31 downto 0);  --ToDO
	--DEBUG(105) <= o_memif.hwt2mem_we;
	--DEBUG(106) <= i_memif.hwt2mem_full;
	--DEBUG(110 downto 107) <= conv_std_logic_vector(i_memif.step, 4);

	-- local dual-port RAM
	local_ram_ctrl_1 : process (HWT_Clk) is
		begin
			if (rising_edge(HWT_Clk)) then
				if (o_RAMWE_reconos = '1') then
					local_ram(to_integer(unsigned(o_RAMAddr_reconos))) := o_RAMData_reconos;
				else
					i_RAMData_reconos <= local_ram(to_integer(unsigned(o_RAMAddr_reconos)));
				end if;
			end if;
		end process;
				
	local_ram_ctrl_2 : process (HWT_Clk) is
		begin
			if (rising_edge(HWT_Clk)) then		
				if (o_RAMWE_sorter = '1') then
					local_ram(to_integer(unsigned(o_RAMAddr_sorter))) := o_RAMData_sorter;
				else
					i_RAMData_sorter <= local_ram(to_integer(unsigned(o_RAMAddr_sorter)));
				end if;
			end if;
		end process;

	-- ReconOS initilization
	osif_setup (
		i_osif,
		o_osif,
		OSIF_Sw2Hw_Data,
		OSIF_Sw2Hw_Empty,
		OSIF_Sw2Hw_RE,
		OSIF_Hw2Sw_Data,
		OSIF_Hw2Sw_Full,
		OSIF_Hw2Sw_WE
	);

	memif_setup (
		i_memif,
		o_memif,
		MEMIF64_Mem2Hwt_Data,
		MEMIF64_Mem2Hwt_Empty,
		MEMIF64_Mem2Hwt_RE,
		MEMIF64_Hwt2Mem_Data,
		MEMIF64_Hwt2Mem_Full,
		MEMIF64_Hwt2Mem_WE
	);
	
	ram_setup (
		i_ram,
		o_ram,
		o_RAMAddr_reconos_2,
		o_RAMData_reconos,
		i_RAMData_reconos,
		o_RAMWE_reconos
	);
	
	o_RAMAddr_reconos(0 to C_LOCAL_RAM_ADDRESS_WIDTH-1) <= o_RAMAddr_reconos_2((64-C_LOCAL_RAM_ADDRESS_WIDTH) to 63);
		

    vector_1 <= x"0000000000000001";
    -- divide summed counters by number of iterations
	counter_read_avg  <= std_logic_vector(shift_right(unsigned(counter_read_sum), to_integer(unsigned(iterations_log2))));		
	counter_write_avg <= std_logic_vector(shift_right(unsigned(counter_write_sum), to_integer(unsigned(iterations_log2))));	

	-- os and memory synchronisation state machine
	reconos_fsm: process (HWT_Clk,HWT_Rst,o_osif,o_memif,o_ram) is
		variable done : boolean;
	begin
		if rising_edge(HWT_Clk) then
			if HWT_Rst = '1' then
				osif_reset(o_osif);
				memif_reset(o_memif);
				ram_reset(o_ram);
				state <= STATE_INIT;
				done := False;
				addr <= (others => '0');
				len <= (others => '0');
				iterations <= (others => '0');
				iterations_log2 <= (others => '0');
				counter_read <= (others => '0');
				counter_write <= (others => '0');
				counter_read_sum <= (others => '0');
				counter_write_sum <= (others => '0');
			else
				case state is
					when STATE_INIT =>
						osif_read(i_osif, o_osif, ignore, done);
						if done then
							state <= STATE_GET_ADDR;
						end if;

					when STATE_GET_ADDR =>
						osif_mbox_get(i_osif, o_osif, RESOURCES_ADDRESS, addr, done);
						if done then
							if (addr = X"FFFFFFFFFFFFFFFF") then
								state <= STATE_THREAD_EXIT;
							else
								state <= STATE_GET_LEN;
							end if;
						end if;
				
					when STATE_GET_LEN =>
						osif_mbox_get(i_osif, o_osif, RESOURCES_LENGTH, len, done);
						if done then
							state <= STATE_GET_ITERATIONS;
						end if;

					when STATE_GET_ITERATIONS =>
						osif_mbox_get(i_osif, o_osif, RESOURCES_ITERATIONS, iterations_log2, done);
						if done then
							iterations <= std_logic_vector(shift_left(unsigned(vector_1), to_integer(unsigned(iterations_log2))));
							counter_read  <= (others => '0');
							counter_write <= (others => '0');
							counter_read_sum  <= (others => '0');
							counter_write_sum <= (others => '0');
							state         <= STATE_READ;
						end if;
					
					when STATE_READ =>
						counter_read <= std_logic_vector(unsigned(counter_read) + 1);
						memif_read(i_ram,o_ram,i_memif,o_memif,addr,X"0000000000000000",len(31 downto 0),done);
						if done then
							state <= STATE_READ_DONE;
						end if;
						
					when STATE_READ_DONE =>
						counter_read_sum <= std_logic_vector(unsigned(counter_read_sum) + unsigned(counter_read));
						state <= STATE_WRITE;

					when STATE_WRITE =>
						counter_write <= std_logic_vector(unsigned(counter_write) + 1);
						memif_write(i_ram,o_ram,i_memif,o_memif,X"0000000000000000",addr,len(31 downto 0),done);
						if done then
							iterations <= std_logic_vector(unsigned(iterations) - 1);
							state <= STATE_WRITE_DONE;
						end if;

					when STATE_WRITE_DONE =>
						counter_write_sum <= std_logic_vector(unsigned(counter_write_sum) + unsigned(counter_write));
						if unsigned(iterations) = 0 then
							state <= STATE_ACK_READ;
						else
							counter_read  <= (others => '0');
							counter_write <= (others => '0');
							state <= STATE_READ;
						end if;
						
					
					when STATE_ACK_READ =>
						osif_mbox_put(i_osif, o_osif, RESOURCES_ACKNOWLEDGE_READ, counter_read_avg, ignore, done);
						if done then state <= STATE_ACK_WRITE; end if;

					when STATE_ACK_WRITE =>
						osif_mbox_put(i_osif, o_osif, RESOURCES_ACKNOWLEDGE_WRITE, counter_write_avg, ignore, done);
						if done then state <= STATE_GET_ADDR; end if;

					-- thread exit
					when STATE_THREAD_EXIT =>
						osif_thread_exit(i_osif,o_osif);
				
				end case;
			end if;
		end if;
	end process;
	
end architecture;
