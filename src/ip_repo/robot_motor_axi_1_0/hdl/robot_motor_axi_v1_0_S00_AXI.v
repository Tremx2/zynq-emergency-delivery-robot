
`timescale 1 ns / 1 ps

	module robot_motor_axi_v1_0_S00_AXI #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line

		// Width of S_AXI data bus
		parameter integer C_S_AXI_DATA_WIDTH	= 32,
		// Width of S_AXI address bus
		parameter integer C_S_AXI_ADDR_WIDTH	= 7
	)
	(
		// Users to add ports here
        input  wire [3:0] enc_a,
        input  wire [3:0] enc_b,

        output wire [3:0] rpwm,
        output wire [3:0] lpwm,
        output wire tof_xshut,
		// User ports ends
		// Do not modify the ports beyond this line

		// Global Clock Signal
		input wire  S_AXI_ACLK,
		// Global Reset Signal. This Signal is Active LOW
		input wire  S_AXI_ARESETN,
		// Write address (issued by master, acceped by Slave)
		input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
		// Write channel Protection type. This signal indicates the
    		// privilege and security level of the transaction, and whether
    		// the transaction is a data access or an instruction access.
		input wire [2 : 0] S_AXI_AWPROT,
		// Write address valid. This signal indicates that the master signaling
    		// valid write address and control information.
		input wire  S_AXI_AWVALID,
		// Write address ready. This signal indicates that the slave is ready
    		// to accept an address and associated control signals.
		output wire  S_AXI_AWREADY,
		// Write data (issued by master, acceped by Slave) 
		input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
		// Write strobes. This signal indicates which byte lanes hold
    		// valid data. There is one write strobe bit for each eight
    		// bits of the write data bus.    
		input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
		// Write valid. This signal indicates that valid write
    		// data and strobes are available.
		input wire  S_AXI_WVALID,
		// Write ready. This signal indicates that the slave
    		// can accept the write data.
		output wire  S_AXI_WREADY,
		// Write response. This signal indicates the status
    		// of the write transaction.
		output wire [1 : 0] S_AXI_BRESP,
		// Write response valid. This signal indicates that the channel
    		// is signaling a valid write response.
		output wire  S_AXI_BVALID,
		// Response ready. This signal indicates that the master
    		// can accept a write response.
		input wire  S_AXI_BREADY,
		// Read address (issued by master, acceped by Slave)
		input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
		// Protection type. This signal indicates the privilege
    		// and security level of the transaction, and whether the
    		// transaction is a data access or an instruction access.
		input wire [2 : 0] S_AXI_ARPROT,
		// Read address valid. This signal indicates that the channel
    		// is signaling valid read address and control information.
		input wire  S_AXI_ARVALID,
		// Read address ready. This signal indicates that the slave is
    		// ready to accept an address and associated control signals.
		output wire  S_AXI_ARREADY,
		// Read data (issued by slave)
		output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
		// Read response. This signal indicates the status of the
    		// read transfer.
		output wire [1 : 0] S_AXI_RRESP,
		// Read valid. This signal indicates that the channel is
    		// signaling the required read data.
		output wire  S_AXI_RVALID,
		// Read ready. This signal indicates that the master can
    		// accept the read data and response information.
		input wire  S_AXI_RREADY
	);

	// AXI4LITE signals
	reg [C_S_AXI_ADDR_WIDTH-1 : 0] 	axi_awaddr;
	reg  	axi_awready;
	reg  	axi_wready;
	reg [1 : 0] 	axi_bresp;
	reg  	axi_bvalid;
	reg [C_S_AXI_ADDR_WIDTH-1 : 0] 	axi_araddr;
	reg  	axi_arready;
	reg [C_S_AXI_DATA_WIDTH-1 : 0] 	axi_rdata;
	reg [1 : 0] 	axi_rresp;
	reg  	axi_rvalid;

	// Example-specific design signals
	// local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
	// ADDR_LSB is used for addressing 32/64 bit registers/memories
	// ADDR_LSB = 2 for 32 bits (n downto 2)
	// ADDR_LSB = 3 for 64 bits (n downto 3)
	localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
	localparam integer OPT_MEM_ADDR_BITS = 4;
	//----------------------------------------------
	//-- Signals for user logic register space example
	//------------------------------------------------
	//-- Number of Slave Registers 16
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg0;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg1;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg2;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg3;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg4;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg5;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg6;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg7;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg8;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg9;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg10;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg11;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg12;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg13;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg14;
	reg [C_S_AXI_DATA_WIDTH-1:0]	slv_reg15;
	wire	 slv_reg_rden;
	wire	 slv_reg_wren;
	reg [C_S_AXI_DATA_WIDTH-1:0]	 reg_data_out;
	integer	 byte_index;
	reg	 aw_en;

	// I/O Connections assignments

	assign S_AXI_AWREADY	= axi_awready;
	assign S_AXI_WREADY	= axi_wready;
	assign S_AXI_BRESP	= axi_bresp;
	assign S_AXI_BVALID	= axi_bvalid;
	assign S_AXI_ARREADY	= axi_arready;
	assign S_AXI_RDATA	= axi_rdata;
	assign S_AXI_RRESP	= axi_rresp;
	assign S_AXI_RVALID	= axi_rvalid;
	// Implement axi_awready generation
	// axi_awready is asserted for one S_AXI_ACLK clock cycle when both
	// S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
	// de-asserted when reset is low.

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_awready <= 1'b0;
	      aw_en <= 1'b1;
	    end 
	  else
	    begin    
	      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
	        begin
	          // slave is ready to accept write address when 
	          // there is a valid write address and write data
	          // on the write address and data bus. This design 
	          // expects no outstanding transactions. 
	          axi_awready <= 1'b1;
	          aw_en <= 1'b0;
	        end
	        else if (S_AXI_BREADY && axi_bvalid)
	            begin
	              aw_en <= 1'b1;
	              axi_awready <= 1'b0;
	            end
	      else           
	        begin
	          axi_awready <= 1'b0;
	        end
	    end 
	end       

	// Implement axi_awaddr latching
	// This process is used to latch the address when both 
	// S_AXI_AWVALID and S_AXI_WVALID are valid. 

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_awaddr <= 0;
	    end 
	  else
	    begin    
	      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
	        begin
	          // Write Address latching 
	          axi_awaddr <= S_AXI_AWADDR;
	        end
	    end 
	end       

	// Implement axi_wready generation
	// axi_wready is asserted for one S_AXI_ACLK clock cycle when both
	// S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is 
	// de-asserted when reset is low. 

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_wready <= 1'b0;
	    end 
	  else
	    begin    
	      if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en )
	        begin
	          // slave is ready to accept write data when 
	          // there is a valid write address and write data
	          // on the write address and data bus. This design 
	          // expects no outstanding transactions. 
	          axi_wready <= 1'b1;
	        end
	      else
	        begin
	          axi_wready <= 1'b0;
	        end
	    end 
	end       

	// Implement memory mapped register select and write logic generation
	// The write data is accepted and written to memory mapped registers when
	// axi_awready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted. Write strobes are used to
	// select byte enables of slave registers while writing.
	// These registers are cleared when reset (active low) is applied.
	// Slave register write enable is asserted when valid address and data are available
	// and the slave is ready to accept the write address and write data.
	assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      slv_reg0 <= 0;
	      slv_reg1 <= 0;
	      slv_reg2 <= 0;
	      slv_reg3 <= 0;
	      slv_reg4 <= 0;
	      slv_reg5 <= 0;
	      slv_reg6 <= 0;
	      slv_reg7 <= 0;
	      slv_reg8 <= 0;
	      slv_reg9 <= 0;
	      slv_reg10 <= 0;
	      slv_reg11 <= 0;
	      slv_reg12 <= 0;
	      slv_reg13 <= 0;
	      slv_reg14 <= 0;
	      slv_reg15 <= 0;
	    end 
	  else begin
	    if (slv_reg_wren)
	      begin
	        case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])

    5'h00: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg0[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    5'h01: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg1[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    5'h02: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg2[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    5'h03: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg3[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    5'h04: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg4[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    5'h14: begin
        for (byte_index = 0;
             byte_index <= (C_S_AXI_DATA_WIDTH/8)-1;
             byte_index = byte_index+1)
            if (S_AXI_WSTRB[byte_index] == 1'b1)
                slv_reg5[(byte_index*8) +: 8]
                    <= S_AXI_WDATA[(byte_index*8) +: 8];
    end

    default: begin
        // 其他地址不保存写入数据
    end

endcase
	      end
	  end
	end    

    // ================================================================
    // Independent FPGA motor watchdog (125 MHz AXI clock, 500 ms)
    // 0x44 write 0xC0DEC0DE: heartbeat (only when armed)
    // 0x48 read: bit0=armed, bit1=timeout fault latched
    // 0x4C write 0xA5C35A3C: arm / clear fault ONLY while
    //       enable and all four target-speed registers are zero.
    // Safety default after reset: disarmed, all PWM outputs LOW.
    // ================================================================
    localparam [25:0] WD_LAST_TICK = 26'd62_499_999;
    reg [25:0] wd_counter;
    reg wd_armed;
    reg wd_fault;

    wire wd_heartbeat_write = slv_reg_wren &&
        (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 5'h11) &&
        (&S_AXI_WSTRB) && (S_AXI_WDATA == 32'hC0DE_C0DE);

    wire wd_arm_write = slv_reg_wren &&
        (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 5'h13) &&
        (&S_AXI_WSTRB) && (S_AXI_WDATA == 32'hA5C3_5A3C);

    wire wd_can_arm = (slv_reg0[3:0] == 4'b0000) &&
        (slv_reg1 == 32'd0) && (slv_reg2 == 32'd0) &&
        (slv_reg3 == 32'd0) && (slv_reg4 == 32'd0);

    wire wd_motor_ok = S_AXI_ARESETN && wd_armed && !wd_fault;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            wd_counter <= 26'd0;
            wd_armed   <= 1'b0;
            wd_fault   <= 1'b0;
        end else if (wd_armed) begin
            // Timeout has priority even if a heartbeat arrives on this edge.
            if (wd_counter >= WD_LAST_TICK) begin
                wd_counter <= 26'd0;
                wd_armed   <= 1'b0;
                wd_fault   <= 1'b1;
            end else if (wd_heartbeat_write) begin
                wd_counter <= 26'd0;
            end else begin
                wd_counter <= wd_counter + 26'd1;
            end
        end else if (wd_arm_write && wd_can_arm) begin
            wd_counter <= 26'd0;
            wd_armed   <= 1'b1;
            wd_fault   <= 1'b0;
        end
    end

		// Implement write response logic generation
	// The write response and response valid signals are asserted by the slave 
	// when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.  
	// This marks the acceptance of address and indicates the status of 
	// write transaction.

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_bvalid  <= 0;
	      axi_bresp   <= 2'b0;
	    end 
	  else
	    begin    
	      if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID)
	        begin
	          // indicates a valid write response is available
	          axi_bvalid <= 1'b1;
	          axi_bresp  <= 2'b0; // 'OKAY' response 
	        end                   // work error responses in future
	      else
	        begin
	          if (S_AXI_BREADY && axi_bvalid) 
	            //check if bready is asserted while bvalid is high) 
	            //(there is a possibility that bready is always asserted high)   
	            begin
	              axi_bvalid <= 1'b0; 
	            end  
	        end
	    end
	end   

	// Implement axi_arready generation
	// axi_arready is asserted for one S_AXI_ACLK clock cycle when
	// S_AXI_ARVALID is asserted. axi_awready is 
	// de-asserted when reset (active low) is asserted. 
	// The read address is also latched when S_AXI_ARVALID is 
	// asserted. axi_araddr is reset to zero on reset assertion.

	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_arready <= 1'b0;
	      axi_araddr  <= 32'b0;
	    end 
	  else
	    begin    
	      if (~axi_arready && S_AXI_ARVALID)
	        begin
	          // indicates that the slave has acceped the valid read address
	          axi_arready <= 1'b1;
	          // Read address latching
	          axi_araddr  <= S_AXI_ARADDR;
	        end
	      else
	        begin
	          axi_arready <= 1'b0;
	        end
	    end 
	end       

	// Implement axi_arvalid generation
	// axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both 
	// S_AXI_ARVALID and axi_arready are asserted. The slave registers 
	// data are available on the axi_rdata bus at this instance. The 
	// assertion of axi_rvalid marks the validity of read data on the 
	// bus and axi_rresp indicates the status of read transaction.axi_rvalid 
	// is deasserted on reset (active low). axi_rresp and axi_rdata are 
	// cleared to zero on reset (active low).  
	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_rvalid <= 0;
	      axi_rresp  <= 0;
	    end 
	  else
	    begin    
	      if (axi_arready && S_AXI_ARVALID && ~axi_rvalid)
	        begin
	          // Valid read data is available at the read data bus
	          axi_rvalid <= 1'b1;
	          axi_rresp  <= 2'b0; // 'OKAY' response
	        end   
	      else if (axi_rvalid && S_AXI_RREADY)
	        begin
	          // Read data is accepted by the master
	          axi_rvalid <= 1'b0;
	        end                
	    end
	end    

	// Implement memory mapped register select and read logic generation
	// Slave register read enable is asserted when valid address is available
	// and the slave is ready to accept the read address.
	assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;
	always @(*)
	begin
	      // Address decoding for reading registers
	      case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])

    // 控制寄存器：Python 可读回自己写入的值
    5'h00: reg_data_out <= slv_reg0;
    5'h01: reg_data_out <= slv_reg1;
    5'h02: reg_data_out <= slv_reg2;
    5'h03: reg_data_out <= slv_reg3;
    5'h04: reg_data_out <= slv_reg4;

    // 四台电机实际速度
    5'h05: reg_data_out <= actual_speed_0;
    5'h06: reg_data_out <= actual_speed_1;
    5'h07: reg_data_out <= actual_speed_2;
    5'h08: reg_data_out <= actual_speed_3;

    // 四台电机编码器累计位置
    5'h09: reg_data_out <= position_0;
    5'h0A: reg_data_out <= position_1;
    5'h0B: reg_data_out <= position_2;
    5'h0C: reg_data_out <= position_3;

    // 四台电机当前 PID / PWM 命令
    5'h0D: reg_data_out <= pwm_command_0;
    5'h0E: reg_data_out <= pwm_command_1;
    5'h0F: reg_data_out <= pwm_command_2;
    5'h10: reg_data_out <= pwm_command_3;

    // Watchdog state: 0x48, bit0=armed, bit1=latched timeout
    5'h12: reg_data_out <= {30'd0, wd_fault, wd_armed};

    // Sensor control: 0x50, bit0 = TOF XSHUT
    5'h14: reg_data_out <= slv_reg5;

    default: reg_data_out <= 0;

    endcase
	end

	// Output register or memory read data
	always @( posedge S_AXI_ACLK )
	begin
	  if ( S_AXI_ARESETN == 1'b0 )
	    begin
	      axi_rdata  <= 0;
	    end 
	  else
	    begin    
	      // When there is a valid read address (S_AXI_ARVALID) with 
	      // acceptance of read address by the slave (axi_arready), 
	      // output the read dada 
	      if (slv_reg_rden)
	        begin
	          axi_rdata <= reg_data_out;     // register read data
	        end   
	    end
	end    
// =====================================================
// Four-motor controller
// AXI register 0: enable[3:0]
// AXI registers 1~4: target speeds
// =====================================================

// Motor 0 status
wire signed [31:0] position_0;
wire signed [31:0] actual_speed_0;
wire signed [31:0] pwm_command_0;

// Motor 1 status
wire signed [31:0] position_1;
wire signed [31:0] actual_speed_1;
wire signed [31:0] pwm_command_1;

// Motor 2 status
wire signed [31:0] position_2;
wire signed [31:0] actual_speed_2;
wire signed [31:0] pwm_command_2;

// Motor 3 status
wire signed [31:0] position_3;
wire signed [31:0] actual_speed_3;
wire signed [31:0] pwm_command_3;

// Also gate the physical PWM pins directly, independent of PID latency.
wire [3:0] rpwm_raw;
wire [3:0] lpwm_raw;
assign rpwm = rpwm_raw & {4{wd_motor_ok}};
assign lpwm = lpwm_raw & {4{wd_motor_ok}};

// TOF XSHUT enable, bit0 of sensor-control register @0x50; defaults LOW (shutdown)
assign tof_xshut = slv_reg5[0];

robot_motor_ctrl #(
    .CLK_FREQ_HZ (125_000_000),
    .SAMPLE_MS   (10),
    .PWM_PERIOD  (6250)
) u_robot_motor_ctrl (

    .clk   (S_AXI_ACLK),
    .rst_n (S_AXI_ARESETN),

    // Python writes these through AXI
    .enable       (slv_reg0[3:0] & {4{wd_motor_ok}}),
    .target_speed_0 (slv_reg1),
    .target_speed_1 (slv_reg2),
    .target_speed_2 (slv_reg3),
    .target_speed_3 (slv_reg4),

    // Encoder inputs
    .enc_a (enc_a),
    .enc_b (enc_b),

    // BTS7960 outputs
    .rpwm (rpwm_raw),
    .lpwm (lpwm_raw),

    // Motor 0 feedback
    .position_0    (position_0),
    .actual_speed_0(actual_speed_0),
    .pwm_command_0 (pwm_command_0),

    // Motor 1 feedback
    .position_1    (position_1),
    .actual_speed_1(actual_speed_1),
    .pwm_command_1 (pwm_command_1),

    // Motor 2 feedback
    .position_2    (position_2),
    .actual_speed_2(actual_speed_2),
    .pwm_command_2 (pwm_command_2),

    // Motor 3 feedback
    .position_3    (position_3),
    .actual_speed_3(actual_speed_3),
    .pwm_command_3 (pwm_command_3)

);
	// Add user logic here

	// User logic ends

	endmodule
