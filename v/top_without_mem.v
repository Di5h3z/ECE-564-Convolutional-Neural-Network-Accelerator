


module MyDesign(
  input dut_run,
  output dut_busy,
  input reset_b,
  input clk,

  output [11:0]dut_sram_write_address,
  output [15:0]dut_sram_write_data,
  output dut_sram_write_enable,

  output [11:0]dut_sram_read_address,
  input [15:0]sram_dut_read_data,

  output [11:0]dut_wmem_read_address,
  input [15:0]wmem_dut_read_data);

  //dimension values and weight matrix
  wire [4:0] xdim, ydim;
  wire [15:0]weight;
  //loading dimensions from the controller
  wire [1:0]input_dim_load_ctrl;
  //addr and write enable from controller for sram interfaces
  wire [11:0]fsm_weight_addr, fsm_input_addr, fsm_output_addr;
  wire fsm_w_enable;
  //Data from interface outputs to device
  wire [15:0]weight_input_data, input_input_data;
  //Whole row output calculation to be sent to sram interface
  wire [15:0]calculation_output;


  //CONTROLLER
  //FSM controller sets addresses and load controls
  fms_controller controller(.clock(clk), .reset(reset_b), .go(dut_run),
    .ydim(ydim),
    .input_data(input_input_data),
    .busy(dut_busy),
    .input_load_ctrl(input_dim_load_ctrl),
    .input_addr(fsm_input_addr),
    .weight_addr(fsm_weight_addr),
    .output_addr(fsm_output_addr),
    .output_w_enable(fsm_w_enable));

//------------------------------------------------------------------------------
  //CONVOLUTION module
  //does the calculations from the previous 3 inputs and left padds the output
  convolution conv_calc(.clock(clk),
    .reset(reset_b),
    .xdim(xdim),
    .weight(weight),
    .input_data(input_input_data),
    .padded_out(calculation_output));

//------------------------------------------------------------------------------
  //WEIGHT AND DIM REGISTERS
  //holds the weight and dimensions after they have been loaded from SRAM
  input_weight_registers weight_reg(.clock(clk),
    .reset(reset_b),
    .input_sram_data(weight_input_data),
    .weight_out(weight));
  input_dim_registers dim_reg(.clock(clk),
    .reset(reset_b),
    .input_sram_data(input_input_data),
    .input_load_ctrl(input_dim_load_ctrl),
    .xdim_reg(xdim),
    .ydim_reg(ydim));


//------------------------------------------------------------------------------
  //READ INTERFACE
  //pipelines inputs and outputs to the SRAM
  sram_read_interface read_input(.clock(clk),
    .reset(reset_b),
    .sram_interface_data(sram_dut_read_data),
    .dev_interface_read_addr(fsm_input_addr),
    .interface_sram_read_addr(dut_sram_read_address),
    .interface_dev_data_out(input_input_data));
  sram_read_interface read_weight(.clock(clk),
    .reset(reset_b),
    .sram_interface_data(wmem_dut_read_data),
    .dev_interface_read_addr(fsm_weight_addr),
    .interface_sram_read_addr(dut_wmem_read_address),
    .interface_dev_data_out(weight_input_data));

//------------------------------------------------------------------------------
  //WRITE INTERFACE
  //Pipelines the outputs to the SRAM
  sram_write_interface write_output(.clock(clk), .reset(reset_b),
    .dev_interface_write_addr(fsm_output_addr),
    .dev_interface_write_data(calculation_output),
    .dev_interface_r_enable(fsm_w_enable),
    .interface_sram_write_addr(dut_sram_write_address),
    .interface_sram_write_data(dut_sram_write_data),
    .interface_sram_r_enable(dut_sram_write_enable)
    );

endmodule

//stores the weight
module input_weight_registers(
  input clock,
  input reset,
  input [15:0]input_sram_data,
  output [15:0]weight_out
  );

  reg [8:0]weight;
  assign weight_out = {6'b0,weight};


  always @ ( posedge clock ) begin
    if(~reset)
      weight<=0;
    else
      weight<=input_sram_data;
  end

endmodule

// This module contains the registers for x and y dim
// It handels when to update them from input_select_ctrl
// 00,10 -> dont update
// 01 -> save input to xdim_reg
// 11 -> save input to ydim_reg
module input_dim_registers(
  input clock,
  input reset,
  input [15:0]input_sram_data,
  input [1:0]input_load_ctrl,
  output reg [4:0] xdim_reg,
  output reg [4:0] ydim_reg
  );
  //next values to be loaded in
  reg [4:0] xdim_reg_next;
  reg [4:0] ydim_reg_next;


  //clear registers on sync reset
  always@(posedge clock)begin
    if(~reset)begin
      xdim_reg <= 0;
      ydim_reg <= 0;
    end
    else begin
      xdim_reg <= xdim_reg_next;
      ydim_reg <= ydim_reg_next;
    end
  end

  always@(*)begin
    //hold value by default
    xdim_reg_next = xdim_reg;
    ydim_reg_next = ydim_reg;

    //otherwise load new value
    casex(input_load_ctrl)
      2'b01:  xdim_reg_next = input_sram_data[4:0];
      2'b11:  ydim_reg_next = input_sram_data[4:0];
    endcase
  end

endmodule


//the inputs to the ram through a flip flop
module sram_read_interface(
  input clock,
  input reset,
  input [15:0]sram_interface_data,           //from mem
  input [11:0]dev_interface_read_addr,      //from ctrl
  output reg [11:0]interface_sram_read_addr, //to mem
  output reg [15:0]interface_dev_data_out  //to inputs
  );

  //inputs and ouputs pass through a flip flop
  always @ ( posedge clock ) begin
    if(~reset)begin
      interface_sram_read_addr <= 0;
      interface_dev_data_out <=0;
    end
    else begin
      interface_sram_read_addr <= dev_interface_read_addr; //data from mem into logic
      interface_dev_data_out <= sram_interface_data; //addr from ctrl into mem
    end

  end


endmodule

//the output to the ram through a flip flop
module sram_write_interface(
  input clock,
  input reset,

  input [11:0]dev_interface_write_addr,//from ctrl
  input [15:0]dev_interface_write_data,      //from ctrl
  input dev_interface_r_enable,             //from ctrl

  output reg [11:0]interface_sram_write_addr, //to mem
  output reg [15:0]interface_sram_write_data, //to mem
  output reg interface_sram_r_enable);

  //inputs and ouputs pass through a flip flop
  always @ ( posedge clock ) begin
    if(~reset)begin
      interface_sram_write_addr <= 0;
      interface_sram_write_data <=0;
      interface_sram_r_enable <=0;
    end
    else begin
      interface_sram_write_addr <= dev_interface_write_addr;
      interface_sram_write_data <=dev_interface_write_data;
      interface_sram_r_enable <=dev_interface_r_enable;
    end

  end


endmodule

//Captures the inputs and piplines them to compute the convolution with the weight
module convolution(
  input clock,
  input reset,
  input [4:0]xdim,
  input [15:0]weight,
  input [15:0]input_data,
  output [15:0]padded_out);


wire [15:0]reg_1, reg_2, reg_3;
wire [13:0]output_raw;

//padd the ouput
output_select_padding padded_output(.xnor_bitcnt_add_cmp_output(output_raw),.xdim(xdim),.padded_output(padded_out));

//clocks the data through the registers to shift them to get the proper outputs
input_registers input_storage_regs(.clock(clock),
  .reset(reset),
  .data_input(input_data),
  .reg_1(reg_1),
  .reg_2(reg_2),
  .reg_3(reg_3));

//output sign determination in parallel for each row
xnor_bitcnt_add_cmp output_bit_0(.weight_row_1(weight[2:0]), .weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[2:0]),.input_reg_2_bits(reg_2[2:0]),.input_reg_3_bits(reg_3[2:0]),.sign_output(output_raw[0]));
xnor_bitcnt_add_cmp output_bit_1(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[3:1]),.input_reg_2_bits(reg_2[3:1]),.input_reg_3_bits(reg_3[3:1]),.sign_output(output_raw[1]));
xnor_bitcnt_add_cmp output_bit_2(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[4:2]),.input_reg_2_bits(reg_2[4:2]),.input_reg_3_bits(reg_3[4:2]),.sign_output(output_raw[2]));
xnor_bitcnt_add_cmp output_bit_3(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[5:3]),.input_reg_2_bits(reg_2[5:3]),.input_reg_3_bits(reg_3[5:3]),.sign_output(output_raw[3]));
xnor_bitcnt_add_cmp output_bit_4(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[6:4]),.input_reg_2_bits(reg_2[6:4]),.input_reg_3_bits(reg_3[6:4]),.sign_output(output_raw[4]));
xnor_bitcnt_add_cmp output_bit_5(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[7:5]),.input_reg_2_bits(reg_2[7:5]),.input_reg_3_bits(reg_3[7:5]),.sign_output(output_raw[5]));
xnor_bitcnt_add_cmp output_bit_6(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[8:6]),.input_reg_2_bits(reg_2[8:6]),.input_reg_3_bits(reg_3[8:6]),.sign_output(output_raw[6]));
xnor_bitcnt_add_cmp output_bit_7(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[9:7]),.input_reg_2_bits(reg_2[9:7]),.input_reg_3_bits(reg_3[9:7]),.sign_output(output_raw[7]));
xnor_bitcnt_add_cmp output_bit_8(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[10:8]),.input_reg_2_bits(reg_2[10:8]),.input_reg_3_bits(reg_3[10:8]),.sign_output(output_raw[8]));
xnor_bitcnt_add_cmp output_bit_9(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]), .weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[11:9]),.input_reg_2_bits(reg_2[11:9]),.input_reg_3_bits(reg_3[11:9]),.sign_output(output_raw[9]));
xnor_bitcnt_add_cmp output_bit_10(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]),.weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[12:10]),.input_reg_2_bits(reg_2[12:10]),.input_reg_3_bits(reg_3[12:10]),.sign_output(output_raw[10]));
xnor_bitcnt_add_cmp output_bit_11(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]),.weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[13:11]),.input_reg_2_bits(reg_2[13:11]),.input_reg_3_bits(reg_3[13:11]),.sign_output(output_raw[11]));
xnor_bitcnt_add_cmp output_bit_12(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]),.weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[14:12]),.input_reg_2_bits(reg_2[14:12]),.input_reg_3_bits(reg_3[14:12]),.sign_output(output_raw[12]));
xnor_bitcnt_add_cmp output_bit_13(.weight_row_1(weight[2:0]),.weight_row_2(weight[5:3]),.weight_row_3(weight[8:6]),.input_reg_1_bits(reg_1[15:13]),.input_reg_2_bits(reg_2[15:13]),.input_reg_3_bits(reg_3[15:13]),.sign_output(output_raw[13]));

endmodule



//shifts the input from reg_3 -> reg_1
module input_registers(
  input clock,
  input reset,
  input [15:0]data_input,
  output reg [15:0]reg_1,
  output reg [15:0]reg_2,
  output reg [15:0]reg_3);

  //shifting the registers from top to bottom handles the vertical shifting
  always @ ( posedge clock ) begin
    if(~reset)begin
      reg_1 <= 0;
      reg_2 <= 0;
      reg_3 <= 0;
    end
    else begin
      reg_1 <= reg_2;
      reg_2 <= reg_3;
      reg_3 <= data_input;
    end

  end

endmodule

//selects the output padding for each weight
module output_select_padding(
  input [13:0]xnor_bitcnt_add_cmp_output,
  input [4:0]xdim,
  output reg [15:0] padded_output
  );

  //combinational logic to properly left padd the output with zeros
  always @ ( * ) begin
  padded_output = 0;
    casex(xdim)
      5'd16: padded_output[15:0] = {2'b0,xnor_bitcnt_add_cmp_output[13:0]};
      5'd12: padded_output[15:0] = {6'b0,xnor_bitcnt_add_cmp_output[9:0]};
      5'd10: padded_output[15:0] = {8'b0,xnor_bitcnt_add_cmp_output[7:0]};
      default: padded_output = 16'bx;
    endcase
  end

endmodule

// takes 4 x 4bit inputs and calculates the sign bit from xnor and weights for variable inputs
module xnor_bitcnt_add_cmp(
  input [2:0]weight_row_1,
  input [2:0]weight_row_2,
  input [2:0]weight_row_3,
  input [2:0]input_reg_1_bits,
  input [2:0]input_reg_2_bits,
  input [2:0]input_reg_3_bits,
  output sign_output);

  wire [3:0] xnor_out_1, xnor_out_2,xnor_out_3;
  wire [3:0] bitcount_out;




  //compute the xnors
  assign xnor_out_1 = ~(weight_row_1^input_reg_1_bits);
  assign xnor_out_2 = ~(weight_row_2^input_reg_2_bits);
  assign xnor_out_3 = ~(weight_row_3^input_reg_3_bits);

  //use the xnor_out_3[2] as the cary in
  assign bitcount_out = ((xnor_out_1[0] + xnor_out_1[1]) + (xnor_out_1[2] + xnor_out_2[0]))
                        + ((xnor_out_2[1] + xnor_out_2[2])+ (xnor_out_3[0] + xnor_out_3[1]))
                        + xnor_out_3[2];

  //calculate the sign of the bitcount
  assign sign_output = bitcount_out > 4'b0100;


endmodule

module fms_controller(
  input clock,
  input reset,
  input go,
  input [4:0]ydim,
  input [15:0]input_data,
  output reg busy,
  output reg [1:0]input_load_ctrl,
  output [11:0]input_addr,
  output [11:0]weight_addr,
  output [11:0]output_addr,
  output reg output_w_enable);

parameter S0 = 3'd0,
          S1 = 3'd1,
          S2 = 3'd2,
          S3 = 3'd3,
          S4 = 3'd4,
          S5 = 3'd5,
          S6 = 3'd6;

reg [3:0] state, next_state;

reg [4:0] input_addr_offset, output_addr_offset;
reg [4:0] next_input_addr_offset, next_output_addr_offset;

reg [6:0]base_input_addr, base_ouput_addr;
reg [6:0]next_base_input_addr, next_base_ouput_addr;

// assigns the next output as current base + offset
assign input_addr = base_input_addr + input_addr_offset;
assign output_addr = base_ouput_addr + output_addr_offset;
assign weight_addr = 1;

//create the flip flops for stored values
always @ ( posedge clock ) begin
  if(~reset)begin
    state <= S0;
    input_addr_offset <= 0;
    output_addr_offset <= 0;
    base_input_addr <= 0;
    base_ouput_addr <= 0;
  end
  else begin
    state <= next_state;
    input_addr_offset <= next_input_addr_offset;
    output_addr_offset <= next_output_addr_offset;
    base_input_addr <= next_base_input_addr;
    base_ouput_addr <= next_base_ouput_addr;
  end
end

always @ ( * ) begin
  //default state
  next_state = S0;
  //busy unless in wait state
  busy = 1;
  //do not load x or y dim
  input_load_ctrl = 0;
  //output only in certain state
  output_w_enable = 0;
  //hold values unless otherwise set
  next_input_addr_offset = input_addr_offset;
  next_output_addr_offset = output_addr_offset;
  next_base_input_addr = base_input_addr;
  next_base_ouput_addr = base_ouput_addr;

  case(state)
    S0: begin
      //reset the offset registers
      next_input_addr_offset = 0;
      next_output_addr_offset = 0;
      next_base_input_addr = 0;
      next_base_ouput_addr = 0;
      busy = 0;
      //change state
      if(go)
        next_state= S1;
      else
        next_state= S0;
    end

    S1:begin
      //inc offset
      next_input_addr_offset = input_addr_offset + 1'b1;

      //detect cycle before dimensions will appear in input
      if(input_addr_offset == 2)
        next_state = S2;
      else
        next_state = S1;

    end

    S2: begin
      //inc offset
      next_input_addr_offset = input_addr_offset + 1'b1;

      //load the xdim on next clock
      input_load_ctrl = 2'b01;
      next_state = S3;

    end

    S3: begin
      //inc input offset update the bases and reset next offsets to zero
      //this allows mutiple inputs to be processed as the cycle is a loop from this point
      next_base_input_addr = next_base_input_addr + input_addr_offset + 1'b1;
      next_base_ouput_addr = next_base_ouput_addr + output_addr_offset;
      next_input_addr_offset = 0;
      next_output_addr_offset = 0;
      //load the ydim on next clock
      input_load_ctrl = 2'b11;
      next_state = S4;
    end

    S4:begin
      //inc offset
      next_input_addr_offset = input_addr_offset + 1'b1;
      //detect that outputs will be ready on next clock cycle
      if(input_addr_offset == 2)
        next_state = S5;
      else
        next_state = S4;
    end

    S5:begin
      //incr input and outputs adress and enable the output
      next_input_addr_offset = input_addr_offset + 1'b1;
      next_output_addr_offset = output_addr_offset + 1'b1;
      output_w_enable = 1;

      //detect when next dim are ready to be loaded
      if(input_addr_offset == ydim-1'd1)
        next_state = S6;
      else
        next_state = S5;

    end

    S6:begin
      //incr inputs and outputs are still processing
      next_input_addr_offset = input_addr_offset + 1'b1;
      next_output_addr_offset = output_addr_offset + 1'b1;
      output_w_enable = 1;
      //load the next dimension
      input_load_ctrl = 2'b01;
      next_state = S3;

      //detect end of data stream
      if(input_data == 16'h00ff)
        next_state = S0;

    end


  endcase
end

endmodule
