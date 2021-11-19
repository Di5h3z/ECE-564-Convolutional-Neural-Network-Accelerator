`include "sram.sv"
module tb_top();


  parameter CLK_PHASE=5;
  parameter ROUND=2;
  parameter ADDR_564=12'h01f;
  parameter num_results_564=32;
  
  time computeCycle[ROUND];
  event computeStart[ROUND];
  event computeEnd[ROUND];
  event checkFinish[ROUND];
  time startTime[ROUND];
  time endTime[ROUND];
  
  int correctResult[ROUND];
  reg [15:0] result_array[int];
  reg [15:0] golden_result_array[int];
  int i;
  int j;
  int k;
  int q;
  int p;
  //---------------------------------------------------------------------------
  // General
  //
  reg                                   clk            ;
  reg                                   reset_b        ;
  reg                                   dut_run        ;
  wire                                  dut_busy       ;
  
  //--------------------------------------------------------------------------
  //----------------------output sram---------------------------------------------
  wire                                     dut_sram_write_enable  ;
  wire [11:0]                              dut_sram_write_address ;
  wire [15:0]                              dut_sram_write_data    ;
  //-----------------------input sram--------------------------------------------
  wire [11:0]                              dut_sram_read_address  ;
  wire [15:0]                              sram_dut_read_data     ;
  //-----------------------weights ------------------------------------------                                         
  wire [11:0]                              dut_wmem_read_address    ;
  wire [15:0]                              wmem_dut_read_data       ;  // read data
  
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  //SRAM
  //sram for inputs
  sram  #(.ADDR_WIDTH    (12),
          .DATA_WIDTH    (16),
          .MEM_INIT_FILE ("input_sram.dat"        ))
          input_mem  (
          .write_enable ( 1'b0  ),
          .write_address( 12'b0 ),
          .write_data   ( 16'b0    ), 
          .read_address ( dut_sram_read_address  ),
          .read_data    ( sram_dut_read_data     ),
	  .reset        ( reset_b				 ),
          .clock        ( clk                    )
         );
		 
  //sram for weights
  sram  #(.ADDR_WIDTH    (12),
          .DATA_WIDTH    (16),
          .MEM_INIT_FILE ("weight_sram.dat"           ))
          weight_mem  (
          .write_enable ( 1'b0                     ),
          .write_address( 12'b0                    ),
          .write_data   ( 16'b0                    ), 
          .read_address ( dut_wmem_read_address  ),
          .read_data    ( wmem_dut_read_data     ), 
	  .reset        ( reset_b				 ),
          .clock        ( clk                    )
         );  
  
  //sram for outputs
  sram  #(.ADDR_WIDTH    (12),
          .DATA_WIDTH    (16),
          .MEM_INIT_FILE ("output_sram.dat"        ))
          output_mem  (
          .write_enable ( dut_sram_write_enable  ),
          .write_address( dut_sram_write_address ),
          .write_data   ( dut_sram_write_data    ), 
          .read_address ( 12'b0  ),
          .read_data    (      ),
	  .reset        ( reset_b				 ),
          .clock        ( clk                    )
         );

		 
//---------------------------------------------------------------------------
// DUT 
//---------------------------------------------------------------------------
  MyDesign dut(
//---------------------------------------------------------------------------
//Control signals
          .dut_run             ( dut_run               ), 
          .dut_busy            ( dut_busy              ),
          .reset_b             ( reset_b                       ),  
          .clk                 ( clk                         ),
 
//---------------------------------------------------------------------------
//input and output SRAM interface
          .dut_sram_write_address  ( dut_sram_write_address    ),
          .dut_sram_write_data     ( dut_sram_write_data       ),
          .dut_sram_write_enable   ( dut_sram_write_enable     ),
          .dut_sram_read_address   ( dut_sram_read_address     ),
          .sram_dut_read_data      ( sram_dut_read_data        ),
//---------------------------------------------------------------------------
//weights SRAM interface                                                       
          .dut_wmem_read_address    ( dut_wmem_read_address    ),
          .wmem_dut_read_data       ( wmem_dut_read_data       )
         );

       
  //---------------------------------------------------------------------------
  //  clk
  initial 
    begin
        clk                     = 1'b0;
        forever # CLK_PHASE clk = ~clk;
    end
  
		 
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  // Stimulus
  initial begin
	$display("-------------------------------start_simulation-------------------------------\n");
      repeat(25) @(posedge clk); 
      reset_b=0;
      dut_run=0;
      repeat(25) @(posedge clk);
      reset_b=1;
      for(j=0;j<ROUND;j=j+1) begin
         if(j!=0) wait(checkFinish[j-1]);
         input_mem.loadInitFile($sformatf("input_%0d/input_sram.dat",j));
         weight_mem.loadInitFile($sformatf("input_%0d/weight_sram.dat",j));
			
		repeat(5) @(posedge clk);
		wait(dut_busy==0);
         @(posedge clk);
         dut_run=1; // DUT starts computing
         ->computeStart[j];
         $display("-------------------------------Round %0d start-------------------------------\n",j);
         wait(dut_busy==1);
         @(posedge clk);
         dut_run=0;
         wait(dut_busy==0);
         ->computeEnd[j];
      end
  end
  
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  // Timer
  //
  initial begin
    for(k=0;k<ROUND;k=k+1) begin 
       wait(computeStart[k]);
       startTime[k]=$time;
       wait(computeEnd[k]);
       endTime[k]=$time;
       computeCycle[k]=endTime[k]-startTime[k];
     end
  end
  
  //---------------------------------------------------------------------------
  // Result collector 
  // Collect your compute results 

  initial begin
    for(q=0;q<ROUND;q=q+1) begin
       wait(computeEnd[q]);
       repeat(10) @(posedge clk);
       $display("-------------------------------Round %0d check start-------------------------------\n",q);
       $display("-------------------------------store results to g_result.dat-------------------------------\n");

       if (q==0)
		$writememb($sformatf("input_%0d/result.dat",q),output_mem.mem,12'h000,ADDR_564);
       else
        $writememb($sformatf("input_%0d/result.dat",q),output_mem.mem,12'h000,ADDR_564);
	   repeat(10) @(posedge clk);

	//---------------------------------------------------------------------------
	//---------------------------------------------------------------------------
	// Result comparator 
	// Compare your compute results with the results computed by Python script
       $display("-------------------------------load results to output_array-------------------------------\n");
       $readmemb($sformatf("input_%0d/result.dat",q),result_array);

       $display("-------------------------------load results to golden_output_array-------------------------------\n");
       $readmemb($sformatf("input_%0d/golden_outputs.dat",q),golden_result_array);
	   
	   $display("-------------------------------Round %0d start compare -------------------------------\n",q);
       if(q==0)begin
       		for(i=0;i<num_results_564;i=i+1) begin
         		if(result_array[i]==golden_result_array[i]) correctResult[q]=correctResult[q]+1;
        	end
       end
       else begin
		for(i=0;i<num_results_564;i=i+1) begin
         		if(result_array[i]==golden_result_array[i]) correctResult[q]=correctResult[q]+1;
        	end
       end
  
	   $display("-------------------------------Round %0d Your report-------------------------------\n",q);
       if(q==0)
		$display("Check 1 : Correct g results = %0d/%0d",correctResult[q],num_results_564);
 
       else
		$display("Check 1 : Correct g results = %0d/%0d",correctResult[q],num_results_564);

       $display("computeCycle=%0d",computeCycle[q]/(2*CLK_PHASE));
       $display("---------------------------------------------------------------------------------\n");
       @(posedge clk);
       ->checkFinish[q];
     end 
     $finish;
	 end
  
endmodule
