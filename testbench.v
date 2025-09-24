`timescale 1ns / 1ps

module testbench;

  // Parameters
  parameter CLK_PERIOD = 10; // Clock period in time units
  parameter SIM_TIME = 1000; // Simulation time in time units

  // Signals
  reg clk;
  // Add signals for other inputs here
  
  // Instantiate the module under test
  processor dut(
    .clk(clk)
    // Connect other inputs here
  );

  // Clock generation
  always #((CLK_PERIOD/2)) clk = ~clk;

  // Initial block
  initial begin
    // Initialize inputs
    // Add initialization for other inputs here
    clk=0;
    // Apply stimulus
    #10; // Wait a bit before starting
  
    // Start clock
    

    // Run simulation for specified time
    #SIM_TIME;
  
    // End simulation
    $stop;
  end

endmodule
