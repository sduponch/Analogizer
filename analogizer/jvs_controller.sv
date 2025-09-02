//////////////////////////////////////////////////////////////////////
// JVS Controller Module for Analogizer - ALPHA VERSION
// Partial JVS Master implementation optimized for gaming performance
// 
// ⚠️  ALPHA STATUS - INCOMPLETE PROTOCOL IMPLEMENTATION ⚠️
//
// This module implements a simplified JVS (JAMMA Video Standard) Master 
// controller that allows connecting JVS arcade cabinets to the Analogue 
// Pocket through the Analogizer. 
//
// CURRENT STATUS:
// - Core protocol working (Reset, Address assignment, Input polling)
// - Basic button mapping functional (D-PAD, face buttons, START/SELECT)
// - JVS escape sequence support implemented (D0 DF → E0, D0 CF → D0)
// - Optimized for gaming performance (1ms polling, minimal latency)
// - FPGA resource usage optimized with configurable buffer sizes
// - Protocol implementation incomplete (missing capabilities, device info)
// - Button positions may need verification/adjustment
// - Single device support only
//
// ARCHITECTURE:
// - RS485 State Machine: Manages transceiver direction and timing
// - Main State Machine: Handles JVS protocol sequence and commands  
// - RX State Machine: Processes incoming JVS responses with escape sequence decoding
// - Two-buffer system: Raw buffer for incoming data, processed buffer for unescaped data
//
// HARDWARE REQUIREMENTS:
// - External MAX485 or equivalent RS485 transceiver
// - Proper 120Ω termination for reliable communication
// - JVS-compatible arcade cabinet (tested with Namco Noir)
//
// Author: DUPONCHEEL Sébastien (sduponch on GitHub)
// Project: Analogizer JVS Controller
// Status: Alpha - Work in Progress
// Date: 2025
//////////////////////////////////////////////////////////////////////

`default_nettype none
`timescale 1ns / 1ps

module jvs_controller #(parameter MASTER_CLK_FREQ = 50_000_000)
(
    // System clock and control signals
    input wire i_clk,        // System clock (typically 50MHz)
    input wire i_rst,        // Asynchronous reset (active high)
    input wire i_ena,        // Module enable (active high)
    input wire i_stb,        // Strobe signal (not used in final version)
    
    // UART interface signals for RS485 communication
    input wire i_uart_rx,    // Serial data received from JVS device
    output wire o_uart_tx,   // Serial data transmitted to JVS device
    input wire i_sense,      // JVS SENSE line (read-only for master)
    output wire o_rx485_dir, // RS485 transceiver direction control (0=RX, 1=TX)
    
    // Output registers compatible with Analogue Pocket SNAC format
    output reg [15:0] p1_btn_state,   // Player 1 button states
    output reg [31:0] p1_joy_state,   // Player 1 analog stick states
    output reg [15:0] p2_btn_state,   // Player 2 button states
    output reg [31:0] p2_joy_state,   // Player 2 analog stick states
    output reg [15:0] p3_btn_state,   // Player 3 button states (reserved)
    output reg [15:0] p4_btn_state    // Player 4 button states (reserved)
);

    //=========================================================================
    // UART TIMING CONFIGURATION
    //=========================================================================
    // Calculate UART clock divider for 115200 baud rate
    // Formula: UART_CLKS_PER_BIT = System_Clock_Frequency / Baud_Rate
    localparam UART_CLKS_PER_BIT = MASTER_CLK_FREQ / 115200;
    
    //=========================================================================
    // UART TRANSMITTER INSTANCE
    //=========================================================================
    // Control signals for UART transmitter
    reg uart_tx_dv;              // Data valid strobe to start transmission
    reg [7:0] uart_tx_byte;      // Byte to transmit
    wire uart_tx_active;         // High when transmission is in progress
    wire uart_tx_done;           // Pulse when transmission completes
    
    // Instantiate UART transmitter module
    uart_tx #(.CLKS_PER_BIT(UART_CLKS_PER_BIT)) uart_tx_inst (
        .i_Clock(i_clk),
        .i_Tx_DV(uart_tx_dv),
        .i_Tx_Byte(uart_tx_byte),
        .o_Tx_Active(uart_tx_active),
        .o_Tx_Serial(o_uart_tx),
        .o_Tx_Done(uart_tx_done)
    );
    
    //=========================================================================
    // UART RECEIVER INSTANCE
    //=========================================================================
    // Status signals from UART receiver
    wire uart_rx_dv;             // Data valid pulse when byte is received
    wire [7:0] uart_rx_byte;     // Received byte data
    
    // Instantiate UART receiver module
    uart_rx #(.CLKS_PER_BIT(UART_CLKS_PER_BIT)) uart_rx_inst (
        .i_Clock(i_clk),
        .i_Rx_Serial(i_uart_rx),
        .o_Rx_DV(uart_rx_dv),
        .o_Rx_Byte(uart_rx_byte)
    );

    //=========================================================================
    // JVS PROTOCOL CONSTANTS
    //=========================================================================
    // Standard JVS protocol bytes as defined in JAMMA specification
    localparam JVS_SYNC_BYTE = 8'hE0;        // Frame start synchronization byte
    localparam JVS_BROADCAST_ADDR = 8'hFF;   // Broadcast address for all devices
    localparam JVS_HOST_ADDR = 8'h00;        // Host/Master address
    localparam CMD_RESET_B1 = 8'hF0;         // Reset command byte 1
    localparam CMD_RESET_B2 = 8'hD9;         // Reset command byte 2 (must follow B1)
    localparam CMD_SETADDR = 8'hF1;          // Set device address command
    localparam CMD_READID = 8'h10;           // Read device identification command
    localparam CMD_CMDREV = 8'h11;           // Command format revision command
    localparam CMD_JVSREV = 8'h12;           // JVS revision command  
    localparam CMD_COMMVER = 8'h13;          // Communications version command
    localparam CMD_FEATCHK = 8'h14;          // Feature check command
    localparam CMD_READ_INPUTS = 8'h20;      // Read input states command
    localparam STATUS_NORMAL = 8'h01;        // Normal status response code

    // JVS Escape sequence constants for data byte escaping
    localparam JVS_ESCAPE_BYTE = 8'hD0;      // Escape marker byte
    localparam JVS_ESCAPED_E0 = 8'hDF;       // E0 becomes D0 DF
    localparam JVS_ESCAPED_D0 = 8'hCF;       // D0 becomes D0 CF
    
    // JVS Frame structure constants for better code readability
    localparam JVS_SYNC_POS = 8'd0;          // Position of sync byte (E0)
    localparam JVS_ADDR_POS = 8'd1;          // Position of address byte
    localparam JVS_LENGTH_POS = 8'd2;        // Position of length byte
    localparam JVS_DATA_START = 8'd3;        // Start position of data bytes (RX)
    localparam JVS_CMD_START = 8'd3;         // Start position of command bytes (TX)
    localparam JVS_OVERHEAD = 8'd2;          // Overhead for length calculation

    // Buffer size configuration for resource optimization
    localparam RX_BUFFER_SIZE = 128;         // Size of RX buffers (I/O Identify max 106 bytes)
    localparam TX_BUFFER_SIZE = 24;          // Size of TX buffer (max frame ~21 bytes)
    
    // JVS node management constants
    localparam MAX_JVS_NODES = 2;            // Maximum supported JVS nodes (current implementation)
    localparam NODE_NAME_SIZE = 100;         // Maximum size for node identification strings (per JVS spec)

    //=========================================================================
    // STATE MACHINE DEFINITIONS
    //=========================================================================
    
    // Main State Machine - Controls overall JVS protocol sequence
    localparam STATE_IDLE = 4'h0;             // Idle state - continuous input polling
    localparam STATE_INIT_DELAY = 4'h1;       // Initial delay for system stabilization
    localparam STATE_FIRST_RESET = 4'h2;      // Send first reset command
    localparam STATE_FIRST_RESET_DELAY = 4'h3; // Delay after first reset
    localparam STATE_SECOND_RESET = 4'h4;     // Send second reset command
    localparam STATE_SECOND_RESET_DELAY = 4'h5; // Delay after second reset
    localparam STATE_SEND_SETADDR = 4'h6;     // Send address assignment command
    localparam STATE_SEND_READID = 4'h7;      // Send device ID request
    localparam STATE_SEND_CMDREV = 4'h8;      // Send command revision request
    localparam STATE_SEND_JVSREV = 4'h9;      // Send JVS revision request
    localparam STATE_SEND_COMMVER = 4'hA;     // Send communications version request
    localparam STATE_SEND_FEATCHK = 4'hB;     // Send feature check request
    localparam STATE_SEND_INPUTS = 4'hC;      // Send input state request
    localparam STATE_WAIT_TX_SETUP = 4'hD;    // Wait for RS485 setup time
    localparam STATE_TRANSMIT_BYTE = 4'hE;    // Transmit data bytes
    localparam STATE_WAIT_TX_DONE = 4'hF;     // Wait for transmission completion
    localparam STATE_WAIT_TX_HOLD = 5'h10;    // Wait for RS485 hold time
    localparam STATE_WAIT_RX = 5'h11;         // Wait for device response

    // RS485 State Machine - Controls transceiver direction with proper timing
    localparam RS485_RECEIVE = 2'b00;         // Receive mode (default)
    localparam RS485_TX_SETUP = 2'b01;        // Setup time before transmission
    localparam RS485_TRANSMIT = 2'b10;        // Active transmission mode
    localparam RS485_TX_HOLD = 2'b11;         // Hold time after transmission

    // RX State Machine - Processes incoming JVS frames byte by byte
    localparam RX_IDLE = 3'h0;                // Waiting for sync byte
    localparam RX_READ_ADDR = 3'h1;           // Reading address byte
    localparam RX_READ_SIZE = 3'h2;           // Reading length byte
    localparam RX_READ_DATA = 3'h3;           // Reading data bytes and checksum
    localparam RX_UNESCAPE = 3'h4;            // Copy from raw buffer to final buffer, processing escapes
    localparam RX_PROCESS = 3'h5;             // Processing complete and unescaped frame
    localparam RX_COPY_NAME = 3'h6;           // Copy node name from response data

    //=========================================================================
    // STATE VARIABLES AND CONTROL REGISTERS
    //=========================================================================
    // Current state for each state machine
    reg [4:0] main_state;        // Main protocol state
    reg [1:0] rs485_state;       // RS485 transceiver state
    reg [2:0] rx_state;          // Receive frame processing state
    
    // Transmission buffer and control
    reg [7:0] tx_buffer [0:TX_BUFFER_SIZE-1];  // Buffer for outgoing JVS frames
    reg [7:0] tx_length;         // Total length of current transmission
    reg [7:0] tx_counter;        // Current byte position in transmission
    reg [7:0] tx_checksum;       // Running checksum calculation
    reg rs485_tx_request;        // Signal to start RS485 transmission
    
    // Reception buffer and control
    reg [7:0] rx_buffer_raw [0:RX_BUFFER_SIZE-1]; // Buffer for raw incoming JVS frames with escape sequences
    reg [7:0] rx_buffer [0:RX_BUFFER_SIZE-1]; // Buffer for unescaped JVS frames (final processed data)
    reg [7:0] rx_length;         // Length of current incoming frame
    reg [7:0] rx_counter;        // Current byte position in reception
    reg [7:0] rx_checksum;       // Running checksum verification
    // Generic copy variables (used for unescape and name copying)
    reg [7:0] copy_read_idx;      // Read index for copy operations
    reg [7:0] copy_write_idx;     // Write index for copy operations

    // Timing and protocol control
    reg [31:0] delay_counter;    // Multi-purpose delay counter
    reg [31:0] timeout_counter;  // Timeout counter for waiting states
    reg [31:0] poll_timer;       // Timer for input polling frequency
    reg [7:0] current_device_addr; // Address assigned to JVS device (usually 0x01)
    reg rx_frame_complete;       // Flag indicating complete frame received
    reg [4:0] last_tx_state;     // Tracks the last command sent for response handling
    
    //=========================================================================
    // JVS NODE INFORMATION STRUCTURES
    //=========================================================================
    // Structure to store information about each JVS node
    reg [7:0] node_name [0:MAX_JVS_NODES-1][0:NODE_NAME_SIZE-1]; // Node identification strings
    reg [7:0] node_cmd_ver [0:MAX_JVS_NODES-1];    // Command version for each node
    reg [7:0] node_jvs_ver [0:MAX_JVS_NODES-1];    // JVS version for each node  
    reg [7:0] node_com_ver [0:MAX_JVS_NODES-1];    // Communication version for each node
    
    //=========================================================================
    // RS485 DIRECTION CONTROL
    //=========================================================================
    // Control RS485 transceiver direction based on current state
    // High = Transmit mode, Low = Receive mode
    assign o_rx485_dir = (rs485_state == RS485_TX_SETUP || 
                          rs485_state == RS485_TRANSMIT || 
                          rs485_state == RS485_TX_HOLD);

    //=========================================================================
    // RS485 STATE MACHINE
    //=========================================================================
    // Manages RS485 transceiver direction with proper setup and hold timing
    // This is critical for reliable RS485 communication
    
    reg [15:0] rs485_setup_counter; // Counter for timing delays
    
    always @(posedge i_clk) begin
        if (i_rst || !i_ena) begin
            rs485_state <= RS485_RECEIVE;
            rs485_setup_counter <= 16'h0;
        end else begin
            case (rs485_state)
                RS485_RECEIVE: begin
                    rs485_setup_counter <= 16'h0;
                    // Switch to transmit mode when requested
                    if (rs485_tx_request) begin
                        rs485_state <= RS485_TX_SETUP;
                    end
                end
                
                RS485_TX_SETUP: begin
                    // Setup time: ~10µs (500 cycles at 50MHz)
                    // This allows the RS485 transceiver to stabilize before data transmission
                    if (rs485_setup_counter < 16'd500) begin
                        rs485_setup_counter <= rs485_setup_counter + 1;
                    end else begin
                        rs485_setup_counter <= 16'h0;
                        rs485_state <= RS485_TRANSMIT;
                    end
                end
                
                RS485_TRANSMIT: begin
                    // Stay in transmit mode while data is being sent
                    if (!rs485_tx_request) begin
                        rs485_state <= RS485_TX_HOLD;
                    end
                end
                
                RS485_TX_HOLD: begin
                    // Hold TX mode after transmission (~30µs)
                    // This ensures the last bit is fully transmitted before switching to receive
                    if (rs485_setup_counter < 16'd1500) begin
                        rs485_setup_counter <= rs485_setup_counter + 1;
                    end else begin
                        rs485_setup_counter <= 16'h0;
                        rs485_state <= RS485_RECEIVE;
                    end
                end
            endcase
        end
    end

    //=========================================================================
    // MAIN STATE MACHINE - JVS PROTOCOL HANDLER
    //=========================================================================
    // Implements the complete JVS initialization sequence and input polling
    
    always @(posedge i_clk) begin
        if (i_rst || !i_ena) begin
            // Initialize all state variables on reset
            main_state <= STATE_INIT_DELAY;
            delay_counter <= 32'h0;
            timeout_counter <= 32'h0;
            poll_timer <= 32'h0;
            current_device_addr <= 8'h01;    // Standard JVS device address
            rs485_tx_request <= 1'b0;
            uart_tx_dv <= 1'b0;
            last_tx_state <= 5'h0;
        end else begin
            case (main_state)
                //-------------------------------------------------------------
                // IDLE STATE - Continuous input polling for responsive gaming
                //-------------------------------------------------------------
                STATE_IDLE: begin
                    rs485_tx_request <= 1'b0; // should be already set by STATE_WAIT_TX_HOLD
                    
                    // Fast polling timer for inputs - 1ms interval
                    // This provides responsive gaming experience with minimal latency
                    if (poll_timer < 32'h0C350) begin  // 50,000 cycles = 1ms at 50MHz
                        poll_timer <= poll_timer + 1;
                    end else begin
                        poll_timer <= 32'h0;
                        main_state <= STATE_SEND_INPUTS;
                    end
                end

                //-------------------------------------------------------------
                // INITIALIZATION DELAY - Wait for system stabilization
                //-------------------------------------------------------------
                STATE_INIT_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // Initial delay for core I/O initialization - 5.4 seconds
                    // This ensures the FPGA core and external circuits are fully stable
                    if (delay_counter < 32'h10000000) begin  // 268,435,456 cycles ≈ 5.4s at 50MHz
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_FIRST_RESET;
                    end
                end

                //-------------------------------------------------------------
                // FIRST RESET COMMAND - Begin JVS device initialization
                //-------------------------------------------------------------
                STATE_FIRST_RESET: begin
                    // Prepare first RESET command frame
                    // JVS requires two reset commands for reliable initialization
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0 - Frame start
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF - Broadcast to all devices
                    tx_buffer[JVS_CMD_START + 0] <= CMD_RESET_B1;        // F0 - Reset command byte 1
                    tx_buffer[JVS_CMD_START + 1] <= CMD_RESET_B2;        // D9 - Reset command byte 2
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;           // Request transmission
                    last_tx_state <= STATE_FIRST_RESET; // Remember command for response handling
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // DELAY AFTER FIRST RESET
                //-------------------------------------------------------------
                STATE_FIRST_RESET_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // 2 second delay after first RESET
                    // Allows JVS devices to complete their reset sequence
                    if (delay_counter < 32'h6000000) begin  // 100,663,296 cycles = 2s at 50MHz
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_SECOND_RESET;
                    end
                end

                //-------------------------------------------------------------
                // SECOND RESET COMMAND - Ensure complete device reset
                //-------------------------------------------------------------
                STATE_SECOND_RESET: begin
                    // Prepare second RESET command frame (identical to first)
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF
                    tx_buffer[JVS_CMD_START + 0] <= CMD_RESET_B1;        // F0
                    tx_buffer[JVS_CMD_START + 1] <= CMD_RESET_B2;        // D9
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SECOND_RESET;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // DELAY AFTER SECOND RESET
                //-------------------------------------------------------------
                STATE_SECOND_RESET_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // 500ms delay after second RESET
                    // Shorter delay as devices should be ready after two resets
                    if (delay_counter < 32'h1800000) begin  // 25,165,824 cycles = 500ms at 50MHz
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_SEND_SETADDR;
                    end
                end

                //-------------------------------------------------------------
                // SET ADDRESS COMMAND - Assign unique address to device
                //-------------------------------------------------------------
                STATE_SEND_SETADDR: begin
                    // Prepare SET ADDRESS command frame
                    // This assigns a unique address (0x01) to the JVS device
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF - Still broadcast for address assignment
                    tx_buffer[JVS_CMD_START + 0] <= CMD_SETADDR;         // F1 - Set address command
                    tx_buffer[JVS_CMD_START + 1] <= current_device_addr; // 01 - Address to assign
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_SETADDR;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // READ ID COMMAND - Request device identification
                //-------------------------------------------------------------
                STATE_SEND_READID: begin
                    // Prepare READ ID command frame
                    // This requests the device to send its identification string
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01 - Address specific device
                    tx_buffer[JVS_CMD_START + 0] <= CMD_READID;          // 10 - Read ID command
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_READID;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // COMMAND REVISION REQUEST - Get command format revision
                //-------------------------------------------------------------
                STATE_SEND_CMDREV: begin
                    // Prepare CMDREV command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_CMDREV;          // 11 - Command revision command
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_CMDREV;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // JVS REVISION REQUEST - Get JVS protocol revision
                //-------------------------------------------------------------
                STATE_SEND_JVSREV: begin
                    // Prepare JVSREV command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_JVSREV;          // 12 - JVS revision command
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_JVSREV;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // COMMUNICATIONS VERSION REQUEST - Get communication version
                //-------------------------------------------------------------
                STATE_SEND_COMMVER: begin
                    // Prepare COMMVER command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_COMMVER;         // 13 - Communications version command
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_COMMVER;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // FEATURE CHECK REQUEST - Get device capabilities
                //-------------------------------------------------------------
                STATE_SEND_FEATCHK: begin
                    // Prepare FEATCHK command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_FEATCHK;         // 14 - Feature check command
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_FEATCHK;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // READ INPUTS COMMAND - Request current input states
                //-------------------------------------------------------------
                STATE_SEND_INPUTS: begin
                    // Prepare READ INPUTS command frame
                    // This complex command specifies exactly which inputs to read
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_READ_INPUTS;     // 20 - Read inputs command
                    
                    // Input specification - tells device what data to return
                    tx_buffer[JVS_CMD_START + 1] <= 8'h02;               // Number of players (2)
                    tx_buffer[JVS_CMD_START + 2] <= 8'h02;               // Bytes per player (2)
                    tx_buffer[JVS_CMD_START + 3] <= 8'h21;               // Player 1 system inputs
                    tx_buffer[JVS_CMD_START + 4] <= 8'h01;               // Player 1 button inputs
                    tx_buffer[JVS_CMD_START + 5] <= 8'h22;               // Player 2 system inputs
                    tx_buffer[JVS_CMD_START + 6] <= 8'h06;               // Player 2 button inputs
                    
                    // Analog channel specifications
                    tx_buffer[JVS_CMD_START + 7] <= 8'h32;              // Analog channel 1 request
                    tx_buffer[JVS_CMD_START + 8] <= 8'h02;              // 2 bytes of analog data
                    tx_buffer[JVS_CMD_START + 9] <= 8'h00;              // Analog 1 data MSB
                    tx_buffer[JVS_CMD_START + 10] <= 8'h00;              // Analog 1 data LSB
                    tx_buffer[JVS_CMD_START + 11] <= 8'h33;              // Analog channel 2 request
                    tx_buffer[JVS_CMD_START + 12] <= 8'h02;              // 2 bytes of analog data
                    tx_buffer[JVS_CMD_START + 13] <= 8'h00;              // Analog 2 data MSB
                    tx_buffer[JVS_CMD_START + 14] <= 8'h00;              // Analog 2 data LSB
                    tx_buffer[JVS_CMD_START + 15] <= 8'h00;              // Padding byte
                    tx_buffer[JVS_CMD_START + 16] <= 8'h00;              // Padding byte
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 16;               // 16 data bytes + overhead
                    
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_INPUTS;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // WAIT FOR RS485 SETUP - Ensure proper transceiver timing
                //-------------------------------------------------------------
                STATE_WAIT_TX_SETUP: begin
                    // Wait for RS485 transceiver to enter transmit mode
                    if (rs485_state == RS485_TRANSMIT) begin
                        tx_counter <= 8'h00;                    // Reset byte counter
                        tx_checksum <= 8'h00;                   // Reset checksum
                        tx_length <= JVS_DATA_START + tx_buffer[JVS_LENGTH_POS];       // Calculate total frame length
                        main_state <= STATE_TRANSMIT_BYTE;
                    end
                end

                //-------------------------------------------------------------
                // TRANSMIT BYTES - Send frame data byte by byte
                //-------------------------------------------------------------
                STATE_TRANSMIT_BYTE: begin
                    if (tx_counter < tx_length) begin
                        // Wait for UART to be ready for next byte
                        if (!uart_tx_active && !uart_tx_dv) begin
                            // Calculate running checksum for data bytes
                            // (Skip sync byte and checksum position)
                            if (tx_counter > 0 && tx_counter < tx_length - 1) begin
                                tx_checksum <= tx_checksum + tx_buffer[tx_counter];
                            end
                            
                            // Send either data byte or calculated checksum
                            if (tx_counter == tx_length - 1) begin
                                uart_tx_byte <= tx_checksum;        // Send checksum as last byte
                            end else begin
                                uart_tx_byte <= tx_buffer[tx_counter]; // Send data byte
                            end
                            
                            uart_tx_dv <= 1'b1;                     // Start UART transmission
                            main_state <= STATE_WAIT_TX_DONE;
                        end
                    end else begin
                        // All bytes transmitted
                        rs485_tx_request <= 1'b0;               // Release RS485 request
                        main_state <= STATE_WAIT_TX_HOLD;
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR TRANSMISSION COMPLETION
                //-------------------------------------------------------------
                STATE_WAIT_TX_DONE: begin
                    // Clear data valid signal when UART starts transmission
                    if (uart_tx_dv && uart_tx_active) begin
                        uart_tx_dv <= 1'b0;
                    end
                    // Move to next byte when current transmission completes
                    if (uart_tx_done) begin
                        tx_counter <= tx_counter + 1;
                        main_state <= STATE_TRANSMIT_BYTE;
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR RS485 HOLD TIME
                //-------------------------------------------------------------
                STATE_WAIT_TX_HOLD: begin
                    // Wait for RS485 to return to receive mode
                    if (rs485_state == RS485_RECEIVE) begin
                        timeout_counter <= 32'h0;
                        // Determine next state based on what was just transmitted
                        case (last_tx_state)
                            STATE_FIRST_RESET: main_state <= STATE_FIRST_RESET_DELAY;
                            STATE_SECOND_RESET: main_state <= STATE_SECOND_RESET_DELAY;
                            STATE_SEND_INPUTS: main_state <= STATE_WAIT_RX;
                            default: main_state <= STATE_WAIT_RX;  // Commands expecting response
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR DEVICE RESPONSE
                //-------------------------------------------------------------
                STATE_WAIT_RX: begin
                    if (rx_frame_complete) begin
                        // Process response based on command sent
                        case (tx_buffer[JVS_CMD_START])
                            CMD_SETADDR: main_state <= STATE_SEND_READID;    // Address set, now read ID
                            CMD_READID: main_state <= STATE_SEND_CMDREV;     // ID read, get command revision
                            CMD_CMDREV: main_state <= STATE_SEND_JVSREV;     // Command revision read, get JVS revision
                            CMD_JVSREV: main_state <= STATE_SEND_COMMVER;    // JVS revision read, get comm version
                            CMD_COMMVER: main_state <= STATE_SEND_FEATCHK;   // Comm version read, check features
                            CMD_FEATCHK: main_state <= STATE_IDLE;           // Features checked, start polling
                            CMD_READ_INPUTS: main_state <= STATE_IDLE;       // Inputs read, continue polling
                            default: main_state <= STATE_IDLE;
                        endcase
                    end else if (timeout_counter < 32'h0C3500) begin  // 10ms timeout - fast for responsive gaming
                        timeout_counter <= timeout_counter + 1;
                    end else begin
                        // Timeout handling - different strategies for different commands
                        case (tx_buffer[JVS_CMD_START])
                            CMD_SETADDR: main_state <= STATE_FIRST_RESET;    // Critical - restart sequence
                            CMD_READID: main_state <= STATE_SEND_READID;     // Retry ID read
                            CMD_CMDREV: main_state <= STATE_SEND_CMDREV;     // Retry command revision
                            CMD_JVSREV: main_state <= STATE_SEND_JVSREV;     // Retry JVS revision
                            CMD_COMMVER: main_state <= STATE_SEND_COMMVER;   // Retry comm version
                            CMD_FEATCHK: main_state <= STATE_SEND_FEATCHK;   // Retry feature check
                            default: main_state <= STATE_IDLE;               // Continue with polling
                        endcase
                    end
                end

                default: main_state <= STATE_IDLE;
            endcase
        end
    end

    //=========================================================================
    // RX STATE MACHINE - PROCESSES INCOMING JVS RESPONSES
    //=========================================================================
    // Handles byte-by-byte reception of JVS frames with checksum validation
    
    always @(posedge i_clk) begin
        if (i_rst || !i_ena) begin
            // Initialize RX state machine and output registers
            rx_state <= RX_IDLE;
            rx_frame_complete <= 1'b0;
            rx_counter <= 8'h00;
            rx_length <= 8'h00;
            rx_checksum <= 8'h00;
            
            // Initialize output button and joystick states
            p1_btn_state <= 16'h0000;           // All buttons released
            p1_joy_state <= 32'h80808080;       // Analog sticks centered (0x80 = center)
            p2_btn_state <= 16'h0000;
            p2_joy_state <= 32'h80808080;
            p3_btn_state <= 16'h0000;
            p4_btn_state <= 16'h0000;
        end else begin
            // Clear frame complete flag when main state machine processes it
            if (main_state != STATE_WAIT_RX) begin
                rx_frame_complete <= 1'b0;
            end
            
            // Process incoming bytes from UART
            if (uart_rx_dv) begin
                case (rx_state)
                    //-----------------------------------------------------
                    // RX_IDLE - Wait for frame start (sync byte)
                    //-----------------------------------------------------
                    RX_IDLE: begin
                        if (uart_rx_byte == JVS_SYNC_BYTE) begin  // E0 detected
                            rx_counter <= 8'h01;                  // Next byte position
                            rx_checksum <= 8'h00;                 // Reset checksum
                            rx_buffer_raw[0] <= uart_rx_byte;         // Store sync byte
                            rx_frame_complete <= 1'b0;
                            rx_state <= RX_READ_ADDR;
                        end else begin
                            rx_counter <= 0;                      // Reset on invalid data
                        end
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_ADDR - Read address byte (should be 0x00 for master)
                    //-----------------------------------------------------
                    RX_READ_ADDR: begin
                        if (uart_rx_byte == JVS_HOST_ADDR) begin          // Valid master address
                            rx_buffer_raw[rx_counter] <= uart_rx_byte;
                            rx_checksum <= rx_checksum + uart_rx_byte; // Add to checksum
                            rx_counter <= rx_counter + 1;
                            rx_state <= RX_READ_SIZE;
                        end else begin
                            rx_counter <= 0;                      // Invalid address, restart
                            rx_state <= RX_IDLE;
                        end
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_SIZE - Read frame length byte
                    //-----------------------------------------------------
                    RX_READ_SIZE: begin
                        rx_buffer_raw[rx_counter] <= uart_rx_byte;
                        rx_checksum <= rx_checksum + uart_rx_byte;
                        rx_length <= uart_rx_byte;                // Store frame length
                        rx_counter <= rx_counter + 1;
                        rx_state <= RX_READ_DATA;
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_DATA - Read data bytes and validate checksum
                    //-----------------------------------------------------
                    RX_READ_DATA: begin
                        rx_buffer_raw[rx_counter] <= uart_rx_byte;
                        
                        if (rx_counter < (JVS_OVERHEAD + rx_length)) begin
                            // Still reading data bytes
                            rx_checksum <= rx_checksum + uart_rx_byte;
                            rx_counter <= rx_counter + 1;
                        end else begin
                            // Last byte (checksum) received
                            if (rx_checksum == uart_rx_byte) begin
                                rx_state <= RX_UNESCAPE;          // Checksum valid, start unescaping
                                copy_read_idx <= 8'd0;            // Start reading from beginning
                                copy_write_idx <= 8'd0;           // Start writing from beginning
                                rx_counter <= 0;
                            end else begin
                                rx_state <= RX_IDLE;              // Checksum invalid, discard frame
                                rx_counter <= 0;
                            end
                        end
                    end
                    
                    default: rx_state <= RX_IDLE;
                endcase
            end
            
            //-------------------------------------------------------------
            // RX_UNESCAPE - Copy from raw buffer to final buffer, processing escape sequences
            //-------------------------------------------------------------
            if (rx_state == RX_UNESCAPE) begin
                if (copy_read_idx <= (JVS_OVERHEAD + rx_length)) begin // Process header + data + checksum
                    if (copy_read_idx < JVS_DATA_START) begin
                        // Copy header bytes as-is (sync, addr, length)
                        rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end else if (copy_read_idx < (JVS_OVERHEAD + rx_length)) begin // In data section
                        // Check for escape sequences in data section
                        if (rx_buffer_raw[copy_read_idx] == JVS_ESCAPE_BYTE && 
                            copy_read_idx + 1 <= (JVS_OVERHEAD + rx_length) &&
                            (rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_E0 || 
                             rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_D0)) begin
                            // Process escape sequence
                            if (rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_E0) begin
                                rx_buffer[copy_write_idx] <= JVS_SYNC_BYTE; // D0 DF -> E0
                            end else begin
                                rx_buffer[copy_write_idx] <= JVS_ESCAPE_BYTE; // D0 CF -> D0
                            end
                            copy_read_idx <= copy_read_idx + 2; // Skip both escape bytes
                            copy_write_idx <= copy_write_idx + 1;
                            // Update length in final buffer (remove 1 byte)
                            rx_buffer[JVS_LENGTH_POS] <= rx_buffer[JVS_LENGTH_POS] - 1;
                        end else begin
                            // Normal byte, copy as-is
                            rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                            copy_read_idx <= copy_read_idx + 1;
                            copy_write_idx <= copy_write_idx + 1;
                        end
                    end else begin
                        // Copy checksum
                        rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end
                end else begin
                    // Finished unescaping, move to process
                    rx_state <= RX_PROCESS;
                end
            end

            //-------------------------------------------------------------
            // RX_COPY_NAME - Copy node name from READ ID response
            //-------------------------------------------------------------
            if (rx_state == RX_COPY_NAME) begin
                if (copy_read_idx < (JVS_OVERHEAD + rx_buffer[JVS_LENGTH_POS]) && copy_write_idx < NODE_NAME_SIZE - 1) begin
                    // Check for null terminator
                    if (rx_buffer[copy_read_idx] == 8'h00) begin
                        // Found null terminator, finish copying
                        node_name[current_device_addr - 1][copy_write_idx] <= 8'h00;
                        rx_state <= RX_PROCESS; // Move to process state
                    end else begin
                        // Copy character and advance indices
                        node_name[current_device_addr - 1][copy_write_idx] <= rx_buffer[copy_read_idx];
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end
                end else begin
                    // Reached end of buffer or max name size, null terminate and finish
                    node_name[current_device_addr - 1][copy_write_idx] <= 8'h00;
                    rx_state <= RX_PROCESS;
                end
            end

            //-------------------------------------------------------------
            // RX_PROCESS - Process complete valid frames
            //-------------------------------------------------------------
            if (rx_state == RX_PROCESS) begin
                rx_frame_complete <= 1'b1;                    // Signal frame ready to main state machine
                
                // Process responses based on the last command sent
                case (last_tx_state)
                    STATE_SEND_READID: begin
                        // Process READID response: E0 00 XX 01 01 [ASCII_NAME] 00 checksum
                        // Example: "namco ltd.;NAJV2;Ver1.00;JPN,Multipurpose."
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Trigger name copying for current node (current_device_addr - 1 as array index)
                            if (current_device_addr > 0 && current_device_addr <= MAX_JVS_NODES) begin
                                // Setup name copying: rx_buffer format [E0][00][LEN][01][01][name...][00][checksum]
                                copy_read_idx <= JVS_DATA_START + 2;    // Start after status and report bytes
                                copy_write_idx <= 8'd0;                 // Start writing at beginning of name array
                                rx_frame_complete <= 1'b0;              // Clear frame complete flag
                                rx_state <= RX_COPY_NAME;               // Switch to name copying state
                            end
                        end
                    end
                    
                    STATE_SEND_CMDREV: begin
                        // Process CMDREV response: E0 00 XX 01 YY (where YY is revision in BCD)
                        // Current expected revision is 1.3, so YY should be 0x13
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 3) begin
                            // Store command revision for current node (current_device_addr - 1 as array index)
                            if (current_device_addr > 0 && current_device_addr <= MAX_JVS_NODES) begin
                                node_cmd_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[4] contains revision
                            end
                        end
                    end
                    
                    STATE_SEND_JVSREV: begin
                        // Process JVSREV response: E0 00 XX 01 YY (where YY is JVS revision in BCD)  
                        // Current expected revision is 3.0, so YY should be 0x30
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 3) begin
                            // Store JVS revision for current node (current_device_addr - 1 as array index)
                            if (current_device_addr > 0 && current_device_addr <= MAX_JVS_NODES) begin
                                node_jvs_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[4] contains revision
                            end
                        end
                    end
                    
                    STATE_SEND_COMMVER: begin
                        // Process COMMVER response: E0 00 XX 01 YY (where YY is comm version in BCD)
                        // Current expected version is 1.0, so YY should be 0x10  
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 3) begin
                            // Store communication version for current node (current_device_addr - 1 as array index)
                            if (current_device_addr > 0 && current_device_addr <= MAX_JVS_NODES) begin
                                node_com_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[4] contains version
                            end
                        end
                    end
                    
                    STATE_SEND_FEATCHK: begin
                        // Process FEATCHK response: E0 00 XX 01 [function_data...] 00
                        // Contains 4-byte function descriptors followed by 00 terminator
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Parse feature data (optional - could extract supported functions)
                            // Format: [func_code][param1][param2][param3] repeating, then 00
                            // For now we just acknowledge receipt
                        end
                    end
                    
                    STATE_SEND_INPUTS: begin
                        // Process input data response
                        // Validate response format: E0 00 XX 01 (sync, master addr, length, normal status)
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_DATA_START] == STATUS_NORMAL) begin
                            // Ensure minimum frame size for button data
                            if (rx_buffer[JVS_LENGTH_POS] >= 8) begin
                            
                            //=================================================
                            // PLAYER 1 BUTTON MAPPING
                            //=================================================
                            // Map JVS button data to Analogue Pocket format
                            // JVS Frame format for inputs:
                            // rx_buffer[6] = P1 buttons (A,B,X,Y,L1,R1,SELECT + unused bit)
                            // rx_buffer[7] = P1 directions (UP,DOWN,LEFT,RIGHT + unused + unused + unused + START)
                            //
                            // Analogue Pocket button format (p1_btn_state[15:0]):
                            // [15] START, [14] SELECT, [13] R3, [12] L3, [11] R2, [10] L2
                            // [9] R1, [8] L1, [7] Y, [6] X, [5] B, [4] A
                            // [3] RIGHT, [2] LEFT, [1] DOWN, [0] UP
                            
                            // D-PAD mapping (directional pad)
                            p1_btn_state[0] <= rx_buffer[6][5];   // UP - Map from JVS bit 5
                            p1_btn_state[1] <= rx_buffer[6][4];   // DOWN - Map from JVS bit 4  
                            p1_btn_state[2] <= rx_buffer[6][3];   // LEFT - Map from JVS bit 3
                            p1_btn_state[3] <= rx_buffer[6][2];   // RIGHT - Map from JVS bit 2
                            
                            // Face buttons mapping (main action buttons)
                            p1_btn_state[4] <= rx_buffer[6][1];   // A - Map from JVS button bit 1
                            p1_btn_state[5] <= rx_buffer[6][0];   // B - Map from JVS button bit 0
                            p1_btn_state[6] <= rx_buffer[7][6];   // X - Map from JVS directions bit 6
                            p1_btn_state[7] <= rx_buffer[7][5];   // Y - Map from JVS directions bit 5
                            
                            // Trigger buttons (shoulder buttons) - not used in basic JVS
                            p1_btn_state[8] <= 1'b0;              // L1 - Not available in this JVS config
                            p1_btn_state[9] <= 1'b0;              // R1 - Not available in this JVS config
                            p1_btn_state[10] <= 1'b0;             // L2 - Not available in this JVS config
                            p1_btn_state[11] <= 1'b0;             // R2 - Not available in this JVS config
                            p1_btn_state[12] <= 1'b0;             // L3 - Not available in this JVS config
                            p1_btn_state[13] <= 1'b0;             // R3 - Not available in this JVS config
                            
                            // System buttons mapping
                            p1_btn_state[14] <= rx_buffer[6][6];  // SELECT - Map from JVS button bit 6
                            p1_btn_state[15] <= rx_buffer[6][7];  // START - Map from JVS button bit 7

                            //=================================================
                            // PLAYER 2 BUTTON MAPPING (if present in frame)
                            //=================================================
                            if (rx_buffer[JVS_LENGTH_POS] >= 10) begin  // Check if frame contains P2 data
                                // JVS P2 data typically at positions 8 and 9
                                // rx_buffer[8] = P2 buttons, rx_buffer[9] = P2 directions
                                
                                // P2 D-PAD mapping
                                p2_btn_state[0] <= rx_buffer[8][5];   // P2 UP
                                p2_btn_state[1] <= rx_buffer[8][4];   // P2 DOWN
                                p2_btn_state[2] <= rx_buffer[8][3];   // P2 LEFT
                                p2_btn_state[3] <= rx_buffer[8][2];   // P2 RIGHT
                                
                                // P2 Face buttons mapping
                                p2_btn_state[4] <= rx_buffer[8][1];   // P2 A
                                p2_btn_state[5] <= rx_buffer[8][0];   // P2 B
                                p2_btn_state[6] <= rx_buffer[9][6];   // P2 X
                                p2_btn_state[7] <= rx_buffer[9][5];   // P2 Y
                                
                                // P2 Trigger buttons (not used)
                                p2_btn_state[8] <= 1'b0;              // P2 L1
                                p2_btn_state[9] <= 1'b0;              // P2 R1
                                p2_btn_state[10] <= 1'b0;             // P2 L2
                                p2_btn_state[11] <= 1'b0;             // P2 R2
                                p2_btn_state[12] <= 1'b0;             // P2 L3
                                p2_btn_state[13] <= 1'b0;             // P2 R3
                                
                                // P2 System buttons mapping
                                p2_btn_state[14] <= rx_buffer[8][6];  // P2 SELECT
                                p2_btn_state[15] <= rx_buffer[8][7];  // P2 START
                            end else begin
                                // No P2 data in frame, clear P2 button state
                                p2_btn_state <= 16'h0000;
                            end
                            
                            //=================================================
                            // ANALOG STICK DATA MAPPING (if present in frame)
                            //=================================================
                            if (rx_buffer[JVS_LENGTH_POS] >= 14) begin  // Check if frame contains analog data
                                // JVS analog data format: 8-bit values where 0x80 = center position
                                // Analogue Pocket expects same format: 0x80 = center
                                
                                // Player 1 analog stick data (typically at positions 10-13)
                                p1_joy_state[7:0] <= rx_buffer[10];    // Left stick X axis
                                p1_joy_state[15:8] <= rx_buffer[11];   // Left stick Y axis  
                                p1_joy_state[23:16] <= rx_buffer[12];  // Right stick X axis (if available)
                                p1_joy_state[31:24] <= rx_buffer[13];  // Right stick Y axis (if available)
                            end else begin
                                // No analog data, set sticks to center position
                                p1_joy_state <= 32'h80808080;  // All axes centered
                            end
                            
                            //=================================================
                            // PLAYER 2 ANALOG DATA (if present in frame)
                            //=================================================
                            if (rx_buffer[JVS_LENGTH_POS] >= 18) begin  // Check if frame contains P2 analog data
                                // Player 2 analog stick data (typically at positions 14-17)
                                p2_joy_state[7:0] <= rx_buffer[14];    // P2 Left stick X
                                p2_joy_state[15:8] <= rx_buffer[15];   // P2 Left stick Y
                                p2_joy_state[23:16] <= rx_buffer[16];  // P2 Right stick X
                                p2_joy_state[31:24] <= rx_buffer[17];  // P2 Right stick Y
                            end else begin
                                // No P2 analog data, set sticks to center position
                                p2_joy_state <= 32'h80808080;
                            end
                        end
                    end
                    end
                    
                    default: begin
                        // For other commands (SETADDR, READID, etc.), just acknowledge receipt
                        // No special processing needed
                    end
                endcase
                
                // Frame processing complete, return to idle state for next frame
                rx_counter <= 8'h00;
                rx_state <= RX_IDLE;
            end
        end
    end
     
endmodule

//=============================================================================
// END OF JVS CONTROLLER MODULE
//=============================================================================

/*
USAGE NOTES:
============

1. Clock Frequency:
   - Module is designed for 50MHz system clock
   - UART baud rate automatically calculated as MASTER_CLK_FREQ / 115200
   - UART modules has been modified from nandland to handle more than 256 cycles single bit time
   - For different clock frequencies, adjust MASTER_CLK_FREQ parameter

2. RS485 Hardware Requirements:
   - External MAX485 or equivalent RS485 transceiver required
   - Connect o_rx485_dir (SNAC_OUT2) to transceiver DE (Driver Enable) and /RE (Receiver Enable) pins (USB D+, the GREEN one)
   - Connect SNAC_OUT1 to DI (USB D-, the WHITE one)
   - Connect IN4 (USB 3.0 RX+, the YELLOW one from twisted pair on my cable) to RO.
   - In a next release, an additional INPUT (IN7) will be required for JVS SENSE (to support chained JVS boards)

3. JVS Device Compatibility:
   - Tested with Namco Noir Cabinet JVS devices, will soon be tested on Viewlix.
   - Should work with most standard JVS arcade systems
   - Some devices may require timing adjustments

4. Timing Considerations:
   - Initial 5.4s delay ensures system stability
   - 2s delay between RESET requests
   - 10ms timeout prevents freeze on invalid frames

6. Debugging:
   - Monitor o_rx485_dir for transmission timing
   - Monitor in the same time data on A or B.
   - Check UART RX signals for communication issues

7. Extensions:
   - Additional players can be supported by extending frame parsing (up to 4 players)
   - Light GUN can be supported (i have a HUUUUGGGEEE TIME CRISIS 4 cabinet for that)
   - Steering wheel ca be supported (with force feeback ?) i have an Initial D 8 Inifinty cabinet too.
   - More JVS commands can be added to command handling
   - Configurable device addressing possible

JVS FRAME ANALYSIS - ACTUAL CAPTURED TRACES:
=============================================

The following section documents the actual JVS communication traces captured 
during development and testing with a Namco Noir cabinet. These real traces 
has served as reference for current JVS protocol implementation.

COMPLETE COMMUNICATION SEQUENCE:
--------------------------------

1. FIRST RESET COMMAND:
   Raw: E0 FF 03 F0 D9 CB
   Decode:
   - E0: Sync byte (frame start)
   - FF: Broadcast address (all devices)
   - 03: Data length (3 bytes total)
   - F0: Reset command byte 1
   - D9: Reset command byte 2  
   - CB: Checksum (FF+03+F0+D9 = 02CB, low byte = CB)

2. SECOND RESET COMMAND:
   Raw: E0 FF 03 F0 D9 CB
   Decode: Identical to first reset (JVS specification requires double reset)

3. SET ADDRESS COMMAND:
   Raw: E0 FF 03 F1 01 F4
   Decode:
   - E0: Sync byte
   - FF: Broadcast address (for initial address assignment)
   - 03: Data length
   - F1: Set address command
   - 01: Address to assign to device
   - F4: Checksum (FF+03+F1+01 = 01F4, low byte = F4)

4. SET ADDRESS RESPONSE (ACK):
   Raw: E0 00 03 01 01 05
   Decode:
   - E0: Sync byte
   - 00: Master address (device responding to master)
   - 03: Data length
   - 01: Status byte (01 = normal/success)
   - 01: Report data (address accepted)
   - 05: Checksum (00+03+01+01 = 05)

5. UNKNOWN COMMAND:
   Raw: E0 01 04 11 12 13 3B
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 04: Data length
   - 11: Unknown command (possibly device-specific)
   - 12 13: Unknown parameters
   - 3B: Checksum (01+04+11+12+13 = 3B)

6. UNKNOWN COMMAND RESPONSE:
   Raw: E0 00 08 01 01 13 01 30 01 10 5F
   Decode:
   - E0: Sync byte
   - 00: Master address
   - 08: Data length (8 bytes)
   - 01: Status (normal)
   - 01 13 01 30 01 10: Unknown response data
   - 5F: Checksum

7. READ ID COMMAND:
   Raw: E0 01 02 10 13
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 02: Data length
   - 10: Read device ID command
   - 13: Checksum (01+02+10 = 13)

8. READ ID RESPONSE:
   Raw: E0 00 2E 01 01 6E 61 6D 63 6F 20 6C 74 64 2E 3B 4E 41 4A 56 32 3B 56 65 72 31 2E 30 30 3B 4A 50 4E 2C 4D 75 6C 74 69 70 75 72 70 6F 73 65 2E 00 29
   Decode:
   - E0: Sync byte
   - 00: Master address
   - 2E: Data length (46 bytes)
   - 01: Status (normal)
   - 01: Report follows
   - ASCII String: "namco ltd.;NAJV2;Ver1.00;JPN,Multipurpose."
     6E 61 6D 63 6F 20 6C 74 64 2E 3B = "namco ltd.;"
     4E 41 4A 56 32 3B = "NAJV2;"
     56 65 72 31 2E 30 30 3B = "Ver1.00;"
     4A 50 4E 2C 4D 75 6C 74 69 70 75 72 70 6F 73 65 2E = "JPN,Multipurpose."
   - 00: String terminator
   - 29: Checksum

9. CAPABILITIES COMMAND:
   Raw: E0 01 02 14 17
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 02: Data length
   - 14: Get capabilities command
   - 17: Checksum (01+02+14 = 17)

10. CAPABILITIES RESPONSE:
    Raw: E0 00 18 01 01 01 02 0D 00 02 02 00 00 03 08 10 00 12 12 00 00 13 02 00 00 00 82
    Decode:
    - E0: Sync byte
    - 00: Master address
    - 18: Data length (24 bytes)
    - 01: Status (normal)
    - 01: Report follows
    - Capabilities data:
      01: Input function (digital inputs)
      02: Number of players supported (2)
      0D: Button configuration
      00: Reserved
      02: Input function (analog inputs)
      02: Number of analog channels
      00 00: Reserved
      03: Input function (rotary encoders)
      08: Number of rotary channels
      10 00: Rotary configuration
      12: Input function (keypad)
      12: Keypad configuration
      00 00: Reserved
      13: Input function (lightgun)
      02: Number of lightgun inputs
      00 00 00: Reserved
    - 82: Checksum

11. DEVICE INFORMATION COMMAND:
    Raw: E0 01 1C 15 4E 42 47 49 2E 3B 57 69 6E 41 72 63 3B 56 65 72 32 2E 31 3B 4A 50 4E 00 7A
    Decode:
    - E0: Sync byte
    - 01: Device address
    - 1C: Data length (28 bytes)
    - 15: Device information command
    - ASCII String: "NBGI.;WinArc;Ver2.1;JPN"
      4E 42 47 49 2E 3B = "NBGI.;"
      57 69 6E 41 72 63 3B = "WinArc;"
      56 65 72 32 2E 31 3B = "Ver2.1;"
      4A 50 4E = "JPN"
    - 00: String terminator
    - 7A: Checksum

12. DEVICE INFO ACK:
    Raw: E0 00 03 01 01 05
    Decode: Same format as SET ADDRESS response (acknowledgment)

13. READ INPUTS COMMAND:
    Raw: E0 01 12 20 02 02 21 01 22 06 32 02 00 00 33 02 00 00 00 00 EA
    Decode:
    - E0: Sync byte
    - 01: Device address
    - 12: Data length (18 bytes)
    - 20: Read inputs command
    - 02: Number of players (2)
    - 02: Bytes per player (2 bytes each)
    - 21: Player 1 system/direction byte request
    - 01: Player 1 button byte request
    - 22: Player 2 system/direction byte request
    - 06: Player 2 button byte request
    - 32: Analog channel 1 request
    - 02: 2 bytes of analog data requested
    - 00 00: Analog channel 1 default values
    - 33: Analog channel 2 request
    - 02: 2 bytes of analog data requested
    - 00 00: Analog channel 2 default values
    - 00 00: Padding
    - EA: Checksum

14. READ INPUTS RESPONSE (no buttons pressed):
    Raw: E0 00 1A 01 01 00 00 00 00 00 01 00 00 01 B5 00 B4 40 B3 C0 B2 80 B5 00 B4 80 01 01 57
    Decode:
    - E0: Sync byte
    - 00: Master address  
    - 1A: Data length (26 bytes)
    - 01: Status (normal)
    - 01: Report data follows
    - Button/Direction Data:
      00: Player 1 buttons (all released)
      00: Player 1 directions (centered/no input)
      00: Player 2 buttons (all released)
      00: Player 2 directions (centered/no input)
      00: Additional system byte
    - Analog Data Block:
      01 00 00 01: Unknown format/header
      B5 00: Analog channel data
      B4 40: Analog channel data
      B3 C0: Analog channel data
      B2 80: Analog channel data (0x80 = center)
      B5 00: Analog channel data
      B4 80: Analog channel data (0x80 = center)
      01 01: Status/footer
    - 57: Checksum

15. READ INPUTS COMMAND (variant):
    Raw: E0 01 12 20 02 02 21 01 22 06 32 02 00 80 33 02 00 00 00 00 6A
    Decode: Similar to #13 but with 80 in analog channel 1 (centered position)

16. READ INPUTS RESPONSE (variant):

CURRENT IMPLEMENTATION SEQUENCE:
===============================

The current implementation uses a simplified JVS sequence focused on input 
polling rather than full protocol compliance. Make it work first then improve it.

IMPLEMENTED COMMAND SEQUENCE:
----------------------------

1. FIRST RESET COMMAND (Implemented):
   Sent: E0 FF 03 F0 D9 CB
   - Full implementation matches captured trace
   - 2 second delay after transmission
   - No response expected (broadcast command)

2. SECOND RESET COMMAND (Implemented):
   Sent: E0 FF 03 F0 D9 CB
   - Identical to first reset
   - 500ms delay after transmission
   - Ensures device is fully reset

3. SET ADDRESS COMMAND (Implemented):
   Sent: E0 FF 03 F1 01 F4
   Expected Response: E0 00 03 01 01 05
   - Waits for ACK response with 2s timeout
   - On success: proceeds to READ ID
   - On timeout: restarts reset sequence
    ⚠️ Should rely on SENSE Line going low to know if all JVS boards are initialized, support only one board for now.

4. READ ID COMMAND (Implemented):
   Sent: E0 01 02 10 13
   Expected Response: E0 00 2E 01 01 [ASCII_STRING] [checksum]
   - Waits for device identification
   - Response parsed but not used for device-specific logic
   - On success: starts input polling loop

5. READ INPUTS COMMAND (Implemented - Simplified):
   Sent: E0 01 12 20 02 02 21 01 22 06 32 02 00 00 33 02 00 00 00 00 EA
   Expected Response: E0 00 1A 01 01 [button_data] [analog_data] [checksum]
   - Continuous 1ms polling for responsive gaming
   - Only processes button/direction data at positions 6-9
   - Analog data received but only basic X/Y sticks mapped
   - 10ms timeout to prevent freeze on invalid frames
   ⚠️Probably a Namco specific, on original hardware the topper light is controlled by JVS (blink when the game is loading).

COMMANDS NOT IMPLEMENTED:
------------------------

The following commands from the full trace are NOT implemented in the current
version, as they are not essential for basic gaming functionality:

- Unknown Command (11 12 13): Device-specific, purpose unclear
- Capabilities Command (14): Not needed for fixed input mapping  
- Device Information Command (15): Additional device info not required

CURRENT LIMITATIONS:
-------------------

1. Button Mapping Incomplete:
   - Only basic D-PAD and face buttons mapped
   - START/SELECT functional but may need position adjustment
   - No support for additional buttons (L1/R1/L2/R2)
   - Player 2 mapping present but not fully tested

2. Analog Support Basic:
   - Only first 4 analog channels mapped (2 sticks)
   - Complex analog format from trace not fully decoded
   - Centering works (0x80) but full range may need calibration

3. Error Handling Simplified:
   - 10ms timeout for all responses (vs proper command-specific timeouts)
   - No retry logic for specific command failures
   - Limited validation of response format

4. Protocol Compliance Partial:
   - Missing capabilities negotiation
   - No device-specific optimizations
   - Simplified frame validation

4. Light gun not supported yet (can not wait to play Duck Hunt on my Time Crisis 4).

5. Steering wheel not supported yet (can not wait to play OutRun with force feedback on my Initial D).

FUTURE IMPROVEMENTS NEEDED:
--------------------------

1. Complete Button Mapping (only 4 buttons for now):

2. Test and Validate Player 2 controls mapping

2. Enhanced Analog Support:
   - Decode complex analog format from real traces
   - Implement proper calibration/centering
   - Support additional analog channels if needed

3. Robust Error Handling:
   - Implement proper retry logic for failed commands
   - Add device-specific timeout handling
   - Improve frame validation and error recovery

4. Protocol Completeness:
   - Add capabilities negotiation for device compatibility
   - Implement device information parsing
   - Support additional JVS commands as needed

DEVELOPMENT STATUS:
------------------
✅ Core Protocol: Working (Reset, Address, ID, Input polling)
✅ Basic Buttons: Working (D-PAD, 4 Face buttons, START)
✅ Basic Analog: Working (2 analog sticks with centering)
✅ Escape Sequences: Working (D0 DF → E0, D0 CF → D0 decoding)
✅ RS485 Timing: Working (Proper setup/hold times)
✅ Performance: Optimized (1ms polling, 10ms timeouts)
✅ FPGA Resources: Optimized with configurable buffer sizes (RX_BUFFER_SIZE, TX_BUFFER_SIZE)

⚠️  Button Mapping: Needs verification (START position unclear)
⚠️  Player 2: Present but not fully tested
⚠️  Advanced Analog: Basic implementation only
⚠️  Error Recovery: Simplified timeout handling

❌ Capabilities: Not implemented
❌ Device Info: Not used for logic
❌ Extended Commands: Not supported
❌ Multi-device: Single device only

BUTTON MAPPING DISCOVERY:
------------------------
During development, the following button positions were empirically determined
by pressing individual buttons and observing the bit changes:

Physical Button -> JVS Data Position -> Analogue Pocket Mapping:
START    -> rx_buffer[6][7] -> p1_btn_state[15]
SELECT   -> rx_buffer[6][6] -> p1_btn_state[14]  
Y        -> rx_buffer[7][5] -> p1_btn_state[7]
X        -> rx_buffer[7][6] -> p1_btn_state[6]
B        -> rx_buffer[6][0] -> p1_btn_state[5]
A        -> rx_buffer[6][1] -> p1_btn_state[4]
RIGHT    -> rx_buffer[6][2] -> p1_btn_state[3] (verified: 0x04 = bit 2)
LEFT     -> rx_buffer[6][3] -> p1_btn_state[2] (verified: 0x08 = bit 3)
DOWN     -> rx_buffer[6][4] -> p1_btn_state[1]
UP       -> rx_buffer[6][5] -> p1_btn_state[0]

TIMING ANALYSIS:
---------------
- JVS communication runs at 115200 baud (8N1)
- RS485 setup time: 10µs before transmission
- RS485 hold time: 30µs after transmission

ERROR CONDITIONS OBSERVED:
--------------------------
- Checksum mismatch: Device ignores frame, no response
- Invalid address: Device ignores frame, no response  
- Timeout scenarios: 10ms timeout prevents system freeze
- Cable disconnection: Continuous timeouts, system continues polling
- Power cycle: Requires full reset sequence (double reset + addressing)

PROTOCOL VARIATIONS:
-------------------
Different JVS devices may implement slight variations:
- Some devices use different button bit positions
- Analog resolution may vary (8-bit is standard)
- Response timing can vary between manufacturers
- Additional data fields for specialized controls (guns, wheels, etc.)

*/