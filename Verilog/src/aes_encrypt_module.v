`timescale 1ns / 1ps


module aes_encrypt_module(CLK,RST,START, OUT_SEL, MEM_SEL, HEX, lcd_mode );
    input CLK;
    input RST;
    input START;
    input [1:0] OUT_SEL;
    input [3:0] MEM_SEL;
    output reg [31:0] HEX;
    output reg lcd_mode;
    
    wire [127:0] PLAIN_TEXT;
    reg [127:0] R_Curr [11:0];
    wire [127:0] R_Curr_S[11:0];
    reg [127:0] R_Curr_S_shift[11:0];
    wire [127:0] R_Curr_shift_mix[11:0];
    reg [3:0] to_sub_addr;
    reg SW_CMD_back;
    reg SW_EDGE;
    integer index;
    reg compute;
    reg START_AES;
    
    wire SW_CMD;
    wire RS_CMD;
    wire sw_1_clk;
    wire sw_0_clk;
    wire [31:0] g [11:0];
    
    reg [127:0] KEY_Arr [11:0];
    reg [11:0] round_num;
    
    initial
    begin
        KEY_Arr[0] = 128'h5468617473206D79204B756E67204675;
         R_Curr[10] = 0;
    end
    
    Filter #(.wd(16), .n(65535), .bound(64000) )BGN_Filt(.clk(CLK), .data_in(START), .data_out(SW_CMD), .data_edge(sw_1_clk) );
    Filter #(.wd(16), .n(65535), .bound(64000) )RST_Filt(.clk(CLK), .data_in(RST), .data_out(RS_CMD), .data_edge(sw_0_clk) );
   
    memory memSelect( .Address(to_sub_addr), .Data(PLAIN_TEXT) ); 
    
    // all 9 instantiations of mix collumn
    Mix_Culumn mix_rnd_1(.in(R_Curr_S_shift[1]), .out(R_Curr_shift_mix[1]));
    Mix_Culumn mix_rnd_2(.in(R_Curr_S_shift[2]), .out(R_Curr_shift_mix[2]));
    Mix_Culumn mix_rnd_3(.in(R_Curr_S_shift[3]), .out(R_Curr_shift_mix[3]));
    Mix_Culumn mix_rnd_4(.in(R_Curr_S_shift[4]), .out(R_Curr_shift_mix[4]));
    Mix_Culumn mix_rnd_5(.in(R_Curr_S_shift[5]), .out(R_Curr_shift_mix[5]));
    Mix_Culumn mix_rnd_6(.in(R_Curr_S_shift[6]), .out(R_Curr_shift_mix[6]));
    Mix_Culumn mix_rnd_7(.in(R_Curr_S_shift[7]), .out(R_Curr_shift_mix[7]));
    Mix_Culumn mix_rnd_8(.in(R_Curr_S_shift[8]), .out(R_Curr_shift_mix[8]));
    Mix_Culumn mix_rnd_0(.in(R_Curr_S_shift[9]), .out(R_Curr_shift_mix[9]));
    
    // all 10 key schedule modules
    key_schedule gen_rnd_1(.round_num(1), .key(KEY_Arr[0]), .g(g[1]));
    key_schedule gen_rnd_2(.round_num(2), .key(KEY_Arr[1]), .g(g[2]));
    key_schedule gen_rnd_3(.round_num(3), .key(KEY_Arr[2]), .g(g[3]));
    key_schedule gen_rnd_4(.round_num(4), .key(KEY_Arr[3]), .g(g[4]));
    key_schedule gen_rnd_5(.round_num(5), .key(KEY_Arr[4]), .g(g[5]));
    key_schedule gen_rnd_6(.round_num(6), .key(KEY_Arr[5]), .g(g[6]));
    key_schedule gen_rnd_7(.round_num(7), .key(KEY_Arr[6]), .g(g[7]));
    key_schedule gen_rnd_8(.round_num(8), .key(KEY_Arr[7]), .g(g[8]));
    key_schedule gen_rnd_9(.round_num(9), .key(KEY_Arr[8]), .g(g[9]));
    key_schedule gen_rnd_0(.round_num(10), .key(KEY_Arr[9]), .g(g[10]));  
    
    // sends the full 32 bytes to a middle module to split the s_byte into 16 modules
    per_round_sbyte s_rnd_1(.full_hex(R_Curr[0]), .return_hex(R_Curr_S[1]));
    per_round_sbyte s_rnd_2(.full_hex(R_Curr[1]), .return_hex(R_Curr_S[2]));
    per_round_sbyte s_rnd_3(.full_hex(R_Curr[2]), .return_hex(R_Curr_S[3]));
    per_round_sbyte s_rnd_4(.full_hex(R_Curr[3]), .return_hex(R_Curr_S[4]));
    per_round_sbyte s_rnd_5(.full_hex(R_Curr[4]), .return_hex(R_Curr_S[5]));
    per_round_sbyte s_rnd_6(.full_hex(R_Curr[5]), .return_hex(R_Curr_S[6]));
    per_round_sbyte s_rnd_7(.full_hex(R_Curr[6]), .return_hex(R_Curr_S[7]));
    per_round_sbyte s_rnd_8(.full_hex(R_Curr[7]), .return_hex(R_Curr_S[8]));
    per_round_sbyte s_rnd_9(.full_hex(R_Curr[8]), .return_hex(R_Curr_S[9]));
    per_round_sbyte s_rnd_0(.full_hex(R_Curr[9]), .return_hex(R_Curr_S[10]));
    
    // given code to start the core
    always @(posedge CLK) begin
        SW_CMD_back <= SW_CMD;
        SW_EDGE <= SW_CMD & (~SW_CMD_back);
    end

    // implementation for RESET, turns display off and doesn't allow FPGA to use AES core
    always @( RST ) begin
        if ( RST ) begin
            compute <= 1; // used to control core
            lcd_mode <= 1; // sent to 7-seg
        end
        else if (~RST) begin
            compute <= 0;
            lcd_mode <= 0;
        end
    end
    
    always @( posedge SW_EDGE) 
        if (compute) begin
            to_sub_addr <= MEM_SEL; // sends the current memory selection to retrieve plaintext
            START_AES = ~START_AES; // initiated core
        end

        
    always @ (  START_AES ) begin
        R_Curr[0] = PLAIN_TEXT ^ KEY_Arr[0];
        round_num = 1;
        
        for (index = 1; index < 10; index = index + 1) begin // first 9 rounds
       
            // row shifting the s_byte bus from module
            R_Curr_S_shift[round_num] = {  R_Curr_S[round_num][127:120], R_Curr_S[round_num][87:80], R_Curr_S[round_num][47:40], R_Curr_S[round_num][7:0],
                R_Curr_S[round_num][95:88], R_Curr_S[round_num][55:48], R_Curr_S[round_num][15:8], R_Curr_S[round_num][103:96], 
                R_Curr_S[round_num][63:56], R_Curr_S[round_num][23:16], R_Curr_S[round_num][111:104], R_Curr_S[round_num][71:64],
                R_Curr_S[round_num][31:24], R_Curr_S[round_num][119:112], R_Curr_S[round_num][79:72], R_Curr_S[round_num][39:32] 
             };

            // could be done with concatination, creating new roundkey with g[] found from module
            KEY_Arr[round_num][127:96] = KEY_Arr[round_num - 1][127:96] ^ g[round_num];
            KEY_Arr[round_num][95:64] = KEY_Arr[round_num - 1][95:64] ^ KEY_Arr[round_num][127:96];
            KEY_Arr[round_num][63:32] = KEY_Arr[round_num - 1][63:32] ^ KEY_Arr[round_num][95:64];
            KEY_Arr[round_num][31:0] = KEY_Arr[round_num - 1][31:0] ^ KEY_Arr[round_num][63:32];
            
            R_Curr[round_num] = KEY_Arr[round_num] ^ R_Curr_shift_mix[round_num]; //final XOR

            round_num = round_num + 1; // increment index for matrix

        end
        
        round_num = 10; // hardcoding last round index
        
        // same s_byte row shifting
        R_Curr_S_shift[round_num] = {  R_Curr_S[round_num][127:120], R_Curr_S[round_num][87:80], R_Curr_S[round_num][47:40], R_Curr_S[round_num][7:0],
                R_Curr_S[round_num][95:88], R_Curr_S[round_num][55:48], R_Curr_S[round_num][15:8], R_Curr_S[round_num][103:96], 
                R_Curr_S[round_num][63:56], R_Curr_S[round_num][23:16], R_Curr_S[round_num][111:104], R_Curr_S[round_num][71:64],
                R_Curr_S[round_num][31:24], R_Curr_S[round_num][119:112], R_Curr_S[round_num][79:72], R_Curr_S[round_num][39:32] 
             };
        // same key schedule
        KEY_Arr[round_num][127:96] = KEY_Arr[round_num - 1][127:96] ^ g[round_num];
        KEY_Arr[round_num][95:64] = KEY_Arr[round_num - 1][95:64] ^ KEY_Arr[round_num][127:96];
        KEY_Arr[round_num][63:32] = KEY_Arr[round_num - 1][63:32] ^ KEY_Arr[round_num][95:64];
        KEY_Arr[round_num][31:0] = KEY_Arr[round_num - 1][31:0] ^ KEY_Arr[round_num][63:32];
        
        R_Curr[round_num] = KEY_Arr[round_num] ^ R_Curr_S_shift[round_num]; //XOR without column mix
    end
    
    always @(OUT_SEL, lcd_mode) begin // case statement to select the proper bank of bytes for display
            case(OUT_SEL)
                2'b00: begin
                    HEX <=  R_Curr[10][31:0];
                end
                2'b01: begin
                    HEX <=  R_Curr[10][63:32];
                end
                2'b10: begin
                    HEX <= R_Curr[10][95:64];
                end
                2'b11: begin
                    HEX <=  R_Curr[10][127:96];
                end
            endcase  
        end
        

endmodule
