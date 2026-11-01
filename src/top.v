module top #(
    parameter OPEN_DVI = 1, // v5.5 split reset domains; 0 restores vendor DVI IP
    parameter AUTO_THRESHOLD = 1, // 1: Otsu + bias; 0: v5.2 fixed threshold
    parameter EDGE_GAUSSIAN = 1,
    parameter EDGE_CLEANUP = 1,
    parameter EDGE_MIN_NEIGHBORS = 2, // eight neighbours, excluding center
    parameter CAMERA_TEST_PATTERN = 1'b0, // optional sensor-generated bars for diagnosis
    parameter [1:0] DEFAULT_IMAGE_MODE = 2'd0, // 0 raw, 1 gray, 2 gradient, 3 binary edge
    parameter [10:0] SOBEL_THRESHOLD = 11'd160, // raw |Gx|+|Gy|, range 0..2040
    parameter [10:0] THRESHOLD_STEP = 11'd16,
    parameter integer OUTPUT_MODE = 2, // 0: direct bars; 1: DDR bars; 2: camera
    parameter         USE_TPG     = (OUTPUT_MODE == 2) ? "false" : "true",
    parameter         USE_1280    = "true",
    parameter integer I2C_MODE    = 3,
    parameter integer USE_DSP_CNT = 0,
    parameter integer SCCB_LOGIC_CLK = 50_000_000
)(
	input                  clk,
//    input                  hdmi_clk,
	input                  key0_n, // Dock KEY0, formerly reset
    input                  key1_n,
    input                  key2_n,
    input                  core_key, // SOM USR.KEY F4: active HIGH, 1.5V
	inout                  cmos_scl,        //cmos i2c clock
	inout                  cmos_sda,        //cmos i2c data
	input                  cmos_vsync,      //cmos vsync
	input                  cmos_href,       //cmos hsync refrence,data valid
	input                  cmos_pclk,       //cmos pxiel clock
    output                 cmos_xclk,       //cmos externl clock 
	input  [7:0]           cmos_db,         //cmos data
	output                 cmos_rst_n,      //cmos reset 
	output                 cmos_pwdn,       //cmos power down
	
	output                 hdmi_tx_en,
    input                  hdmi_hpd,

	output [15-1:0]        ddr_addr,        //ROW_WIDTH=15
	output [3-1:0]         ddr_bank,        //BANK_WIDTH=3
	output                 ddr_cs,
	output                 ddr_ras,
	output                 ddr_cas,
	output                 ddr_we,
	output                 ddr_ck,
	output                 ddr_ck_n,
	output                 ddr_cke,
	output                 ddr_odt,
	output                 ddr_reset_n,
	output [4-1:0]         ddr_dm,          //DM_WIDTH=4
	inout  [32-1:0]        ddr_dq,          //DQ_WIDTH=32
	inout  [4-1:0]         ddr_dqs,         //DQS_WIDTH=4
	inout  [4-1:0]         ddr_dqs_n,       //DQS_WIDTH=4
  
    output                 tmds_clk_n_0,
    output                 tmds_clk_p_0,
    output [2:0]           tmds_d_n_0,      //{r,g,b}
    output [2:0]           tmds_d_p_0

);
    wire cmos_16bit_wr, cmos_frame_vs, camera_reset_n, video_clk, cmos_resetn, syn_off0_vs;
    wire [15:0] cmos_16bit_data;
    // Declared before first use: an implicit net here would hide width errors.
    wire TMDS_DDR_pll_lock;
    wire image_vs, image_hs, image_de;
    // Gowin configuration initializes this counter. Hold reset for 100ms.
    reg [22:0] por_count = 23'd0;
    // Registered terminal flag: multi-bit counter decode never directly drives reset.
    reg por_done = 1'b0;
    wire rst_n = por_done;
    always @(posedge clk) begin
        if(!por_done) begin
            por_count <= por_count + 1'b1;
            if(por_count == 23'd4999999) por_done <= 1'b1;
        end
    end

    wire [1:0] image_mode_src,image_mode_video;
    wire [10:0] image_threshold_src,image_threshold_video;
    wire edge_pseudo_enable_src;
    wire red_enable_src;
    wire edge_key_any_short;
    image_keys #(.DEFAULT_MODE(DEFAULT_IMAGE_MODE),
        .DEFAULT_THRESHOLD(AUTO_THRESHOLD ? 11'd160 : SOBEL_THRESHOLD),
        .MAX_THRESHOLD(AUTO_THRESHOLD ? 11'd304 : 11'd2040),.THRESHOLD_STEP(THRESHOLD_STEP)) u_image_keys (
        .clk(clk),.rst_n(rst_n),.key_pressed({core_key,~key2_n,~key1_n,~key0_n}),
        .any_short(edge_key_any_short),
        .mode(image_mode_src),.threshold(image_threshold_src));
    // Long KEY1 enters/leaves the pseudo-color view; any debounced short press
    // leaves it. None of the four short-press modes is modified.
    image_settings_cdc #(.DEFAULT_MODE(DEFAULT_IMAGE_MODE),
        .DEFAULT_THRESHOLD(AUTO_THRESHOLD ? 11'd160 : SOBEL_THRESHOLD)) u_image_settings (
        .src_clk(clk),.dst_clk(video_clk),.rst_n(rst_n & TMDS_DDR_pll_lock),.frame_vs(syn_off0_vs),
        .src_mode(image_mode_src),.src_threshold(image_threshold_src),
        .dst_mode(image_mode_video),.dst_threshold(image_threshold_video));
    // KEY1 long press toggles the full-screen Jet pseudo-color view; the
    // legacy short-press mode selection and threshold gestures are untouched.
    wire [1:0] edge_otsu_flags_video;
    wire [10:0] edge_otsu_unused_threshold;
    edge_otsu_key u_edge_pseudo_key(.clk(clk),.rst_n(rst_n),.clear(edge_key_any_short),
        .key_pressed(~key1_n),.enable(edge_pseudo_enable_src));
    // Independent long-KEY0 toggle; short mode selections retain recognition.
    edge_otsu_key u_red_key(.clk(clk),.rst_n(rst_n),.clear(1'b0),
        .key_pressed(~key0_n),.enable(red_enable_src));
    image_settings_cdc #(.DEFAULT_MODE(2'd0),.DEFAULT_THRESHOLD(11'd0)) u_edge_otsu_settings (
        .src_clk(clk),.dst_clk(video_clk),.rst_n(rst_n & TMDS_DDR_pll_lock),.frame_vs(image_vs),
        .src_mode({red_enable_src,edge_pseudo_enable_src}),.src_threshold(11'd0),
        .dst_mode(edge_otsu_flags_video),.dst_threshold(edge_otsu_unused_threshold));
    // Status signals remain internal for GAO; do not assign camera pins to LEDs.
    wire ack_any, ack_all;
    wire i2c_done;
    wire DDR_pll_lock, init_calib_complete;
    wire cmos_clk, aux_clk, sccb_clk, pll_stop;
    wire fifo_full, fifo_empty;
    wire video_reset_n;
    assign hdmi_tx_en = 1'b1; // K14 DVI_DIR_SEL, Dock 31005: 1=TX.
    reset_release u_video_reset(.clk(video_clk), .arst_n(rst_n & TMDS_DDR_pll_lock), .srst_n(video_reset_n));
    reset_release u_camera_reset(.clk(cmos_pclk), .arst_n(rst_n & cmos_resetn & i2c_done & init_calib_complete), .srst_n(camera_reset_n));
//memory interface
    wire                   memory_clk         ;
    wire                   dma_clk         	  ;
    wire                   cmd_ready          ;
    wire[2:0]              cmd                ;
    wire                   cmd_en             ;
    //wire[5:0]              app_burst_number   ;
    wire[ADDR_WIDTH-1:0]   addr               ;
    wire                   wr_data_rdy        ;
    wire                   wr_data_en         ;
    wire                   wr_data_end        ;
    wire[DATA_WIDTH-1:0]   wr_data            ;   
    wire[DATA_WIDTH/8-1:0] wr_data_mask       ;   
    wire                   rd_data_valid      ;  
    wire                   rd_data_end        ;//unused 
    wire[DATA_WIDTH-1:0]   rd_data            ;   
    wire                   err;

    //According to IP parameters to choose
    `define	    WR_VIDEO_WIDTH_32
    `define	DEF_WR_VIDEO_WIDTH 32

    `define	    RD_VIDEO_WIDTH_32
    `define	DEF_RD_VIDEO_WIDTH 32

    `define	USE_THREE_FRAME_BUFFER

    `define	DEF_ADDR_WIDTH 29
    `define	DEF_SRAM_DATA_WIDTH 256
    
    //=========================================================
    //SRAM parameters
    localparam ADDR_WIDTH          = `DEF_ADDR_WIDTH;        //The memory is byte, total capacity = 2^29*16bit = 8Gbit, add 1 bit of rank address, {rank[0],bank[2:0],row[15:0],cloumn[9:0]}
    localparam DATA_WIDTH          = `DEF_SRAM_DATA_WIDTH;   //Related to generating DDR3 IP
    localparam WR_VIDEO_WIDTH      = `DEF_WR_VIDEO_WIDTH;  
    localparam RD_VIDEO_WIDTH      = `DEF_RD_VIDEO_WIDTH;  

    //-------------------
    //syn_code
    wire                      syn_off0_hs;

    wire                      off0_syn_de  ;
    wire [RD_VIDEO_WIDTH-1:0] off0_syn_data;

    wire[15:0] 						write_data;

    wire[7:0]                       lut_index;
    wire[31:0]                      lut_data;
    wire i2c_err;

    assign cmos_xclk = cmos_clk;
    assign cmos_pwdn = 1'b0;
    //assign cmos_rst_n = 1'b1;
    assign cmos_rst_n = cmos_resetn;
    //assign cmos_rst_n = rst_n;
    assign write_data = cmos_16bit_data;
    //assign write_data = {cmos_16bit_data[4:0],cmos_16bit_data[10:5],cmos_16bit_data[15:11]};
    //assign hdmi_hpd = 1;

    //generate the CMOS sensor clock and the SDRAM controller, I2C controller clock
    Gowin_PLL Gowin_PLL_m0(
    	.clkin                      (clk                ),
        .init_clk                   (clk                ),
    	.clkout0                    (cmos_clk 	        ),
        .clkout1                    (aux_clk 	        ),
        .clkout2                    (memory_clk         ),
    	.lock 					    (DDR_pll_lock       ),
        .reset                      (~rst_n             ),
        .enclk0                     (1'b1               ), //input enclk0
        .enclk1                     (1'b1               ), //input enclk1
        .enclk2                     (pll_stop           )  //input enclk2
	);

    //CMOS SCCB Clock；
    wire clk_35M;
    Gowin_OSC Gowin_OSC_inst(
        .oscout(clk_35M) //output 35MHz
    );

    wire cmos_start_cfg;
    wire cmos_start_config = cmos_start_cfg & cmos_resetn;

    cmos_reset_gen #(
        .USE_DSP_CNT                (0                  ),
        .CNT_WIDTH                  (22                 )
    ) cmos_reset_gen_m0(
        .clk                        (sccb_clk           ),
        .rst_n                      (rst_n & DDR_pll_lock),
        .cmos_resetn                (cmos_resetn        ),
        .cmos_start_config          (cmos_start_cfg     )
    );

    //configure look-up table
    localparam [3:0]   LUT_ADDR_WIDTH = 4'd8;
    localparam integer CMOS_COLOR_BAR = CAMERA_TEST_PATTERN;
    generate 
        if(USE_1280 == "true") begin
            lut_ov5640_rgb565 #(
            	.HActive(12'd1280),
            	.VActive(12'd720),
            	.HTotal(13'd1892),
            	.VTotal(13'd740),
                .LUT_AW(LUT_ADDR_WIDTH),
                .USE_colour_bar(CMOS_COLOR_BAR),
                .USE_4vs3_frame(1'b0)
            )lut_ov5640_rgb565_m0(
            	.lut_index(lut_index),
            	.lut_data(lut_data)
            );
        end
        else begin
            lut_ov5640_rgb565 #(
            	.HActive(12'd800),
            	.VActive(12'd600),
            	.HTotal(13'd1892),
            	.VTotal(13'd740),
                .LUT_AW(LUT_ADDR_WIDTH),
                .USE_colour_bar(CMOS_COLOR_BAR),
                .USE_4vs3_frame(1'b1)
            )lut_ov5640_rgb565_m0(
            	.lut_index(lut_index),
            	.lut_data(lut_data)
            );
        end
    endgenerate
    
    //I2C master controller
    wire [15:0] i2c_clk_cnt;
    generate
        if(SCCB_LOGIC_CLK == 35_000_000) begin
            assign sccb_clk    = clk_35M;
            assign i2c_clk_cnt = 16'd69;
        end
        else begin
            assign sccb_clk    = clk;
            assign i2c_clk_cnt = 16'd99;
        end
    endgenerate

    generate
        if(I2C_MODE == 1) begin
            assign ack_any=1'b0; assign ack_all=1'b0;
            i2c_config i2c_config_m0(
            	.rst                        (~cmos_start_config       ),
            	.clk                        (sccb_clk                 ),
            	.clk_div_cnt                (i2c_clk_cnt              ),
            	.i2c_addr_2byte             (1'b1                     ),
            	.lut_index                  (lut_index                ),
            	.lut_dev_addr               (lut_data[31:24]          ),
            	.lut_reg_addr               (lut_data[23:8]           ),
            	.lut_reg_data               (lut_data[7:0]            ),
            	.error                      (i2c_err                  ),
            	.done                       (i2c_done                 ),
            	.i2c_scl                    (cmos_scl                 ),
            	.i2c_sda                    (cmos_sda                 )
            );
        end

        else if(I2C_MODE == 2) begin
            assign ack_any=1'b0; assign ack_all=1'b0;
            i2c_master_gw_init #(
                .DEV_ADDR_WIDTH             (4'd8                     ),
                .REG_ADDR_WIDTH             (8'd16                    ),
                .REG_DATA_WIDTH             (4'd8                     ),
                .I2C_FAST_MODE              (1'b0                     ),
                .INPUT_CLK_FREQ             (SCCB_LOGIC_CLK           ),
                .LUT_ADDR_WIDTH             (LUT_ADDR_WIDTH           )
            )i2c_master_gw_init_m0(
                .rst_n                      (cmos_start_config        ),
                .clk                        (sccb_clk                 ),
                .lut_index                  (lut_index                ),
                .lut_dev_addr               (lut_data[31:24]          ),
                .lut_reg_addr               (lut_data[23:8]           ),
                .lut_reg_data               (lut_data[7:0]            ),
                .ERROR                      (i2c_err                  ),
                .DONE                       (i2c_done                 ),
                .SCL                        (cmos_scl                 ),
                .SDA                        (cmos_sda                 )
            );
        end
        
        else begin
            sccb_init_top #(
                .DEV_ADDR_WIDTH             (4'd8                     ),
                .REG_ADDR_WIDTH             (8'd16                    ),
                .REG_DATA_WIDTH             (4'd8                     ),
                .I2C_FAST_MODE              (1'b0                     ),
                .I2C_ACK_MODE               (1'b1                     ),
                .INPUT_CLK_FREQ             (SCCB_LOGIC_CLK           ),
                .LUT_ADDR_WIDTH             (LUT_ADDR_WIDTH           ) 
            )sccb_init_top_Inst(
                .rst_n                      (cmos_start_config        ),
                .clk                        (sccb_clk                 ),
                .lut_index                  (lut_index                ),
                .lut_dev_addr               (lut_data[31:24]          ),
                .lut_reg_addr               (lut_data[23:8]           ),
                .lut_reg_data               (lut_data[7:0]            ),
                //.ERROR                      (i2c_err                  ),
                .INIT_DONE                  (i2c_done                 ),
                .ACK_ANY(ack_any), .ACK_ALL(ack_all),
                .SCL                        (cmos_scl                 ),
                .SDA                        (cmos_sda                 ) 
            );
        end
    endgenerate

    // One clock domain: PCLK + pixel-valid enable, never a fabric-generated clock.
    dvp_rgb565_capture u_capture (
        .pclk(cmos_pclk), .rst_n(camera_reset_n),
        .vsync(cmos_vsync), .href(cmos_href), .data(cmos_db),
        .pixel(cmos_16bit_data), .pixel_valid(cmos_16bit_wr),
        .frame_vs(cmos_frame_vs)
    );
    //The video output timing generator and generate a frame read data request
    //Output
    wire out_de;
    wire [11:0] active_x,active_y;

    generate 
        if(USE_1280 == "true") begin
            vga_timing #(
                .H_ACTIVE(16'd1280), 
                .H_FP(16'd110),
                .H_SYNC(16'd40),
                .H_BP(16'd220),
                .V_ACTIVE(16'd720),
                .V_FP(16'd5),
                .V_SYNC(16'd5),
                .V_BP(16'd20), 	
                .HS_POL(1'b1),   	
                .VS_POL(1'b1)
            ) vga_timing_m0(
                .clk (video_clk),
                .rst (~video_reset_n),

                .active_x(active_x),
                .active_y(active_y),

                .hs(syn_off0_hs),
                .vs(syn_off0_vs),
                .de(out_de)
            );
        end
    else begin   //800x600
        vga_timing #(
            .H_ACTIVE(16'd800), 
            .H_FP(16'd40),
            .H_SYNC(16'd128),
            .H_BP(16'd88),
            .V_ACTIVE(16'd600),
            .V_FP(16'd1),
            .V_SYNC(16'd4),
            .V_BP(16'd23), 	
            .HS_POL(1'b1),   	
            .VS_POL(1'b1)
        ) vga_timing_m0(
            .clk (video_clk),
            .rst (~video_reset_n),

            .active_x(active_x),
            .active_y(active_y),

            .hs(syn_off0_hs),
            .vs(syn_off0_vs),
            .de(out_de)
        );
    end
    endgenerate
    

    //Input testpattern
    ///--------------------------
    wire        tp0_vs_in ;
    wire        tp0_hs_in ;
    wire        tp0_de_in ;
    wire [ 7:0] tp0_data_r;
    wire [ 7:0] tp0_data_g;
    wire [ 7:0] tp0_data_b;

    generate if(USE_TPG == "true")         
    begin
        if(USE_1280 == "true") begin
            testpattern testpattern_inst_1280(
                .I_pxl_clk   (video_clk    ),//pixel clock
                .I_rst_n     (video_reset_n),//low active 
                .I_mode      (3'b010       ),//data select
                .I_single_r  (8'd255       ),
                .I_single_g  (8'd255       ),
                .I_single_b  (8'd255       ),                  //800x600    //1024x768   //1280x720   //1920x1080 
                .I_h_total   (12'd1650     ),//hor total time  // 12'd1056  // 12'd1344  // 12'd1650  // 12'd2200
                .I_h_sync    (12'd40       ),//hor sync time   // 12'd128   // 12'd136   // 12'd40    // 12'd44  
                .I_h_bporch  (12'd220      ),//hor back porch  // 12'd88    // 12'd160   // 12'd220   // 12'd148 
                .I_h_res     (12'd1280     ),//hor resolution  // 12'd800   // 12'd1024  // 12'd1280  // 12'd1920
                .I_v_total   (12'd750      ),//ver total time  // 12'd628   // 12'd806   // 12'd750   // 12'd1125 
                .I_v_sync    (12'd5        ),//ver sync time   // 12'd4     // 12'd6     // 12'd5     // 12'd5   
                .I_v_bporch  (12'd20       ),//ver back porch  // 12'd23    // 12'd29    // 12'd20    // 12'd36  
                .I_v_res     (12'd720      ),//ver resolution  // 12'd600   // 12'd768   // 12'd720   // 12'd1080 
                .I_hs_pol    (1'b1         ),//0,negative;1,positive
                .I_vs_pol    (1'b1         ),//0,negative;1,positive
                .O_de        (tp0_de_in    ),   
                .O_hs        (tp0_hs_in    ),
                .O_vs        (tp0_vs_in    ),
                .O_data_r    (tp0_data_r   ),   
                .O_data_g    (tp0_data_g   ),
                .O_data_b    (tp0_data_b   )
            );
        end 
        else begin
            testpattern testpattern_inst_800(
                .I_pxl_clk   (video_clk    ),//pixel clock
                .I_rst_n     (video_reset_n),//low active 
                .I_mode      (3'b000       ),//data select
                .I_single_r  (8'd100       ),
                .I_single_g  (8'd255       ),
                .I_single_b  (8'd100       ),                  //800x600    //1024x768   //1280x720   //1920x1080 
                .I_h_total   (12'd1056     ),//hor total time  // 12'd1056  // 12'd1344  // 12'd1650  // 12'd2200
                .I_h_sync    (12'd128      ),//hor sync time   // 12'd128   // 12'd136   // 12'd40    // 12'd44  
                .I_h_bporch  (12'd88       ),//hor back porch  // 12'd88    // 12'd160   // 12'd220   // 12'd148 
                .I_h_res     (12'd800      ),//hor resolution  // 12'd800   // 12'd1024  // 12'd1280  // 12'd1920
                .I_v_total   (12'd628      ),//ver total time  // 12'd628   // 12'd806   // 12'd750   // 12'd1125 
                .I_v_sync    (12'd4        ),//ver sync time   // 12'd4     // 12'd6     // 12'd5     // 12'd5   
                .I_v_bporch  (12'd23       ),//ver back porch  // 12'd23    // 12'd29    // 12'd20    // 12'd36  
                .I_v_res     (12'd600      ),//ver resolution  // 12'd600   // 12'd768   // 12'd720   // 12'd1080 
                .I_hs_pol    (1'b1         ),//0,negative;1,positive
                .I_vs_pol    (1'b1         ),//0,negative;1,positive
                .O_de        (tp0_de_in    ),   
                .O_hs        (tp0_hs_in    ),
                .O_vs        (tp0_vs_in    ),
                .O_data_r    (tp0_data_r   ),   
                .O_data_g    (tp0_data_g   ),
                .O_data_b    (tp0_data_b   )
            );
        end
    end
    endgenerate
    
    
    wire fb_vin_clk;
    wire fb_vin_vsync;
    wire [31:0] fb_vin_data;
    wire fb_vin_de;

    generate if(USE_TPG == "true")
    begin
        assign fb_vin_clk      = video_clk;
        assign fb_vin_vsync    = tp0_vs_in;
        assign fb_vin_data     = {tp0_data_r[7:3],tp0_data_g[7:2],tp0_data_b[7:3]};
        assign fb_vin_de       = tp0_de_in;
    end else begin //CMOS DATA
        assign fb_vin_clk      = cmos_pclk;
        assign fb_vin_vsync    = cmos_frame_vs;
        assign fb_vin_data     = write_data;
        assign fb_vin_de       = cmos_16bit_wr;
    end
    endgenerate
    
    Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst
    ( 
        .I_rst_n              (init_calib_complete ),
        .I_dma_clk            (dma_clk          ),
    `ifdef USE_THREE_FRAME_BUFFER 
        .I_wr_halt            (1'd0             ), //1:halt,  0:no halt
        .I_rd_halt            (1'd0             ), //1:halt,  0:no halt
    `endif

        // video data input       
        .I_vin0_clk           (fb_vin_clk   ),
        .I_vin0_vs_n          (~fb_vin_vsync),  //negative only 
        .I_vin0_de            (fb_vin_de    ),
        .I_vin0_data          (fb_vin_data  ),
        .O_vin0_fifo_full     (fifo_full    ),

        // video data output            
        .I_vout0_clk          (video_clk        ),
        .I_vout0_vs_n         (~syn_off0_vs     ),//negative only 
        .I_vout0_de           (out_de           ),
        .O_vout0_den          (off0_syn_de      ),
        .O_vout0_data         (off0_syn_data    ),
        .O_vout0_fifo_empty   (fifo_empty       ),
        // ddr write request
        .I_cmd_ready          (cmd_ready          ),
        .O_cmd                (cmd                ),//0:write;  1:read
        .O_cmd_en             (cmd_en             ),
        //.O_app_burst_number   (app_burst_number   ),
        .O_addr               (addr               ),//[ADDR_WIDTH-1:0]
        .I_wr_data_rdy        (wr_data_rdy        ),
        .O_wr_data_en         (wr_data_en         ),//
        .O_wr_data_end        (wr_data_end        ),//
        .O_wr_data            (wr_data            ),//[DATA_WIDTH-1:0]
        .O_wr_data_mask       (wr_data_mask       ),
        .I_rd_data_valid      (rd_data_valid      ),
        .I_rd_data_end        (rd_data_end        ),//unused 
        .I_rd_data            (rd_data            ),//[DATA_WIDTH-1:0]
        .I_init_calib_complete(init_calib_complete)
    ); 

    DDR3MI u_ddr3 
    (
        .clk                (clk                ),
        .memory_clk         (memory_clk         ),
        .pll_stop           (pll_stop           ),
        .pll_lock           (DDR_pll_lock       ),
        .rst_n              (rst_n              ),
        //.app_burst_number   (app_burst_number   ),
        .cmd_ready          (cmd_ready          ),
        .cmd                (cmd                ),
        .cmd_en             (cmd_en             ),
        .addr               (addr               ),
        .wr_data_rdy        (wr_data_rdy        ),
        .wr_data            (wr_data            ),
        .wr_data_en         (wr_data_en         ),
        .wr_data_end        (wr_data_end        ),
        .wr_data_mask       (wr_data_mask       ),
        .rd_data            (rd_data            ),
        .rd_data_valid      (rd_data_valid      ),
        .rd_data_end        (rd_data_end        ),
        .sr_req             (1'b0               ),
        .ref_req            (1'b0               ),
        .sr_ack             (                   ),
        .ref_ack            (                   ),
        .init_calib_complete(init_calib_complete),
        .clk_out            (dma_clk            ),
        .burst              (1'b1               ),
        // mem interface
        .ddr_rst            (                 ),
        .O_ddr_addr         (ddr_addr         ),
        .O_ddr_ba           (ddr_bank         ),
        .O_ddr_cs_n         (ddr_cs           ),
        .O_ddr_ras_n        (ddr_ras          ),
        .O_ddr_cas_n        (ddr_cas          ),
        .O_ddr_we_n         (ddr_we           ),
        .O_ddr_clk          (ddr_ck           ),
        .O_ddr_clk_n        (ddr_ck_n         ),
        .O_ddr_cke          (ddr_cke          ),
        .O_ddr_odt          (ddr_odt          ),
        .O_ddr_reset_n      (ddr_reset_n      ),
        .O_ddr_dqm          (ddr_dm           ),
        .IO_ddr_dq          (ddr_dq           ),
        .IO_ddr_dqs         (ddr_dqs          ),
        .IO_ddr_dqs_n       (ddr_dqs_n        )
    );
    //==============================================================================
    //TMDS TX(HDMI4)

    //---------------------------------------------
    wire [4:0] lcd_r,lcd_b;
    wire [5:0] lcd_g;
    wire lcd_vs,lcd_de,lcd_hs,lcd_dclk;
    
    assign {lcd_r,lcd_g,lcd_b}    = off0_syn_de ? off0_syn_data[15:0] : 16'h0000;//{r,g,b}
    assign lcd_vs      			  = Pout_vs_dn[1];//syn_off0_vs;
    assign lcd_hs      			  = Pout_hs_dn[1];//syn_off0_hs;
    assign lcd_de      			  = Pout_de_dn[1];//off0_syn_de;
    assign lcd_dclk    			  = video_clk;//video_clk_phs;

    reg  [1:0]  Pout_hs_dn;
    reg  [1:0]  Pout_vs_dn;
    reg  [1:0]  Pout_de_dn;

    always@(posedge video_clk or negedge video_reset_n)
    begin
        if(!video_reset_n) begin                          
            Pout_hs_dn  <= {2'b11};
            Pout_vs_dn  <= {2'b11}; 
            Pout_de_dn  <= {2'b00}; 
        end
        else begin                          
            Pout_hs_dn  <= {Pout_hs_dn[0],syn_off0_hs};
            Pout_vs_dn  <= {Pout_vs_dn[0],syn_off0_vs}; 
            Pout_de_dn  <= {Pout_de_dn[0],out_de}; 
        end
    end


    wire serial_clk;
    wire reset_phase_clk;
    wire hdmi4_rst_n;

    TMDS_PLL u_tmds_pll(
        .clkin     (clk              ),     //input clk
        .init_clk  (clk              ),
        .clkout0   (serial_clk       ),     //output clk x5ni
        .clkout2   (reset_phase_clk  ), // reset-only 9-degree pixel clock
        .clkout1   (video_clk        ),     //output clk x1
        .lock      (TMDS_DDR_pll_lock)      //output lock
        );

    assign hdmi4_rst_n = video_reset_n;




    wire dvi0_rgb_clk;
    wire dvi0_rgb_vs ;
    wire dvi0_rgb_hs ;
    wire dvi0_rgb_de ;
    wire [7:0] dvi0_rgb_r  ;
    wire [7:0] dvi0_rgb_g  ;
    wire [7:0] dvi0_rgb_b  ;

    //DVI 1 use DDR & Framebuffer. Identical in both TPG and camera builds, so
    //it is unconditional: an empty generate branch would leave these dangling.
    assign dvi0_rgb_clk = lcd_dclk;
    assign dvi0_rgb_vs  = lcd_vs;
    assign dvi0_rgb_hs  = lcd_hs;
    assign dvi0_rgb_de  = lcd_de;
    assign dvi0_rgb_r   = {lcd_r,3'd0};
    assign dvi0_rgb_g   = {lcd_g,2'd0};
    assign dvi0_rgb_b   = {lcd_b,3'd0};


    `ifdef DIAG_BARS
    // Sticky observations: green means seen since reset, not a frequency test.
    reg [10:0] diag_pc;
    reg diag_href, diag_vs, diag_pixel;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            diag_pc<=0; diag_href<=0; diag_vs<=0; diag_pixel<=0;
        end else begin
            if (!diag_pc[10]) diag_pc<=diag_pc+1'b1;
            if (cmos_href) diag_href<=1;
            if (cmos_vsync) diag_vs<=1;
            if (cmos_16bit_wr) diag_pixel<=1;
        end
    end
    wire [7:0] diag_async = {diag_pixel,diag_vs,diag_href,diag_pc[10],
                            (i2c_done & ack_all),ack_any,i2c_done,init_calib_complete};
    reg [7:0] diag_meta, diag_sync, diag_frame;
    reg [11:0] diag_x1,diag_x2,diag_y1,diag_y2;
    always @(posedge video_clk or negedge video_reset_n) begin
        if (!video_reset_n) begin
            diag_meta<=0; diag_sync<=0; diag_frame<=0;
            diag_x1<=0;diag_x2<=0;diag_y1<=0;diag_y2<=0;
        end else begin
            diag_meta<=diag_async; diag_sync<=diag_meta;
            if (syn_off0_vs) diag_frame<=diag_sync;
            diag_x1<=active_x;diag_x2<=diag_x1;
            diag_y1<=active_y;diag_y2<=diag_y1;
        end
    end
    wire [2:0] diag_column = (diag_x2<160)?0:(diag_x2<320)?1:
        (diag_x2<480)?2:(diag_x2<640)?3:(diag_x2<800)?4:
        (diag_x2<960)?5:(diag_x2<1120)?6:7;
    wire diag_on = 1'b0; // Set diagnostic expression back only when debugging.
    wire diag_good = diag_frame[diag_column];
    wire diag_gap = (diag_x2==159)||(diag_x2==319)||(diag_x2==479)||
        (diag_x2==639)||(diag_x2==799)||(diag_x2==959)||(diag_x2==1119);
    wire [7:0] diag_r = diag_gap?8'h00:(diag_good?8'h00:8'hff);
    wire [7:0] diag_g = diag_gap?8'h00:(diag_good?8'hff:8'h00);
    wire [7:0] diag_b = 8'h00;
    `endif

    wire [10:0] actual_threshold;
    wire adaptive_ready;
    wire [15:0] image_rgb;
    wire [23:0] image_rgb888={image_rgb[15:11],3'b0,image_rgb[10:5],2'b0,image_rgb[4:0],3'b0};
    wire [23:0] image_with_hud;
    video_processing #(.GAUSSIAN(EDGE_GAUSSIAN),.CLEANUP(EDGE_CLEANUP),
        .MIN_NEIGHBORS(EDGE_MIN_NEIGHBORS),.ADAPTIVE(AUTO_THRESHOLD)) u_video_processing(.clk(video_clk),.rst_n(video_reset_n),
        .in_vs(dvi0_rgb_vs),.in_hs(dvi0_rgb_hs),.in_de(dvi0_rgb_de),
        .in_rgb({lcd_r,lcd_g,lcd_b}),.mode(image_mode_video),.threshold(image_threshold_video),
        .pseudo_enable(edge_otsu_flags_video[0]),
        .out_vs(image_vs),.out_hs(image_hs),.out_de(image_de),.out_rgb(image_rgb),
        .effective_threshold(actual_threshold),.auto_ready(adaptive_ready));
    image_hud u_image_hud(.clk(video_clk),.rst_n(video_reset_n),.vs(image_vs),.de(image_de),
        .mode(image_mode_video),.threshold(actual_threshold),
        .auto_ready(adaptive_ready),.bias_value(AUTO_THRESHOLD ? $signed({1'b0,image_threshold_video})-12'sd160 : 12'sd0),
        .keys({core_key,~key2_n,~key1_n,~key0_n}),.in_rgb(image_rgb888),.out_rgb(image_with_hud));

    // Pseudo-color view overrides every mode, including RAW. The bypass is
    // only taken when the view is off, and RGB/VS/HS/DE share this condition.
    // Detect red components independently from the Otsu/display stream.
    wire red_vs,red_hs,red_de,red_found;
    wire [15:0] red_rgb;
    // v5.9: the OV5640 auto white balance settles green-yellow on this module in
    // fluorescent light, so the displayed picture gets a fixed digital trim with
    // G as the reference: R x1.047 (r + r/32 + r/64), B x1.25 (b + b/4).
    // Both terms saturate. Only the raw/overlay path is trimmed, so the edge
    // pipeline statistics stay exactly as measured.
    wire [7:0] cam_r8={lcd_r,3'd0};
    wire [7:0] cam_g8={lcd_g,2'd0};
    wire [7:0] cam_b8={lcd_b,3'd0};
    wire [8:0] wb_r9={1'b0,cam_r8}+{4'b0,cam_r8[7:5]}+{3'b0,cam_r8[7:6]};
    wire [8:0] wb_b9={1'b0,cam_b8}+{2'b0,cam_b8[7:2]};
    wire [7:0] cam_r_wb=(wb_r9>9'd255)?8'hff:wb_r9[7:0];
    wire [7:0] cam_b_wb=(wb_b9>9'd255)?8'hff:wb_b9[7:0];
    wire [15:0] cam_rgb_wb={cam_r_wb[7:3],cam_g8[7:2],cam_b_wb[7:3]};
    // Registered so the trim never becomes part of a long combinational path
    // into the overlay; DE/VS/HS move with it to keep the pixels aligned.
    reg [15:0] cam_rgb_q;
    reg cam_de_q,cam_vs_q,cam_hs_q;
    always @(posedge video_clk or negedge video_reset_n) begin
        if(!video_reset_n) begin cam_rgb_q<=0;cam_de_q<=0;cam_vs_q<=0;cam_hs_q<=0; end
        else begin
            cam_rgb_q<=cam_rgb_wb;
            cam_de_q<=dvi0_rgb_de;cam_vs_q<=dvi0_rgb_vs;cam_hs_q<=dvi0_rgb_hs;
        end
    end
    red_region_overlay #(.W(USE_1280=="true"?1280:800),
        .H(USE_1280=="true"?720:600)) u_red_overlay(
        .clk(video_clk),.rst_n(video_reset_n),.enable(edge_otsu_flags_video[1]),
        .vs(cam_vs_q),.hs(cam_hs_q),.de(cam_de_q),
        .rgb(cam_rgb_q),.ovs(red_vs),.ohs(red_hs),.ode(red_de),
        .orgb(red_rgb),.found(red_found));
    wire [23:0] red_rgb888={red_rgb[15:11],3'b0,red_rgb[10:5],2'b0,red_rgb[4:0],3'b0};
    // The overlay path and the processed path do not share latency, so the HDMI
    // source may only change during vertical blanking. A mid-frame switch tore
    // the picture and made the capture card lose sync for one frame.
    reg raw_v3_direct;
    always @(posedge video_clk or negedge video_reset_n) begin
        if(!video_reset_n) raw_v3_direct<=1'b1;
        else if (image_vs) raw_v3_direct <= (image_mode_video==0) && !edge_otsu_flags_video[0];
    end
    // Register the COMPLETE selected video bus after HUD/mode selection.
    // This breaks the combinational path into TMDS encoding for every mode.
    reg [23:0] hdmi_rgb_q;
    reg hdmi_vs_q,hdmi_hs_q,hdmi_de_q;
    always @(posedge video_clk or negedge video_reset_n) begin
        if(!video_reset_n) begin
            hdmi_rgb_q<=0;hdmi_vs_q<=0;hdmi_hs_q<=0;hdmi_de_q<=0;
        end else begin
            hdmi_vs_q <= (OUTPUT_MODE==0)?tp0_vs_in:(raw_v3_direct?red_vs:image_vs);
            hdmi_hs_q <= (OUTPUT_MODE==0)?tp0_hs_in:(raw_v3_direct?red_hs:image_hs);
            hdmi_de_q <= (OUTPUT_MODE==0)?tp0_de_in:(raw_v3_direct?red_de:image_de);
            hdmi_rgb_q <= (OUTPUT_MODE==0)?{tp0_data_r,tp0_data_g,tp0_data_b}:
                (raw_v3_direct?red_rgb888:image_with_hud);
        end
    end
    generate if(OPEN_DVI)begin:g_open_dvi
        dvi_tx_open u_tx(.pixel_clk(video_clk),.serial_clk(serial_clk),.reset_clk(reset_phase_clk),
            .arst_n(rst_n & TMDS_DDR_pll_lock),
            .vs(hdmi_vs_q),.hs(hdmi_hs_q),.de(hdmi_de_q),.rgb(hdmi_rgb_q),
            .clk_p(tmds_clk_p_0),.clk_n(tmds_clk_n_0),.data_p(tmds_d_p_0),.data_n(tmds_d_n_0));
    end else begin:g_vendor_dvi
    DVI_TX_Top DVI_TX_Top_inst0
    (
        .I_rst_n       (hdmi4_rst_n   ),  //asynchronous reset, low active
        .I_serial_clk  (serial_clk    ),

        //CMOS
        .I_rgb_clk     (dvi0_rgb_clk),  //pixel clock
        .I_rgb_vs (hdmi_vs_q),
        .I_rgb_hs (hdmi_hs_q),
        .I_rgb_de (hdmi_de_q),
        .I_rgb_r (hdmi_rgb_q[23:16]),
        .I_rgb_g (hdmi_rgb_q[15:8]),
        .I_rgb_b (hdmi_rgb_q[7:0]),

        .O_tmds_clk_p  (tmds_clk_p_0  ),
        .O_tmds_clk_n  (tmds_clk_n_0  ),
        .O_tmds_data_p (tmds_d_p_0    ),  //{r,g,b}
        .O_tmds_data_n (tmds_d_n_0    )
    );

    end endgenerate
endmodule
