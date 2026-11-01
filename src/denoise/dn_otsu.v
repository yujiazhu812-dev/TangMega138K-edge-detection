// Global 64-bin Otsu for <=1280x720, one shared multiplier.
// N*S uses 46 bits (44-bit intermediates are insufficient at 720p).
// Collect a complete frame, stop, calculate, clear, then collect another frame.
// Threshold banks publish only at VS rising edge. No current-frame retroactivity.
module dn_otsu #(parameter W=1280,H=720)(
 input clk,rst_n,restart,vs,ce,good,input[7:0] magnitude,
 input[3:0] query_tile, input[7:0] fallback,
 output[7:0] global_threshold,local_threshold,output reg ready,
 output reg[31:0] updates
);
 localparam CLEAR=0,WAIT_FRAME=1,COLLECT=2,DRAIN=3,READ_BIN=4,READ_WAIT=5,
 PREFIX=6,MUL_START=7,MUL_WAIT=8,NEXT_BIN=9,PUBLISH=10;
 reg[3:0] state;reg vs_d,pending_bank;wire frame=vs&&!vs_d;
 reg[10:0] clear_addr;
 wire sample=state==COLLECT&&ce&&good&&!vs&&!restart;
 wire[5:0] bin=magnitude[7:2];
 reg[19:0] count[0:0];reg[25:0] sum[0:0];
 reg[7:0] active[0:0],staged[0:0];
 reg[4:0] region;reg[5:0] scan;
 wire[19:0] hg,hl;
 dn_histogram #(.AW(6),.DEPTH(64)) u_global(clk,rst_n,sample,bin,
 state==CLEAR&&clear_addr<64,clear_addr[5:0],state==READ_BIN,scan,hg);
 assign hl=20'b0;
 reg[19:0] n,w0;reg[25:0] total,s0;
 reg[45:0] ns;reg[43:0] delta,den,best_den;reg[87:0] numerator,best_num;
 reg[131:0] lhs;reg[5:0] best_bin;reg have_best;reg take_candidate;
 reg[2:0] operation;reg start;reg[87:0] ma;reg[43:0] mb;
 wire busy,done;wire[131:0] product;
 dn_serial_mul u_mul(clk,rst_n,start,ma,mb,busy,done,product);
 // 46-bit magnitude difference before the documented 44-bit narrowing.
 wire[45:0] delta_diff=(ns>=product[45:0])?ns-product[45:0]:product[45:0]-ns;
 wire[19:0] h=(region==0)?hg:hl;
 // Six shifted addends replace bin * count.
 wire[25:0] hc={6'b0,h};
 wire[25:0] weighted01=(scan[0]?hc:26'd0)+(scan[1]?(hc<<1):26'd0);
 wire[25:0] weighted23=(scan[2]?(hc<<2):26'd0)+(scan[3]?(hc<<3):26'd0);
 wire[25:0] weighted45=(scan[4]?(hc<<4):26'd0)+(scan[5]?(hc<<5):26'd0);
 wire[25:0] weighted=(weighted01+weighted23)+weighted45;
 // READ_WAIT already exists between the synchronous RAM read and PREFIX.
 // Use that cycle to register bin*count; state/threshold publication timing stays fixed.
 reg[25:0] weighted_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)weighted_q<=0;
  else if(restart)weighted_q<=0;
  else if(state==READ_WAIT)weighted_q<=weighted;
 end
 assign global_threshold=ready?active[0]:fallback;
 assign local_threshold=global_threshold;
 // Unsigned 132-bit comparison with parallel 12-bit leaves and balanced merges.
 // Preserve exact strict-greater behavior and the existing calculation latency.
 function greater132;
  input[131:0] left_value,right_value;
  reg[15:0] leaf_gt,leaf_eq;
  reg[7:0] pair_gt,pair_eq;
  reg[3:0] quad_gt,quad_eq;
  reg[1:0] half_gt,half_eq;
  integer j;
  begin
   leaf_gt=16'b0;leaf_eq=16'hffff;
   for(j=0;j<11;j=j+1)begin
    leaf_gt[j]=left_value[j*12+:12]>right_value[j*12+:12];
    leaf_eq[j]=left_value[j*12+:12]==right_value[j*12+:12];
   end
   for(j=0;j<8;j=j+1)begin
    pair_gt[j]=leaf_gt[j*2+1]||(leaf_eq[j*2+1]&&leaf_gt[j*2]);
    pair_eq[j]=leaf_eq[j*2+1]&&leaf_eq[j*2];
   end
   for(j=0;j<4;j=j+1)begin
    quad_gt[j]=pair_gt[j*2+1]||(pair_eq[j*2+1]&&pair_gt[j*2]);
    quad_eq[j]=pair_eq[j*2+1]&&pair_eq[j*2];
   end
   for(j=0;j<2;j=j+1)begin
    half_gt[j]=quad_gt[j*2+1]||(quad_eq[j*2+1]&&quad_gt[j*2]);
    half_eq[j]=quad_eq[j*2+1]&&quad_eq[j*2];
   end
   greater132=half_gt[1]||(half_eq[1]&&half_gt[0]);
  end
 endfunction
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   state<=CLEAR;clear_addr<=0;vs_d<=0;pending_bank<=0;ready<=0;updates<=0;
   region<=0;scan<=0;n<=0;w0<=0;total<=0;s0<=0;ns<=0;delta<=0;
   den<=0;best_den<=0;numerator<=0;best_num<=0;lhs<=0;best_bin<=0;
   have_best<=0;take_candidate<=0;operation<=0;start<=0;ma<=0;mb<=0;
   for(i=0;i<1;i=i+1)begin count[i]<=0;sum[i]<=0;active[i]<=32;staged[i]<=32;end
  end else if(restart)begin
   state<=CLEAR;clear_addr<=0;vs_d<=0;pending_bank<=0;ready<=0;updates<=0;
   region<=0;scan<=0;n<=0;w0<=0;total<=0;s0<=0;ns<=0;delta<=0;
   den<=0;best_den<=0;numerator<=0;best_num<=0;lhs<=0;best_bin<=0;
   have_best<=0;take_candidate<=0;operation<=0;start<=0;ma<=0;mb<=0;
   for(i=0;i<1;i=i+1)begin count[i]<=0;sum[i]<=0;active[i]<=32;staged[i]<=32;end
  end else begin
   vs_d<=vs;start<=0;take_candidate<=0;
   case(state)
    CLEAR:if(clear_addr==63)begin clear_addr<=0;state<=pending_bank?PUBLISH:WAIT_FRAME;end else clear_addr<=clear_addr+1'b1;
    WAIT_FRAME:if(frame)begin
     for(i=0;i<1;i=i+1)begin count[i]<=0;sum[i]<=0;end
     state<=COLLECT;
    end
    COLLECT:begin
     if(sample)begin
      count[0]<=count[0]+1'b1;sum[0]<=sum[0]+bin;
     end
     if(frame)state<=DRAIN;
    end
    DRAIN:begin region<=0;scan<=0;w0<=0;s0<=0;n<=count[0];total<=sum[0];
     have_best<=0;best_num<=0;best_den<=0;best_bin<=0;state<=READ_BIN;end
    READ_BIN:state<=READ_WAIT;
    READ_WAIT:state<=PREFIX;
    PREFIX:begin w0<=w0+h;s0<=s0+weighted_q;operation<=0;state<=MUL_START;end
    MUL_START:begin
     if(w0==0||w0==n)state<=NEXT_BIN;
     else begin
      start<=1;state<=MUL_WAIT;
      case(operation)
       0:begin ma<={68'b0,n};mb<={18'b0,s0};end
       1:begin ma<={62'b0,total};mb<={24'b0,w0};end
       2:begin ma<={44'b0,delta};mb<=delta;end
       3:begin ma<={68'b0,w0};mb<={24'b0,(n-w0)};end
       4:begin ma<=numerator;mb<=best_den;end
       5:begin ma<=best_num;mb<=den;end
      endcase
     end
    end
    MUL_WAIT:if(done)begin
     state<=MUL_START;operation<=operation+1'b1;
     case(operation)
      0:ns<=product[45:0];
      1:delta<=delta_diff[43:0];
      2:numerator<=product[87:0];
      3:den<=product[43:0];
      4:lhs<=product;
      5:begin
       // Register the comparison before it drives the wide best-score enables.
       take_candidate<=!have_best||greater132(lhs,product);
       state<=NEXT_BIN;
      end
     endcase
    end
    NEXT_BIN:begin
     if(take_candidate)begin have_best<=1;best_num<=numerator;best_den<=den;best_bin<=scan;end
     if(scan==63)begin
     staged[region]<=(take_candidate||have_best)?{(take_candidate?scan:best_bin),2'b11}:fallback;
     if(region==0)begin pending_bank<=1;clear_addr<=0;state<=CLEAR;end

    end else begin scan<=scan+1'b1;state<=READ_BIN;end
    end
    PUBLISH:if(frame)begin
     for(i=0;i<1;i=i+1)active[i]<=staged[i];ready<=1;updates<=updates+1'b1;state<=COLLECT;pending_bank<=0;
     for(i=0;i<1;i=i+1)begin count[i]<=0;sum[i]<=0;end
    end
    default:state<=CLEAR;
   endcase
  end
 end
endmodule
