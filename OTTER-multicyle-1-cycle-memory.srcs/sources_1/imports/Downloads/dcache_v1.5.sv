`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
//           Joseph Callenes
//           
// 
// Create Date: 02/06/2020 06:40:37 PM
// Design Name: 
// Module Name: dcache
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

package cache_def;

parameter int TAG_MSB = 31;
parameter int TAG_LSB = 12;

typedef struct packed{
    logic valid;
    logic dirty;
    logic [TAG_MSB:TAG_LSB] tag;
}cache_tag_type;

typedef struct {
    logic [7:0] index;
    logic we;
}cache_req_type;

//128-bit cache line
typedef logic [127:0] cache_data_type;

//CPU request (CPU ->cache controller)
typedef struct{
    logic [31:0] addr;
    logic [31:0] data;
    logic rw;
    logic valid;
}cpu_req_type;

//Cache result (cache controller -> CPU)
typedef struct {
    logic [31:0]data;
    logic ready;
}cpu_result_type;

//memory request (cache controller -> memory)
typedef struct {
    logic [31:0]addr;
    logic [127:0]data;
    logic rw;
    logic valid;
}mem_req_type;

//memory controller response (memory -> cache controller)
typedef struct {
cache_data_type data;
logic ready;
}mem_data_type;

endpackage

import cache_def::*;
import memory_bus_sizes::*; 

module L1_cache_data ( 
    input clk,
    input cache_req_type data_req, //request info: index, write enable
    input cache_data_type data_write, //128 bit dataline
    input [1:0] be,
    input [1:0] block_offset,
    input from_ram,
    output cache_data_type data_read); //128 bit port that has data read
    
    cache_data_type data_mem[0:255]; //set up memory
    
    initial begin //initalize memory to 0
        for(int i=0; i<256; i++)
            data_mem[i]='0;
    end

    always_ff @(posedge clk) begin
        if(data_req.we) begin //check if write enable for request is high
            if(from_ram) //check if data is coming from ram
                data_mem[data_req.index] <= data_write; //write data to specified index
            if(!from_ram) begin
              for (int b = 0; b < WORD_SIZE; b++) begin
                if (be[b]) begin
                    data_mem[data_req.index][block_offset*WORD_WIDTH+b*8+:8] <= data_write[block_offset*WORD_WIDTH+b*8+:8];  //[b*8+:8];
                end
              end
            end
        end
            
        data_read <= data_mem[data_req.index]; //read back the data at specified index
    end
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req, //tag request, index/write enable
    input cache_tag_type tag_write, //write port:valid,dirty,tag
    output cache_tag_type tag_read); //read port:valid,dirty,tag
    
	// tag storage 
	cache_tag_type tag_storage[0:255];
	
	//initialize storage to zeros
	initial
	begin
	   for(int i = 0; i<256; i++)
	       tag_storage[i] = 0;
	end
	
	// incluces valid and dirty bits
	// async read, sync write
	
	//asynch read
	assign tag_read = tag_storage[tag_req.index];
	
	//sync write
	always_ff @(posedge(clk))
	begin
	   //check if write enable of request is enabled
	   if(tag_req.we == 1)
	       //write tag info at index
	       tag_storage[tag_req.index] <= tag_write;
	end
	
	
endmodule


module dcache(
    input clk, RESET,
    axi_bus_rw.device cpu,
    axi_bus_rw.controller mem
    );

    cpu_req_type cpu_req;     //CPU->cache
    mem_data_type mem_data;   //memory->cache
    
    mem_req_type mem_req;    //cache->memory
    cpu_result_type cpu_res;  //cache->CPU
    
    logic [1:0] block_offset;
    logic [1:0] be;
    logic from_ram;
    logic wait_read, next_wait_read;   
    
    
    typedef enum {save_in_cache, compare_tag, allocate, writeback} cache_state_type;
   
    cache_state_type state, next_state;

    cache_tag_type tag_read;
    cache_tag_type tag_write;
    cache_req_type tag_req;
    
    cache_data_type data_read;
    cache_data_type data_write;
    cache_req_type data_req;
    
    cpu_result_type next_cpu_res;
    
    logic [31:0] cpu_addr;
    logic [7:0] cpu_index;
    logic [19:0] cpu_tag;
    
	//0-1 = byte offset
	//2-3 = block offset
	//4-11 = index
	//12-31 = tag
    //get the address
    assign be = cpu_addr[1:0];
    assign block_offset = cpu_addr[3:2];
    assign cpu_addr = (cpu.write_addr_valid) ? cpu.write_addr : cpu.read_addr;
    assign cpu_index = cpu_addr[11:4]; //Q: is this the right bits?
    assign cpu_tag = cpu_addr[31:12];
    assign tag_req.index = cpu_index;
    //checks if the tag requested by the cpu address matches the tag in the tag cache and checks if it is valid
    assign hit = ((cpu_tag == tag_read.tag) && tag_read.valid);
    //TODO: something wrong with the hit. seem the valid bit is the problem
	
	
	//cache axi controller FSM
	always_comb 
    begin 
        case(state)
          save_in_cache: begin  
            mem.read_addr_valid = 0;
            mem.write_addr_valid = 0;
            cpu.read_addr_ready = 0;
            cpu.write_addr = 0;
            cpu.read_addr_valid = 0;
            mem.read_addr = {cpu_tag ,cpu_index}; //question: what is i? should this be {tag_read.tag, tag_req.index or data_req.index?}  A:
            
            tag_write.tag = cpu_tag; //this is input to tag cache. should dupdate on clock edge
            tag_write.valid = 1;
            tag_write.dirty = 0;
            tag_req.we = 1;
            data_req.index = cpu_index;
            data_req.we = 1;
            from_ram = 1;
            data_write = mem.read_data; //input to data cache, should write to memory on clock edge
            next_state = compare_tag;
          end



          compare_tag: begin
            //checks if there is a tag hit and read bit is valid
                cpu.read_addr_ready = 1;
                cpu.write_addr_ready = 1;
                tag_req.we = 0;
                
                //successful write
                if(cpu.write_addr_valid && hit)
                begin
                    data_req.index = cpu_index; //update index so the correct index is written to on the clock edge
                end
                
                
                //successful read
                if(cpu.read_addr_valid  && hit)
                begin
                    data_req.index = cpu_index; //update index so the correct index is written to on the clock edge
                end
                
                
                if((cpu.read_addr_valid || cpu.write_addr_valid) && !(hit) && !tag_read.dirty) //Question: tag hit? A:
                begin
                    next_state = allocate;
                end
                

                if((cpu.read_addr_valid &&!(hit) && tag_read.dirty)
                 || cpu.write_addr_valid && !(hit) && tag_write.dirty) //tag_read.tag? for hit. tag_read.dirty?
                begin
                    next_state = writeback;
                end   
          end
          
          allocate: begin
            //check if the memory is ready to be written
            mem.write_addr_valid = 0;
            cpu.read_addr_ready = 0;
            cpu.write_addr_ready = 0;
            cpu.read_data_valid = 0;
            mem.read_addr = {cpu_tag ,cpu_index}; 

            
            if(!mem.read_data_valid)
            begin
                //Question: should this be in ff block? A: 
                mem.read_addr_valid = 1;
                next_state = allocate;
            end
            
            if(mem.read_data_valid)
            begin 
                mem.read_addr_valid = 0;
                next_state = save_in_cache;
            end
          end

          writeback: begin
            mem.read_addr_valid = 0;
            mem.write_addr_valid = 1;
            cpu.read_addr_ready = 0;
            cpu.write_addr_ready = 0;
            cpu.read_data_valid = 0;
            mem.write_addr  = {tag_write.tag, cpu_index}; 
 
            if(!mem.write_addr_ready)
            begin
                next_state = writeback;
            end
            
            if(mem.write_addr_ready)
            begin
                next_state = allocate;
            end
            
          end
        endcase     
    end
	
	
	//ff block to change states and update values
	always_ff @(posedge(clk))
	begin
	   
	   /*---------------------Save IN CACHE-----------------------------------------------------------*/
	   if((state == save_in_cache) && (next_state == compare_tag))
	   begin

             
            tag_write.valid <= 1; //tag_read[tag_req.index].valid?
            tag_write.dirty <= 0; //tag_read[tag_req.index].dirty?
	   end
	   
	   
	   /*---------------------COMPARE TAG-----------------------------------------------------------*/
	   if(state == compare_tag)
	   begin 
	        from_ram <= 0;
            cpu.read_data_valid <= 0;
            cpu.write_resp_valid <= 0;
	   end
	   
	   //-----------------------------------successful read----------------------------------------------------
	   if(cpu.read_addr_valid  && hit) 
        begin
            //data_req.index = cpu_req.index; //update index with the one requested from cpu
            cpu.read_data <= data_read;//read data from mem_cache at cpu index    //data_read? data_read.data[] what in the box? index?
            //cpu.read_data <= data_read.data[strobe]? Q
            cpu.read_data_valid <= 1;
            //next state <= compare_tag;
        end
	   
	   //----------------------------------successful write------------------------------------------------------------
	   if(cpu.write_addr_valid && hit)//look at diagram for the hit logic
        begin


            data_write <= cpu.write_data; //what am I doing, it's 11:46pm and I'm tired

            //set the tag to dirty
            tag_write.dirty <= 1;
            cpu.write_resp_valid <= 1; //?
            //next state <= compare_tag;
        end
	   /*-----------------------------------------------------------------------------------------------------*/
	   
	   //-------------------------------------------WRITEBACK-----------------------------------------------------
	   //compare tag to writeback transition
	   if((state == compare_tag) && (next_state == writeback))
	   begin
	       //data_req.index <= mem_req.index;
	       mem.write_data <= data_write; //Question: should this be mem_data, data_read, or data_write?
	   end
	   
	   if((state == writeback) && (next_state == allocate))
	   begin
	   //Question: what am I setting to dirty? write, read, request?     
            tag_write.dirty <= 0; //tag_req.dirty Q
       end
	   
	   
	   
	   //------------------------------------------------------------------------------------------------------------
	   
	   /*-------------------UPDATE STATES---------------------------------------------------------------------*/
	   if(RESET)
	   begin
	       state <= compare_tag;
	   end
	   else
	       state <= next_state;
	end
	
	
	

    L1_cache_tag L1_tags(.*);
    L1_cache_data L1_data(.*);

endmodule
