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
    input cache_req_type data_req,
    input cache_data_type data_write,
    input [3:0] be,
    input [1:0] block_offset,
    input from_ram,
    output cache_data_type data_read);
    
    cache_data_type data_mem[0:255];
    
    initial begin
        for(int i=0; i<256; i++)
            data_mem[i]='0;
    end

    always_ff @(posedge clk) begin
        if(data_req.we) begin
            if(from_ram) 
                data_mem[data_req.index] <= data_write;
            if(!from_ram) begin
              for (int b = 0; b < WORD_SIZE; b++) begin
                if (be[b]) begin
                    data_mem[data_req.index][block_offset*WORD_WIDTH+b*8+:8] <= data_write[block_offset*WORD_WIDTH+b*8+:8];  //[b*8+:8];
                end
              end
            end
        end
            
        data_read <= data_mem[data_req.index];
    end
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req,
    input cache_tag_type tag_write,
    output cache_tag_type tag_read);
    
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
    logic [3:0] be;
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
    //Got idea to use intermediate signals from classmates Patrick and Nolan
    logic [31:0] read_data_hold;
    logic data_req_we_hold;
    logic write_resp_valid_hold;
    logic read_data_valid_hold;

    assign cpu_addr = (cpu.write_addr_valid) ? cpu.write_addr : cpu.read_addr;
    assign be = cpu.strobe;
    assign block_offset = cpu_addr[3:2];
    assign cpu_index = cpu_addr[11:4];
    assign cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
    assign tag_req.index  = cpu_addr[11:4];
    assign data_req.index = cpu_addr[11:4];
    assign hit = ((cpu_tag == tag_read.tag) && tag_read.valid);
    
    //suggestion from student Dino
	assign mem.write_data = data_read;
	
	//FSM for Cache Controller
	always_comb 
	begin
        case(state)
        
            compare_tag: begin
                tag_req.we = 1;
                data_req_we_hold = 0;
                cpu.read_addr_ready = 1;
                cpu.write_addr_ready = 1;
                read_data_valid_hold = 0;
                write_resp_valid_hold = 0;
                
                //successful write
                if(cpu.write_addr_valid && hit)
                begin
                    //store the data on the clock edge (32 bits for the data)
                    data_write[block_offset*32+:32] = cpu.write_data;
                    //just a bit to acknowldege the write and set the write address valid bit low
                    write_resp_valid_hold = 1;
                    //enable writing to tag/data cache
                    tag_req.we = 1;
                    data_req_we_hold = 1;
                    //make this entry dirty so it can be written back later
                    tag_write.dirty = 1;
                    //indicate that the data is from the cache
                    from_ram = 0;
                    write_resp_valid_hold = 1;
                    next_state = compare_tag;
                end
                
                //seuccessful read
                if(cpu.read_addr_valid && hit)
                begin
                    //disable writing because we are reading
                    tag_req.we = 0;
                    data_req_we_hold = 0;
                    //read the data from the cache into the cpu (32 bits)
                    read_data_hold = data_read[block_offset*32+:32];
                    //set the read data valid bit high
                    read_data_valid_hold = 1;
                    
                    next_state = compare_tag;
                end
                
                //checks if the read/write address is valid in the cache and if there is not a hit and the data is dirty
                //this means that the data there needs to be updated in the memory
                if((cpu.read_addr_valid || cpu.write_addr_valid ) && !hit && tag_read.dirty)
                begin
                    //want to keep the tag the same since we are writing to the same tag in memory
                    tag_req.we = 0;
                    //write the dirty cache data to memory since it will be overwritten
                    mem.write_data = data_read[block_offset*32+:32];
                    
                    next_state = writeback;
                end
                
                //checks if the read/write is valid and it is not in the cache and the data isn't dirty
                //it means that the data was not in the cache and needs to be retrieved from main memory
                if((cpu.read_addr_valid || cpu.write_addr_valid) && !hit && !tag_read.dirty)
                begin
                    next_state = allocate;
                end
                
            end
            
            allocate: begin
                //keeps tag the same and allows to system to keep reading from memory
                tag_req.we = 0;
                data_req_we_hold = 1;
                mem.write_addr_valid = 0;
                cpu.read_addr_ready = 0;
                cpu.write_addr_ready = 0;
                cpu.read_data_valid = 0;
                //put the address for the memory to read from
                mem.read_addr = {cpu_tag, cpu_index};
                
                //will keep polling the memory until it gets the data requested
                if(!mem.read_data_valid)
                begin
                    mem.read_addr_valid = 1;
                    
                    next_state = allocate;
                end
               
               //once the data has been allocated it stops it from polling and move to save in cache
               if(mem.read_data_valid)
               begin
                    mem.read_addr_valid = 0;
                    
                    next_state = save_in_cache;
               end
               
            end
            
            save_in_cache: begin
                //makes sure that no reading or writing is happening while saving in the cache
                mem.read_addr_valid = 0;
                mem.write_addr_valid = 0;
                cpu.read_addr_ready = 0;
                cpu.write_addr_ready  = 0;
                cpu.read_data_valid = 0;
                mem.read_addr = {cpu_tag, cpu_index};
                
                //write data from ram to cache
                from_ram = 1;
                data_req_we_hold = 0;
                tag_req.we = 1;
                //get a full cache line (128 bits) and put it into cache
                data_write = mem.read_data;
                //write tag data from memory to cache
                tag_write.tag = cpu_tag;
                //read from memory so cache entry is valid and clean
                tag_write.valid = 1;
                tag_write.dirty = 0;
                
                next_state = compare_tag;
            end
            
            //will write the cache data to memory
            writeback: begin
                //make sure only a mem write will happen
                tag_req.we = 0;
                mem.read_addr_valid = 0;
                mem.write_addr_valid = 1;
                cpu.read_addr_ready = 0;
                cpu.write_addr_ready = 0;
                cpu.read_data_valid = 0;
                //write to the address we want to write to in the cache
                mem.write_addr = {tag_read.tag, tag_req.index};
                
                //will keep polling until the memory address is ready to be written to
                if(!mem.write_addr_ready)
                begin
                    next_state = writeback;
                end
                
                //the memory data was updated so it is no longer dirty (old data)
                if(mem.write_addr_ready)
                begin
                    tag_write.dirty = 0;
                    
                    next_state = allocate;
                end
            end
        endcase
	end
	
	always_ff @(posedge clk)
	begin
	   if(RESET)
	   begin
	       state <= compare_tag;
	   end
	   //write intermidiate values to the corresponds values on the clock edge
	   else
	       //avoid multi driven errors and update valid bit for comparetag with the right timing
	       if(state == compare_tag)
	           cpu.read_data_valid <= read_data_valid_hold;
	       //update data and control signals with correct timing
	       cpu.read_data <= read_data_hold;
	       data_req.we <= data_req_we_hold;
	       cpu.write_resp_valid <= write_resp_valid_hold;
	       state <= next_state;
	end
    
    L1_cache_tag L1_tags(.*);
    L1_cache_data L1_data(.*);

endmodule
