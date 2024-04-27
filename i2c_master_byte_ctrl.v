`include "../misc/timescale.v"
`include "../misc/i2c_master_defines.v" 

module i2c_master_byte_ctrl (
    input             Clk,        // system clock
    input             Rst_n,      // asynchronous active low reset
    
	input             Start,      // Generar condició Start. CR bit 7
    input             Stop,       // Generar la condició de Stop. CR bit 6
    input             Read,       // Generar tranferència Lectura. CR bit 5
    input             Write,      // Generar transferència de escriptura. CR bit 4

    input             Tx_ack,     // Data transmessa. Registre envia ack bit (Txack='0') or NACK (Txack ='1') (CR bit 3).
    output reg        Rx_ack,     // Data rebuda. Registre rep ack bit (SR bit 7)
    output reg        I2C_done,   // command completed, used to clear command register.
    input      		  I2C_al,     // I2C bus arbitration lost. //Input

    input             SR_sout,    // Shift register serial Out. Longitud 1 bits 
    output reg        SR_load,    // Carreguem la dada a desplaçar
    output reg        SR_shift,   // dona la ordre de desplaçar //shift i la dada van a la vegada

    //Revisar. No sabemos si son bidireccionales o que son.
	output reg [3:0]  Bit_cmd,    // Command (from byte controller). Output
    output reg        Bit_txd,    // data to transmit. Output
    input      		  Bit_ack,    // command complete acknowledge. input
    input       	  Bit_rxd    // input
);

	//Signals for state machine
    reg [2:0] count;            //contador de 3 bits
    reg       count_finish;     //Variable per controlar si el contador ha finalitzat.

	// state machine
    reg [3:0] 	state, next; // state machine variable

	// generate counter
	always @(posedge Clk or negedge Rst_n)
		if (!Rst_n) begin
			count <= 3'h0;
		end
		else if (SR_load) begin
			count <= 3'h7;
		end
		else if (SR_shift) begin
		count <= count - 3'h1;
		end else begin
			count <= count;
		end
    
    always @(*) begin
		if (count == 3'h0)
			count_finish = 1'b1;
		else
			count_finish = 1'b0; 
	end

	// state decoder
	localparam IDLE = 4'd0,
			   START = 4'd1,
			   STOP = 4'd2,
			   WRITE = 4'd3,
			   READ = 4'd4,
			   ACK = 4'd5;

	always @(posedge Clk or negedge Rst_n) //no se toca
    if(!Rst_n) state <= IDLE; //Tornem al estat base
    else       state <= next; //avançem de estat

	always @(*) //definicion cambios de estado byte
    case(state)
      IDLE : if(Start) next <= START;
	   		 else if (Read) next <= READ; //preferència a la lectura davant l'escriptura
             else if (Write) next <= WRITE;
			 else if (Stop) next <= STOP;
			 else next <= IDLE;

	  START : if (Bit_ack) begin
				if(Read) next <= READ;
				else if (Write) next <= WRITE;
				else next <= START;
	  		  end
			 else next <= START;
	  			
	  
	  READ : if (Bit_ack) begin
				if (count_finish) next  <= ACK; //ja hem acabat de llegir
				else next  <= READ; //seguim llegint	
			 end
			 else next <= READ;

	  WRITE : if (Bit_ack) begin
				if (count_finish) next  <= ACK; //ja hem acabat de escriure	
			 	else next  <= WRITE;
			 end
			 else next <= WRITE;
	  
	  ACK : if (Bit_ack) begin
				if (Stop) next <= STOP;
				else next <= IDLE; //hem acabat, tornem al Estat inicial
			end
			else next  <= ACK;

	  STOP : if(Bit_ack) next <= IDLE; 
             else next <= STOP;

	endcase

	always @(posedge Clk or negedge Rst_n) begin //definicion cambios estado bit
		if (!Rst_n) begin //definicion de señales a resetear
			Bit_cmd <= `I2C_CMD_NOP; //Estat IDLE
	        Bit_txd <= 1'b0; //Transmetem un 0
			SR_shift <= 1'b0; //Desactivem desplaçament
	        SR_load  <= 1'b0; //Valor a carregar 0
			I2C_done <= 1'b0; 
			//I2C_al <= 1'b0;
			Rx_ack <= 1'b0;
		end

		else begin
			SR_load <= 1'b0;
			SR_shift <= 1'b0;
			Rx_ack <= Rx_ack;
			Bit_cmd <= Bit_cmd;
			I2C_done <= 1'b0; 
			Bit_txd <= SR_sout;
			//I2C_al <= 1'b0;

		case(state)
			IDLE :  begin
					SR_load <= 1'b1;
					if (Start) Bit_cmd <= `I2C_CMD_START;
					else if (Read) Bit_cmd <= `I2C_CMD_READ; //preferència a la lectura davant l'escriptura
					else if (Write) Bit_cmd <= `I2C_CMD_WRITE;
					else if (Stop) Bit_cmd <= `I2C_CMD_STOP;
					else Bit_cmd <= `I2C_CMD_NOP;
			end

			START : begin 
					SR_load <= 1'b1;
					if(Bit_ack) begin
						if (Read) Bit_cmd <= `I2C_CMD_READ; //llegim
						else if (Write) Bit_cmd <= `I2C_CMD_WRITE; 	
					end
			end

			READ :  if (Bit_ack) begin
						Bit_txd <= Tx_ack; //profe
						SR_shift <= 1'b1;
						if(count_finish) begin 
							Bit_cmd <= `I2C_CMD_WRITE;
						end
						else begin
							//SR_shift <= 1'b1; //Introduim nou valor
							Bit_cmd <= `I2C_CMD_READ; // llegim el següent bit
						end
					end
					else begin 
						SR_shift <= 1'b0; 
					end

			WRITE : if (Bit_ack) begin
						SR_shift <= 1'b1;
						Bit_txd <= SR_sout;
						if(count_finish) begin 
							Bit_cmd <= `I2C_CMD_READ; 
							Rx_ack <= (!Tx_ack); // slave should have send NACK
						end
						else begin
							Bit_cmd <= `I2C_CMD_WRITE; // llegim el següent bit
						end
					end 
					else SR_shift <= 1'b0;
					
			ACK	: 	if (Bit_ack) begin
					I2C_done <= 1'b1; 
						if (Stop) Bit_cmd <= `I2C_CMD_STOP;
						else Bit_cmd <= `I2C_CMD_NOP;
					end
					
			STOP : 	
					if (Stop) begin
						Bit_cmd <= `I2C_CMD_NOP; 
						I2C_done <= 1'b1; //añadido
						//I2C_al <= 1'b1; //añadido
						end

		endcase

		end
		
	end



endmodule