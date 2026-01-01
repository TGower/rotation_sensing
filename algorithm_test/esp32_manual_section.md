Processor Instruction Extensions (PIE)
1.1 Overview
The ESP32-S3 adds a series of extended instruction set in order to improve the operation efficiency of
specific AI and DSP (Digital Signal Processing) algorithms. This instruction set is designed from the TIE
(Tensilica Instruction Extension) language, and adds general-purpose registers with large bit width, various
special registers and processor ports. Based on the SIMD (Single Instruction Multiple Data) concept, this
instruction set supports 8-bit, 16-bit, and 32-bit vector operations, which greatly increases data operation
efficiency. In addition, the arithmetic instructions, such as multiplication, shifting, and accumulation, can
perform data operations and transfer data at the same time, thus further increasing execution efficiency of a
single instruction.
1.2 Features
The PIE (Processor Instruction Extensions) has the following features:
• 128-bit general-purpose registers
• 128-bit vector operations, e.g., multiplication, addition, subtraction, accumulation, shifting, comparison,
etc.
• Integration of data transfer into arithmetic instructions
• Support for non-aligned 128-bit vector data
• Support for saturation operation
1.3 Structure Overview
A structure overview should help to understand list of available instructions, instructions possibilities, and
limits. It is not intended to describe hardware details.
The internal structure of PIE for multiplication-accumulation (MAC) instructions overview could be described
as shown below:
Espressif Systems 39
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Figure 1.3-1. PIE Internal Structure (MAC)
The diagram above shows the data flow paths and PIE components.
The PIE unit contains:
• Address unit that reads 8/16/32/64/128-bit aligned data
• Bank of eight 128-bit vector QR registers
• Arithmetic logic unit (ALU) with
– sixteen 8-bit multipliers
– eight 16-bit multipliers
• QACC_H/QACC_L - 2 160-bit accumulators
• ACCX - 40-bit accumulator
1.3.1 Bank of Vector Registers
Bank of vector registers contains 8 vector registers (QR). Each register could be represented as an array of 16 x
8-bit data words, array of 8 x 16-bit data words, or array of 4 x 32-bit data words. Depending on the used
instructions, 8, 16 or 32-bit data format will be chosen.
Espressif Systems 40
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
1.3.2 ALU
Arithmetic logic unit (ALU) could work for 8-bit input data, as 8-bit ALU, for 16-bit input data, as 16-bit ALU, or
for 32-bit input data, as 32-bit ALU. 8-bit multiplication ALU contains 16 multipliers and is able to make up to 16
multiplications and accumulation in one instruction. With multiplication almost any other combinations of
arithmetic operation are possible. For example, FFT instructions include multiplication, addition, and
subtraction operations in one instruction. Also, ALU includes logic operations like AND, OR, shift, and so
on.
The input for ALU operation comes from QR registers. The result of operations could be saved to the QR
registers or special accumulator registers (ACCX, QACC).
1.3.3 QACC Accumulator Register
The QACC accumulator register is used for multiplication-accumulation operations on 8-bit or 16-bit data. In
the case of 8-bit data, QACC consists of 16 accumulator registers with 20-bit width. In the case of 16-bit data,
QACC consists of 8 accumulator registers with 40-bit width. The following description reflects the case of 8-bit
arithmetic. For 16-bit arithmetic, the logic is similar.
After multiplication and accumulation on two vector QR registers, the result of 16 operations will be written to
16 20-bit accumulator registers.
QACC is divided into two parts: 160-bit QACC_H and 160-bit QACC_L. The former stores the higher 160-bit
data of QACC, and the latter stores the lower 160-bit data. To store the accumulator result in QR registers, it is
possible to convert 20-bit result numbers to 8 bits by right-shifting it. For 16-bit multiplication-accumulation
operation, convert the 40-bit result to 16-bit by right-shifting it.
It is possible to load data from memory to QACC or reset the initial value to 0.
1.3.4 ACCX Accumulator Register
Some operations require accumulating the result of all multipliers to one value. In this case, the ACCX
accumulator should be used.
ACCX is a 40-bit accumulator register. The result of the accumulators could be shifted and stored in the
memory as an 8-bit or 16-bit value.
It is possible to load data from memory to ACCX or reset the initial value to 0.
1.3.5 Address Unit
Most of the instructions in PIE allow loading or storing data from/to 128-bit Q registers in parallel in one cycle.
In most cases, the data should be 128-bit aligned, and the lower 4 bits of address will be ignored. The Address
unit provides functionality to manipulate address registers in parallel, which saves the time to update address
registers.
It is possible to make address register operations like AR + signed constant, ARx + ARy, and AR + 16.
The Address unit makes post-processing operations. It means that all operations with address registers are
done after instructions are finished.
Espressif Systems 41
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
1.4 Syntax Description
This section provides introduction to the encoding order of instructions and the meaning of characters that
appear in the instruction descriptions.
1.4.1 Bit/Byte Order
The encoding order of instructions is divided into two types based on the granularity, i.e., bit order and byte
order. According to the located side of the least bit or byte, there are big-endian order and little-endian order.
That is to say, the most common encoding types for instructions are: little-endian bit order, big-endian bit
order, little-endian byte order and big-endian byte order.
• Little-endian bit order: the instruction is encoded in bit order, with the least significant bit on the right.
• Big-endian bit order: the instruction is encoded in bit order, with the least significant bit on the left.
• Little-endian byte order: the instruction is encoded in byte order, with the least significant byte on the
right.
• Big-endian byte order: the instruction is encoded in byte order, with the least significant byte on the left.
Among them, the instruction encoding bit sequences obtained using little-endian byte order and little-endian
bit order are identical. Taking the 24-bit instruction EE.ZERO.QACC as an example, Figure 1.4-1, Figure 1.4-2,
Figure 1.4-3 and Figure 1.4-4 show the code of this instruction in little-endian bit order, big-endian bit order,
little-endian byte order and big-endian byte order, respectively.
Please note that all instructions and register descriptions appear in this chapter use little-endian bit order,
which means the least significant bit is stored in the lowest addresses.
Figure 1.4-1. EE.ZERO.QACC in Little-Endian Bit Order
Figure 1.4-2. EE.ZERO.QACC in Big-Endian Bit Order
Espressif Systems 42
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Figure 1.4-3. EE.ZERO.QACC in Little-Endian Byte Order
Figure 1.4-4. EE.ZERO.QACC in Big-Enidan Byte Order
1.4.2 Instruction Field Definition
Table 1.4-1 provides the meaning of the characters covered in instruction descriptions. You can find such
characters and corresponding descriptions in Section 1.8.
Espressif Systems 43
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Table 1.4-1. Instruction Field Names and Descriptions
Name Description
a* 32-bit general-purpose registers
as
In-out type (used as input/output operand). Stores
address information for read/write operations, which is
updated after such operations are completed.
at
In-out type. Temporarily stores operation results to the
EE.FFT.AMS.S16.ST.INCP instruction, which will be part of
the data to be written to memory.
ad
In type (used as input operand). Stores data used to
update address information.
av In type. Stores data to be written to memory.
ax,ay
In type. Stores data involved in arithmetic operations, e.g.,
shifting amounts, multipliers and etc.
au
Out type (used as output operand). Stores results of
instruction operations.
q* 128-bit general-purpose registers
qs
In type. Stores 128-bit data used for concatenation
operations.
qa,qx,qy,qm In type. Stores data used for vector operations.
qz Out type. Stores results of vector operations.
qu Out type. Stores data read from memory.
qv In type. Stores data to be written to memory.
fu
32-bit general-purpose floating-point register. Stores
floating-point data read from memory.
fv
32-bit general-purpose floating-point register. Stores
floating-point data to be written to memory.
sel2
1-bit immediate value ranging from 0 to 1. Used to select
signals.
sel4,upd4
2-bit immediate value ranging from 0 to 3. Used to select
signals.
sel8
3-bit immediate value ranging from 0 to 7. Used to select
signals.
sel16
4-bit immediate value ranging from 0 to 15. Used to select
signals.
sar2
1-bit immediate value ranging from 0 to 1. Represents
shifting numbers.
sar4
2-bit immediate value ranging from 0 to 3. Represents
shifting numbers.
sar16
4-bit immediate value ranging from 0 to 15. Represents
shifting numbers.
imm1
7-bit unsigned immediate value ranging from 0 to 127 with
an interval of 1. This is used to show the size of the
updated read/write operation address value.
Cont’d on next page
Espressif Systems 44
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Table 1.4-1 – cont’d from previous page
Name Description
imm2
7-bit unsigned immediate value ranging from 0 to 254 with
an interval of 2. This is used to show the size of the
updated read/write operation address value.
imm4
8-bit signed immediate value ranging from -256 to 252
with an interval of 8. This is used to show the size of the
updated read/write operation address value.
imm16
8-bit signed immediate value ranging from -2048 to 2032
with an interval of 16. This is used to show the size of the
updated read/write operation address value.
imm16f
4-bit signed immediate value ranging from -128 to 112 with
an interval of 16. This is used to show the size of the
updated read/write operation address value.
Some instructions have multiple operands with the same function. Those operands are distinguished by
adding numbers after field names. For example, the EE.LDF.128.IP instruction has four fu registers, fu0 ~ 3.
They are used to store 128-bit data read from memory.
1.5 Components of Extended Instruction Set
1.5.1 Registers
This section introduces all kinds of registers related to ESP32-S3’s extended instruction set, including the
original registers defined by Xtensa as well as customized registers. For register information, please refer to
Table 1.5-1.
Table 1.5-1. Register List of ESP32-S3 Extended Instruction Set
Register Mnemonics Quantity Bit Width Access Type
AR 161 32 R/W General-purpose registers
FR 16 32 R/W General-purpose registers to FPU
QR 8 128 R/W Customized general-purpose registers
SAR 1 6 R/W Special register
SAR_BYTE 1 4 R/W Customized special register
ACCX 1 40 R/W Customized special register
QACC_H 1 160 R/W Customized special register
QACC_L 1 160 R/W Customized special register
FFT_BIT_WIDTH 1 4 R/W Customized special register
UA_STATE 1 128 R/W Customized special register
1 The Xtensa processor has 64 internal AR registers. It is designed with the register windowing
technique, so that the software can only access 16 of the 64 AR registers at any given time. The
programming performance can be effectively improved by rotating windows, replacing function
calls, and saving registers when exceptions are triggered.
Espressif Systems 45
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
1.5.1.1 General-Purpose Registers
When using general-purpose register as operands in instructions, you need to explicitly declare the number of
the assigned register. For example:
EE.V ADDS.S8 q2, q0, q1
This instruction uses No.0 and No.1 QR registers as input vectors and stores the vector addition result in the
No.2 QR register.
AR
Each AR register operand in the instruction will occupy a 4-bit code length. You can select any of the 16 AR
registers as operands, and the 4-bit code value indicates the number to declare. The row ”a*” in table 1.4-1
lists various purposes of AR registers in the extended instruction set, including address storage and data
storage.
FR
Each FR register operand in the instruction will occupy a 4-bit code length. You can select any of the 16 FR
registers as operands, and the 4-bit code value indicates the number to declare. In ESP32-S3 extended
instruction set, there are only read and write instructions for floating-point data. They are 4 times more
efficient than the 32-bit floating-point data R/W instructions that are native to the Xtensa processor, thanks to
the 128-bit access bandwidth.
QR
In order to improve the execution efficiency of the program, operands are usually stored in general-purpose
registers to save time spent in reading from memory. The AR registers native to Xtensa only have 32-bit width,
while ESP32-S3 can access 128-bit data at a time, so they can only use 1/4 bandwidth capacity of the existing
data bus. For this reason, ESP32-S3 has added eight 128-bit customized general-purpose registers, i.e., QR
registers. QR registers are mainly used to store the data acquired/used by the 128-bit data bus to read or write
memory, as well as to temporarily store the operation results generated from 128-bit data operations.
As the processor executes instructions, an individual QR register is treated as 16 8-bit or 8 16-bit or 4 32-bit
operands depending on the vector operation bit width defined by the instruction, thus enabling a single
instruction to perform operations on multiple operands.
1.5.1.2 Special Registers
Different from general-purpose registers, special registers are implicitly called in specific instructions. You do
not need to and cannot specify a certain special register when executing instructions. For example:
EE.V MUL.S16 q2, q0, q1
This vector multiplication instruction uses q0 and q1 general-purpose registers as inputs. During the internal
operation, the intermediate 32-bit multiplication result is shifted to the right, and then the lower 16-bit of the
result is retained to form a 128-bit output to q2. The shift amount in the process is determined by the value in
the Shift Amount Register (SAR) and this SAR register will not appear in the instruction operand list.
Espressif Systems 46
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
SAR
The Shift Amount Register (SAR) stores the shift value in bits. There are two types of instructions in
ESP32-S3’s extended instruction set that use SAR. One is the type of instructions to shift vector data, including
EE.VSR.32 and EE.VSL.32. The former uses the lower 5 bits of SAR as the right-shift value, and the latter uses
the lower 5 bits of SAR as the left-shift value. The other type is multiplication instructions, including EE.VMUL.*,
EE.CMUL.*, EE.FFT.AMS.* and EE.FFT.CMUL.*. This type of instructions uses the value in SAR as the value for
the right shift of the intermediate multiplication result, which determines the accuracy of the final result.
SAR_BYTE
The SAR_BYTE stores the shift value in bytes. This special register is designed to handle the non-aligned
128-bit data (see Section 1.5.3). For vector arithmetic instructions, the data read or stored by extended
instructions are forced to be 16-byte aligned, but in practice, there is no guarantee that the data addresses
used are always 16-byte aligned.
EE.LD.128.USAR.IP and EE.LD.128.USAR.XP instructions write the lower 4-bit values of the memory access
register that represent non-aligned data to SAR_BYTE while reading 128-bit data from memory.
There are two types of instruction in ESP32-S3’s extended instruction set that use SAR_BYTE. One is
dedicated to handling non-aligned data in QR registers, including EE.SRCQ.* and EE.SRC.Q*. This type of
instruction will read two 16-byte data from two aligned addresses, before and after the non-aligned address,
put them together, and then shift it by the byte size of SAR_BYTE to get a 128-bit data from the non-aligned
address. The other type of instruction handles non-aligned data while executing arithmetic operations, which
usually has a suffix of ”.QUP”.
ACCX
Multiplier-accumulator. Instructions such as EE.VMULAS.*.ACCX* and EE.SRS.ACCX use this register during
operations. The former uses ACCX to accumulate all vector multiplication results of two QR registers, and the
latter right shifts the ACCX register.
QACC_H,QACC_L
Successive accumulators partitioned by segments. Instructions such as EE.VMULAS.*.QACC* and
EE.SRCMB.*.QACC use this type of registers during operations. These registers are mainly used to accumulate
vector multiplication results of two QR registers into the corresponding segments of QACC_H and QACC_L
respectively. The 16-bit vector multiplication results are accumulated into the corresponding 16 20-bit
segments respectively and the 32-bit results are accumulated into the corresponding 8 40-bit segement
respectively.
FFT_BIT_WIDTH
This special register is dedicated to the EE.BITREV instruction. The value inside this register is used to indicate
the operating mode of EE.BITREV. Its range is 0 ~ 7, indicating 3-bit ~ 10-bit operating mode respectively. For
more details, please refer to instruction EE.BITREV.
Espressif Systems 47
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
UA_STATE
This special register is dedicated to the EE.FFT.AMS.S16.LD.INCP.UAUP instruction. This register is used to
store the non-aligned 128-bit data read from memory. Next time when this instruction is called, the data in this
register is concatenated to the newly read non-aligned data and then the result is shifted to obtain the 128-bit
aligned data.
1.5.2 Fast GPIO Interface
ESP32-S3’s Xtensa processor adds two signal ports, i.e., GPIO_OUT and GPIO_IN. You can route signals from
the two ports to specified GPIO pins via the GPIO Matrix.
1.5.2.1 GPIO_OUT
An 8-bit processor output interface. Firstly, configure the 8-bit port signals to specified pins via GPIO Matrix.
For core0, their names are pro_alonegpio_out0~7. For core1, their names are core1_gpio_out0~7. Then you
can set certain bits of GPIO_OUT to 1 via instructions EE.WR_MASK_GPIO_OUT and EE.SET_BIT_GPIO_OUT, or
set certain bits to 0 via instruction EE.CLR_BIT_GPIO_OUT, so as to pull certain pins to high level or low level.
Using this method, you can get faster response than pulling pins through register configurations.
1.5.2.2 GPIO_IN
An 8-bit processor input interface. Firstly, configure the 8-bit port signals to specified pins via GPIO Matrix. For
core0, their names are pro_alonegpio_in0~7. For core1, their names are core1_gpio_in0~7. Then you can read
the eight GPIO pin levels and store them to the AR register through instruction EE.GET_GPIO_IN. Using this
method, you can get and handle the level changes on GPIO pins faster than reading registers to get pin level
status.
1.5.3 Data Format and Alignment
The current extended instruction set supports 1-byte, 2-byte, 4-byte, 8-byte and 16-byte data formats.
Besides, there is also a 20-byte format: QACC_H and QACC_L. However, there is no direct way to switch the
data between the two special registers and memory. You can read and write data of QACC_H and QACC_L via
five 4-byte (AR) registers or two 16-byte (QR) registers.
The table 1.5-2 lists bit length and alignment information for common data format (’x’ indicates that the bit is
either 0 and 1). The Xtensa processor uses byte as the smallest unit for addresses stored in memory in all data
formats. And little-endian byte order is used, with byte 0 stored in the lowest bit (the right side), as shown in
Figure 1.4-3.
Table 1.5-2. Data Format and Alignment
Data Format Length Aligned Addresses in Memory
1-byte 8 bits xxxx
2-byte 16 bits xxx0
4-byte 32 bits xx00
8-byte 64 bits x000
16-byte 128 bit 0000
Espressif Systems 48
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
However, if data is stored in memory at a non-aligned address, direct access to this address may cause it
being split into two accesses, which in turn affects the performance of the code. For example, if you expect
to read a 16-byte data from memory, as shown in Table 1.5-2, the data is stored in memory at 0000 when the
data is aligned. But actually the data is not aligned, so the low nibble of its address may be any one between
0000 ~ 1111 (binary). Assuming the lowest bit of its address is 0_0100, the processor will split the one-time
access to this data into two accesses, i.e., to 0_0000 and 1_0000 respectively. The processor then put
together the obtained two 16-byte data to get the required 16-byte data.
To avoid performance degradation caused by the above non-aligned access operations, all access addresses
in the extended instruction set are forced to be aligned, i.e., the lowest bits will be replaced by 0. For
example, if a read operation is initiated for 128-bit data at 0x3fc8_0024, the lowest 4-bit of this access address
will be forced to be set to 0. Eventually, the 128-bit data stored at 0x3fc8_0020 will be read. Similarly, the
lowest 3-bit of the access address for 64-bit data will be set to 0; the lowest 2-bit of the access address for
32-bit data will be set to 0; the lowest 1-bit of the access address for 16-bit data will be set to 0.
The above design requires aligned addresses of the access operations initiated. Otherwise, the data read will
not be what you expected. In application code, you need to explicitly declare the alignment of the variable or
array in memory. 16-byte alignment can meet the needs of most application scenarios.
The aligned(16) parameter declares that the variable is stored in a 16-byte aligned memory address. You can
also request a data space with its starting address 16-byte aligned via heap_caps_aligned_alloc.
Since the memory address of the data involved in some operations is uncertain in specific application
scenarios, this extended instruction set provides a special register SAR_BYTE and related instructions such as
EE.LD.128.USAR.* and EE.SRC.*, to handle non-aligned data.
Assume that the 128-bit non-aligned data address is stored in the general-purpose register a8. This 128-bit
data can be read into the specified QR register (q2 in the following example) by the following code:
EE.LD.128.USAR.IP q0, a8, 16
EE.V LD.128.IP q1, a8, 16
EE.SRC.Q q2, q0, q1
1.5.4 Data Overflow and Saturation Handling
Data overflow means that the size of the operation result exceeds the maximum value that can be stored in
the result register. Take the EE.VMUL.S8 instruction as an example, the result of two 8-bit multipliers is 16-bit,
and it should still be 16-bit after right-shifting. However, the final result will be stored in the 8-bit register, which
may cause the risk of data overflow.
In the design of the ESP32-S3’s instruction extensions, there are two ways to handle data overflow, namely
taking saturation and truncating the least significant bit. The former controls the calculation result according
the range of values that can be stored in the result register. If the result exceeds the maximum value of the
result register, take the maximum value; if the result is smaller than the minimum value of the result register,
take the minimum value. This approach will be explicitly indicated in the instruction descriptions. For example,
the EE.VADDS.* instructions perform saturation to the results of addition operations. Regarding the data
overflow handling for more instructions of their internal calculation results, the wraparound approach is used,
i.e., only the lower bit of the result that is consistent with the bit width of the result register will be retained and
stored in the result register.
Please note that for instructions that do not mention saturation handling method, the wraparound approach
Espressif Systems 49
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
will be used.
1.6 Extended Instruction List
Table 1.6-1 lists instruction types and corresponding instruction information included in the extended
instruction set. This section gives brief introduction to all types of instructions.
Espressif Systems 50
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Table 1.6-1. Extended Instruction List
Instruction Type Instruction1
Reference
Section
Read instructions
LD.QR
1.6.1
EE.VLD.128.[XP/IP]
EE.VLD.[H/L].64.[XP/IP]
EE.VLDBC.[8/16/32].[-/XP/IP]
EE.VLDHBC.16.INCP
EE.LDF.[64/128].[XP/IP]
EE.LD.128.USAR.[XP/IP]
EE.LDQA.[U/S][8/16].128.[XP/IP]
EE.LD.QACC_[H/L].[H.32/L.128].IP
EE.LD.ACCX.IP
EE.LD.UA_STATE.IP
EE.LDXQ.32
Write instructions
ST.QR
1.6.2
EE.VST.128.[XP/IP]
EE.VST.[H/L].64.[XP/IP]
EE.STF.[64/128].[XP/IP]
EE.ST.QACC_[H/L].[H.32/L.128].IP
EE.ST.ACCX.IP
EE.ST.UA_STATE.IP
EE.STXQ.32
Data exchange instructions
MV.QR
1.6.3
EE.MOVI.32.A
EE.MOVI.32.Q
EE.VZIP.[8/16/32]
EE.VUNZIP.[8/16/32]
EE.ZERO.Q
EE.ZERO.QACC
EE.ZERO.ACCX
EE.MOV.S8.QACC
EE.MOV.S16.QACC
EE.MOV.U8.QACC
EE.MOV.U16.QACC
Arithmetic instructions
EE.VADDS.S[8/16/32].[-/LD.INCP/ST.INCP]
1.6.4
EE.VSUBS.S[8/16/32].[-/LD.INCP/ST.INCP]
EE.VMUL.[U/S][8/16].[-/LD.INCP/ST.INCP]
EE.CMUL.S16.[-/LD.INCP/ST.INCP]
EE.VMULAS.[U/S][8/16].ACCX.[-/LD.IP/LD.XP]
EE.VMULAS.[U/S][8/16].QACC.[-/LD.IP/LD.XP/LDBC.INCP]
EE.VMULAS.[U/S][8/16].ACCX.[LD.IP/LD.XP].QUP
EE.VMULAS.[U/S][8/16].QACC.[LD.IP/LD.XP/LDBC.INCP].QUP
EE.VSMULAS.S[8/16].QACC.[-/LD.INCP]
Con’t on next page
Espressif Systems 51
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Table1.6-1 – con’t from previous page
Instruction Type Instruction1
Reference
Section
EE.SRCMB.S[8/16].QACC
EE.SRS.ACCX
EE.VRELU.S[8/16]
EE.VPRELU.S[8/16]
Comparison instructions
EE.VMAX.S[8/16/32].[-/LD.INCP/ST.INCP]
EE.VMIN.S[8/16/32].[-/LD.INCP/ST.INCP] 1.6.5
EE.VCMP.[EQ/LT/GT].S[8/16/32]
Bitwise logic instructions
EE.ORQ
1.6.6
EE.XORQ
EE.ANDQ
EE.NOTQ
Con’t on next page
Espressif Systems 52
Submit Documentation Feedback
ESP32-S3 TRM (Version 1.7)
Chapter 1 Processor Instruction Extensions (PIE) GoBack
Table1.6-1 – con’t from previous page
Instruction Type Instruction1
Reference
Section
Shift instructions
EE.SRC.Q
1.6.7
EE.SRC.Q.QUP
EE.SRC.Q.LD.[XP/IP]
EE.SLCI.2Q
EE.SLCXXP.2Q
EE.SRCI.2Q
EE.SRCXXP.2Q
EE.SRCQ.128.ST.INCP
EE.VSR.32
EE.VSL.32
FFT dedicated instructions
EE.FFT.R2BF.S16.[-/ST.INCP]
1.6.8
EE.FFT.CMUL.S16.[LD.XP/ST.XP]
EE.BITREV
EE.FFT.AMS.S16.[LD.INCP.UAUP/LD.INCP/LD.R32.DECP/ST.INCP]
EE.FFT.VST.R32.DECP
GPIO control instructions
EE.WR_MASK_GPIO_OUT
1.6.9
EE.SET_BIT_GPIO_OUT
EE.CLR_BIT_GPIO_OUT
EE.GET_GPIO_IN
Processor control instructions
RSR.*
1.6.10
WSR.*
XSR.*
RUR.*
WUR.*
1 For detailed information of these instructions, please refer to Section 1.8.
