    .text
    .align 4
    .global calculate_sad_vector
    .type   calculate_sad_vector,@function

// int32_t calculate_sad_vector(const int8_t *A, const int8_t *B, int len, const int8_t *ones)
// A2: ptr A
// A3: ptr B
// A4: len (number of bytes, assumed multiple of 16)
// A5: ptr ones (aligned 16 bytes)
// Return A2: sum

calculate_sad_vector:
    entry   a1, 32            # Reserve stack

    // Clear ACCX
    ee.zero.accx
    
    // Clear SAR? No, EE.LD.USAR sets SAR_BYTE. We don't need manual SAR clear.
    // Wait, does SRC.Q use SAR or SAR_BYTE?
    // Manual: "This type of instruction ... shift it by the byte size of SAR_BYTE"
    // So we don't need to clear SAR. 
    
    // Load Ones Vector into Q7
    mov     a6, a5
    ee.vld.128.ip q7, a6, 16 

    // Prepare Pipeline for B (Unaligned)
    // A3 is ptrB.
    // We load first block into Q5 (using Q5 as prev).
    // EE.LD.128.USAR.IP updates A3.
    // Q5 = Pre-load
    ee.ld.128.usar.ip q5, a3, 16 

loop_sad:
    // Check if len < 16
    blti    a4, 16, end_loop

    // Load A (Aligned) -> Q0
    ee.vld.128.ip q0, a2, 16
    
    // Load B (Next Aligned Block) -> Q6 (curr)
    ee.vld.128.ip q6, a3, 16
    
    // Construct B (Actual) -> Q1
    // Combine Q5 (prev) and Q6 (curr) based on SAR_BYTE (set by LD.USAR)
    // EE.SRC.Q result, low, high
    // Q5 is low address (prev), Q6 is high address (curr)
    ee.src.q      q1, q5, q6
    
    // Calculate Abs Diff: |A - B| -> Q4
    ee.vmax.s8    q2, q0, q1
    ee.vmin.s8    q3, q0, q1
    ee.vsubs.s8   q4, q2, q3
    
    // Accumulate -> ACCX
    ee.vmulas.u8.accx q4, q7
    
    // Pipeline update: Q5 = Q6
    // Use data exchange instruction or logic
    ee.orq        q5, q6, q6
    
    // Decrement len (A4)
    addi    a4, a4, -16
    j       loop_sad

end_loop:
    // Store Result
    mov     a6, a1
    ee.st.accx.ip a6, 0  
    l32i.n  a2, a1, 0
    retw.n
