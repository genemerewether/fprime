; ======================================================================
; \title  HL_sys_stack.asm
; \author kubiak
; \brief  asm file for check stack usage
;
; \copyright
; Copyright 2009-2015, by the California Institute of Technology.
; ALL RIGHTS RESERVED.  United States Government Sponsorship
; acknowledged. Any commercial use must be negotiated with the Office
; of Technology Transfer at the California Institute of Technology.
;
; This software may be subject to U.S. export control laws and
; regulations.  By accepting this document, the user agrees to comply
; with all U.S. export laws and regulations.  User has the
; responsibility to obtain export licenses, or other export authority
; as may be required before exporting such information to foreign
; countries or providing access to foreign persons.
; ======================================================================

    .text
    .arm

;-------------------------------------------------------------------------------
; Initialize stack with pattern up to (not including) $sp.

    .def initStackPattern
    .asmfunc

; User stack runs from 0x08000000 to 0x08020000
initStackPattern
    ; Pattern passed in as r0

    mov r1, #0x08000000 ; Start address

    sub r2, sp, r1      ; Load number of bytes to write into r2

_initStackPatternLoop_
    stmia r1!, {r0}      ; Write word to RAM and increment r1 by one word
    subs r2, r2, #4      ; Subtract 4 bytes
    bne _initStackPatternLoop_ ; Continue loop if non-zero
    
    bx lr ; Return

    .endasmfunc


;-------------------------------------------------------------------------------
; Check stack for pattern from 0x08000000

    .def checkStackPattern
    .asmfunc

checkStackPattern
    ; r0 contains the pattern

    push {r4-r11}

    mov r1, #0x08000000 ; Start address
    mov r3, sp          ; Stop address

    ; Check 4 words on the stack at 2048 by increments. This quickly
    ; jumps through the stack looking for the end. We will do a more
    ; thorough seach next
_checkStackPatternLoopFast_
    cmp r1, r3 ; Check if we've reached the end of stack memory
    bhi _checkStackPatternLoopFastDone_ ; Exit this loop if we have

    ldmia r1, {r4-r7}

    cmp r4, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopFastDone_ ; Break if the pattern does not match

    cmp r5, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopFastDone_ ; Break if the pattern does not match

    cmp r6, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopFastDone_ ; Break if the pattern does not match
    
    cmp r7, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopFastDone_ ; Break if the pattern does not match

    add r1, r1, #2048

    b _checkStackPatternLoopFast_

_checkStackPatternLoopFastDone_

    ; The previous check did not pass. r1 is the value of the failed check

    cmp r1, #0x08000000 ; If we never update r1 then subtracting from it would be bad
    ; This means that stacks with less than 4 words free will not be accuratly counted
    beq _checkStackPatternLoopSlowDone_

    ; Go back 2048 bytes to start slow search
    sub r1, r1, #2048

_checkStackPatternLoopSlow_
    cmp r1, r3 ; Check if we've reached the end of stack memory
    bhi _checkStackPatternLoopSlowDone_ ; Quit if we have

    ldmia r1, {r4-r11} ; Read the next 4 stack words into r4-r7

    cmp r4, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r5, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r6, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match
    
    add r1, r1, #4
    cmp r7, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r8, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r9, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r10, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    cmp r11, r0 ; Compare the word to the pattern
    bne _checkStackPatternLoopSlowDone_ ; Break if the pattern does not match

    add r1, r1, #4
    b _checkStackPatternLoopSlow_
    
_checkStackPatternLoopSlowDone_

    sub r0, r1, #0x08000000 ; Calculate the total space used. Return value in r0

    pop {r4-r11}
    bx lr ; Return

    .endasmfunc


