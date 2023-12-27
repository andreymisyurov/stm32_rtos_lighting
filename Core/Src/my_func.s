.syntax unified

.global my_strlen 					@ Function for calculating the length of a str.
.global my_strcpy					@ Copying a string into another str.
.global my_strcmp					@ Comparison of two strings.
.global my_strncmp					@ Comparison of two strings with a length limit.
.global my_strcat					@ Concatenation (joining) of two strings.
.global my_strchr					@ Search for the first occurrence of a char in a str.
.global my_strrchr 					@ Search for the last occurrence of a char in a str.
.global my_findchr					@ Finding the index of the first occurrence of a char in a str.
.global my_strstr 					@ Searching for a substr within a str.

.type my_strlen, 	%function
.type my_strcpy, 	%function
.type my_strcmp, 	%function
.type my_strncmp, 	%function
.type my_strcat, 	%function
.type my_strchr, 	%function
.type my_strrchr, 	%function
.type my_findchr, 	%function
.type my_strstr, 	%function

my_strlen:
    MOV  r3, #0    					@ initialize counter

    strlen_loop:
    LDRB  	r2, [r0]      			@ load current char from str
    CMP   	r2, #0        			@ check if it's end of str
    BEQ   	strlen_done       		@ if yes, exit loop
    ADD   	r3, r3, #1    			@ increment counter
    ADD   	r0, r0, #1    			@ move to next char
    B     	strlen_loop      		@ repeat loop

    strlen_done:
    MOV 	r0, r3      			@ return counter as result
    BX 		lr           			@ return from function
.size my_strlen, .-my_strlen

my_strcpy:

    strcpy_loop:
    LDRB  	r2, [r0]      			@ load current char from src
    CMP   	r2, #0        			@ check for end of src str
    BEQ   	strcpy_done    			@ if end, exit loop
    STRB  	r2, [r1]      			@ store char in dest
    ADD   	r0, r0, #1    			@ move to next char in src
    ADD   	r1, r1, #1    			@ move to next position in dest
    B     	strcpy_loop   			@ repeat loop

    strcpy_done:
    BX 		lr        				@ return
.size my_strcpy, .-my_strcpy

my_strcmp:
    PUSH 	{r4, r5, r6, lr}       	@ save used registers and return address
    MOV 	r4, r0                  @ store address of first str in r4
    MOV 	r5, r1                  @ store address of second str in r5
    BL 		my_strlen               @ get length of first str
    MOV 	r6, r0                  @ store length of first str in r6
    MOV 	r0, r5                  @ pass address of second str to r0 for my_strlen
    BL 		my_strlen               @ get length of second str
    CMP 	r6, r0                  @ compare lengths of strings
    BLT  	strcmp_lesser          	@ if first str is shorter
    BGT  	strcmp_greater         	@ if first str is longer

    strcmp_loop:
    LDRB 	r0, [r4]               	@ load char from first str
    LDRB 	r1, [r5]               	@ load char from second str
    CMP  	r0, #0                 	@ check for end of first str
    BEQ  	strcmp_equal           	@ if end, strings are equal
    CMP  	r0, r1                 	@ compare characters
    BLT  	strcmp_lesser          	@ if char of first str is lesser
    BGT  	strcmp_greater         	@ if char of first str is greater
    ADD  	r4, r4, #1             	@ move to next char of first str
    ADD  	r5, r5, #1             	@ move to next char of second str
    B    	strcmp_loop            	@ repeat loop

    strcmp_equal:
    MOV 	r0, 0                   @ set result to 0
    POP 	{r4, r5, r6, pc}        @ restore registers and return

    strcmp_lesser:
    MOV 	r0, -1                  @ set result to -1
    POP 	{r4, r5, r6, pc}        @ restore registers and return

    strcmp_greater:
    MOV 	r0, 1                   @ set result to 1
    POP 	{r4, r5, r6, pc}        @ restore registers and return
.size my_strcmp, .-my_strcmp

my_strcat:
    strcat_find_end:
    LDRB 	r2, [r0]               	@ load char from first str
    CMP 	r2, #0                  @ check for end of str
    BNE 	strcat_next_char        @ if not end, go to next char
    B 		strcat_start_copy       @ if end, start copying

    strcat_next_char:
    ADD 	r0, r0, #1              @ increment position in first str
    B 		strcat_find_end         @ return to finding end

    strcat_start_copy:
    LDRB 	r2, [r1]               	@ load char from second str
    CMP 	r2, #0                  @ check for end of second str
    BEQ 	strcat_exit             @ if end, exit func
    STRB 	r2, [r0]               	@ copy char to first str
    ADD 	r0, r0, #1              @ move to next position in first str
    ADD 	r1, r1, #1              @ move to next char in second str
    B 		strcat_start_copy       @ repeat copy process

    strcat_exit:
    BX lr                       	@ return from function
.size my_strcat, .-my_strcat

my_strchr:
    strchr_loop:
    LDRB 	r2, [r0]            	@ load char from str
    CMP 	r2, r1              	@ compare with target char
    BEQ 	strchr_found        	@ if found, exit function
    CMP 	r2, #0              	@ check for end of str
    BEQ 	strchr_not_found    	@ if end, char not found
    ADD 	r0, r0, #1          	@ move to next char
    B 		strchr_loop         	@ repeat loop

    strchr_found:
    BX 		lr                  	@ return from function

    strchr_not_found:
    MOV 	r0, #0              	@ set return value to NULL
    BX 		lr                  	@ return from function
.size my_strchr, .-my_strchr

my_strrchr:
    MOV 	r3, #0              	@ initialize last occurrence as NULL

    strrchr_loop:
    LDRB 	r2, [r0]            	@ load char from str
    CMP 	r2, r1              	@ compare with target char
    BEQ 	strrchr_update_last 	@ if found, update last occurrence
    CMP 	r2, #0              	@ check for end of str
    BEQ 	strrchr_exit        	@ if end, exit function
    ADD 	r0, r0, #1          	@ move to next char
    B 		strrchr_loop        	@ repeat loop

    strrchr_update_last:
    MOV 	r3, r0              	@ store current position as last occurrence
    ADD 	r0, r0, #1          	@ move to next char
    B 		strrchr_loop        	@ repeat loop

    strrchr_exit:
    MOV 	r0, r3              	@ return last occurrence
    BX 		lr                  	@ return from function
.size my_strrchr, .-my_strrchr

my_findchr:
    MOV 	r3, #0                  @ initialize position counter
    findchr_search_loop:
    LDRB 	r2, [r0]               	@ load char from str
    CMP 	r2, r1                  @ compare with target char
    BEQ 	findchr_found           @ if found, exit function
    CMP 	r2, #0                  @ check for end of str
    BEQ 	findchr_not_found       @ if end, char not found
    ADD 	r0, r0, #1              @ move to next char
    ADD 	r3, r3, #1              @ increment position counter
    B 		findchr_search_loop     @ repeat loop

    findchr_found:
    MOV 	r0, r3                  @ return position of found char
    BX 		lr                      @ return from function

    findchr_not_found:
    MOV 	r0, #-1                 @ set return value to -1 (not found)
    BX 		lr                      @ return from function
.size my_findchr, .-my_findchr

my_strncmp:
    PUSH 	{r4, r5, r7, lr}       	@ save used registers
    LDRB 	r7, [r2]               	@ load number of characters to compare into r7
    MOV 	r4, r0                  @ store address of first str in r4
    MOV 	r5, r1                  @ store address of second str in r5

    strncmp_loop:
    CMP 	r7, #0                  @ check if characters left
    BEQ 	strncmp_end_equal       @ exit if none left
    LDRB 	r0, [r4]               	@ load char from first str
    LDRB 	r1, [r5]               	@ load char from second str
    CMP 	r0, r1                  @ compare chars
    BLT 	strncmp_end_less        @ if first is less
    BGT 	strncmp_end_greater     @ if first is greater
    CMP 	r0, #0                  @ check for str end
    BEQ 	strncmp_end_equal       @ exit if end reached
    ADD 	r4, r4, #1              @ next char in first str
    ADD 	r5, r5, #1              @ next char in second str
    SUB 	r7, r7, #1              @ decrement compare count
    B 		strncmp_loop            @ repeat comparison

    strncmp_end_equal:
    MOV 	r0, #0                  @ set result to 0
    POP 	{r4, r5, r7, pc}        @ restore registers and return

    strncmp_end_less:
    MOV 	r0, #-1                 @ first str is less
    POP 	{r4, r5, r7, pc}        @ restore registers and return

    strncmp_end_greater:
    MOV 	r0, #1                  @ first str is greater
    POP 	{r4, r5, r7, pc}        @ restore registers and return
.size my_strncmp, .-my_strncmp

my_strstr:
	PUSH 	{r4, r5, r10, r11, lr}	@ save used registers
	MOV 	r4, r0					@ store main str address
	MOV 	r5, r1					@ store target substr address
	MOV		r0, r1					@ set target substr for length calculation
	BL		my_strlen				@ call my_strlen for target substr
	MOV		r10, r0					@ store target len
	LDRB 	r3, [r5]				@ load first char of target

	strstr_main_loop:
	LDRB 	r2, [r4]				@ load current char from main str
	CMP		r2, #0					@ check for str end
	BEQ		strstr_no_match			@ exit if end reached
	CMP 	r2, r3					@ compare with target's first char
	BNE		strstr_next_char		@ next char if no match
	MOV		r0, r4					@ set remaining main str for length check
	BL		my_strlen				@ call my_strlen for remaining main str
	CMP		r10, r0					@ compare lengths
	BGT		strstr_no_match			@ exit if target is longer
	MOV		r0, r4					@ set main str for my_strncmp
	MOV		r1, r5					@ set target substr for my_strncmp
	MOV		r3, r10					@ set length for my_strncmp
	BL		my_strncmp				@ call my_strncmp
	CMP		r0, #0					@ check my_strncmp result
	BEQ		strstr_match_found		@ exit if match found

	strstr_next_char:
	ADD 	r4, r4, #1				@ move to next char in main str
	B		strstr_main_loop		@ repeat the loop

	strstr_match_found:
	MOV 	r0, r4					@ return start of match
	POP 	{r4, r5, r10, r11, pc}	@ restore registers and return

	strstr_no_match:
	MOV		r0, #0					@ set result to 0
	POP 	{r4, r5, r10, r11, pc}	@ restore registers and return
.size my_strstr, .-my_strstr
