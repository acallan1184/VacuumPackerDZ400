#include <xc.h>

// =========================
// Configuration Bits
// =========================
#pragma config BOREN = ON // Brown-out reset enable bit enabled
#pragma config CP    = OFF // Code Protection bit disabled
#pragma config PWRTE = 0 // Power-up timer enable bit is disablled
#pragma config WDTE  = 0 // Watchdog Timer Enable bit is disabled
#pragma config FOSC  = HS // Oscillator Selection bit

#define _XTAL_FREQ 4000000 // 4 MHz Crystal
#define NUM_SWITCHES 7

#define NO_HEAT 0
#define LOW_HEAT 1
#define MEDIUM_HEAT 2
#define HIGH_HEAT 3

// g‑segment only (bit4 = 1)
#define SEG_G_ONLY     0x10
#define SEG_A_ONLY     0x01
#define SEG_DP_ONLY    0x08
#define SEG_CDEG_ONLY  0x17
#define SEG_CDE_ONLY   0x07

// LED patterns for state indication on third display (7 individual LEDs, common cathode)
#define LED_ALL    0xFF   // All LEDs on
#define LED_VACUUM 0x80   // LED for vacuum state
#define LED_COOL   0x40   // LED for cool state
#define LED_UNUSED 0x20   // LED for seal state
#define LED_FATAL  0x10   // LED for fatal state
#define LED_SEAL   0x08   // LED for high temperature state
#define LED_MED    0x04   // LED for medium temperature state
#define LED_LOW    0x02   // LED for low temperature state
#define LED_HIGH   0x01   // There is no on board LED for this.
#define LED_OFF    0x00   // All LEDs off

// =========================
// State Machine
// =========================
#define STATE_BOOT      0
#define STATE_IDLE      1
#define STATE_VACUUM    2
#define STATE_SEAL      3
#define STATE_COOL      4
#define STATE_DEFLATE   5
#define STATE_LID_STUCK 6

// =========================
// Segment pins    (PB0,PB1,PB2, PB3,PB4,PB5,PB6,PB7)
// Segment mapping (c,d,e,dp,g,f,a,b)
// bit7=b, bit6=a, bit5=f, bit4=g, bit3=dp, bit2=e, bit1=d, bit0=c
// =========================
const unsigned char segmap[10] = 
{
    0xE7, // 0
    0x81, // 1
    0xD6, // 2
    0xD3, // 3
    0xB1, // 4
    0x73, // 5
    0x77, // 6
    0xC1, // 7
    0xF7, // 8
    0xF3  // 9
};

volatile unsigned char digits[3] = {0, 0, 0};
volatile unsigned char switches_state = 0;
volatile unsigned char current_digit = 0;
volatile unsigned char countdown_tens = 0;
volatile unsigned char countdown_ones = 0;
volatile unsigned char buzzer_update = 1; // Flag to trigger buzzer update
volatile unsigned int buzzer_timer = 0;
volatile unsigned char lidstuck_symbol_toggle = 0; // Flag to toggle the lid stuck symbol

// Countdown timers inital settings
#define VACUUM_USER_SETTING 40 // 40 seconds
#define SEAL_USER_SETTING 25 // 2.5 seconds (25 * 0.1s)
#define COOL_USER_SETTING 50 // 5 seconds (50 * 0.1s)
#define DEFLATE_USER_SETTING 50 // 5 seconds (50 * 0.1s)

// Countdown timers live settings
volatile unsigned char vacuum_countdown_seconds=VACUUM_USER_SETTING;
volatile unsigned char seal_countdown_100ms=SEAL_USER_SETTING;
volatile unsigned char cool_countdown_100ms=COOL_USER_SETTING;
volatile unsigned char deflate_countdown_100ms=DEFLATE_USER_SETTING;

volatile unsigned char state = STATE_BOOT;

// ISR counters for various states
volatile unsigned int vacuum_isr_counter = 0;  // Counts ISR ticks for 1 second intervals
volatile unsigned int seal_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int cool_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int deflate_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int lidstuck_isr_counter = 0;  // Counts ISR ticks for 1 second intervals

// Switch sampling
volatile unsigned char switch_raw[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Initialize all switches to unpressed (high)
volatile unsigned char switch_last[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Initialize all switches to unpressed (high);
volatile unsigned char switch_debounced[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Initialize all switches to unpressed (high);
volatile unsigned char switch_stable_count[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Initialize all switches to unpressed (high);
volatile unsigned char temp_setting = NO_HEAT; // Default temperature setting
volatile unsigned char LED_TEMP = LED_OFF; // Default LED state for temperature
// =========================
// Buzzer tone generator (2 kHz)
// =========================
void buzz_2khz(unsigned int duration_ms)
{
    buzzer_timer = duration_ms;
}

// =========================
// Interrupt (Timer0)
// =========================
void __interrupt() isr(void)
{
    if (T0IF) 
    {
        T0IF = 0; // Reset the overflow bit
        TMR0 = 225; // Preload for ~1ms overflow (4MHz / 4 = 1MHz, prescaler 1:32 => 31.25kHz, 8ms overflow)

        // turn off all digits
        RA0 = 0; // digit 0 OFF
        RA1 = 0; // digit 1 OFF
        RA2 = 0; // LEDs OFF
        PORTB = 0x00;  // Clear PORTB to prevent ghosting/dimly lit segments

        // read inputs
        switch_raw[0] = RA3; // Read RA3 (lid sensor)
        TRISB = 0xFF; // Set PORTB as input
        //__delay_us(50); // Wait for pin state to change after switching to input (might not be needed)
        switch_raw[1] = RB7; // Read RB7 (switch 1)
        switch_raw[2] = RB6; // Read RB6 (switch 2)
        switch_raw[3] = RB3; // Read RB3 (switch 3)
        switch_raw[4] = RB1; // Read RB1 (switch 4)
        switch_raw[5] = RB2; // Read RB2 (switch 5)
        switch_raw[6] = RB0; // Read RB0 (switch 6)

        for (unsigned char i = 0; i < NUM_SWITCHES; i++) {

            if (switch_raw[i] == switch_last[i]) {
                if (switch_stable_count[i] < 20)   // 20 ms debounce
                    switch_stable_count[i]++;
            } else {
                switch_stable_count[i] = 0;
                switch_last[i] = switch_raw[i];
            }

            if (switch_stable_count[i] >= 20) {
                switch_debounced[i] = switch_last[i];
            }
        }

        // drive next digit
        TRISB = 0x00; // Set PORTB as output
        PORTB = digits[current_digit]; // Output the segment pattern for the current digit

        switch (current_digit) 
        {
            case 0: RA0 = 1; break;   // digit 0 ON
            case 1: RA1 = 1; break;   // digit 1 ON
            case 2: RA2 = 1; break;   // digit 2 ON (LEDs)
        }

        current_digit++; // Move to the next digit for the next ISR call
        if (current_digit >= 3) // Wrap around after the last digit
            current_digit = 0; // Reset to the first digit
        
        // Track vacuum countdown timing (every 1ms tick)
        if (state == STATE_VACUUM) 
        {
            vacuum_isr_counter++;
        }
        // Track seal countdown timing (every 1ms tick)
        if (state == STATE_SEAL) 
        {
            seal_isr_counter++;
        }
        // Track cool countdown timing (every 1ms tick)
        if (state == STATE_COOL) 
        {
            cool_isr_counter++;
        }
        // Track deflate countdown timing (every 1ms tick)
        if (state == STATE_DEFLATE) 
        {
            deflate_isr_counter++;
        }
        // Track lid stuck countdown timing (every 1ms tick)
        if (state == STATE_LID_STUCK) 
        {
            lidstuck_isr_counter++;
        }

        // Buzzer tone generator (2 kHz)
        // ============================
        if (buzzer_timer > 0) 
        {
            RC0 = ~RC0;              // toggle buzzer pin
            buzzer_timer--;              // countdown tone duration
        } 
        else 
        {
            RC0 = 0;                     // ensure buzzer OFF
        }
    }
}

// =========================
// Main
// =========================
void main(void)
{
    // all pins digital
    ADCON0 = 0x00; // Disable ADC
    ADCON1 = 0x06; // Set all pins to digital I/O

    TRISA0 = 0;  // RA0 digit 0 as output
    TRISA1 = 0;  // RA1 digit 1 as output
    TRISA2 = 0;  // RA2 digit 2 as output
    TRISA3 = 1;  // RA3 lid sensor as input
    TRISB = 0x00; // Set PORTB as output for segments
    TRISC0 = 0;  // RC0 buzzer as output
    TRISC2 = 0;  // RC2 vent air solenoid as output
    TRISC4 = 0;  // RC4 vacuum pump relay and vacuum air solenoid as output
    TRISC5 = 0;  // RC5 medium temperature relay as output
    TRISC6 = 0;  // RC6 low temperature relay as output
    TRISC7 = 0;  // RC7 high temperature relay as output

    RA0 = RA1 = RA2 = RC0 = RC2 = RC4 = RC5 = RC6 = RC7 = 0; // Turn off all outputs initially
    PORTB = 0x00; // Clear PORTB to start with all segments off
    // Timer0: 4MHz / 4 = 1MHz instruction cycle, prescaler 1:32 => 31.25kHz timer frequency => 8ms overflow
    OPTION_REG = 0b00000100; // Prescaler 1:32, TMR0 source = internal instruction cycle clock (Fosc/4)
    TMR0 = 225; // Preload for ~1ms overflow

    T0IE = 1; // Enable Timer0 interrupt
    GIE  = 1; // Enable global interrupts

    while (1)
    {   
        if (switch_debounced[6] == 0 && switch_last[6] == 1 && state == STATE_IDLE) // Check for rising edge on switch 6 (temperature setting button)
        {
            temp_setting++; // Increment temperature setting
            if (temp_setting > HIGH_HEAT) // Wrap around if it exceeds HIGH_HEAT
            {
                temp_setting = NO_HEAT; // Reset to NO_HEAT
            }
            if (temp_setting == NO_HEAT)
            {
                LED_TEMP = LED_OFF; // Turn off all temperature LEDs
            }
            else if (temp_setting == LOW_HEAT)
            {
                LED_TEMP = LED_LOW; // Turn on low temperature LED
            }
            else if (temp_setting == MEDIUM_HEAT)
            {
                LED_TEMP = LED_MED; // Turn on medium temperature LED
            }
            else if (temp_setting == HIGH_HEAT)
            {
                LED_TEMP = LED_HIGH; // Turn on high temperature LED
            }
            digits[2] = digits[2] | LED_TEMP; // Update the third display to show the current temperature setting
        }
        if (switch_debounced[1] == 0 && switch_last[1] == 1 && state !=STATE_IDLE && state != STATE_BOOT)
        {
            // Display all segments
            RC2=1; // Turn on vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            state = STATE_DEFLATE; // Reset to idle state
            vacuum_countdown_seconds = VACUUM_USER_SETTING; // Reset vacuum countdown
            seal_countdown_100ms = SEAL_USER_SETTING; // Reset seal countdown
            cool_countdown_100ms = COOL_USER_SETTING; // Reset cool countdown
            deflate_countdown_100ms = DEFLATE_USER_SETTING; // Reset deflate countdown
            vacuum_isr_counter = 0; // Reset vacuum ISR counter
            seal_isr_counter = 0; // Reset seal ISR counter
            cool_isr_counter = 0; // Reset cool ISR counter
            deflate_isr_counter = 0; // Reset deflate ISR counter
            lidstuck_isr_counter = 0; // Reset lid stuck ISR counter
        }
        // Handle boot state
        if (state == STATE_BOOT) 
        {
            // Display all segments
            RC2=0; // Turn off vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            digits[0] = 0xFF;
            digits[1] = 0xFF;
            digits[2] = LED_ALL;  // LEDs on during boot for visual feedback
            if (buzzer_update == 1)
            {  
            buzz_2khz(1000);  // 1 second of 2 kHz tone
            buzzer_update = 0;  // Reset the update flag
            }
            if (buzzer_timer == 0) 
            {
            state = STATE_IDLE;
            }    
        }      
        else if (state == STATE_IDLE) 
        {
            RC2=0; // Turn off vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            digits[0] = SEG_G_ONLY;
            digits[1] = SEG_G_ONLY;
            digits[2] = LED_TEMP;  // Turn off all LEDs in idle state
            if (switch_debounced[0] == 0) 
            {
                state = STATE_VACUUM;
            } 
        } 
        else if (state == STATE_VACUUM) 
        {
            RC2=0; // Turn off vent air solenoid
            RC4=1; // Turn on vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            if (switch_debounced[0] == 1 || switch_debounced[1] == 0) 
            {
                state = STATE_IDLE;
            }
            else
            {
                countdown_tens = vacuum_countdown_seconds / 10;
                countdown_ones = vacuum_countdown_seconds % 10;
                digits[0] = segmap[countdown_ones];
                digits[1] = segmap[countdown_tens];
                digits[2] = LED_VACUUM|LED_TEMP;  // Turn off all LEDs in idle state
                if (vacuum_isr_counter >= 1000) // 1 second has passed
                {
                    vacuum_isr_counter = 0; // Reset the counter
                    vacuum_countdown_seconds--;
                    if (vacuum_countdown_seconds == 0) 
                    {
                        vacuum_countdown_seconds = VACUUM_USER_SETTING; // Reset for next time
                        state = STATE_SEAL;
                    } 
                    else 
                    {
                        countdown_tens = vacuum_countdown_seconds / 10;
                        countdown_ones = vacuum_countdown_seconds % 10;
                        digits[0] = segmap[countdown_ones];
                        digits[1] = segmap[countdown_tens];
                    }
                }
            }
        }
        else if (state == STATE_SEAL) 
        {
            RC2=0; // Turn off vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            if (temp_setting == NO_HEAT)
            {
                RC5=0; // Turn off medium temperature relay
                RC6=0; // Turn off low temperature relay
                RC7=0; // Turn off high temperature relay
            }
            else if (temp_setting == LOW_HEAT)
            {
                RC5=0; // Turn off medium temperature relay
                RC6=1; // Turn on low temperature relay
                RC7=0; // Turn off high temperature relay
            }
            else if (temp_setting == MEDIUM_HEAT)
            {
                RC5=1; // Turn on medium temperature relay
                RC6=0; // Turn off low temperature relay
                RC7=0; // Turn off high temperature relay
            }
            else if (temp_setting == HIGH_HEAT)
            {
                RC5=0; // Turn off medium temperature relay
                RC6=0; // Turn off low temperature relay
                RC7=1; // Turn on high temperature relay
            }
            else 
            {
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            }
            countdown_tens = seal_countdown_100ms / 10;
            countdown_ones = seal_countdown_100ms % 10;
            digits[0] = segmap[countdown_ones];
            digits[1] = segmap[countdown_tens] | SEG_DP_ONLY;
            digits[2] = LED_SEAL|LED_TEMP;  // Turn off all LEDs in idle state
            if (seal_isr_counter >= 100) // 0.1 second has passed
            {
                seal_isr_counter = 0; // Reset the counter
                seal_countdown_100ms--;
                if (seal_countdown_100ms == 0) 
                {
                    seal_countdown_100ms = SEAL_USER_SETTING; // Reset for next time
                    state = STATE_COOL;
                } 
                else 
                {
                    countdown_tens = seal_countdown_100ms / 10;
                    countdown_ones = seal_countdown_100ms % 10;
                    digits[0] = segmap[countdown_ones];
                    digits[1] = segmap[countdown_tens];
                }
            }
        }
        else if (state == STATE_COOL) 
        {
            RC2=0; // Turn off vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            countdown_tens = cool_countdown_100ms / 10;
            countdown_ones = cool_countdown_100ms % 10;
            digits[0] = segmap[countdown_ones];
            digits[1] = segmap[countdown_tens] | SEG_DP_ONLY;
            digits[2] = LED_COOL|LED_TEMP;  // Turn off all LEDs in idle state
            if (cool_isr_counter >= 100) // 0.1 second has passed
            {
                cool_isr_counter = 0; // Reset the counter
                cool_countdown_100ms--;
                if (cool_countdown_100ms == 0) 
                {
                    cool_countdown_100ms = COOL_USER_SETTING; // Reset for next time
                    state = STATE_DEFLATE;
                } 
                else 
                {
                    countdown_tens = cool_countdown_100ms / 10;
                    countdown_ones = cool_countdown_100ms % 10;
                    digits[0] = segmap[countdown_ones];
                    digits[1] = segmap[countdown_tens];
                }
            }
        }
        else if (state == STATE_DEFLATE) 
        {
            RC2=1; // Turn on vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            digits[0] = SEG_CDEG_ONLY;
            digits[1] = SEG_CDEG_ONLY;
            digits[2] = LED_COOL|LED_TEMP;  // Turn off all LEDs in idle state
            if (deflate_isr_counter >= 100) // 0.1 second has passed
            {
                deflate_isr_counter = 0; // Reset the counter
                deflate_countdown_100ms--;
                if (deflate_countdown_100ms == 0) 
                {
                    deflate_countdown_100ms = DEFLATE_USER_SETTING; // Reset for next time
                    state = STATE_LID_STUCK;
                } 
                if (switch_debounced[0] == 1) 
                {
                    state = STATE_IDLE;
                }
            }
        }
        else if (state == STATE_LID_STUCK) 
        {
            RC2=0; // Turn off vent air solenoid
            RC4=0; // Turn off vacuum pump relay and vacuum air solenoid
            RC5=0; // Turn off medium temperature relay
            RC6=0; // Turn off low temperature relay
            RC7=0; // Turn off high temperature relay
            if (lidstuck_symbol_toggle)
            {
                digits[0] = SEG_CDE_ONLY;
                digits[1] = SEG_CDE_ONLY;
                digits[2] = LED_OFF|LED_TEMP;
            }
            else
            {
                digits[0] = SEG_CDEG_ONLY;
                digits[1] = SEG_CDEG_ONLY;
                digits[2] = LED_FATAL|LED_TEMP;
            }
            if (lidstuck_isr_counter >= 500) // 1 second has passed
            {
                lidstuck_isr_counter = 0; // Reset the counter
                lidstuck_symbol_toggle = !lidstuck_symbol_toggle; // Toggle the symbol state
            }
            if (switch_debounced[0] == 1) 
            {
                state = STATE_IDLE;
            }   
        }
    }
}
