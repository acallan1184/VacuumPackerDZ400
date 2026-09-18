#include <xc.h>

// =========================
// Configuration Bits
// =========================
#pragma config BOREN = ON // Brown-out reset enable bit enabled
#pragma config CP    = ON // Code protection enabled
#pragma config PWRTE = ON // Power-up timer enable bit is enabled
#pragma config WDTE  = 0 // Watchdog Timer Enable bit is disabled
#pragma config FOSC  = HS // Oscillator Selection bit

#define _XTAL_FREQ 4000000 // 4 MHz Crystal
#define NUM_SWITCHES 7

#define NO_HEAT 0
#define LOW_HEAT 1
#define MEDIUM_HEAT 2
#define HIGH_HEAT 3

// g?segment only (bit4 = 1)
#define SEG_G_ONLY     0x10
#define SEG_DP_ONLY    0x08
#define SEG_CDEG_ONLY  0x17
#define SEG_CDE_ONLY   0x07

// LED patterns for state indication on third display (7 individual LEDs, common cathode)
#define LED_ALL    0xFF   // All LEDs on
#define LED_VACUUM 0x80   // LED for vacuum state
#define LED_COOL   0x40   // LED for cool state
#define LED_FATAL  0x10   // LED for fatal state
#define LED_SEAL   0x08   // LED for high temperature state
#define LED_MED    0x04   // LED for medium temperature state
#define LED_LOW    0x02   // LED for low temperature state
#define LED_HIGH   0x01   // There is no on board LED for this.
#define LED_OFF    0x00   // All LEDs off

// =========================
// State Machine
// =========================
#define STATE_BOOT          0
#define STATE_IDLE          1
#define STATE_VACUUM        2
#define STATE_CONFIG_VACUUM 3
#define STATE_CONFIG_SEAL   4
#define STATE_SEAL          5
#define STATE_COOL          6
#define STATE_DEFLATE       7
#define STATE_LID_STUCK     8

// =========================
// Segment pins    (PB0,PB1,PB2, PB3,PB4,PB5,PB6,PB7)
// Segment mapping (c,d,e,dp,g,f,a,b)
// bit7=b, bit6=a, bit5=f, bit4=g, bit3=dp, bit2=e, bit1=d, bit0=c
// =========================
const unsigned char segmap[10] = 
{
    0xE7, // 0 in 7-seg format c,d,e,dp,g,f,a,b
    0x81, // 1 in 7-seg format c,d,e,dp,g,f,a,b
    0xD6, // 2 in 7-seg format c,d,e,dp,g,f,a,b
    0xD3, // 3 in 7-seg format c,d,e,dp,g,f,a,b
    0xB1, // 4 in 7-seg format c,d,e,dp,g,f,a,b
    0x73, // 5 in 7-seg format c,d,e,dp,g,f,a,b
    0x77, // 6 in 7-seg format c,d,e,dp,g,f,a,b
    0xC1, // 7 in 7-seg format c,d,e,dp,g,f,a,b
    0xF7, // 8 in 7-seg format c,d,e,dp,g,f,a,b
    0xF3  // 9 in 7-seg format c,d,e,dp,g,f,a,b
};

volatile unsigned char digits[3] = {0, 0, 0}; // Array to hold segment patterns for digits - units, tens, and LEDs
volatile unsigned char current_digit = 0; // Index to track which digit is currently being displayed (0 = units, 1 = tens, 2 = LEDs)
unsigned char buzzer_update = 1; // Flag to trigger buzzer update
volatile unsigned int buzzer_timer = 0; // Variable to hold the remaining duration for the buzzer tone
unsigned char lidstuck_symbol_toggle = 0; // Flag to toggle the lid stuck symbol

// Countdown timers inital settings
unsigned char vacuum_eeprom_readback = 40; // User-defined vacuum countdown setting in seconds
unsigned char seal_eeprom_readback = 25; // User-defined seal countdown setting in 0.1 seconds
unsigned char temp_eeprom_readback = NO_HEAT; // User-defined termperature setting (0 = no heat, 1 = low, 2 = medium, 3 = high)
const unsigned char cool_setting = 30; // Fixed cool countdown setting in 0.1 seconds
const unsigned char deflate_setting = 50; // Fixed deflate countdown setting in 0.1 seconds

// Countdown timers live settings
unsigned char vacuum_countdown_seconds; // Variable to hold the remaining vacuum countdown time in seconds
unsigned char seal_countdown_100ms; // Variable to hold the remaining seal countdown time in 0.1 seconds
unsigned char cool_countdown_100ms; // Variable to hold the remaining cool countdown time in 0.1 seconds
unsigned char deflate_countdown_100ms; // Variable to hold the remaining deflate countdown time in 0.1 seconds

volatile unsigned char state = STATE_BOOT; // Variable to hold the current state of the state machine

// ISR counters for various states
volatile unsigned int vacuum_isr_counter = 0;  // Counts ISR ticks for 1 second intervals
volatile unsigned int seal_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int cool_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int deflate_isr_counter = 0;  // Counts ISR ticks for 0.1 second intervals
volatile unsigned int lidstuck_isr_counter = 0;  // Counts ISR ticks for 1 second intervals

// Switch sampling
volatile unsigned char switch_raw[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Array to hold the raw switch states (1 = unpressed, 0 = pressed)
volatile unsigned char switch_last[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Array to hold the previous switch states
volatile unsigned char switch_debounced[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Array to hold the debounced switch states
volatile unsigned char switch_stable_count[NUM_SWITCHES]={1,1,1,1,1,1,1}; // Array to hold the stable switch counts
volatile unsigned char switch_prev_debounced[NUM_SWITCHES] = {1,1,1,1,1,1,1}; // Track previous debounced value (for ISR edge detection)
volatile unsigned char edge_detected[NUM_SWITCHES] = {0,0,0,0,0,0,0}; // Flags: edge detected, main loop clears after handling
volatile unsigned char LED_TEMP = LED_OFF; // Variable to hold the current LED pattern for temperature indication
// =========================
// Buzzer tone generator (approximately 500 Hz with a 1 ms Timer0 tick)
// =========================
static void eeprom_write(unsigned char addr, unsigned char data);

static const unsigned char temp_led_map[4] = {LED_OFF, LED_LOW, LED_MED, LED_HIGH};

static void show_value(unsigned char value, unsigned char decimal_point)
{
    digits[0] = segmap[value % 10];
    digits[1] = segmap[value / 10] | (decimal_point ? SEG_DP_ONLY : 0);
}

static void set_heat_relays(unsigned char heat)
{
    RC5 = (heat == MEDIUM_HEAT);
    RC6 = (heat == LOW_HEAT);
    RC7 = (heat == HIGH_HEAT);
}

static void all_relays_off(void)
{
    RC2 = 0;
    RC4 = 0;
    RC5 = 0;
    RC6 = 0;
    RC7 = 0;
}

// Timer counters are 16-bit values on an 8-bit MCU. Keep each main-loop
// read/reset together so the ISR cannot access only half of the value.
static unsigned char timer_expired(volatile unsigned int *timer, unsigned int limit)
{
    unsigned char expired;
    T0IE = 0;
    expired = (*timer >= limit);
    if (expired)
    {
        *timer = 0;
    }
    T0IE = 1;
    return expired;
}

static void timer_reset(volatile unsigned int *timer)
{
    T0IE = 0;
    *timer = 0;
    T0IE = 1;
}

static unsigned char timer_is_zero(volatile unsigned int *timer)
{
    unsigned char is_zero;

    T0IE = 0;
    is_zero = (*timer == 0);
    T0IE = 1;
    return is_zero;
}

static void save_eeprom(unsigned char address, unsigned char value)
{
    eeprom_write(address, value);
    __delay_ms(10);
}

static void buzz_500hz(unsigned int duration_ms)
{
    // The ISR decrements this value, so load both bytes with Timer0 paused.
    T0IE = 0;
    buzzer_timer = duration_ms; // Set the buzzer duration in Timer0 ticks
    T0IE = 1;
}

static void update_temperature_led(void)
{
    if (temp_eeprom_readback > HIGH_HEAT)
    {
        temp_eeprom_readback = LOW_HEAT;
    }
    LED_TEMP = temp_led_map[temp_eeprom_readback];
}

// =========================
// I2C Functions
void i2c_init(void) // Just to set up the bus and start in a known state especially during boot
{
    RA4=0; // Pull SDA low
    TRISA4=0; // SDA is an open-drain output, for now
    RA5=0; // Drive SCL low    
    TRISA5=0; // SCL is a push-pull output, always
}
void i2c_start(void) // This is a "start" condition, SDA goes low while SCL is high
{
    TRISA4=0; // SDA is an open-drain output
    RA4=1; // Let go of SDA (high)
    RA5=1; // Drive SCL high
    __delay_us(5); 
    RA4=0; // Pull SDA low
    __delay_us(5);
    RA5=0; // Drive SCL low
}

void i2c_stop(void) // This is a "stop" condition, SDA goes high while SCL is high
{
    TRISA4=0; // SDA is an open-drain output, for now
    RA4=0; // Pull SDA low
    RA5=0; // Drive SCL low
    __delay_us(5);
    RA5=1; // Drive SCL high
    __delay_us(5);
    RA4=1; // Let go of SDA (high)
}
// Read a byte from the I2C bus, sending an ACK or NACK after the byte is read
unsigned char i2c_read(unsigned char ack) 
{
    unsigned char byte = 0;
    TRISA4 = 1;      // SDA is an input now
    for(unsigned char i = 0; i < 8; i++) // For each bit in the byte
    {
        RA5 = 1; // Drive SCL high to clock in the bit
        __delay_us(5);
        byte = (unsigned char)(((unsigned int)byte << 1) |(unsigned int)(RA4 != 0)); // Read the bit from SDA and shift it into the byte
        RA5 = 0; // Drive SCL low to prepare for the next bit
        __delay_us(5);
    }
    // ACK/NACK
    if(ack) // If ack is 1, send an ACK (pull SDA low)
    {
        TRISA4 = 0; // SDA is an output now
        RA4 = 0; // ACK - pull SDA low
    } 
    else  // If ack is 0, send a NACK - Let go of SDA (high)
    {
        TRISA4 = 0; // SDA is an output now
        RA4 = 1; // NACK - // Let go of SDA (high)
    }
    __delay_us(2);
    RA5 = 1; // Drive SCL high to clock the ACK/NACK bit
    __delay_us(5);
    RA5 = 0; // Drive SCL low to finish the ACK/NACK bit
    RA4 = 1; // Let go of SDA (high)
    return byte; // Return the byte read from the I2C bus
}
// Write a byte to the I2C bus, returns 1 if ACK received, 0 if NACK received
unsigned char i2c_write(unsigned char byte)
{
    for(unsigned char i = 0; i < 8; i++) // For each bit in the byte
    {
        if(byte & 0x80) // If the MSB is 1, set SDA high
        {
            TRISA4 = 0; // SDA is an output now
            RA4=1; // Let go of SDA (high)
        }
        else 
        {
            TRISA4 = 0; // SDA is an output now
            RA4=0; // Pull SDA low
        }
        __delay_us(2);
        RA5 = 1; // Drive SCL high to clock the bit
        __delay_us(5);
        RA5 = 0; // Drive SCL low to prepare for the next bit
        byte <<= 1; // Shift the byte left to get the next bit into the MSB position
    }
    // ACK bit
    TRISA4 = 1;      // SDA is an input now
    __delay_us(2);
    RA5 = 1; // Drive SCL high to clock the ACK bit
    __delay_us(5);
    unsigned char ack = RA4;   // 0 = ACK received, 1 = NACK received
    RA5 = 0; // Drive SCL low to finish the ACK bit
    return (ack == 0); // Return 1 if ACK received, 0 if NACK received
}
// write a specified byte to the EEPROM at the specified address at device address 0xA0 (write)
static void eeprom_write(unsigned char addr, unsigned char data)
{
    unsigned char timer_enabled = T0IE;
    // Keep Timer0 from interrupting the timing-sensitive bit-banged bus.
    T0IE = 0;

    i2c_start();         // Start condition
    i2c_write(0xA0);     // device address + write bit
    i2c_write(addr);     // Send the EEPROM address to write to
    i2c_write(data);     // Send the data byte to write
    i2c_stop();          // Stop condition

    if (timer_enabled)
    {
        T0IE = 1;
    }
}
// read a byte from the EEPROM at the specified address at device address 0xA0 (write) and 0xA1 (read)
unsigned char eeprom_read(unsigned char addr)
{
    unsigned char data;  // Variable to hold the data read from EEPROM
    unsigned char timer_enabled = T0IE;
    // Restore the caller's Timer0 state after the complete bus transaction.
    T0IE = 0;

    i2c_start();         // Start condition
    i2c_write(0xA0);     // device address + write bit
    i2c_write(addr);     // Send the EEPROM address to read from
    i2c_start();         // repeated start
    i2c_write(0xA1);     // device address + read bit
    data = i2c_read(0);  // Read the data byte from EEPROM and send NACK to indicate end of reading
    i2c_stop();          // Stop condition

    if (timer_enabled)
    {
        T0IE = 1;
    }
    return data;         // Return the data read from EEPROM
}

// =========================
// Interrupt (Timer0)
// =========================
void __interrupt() isr(void)
{
    // Only one ISR for all interrupts, so check which interrupt occurred
    if (T0IF) // Check if Timer0 overflow interrupt flag is set
    {
        T0IF = 0; // Reset the overflow bit
        TMR0 = 225; // Preload for ~0.992 ms: 31 Timer0 ticks at 32 us each

        // turn off all digits so they don't ghost or dimly light segments
        RA0 = 0; // digit 0 OFF
        RA1 = 0; // digit 1 OFF
        RA2 = 0; // LEDs OFF
        PORTB = 0x00;  // Clear PORTB so they don't ghost or dimly light segments 

        // read inputs
        switch_raw[0] = RA3; // Read RA3 (lid sensor)
        TRISB = 0xFF; // Set PORTB as input
        switch_raw[1] = RB7; // Read RB7 (switch 1)
        switch_raw[2] = RB6; // Read RB6 (switch 2)
        switch_raw[3] = RB3; // Read RB3 (switch 3)
        switch_raw[4] = RB1; // Read RB1 (switch 4)
        switch_raw[5] = RB2; // Read RB2 (switch 5)
        switch_raw[6] = RB0; // Read RB0 (switch 6)
        // Debounce switches
        // for each switch, if the raw value is the same as the last value, increment the stable count. 
        // If it changes, reset the stable count and update the last value. 
        // If the stable count reaches 20 (20 ms), update the debounced value and check for edge detection.
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
                // Detect edge: if debounced value changed, set flag for main loop
                if (switch_debounced[i] != switch_prev_debounced[i]) {
                    edge_detected[i] = 1;
                    switch_prev_debounced[i] = switch_debounced[i];
                }
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

        // Buzzer tone generator: toggle once per approximately 1 ms Timer0 tick
        // ============================
        if (buzzer_timer > 0) 
        {
            RC0 = ~RC0;              // toggle buzzer pin
            buzzer_timer--;          // countdown tone duration
        } 
        else 
        {
            RC0 = 0;                     // ensure buzzer OFF when the timer runs out and after
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
    // Timer0: 32 us tick with 1:32 prescaler; preload 225 gives approximately 0.992 ms
    OPTION_REG = 0b00000100; // Prescaler 1:32, TMR0 source = internal instruction cycle clock (Fosc/4)
    TMR0 = 225; // Preload for ~1ms overflow

    T0IE = 1; // Enable Timer0 interrupt
    GIE  = 1; // Enable global interrupts

    while (1)
    {   
        // Handle emergency stop (switch 1) - if pressed, reset to idle state and turn off all outputs
        if (edge_detected[1] && switch_debounced[1] == 0 && state !=STATE_IDLE && state != STATE_BOOT) //
        {
            state = STATE_DEFLATE; // Reset to idle state
            vacuum_countdown_seconds = vacuum_eeprom_readback; // Reset vacuum countdown
            seal_countdown_100ms = seal_eeprom_readback; // Reset seal countdown
            cool_countdown_100ms = cool_setting; // Reset cool countdown
            deflate_countdown_100ms = deflate_setting; // Reset deflate countdown
            timer_reset(&vacuum_isr_counter);
            timer_reset(&seal_isr_counter);
            timer_reset(&cool_isr_counter);
            timer_reset(&deflate_isr_counter);
            timer_reset(&lidstuck_isr_counter);
            edge_detected[1] = 0; // Clear edge flag after handling
        }
        // Handle boot state
        if (state == STATE_BOOT) 
        {
            i2c_init(); // Initialize I2C for EEPROM communication
            digits[0] = 0xFF;
            digits[1] = 0xFF;
            digits[2] = LED_ALL;  // LEDs on during boot for visual feedback
            if (buzzer_update == 1)
            {  
                buzz_500hz(1000);  // Approximately 1 second of buzzer tone
                buzzer_update = 0;  // Reset the update flag
            }
            if (timer_is_zero(&buzzer_timer)) 
            {
                state = STATE_IDLE;
                vacuum_eeprom_readback = eeprom_read(0x50); // Read vacuum countdown setting from EEPROM address 0x50
                __delay_ms(10); // Small delay to ensure EEPROM read completes
                seal_eeprom_readback = eeprom_read(0x51); // Read seal countdown setting from EEPROM address 0x51
                __delay_ms(10); // Small delay to ensure EEPROM read completes
                temp_eeprom_readback = eeprom_read(0x52); // Read temperature setting from EEPROM address 0x52
                __delay_ms(10); // Small delay to ensure EEPROM read completes
                if (vacuum_eeprom_readback < 1 || vacuum_eeprom_readback > 99) // Ensure the value read from EEPROM is within valid range
                {
                    vacuum_eeprom_readback = 40; // Limit to 40 if out of range
                    save_eeprom(0x50, vacuum_eeprom_readback);
                }
                if (seal_eeprom_readback < 1 || seal_eeprom_readback > 60) // Ensure the value read from EEPROM is within valid range
                {
                    seal_eeprom_readback = 25; // Limit to 25 if out of range
                    save_eeprom(0x51, seal_eeprom_readback);
                }
                if (temp_eeprom_readback > HIGH_HEAT) // Ensure the value read from EEPROM is within valid range
                {
                    temp_eeprom_readback = LOW_HEAT; // Limit to LOW_HEAT if out of range
                    save_eeprom(0x52, temp_eeprom_readback);
                }
                update_temperature_led(); // Update the LED_TEMP variable based on the temperature setting
            }
        }
        else if (state == STATE_IDLE)
        {
            all_relays_off();
            vacuum_countdown_seconds = vacuum_eeprom_readback; // Initialize vacuum countdown from EEPROM setting
            seal_countdown_100ms = seal_eeprom_readback; // Initialize seal countdown from EEPROM setting
            cool_countdown_100ms = cool_setting; // Initialize cool countdown from fixed setting
            deflate_countdown_100ms = deflate_setting; // Initialize deflate countdown from fixed setting
            digits[0] = SEG_G_ONLY;
            digits[1] = SEG_G_ONLY;
            digits[2] = LED_TEMP;  // Turn off all LEDs in idle state
            if (edge_detected[0] && switch_debounced[0] == 0) // Falling edge detected (lid closed)
            {
                state = STATE_VACUUM; // Transition to vacuum state
                edge_detected[0] = 0; // Clear edge flag
            }
            if (edge_detected[2] && switch_debounced[2] == 0) // Falling edge detected (button press)
            {
                state = STATE_CONFIG_VACUUM; // Transition to config vacuum state
                edge_detected[2] = 0; // Clear edge flag
            }
            if (edge_detected[3] && switch_debounced[3] == 0) // Falling edge detected (button press)
            {
                state = STATE_CONFIG_SEAL; // Transition to config seal state
                edge_detected[3] = 0; // Clear edge flag
            }
            // Check for falling edge on switch 6 (temperature setting button)
            if (edge_detected[6] && switch_debounced[6] == 0) // Falling edge detected (button press)
            {
                temp_eeprom_readback++; // Increment temperature setting
                if (temp_eeprom_readback > HIGH_HEAT)
                {
                    temp_eeprom_readback = NO_HEAT;
                }
                update_temperature_led();
                save_eeprom(0x52, temp_eeprom_readback);
                edge_detected[6] = 0; // Clear edge flag
            }

        } 
        else if (state == STATE_CONFIG_VACUUM) 
        {
            show_value(vacuum_eeprom_readback, 0);
            digits[2] = LED_VACUUM|LED_TEMP;  // Indicate vacuum configuration mode
            if (edge_detected[4] && switch_debounced[4] == 0) // Falling edge detected (button press)
            {
                if (vacuum_eeprom_readback < 99) // Limit to 99 seconds
                {
                    vacuum_eeprom_readback++; // Increment vacuum countdown setting
                }
                edge_detected[4] = 0; // Clear edge flag
            }
            if (edge_detected[5] && switch_debounced[5] == 0) // Falling edge detected (button press)
            {
                if (vacuum_eeprom_readback > 1) // Limit to minimum of 1 second
                {
                    vacuum_eeprom_readback--; // Decrement vacuum countdown setting
                }
                edge_detected[5] = 0; // Clear edge flag
            }
            if (edge_detected[2] && switch_debounced[2] == 0) // Falling edge detected (button press)
            {
                save_eeprom(0x50, vacuum_eeprom_readback);
                state = STATE_IDLE; // Exit configuration mode and return to idle state
                edge_detected[2] = 0; // Clear edge flag
            }
        }
        else if (state == STATE_CONFIG_SEAL) 
        {
            show_value(seal_eeprom_readback, 1);
            digits[2] = LED_SEAL|LED_TEMP;  // Indicate seal configuration mode
            if (edge_detected[4] && switch_debounced[4] == 0) // Falling edge detected (button press)
            {
                if (seal_eeprom_readback < 60) // Limit to 6.0 seconds (60 * 0.1s)
                {
                    seal_eeprom_readback++; // Increment seal countdown setting
                }
                edge_detected[4] = 0; // Clear edge flag
            }
            if (edge_detected[5] && switch_debounced[5] == 0) // Falling edge detected (button press)
            {
                if (seal_eeprom_readback > 1) // Limit to minimum of 0.1 second (1 * 0.1s)
                {
                    seal_eeprom_readback--; // Decrement seal countdown setting
                }
                edge_detected[5] = 0; // Clear edge flag
            }
            if (edge_detected[3] && switch_debounced[3] == 0) // Falling edge detected (button press)
            {
                save_eeprom(0x51, seal_eeprom_readback);
                state = STATE_IDLE; // Exit configuration mode and return to idle state
                edge_detected[3] = 0; // Clear edge flag
            }
        }
        else if (state == STATE_VACUUM) 
        {
            all_relays_off();
            RC4 = 1;
            show_value(vacuum_countdown_seconds, 0);
            digits[2] = LED_VACUUM|LED_TEMP;  // Turn off all LEDs in idle state
            if (timer_expired(&vacuum_isr_counter, 1000)) // 1 second has passed
            {
                vacuum_countdown_seconds--;
                if (vacuum_countdown_seconds == 0) 
                {
                    vacuum_countdown_seconds = vacuum_eeprom_readback; // Reset for next time
                    state = STATE_SEAL;
                } 
                else 
                {
                    show_value(vacuum_countdown_seconds, 0);
                }
            }
    
        }
        else if (state == STATE_SEAL) 
        {
            all_relays_off();
            set_heat_relays(temp_eeprom_readback);
            show_value(seal_countdown_100ms, 1);
            digits[2] = LED_SEAL|LED_TEMP;  // Turn off all LEDs in idle state
            if (timer_expired(&seal_isr_counter, 100)) // 0.1 second has passed
            {
                seal_countdown_100ms--;
                if (seal_countdown_100ms == 0) 
                {
                    seal_countdown_100ms = seal_eeprom_readback; // Reset for next time
                    state = STATE_COOL;
                } 
                else 
                {
                    show_value(seal_countdown_100ms, 1);
                }
            }
        }
        else if (state == STATE_COOL) 
        {
            all_relays_off();
            show_value(cool_countdown_100ms, 1);
            digits[2] = LED_COOL|LED_TEMP;  // Turn off all LEDs in idle state
            if (timer_expired(&cool_isr_counter, 100)) // 0.1 second has passed
            {
                cool_countdown_100ms--;
                if (cool_countdown_100ms == 0) 
                {
                    cool_countdown_100ms = cool_setting; // Reset for next time
                    state = STATE_DEFLATE;
                } 
                else 
                {
                    show_value(cool_countdown_100ms, 1);
                }
            }
        }
        else if (state == STATE_DEFLATE) 
        {
            all_relays_off();
            RC2 = 1;
            digits[0] = SEG_CDEG_ONLY;
            digits[1] = SEG_CDEG_ONLY;
            digits[2] = LED_COOL|LED_TEMP;  // Turn off all LEDs in idle state
            if (timer_expired(&deflate_isr_counter, 100)) // 0.1 second has passed
            {
                deflate_countdown_100ms--;
                if (deflate_countdown_100ms == 0) 
                {
                    deflate_countdown_100ms = deflate_setting; // Reset for next time
                    state = STATE_LID_STUCK;
                } 
                if (switch_debounced[0] == 1) 
                {
                    state = STATE_IDLE; // This is the expected behavior, will reset setting in IDLE state
                }
            }
        }
        else if (state == STATE_LID_STUCK) 
        {
            all_relays_off();
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
            if (timer_expired(&lidstuck_isr_counter, 500)) // 1 second has passed
            {
                lidstuck_symbol_toggle = !lidstuck_symbol_toggle; // Toggle the symbol state
            }
            if (switch_debounced[0] == 1) 
            {
                state = STATE_IDLE;
            }   
        }
    }
}